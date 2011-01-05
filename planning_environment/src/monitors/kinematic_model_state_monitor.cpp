/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** \author Ioan Sucan */

#include "planning_environment/monitors/kinematic_model_state_monitor.h"
#include "planning_environment/util/construct_object.h"
#include <angles/angles.h>
#include <sstream>

void planning_environment::KinematicModelStateMonitor::setupRSM(void)
{
  stateMonitorStarted_ = false;
  onStateUpdate_ = NULL;
  havePose_ = haveJointState_ = false;
  robotVelocity_ = 0.0;
    
  printed_out_of_date_ = false;
  if (rm_->loadedModels())
  {
    kmodel_ = rm_->getKinematicModel();
    robot_frame_ = kmodel_->getRoot()->getChildLinkModel()->getName();
    ROS_INFO("Robot frame is '%s'", robot_frame_.c_str());
    startStateMonitor();
  } else {
    ROS_INFO("Can't start state monitor yet");
  }
    
  nh_.param<double>("joint_state_cache_time", joint_state_cache_time_, 2.0);
  nh_.param<double>("joint_state_cache_allowed_difference", joint_state_cache_allowed_difference_, .25);

}

void planning_environment::KinematicModelStateMonitor::startStateMonitor(void)
{   
  if (stateMonitorStarted_)
    return;
    
  if (rm_->loadedModels())
  {
    jointStateSubscriber_ = root_handle_.subscribe("joint_states", 25, &KinematicModelStateMonitor::jointStateCallback, this);
    ROS_DEBUG("Listening to joint states");
	
  }
  stateMonitorStarted_ = true;
}

void planning_environment::KinematicModelStateMonitor::stopStateMonitor(void)
{
  if (!stateMonitorStarted_)
    return;
    
  jointStateSubscriber_.shutdown();

  joint_state_map_cache_.clear();
    
  ROS_DEBUG("Kinematic state is no longer being monitored");
    
  stateMonitorStarted_ = false;
}

void planning_environment::KinematicModelStateMonitor::jointStateCallback(const sensor_msgs::JointStateConstPtr &jointState)
{
  //bool change = !haveJointState_;

  planning_models::KinematicState state(kmodel_);

  current_joint_values_lock_.lock();

  state.setKinematicState(current_joint_state_map_);

  static bool first_time = true;

  unsigned int n = jointState->name.size();
  if (jointState->name.size() != jointState->position.size() || jointState->name.size() !=jointState->velocity.size())
  {
    ROS_ERROR("Planning environment received invalid joint state");
    current_joint_values_lock_.unlock();
    return;
  }
  
  robotVelocity_ = 0.0;
  std::map<std::string, double> joint_state_map;
  for (unsigned int i = 0 ; i < n ; ++i)
  {
    joint_state_map[jointState->name[i]] = jointState->position[i];
    robotVelocity_ += jointState->velocity[i];
  }

  std::vector<planning_models::KinematicState::JointState*>& joint_state_vector = state.getJointStateVector();
  
  for(std::vector<planning_models::KinematicState::JointState*>::iterator it = joint_state_vector.begin();
      it != joint_state_vector.end();
      it++) {
    bool tfSets = false;
    bool jointStateSets = false;
    //see if we need to update any transforms
    std::string parent_frame_id = (*it)->getParentFrameId();
    std::string child_frame_id = (*it)->getChildFrameId();
    if(!parent_frame_id.empty() && !child_frame_id.empty()) {
      std::string err;
      ros::Time tm;
      tf::StampedTransform transf;
      bool ok = false;
      if (tf_->getLatestCommonTime(parent_frame_id, child_frame_id, tm, &err) == tf::NO_ERROR) {
        ok = true;
        try
        {
          tf_->lookupTransform(parent_frame_id, child_frame_id, tm, transf);
        }
        catch(tf::TransformException& ex)
        {
          ROS_ERROR("Unable to lookup transform from %s to %s.  Exception: %s", parent_frame_id.c_str(), child_frame_id.c_str(), ex.what());
          ok = false;
        }
      } else {
        ROS_DEBUG("Unable to lookup transform from %s to %s: no common time.", parent_frame_id.c_str(), child_frame_id.c_str());
        ok = false;
      }
      if(ok) {
        tfSets = (*it)->setJointStateValues(transf);
        if(tfSets) {
          const std::vector<std::string>& joint_state_names = (*it)->getJointStateNameOrder();
          for(std::vector<std::string>::const_iterator it = joint_state_names.begin();
              it != joint_state_names.end();
              it++) {
            last_joint_update_[*it] = tm;
          }
        }
        // btTransform transf = getKinematicModel()->getRoot()->variable_transform;
        // ROS_INFO_STREAM("transform is to " << transf.getRotation().x() << " " 
        //                 << transf.getRotation().y() << " z " << transf.getRotation().z()
        //                 << " w " << transf.getRotation().w());
        havePose_ = tfSets;
        lastPoseUpdate_ = tm;
      }
    }
    //now we update from joint state
    jointStateSets = (*it)->setJointStateValues(joint_state_map);
    if(jointStateSets) {
      const std::vector<std::string>& joint_state_names = (*it)->getJointStateNameOrder();
      for(std::vector<std::string>::const_iterator it = joint_state_names.begin();
          it != joint_state_names.end();
          it++) {
        last_joint_update_[*it] = jointState->header.stamp;
      }
    }
  }
  
  state.updateKinematicLinks();
  state.getKinematicStateValues(current_joint_state_map_);

  if(allJointsUpdated()) {
    haveJointState_ = true;
    lastJointStateUpdate_ = jointState->header.stamp;
    
    if(!joint_state_map_cache_.empty()) {
      if(jointState->header.stamp-joint_state_map_cache_.back().first > ros::Duration(joint_state_cache_allowed_difference_)) {
        ROS_DEBUG_STREAM("Introducing joint state cache sparsity time of " << (jointState->header.stamp-joint_state_map_cache_.back().first).toSec());
      }
    }

    joint_state_map_cache_.push_back(std::pair<ros::Time, std::map<std::string, double> >(jointState->header.stamp,
                                                                                          current_joint_state_map_));
  } 

  if(haveJointState_) {
    while(1) {
      if(joint_state_map_cache_.empty()) {
        ROS_WARN("Empty joint state map cache");
        break;
      }
      if((ros::Time::now()-joint_state_map_cache_.front().first) > ros::Duration(joint_state_cache_time_)) {
        joint_state_map_cache_.pop_front();
      } else {
        break;
      }
    }
  }
  robotVelocity_ = sqrt(robotVelocity_);
    
  first_time = false;

  if(!allJointsUpdated(ros::Duration(1.0))) {
    if(!printed_out_of_date_) {
      ROS_WARN_STREAM("Got joint state update but did not update some joints for more than 1 second.  Turn on DEBUG for more info");
      printed_out_of_date_ = true;
    }
  } else {
    printed_out_of_date_ = false;
  }
  if(onStateUpdate_ != NULL) {
    onStateUpdate_();
  }
  current_joint_values_lock_.unlock();
}

bool planning_environment::KinematicModelStateMonitor::allJointsUpdated(ros::Duration allowed_dur) const {
  current_joint_values_lock_.lock();
  bool ret = true;
  for(std::map<std::string, double>::const_iterator it = current_joint_state_map_.begin();
      it != current_joint_state_map_.end();
      it++) {
    if(last_joint_update_.find(it->first) == last_joint_update_.end()) {
      ROS_DEBUG_STREAM("Joint " << it->first << " not yet updated");
      ret = false;
      continue;
    }
    if(allowed_dur != ros::Duration()) {
      ros::Duration dur = ros::Time::now()-last_joint_update_.find(it->first)->second; 
      if(dur > allowed_dur) {
        ROS_DEBUG_STREAM("Joint " << it->first << " last updated " << dur.toSec() << " where allowed duration is " << allowed_dur.toSec());
        ret = false;
        continue;
      }
    }
  }
  current_joint_values_lock_.unlock();
  return ret;
}

void planning_environment::KinematicModelStateMonitor::setStateValuesFromCurrentValues(planning_models::KinematicState& state) const
{
  current_joint_values_lock_.lock();
  state.setKinematicState(current_joint_state_map_);
  current_joint_values_lock_.unlock();
}

void planning_environment::KinematicModelStateMonitor::setRobotStateAndComputeTransforms(const motion_planning_msgs::RobotState &robot_state,
                                                                                         planning_models::KinematicState& state) const
{
  setStateValuesFromCurrentValues(state);
  std::map<std::string, double> joint_state_map;
  for (unsigned int i = 0 ; i < robot_state.joint_state.name.size(); ++i)
  {
    joint_state_map[robot_state.joint_state.name[i]] = robot_state.joint_state.position[i];
  }

  //first we are going to apply transforms
  for(unsigned int i = 0; i < robot_state.multi_dof_joint_state.joint_names.size(); i++) {
    std::string joint_name = robot_state.multi_dof_joint_state.joint_names[i];
    if(!state.hasJointState(joint_name)) {
      ROS_WARN_STREAM("No joint matching multi-dof joint " << joint_name);
      continue;
    }
    planning_models::KinematicState::JointState* joint_state = state.getJointState(joint_name);
    if(robot_state.multi_dof_joint_state.frame_ids[i] != joint_state->getParentFrameId() ||
       robot_state.multi_dof_joint_state.child_frame_ids[i] != joint_state->getChildFrameId()) {
      ROS_WARN_STREAM("Robot state msg has bad multi_dof transform");
    } else {
      tf::StampedTransform transf;
      tf::poseMsgToTF(robot_state.multi_dof_joint_state.poses[i], transf);
      joint_state->setJointStateValues(transf);
    }
  }
  state.setKinematicState(joint_state_map);
}

bool planning_environment::KinematicModelStateMonitor::getCurrentRobotState(motion_planning_msgs::RobotState& robot_state) const
{
  planning_models::KinematicState state(kmodel_);
  setStateValuesFromCurrentValues(state);
  return(convertKinematicStateToRobotState(state,robot_state));
}

bool planning_environment::KinematicModelStateMonitor::setKinematicStateToTime(const ros::Time& time,
                                                                               planning_models::KinematicState& state) const
{
  std::map<std::string, double> joint_value_map;
  if(!getCachedJointStateValues(time, joint_value_map)) {
    return false;
  }
  state.setKinematicState(joint_value_map);
  return true;
}

bool planning_environment::KinematicModelStateMonitor::convertKinematicStateToRobotState(const planning_models::KinematicState& kinematic_state,
                                                                                         motion_planning_msgs::RobotState &robot_state) const
{
  robot_state.joint_state.position.clear();
  robot_state.joint_state.name.clear();

  const std::vector<planning_models::KinematicState::JointState*> joints = kinematic_state.getJointStateVector();
  for(std::vector<planning_models::KinematicState::JointState*>::const_iterator it = joints.begin();
      it != joints.end();
      it++) {
    const std::vector<double>& joint_state_values = (*it)->getJointStateValues();
    const std::vector<std::string>& joint_state_value_names = (*it)->getJointStateNameOrder();
    for(unsigned int i = 0; i < joint_state_values.size(); i++) {
      robot_state.joint_state.name.push_back(joint_state_value_names[i]);
      robot_state.joint_state.position.push_back(joint_state_values[i]);
    }
    if(!(*it)->getParentFrameId().empty() && !(*it)->getChildFrameId().empty()) {
      robot_state.multi_dof_joint_state.stamp = lastPoseUpdate();
      robot_state.multi_dof_joint_state.joint_names.push_back((*it)->getName());
      robot_state.multi_dof_joint_state.frame_ids.push_back((*it)->getParentFrameId());
      robot_state.multi_dof_joint_state.child_frame_ids.push_back((*it)->getChildFrameId());
      tf::Pose p((*it)->getVariableTransform());
      geometry_msgs::Pose pose;
      tf::poseTFToMsg(p, pose);
      robot_state.multi_dof_joint_state.poses.push_back(pose);
    }
  }
  robot_state.joint_state.header.stamp = lastJointStateUpdate();
  robot_state.joint_state.header.frame_id = getWorldFrameId();
  return true;
}

bool planning_environment::KinematicModelStateMonitor::getCachedJointStateValues(const ros::Time& time, std::map<std::string, double>& ret_map) const {
  
  current_joint_values_lock_.lock();

  //first we check the front and backs of the cache versus the time for error states
  if(time-ros::Duration(joint_state_cache_allowed_difference_) > joint_state_map_cache_.back().first) {
    ROS_WARN("Asking for time substantially newer than that contained in cache");
    current_joint_values_lock_.unlock(); 
    return false;
  }
  if(time+ros::Duration(joint_state_cache_allowed_difference_) < joint_state_map_cache_.front().first) {
    ROS_WARN_STREAM("Asking for time substantially older than that contained in cache " << time.toSec() << " " << joint_state_map_cache_.front().first.toSec());
    current_joint_values_lock_.unlock();
    return false;
  }
  
  //then we check if oldest or newest is being requested
  if(time < joint_state_map_cache_.front().first) {
    ret_map = joint_state_map_cache_.front().second;
  } else if (time > joint_state_map_cache_.back().first) {
    ret_map = joint_state_map_cache_.back().second;
  } else {
    std::list<std::pair<ros::Time, std::map<std::string, double> > >::const_reverse_iterator next = ++joint_state_map_cache_.rbegin();
    std::list<std::pair<ros::Time, std::map<std::string, double> > >::const_reverse_iterator cur = joint_state_map_cache_.rbegin();
    while(next != joint_state_map_cache_.rbegin()) {
      if(time < cur->first && time > next->first) {
        ros::Duration ear_diff = time - next->first;
        ros::Duration lat_diff = cur->first - time;
        if(ear_diff > ros::Duration(joint_state_cache_allowed_difference_) &&
           lat_diff > ros::Duration(joint_state_cache_allowed_difference_)) {
          ROS_WARN("Asking for time in joint state area that's too sparse");
          current_joint_values_lock_.unlock();
          return false;
        } else {
          if(ear_diff > lat_diff) {
            ROS_DEBUG_STREAM("Got time later within " << lat_diff.toSec());
            ret_map = cur->second;
            break;
          } else {
            ROS_DEBUG_STREAM("Got time earlier within " << ear_diff.toSec());
            ret_map = next->second;
            break;
          }
        }
      }
      next++;
      cur++;
    }
  }
  current_joint_values_lock_.unlock();
  return true;
}

void planning_environment::KinematicModelStateMonitor::waitForState(void) const
{
  int s = 0;
  while (nh_.ok() && !haveState())
  {
    if (s == 0)
    {
      ROS_INFO("Waiting for robot state ...");
      if (!haveJointState_)
        ROS_INFO("Waiting for joint state ...");
    }
    s = (s + 1) % 40;
    ros::spinOnce();
    ros::Duration().fromSec(0.05).sleep();
  }
  if (haveState())
    ROS_INFO("Robot state received!");
}

bool planning_environment::KinematicModelStateMonitor::isJointStateUpdated(double sec) const
{  
  //ROS_ERROR_STREAM("Here " << sec << " time " << lastJointStateUpdate_.toSec() << " now " << (ros::Time::now().toSec()));

  //three cases
  //1. interval to small - less than 10us is considered 0
  if(sec < 1e-5) 
  {
    return false;
  }

  //ROS_ERROR_STREAM("cond 1 " << (sec > 1e-5) << " cond 2 " << (lastJointStateUpdate_ > ros::TIME_MIN) << " cond 3 " << (ros::Time::now() < ros::Time(sec)));

  //2. it hasn't yet been a full second interval but we've updated
  if(sec > 1e-5 && lastJointStateUpdate_ > ros::TIME_MIN && ros::Time::now() < ros::Time(sec)) 
  {
    return true;
  }

  ROS_DEBUG("Last joint update %g interval begins %g", lastJointStateUpdate_.toSec(), (ros::Time::now()-ros::Duration(sec)).toSec());

  //3. Been longer than sec interval, so we check that the update has happened in the indicated interval
  if (lastJointStateUpdate_ < ros::Time::now()-ros::Duration(sec)) 
  {
    return false;
  }

  return true;
}

bool planning_environment::KinematicModelStateMonitor::isPoseUpdated(double sec) const
{  

  if(sec < 1e-5) 
  {
    return false;
  }

  //2. it hasn't yet been a full second interval but we've updated
  if(lastPoseUpdate_ > ros::TIME_MIN && ros::Time::now() < ros::Time(sec)) 
  {
    return true;
  }

  //3. Been longer than sec interval, so we check that the update has happened in the indicated interval
  if (lastPoseUpdate_ < ros::Time::now()-ros::Duration(sec)) 
  {
    return false;
  }

  return true;
}
