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

/** \author Sachin Chitta */

#include <planning_environment/monitors/joint_state_monitor.h>
#include <angles/angles.h>
#include <sstream>

planning_environment::JointStateMonitor::JointStateMonitor()
{
  first_time_ = true;  
  std::string urdf_xml,full_urdf_xml;
  root_handle_.param("urdf_xml",urdf_xml,std::string("robot_description"));

  if(!root_handle_.getParam(urdf_xml,full_urdf_xml))
  {
    ROS_ERROR("Could not load the xml from parameter server: %s\n", urdf_xml.c_str());
    active_ = false;
  }
  else
  {
    robot_model_.initString(full_urdf_xml);
    active_ = true;
    joint_state_subscriber_ =  root_handle_.subscribe("joint_states", 1, &planning_environment::JointStateMonitor::jointStateCallback, this);
    ROS_INFO("Joint state monitor active");
  }
}

void planning_environment::JointStateMonitor::stop(void)
{
  if (!active_)
    return;    
  joint_state_subscriber_.shutdown();    
  active_ = false;
}

void planning_environment::JointStateMonitor::jointStateCallback(const sensor_msgs::JointStateConstPtr &joint_state)
{    
  if (!active_)
    return;    
  ROS_DEBUG("Joint state monitor callback");
  if (joint_state->name.size() != joint_state->position.size() || joint_state->name.size() !=joint_state->velocity.size())
  {
    ROS_ERROR("Planning environment received invalid joint state");
    return;
  }
  boost::mutex::scoped_lock lock(state_mutex_);
  if(first_time_)
  {
    joint_state_.header.frame_id = "base_footprint";
    joint_state_.name = joint_state->name;
    joint_state_.position.resize(joint_state->position.size());
    for(unsigned int i=0; i < joint_state_.name.size(); i++)
    {
      joint_state_index_[joint_state_.name[i]] = i;

      boost::shared_ptr<const urdf::Joint> joint = robot_model_.getJoint(joint_state_.name[i]);
      if(!joint->child_link_name.empty())
      {
        joint_real_state_index_.push_back(i);
      }
    }
  }
  joint_state_.header.stamp = ros::Time::now();
  for(unsigned int i=0; i < joint_state->position.size(); i++)
  {
    boost::shared_ptr<const urdf::Joint> joint = robot_model_.getJoint(joint_state_.name[i]);
    if (joint->type == urdf::Joint::CONTINUOUS)
      joint_state_.position[i] = angles::normalize_angle(joint_state->position[i]);
    else
      joint_state_.position[i] = joint_state->position[i];
  }
  first_time_ = false;
  last_update_ = joint_state->header.stamp;
}

sensor_msgs::JointState planning_environment::JointStateMonitor::getJointState()
{
  boost::mutex::scoped_lock lock(state_mutex_);
  return joint_state_;
}

sensor_msgs::JointState planning_environment::JointStateMonitor::getJointStateRealJoints()
{
  if (!active_)
    return joint_state_;    
  boost::mutex::scoped_lock lock(state_mutex_);
  sensor_msgs::JointState joint_state;

  unsigned int num_real_joints = joint_real_state_index_.size();
  joint_state.header = joint_state_.header;
  joint_state.name.resize(num_real_joints);
  joint_state.position.resize(num_real_joints);

  for(unsigned int i=0; i< num_real_joints; i++)
  {
    int index = joint_real_state_index_[i];
    joint_state.name[i] = joint_state_.name[index];
    joint_state.position[i] = joint_state_.position[index];
  }
  return joint_state;
}


sensor_msgs::JointState planning_environment::JointStateMonitor::getJointState(std::vector<std::string> names)
{
  if (!active_)
    return joint_state_;    
  sensor_msgs::JointState state;
  state.position.resize(names.size());
  state.name = names;

  boost::mutex::scoped_lock lock(state_mutex_);
  for(unsigned int i=0; i < state.name.size(); i++)
  {
    std::map<std::string, int>::iterator joint_it = joint_state_index_.find(state.name[i]);
    if(joint_it == joint_state_index_.end())
    {
      continue;
    }
    else
    {
      int index = joint_it->second;
      state.position[i]  = joint_state_.position[index];
    }
  }
  return state;
}
