/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#include <collision_free_spline_smoother/cubic_spline_shortcutter.h>
#include <spline_smoother/spline_smoother_utils.h>
#include <stdlib.h>

namespace collision_free_spline_smoother
{

CubicSplineShortCutter::CubicSplineShortCutter()
{
  node_handle_.param<double>("dT", dT_, 0.1);
  if(!setupCollisionEnvironment())
    active_ = false;
  else
    active_ = true;
}

CubicSplineShortCutter::~CubicSplineShortCutter()
{
}

int CubicSplineShortCutter::getRandomInt(int min_index,int max_index)const
{
  int rand_num = rand()%100+1;
  double result = min_index + (double)((max_index-min_index)*rand_num)/101.0;
  return (int) result;
}

bool CubicSplineShortCutter::smooth(const motion_planning_msgs::JointTrajectoryWithLimits& trajectory_in, 
                                    motion_planning_msgs::JointTrajectoryWithLimits& trajectory_out) const
{
  srand(time(NULL)); // initialize random seed: 

  if(!active_)
  {
    ROS_ERROR("Smoother is not active");
    return false;
  }

  motion_planning_msgs::ArmNavigationErrorCodes error_code;
  std::vector<motion_planning_msgs::ArmNavigationErrorCodes> trajectory_error_codes;
  motion_planning_msgs::RobotState robot_state;
  planning_monitor_->getRobotStateMsg(robot_state);

  spline_smoother::CubicTrajectory traj;
  spline_smoother::SplineTrajectory spline;

  motion_planning_msgs::JointTrajectoryWithLimits shortcut;
  trajectory_out = trajectory_in;

  if (!spline_smoother::checkTrajectoryConsistency(trajectory_out))
    return false;

  shortcut.limits = trajectory_in.limits;
  shortcut.trajectory.joint_names = trajectory_in.trajectory.joint_names;
  ros::Time start_time = ros::Time::now();
  ros::Duration timeout = ros::Duration(2.0);
  bool success = true;

  //  planning_monitor_->getEnvironmentModel()->lock();
  //  planning_monitor_->getKinematicModel()->lock();
  motion_planning_msgs::OrderedCollisionOperations operations, ordered_collision_operations;
  std::vector<std::string> child_links;
  planning_monitor_->getChildLinks(trajectory_in.trajectory.joint_names, child_links);
  planning_monitor_->getOrderedCollisionOperationsForOnlyCollideLinks(child_links,ordered_collision_operations,operations);
  planning_monitor_->applyOrderedCollisionOperationsToCollisionSpace(operations);

  while(ros::Time::now() - start_time < timeout)
  {
    int trajectory_size = (int) trajectory_out.trajectory.points.size();
    if(trajectory_size < 3)
      break;
    shortcut.trajectory.points.clear();
    spline.segments.clear();
    int index1, index2;
    index1 = getRandomInt(0,trajectory_size);
    index2 = getRandomInt(index1,trajectory_size);
    if(index1 == index2 || index2-index1 < 2)
      continue;
    ROS_DEBUG("index1: %d, index2: %d, trajectory size: %d",index1,index2,trajectory_size);
    shortcut.trajectory.points.push_back(trajectory_out.trajectory.points[index1]);
    shortcut.trajectory.points.push_back(trajectory_out.trajectory.points[index2]);
    ROS_DEBUG("Pushed back 2 points");
    shortcut.trajectory.points[0].time_from_start = ros::Duration(0.0);
    shortcut.trajectory.points[1].time_from_start = ros::Duration(0.0);
    success = traj.parameterize(shortcut,spline);      
    if(!success)
      break;
    ROS_DEBUG("Set trajectory");

    double total_time;
    spline_smoother::getTotalTime(spline,total_time);
    ROS_DEBUG("Succeeded in setting trajectory with total time: %f",total_time);

    std::vector<double> times;
    times.resize(std::max((int)(total_time/dT_)+1,2));
    times[0] = 0.0;
    for(int i=0; i< (int) times.size()-1; i++)
      times[i+1] = times[i] + dT_; 
    times.back() = total_time;

    trajectory_msgs::JointTrajectory joint_traj;
    spline_smoother::sampleSplineTrajectory(spline,times,joint_traj);
    joint_traj.joint_names = shortcut.trajectory.joint_names;
    ROS_DEBUG("Succeeded in sampling trajectory with size: %d",(int)joint_traj.points.size());

    if(planning_monitor_->isTrajectoryValid(joint_traj,
                                            robot_state,
                                            0,
                                            joint_traj.points.size(),
                                            planning_environment::PlanningMonitor::COLLISION_TEST,
                                            false,
                                            error_code, 
                                            trajectory_error_codes))
    {
      ros::Duration tmp_index_time = trajectory_out.trajectory.points[index1].time_from_start;
      ros::Duration dt_index_time = trajectory_out.trajectory.points[index2].time_from_start - tmp_index_time;
      trajectory_out.trajectory.points[index1].positions = joint_traj.points.front().positions;
      trajectory_out.trajectory.points[index2].positions = joint_traj.points.back().positions;
      trajectory_out.trajectory.points[index1].time_from_start = tmp_index_time;

      for(unsigned int i = index2; i < trajectory_out.trajectory.points.size(); i++)
      {
        trajectory_out.trajectory.points[i].time_from_start += ros::Duration(total_time)-dt_index_time;        
      }

      ROS_DEBUG("Removing points %d through %d from trajectory of size %d",index1+1,index2-1,(int)trajectory_out.trajectory.points.size());
      std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator remove_start = trajectory_out.trajectory.points.begin() + index1 + 1;
      std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator remove_end = trajectory_out.trajectory.points.begin() + index2 - 1;
      if(remove_start != remove_end)
        trajectory_out.trajectory.points.erase(remove_start,remove_end);
      else
        trajectory_out.trajectory.points.erase(remove_start);

    }
    else 
      continue;
  }
	planning_monitor_->getKinematicModel()->lock();
	planning_monitor_->getEnvironmentModel()->lock();
	planning_monitor_->revertAllowedCollisionToDefault();
	planning_monitor_->getKinematicModel()->unlock();
	planning_monitor_->getEnvironmentModel()->unlock();


  std::set<double> times;
  double total_time;
  success = traj.parameterize(trajectory_out,spline);      
  spline_smoother::getTotalTime(spline,total_time);
  for(int i=1; i< (int) (total_time/dT_); i++)
  times.insert(i*dT_);
  times.insert(total_time);

  double insert_time = 0;
  for(unsigned int i=0; i < spline.segments.size(); i++)
  {
    insert_time += spline.segments[i].duration.toSec();
    times.insert(insert_time);
  }

  std::vector<double> times_vec;
  for(std::set<double>::iterator set_iter = times.begin(); set_iter != times.end(); set_iter++)
  {
    times_vec.push_back(*set_iter);
  }
  std::sort(times_vec.begin(), times_vec.end());

  if(!spline_smoother::sampleSplineTrajectory(spline,times_vec,trajectory_out.trajectory))
    return false;
	
  ROS_DEBUG("Final trajectory has %d points and %f total time",(int)trajectory_out.trajectory.points.size(),trajectory_out.trajectory.points.back().time_from_start.toSec());
  //  planning_monitor_->getEnvironmentModel()->unlock();
  //  planning_monitor_->getKinematicModel()->unlock();
  return success;
}

bool CubicSplineShortCutter::setupCollisionEnvironment()
{
  bool use_collision_map, compute_contacts, show_collisions;
  node_handle_.param<bool>("use_collision_map", use_collision_map, true);
  node_handle_.param<bool>("compute_contacts", compute_contacts, true);
  node_handle_.param<bool>("show_collisions", show_collisions, false);

  // monitor robot
  collision_models_ = new planning_environment::CollisionModels("robot_description");
  planning_monitor_ = new planning_environment::PlanningMonitor(collision_models_, &tf_);
  planning_monitor_->use_collision_map_ = use_collision_map;
  planning_monitor_->compute_contacts_ = compute_contacts;
  if(!collision_models_->loadedModels())
    return false;

  if (planning_monitor_->getExpectedJointStateUpdateInterval() > 1e-3)
    planning_monitor_->waitForState();
  if (planning_monitor_->getExpectedMapUpdateInterval() > 1e-3 && use_collision_map)
    planning_monitor_->waitForMap();
  return true;
}

}

REGISTER_SPLINE_SMOOTHER(CubicSplineShortCutter, collision_free_spline_smoother::CubicSplineShortCutter)
