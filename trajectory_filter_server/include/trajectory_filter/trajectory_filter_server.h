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

/** \author Mrinal Kalakrishnan */

#ifndef TRAJECTORY_FILTER_H_
#define TRAJECTORY_FILTER_H_

#include <ros/ros.h>
#include <filters/filter_chain.h>
#include <motion_planning_msgs/JointTrajectoryWithLimits.h>
#include <motion_planning_msgs/JointLimits.h>
#include <motion_planning_msgs/FilterJointTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace trajectory_filter
{

class TrajectoryFilterServer
{

public:
  TrajectoryFilterServer();
  virtual ~TrajectoryFilterServer();
  bool init();

private:
  bool filter(motion_planning_msgs::FilterJointTrajectory::Request &req, 
              motion_planning_msgs::FilterJointTrajectory::Response &resp);
  ros::NodeHandle priv_handle_, root_handle_;
  std::map<std::string, motion_planning_msgs::JointLimits> joint_limits_;
  filters::FilterChain<motion_planning_msgs::JointTrajectoryWithLimits> filter_chain_;
  void jointTrajectoryToJointTrajectoryWithLimits(const trajectory_msgs::JointTrajectory& joint_traj, motion_planning_msgs::JointTrajectoryWithLimits& waypoint_traj);

  ros::ServiceServer filter_trajectory_service_;
};

}

#endif /* JOINT_WAYPOINT_CONTROLLER_H_ */
