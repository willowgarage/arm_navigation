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

#ifndef TRAJECTORY_FILTER_SERVER_H_
#define TRAJECTORY_FILTER_SERVER_H_

#include <ros/ros.h>
#include <filters/filter_chain.h>
#include <arm_navigation_msgs/FilterJointTrajectory.h>
#include <arm_navigation_msgs/FilterJointTrajectoryWithConstraints.h>
#include <urdf/model.h>

namespace trajectory_filter_server
{

static const int FILTER_JOINT_TRAJECTORY = 0;
static const int FILTER_JOINT_TRAJECTORY_WITH_CONSTRAINTS = 1;

class TrajectoryFilterServer
{

public:
  TrajectoryFilterServer();
  virtual ~TrajectoryFilterServer();
  bool init();

private:
  bool filter(arm_navigation_msgs::FilterJointTrajectory::Request &req, 
              arm_navigation_msgs::FilterJointTrajectory::Response &resp);

  bool filterConstraints(arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Request &req, 
                         arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Response &resp);

  //  void jointTrajectoryToJointTrajectoryWithLimits(const trajectory_msgs::JointTrajectory& joint_traj, 
  //arm_navigation_msgs::JointTrajectoryWithLimits& waypoint_traj);

  void getLimits(const trajectory_msgs::JointTrajectory& trajectory, 
                 std::vector<arm_navigation_msgs::JointLimits>& limits_out);

  ros::NodeHandle private_handle_, root_handle_;
  ros::ServiceServer filter_service_, filter_constraints_service_;
  std::map<std::string, arm_navigation_msgs::JointLimits> joint_limits_;

  int service_type_;

  filters::FilterChain<arm_navigation_msgs::FilterJointTrajectory> filter_chain_;
  filters::FilterChain<arm_navigation_msgs::FilterJointTrajectoryWithConstraints> filter_constraints_chain_;

  urdf::Model urdf_model_;

  bool loadURDF();
  bool use_safety_limits_;

};

}

#endif /* TRAJECTORY_FILTER_SERVER_H_ */
