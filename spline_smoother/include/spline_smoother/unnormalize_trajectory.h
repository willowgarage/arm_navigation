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

/** \author Matei Ciocarlei, Sachin Chitta */

#ifndef UNNORMALIZE_TRAJECTORY_H_
#define UNNORMALIZE_TRAJECTORY_H_

#include <ros/ros.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
#include <spline_smoother/spline_smoother.h>

namespace spline_smoother
{

/**
 * \brief Scales the time intervals stretching them if necessary so that the trajectory conforms to velocity limits
 */
class UnNormalizeTrajectory: public SplineSmoother
{
private:
  //! The node handle
  ros::NodeHandle nh_;

  //! A model of the robot to see which joints wrap around
  urdf::Model robot_model_;

  //! Flag that tells us if the robot model was initialized successfully
  bool robot_model_initialized_;

  //! The latest joint states received
  sensor_msgs::JointState::ConstPtr raw_joint_states_;

  //! Subscriber to joint states topic
  ros::Subscriber joint_states_sub_;

  //! Remembers the latest joint state
  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& states);

public:
  UnNormalizeTrajectory();
  virtual ~UnNormalizeTrajectory();

  virtual bool smooth(const motion_planning_msgs::JointTrajectoryWithLimits& trajectory_in, motion_planning_msgs::JointTrajectoryWithLimits& trajectory_out) const;
};
}

#endif /* UNNORMALIZE_TRAJECTORY_H_ */
