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

#ifndef UNNORMALIZE_JOINT_TRAJECTORY_H_
#define UNNORMALIZE_JOINT_TRAJECTORY_H_

#include <ros/ros.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
#include <spline_smoother/spline_smoother.h>
#include <spline_smoother/spline_smoother_utils.h>

namespace joint_normalization_filters
{

/**
 * \brief Scales the time intervals stretching them if necessary so that the trajectory conforms to velocity limits
 */
template <typename T>
class UnNormalizeJointTrajectory: public spline_smoother::SplineSmoother<T>
{
private:
  //! The node handle
  ros::NodeHandle nh_;

  //! A model of the robot to see which joints wrap around
  urdf::Model robot_model_;

  //! Flag that tells us if the robot model was initialized successfully
  bool robot_model_initialized_;

public:
  UnNormalizeJointTrajectory();
  virtual ~UnNormalizeJointTrajectory();

  virtual bool smooth(const T& trajectory_in, 
                      T& trajectory_out) const;
};

template <typename T>
UnNormalizeJointTrajectory<T>::UnNormalizeJointTrajectory()
{
  std::string urdf_xml,full_urdf_xml;
  nh_.param("urdf_xml", urdf_xml, std::string("robot_description"));
  if(!nh_.getParam(urdf_xml,full_urdf_xml))
  {
    ROS_ERROR("Could not load the xml from parameter server: %s\n", urdf_xml.c_str());
    robot_model_initialized_ = false;
  }
  else
  {
    robot_model_.initString(full_urdf_xml);
    robot_model_initialized_ = true;
  }
}

template <typename T>
UnNormalizeJointTrajectory<T>::~UnNormalizeJointTrajectory()
{
}

template <typename T>
bool UnNormalizeJointTrajectory<T>::smooth(const T& trajectory_in, 
                                      T& trajectory_out) const
{
  trajectory_out = trajectory_in;

  if (!spline_smoother::checkTrajectoryConsistency(trajectory_out))
    return false;

  if (!robot_model_initialized_) 
  {
    ROS_ERROR("Robot model not initialized; can not tell continuous joints");
    return false;
  }

  if(trajectory_in.request.start_state.joint_state.name.size() == 0) {
    ROS_WARN_STREAM("Unnormalizer requires the start state to be set");
    return false;
  }
  std::map<std::string, double> joint_values;
  for(unsigned int i = 0; i < trajectory_in.request.start_state.joint_state.name.size(); i++) {
    joint_values[trajectory_in.request.start_state.joint_state.name[i]] = trajectory_in.request.start_state.joint_state.position[i];
  }

  std::vector<double> current_values;
  std::vector<int> wraparound;
  trajectory_msgs::JointTrajectory input_trajectory = trajectory_in.request.trajectory;
  for (size_t i=0; i<input_trajectory.joint_names.size(); i++)
  {
    std::string name = input_trajectory.joint_names[i];
    if(joint_values.find(name) == joint_values.end()) {
      ROS_WARN_STREAM("No value set in start state for joint name " << name);
      return false;
    }
    //first waypoint is unnormalized relative to current joint states
    current_values.push_back(joint_values[name]);
    
    boost::shared_ptr<const urdf::Joint> joint = robot_model_.getJoint(name);
    if (joint.get() == NULL)
    {
      ROS_ERROR("Joint name %s not found in urdf model", name.c_str());
      return false;
    }
    if (joint->type == urdf::Joint::CONTINUOUS) wraparound.push_back(1);
    else wraparound.push_back(0);
  }

  trajectory_msgs::JointTrajectory normalized_trajectory = input_trajectory;
  for (size_t i=0; i<normalized_trajectory.points.size(); i++)
  {
    for (size_t j=0; j<normalized_trajectory.points[i].positions.size(); j++ )
    {
      if (!wraparound.at(j)) continue;
      double current = current_values.at(j);
      double traj = normalized_trajectory.points[i].positions[j];
      while ( current - traj > M_PI ) traj += 2*M_PI;
      while ( traj - current > M_PI ) traj -= 2*M_PI;
      ROS_DEBUG("Normalizing joint %s from %f to %f", normalized_trajectory.joint_names.at(j).c_str(), 
                normalized_trajectory.points[i].positions[j], traj);
      normalized_trajectory.points[i].positions[j] = traj;
      //all other waypoints are unnormalized relative to the previous waypoint
      current_values.at(j) = traj;
    }  
  }
  trajectory_out.request.trajectory = normalized_trajectory;
  return true;
}
}

#endif /* UNNORMALIZE_JOINT_TRAJECTORY_H_ */
