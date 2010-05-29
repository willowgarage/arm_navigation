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
#ifndef MOTION_PLANNING_CONVERT_MESSAGES_
#define MOTION_PLANNING_CONVERT_MESSAGES_

#include <ros/ros.h>
#include <tf/tf.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <motion_planning_msgs/JointConstraint.h>

#include <motion_planning_msgs/OrientationConstraint.h>
#include <motion_planning_msgs/SimplePoseConstraint.h>
#include <motion_planning_msgs/PositionConstraint.h>

namespace motion_planning_msgs
{

  /**
     @brief Convert a joint state to a joint trajectory point message
     @param The input joint state message
     @return The output joint trajectory point message only contains position information from the joint state message.
  */
inline  trajectory_msgs::JointTrajectoryPoint jointStateToJointTrajectoryPoint(const sensor_msgs::JointState &state)
  {
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = state.position;
    return point;
  }

  /**
     @brief Extract joint position information from a set of joint constraints into a joint state message
     @param The input vector of joint constraints
     @return The nominal joint positions from the joint constraints are encoded into the joint state message.
   */
inline  sensor_msgs::JointState jointConstraintsToJointState(const std::vector<motion_planning_msgs::JointConstraint> &constraints)
  {
    sensor_msgs::JointState state;
    state.name.resize(constraints.size());
    state.position.resize(constraints.size());
    for(unsigned int i=0; i < constraints.size(); i++)
    {
      state.name[i] = constraints[i].joint_name;
      state.position[i] = constraints[i].position;
    }
    return state;
  }

  /**
     @brief Extract joint position information from a set of joint constraints into a joint state message
     @param The input vector of joint constraints
     @return The nominal joint positions from the joint constraints are encoded into the joint state message.
   */
inline  trajectory_msgs::JointTrajectory jointConstraintsToJointTrajectory(const std::vector<motion_planning_msgs::JointConstraint> &constraints)
  {
    trajectory_msgs::JointTrajectory path;
    if(constraints.empty())
      return path;
    sensor_msgs::JointState state = jointConstraintsToJointState(constraints);
    trajectory_msgs::JointTrajectoryPoint point = jointStateToJointTrajectoryPoint(state);
    //    path.header = constraints[0].header;
    path.points.push_back(point);
    path.joint_names = state.name;
    return path;
  }

  /**
     @brief Extract pose information from a position and orientation constraint into a pose stamped message
     @param The input position constraint
     @param The input orientation constraint
     @return The nominal position and orientation from the constraints are encoded into the output pose message
   */
inline  geometry_msgs::PoseStamped poseConstraintsToPoseStamped(const motion_planning_msgs::PositionConstraint &position_constraint, const motion_planning_msgs::OrientationConstraint &orientation_constraint)
  {
    geometry_msgs::PoseStamped pose_stamped;
    btQuaternion tmp_quat;
    pose_stamped.header = position_constraint.header;
    pose_stamped.pose.position = position_constraint.position;
    //    tmp_quat.setRPY(orientation_constraint.orientation.x,orientation_constraint.orientation.y,orientation_constraint.orientation.z);
    //    tf::quaternionTFToMsg(tmp_quat,pose_stamped.pose.orientation);
    pose_stamped.pose.orientation = orientation_constraint.orientation;
    return pose_stamped;
  }

  /**
     @brief Create a joint state from a std vector of names and values
     @param The input vector of joint names
     @param The input vector of joint values
     @return The resultant joint state
   */
inline  sensor_msgs::JointState createJointState(std::vector<std::string> joint_names, std::vector<double> joint_values)
  {
    sensor_msgs::JointState state;
    state.name = joint_names;
    state.position = joint_values;
    return state;
  }

  /**
     @brief Convert a simple pose constraint into a position and orientation constraint 
     @param The input pose constraint of SimplePoseConstraint form
     @param The output position constraint
     @return The output orientation constraint
   */
inline  void poseConstraintToPositionOrientationConstraints(const motion_planning_msgs::SimplePoseConstraint &pose_constraint, motion_planning_msgs::PositionConstraint &position_constraint, motion_planning_msgs::OrientationConstraint &orientation_constraint)
  {
    position_constraint.header = pose_constraint.header;
    position_constraint.link_name = pose_constraint.link_name;
    position_constraint.position = pose_constraint.pose.position;
    position_constraint.constraint_region_shape.type = geometric_shapes_msgs::Shape::BOX;
    position_constraint.constraint_region_shape.dimensions.push_back(2*pose_constraint.absolute_position_tolerance.x);
    position_constraint.constraint_region_shape.dimensions.push_back(2*pose_constraint.absolute_position_tolerance.y);
    position_constraint.constraint_region_shape.dimensions.push_back(2*pose_constraint.absolute_position_tolerance.z);

    position_constraint.constraint_region_orientation.x = 0.0;
    position_constraint.constraint_region_orientation.y = 0.0;
    position_constraint.constraint_region_orientation.z = 0.0;
    position_constraint.constraint_region_orientation.w = 1.0;

    position_constraint.weight = 1.0;

    orientation_constraint.header = pose_constraint.header;
    orientation_constraint.link_name = pose_constraint.link_name;
    orientation_constraint.orientation = pose_constraint.pose.orientation;
    orientation_constraint.type = pose_constraint.orientation_constraint_type;

    orientation_constraint.absolute_roll_tolerance = pose_constraint.absolute_roll_tolerance;
    orientation_constraint.absolute_pitch_tolerance = pose_constraint.absolute_pitch_tolerance;
    orientation_constraint.absolute_yaw_tolerance = pose_constraint.absolute_yaw_tolerance;
    orientation_constraint.weight = 1.0;
  }


  /**
     @brief Convert a stamped pose into a position and orientation constraint 
     @param The input pose stamped
     @param The output position constraint
     @return The output orientation constraint
   */
inline  void poseStampedToPositionOrientationConstraints(const geometry_msgs::PoseStamped &pose_stamped, const std::string &link_name, motion_planning_msgs::PositionConstraint &position_constraint, motion_planning_msgs::OrientationConstraint &orientation_constraint)
  {
    position_constraint.header = pose_stamped.header;
    position_constraint.link_name = link_name;
    position_constraint.position = pose_stamped.pose.position;
    position_constraint.weight = 1.0;
    position_constraint.constraint_region_shape.type = geometric_shapes_msgs::Shape::BOX;
    position_constraint.constraint_region_shape.dimensions.push_back(0.01);
    position_constraint.constraint_region_shape.dimensions.push_back(0.01);
    position_constraint.constraint_region_shape.dimensions.push_back(0.01);

    orientation_constraint.header = pose_stamped.header;
    orientation_constraint.link_name = link_name;
    orientation_constraint.orientation = pose_stamped.pose.orientation;

    orientation_constraint.absolute_roll_tolerance  = 0.01;
    orientation_constraint.absolute_pitch_tolerance = 0.01;
    orientation_constraint.absolute_yaw_tolerance   = 0.01;
    orientation_constraint.weight = 1.0;
  }


  /**
     @brief Print the joint state information
     @param The joint state information to be printed
   */
inline void printJointState(const sensor_msgs::JointState &joint_state)
 {
   ROS_INFO("frame_id: %s stamp: %f",joint_state.header.frame_id.c_str(),joint_state.header.stamp.toSec());
   if(joint_state.name.size() != joint_state.position.size())
     ROS_ERROR("Size of joint_names field: %d does not match size of positions field: %d",(int) joint_state.name.size(),(int) joint_state.position.size());
   else
     {
       for(unsigned int i=0; i< joint_state.name.size(); i++)
	 {
	   ROS_INFO("Joint name: %s, position: %f",joint_state.name[i].c_str(),joint_state.position[i]);
	 }
     }
 } 

}

#endif
