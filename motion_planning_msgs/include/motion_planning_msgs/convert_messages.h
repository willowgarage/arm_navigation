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

#include <motion_planning_msgs/RobotTrajectory.h>
#include <motion_planning_msgs/RobotState.h>

#include <motion_planning_msgs/JointConstraint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <motion_planning_msgs/JointPath.h>
#include <sensor_msgs/JointState.h>

#include <motion_planning_msgs/OrientationConstraint.h>
#include <motion_planning_msgs/SimplePoseConstraint.h>
#include <motion_planning_msgs/PositionConstraint.h>
#include <motion_planning_msgs/ArmNavigationErrorCodes.h>
#include <motion_planning_msgs/GetMotionPlan.h>

namespace motion_planning_msgs
{

  /**
     @brief Convert a joint path message to a joint trajectory message
     @param A reference to the input joint path message
     @return The joint trajectory message that incorporates the joint path information with zero segment durations, velocities and accelerations.
   */
inline  trajectory_msgs::JointTrajectory jointPathToJointTrajectory(const motion_planning_msgs::JointPath &path)
  {
    trajectory_msgs::JointTrajectory trajectory;
    trajectory.header = path.header;
    trajectory.joint_names = path.joint_names;
    trajectory.points.resize(path.points.size());
    for(unsigned int i=0; i< path.points.size(); i++)
      trajectory.points[i].positions = path.points[i].positions;
    return trajectory;
  }

  /**
     @brief Convert a joint trajectory message to a joint path message (this conversion will cause loss of velocity and acceleration information)
     @param The input trajectory
     @return The joint path that contains only the position information from the joint trajectory.
   */
inline  motion_planning_msgs::JointPath jointTrajectoryToJointPath(const trajectory_msgs::JointTrajectory &trajectory)
  {
    motion_planning_msgs::JointPath path;   
    path.header = trajectory.header;
    path.joint_names = trajectory.joint_names;
    path.points.resize(trajectory.points.size());
    for(unsigned int i=0; i< trajectory.points.size(); i++)
      path.points[i].positions = trajectory.points[i].positions;
    return path;
  }

  /**
     @brief Convert a joint state to a joint path point message
     @param The input joint state message
     @return The output joint path point message
  */
inline  motion_planning_msgs::JointPathPoint jointStateToJointPathPoint(const sensor_msgs::JointState &state)
  {
    motion_planning_msgs::JointPathPoint point;
    point.positions = state.position;
    return point;
  }

  /**
     @brief Convert a joint state to a joint trajectory point message
     @param The input joint state message
     @return The output joint trajectory point message only contains position information from the joint state message.
  */
inline trajectory_msgs::JointTrajectoryPoint jointStateToJointTrajectoryPoint(const sensor_msgs::JointState &state)
  {
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = state.position;
    return point;
  }

  /**
     @brief Convert a multi-dof state to a multi-dof joint trajectory point message
     @param The input multi-dof state
     @return The output multi-dof joint trajectory point .
  */
inline motion_planning_msgs::MultiDOFJointTrajectoryPoint multiDOFJointStateToMultiDOFJointTrajectoryPoint(const motion_planning_msgs::MultiDOFJointState &state)
  {
    motion_planning_msgs::MultiDOFJointTrajectoryPoint point;
    point.poses = state.poses;
    point.time_from_start = ros::Duration(0.0);
    return point;
  }

  /**
     @brief Convert a robot state to a robot trajectory message
     @param The input robot state
     @return The output robot trajectory point
  */
inline void robotStateToRobotTrajectoryPoint(const motion_planning_msgs::RobotState &state,
                                             trajectory_msgs::JointTrajectoryPoint &point,
                                             motion_planning_msgs::MultiDOFJointTrajectoryPoint &multi_dof_point)
  {
    point = jointStateToJointTrajectoryPoint(state.joint_state);
    multi_dof_point = multiDOFJointStateToMultiDOFJointTrajectoryPoint(state.multi_dof_joint_state);
    return;
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
inline  motion_planning_msgs::JointPath jointConstraintsToJointPath(const std::vector<motion_planning_msgs::JointConstraint> &constraints)
  {
    motion_planning_msgs::JointPath path;
    if(constraints.empty())
      return path;
    sensor_msgs::JointState state = jointConstraintsToJointState(constraints);
    motion_planning_msgs::JointPathPoint point = jointStateToJointPathPoint(state);
    //    path.header = constraints[0].header;
    path.points.push_back(point);
    path.joint_names = state.name;
    return path;
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

  /**
     @brief Convert an error code into a string value
     @param error_code The input error code
     @return The resultant string message
   */
 inline std::string armNavigationErrorCodeToString(const motion_planning_msgs::ArmNavigationErrorCodes &error_code)
 {
   std::string result;
   if(error_code.val == error_code.PLANNING_FAILED)
     result = "Planning failed";
   else if(error_code.val == error_code.SUCCESS)
     result = "Success";
   else if(error_code.val == error_code.TIMED_OUT)
     result = "Timed out";
   else if (error_code.val == error_code.START_STATE_IN_COLLISION)
     result = "Start state in collision";
   else if (error_code.val == error_code.START_STATE_VIOLATES_PATH_CONSTRAINTS)
     result = "Start state violates path constraints";
   else if (error_code.val == error_code.GOAL_IN_COLLISION)
     result = "Goal in violation";
   else if (error_code.val == error_code.GOAL_VIOLATES_PATH_CONSTRAINTS)
     result = "Goal violates path constraints";
   else if (error_code.val == error_code.INVALID_ROBOT_STATE)
     result = "Initial robot state invalid";
   else if (error_code.val == error_code.INCOMPLETE_ROBOT_STATE)
     result = "Initial robot state incomplete";
   else if (error_code.val == error_code.INVALID_PLANNER_ID)
     result = "Invalid planner id";
   else if (error_code.val == error_code.INVALID_NUM_PLANNING_ATTEMPTS)
     result = "Invalid num planning attempts (must be > 0)";
   else if (error_code.val == error_code.INVALID_ALLOWED_PLANNING_TIME)
     result = "Invalid allowed planning time (must be > 0)";
   else if (error_code.val == error_code.INVALID_GROUP_NAME)
     result = "Invalid group name for planning";
   else if (error_code.val == error_code.INVALID_GOAL_JOINT_CONSTRAINTS)
     result = "Invalid goal joint constraints";
   else if (error_code.val == error_code.INVALID_GOAL_POSITION_CONSTRAINTS)
     result = "Invalid goal position constraints";
   else if (error_code.val == error_code.INVALID_GOAL_ORIENTATION_CONSTRAINTS)
     result = "Invalid goal orientation constraints";
   else if (error_code.val == error_code.INVALID_PATH_JOINT_CONSTRAINTS)
     result = "Invalid path joint constraints";
   else if (error_code.val == error_code.INVALID_PATH_POSITION_CONSTRAINTS)
     result = "Invalid path position constraints";
   else if (error_code.val == error_code.INVALID_PATH_ORIENTATION_CONSTRAINTS)
     result = "Invalid path orientation constraints";
   else if (error_code.val == error_code.INVALID_TRAJECTORY)
     result = "Invalid trajectory";
   else if (error_code.val == error_code.INVALID_INDEX)
     result = "Invalid index for trajectory check";
   else if (error_code.val == error_code.JOINT_LIMITS_VIOLATED)
     result = "Joint limits violated";
   else if (error_code.val == error_code.PATH_CONSTRAINTS_VIOLATED)
     result = "Path constraints violated";
   else if (error_code.val == error_code.COLLISION_CONSTRAINTS_VIOLATED)
     result = "Collision constraints violated";
   else if (error_code.val == error_code.GOAL_CONSTRAINTS_VIOLATED)
     result = "Collision constraints violated";
   else if (error_code.val == error_code.JOINTS_NOT_MOVING)
     result = "Joints not moving - robot may be stuck";
   else if (error_code.val == error_code.TRAJECTORY_CONTROLLER_FAILED)
     result = "Trajectory controller failed";
   else if (error_code.val == error_code.FRAME_TRANSFORM_FAILURE)
     result = "Frame transform failed";
   else if (error_code.val == error_code.COLLISION_CHECKING_UNAVAILABLE)
     result = "Collision checking unavailable";
   else if (error_code.val == error_code.ROBOT_STATE_STALE)
     result = "Robot state is not being updated";
   else if (error_code.val == error_code.SENSOR_INFO_STALE)
     result = "Sensor information is not being updated";
   else if (error_code.val == error_code.NO_IK_SOLUTION)
     result = "Inverse kinematics solution was not found";
   else if (error_code.val == error_code.IK_LINK_IN_COLLISION)
     result = "Inverse kinematics link was in collision";
   else if (error_code.val == error_code.INVALID_LINK_NAME)
     result = "Invalid link name";
   else if (error_code.val == error_code.NO_FK_SOLUTION)
     result = "No forward kinematics solution";
   else if (error_code.val == error_code.KINEMATICS_STATE_IN_COLLISION)
     result = "Current robot state is in collision";
   else if (error_code.val == error_code.INVALID_TIMEOUT)
     result = "Time given for planning invalid (must be > 0)";
   else
     result = "Unknown error code";
   return result;
 } 

  /**
     @brief Extract pose information from a position and orientation constraint into a pose stamped message
     @param The input position constraint
     @param The input orientation constraint
     @return The nominal position and orientation from the constraints are encoded into the output pose message
   */
  inline  bool constraintsToPoseStampedVector(const motion_planning_msgs::Constraints &constraints,
                                              std::vector<geometry_msgs::PoseStamped> &poses)
  {
    if(constraints.position_constraints.size() != constraints.orientation_constraints.size())
    {
      ROS_ERROR("Number of position constraints does not match number of orientation constraints");
      return false;
    }
    for(unsigned int i =0; i < constraints.position_constraints.size(); i++)
    {
      geometry_msgs::PoseStamped pose = poseConstraintsToPoseStamped(constraints.position_constraints[i],
                                                                     constraints.orientation_constraints[i]);
      poses.push_back(pose);
    }
    return true;
  }


  /**
     @brief Extract pose information from a position and orientation constraint into a multi dof joint state
     @param The input position constraint
     @param The input orientation constraint
     @return The nominal position and orientation from the constraints are encoded into the output pose message
   */
  inline motion_planning_msgs::MultiDOFJointState poseConstraintsToMultiDOFJointState(const std::vector<motion_planning_msgs::PositionConstraint> &position_constraints, 
                                                                                      const std::vector<motion_planning_msgs::OrientationConstraint> &orientation_constraints)
  {
    motion_planning_msgs::MultiDOFJointState multi_dof_joint_state;
    if(position_constraints.size() != orientation_constraints.size())
      return multi_dof_joint_state;
    for(unsigned int i=0; i < position_constraints.size(); i++)
    {
      if(position_constraints[i].header.frame_id != orientation_constraints[i].header.frame_id)
      {
        ROS_ERROR("Frame id for position constraint %d does not match frame id for corresponding orientation constraint",i);
        return multi_dof_joint_state;
      }
      if(position_constraints[i].link_name != orientation_constraints[i].link_name)
      {
        ROS_ERROR("Link name for position constraint %d does not match link name for corresponding orientation constraint",i);
        return multi_dof_joint_state;
      }
    }
    return multi_dof_joint_state;
  }


}

#endif
