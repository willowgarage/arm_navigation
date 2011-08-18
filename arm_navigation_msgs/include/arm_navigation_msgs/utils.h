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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

#ifndef ARM_NAVIGATION_UTILS_H_
#define ARM_NAVIGATION_UTILS_H_

#include <ros/ros.h>
#include <algorithm>
#include <arm_navigation_msgs/CollisionOperation.h>
#include <arm_navigation_msgs/SimplePoseConstraint.h>
#include <arm_navigation_msgs/convert_messages.h>
#include <arm_navigation_msgs/MoveArmGoal.h>

namespace arm_navigation_msgs
{

/** @brief Add a goal constraint to the move arm action goal.
    @param A reference to a simple pose constraint.
    @param A reference to a move arm goal message. The pose constraint will be added to the goal message as a position and orientation constraint.
*/
inline void addGoalConstraintToMoveArmGoal(const arm_navigation_msgs::SimplePoseConstraint &pose_constraint, arm_navigation_msgs::MoveArmGoal &move_arm_goal)
{
  arm_navigation_msgs::PositionConstraint position_constraint;
  arm_navigation_msgs::OrientationConstraint orientation_constraint;
  poseConstraintToPositionOrientationConstraints(pose_constraint,position_constraint,orientation_constraint);
  move_arm_goal.motion_plan_request.goal_constraints.position_constraints.push_back(position_constraint);
  move_arm_goal.motion_plan_request.goal_constraints.orientation_constraints.push_back(orientation_constraint);
}

/** @brief Generate ordered collision operates that disable all collisions in a vector of names except
           for a specied vector of names
    @param All names
    @param Names to exclude
    @param Vector for appending collision_operations
*/

inline void generateDisableAllowedCollisionsWithExclusions(const std::vector<std::string>& all_names,
                                                           const std::vector<std::string>& exclude_names,
                                                           std::vector<arm_navigation_msgs::CollisionOperation>& collision_operations) 
{
  for(std::vector<std::string>::const_iterator it = all_names.begin();
      it != all_names.end();
      it++) {
    if(std::find(exclude_names.begin(), exclude_names.end(), *it) == exclude_names.end()) {
      arm_navigation_msgs::CollisionOperation coll;
      coll.object1 = *it;
      coll.object2 = coll.COLLISION_SET_OBJECTS;
      coll.operation = coll.DISABLE;
      collision_operations.insert(collision_operations.end(),coll);
      coll.object2 = coll.COLLISION_SET_ATTACHED_OBJECTS;
      collision_operations.insert(collision_operations.end(),coll);
      for(std::vector<std::string>::const_iterator it2 = all_names.begin();
          it2 != all_names.end();
          it2++) {
        if(*it != *it2 && std::find(exclude_names.begin(), exclude_names.end(), *it2) == exclude_names.end()) {
          coll.object1 = *it;
          coll.object2 = *it2;
          collision_operations.insert(collision_operations.end(),coll);
        }
      }
    }
  }
}

inline bool isPoseGoal(arm_navigation_msgs::GetMotionPlan::Request &req)
{
  if (req.motion_plan_request.goal_constraints.joint_constraints.empty() &&         // we have no joint constraints on the goal,
      req.motion_plan_request.goal_constraints.position_constraints.size() == 1 &&      // we have a single position constraint on the goal
      req.motion_plan_request.goal_constraints.orientation_constraints.size() ==  1)  // that is active on all 6 DOFs
    return true;
  else
    return false;
}       

inline bool hasPoseGoal(arm_navigation_msgs::GetMotionPlan::Request &req)
{
  if (req.motion_plan_request.goal_constraints.position_constraints.size() >= 1 &&      // we have a single position constraint on the goal
      req.motion_plan_request.goal_constraints.orientation_constraints.size() >=  1)  // that is active on all 6 DOFs
    return true;
  else
    return false;
}       

inline bool isJointGoal(arm_navigation_msgs::GetMotionPlan::Request &req)
{
  if (req.motion_plan_request.goal_constraints.position_constraints.empty() && 
      req.motion_plan_request.goal_constraints.orientation_constraints.empty() && 
      !req.motion_plan_request.goal_constraints.joint_constraints.empty())
    return true;
  else
    return false;
}                   

inline void discretizeTrajectory(const trajectory_msgs::JointTrajectory &trajectory, 
                                 trajectory_msgs::JointTrajectory &trajectory_out,
                                 const double &trajectory_discretization)
{    
  trajectory_out.joint_names = trajectory.joint_names;
  for(unsigned int i=1; i < trajectory.points.size(); i++)
  {
    double diff = 0.0;      
    for(unsigned int j=0; j < trajectory.points[i].positions.size(); j++)
    {
      double start = trajectory.points[i-1].positions[j];
      double end   = trajectory.points[i].positions[j];
      if(fabs(end-start) > diff)
        diff = fabs(end-start);        
    }
    int num_intervals =(int) (diff/trajectory_discretization+1.0);
    
    for(unsigned int k=0; k < (unsigned int) num_intervals; k++)
    {
      trajectory_msgs::JointTrajectoryPoint point;
      for(unsigned int j=0; j < trajectory.points[i].positions.size(); j++)
      {
        double start = trajectory.points[i-1].positions[j];
        double end   = trajectory.points[i].positions[j];
        point.positions.push_back(start + (end-start)*k/num_intervals);
      }
      point.time_from_start = ros::Duration(trajectory.points[i].time_from_start.toSec() + k* (trajectory.points[i].time_from_start - trajectory.points[i-1].time_from_start).toSec()/num_intervals);
      trajectory_out.points.push_back(point);
    }
  }
  trajectory_out.points.push_back(trajectory.points.back());
}


inline void printJointConstraint(const arm_navigation_msgs::JointConstraint &constraint, 
                                 const unsigned int &constraint_number)
{
  ROS_DEBUG("JC(%d):: %s: desired: %f, range desired: (%f,%f)",(int)constraint_number,constraint.joint_name.c_str(),constraint.position,constraint.position-constraint.tolerance_below,constraint.position+constraint.tolerance_above);
}

inline void printPositionConstraint(const arm_navigation_msgs::PositionConstraint &pc, 
                                    const unsigned int &constraint_number)
{
  ROS_DEBUG("PC(%d):: %s, frame_id: %s",constraint_number,pc.link_name.c_str(),pc.header.frame_id.c_str());
  ROS_DEBUG("PC(%d):: Desired position: (%f,%f,%f)",constraint_number,pc.position.x,pc.position.y,pc.position.z);
  if(pc.constraint_region_shape.type == pc.constraint_region_shape.BOX)
  {
    if(pc.constraint_region_shape.dimensions.size() == 3)
      ROS_DEBUG("PC(%d):: Contraint Region Box Extents (L(x),B(y),H(z)): (%f,%f,%f)",constraint_number,pc.constraint_region_shape.dimensions[0],pc.constraint_region_shape.dimensions[1],pc.constraint_region_shape.dimensions[2]);    
    ROS_DEBUG("PC(%d):: Constraint Region Orientation: (%f,%f,%f,%f)",constraint_number,pc.constraint_region_orientation.x,pc.constraint_region_orientation.y,pc.constraint_region_orientation.z,pc.constraint_region_orientation.w);    
  }
}

inline void printOrientationConstraint(const arm_navigation_msgs::OrientationConstraint &oc,
                                      const unsigned int &constraint_number)
{
  if(oc.type == oc.LINK_FRAME)
    ROS_DEBUG("OC(%d):: %s, frame_id: %s, type: LINK_FRAME",constraint_number,oc.link_name.c_str(),oc.header.frame_id.c_str());
  else if(oc.type == oc.HEADER_FRAME)
    ROS_DEBUG("OC(%d):: %s: frame_id: %s, type: HEADER_FRAME",constraint_number,oc.link_name.c_str(),oc.header.frame_id.c_str());
  ROS_DEBUG("OC(%d):: Quaternion desired: %f %f %f %f",constraint_number,oc.orientation.x,oc.orientation.y,oc.orientation.z,oc.orientation.w);
  ROS_DEBUG("OC(%d):: Tolerance(RPY): %f %f %f ",constraint_number,oc.absolute_roll_tolerance,oc.absolute_pitch_tolerance,oc.absolute_yaw_tolerance);
}

inline void printConstraints(const arm_navigation_msgs::Constraints &constraints, const bool &print_joint=true, const bool print_position=true, const bool &print_orientation=true)
{
  ROS_DEBUG("Number constraints (Joint,Position, Orientation): %d %d %d",
            (int) constraints.joint_constraints.size(),
            (int) constraints.position_constraints.size(),
            (int) constraints.orientation_constraints.size());
  if(print_joint)
    for(unsigned int i=0; i < constraints.joint_constraints.size(); i++)
      printJointConstraint(constraints.joint_constraints[i],i);
  if(print_position)
    for(unsigned int i=0; i < constraints.joint_constraints.size(); i++)
      printPositionConstraint(constraints.position_constraints[i],i);
  if(print_orientation)
    for(unsigned int i=0; i < constraints.joint_constraints.size(); i++)
      printOrientationConstraint(constraints.orientation_constraints[i],i);
}

inline arm_navigation_msgs::Constraints getPoseConstraintsForGroup(const std::vector<std::string> &link_names,
                                                                   const arm_navigation_msgs::Constraints &constraints)
{
  arm_navigation_msgs::Constraints group_constraints;
  for(unsigned int i = 0; i < link_names.size(); i++)
  {
    for(unsigned int j=0; j < constraints.position_constraints.size(); j++)
      if(constraints.position_constraints[j].link_name == link_names[i])
        group_constraints.position_constraints.push_back(constraints.position_constraints[j]);
    for(unsigned int j=0; j < constraints.orientation_constraints.size(); j++)
      if(constraints.orientation_constraints[j].link_name == link_names[i])
        group_constraints.orientation_constraints.push_back(constraints.orientation_constraints[j]);
  }
  return group_constraints;
}

inline void clearPoseConstraintsForGroup(arm_navigation_msgs::Constraints &constraints,
                                         const std::vector<std::string> &link_names)
{
  for(unsigned int i =0; i < link_names.size(); i++)
  {
    for(std::vector<arm_navigation_msgs::PositionConstraint>::iterator it=constraints.position_constraints.begin(); 
        it != constraints.position_constraints.end(); 
        it++)
    {
      if(it->link_name == link_names[i])
        constraints.position_constraints.erase(it);
    }
    for(std::vector<arm_navigation_msgs::OrientationConstraint>::iterator it=constraints.orientation_constraints.begin(); 
        it != constraints.orientation_constraints.end(); 
        it++)
    {
      if(it->link_name == link_names[i])
        constraints.orientation_constraints.erase(it);  
    }
  }
}

inline void getJointValueMap(const std::vector<arm_navigation_msgs::JointConstraint> &joint_constraints,
                             std::map<std::string, double> &joint_value_map)
{
  for(unsigned int i=0; i < joint_constraints.size(); i++)
    joint_value_map[joint_constraints[i].joint_name] = joint_constraints[i].position;
}
      
inline std::vector<arm_navigation_msgs::JointConstraint> getJointConstraints(const std::vector<std::vector<double> > &joint_angles,
                                                                             const std::vector<std::vector<std::string> > &joint_names,
                                                                             const double &tolerance)
{
  std::vector<arm_navigation_msgs::JointConstraint> constraints;
  if(joint_angles.size() != joint_names.size())
  {
    ROS_ERROR("Joint angles size does not match names size");
    return constraints;
  }
  for(unsigned int i=0; i < joint_angles.size(); i++)
  {
    if(joint_angles[i].size() != joint_names[i].size())
    {
      ROS_ERROR("Joint angles size does not match names size");
      constraints.clear();
      return constraints;
    }
    for(unsigned int j=0; j < joint_angles[i].size(); j++)
    {
      arm_navigation_msgs::JointConstraint jc;
      jc.position = joint_angles[i][j];
      jc.tolerance_above = tolerance;
      jc.tolerance_below = tolerance;
      jc.joint_name = joint_names[i][j];
    }
  }
  return constraints;
}

inline void printJointTrajectory(const trajectory_msgs::JointTrajectory &joint_trajectory)
{
  ROS_DEBUG("Joint Trajectory");
  ROS_DEBUG("Frame id: %s, Stamp: %f, Num points: %d",joint_trajectory.header.frame_id.c_str(),joint_trajectory.header.stamp.toSec(),(int)joint_trajectory.points.size());

  std::stringstream joint_stream;
  for(unsigned int i=0; i < joint_trajectory.joint_names.size(); i++)
    joint_stream << joint_trajectory.joint_names[i] << ",";
  ROS_DEBUG("Joint names: %s",joint_stream.str().c_str());

  for(unsigned int i=0; i < joint_trajectory.points.size(); i++)
  {
    std::stringstream string_stream;
    for(unsigned int j =0; j < joint_trajectory.points[i].positions.size(); j++)
      string_stream << joint_trajectory.points[i].positions[j] << ",";
    ROS_DEBUG("%d: %s",i,string_stream.str().c_str());
  }
  ROS_DEBUG(" ");
}

}

#endif
