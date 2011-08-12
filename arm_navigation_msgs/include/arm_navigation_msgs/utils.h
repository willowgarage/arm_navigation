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
void addGoalConstraintToMoveArmGoal(const arm_navigation_msgs::SimplePoseConstraint &pose_constraint, arm_navigation_msgs::MoveArmGoal &move_arm_goal)
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

bool isPoseGoal(arm_navigation_msgs::GetMotionPlan::Request &req)
{
  if (req.motion_plan_request.goal_constraints.joint_constraints.empty() &&         // we have no joint constraints on the goal,
      req.motion_plan_request.goal_constraints.position_constraints.size() == 1 &&      // we have a single position constraint on the goal
      req.motion_plan_request.goal_constraints.orientation_constraints.size() ==  1)  // that is active on all 6 DOFs
    return true;
  else
    return false;
}       

bool hasPoseGoal(arm_navigation_msgs::GetMotionPlan::Request &req)
{
  if (req.motion_plan_request.goal_constraints.position_constraints.size() >= 1 &&      // we have a single position constraint on the goal
      req.motion_plan_request.goal_constraints.orientation_constraints.size() >=  1)  // that is active on all 6 DOFs
    return true;
  else
    return false;
}       

bool isJointGoal(arm_navigation_msgs::GetMotionPlan::Request &req)
{
  if (req.motion_plan_request.goal_constraints.position_constraints.empty() && 
      req.motion_plan_request.goal_constraints.orientation_constraints.empty() && 
      !req.motion_plan_request.goal_constraints.joint_constraints.empty())
    return true;
  else
    return false;
}                   

void discretizeTrajectory(const trajectory_msgs::JointTrajectory &trajectory, 
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

void printConstraints(const arm_navigation_msgs::Constraints &constraints)
{
  ROS_DEBUG("Number constraints (Joint,Position, Orientation): %d %d %d");
  for(unsigned int i=0; i < constraints.joint_constraints.size(); i++)
    printJointConstraint(constraints.joint_constraint[i],i);
  for(unsigned int i=0; i < constraints.joint_constraints.size(); i++)
    printPositionConstraint(constraints.position_constraint[i],i);
  for(unsigned int i=0; i < constraints.joint_constraints.size(); i++)
    printOrientationConstraint(constraints.orientation_constraint[i],i);
}

void printJointConstraint(const arm_navigation_msgs::JointConstraint &constraint, 
                          const unsigned int &constraint_number)
{
  ROS_DEBUG("JC(%d):: %s: desired: %f, range desired: (%f,%f)",(int)constraint_number,constraint.joint_name.c_str(),constraint.position,constraint.position-constraint.tolerance_below,constraint.position+constraint.tolerance_above);
}

void printPositionConstraint(const arm_navigation_msgs::PositionConstraint &pc, 
                             const unsigned int &constraint_number)
{
  ROS_DEBUG("PC(%d):: %s, frame_id: %s",constraint_number,pc.link_name.c_str(),pc.header.frame_id.c_str());
  ROS_DEBUG("PC(%d):: Desired position: (%f,%f,%f)",pc.position.x,pc.position.y,pc.position.z);
  if(pc.constraint_region_shape.type == pc.constraint_region_shape.BOX)
  {
    if(pc.constraint_region_shape.dimensions == 3)
      ROS_DEBUG("PC(%d):: Contraint Region Box Extents (L(x),B(y),H(z)): (%f,%f,%f)",pc.constraint_region_shape.dimensions[0],pc.constraint_region_shape.dimensions[1],pc.constraint_region_shape.dimensions[2]);    
    ROS_DEBUG("PC(%d):: Constraint Region Orientation: (%f,%f,%f,%f)",pc.constraint_region_orientation.x,pc.constraint_region_orientation.y,pc.constraint_region_orientation.z,pc.constraint_region_orientation.w);    
  }
}

void printOrientationConstraint(const arm_navigation_msgs::OrientationConstraint &oc,
                                const unsigned int &constraint_number)
{
  if(oc.type == oc.LINK_FRAME)
    ROS_DEBUG("OC(%d):: %s, frame_id: %s, type: LINK_FRAME",constraint_number,oc.link_name.c_str(),oc.header.frame_id.c_str());
  else if(oc.type == oc.HEADER_FRAME)
    ROS_DEBUG("OC(%d):: %s: frame_id: %s, type: HEADER_FRAME",constraint_number,oc.link_name.c_str(),oc.header.frame_id.c_str());
  ROS_DEBUG("OC(%d):: Quaternion desired: %f %f %f %f",oc.orientation.x,oc.orientation.y,oc.orientation.z,oc.orientation.w);
  ROS_DEBUG("OC(%d):: Tolerance(RPY): %f %f %f ",oc.absolute_roll_tolerance,oc.absolute_pitch_tolerance,oc.absolute_yaw_tolerance);
}


}
