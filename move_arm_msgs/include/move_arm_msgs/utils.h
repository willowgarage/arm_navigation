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
#ifndef MOVE_ARM_MSGS_UTILS_
#define MOVE_ARM_MSGS_UTILS_

#include <ros/ros.h>
//#include <tf/tf.h>
//#include <tf/transform_datatypes.h>

#include <motion_planning_msgs/SimplePoseConstraint.h>
#include <motion_planning_msgs/convert_messages.h>
#include <move_arm_msgs/MoveArmGoal.h>

namespace move_arm_msgs
{
  /** @brief Add a goal constraint to the move arm action goal.
      @param A reference to a simple pose constraint.
      @param A reference to a move arm goal message. The pose constraint will be added to the goal message as a position and orientation constraint.
   */
  void addGoalConstraintToMoveArmGoal(const motion_planning_msgs::SimplePoseConstraint &pose_constraint, move_arm_msgs::MoveArmGoal &move_arm_goal)
  {
    motion_planning_msgs::PositionConstraint position_constraint;
    motion_planning_msgs::OrientationConstraint orientation_constraint;
    poseConstraintToPositionOrientationConstraints(pose_constraint,position_constraint,orientation_constraint);
    move_arm_goal.motion_plan_request.goal_constraints.position_constraints.push_back(position_constraint);
    move_arm_goal.motion_plan_request.goal_constraints.orientation_constraints.push_back(orientation_constraint);
  }
}

#endif
