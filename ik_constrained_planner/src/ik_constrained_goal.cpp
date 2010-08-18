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

/** \author Sachin Chitta, Ioan Sucan */

#include <ik_constrained_planner/ik_constrained_goal.h>

namespace ik_constrained_planner
{    	
bool IKConstrainedGoal::isSatisfied(const ompl::base::State *state, 
                                    double *dist) const
{
  if(dist)
  {
    // We know that the state is (x,y,z), (roll,pitch,yaw), redundancy
    *dist = distanceGoal(state);
  }

  btMatrix3x3 orientation;
  orientation.setRPY(state->values[3],state->values[4],state->values[5]);
  bool orientation_satisfied = orientation_constraint_evaluator_.isSatisfied(orientation);
  btVector3 position(state->values[0],state->values[1],state->values[2]);
  bool position_satisfied = position_constraint_evaluator_.isSatisfied(position);
  bool joint_satisfied = joint_constraint_evaluator_.isSatisfied(state->values[6]);

  return (joint_satisfied && position_satisfied && orientation_satisfied);

}

double IKConstrainedGoal::distanceGoal(const ompl::base::State *state) const
{  
  btMatrix3x3 orientation;
  orientation.setRPY(state->values[3],state->values[4],state->values[5]);
  double distance_orientation = orientation_constraint_evaluator_.distance(orientation);

  btVector3 position(state->values[0],state->values[1],state->values[2]);
  double distance_position = position_constraint_evaluator_.distance(position);
  double distance_joint = joint_constraint_evaluator_.distance(state->values[6]);

  return std::max<double>(std::max<double>(fabs(distance_joint),fabs(distance_position)),fabs(distance_orientation));
}

void IKConstrainedGoal::print(std::ostream &out) const
{
}

void IKConstrainedGoal::setup(motion_planning_msgs::Constraints &goal_constraint,                              
                              const double &redundant_joint_value)
{
  // Setup the position constraint
  position_constraint_evaluator_.setup(goal_constraint.position_constraints[0]);
  orientation_constraint_evaluator_.setup(goal_constraint.orientation_constraints[0]);
  if(!goal_constraint.joint_constraints.empty())
    joint_constraint_evaluator_.setup(goal_constraint.joint_constraints[0]);

  geometry_msgs::PoseStamped desired_pose = motion_planning_msgs::poseConstraintsToPoseStamped(goal_constraint.position_constraints[0],
                                                                                               goal_constraint.orientation_constraints[0]);
  btTransform desired_pose_tf;
  btTransform kinematics_planner_frame_tf;
  tf::poseMsgToTF(desired_pose.pose,desired_pose_tf);
  double roll, pitch, yaw;
  desired_pose_tf.getBasis().getRPY(roll,pitch,yaw);

  state->values[0] = desired_pose_tf.getOrigin().x();
  state->values[1] = desired_pose_tf.getOrigin().y();
  state->values[2] = desired_pose_tf.getOrigin().z();

  state->values[3] = roll;
  state->values[4] = pitch;
  state->values[5] = yaw;
  state->values[6] = redundant_joint_value;


  ROS_INFO("Goal State:");
  ROS_INFO("Position   : %f %f %f",state->values[0],state->values[1],state->values[2]);
  ROS_INFO("Orientation: %f %f %f %f",state->values[3],state->values[4],state->values[5],state->values[6]);

}

}


