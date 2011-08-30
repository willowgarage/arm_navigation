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

/** \author Sachin Chitta */

#include <ompl_ros_interface/state_validity_checkers/ompl_ros_task_space_validity_checker.h>

namespace ompl_ros_interface
{

bool OmplRosTaskSpaceValidityChecker::isValid(const ompl::base::State *ompl_state) const
{
  arm_navigation_msgs::RobotState robot_state_msg;
  if(!state_transformer_->inverseTransform(*ompl_state,
                                           robot_state_msg))
    return false;

  ompl_ros_interface::robotStateToJointStateGroup(robot_state_msg,
                                                  robot_state_to_joint_state_group_mapping_,
                                                  joint_state_group_);
  std::vector<planning_models::KinematicState::JointState*> joint_states = joint_state_group_->getJointStateVector();
  for(unsigned int i=0; i < joint_states.size(); i++)
  {
    if(!joint_states[i]->areJointStateValuesWithinBounds())
    {
      ROS_DEBUG("State violates joint limits for Joint %s",joint_states[i]->getName().c_str());
      return false;
    }
  }
  joint_state_group_->updateKinematicLinks();
  if(!path_constraint_evaluator_set_.decide(kinematic_state_, false))
  {
    ROS_DEBUG("Path constraints violated in task space");
    return false;
  }
  if(collision_models_interface_->isKinematicStateInCollision(*kinematic_state_))
  {
    ROS_DEBUG("State is in collision");
    return false;
  }
  return true;
}

bool OmplRosTaskSpaceValidityChecker::isStateValid(const ompl::base::State *ompl_state) 
{
  arm_navigation_msgs::RobotState robot_state_msg;
  if(!state_transformer_->inverseTransform(*ompl_state,
                                           robot_state_msg))
  {
    ROS_DEBUG("State transformation failed");
    error_code_.val = error_code_.NO_IK_SOLUTION;
    return false;
  }
  ompl_ros_interface::robotStateToJointStateGroup(robot_state_msg,
                                                  robot_state_to_joint_state_group_mapping_,
                                                  joint_state_group_);
  std::vector<planning_models::KinematicState::JointState*> joint_states = joint_state_group_->getJointStateVector();
  for(unsigned int i=0; i < joint_states.size(); i++)
  {
    if(!joint_states[i]->areJointStateValuesWithinBounds())
    {
      ROS_DEBUG("State violates joint limits for Joint %s",joint_states[i]->getName().c_str());
      error_code_.val = error_code_.JOINT_LIMITS_VIOLATED;
      return false;
    }
  }

  if(!path_constraint_evaluator_set_.decide(kinematic_state_, false))
  {
    ROS_DEBUG("Path constraints violated");
    error_code_.val = error_code_.PATH_CONSTRAINTS_VIOLATED;
    return false;
  }

  joint_state_group_->updateKinematicLinks();
  if(collision_models_interface_->isKinematicStateInCollision(*kinematic_state_))
  {
    ROS_DEBUG("State is in collision");
    error_code_.val = error_code_.COLLISION_CONSTRAINTS_VIOLATED;        
    return false;
  }
  return true;
}

bool OmplRosTaskSpaceValidityChecker::initialize()
{
  return true;
}

bool OmplRosTaskSpaceValidityChecker::setStateTransformer(boost::shared_ptr<ompl_ros_interface::OmplRosStateTransformer> &state_transformer)
{
  if(state_transformer)
  {
    if(state_transformer->getFrame() != parent_frame_)
    {
      ROS_ERROR("State transformer has parent frame %s. State transformer should function in same frame as planning state space %s",state_transformer->getFrame().c_str(),parent_frame_.c_str());
      return false;
    }
    state_transformer_ = state_transformer;
  }
  else 
    return false;
  return true;
}

void OmplRosTaskSpaceValidityChecker::configureOnRequest(planning_models::KinematicState *kinematic_state,
                                                         planning_models::KinematicState::JointStateGroup *joint_state_group,
                                                         const arm_navigation_msgs::GetMotionPlan::Request &request)
{
  kinematic_state_ = kinematic_state;
  joint_state_group_ = joint_state_group;

  goal_constraint_evaluator_set_.clear();
  path_constraint_evaluator_set_.clear();

  //Get the valid set of constraints that correspond to constraints on the physical joints and links of the robot
  arm_navigation_msgs::Constraints goal_constraints = getPhysicalConstraints(request.motion_plan_request.goal_constraints);
  arm_navigation_msgs::Constraints path_constraints = getPhysicalConstraints(request.motion_plan_request.path_constraints);

  goal_constraint_evaluator_set_.add(goal_constraints.joint_constraints);
  goal_constraint_evaluator_set_.add(goal_constraints.position_constraints);
  goal_constraint_evaluator_set_.add(goal_constraints.orientation_constraints);
  goal_constraint_evaluator_set_.add(goal_constraints.visibility_constraints);

  path_constraint_evaluator_set_.add(path_constraints.joint_constraints);
  path_constraint_evaluator_set_.add(path_constraints.position_constraints);
  path_constraint_evaluator_set_.add(path_constraints.orientation_constraints);
  path_constraint_evaluator_set_.add(path_constraints.visibility_constraints);

  arm_navigation_msgs::RobotState default_state = state_transformer_->getDefaultState();
  if(!getRobotStateToJointModelGroupMapping(default_state,joint_state_group_->getJointModelGroup(),robot_state_to_joint_state_group_mapping_))
    return;
}

}
