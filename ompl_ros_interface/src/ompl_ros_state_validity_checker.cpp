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

#include <ompl_ros_interface/ompl_ros_state_validity_checker.h>

namespace ompl_ros_interface
{    
void OmplRosStateValidityChecker::configureOnRequest(planning_models::KinematicState *kinematic_state,
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
}

arm_navigation_msgs::Constraints OmplRosStateValidityChecker::getPhysicalConstraints(const arm_navigation_msgs::Constraints &constraints)
{
  arm_navigation_msgs::Constraints result_constraints;
  for(unsigned int i=0; i < constraints.joint_constraints.size(); i++)
    if(collision_models_interface_->getKinematicModel()->hasJointModel(constraints.joint_constraints[i].joint_name))
      result_constraints.joint_constraints.push_back(constraints.joint_constraints[i]);

  for(unsigned int i=0; i < constraints.position_constraints.size(); i++)
    if(collision_models_interface_->getKinematicModel()->hasLinkModel(constraints.position_constraints[i].link_name))
      result_constraints.position_constraints.push_back(constraints.position_constraints[i]);

  for(unsigned int i=0; i < constraints.orientation_constraints.size(); i++)
    if(collision_models_interface_->getKinematicModel()->hasLinkModel(constraints.orientation_constraints[i].link_name))
      result_constraints.orientation_constraints.push_back(constraints.orientation_constraints[i]);

  for(unsigned int i=0; i < constraints.visibility_constraints.size(); i++)
    if(collision_models_interface_->getKinematicModel()->hasLinkModel(constraints.visibility_constraints[i].sensor_pose.header.frame_id))
      result_constraints.visibility_constraints.push_back(constraints.visibility_constraints[i]);

  return result_constraints;
}

void OmplRosStateValidityChecker::printSettings(std::ostream &out) const
{    
  out << "ROS State Validity Checker" << std::endl;
}

}
