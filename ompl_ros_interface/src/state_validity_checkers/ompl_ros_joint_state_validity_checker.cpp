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

#include <ompl_ros_interface/state_validity_checkers/ompl_ros_joint_state_validity_checker.h>

namespace ompl_ros_interface
{    

bool OmplRosJointStateValidityChecker::isValid(const ompl::base::State *ompl_state) const
{
  //ros::WallTime n1 = ros::WallTime::now();
  ompl_ros_interface::omplStateToKinematicStateGroup(ompl_state,
                                                     ompl_state_to_kinematic_state_mapping_,
                                                     joint_state_group_);
  std::vector<planning_models::KinematicState::JointState*> joint_states = joint_state_group_->getJointStateVector();
  for(unsigned int i=0; i < joint_states.size(); i++)
  {
    if(!joint_states[i]->areJointStateValuesWithinBounds())
    {
      ROS_ERROR("State violates joint limits for Joint %s",joint_states[i]->getName().c_str());
      return false;
    }
  }

  if(!path_constraint_evaluator_set_.decide(kinematic_state_, false))
  {
    ROS_DEBUG("Path constraints violated");
    return false;
  }

  joint_state_group_->updateKinematicLinks();
  //ros::WallTime n2 = ros::WallTime::now();
  if(collision_models_interface_->isKinematicStateInCollision(*kinematic_state_))
  {
    ROS_DEBUG("State is in collision");
    //ROS_INFO_STREAM("Positive collision check took " << (ros::WallTime::now()-n2).toSec());
    return false;
  }
  //ROS_INFO_STREAM("Negative collision check took " << (ros::WallTime::now()-n2).toSec());
  return true;
}

bool OmplRosJointStateValidityChecker::isStateValid(const ompl::base::State *ompl_state) 
{
  ompl_ros_interface::omplStateToKinematicStateGroup(ompl_state,
                                                     ompl_state_to_kinematic_state_mapping_,
                                                     joint_state_group_);
  std::vector<planning_models::KinematicState::JointState*> joint_states = joint_state_group_->getJointStateVector();
  for(unsigned int i=0; i < joint_states.size(); i++)
  {
    if(!joint_states[i]->areJointStateValuesWithinBounds())
    {
      ROS_ERROR("State violates joint limits for Joint %s",joint_states[i]->getName().c_str());
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
  ROS_DEBUG("State is ok");
  return true;
}

}
