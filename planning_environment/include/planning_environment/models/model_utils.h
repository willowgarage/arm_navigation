/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2010, Willow Garage, Inc.
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

/** \author E. Gil Jones */

#ifndef _MODEL_UTILS_H_
#define _MODEL_UTILS_H_

#include <tf/tf.h>
#include <planning_models/kinematic_state.h>
#include <motion_planning_msgs/RobotState.h>
#include <motion_planning_msgs/Constraints.h>
#include <motion_planning_msgs/OrderedCollisionOperations.h>

#include <planning_environment/util/kinematic_state_constraint_evaluator.h>

namespace planning_environment {

//returns true if the joint_state_map sets all the joints in the state, 
inline bool setRobotStateAndComputeTransforms(const motion_planning_msgs::RobotState &robot_state,
                                       planning_models::KinematicState& state)
{
  std::map<std::string, double> joint_state_map;
  for (unsigned int i = 0 ; i < robot_state.joint_state.name.size(); ++i)
  {
    joint_state_map[robot_state.joint_state.name[i]] = robot_state.joint_state.position[i];
  }

  //first we are going to apply transforms
  for(unsigned int i = 0; i < robot_state.multi_dof_joint_state.joint_names.size(); i++) {
    std::string joint_name = robot_state.multi_dof_joint_state.joint_names[i];
    if(!state.hasJointState(joint_name)) {
      ROS_WARN_STREAM("No joint matching multi-dof joint " << joint_name);
      continue;
    }
    planning_models::KinematicState::JointState* joint_state = state.getJointState(joint_name);
    if(robot_state.multi_dof_joint_state.frame_ids[i] != joint_state->getParentFrameId() ||
       robot_state.multi_dof_joint_state.child_frame_ids[i] != joint_state->getChildFrameId()) {
      ROS_WARN_STREAM("Robot state msg has bad multi_dof transform");
    } else {
      tf::StampedTransform transf;
      tf::poseMsgToTF(robot_state.multi_dof_joint_state.poses[i], transf);
      joint_state->setJointStateValues(transf);
    }
  }
  return(state.setKinematicState(joint_state_map));
}

inline void convertKinematicStateToRobotState(const planning_models::KinematicState& kinematic_state,
                                       const ros::Time& timestamp,
                                       const std::string& header_frame,
                                       motion_planning_msgs::RobotState &robot_state)
{
  robot_state.joint_state.position.clear();
  robot_state.joint_state.name.clear();

  const std::vector<planning_models::KinematicState::JointState*>& joints = kinematic_state.getJointStateVector();
  for(std::vector<planning_models::KinematicState::JointState*>::const_iterator it = joints.begin();
      it != joints.end();
      it++) {
    const std::vector<double>& joint_state_values = (*it)->getJointStateValues();
    const std::vector<std::string>& joint_state_value_names = (*it)->getJointStateNameOrder();
    for(unsigned int i = 0; i < joint_state_values.size(); i++) {
      robot_state.joint_state.name.push_back(joint_state_value_names[i]);
      robot_state.joint_state.position.push_back(joint_state_values[i]);
    }
    if(!(*it)->getParentFrameId().empty() && !(*it)->getChildFrameId().empty()) {
      robot_state.multi_dof_joint_state.stamp = ros::Time::now();
      robot_state.multi_dof_joint_state.joint_names.push_back((*it)->getName());
      robot_state.multi_dof_joint_state.frame_ids.push_back((*it)->getParentFrameId());
      robot_state.multi_dof_joint_state.child_frame_ids.push_back((*it)->getChildFrameId());
      tf::Pose p((*it)->getVariableTransform());
      geometry_msgs::Pose pose;
      tf::poseTFToMsg(p, pose);
      robot_state.multi_dof_joint_state.poses.push_back(pose);
    }
  }
  robot_state.joint_state.header.stamp = timestamp;
  robot_state.joint_state.header.frame_id = header_frame;
  return;
}

inline void applyOrderedCollisionOperationsToMatrix(const motion_planning_msgs::OrderedCollisionOperations &ord,
                                             std::vector<std::vector<bool> > &curAllowed,
                                             std::map<std::string, unsigned int> &vecIndices) {
  size_t all_size = curAllowed.size();

  for(size_t i = 0; i < ord.collision_operations.size(); i++) {
    
    bool allowed = (ord.collision_operations[i].operation == motion_planning_msgs::CollisionOperation::DISABLE);
    
    if(ord.collision_operations[i].object1 == ord.collision_operations[i].COLLISION_SET_ALL &&
       ord.collision_operations[i].object2 == ord.collision_operations[i].COLLISION_SET_ALL) {
      //first case - both all
      for(size_t j = 0; j < all_size; j++) {
        for(size_t k = 0; k < all_size; k++) {
          curAllowed[j][k] = allowed;
        }
      }
    } else if(ord.collision_operations[i].object1 == ord.collision_operations[i].COLLISION_SET_ALL ||
              ord.collision_operations[i].object2 == ord.collision_operations[i].COLLISION_SET_ALL) {
      //second case - only one all
      std::string other;
      if(ord.collision_operations[i].object1 == ord.collision_operations[i].COLLISION_SET_ALL) {
        other = ord.collision_operations[i].object2;
      } else {
        other = ord.collision_operations[i].object1;
      }
      if(vecIndices.find(other) == vecIndices.end()) {
        continue;
      }
      unsigned int obj_ind = vecIndices.find(other)->second;
      for(size_t j = 0; j < all_size; j++) {
        curAllowed[obj_ind][j] = allowed;
        curAllowed[j][obj_ind] = allowed;
      }
    } else {
      //third case - must be both objects
      if(vecIndices.find(ord.collision_operations[i].object1) == vecIndices.end() ||
         vecIndices.find(ord.collision_operations[i].object2) == vecIndices.end()) {
        continue;
      }
      unsigned int obj1ind = vecIndices.find(ord.collision_operations[i].object1)->second;
      unsigned int obj2ind = vecIndices.find(ord.collision_operations[i].object2)->second;
      curAllowed[obj1ind][obj2ind] = allowed;
      curAllowed[obj2ind][obj1ind] = allowed;
    }
  }
}

inline void printAllowedCollisionMatrix(const std::vector<std::vector<bool> > &curAllowed,
                                 const std::map<std::string, unsigned int> &vecIndices) {
  size_t all_size = curAllowed.size();
  for(unsigned int i = 0; i < vecIndices.size(); i++) {
    std::string n;
    for(std::map<std::string, unsigned int>::const_iterator it = vecIndices.begin();
        it != vecIndices.end();
        it++) {
      if(it->second == i) {
        n = it->first; 
      }
    }
    if(n.empty()) {
      ROS_WARN_STREAM("Can't find index " << i << " in vecIndex");
      return;
    }
    std::cout << std::setw(40) << n;
    std::cout << " | ";
    for(size_t j = 0; j < all_size; j++) {
      std::cout << std::setw(3) << curAllowed[i][j];
    }
    std::cout << std::endl;
  }
}

inline bool doesKinematicStateObeyConstraints(const planning_models::KinematicState& state,
                                       const motion_planning_msgs::Constraints& constraints,
                                       bool verbose = false) {
  planning_environment::KinematicConstraintEvaluatorSet constraint_evaluator;
  
  constraint_evaluator.add(constraints.joint_constraints);
  constraint_evaluator.add(constraints.position_constraints);
  constraint_evaluator.add(constraints.orientation_constraints);
  constraint_evaluator.add(constraints.visibility_constraints);
  return(constraint_evaluator.decide(&state, verbose));
}

}
#endif
