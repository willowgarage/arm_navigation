/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *
 *  \author Sachin Chitta
 *********************************************************************/
#ifndef MOVE_GROUP_H_
#define MOVE_GROUP_H_

#include <ros/ros.h>

// TF
#include <tf/tf.h>
#include <tf/transform_listener.h>

// URDF
#include <urdf/model.h>
#include <angles/angles.h>

//actionlib
#include <actionlib/client/action_client.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>

// ROS msgs
#include <arm_navigation_msgs/FilterJointTrajectoryWithConstraints.h>
#include <arm_navigation_msgs/ArmNavigationErrorCodes.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <arm_navigation_msgs/GetMotionPlan.h>
#include <trajectory_msgs/JointTrajectory.h>

// util
#include <planning_environment/util/construct_object.h>
#include <arm_navigation_msgs/convert_messages.h>
#include <arm_navigation_msgs/utils.h>

// arm navigation
#include <arm_kinematics_constraint_aware/multi_arm_kinematics_constraint_aware.h>
#include <arm_kinematics_constraint_aware/multi_arm_kinematics_exception.h>
#include <planning_environment/models/collision_models.h>
#include <planning_environment/models/collision_models.h>
#include <planning_environment/models/model_utils.h>
#include <geometric_shapes/bodies.h>

// C++
#include <valarray>
#include <algorithm>
#include <cstdlib>

namespace move_arm
{

typedef actionlib::ActionClient<control_msgs::FollowJointTrajectoryAction> JointExecutorActionClient;

enum ControllerStatus {
  QUEUED,
  ACTIVE,
  SUCCESS,
  FAILED
};

class MoveGroup
{
public:	

  MoveGroup(const std::string &group_name, planning_environment::CollisionModels *collision_models);

  virtual ~MoveGroup();

  bool initialize();

  bool checkRequest(arm_navigation_msgs::GetMotionPlan::Request &request,
                    arm_navigation_msgs::GetMotionPlan::Response &response,
                    planning_models::KinematicState *kinematic_state,
                    bool check_start_state = true);

  bool runIKOnRequest(arm_navigation_msgs::GetMotionPlan::Request  &request,  
                      arm_navigation_msgs::GetMotionPlan::Response &response,
                      planning_models::KinematicState *kinematic_state,
                      double &ik_allowed_time);

  bool sendTrajectory(trajectory_msgs::JointTrajectory &current_trajectory);

  bool isControllerDone(arm_navigation_msgs::ArmNavigationErrorCodes& error_code);

  bool checkState(arm_navigation_msgs::Constraints &goal_constraints,
                  arm_navigation_msgs::Constraints &path_constraints,
                  const planning_models::KinematicState *kinematic_state,
                  arm_navigation_msgs::ArmNavigationErrorCodes &error_code);

  std::string getPhysicalGroupName()
  {	
    return physical_group_name_;
  }

  // Planning
  arm_navigation_msgs::Constraints original_goal_constraints_;

private:

  arm_navigation_msgs::ArmNavigationErrorCodes kinematicsErrorCodeToArmNavigationErrorCode(const int& error_code);

  bool loadGroups();

  bool findGroup(std::string &group_name, move_arm::MoveGroup& group);

  bool getConfigurationParams(std::vector<std::string> &arm_names,
                              std::vector<std::string> &kinematics_solver_names,
                              std::vector<std::string> &end_effector_link_names);

  bool createJointGoalFromPoseGoal(arm_navigation_msgs::GetMotionPlan::Request &request,
                                   arm_navigation_msgs::GetMotionPlan::Response &response,
                                   planning_models::KinematicState *kinematic_state,
                                   double &ik_allowed_time);

  bool checkStartState(arm_navigation_msgs::Constraints &goal_constraints,
                       arm_navigation_msgs::Constraints &path_constraints,
                       const planning_models::KinematicState *kinematic_state,
                       arm_navigation_msgs::ArmNavigationErrorCodes &error_code);

  bool checkGoalState(arm_navigation_msgs::GetMotionPlan::Request &request,
                      arm_navigation_msgs::GetMotionPlan::Response &response,
                      planning_models::KinematicState *kinematic_state);
  
  bool initializeControllerInterface();

  bool stopTrajectory();

  void controllerTransitionCallback(JointExecutorActionClient::GoalHandle gh);

  // ROS
  ros::NodeHandle private_handle_, root_handle_;
  planning_environment::CollisionModels* collision_models_;
  
  // Group Information
  std::string group_name_,physical_group_name_;
  std::vector<std::string> group_joint_names_;

  std::vector<std::string> arm_names_;
  std::vector<std::string> kinematics_solver_names_;
  std::vector<std::string> end_effector_link_names_;
  
  // Planning
  double default_joint_tolerance_;
  
  // Control
  ControllerStatus controller_status_;
  JointExecutorActionClient* controller_action_client_;
  JointExecutorActionClient::GoalHandle controller_goal_handle_;
  
  // Kinematics
  arm_kinematics_constraint_aware::MultiArmKinematicsConstraintAware* kinematics_solver_;
  unsigned int num_arms_;
  bool ik_solver_initialized_;
};
}

#endif
