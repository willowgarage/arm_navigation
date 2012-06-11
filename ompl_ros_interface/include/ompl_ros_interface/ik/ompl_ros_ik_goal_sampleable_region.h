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

/** \author Sachin Chitta, Ioan Sucan */

#ifndef OMPL_ROS_IK_GOAL_SAMPLEABLE_H_
#define OMPL_ROS_IK_GOAL_SAMPLEABLE_H_

// OMPL
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/State.h>

// Planning environment and models
#include <planning_environment/models/collision_models_interface.h>
#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>

// OMPL ROS Interface
#include <ompl_ros_interface/ompl_ros_state_validity_checker.h>
#include <ompl_ros_interface/ompl_ros_projection_evaluator.h>
#include <ompl_ros_interface/ompl_ros_planner_config.h>

#include <ompl_ros_interface/helpers/ompl_ros_conversions.h>

// Kinematics
#include <pluginlib/class_loader.h>
#include <kinematics_base/kinematics_base.h>

namespace ompl_ros_interface
{
/**
 * @class OmplRosIKSampleableRegion 
 * @brief Inherits from ompl::base::GoalSampleableRegion and can be used to sample goals using IK.
 */
class OmplRosIKSampleableRegion : public ompl::base::GoalSampleableRegion
{
public:
  /**
   * @brief Inherits from ompl::base::GoalSampleableRegion and can be used to sample goals using IK.
   * @param space_information An instance of the space information to use for initialization
   */
  OmplRosIKSampleableRegion(const ompl::base::SpaceInformationPtr &space_information)
    : GoalSampleableRegion(space_information), 
      kinematics_loader_("kinematics_base","kinematics::KinematicsBase"),
      scoped_state_(space_information)  
  {
  }

  /**
   * @brief Initialize the goal sampleable region
   * @param state_space - The state space to use for initialization of the goal sampleable region
   * @param kinematics_solver_name - The name of the kinematics solver to be used to populate the goal sampleable region
   * @param group_name - The name of the group 
   * @param end_effector_name - The name of the end effector for this particular group
   * @param planning_monitor - A pointer to the planning monitor
   */
  bool initialize(const ompl::base::StateSpacePtr &state_space,
                  const std::string &kinematics_solver_name,
                  const std::string &group_name,
                  const std::string &end_effector_name,
                  const planning_environment::CollisionModelsInterface* cmi);

  /**
   * @brief Configure the GoalSampleableRegion when a request is received. 
   * This is typically a one time configuration for each planning request.
   * @param request The request that the planner gets
   * @param response The response to the planning request
   */ 
  bool configureOnRequest(const arm_navigation_msgs::GetMotionPlan::Request &request,
                          arm_navigation_msgs::GetMotionPlan::Response &response,
                          const unsigned int &max_sample_count = 100);

  /**
   * @brief Sample a goal in the goal sampleable region
   * @param state - The state to be sampled and filled in
   */
  virtual void sampleGoal(ompl::base::State *state) const;

  /**
   * @brief Get the maximum sample count
   * @return maximum sample count
   */
  virtual unsigned int maxSampleCount(void) const;

private:
  void sampleGoals(const unsigned int &number_goals,
                   std::vector<arm_navigation_msgs::RobotState> &sampled_states_vector) const;

  std::vector<geometry_msgs::PoseStamped> ik_poses_;
  unsigned int max_sample_count_;
  unsigned int ik_poses_counter_;
  ompl::base::StateSpacePtr state_space_;
  kinematics::KinematicsBase* kinematics_solver_;
  
  std::string kinematics_solver_name_, group_name_, end_effector_name_;
  pluginlib::ClassLoader<kinematics::KinematicsBase> kinematics_loader_;
  ompl::base::ScopedState<ompl::base::CompoundStateSpace> scoped_state_;
  arm_navigation_msgs::RobotState seed_state_, solution_state_;

  ompl_ros_interface::OmplStateToRobotStateMapping ompl_state_to_robot_state_mapping_;
  ompl_ros_interface::RobotStateToOmplStateMapping robot_state_to_ompl_state_mapping_;
  const planning_environment::CollisionModelsInterface* collision_models_interface_;

};

}

#endif //OMPL_ROS_IK_GOAL_SAMPLEABLE_H_
