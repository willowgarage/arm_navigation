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



#ifndef MOVE_ARM_ACTION_H_
#define MOVE_ARM_ACTION_H_

#include <move_arm/move_group.h>

//actionlib
#include <actionlib/server/simple_action_server.h>

// ROS msgs
#include <arm_navigation_msgs/FilterJointTrajectoryWithConstraints.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <arm_navigation_msgs/MoveArmStatistics.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/GetRobotState.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

// arm navigation
#include <arm_navigation_msgs/planning_visualizer.h>

namespace move_arm
{

enum MoveArmState {
  PLANNING,
  START_FILTER,
  START_CONTROL,
  VISUALIZE_PLAN,
  MONITOR
};
  
static const std::string DISPLAY_PATH_TOPIC  = "display_path";
static const std::string DISPLAY_MARKER_TOPIC  = "display_marker";
static const std::string GET_STATE_SERVICE_NAME = "/environment_server/get_robot_state";
static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";
static const double MIN_TRAJECTORY_MONITORING_FREQUENCY = 1.0;
static const double MAX_TRAJECTORY_MONITORING_FREQUENCY = 100.0;
  
class MoveArm
{
public:

  MoveArm();

  virtual ~MoveArm();

  bool initialize();

private:

  bool loadGroups();

  bool findGroup(std::string &group_name, boost::shared_ptr<move_arm::MoveGroup> &group);

  bool filterTrajectory(const trajectory_msgs::JointTrajectory &trajectory_in, 
                        trajectory_msgs::JointTrajectory &trajectory_out,
                        arm_navigation_msgs::Constraints &goal_constraints,
                        arm_navigation_msgs::Constraints &path_constraints,
                        planning_models::KinematicState *kinematic_state,
                        arm_navigation_msgs::ArmNavigationErrorCodes &error_code);

  bool getRobotState(planning_models::KinematicState* state);

  bool setup(const arm_navigation_msgs::MoveArmGoalConstPtr& goal, 
             arm_navigation_msgs::GetMotionPlan::Request &req);

  void setAborted(arm_navigation_msgs::GetMotionPlan::Response &response);

  void setAborted(arm_navigation_msgs::ArmNavigationErrorCodes &error_code);

  void setAborted();

  bool visualizeCollisions(const planning_models::KinematicState *kinematic_state);

  bool createPlan(arm_navigation_msgs::GetMotionPlan::Request &request,  
                  arm_navigation_msgs::GetMotionPlan::Response &response);

  void fillTrajectoryMsg(const trajectory_msgs::JointTrajectory &trajectory_in, 
                         trajectory_msgs::JointTrajectory &trajectory_out);

  void resetStateMachine();

  bool jointTrajectoryValid(trajectory_msgs::JointTrajectory &joint_trajectory,
                            arm_navigation_msgs::Constraints &goal_constraints,
                            arm_navigation_msgs::Constraints &path_constraints,
                            planning_models::KinematicState *kinematic_state,
                            arm_navigation_msgs::ArmNavigationErrorCodes &error_code);
  
  bool executeCycle(arm_navigation_msgs::GetMotionPlan::Request &request);

  void resetStats();

  void resetActionResult();

  void execute(const arm_navigation_msgs::MoveArmGoalConstPtr& goal);

  bool getAndSetPlanningScene(const arm_navigation_msgs::PlanningScene& planning_diff,
                              const arm_navigation_msgs::OrderedCollisionOperations& operations);

  void resetToStartState(planning_models::KinematicState* state);

  bool revertPlanningScene();

  void publishStats();

  // Group information
  std::map<std::string,boost::shared_ptr<move_arm::MoveGroup> > group_map_;
  boost::shared_ptr<move_arm::MoveGroup> current_group_;

  // ROS
  ros::NodeHandle private_handle_, root_handle_;

  // Action
  boost::shared_ptr<actionlib::SimpleActionServer<arm_navigation_msgs::MoveArmAction> > action_server_;	
  arm_navigation_msgs::MoveArmResult move_arm_action_result_;
  arm_navigation_msgs::MoveArmFeedback move_arm_action_feedback_;

  // Planning scene
  planning_environment::CollisionModels* collision_models_;
  arm_navigation_msgs::PlanningScene current_planning_scene_;
  planning_models::KinematicState* planning_scene_state_;
  arm_navigation_msgs::Constraints original_goal_constraints_;

  // Services, Publishers
  ros::Publisher stats_publisher_;

  ros::ServiceClient get_state_client_;
  ros::ServiceClient set_planning_scene_diff_client_;

  arm_navigation_msgs::SetPlanningSceneDiff::Request set_planning_scene_diff_req_;
  arm_navigation_msgs::SetPlanningSceneDiff::Response set_planning_scene_diff_res_;

  // MoveArm state machine
  MoveArmState state_;
  double move_arm_frequency_;      	

  // Results
  trajectory_msgs::JointTrajectory current_trajectory_;

  // Parameters
  bool publish_stats_;
  int num_planning_attempts_;
  double trajectory_discretization_;
  arm_navigation_msgs::MoveArmStatistics move_arm_stats_;
  double trajectory_filter_allowed_time_, ik_allowed_time_;
  double disable_ik_, allowed_planning_time_;
  std::string planner_service_name_;

  //Visualization
  arm_navigation_msgs::PlanningVisualizer planning_visualizer_;
};
}

#endif
