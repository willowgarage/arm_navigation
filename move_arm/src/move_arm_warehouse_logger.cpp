/*********************************************************************
 *
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
 *
 *  \author E. Gil Jones
 *********************************************************************/

#include <ros/console.h>
#include <boost/foreach.hpp>

#include <move_arm/move_arm_warehouse_logger.h>
#include <unistd.h>

using namespace move_arm;

MoveArmWarehouseLogger::MoveArmWarehouseLogger() :
  warehouse_client_("move_arm_warehouse_logger")
{
  char hostname[256];
  
  gethostname(hostname, 256);

  hostname_ = hostname;

  ROS_INFO_STREAM("Hostname is " << hostname_);

  std::vector<std::string> indexed_fields;
  indexed_fields.push_back("robot_state.joint_state.header.stamp");
  planning_scene_collection_ = warehouse_client_.setupCollection<planning_environment_msgs::PlanningScene>("planning_scene", indexed_fields);

  indexed_fields.clear();
  indexed_fields.push_back("group_name");
  motion_plan_request_collection_ = warehouse_client_.setupCollection<motion_planning_msgs::MotionPlanRequest>("motion_plan_request", indexed_fields);

  indexed_fields.clear();
  trajectory_collection_ = warehouse_client_.setupCollection<trajectory_msgs::JointTrajectory>("trajectory", indexed_fields);

  indexed_fields.clear();
  indexed_fields.push_back("val");
  outcome_collection_ = warehouse_client_.setupCollection<motion_planning_msgs::ArmNavigationErrorCodes>("outcome", indexed_fields);

  indexed_fields.clear();
  indexed_fields.push_back("paused_collision_map.header.stamp");
  paused_state_collection_ = warehouse_client_.setupCollection<move_arm_msgs::HeadMonitorFeedback>("paused_state", indexed_fields);
}

std::string MoveArmWarehouseLogger::makeMetaStringFromHostname()
{
  std::string s;
  addToMetadataString(s, "hostname", hostname_);
  return s;
}

void MoveArmWarehouseLogger::addPlanningSceneTimeToMetadata(const planning_environment_msgs::PlanningScene& planning_scene,
                                                            std::string& metadata)
{
  std::stringstream ss;
  ss << planning_scene.robot_state.joint_state.header.stamp;

  addToMetadataString("planning_scene_time", ss.str(), metadata);
}

void MoveArmWarehouseLogger::pushPlanningSceneToWarehouse(const planning_environment_msgs::PlanningScene planning_scene)
{
  std::string metadata_string = makeMetaStringFromHostname();
  planning_scene_collection_.publish(planning_scene, metadata_string);
}

void MoveArmWarehouseLogger::pushMotionPlanRequestToWarehouse(const planning_environment_msgs::PlanningScene& planning_scene,
                                                              const std::string& stage_name,
                                                              const motion_planning_msgs::MotionPlanRequest& motion_plan_request)
{
  std::string metadata_string = makeMetaStringFromHostname();
  addPlanningSceneTimeToMetadata(planning_scene, metadata_string);

  addToMetadataString("stage_name", stage_name, metadata_string);
  
  //adding the presence of goal pose constraints to metadata
  std::stringstream ss;
  ss << !motion_plan_request.goal_constraints.position_constraints.empty();
  addToMetadataString("has_goal_position_constraints", ss.str(), metadata_string);

  ss.str("");
  ss << (!motion_plan_request.path_constraints.orientation_constraints.empty() || motion_plan_request.path_constraints.position_constraints.empty());
  addToMetadataString("has_path_constraints", ss.str(), metadata_string);
  
  motion_plan_request_collection_.publish(motion_plan_request, metadata_string);
}

void MoveArmWarehouseLogger::pushJointTrajectoryToWarehouse(const planning_environment_msgs::PlanningScene& planning_scene,
                                                            const std::string& trajectory_source,
                                                            const ros::Duration& production_time,
                                                            const trajectory_msgs::JointTrajectory& trajectory)
{
  std::string metadata_string = makeMetaStringFromHostname();
  addPlanningSceneTimeToMetadata(planning_scene, metadata_string);
  
  addToMetadataString("trajectory_source", trajectory_source, metadata_string);

  std::stringstream ss;
  ss << production_time;

  addToMetadataString("production_time", ss.str(), metadata_string);
 
  trajectory_collection_.publish(trajectory, metadata_string);
}

void MoveArmWarehouseLogger::pushOutcomeToWarehouse(const planning_environment_msgs::PlanningScene& planning_scene,
                                                    const std::string& pipeline_stage,
                                                    const motion_planning_msgs::ArmNavigationErrorCodes& error_codes)
{
  std::string metadata_string = makeMetaStringFromHostname();
  addPlanningSceneTimeToMetadata(planning_scene, metadata_string);
  
  addToMetadataString("pipeline_stage", pipeline_stage, metadata_string);

  ROS_INFO_STREAM("Trying to push error code " << pipeline_stage << " val " << error_codes.val);

  outcome_collection_.publish(error_codes, metadata_string);
}

void MoveArmWarehouseLogger::pushPausedStateToWarehouse(const planning_environment_msgs::PlanningScene& planning_scene,
                                                        const move_arm_msgs::HeadMonitorFeedback& feedback)
{
  std::string metadata_string = makeMetaStringFromHostname();
  addPlanningSceneTimeToMetadata(planning_scene, metadata_string);
  
  paused_state_collection_.publish(feedback, metadata_string);

}
                                                       
