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
#include <sstream>

#include <move_arm/move_arm_warehouse_reader.h>

using namespace move_arm;

typedef warehouse::MessageWithMetadata<planning_environment_msgs::PlanningScene>::ConstPtr PlanningSceneWithMetadata;
typedef warehouse::MessageWithMetadata<motion_planning_msgs::MotionPlanRequest>::ConstPtr MotionPlanRequestWithMetadata;
typedef warehouse::MessageWithMetadata<trajectory_msgs::JointTrajectory>::ConstPtr JointTrajectoryWithMetadata;
typedef warehouse::MessageWithMetadata<motion_planning_msgs::ArmNavigationErrorCodes>::ConstPtr ErrorCodesWithMetadata;
typedef warehouse::MessageWithMetadata<move_arm_msgs::HeadMonitorFeedback>::ConstPtr HeadMonitorFeedbackWithMetadata;
  
MoveArmWarehouseReader::MoveArmWarehouseReader() :
  warehouse_client_("move_arm_warehouse_logger")
{
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

void MoveArmWarehouseReader::getAvailablePlanningSceneList(const std::string& hostname, std::vector<ros::Time>& creation_times)
{
  creation_times.clear();
  std::vector<warehouse::Condition> cond;

  warehouse::OrderingCriterion order;
  order.field = "_creation_time";
  order.reverse = false;

  // std::stringstream fin(planning_scenes[i]->metadata);
  // YAML::Parser parser(fin);
  // YAML::Node doc;
  // while(parser.GetNextDocument(doc)) {    }

  std::vector<PlanningSceneWithMetadata> planning_scenes = planning_scene_collection_.pullAllResults(cond, true, order);

  creation_times.resize(planning_scenes.size());

  for(unsigned int i = 0; i < planning_scenes.size(); i++) {
    std::stringstream fin(planning_scenes[i]->metadata);
    YAML::Parser parser(fin);
    YAML::Node doc;
    while(parser.GetNextDocument(doc)) { 
      std::string s;
      doc["robot_state___joint_state___header___stamp"] >> s;
      std::stringstream ss(s);
      double t;
      ss >> t;
      creation_times[i] = ros::Time(t);
    }
  }
}

std::vector<warehouse::Condition> MoveArmWarehouseReader::makeConditionForPlanningSceneTime(const ros::Time& time)
{
  std::vector<warehouse::Condition> cond(1);
  cond[0].field_name = "planning_scene_time";
  cond[0].predicate = warehouse::Condition::EQUALS;
  std::stringstream ss;
  ss << time;  
  cond[0].args.push_back(ss.str());  
  return cond;
}

bool MoveArmWarehouseReader::getPlanningScene(const std::string& hostname, const ros::Time& time, 
                                              planning_environment_msgs::PlanningScene& planning_scene)
{
  std::vector<warehouse::Condition> cond(1);
  cond[0].field_name = "robot_state.joint_state.header.stamp";
  cond[0].predicate = warehouse::Condition::EQUALS;
  std::stringstream ss;
  ss << time;
  cond[0].args.push_back(ss.str());  

  std::vector<PlanningSceneWithMetadata> planning_scenes = planning_scene_collection_.pullAllResults(cond, false);

  if(planning_scenes.size() == 0) {
    ROS_WARN_STREAM("No scenes with that time");
    return false;
  } else if(planning_scenes.size() > 1) {
    ROS_WARN_STREAM("More than one stream with that time " << planning_scenes.size());
  }
  planning_scene = *planning_scenes[0];
  return true;
}

bool MoveArmWarehouseReader::getAssociatedOutcomes(const std::string& hostname,
                                                   const ros::Time& time,
                                                   std::vector<std::string>& pipeline_names,
                                                   std::vector<motion_planning_msgs::ArmNavigationErrorCodes>& error_codes)
{

  std::vector<warehouse::Condition> cond = makeConditionForPlanningSceneTime(time);
  std::vector<ErrorCodesWithMetadata> meta_error_codes = outcome_collection_.pullAllResults(cond, false);

  if(meta_error_codes.size() == 0) {
    ROS_WARN_STREAM("No outcomes associated with time " << time);
    return false;
  } 
  error_codes.resize(meta_error_codes.size());
  pipeline_names.resize(meta_error_codes.size());

  for(unsigned int i = 0; i < meta_error_codes.size(); i++) {
    std::stringstream fin(meta_error_codes[i]->metadata);
    YAML::Parser parser(fin);
    YAML::Node doc;
    while(parser.GetNextDocument(doc)) { 
      doc["pipeline_stage"] >> pipeline_names[i];
    }
    error_codes[i] = *meta_error_codes[i];
  }
  return true;
}

                                                 
bool MoveArmWarehouseReader::getAssociatedMotionPlanRequestsStageNames(const std::string& hostname, 
                                                                       const ros::Time& time,
                                                                       std::vector<std::string>& stage_names)
{
  warehouse::OrderingCriterion order;
  order.field = "_creation_time";
  order.reverse = false;

  stage_names.clear();
  std::vector<warehouse::Condition> cond = makeConditionForPlanningSceneTime(time);
  
  std::vector<MotionPlanRequestWithMetadata> motion_plan_requests = motion_plan_request_collection_.pullAllResults(cond, true, order);

  if(motion_plan_requests.size() == 0) {
    ROS_WARN_STREAM("No motion plan requests with that time");
    return false;
  } 
  for(unsigned int i = 0; i < motion_plan_requests.size(); i++) {
    std::stringstream fin(motion_plan_requests[i]->metadata);
    YAML::Parser parser(fin);
    YAML::Node doc;
    while(parser.GetNextDocument(doc)) {    
      stage_names.push_back(doc["stage_name"]);
    }
  }
  return true; 
}

bool MoveArmWarehouseReader::getAssociatedMotionPlanRequest(const std::string& hostname, 
                                                            const ros::Time& time,
                                                            const std::string& stage_name,
                                                            motion_planning_msgs::MotionPlanRequest& request)
{  
  std::vector<warehouse::Condition> cond = makeConditionForPlanningSceneTime(time);
  
  warehouse::Condition stage_cond;
  stage_cond.field_name = "stage_name";
  stage_cond.predicate = warehouse::Condition::EQUALS;
  stage_cond.args.push_back(stage_name);  
  
  cond.push_back(stage_cond);

  std::vector<MotionPlanRequestWithMetadata> motion_plan_requests = motion_plan_request_collection_.pullAllResults(cond, false);

  if(motion_plan_requests.size() == 0) {
    ROS_WARN_STREAM("No motion plan requests with that time and stage name " << stage_name);
    return false;
  } else if(motion_plan_requests.size() > 1) {
    ROS_WARN_STREAM("More than one motion plan requests with that time and stage name " << stage_name);
    return false;
  }
  request = *motion_plan_requests[0];
  return true;
}

bool MoveArmWarehouseReader::getAssociatedJointTrajectorySources(const std::string& hostname, 
                                                                 const ros::Time& time,
                                                                 std::vector<std::string>& trajectory_sources)
{
  trajectory_sources.clear();
  std::vector<warehouse::Condition> cond = makeConditionForPlanningSceneTime(time);
  
  std::vector<JointTrajectoryWithMetadata> joint_trajectories = trajectory_collection_.pullAllResults(cond, true);

  if(joint_trajectories.size() == 0) {
    ROS_WARN_STREAM("No joint trajectories with that time");
    return false;
  } 
  for(unsigned int i = 0; i < joint_trajectories.size(); i++) {
    std::stringstream fin(joint_trajectories[i]->metadata);
    YAML::Parser parser(fin);
    YAML::Node doc;
    while(parser.GetNextDocument(doc)) {    
      trajectory_sources.push_back(doc["trajectory_source"]);
    }
  }
  return true; 
}

bool MoveArmWarehouseReader::getAssociatedJointTrajectory(const std::string& hostname, 
                                                          const ros::Time& time,
                                                          const std::string& trajectory_source,
                                                          const unsigned int& trajectory_index,
                                                          ros::Duration& duration, 
                                                          trajectory_msgs::JointTrajectory& joint_trajectory)
{
  std::vector<warehouse::Condition> cond = makeConditionForPlanningSceneTime(time);
  
  warehouse::Condition source_cond;
  source_cond.field_name = "trajectory_source";
  source_cond.predicate = warehouse::Condition::EQUALS;
  source_cond.args.push_back(trajectory_source);  
  
  cond.push_back(source_cond);

  std::vector<JointTrajectoryWithMetadata> joint_trajectories = trajectory_collection_.pullAllResults(cond, false);

  if(joint_trajectories.size() == 0) {
    ROS_WARN_STREAM("No joint trajectories with that time and source name " << trajectory_source);
    return false;
  } else if(joint_trajectories.size() <= trajectory_index) {
    ROS_WARN_STREAM("Not enough trajectories for that index: " << trajectory_index);
    return false;
  }

  std::stringstream fin(joint_trajectories[trajectory_index]->metadata);
  YAML::Parser parser(fin);
  YAML::Node doc;
  while(parser.GetNextDocument(doc)) {
    float t;
    t = doc["production_time"];
    ros::Duration d(t);
    duration = d;
  }
  joint_trajectory = *joint_trajectories[trajectory_index];
  return true;
}

bool MoveArmWarehouseReader::getAssociatedPausedStates(const std::string& hostname, 
                                                       const ros::Time& time,
                                                       std::vector<ros::Time>& paused_times)
{
  paused_times.clear();
  std::vector<warehouse::Condition> cond = makeConditionForPlanningSceneTime(time);
  
  std::vector<HeadMonitorFeedbackWithMetadata> paused_states = paused_state_collection_.pullAllResults(cond, true);

  if(paused_states.size() == 0) {
    return false;
  } 
  paused_times.resize(paused_states.size());
  for(unsigned int i = 0; i < paused_states.size(); i++) {
    std::stringstream fin(paused_states[i]->metadata);
    YAML::Parser parser(fin);
    YAML::Node doc;
    while(parser.GetNextDocument(doc)) { 
      std::string s;
      doc["paused_collision_map___header___stamp"] >> s;
      std::stringstream ss(s);
      double t;
      ss >> t;
      paused_times[i] = ros::Time(t);
    }
  }
  return true;   
}

bool MoveArmWarehouseReader::getAssociatedPausedState(const std::string& hostname, 
                                                      const ros::Time& planning_time, 
                                                      const ros::Time& paused_time,
                                                      move_arm_msgs::HeadMonitorFeedback& paused_state)
{
  std::vector<warehouse::Condition> cond = makeConditionForPlanningSceneTime(planning_time);

  warehouse::Condition time_cond;
  time_cond.field_name = "paused_collision_map.header.stamp";
  time_cond.predicate = warehouse::Condition::EQUALS;
  std::stringstream ss;
  ss << paused_time;  
  time_cond.args.push_back(ss.str());  

  cond.push_back(time_cond);

  std::vector<HeadMonitorFeedbackWithMetadata> paused_states = paused_state_collection_.pullAllResults(cond, false);

  if(paused_states.size() == 0) {
    ROS_WARN_STREAM("No paused states with that time");
    return false;
  } else if(paused_states.size() > 1) {
    ROS_WARN_STREAM("Multiple paused states with time");
    return false;
  }
  paused_state = *paused_states[0];
  return true;
}
