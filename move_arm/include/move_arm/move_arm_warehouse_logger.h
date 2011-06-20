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

#ifndef MOVE_ARM_WAREHOUSE_LOGGER_H_
#define MOVE_ARM_WAREHOUSE_LOGGER_H_

#include <warehouse/warehouse_client.h>
#include <yaml-cpp/yaml.h>


#include <planning_environment_msgs/PlanningScene.h>
#include <motion_planning_msgs/MotionPlanRequest.h>
#include <motion_planning_msgs/ArmNavigationErrorCodes.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <move_arm_msgs/HeadMonitorFeedback.h>

namespace move_arm
{

inline void addToMetadataString(const std::string& key, const std::string& val, std::string& metadata)
{
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << key;
  out << YAML::Value << val;
  out << YAML::EndMap;
  std::string outs = std::string(out.c_str());
  outs.erase(0, 3);
  metadata.append(outs);
}

class MoveArmWarehouseLogger {
  
public:

  MoveArmWarehouseLogger();

  void pushPlanningSceneToWarehouse(const planning_environment_msgs::PlanningScene planning_scene);

  void pushMotionPlanRequestToWarehouse(const planning_environment_msgs::PlanningScene& planning_scene,
                                        const std::string& stage_name,
                                        const motion_planning_msgs::MotionPlanRequest& motion_plan_request);
    
  void pushJointTrajectoryToWarehouse(const planning_environment_msgs::PlanningScene& planning_scene,
                                      const std::string& trajectory_source,
                                      const ros::Duration& production_time, 
                                      const trajectory_msgs::JointTrajectory& trajectory);
    
  void pushOutcomeToWarehouse(const planning_environment_msgs::PlanningScene& planning_scene,
                              const std::string& pipeline_stage,
                              const motion_planning_msgs::ArmNavigationErrorCodes& error_codes);
  
  void pushPausedStateToWarehouse(const planning_environment_msgs::PlanningScene& planning_scene,
                                  const move_arm_msgs::HeadMonitorFeedback& feedback);

protected:

  std::string makeMetaStringFromHostname();

  void addPlanningSceneTimeToMetadata(const planning_environment_msgs::PlanningScene& planning_scene, std::string& metadata);

  warehouse::WarehouseClient warehouse_client_;
  warehouse::Collection<planning_environment_msgs::PlanningScene> planning_scene_collection_;
  warehouse::Collection<motion_planning_msgs::MotionPlanRequest> motion_plan_request_collection_;
  warehouse::Collection<trajectory_msgs::JointTrajectory> trajectory_collection_;
  warehouse::Collection<motion_planning_msgs::ArmNavigationErrorCodes> outcome_collection_;
  warehouse::Collection<move_arm_msgs::HeadMonitorFeedback> paused_state_collection_;
  
  std::string hostname_;
  
};

}
#endif
