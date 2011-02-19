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

#include "planning_environment/models/collision_models_interface.h"
#include "planning_environment/models/model_utils.h"

planning_environment::CollisionModelsInterface::CollisionModelsInterface(const std::string& description)
  : CollisionModels(description)
{
  planning_scene_state_ = NULL;
  set_planning_scene_service_ = priv_nh_.advertiseService("set_planning_scene", &CollisionModelsInterface::setPlanningSceneService, this);
  revert_planning_scene_service_ = priv_nh_.advertiseService("revert_planning_scene", &CollisionModelsInterface::revertPlanningSceneService, this);
}

planning_environment::CollisionModelsInterface::~CollisionModelsInterface()
{
  if(planning_scene_state_ != NULL) {
    delete planning_scene_state_;
  }
}

bool planning_environment::CollisionModelsInterface::setPlanningSceneService(planning_environment_msgs::SetPlanningScene::Request& request,
                                                                             planning_environment_msgs::SetPlanningScene::Response& response)
{
  if(planning_scene_set_) {
    response.ok = false;
    return true;
  }
  planning_scene_state_ = setPlanningScene(request.planning_scene);
  if(planning_scene_state_ == NULL) {
    response.ok = false;
    ROS_ERROR("Setting planning scene state to NULL");
    return true;
  }
  last_planning_scene_ = request.planning_scene;
  response.ok = true;
  return true;
}

bool planning_environment::CollisionModelsInterface::revertPlanningSceneService(std_srvs::Empty::Request& request,
                                                                                std_srvs::Empty::Response& response)
{
  if(!planning_scene_set_) {
    return true;
  }
  revertPlanningScene(planning_scene_state_);
  return true;
}

void planning_environment::CollisionModelsInterface::resetToStartState(planning_models::KinematicState& state) const {
  setRobotStateAndComputeTransforms(last_planning_scene_.robot_state, state);
}
