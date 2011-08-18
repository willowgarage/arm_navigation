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

#include <std_srvs/Empty.h>

#include "planning_environment/models/collision_models_interface.h"
#include "planning_environment/models/model_utils.h"

static const std::string REGISTER_PLANNING_SCENE_NAME = "register_planning_scene";

planning_environment::CollisionModelsInterface::CollisionModelsInterface(const std::string& description, bool register_with_server)
  : CollisionModels(description)
{
  planning_scene_state_ = NULL;

  set_planning_scene_callback_ = NULL;
  revert_planning_scene_callback_ = NULL;

  ros::NodeHandle root_nh;
  std::string env_service_name = root_nh.resolveName(REGISTER_PLANNING_SCENE_NAME, true);

  while(register_with_server && root_nh.ok()) {
    if(ros::service::waitForService(env_service_name, ros::Duration(1.0))) {
      break;
    }
    ROS_INFO_STREAM("Waiting for environment server planning scene registration service " << env_service_name);
  }
  
  //need to create action server before we request
  action_server_ = new actionlib::SimpleActionServer<arm_navigation_msgs::SyncPlanningSceneAction>(priv_nh_, "sync_planning_scene",
                                                                                                   boost::bind(&CollisionModelsInterface::syncPlanningSceneCallback, this, _1), false);
  action_server_->start();

  if(register_with_server) {
    env_server_register_client_ = root_nh.serviceClient<std_srvs::Empty>(env_service_name);
    
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response res;
    while(ros::ok() && true) {
      if(!env_server_register_client_.call(req, res)) {
        ROS_INFO_STREAM("Couldn't register for planning scenes");
        ros::WallDuration(1.0).sleep();
      } else {
        break;
      }
    }
  }
}

planning_environment::CollisionModelsInterface::~CollisionModelsInterface()
{
  delete action_server_;
  if(planning_scene_state_ != NULL) {
    delete planning_scene_state_;
  }
}

void planning_environment::CollisionModelsInterface::syncPlanningSceneCallback(const arm_navigation_msgs::SyncPlanningSceneGoalConstPtr& scene)
{
  ros::WallTime t1 = ros::WallTime::now();
  bodiesLock();
  arm_navigation_msgs::SyncPlanningSceneResult res;
  res.ok = true;

  ROS_DEBUG("Syncing planning scene");

  if(planning_scene_set_) {
    ROS_DEBUG("Reverting planning scene");
    revertPlanningScene(planning_scene_state_);
    planning_scene_state_ = NULL;
    if(revert_planning_scene_callback_ != NULL) {
      revert_planning_scene_callback_();
    }
  }
  planning_scene_state_ = setPlanningScene(scene->planning_scene);
  if(planning_scene_state_ == NULL) {
    ROS_ERROR("Setting planning scene state to NULL");
    res.ok = false;
    action_server_->setAborted(res);
    bodiesUnlock();
    return;
  }
  last_planning_scene_ = scene->planning_scene;
  arm_navigation_msgs::SyncPlanningSceneFeedback feedback;
  feedback.client_processing = true;
  feedback.ready = false;
  action_server_->publishFeedback(feedback);
  //TODO - we can run the callback in a new thread, but it's going to mean communicating
  //preempts over semaphors and whatnot
  if(set_planning_scene_callback_ != NULL) {
    set_planning_scene_callback_(scene->planning_scene);
  }
  //if we're here, assuming client is ready
  feedback.ready = true;
  action_server_->publishFeedback(feedback);
  action_server_->setSucceeded(res);
  ROS_DEBUG_STREAM("Setting took " << (ros::WallTime::now()-t1).toSec());
  bodiesUnlock();
}

bool planning_environment::CollisionModelsInterface::setPlanningSceneWithCallbacks(const arm_navigation_msgs::PlanningScene& planning_scene)
{
  if(planning_scene_set_) {
    revertPlanningScene(planning_scene_state_);
    planning_scene_state_ = NULL;
    if(revert_planning_scene_callback_ != NULL) {
      revert_planning_scene_callback_();
    }
  }
  planning_scene_state_ = setPlanningScene(planning_scene);
  if(planning_scene_state_ == NULL) {
    ROS_ERROR("Setting planning scene state to NULL");
    return false;
  }
  last_planning_scene_ = planning_scene;
  //TODO - we can run the callback in a new thread, but it's going to mean communicating
  //preempts over semaphors and whatnot
  if(set_planning_scene_callback_ != NULL) {
    set_planning_scene_callback_(planning_scene);
  }
  return true;
}

void planning_environment::CollisionModelsInterface::resetToStartState(planning_models::KinematicState& state) const {
  setRobotStateAndComputeTransforms(last_planning_scene_.robot_state, state);
}
