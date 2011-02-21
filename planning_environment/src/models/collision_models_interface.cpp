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

  set_planning_scene_callback_ = NULL;
  revert_planning_scene_callback_ = NULL;

  action_server_ = new actionlib::SimpleActionServer<planning_environment_msgs::SetPlanningSceneAction>(nh_, "set_planning_scene",
                                                                                                        boost::bind(&CollisionModelsInterface::setPlanningSceneCallback, this, _1), true);
}

planning_environment::CollisionModelsInterface::~CollisionModelsInterface()
{
  if(planning_scene_state_ != NULL) {
    delete planning_scene_state_;
  }
}

void planning_environment::CollisionModelsInterface::setPlanningSceneCallback(const planning_environment_msgs::SetPlanningSceneGoalConstPtr& scene)
{
  planning_environment_msgs::SetPlanningSceneResult res;
  res.ok = true;

  //locks the callback thread
  if(planning_scene_set_) {
    ROS_WARN_STREAM("Planning scene set, but we're in the action callback");
    res.ok = false;
    action_server_->setAborted(res);
  }
  planning_scene_state_ = setPlanningScene(scene->planning_scene);
  if(planning_scene_state_ == NULL) {
    ROS_ERROR("Setting planning scene state to NULL");
    res.ok = false;
    action_server_->setAborted(res);
    return;
  }
  last_planning_scene_ = scene->planning_scene;
  planning_environment_msgs::SetPlanningSceneFeedback feedback;
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
  ros::Rate r(10.0);
  while(ros::ok() && !action_server_->isPreemptRequested())
  {
    r.sleep();
  }
  ROS_INFO_STREAM("Reverting planning scene");
  revertPlanningScene(planning_scene_state_);
  planning_scene_state_ = NULL;
  if(revert_planning_scene_callback_ != NULL) {
    revert_planning_scene_callback_();
  }
  action_server_->setPreempted(res);
}

void planning_environment::CollisionModelsInterface::resetToStartState(planning_models::KinematicState& state) const {
  setRobotStateAndComputeTransforms(last_planning_scene_.robot_state, state);
}
