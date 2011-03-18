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
 *  \author Sachin Chitta, Ioan Sucan
 *********************************************************************/

#include "planning_environment/monitors/planning_monitor.h"

#include <planning_environment_msgs/GetRobotState.h>
#include <planning_environment_msgs/GetStateValidity.h>
#include <planning_environment_msgs/GetPlanningScene.h>
#include <planning_environment_msgs/LogPlanningScene.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <planning_environment_msgs/SetPlanningSceneAction.h>
#include <actionlib/client/simple_action_client.h>

static const std::string SET_PLANNING_SCENE_NAME ="set_planning_scene";

namespace planning_environment
{
class EnvironmentServer
{
public:	
  EnvironmentServer() :
  private_handle_("~")
  {
    private_handle_.param<bool>("use_monitor", use_monitor_, true);
    private_handle_.param<bool>("use_collision_map", use_collision_map_, false);

    std::string robot_description_name = root_handle_.resolveName("robot_description", true);

    collision_models_ = new planning_environment::CollisionModels(robot_description_name);
    if(use_monitor_) {
      planning_monitor_ = new planning_environment::PlanningMonitor(collision_models_, &tf_);
      planning_monitor_->setUseCollisionMap(use_collision_map_);

      planning_monitor_->waitForState();
      planning_monitor_->startEnvironmentMonitor();
      bool printed = false;
      while(root_handle_.ok() && use_collision_map_ && !planning_monitor_->haveMap()){
        ros::Duration().fromSec(1.0).sleep();
        if(!printed) {
          ROS_INFO_STREAM("Waiting for collision map");
          printed = true;
        }
      }
    } else {
      planning_monitor_ = NULL;
    }
    
    register_planning_scene_service_ = root_handle_.advertiseService("register_planning_scene", &EnvironmentServer::registerPlanningScene, this);
    get_robot_state_service_ = private_handle_.advertiseService("get_robot_state", &EnvironmentServer::getRobotState, this);
    get_planning_scene_service_ = private_handle_.advertiseService("get_planning_scene", &EnvironmentServer::getPlanningScene, this);
    log_planning_scene_service_ = private_handle_.advertiseService("log_planning_scene", &EnvironmentServer::logPlanningScene, this);

    ROS_INFO("Environment server started");
  }
	
  virtual ~EnvironmentServer()
  {
    for(unsigned int i = 0; i < set_planning_scene_clients_.size(); i++) {
      delete set_planning_scene_clients_[i];
    }
    delete collision_models_;
    if(planning_monitor_) {
      delete planning_monitor_;
    }
  }
	
private:
  
  bool registerPlanningScene(std_srvs::Empty::Request &req,
                             std_srvs::Empty::Response &res)
  {
    if(req.__connection_header->find("callerid") == req.__connection_header->end()) {
      ROS_ERROR_STREAM("Request has no callerid");
      return false;
    }
    std::string callerid = req.__connection_header->find("callerid")->second;
    planning_scene_client_names_.push_back(callerid);
    set_planning_scene_clients_.push_back(new actionlib::SimpleActionClient<planning_environment_msgs::SetPlanningSceneAction>(callerid+"/"+SET_PLANNING_SCENE_NAME, true));
    while(ros::ok()) {
      if(!set_planning_scene_clients_.back()->waitForServer(ros::Duration(1.0))) {
        ROS_INFO_STREAM("Couldn't connect back to action server for " << callerid);
      } else {
        break;
      }
    }
    ROS_INFO_STREAM("Successfully connected to planning scene action server for " << callerid);
    return true;
  }
		
  bool getRobotState(planning_environment_msgs::GetRobotState::Request &req, 
                     planning_environment_msgs::GetRobotState::Response &res)
  {
    planning_monitor_->getCurrentRobotState(res.robot_state);
    return true;    
  }

  bool getPlanningScene(planning_environment_msgs::GetPlanningScene::Request &req, 
                        planning_environment_msgs::GetPlanningScene::Response &res) 
  {
    if(use_monitor_) {
      planning_monitor_->getCompletePlanningScene(req.planning_scene_diff,
                                                  req.operations,
                                                  res.planning_scene);
    } else {
      //TODO - need to be able to load planning scenes
      res.planning_scene = req.planning_scene_diff;
    }
    planning_environment_msgs::SetPlanningSceneGoal planning_scene_goal;
    planning_scene_goal.planning_scene = res.planning_scene;
    for(unsigned int i = 0; i < set_planning_scene_clients_.size(); i++) {
      set_planning_scene_clients_[i]->sendGoal(planning_scene_goal);
    }
    for(unsigned int i = 0; i < set_planning_scene_clients_.size(); i++) {
      while(ros::ok() && !set_planning_scene_clients_[i]->waitForResult(ros::Duration(1.0))) {
        ROS_INFO_STREAM("Waiting for response from planning scene client " << planning_scene_client_names_[i] << ".  State is " << set_planning_scene_clients_[i]->getState().state_);
      }
    }
    return true;
  }

  bool logPlanningScene(planning_environment_msgs::LogPlanningScene::Request &req, 
                        planning_environment_msgs::LogPlanningScene::Response &res) 
  {
    planning_environment_msgs::PlanningScene full_planning_scene;
    planning_monitor_->getCompletePlanningScene(req.planning_scene_diff,
                                                req.operations,
                                                full_planning_scene);
    std::string filename = req.filename;
    if(!req.package_name.empty()) {
      filename = ros::package::getPath(req.package_name)+"/"+req.filename;
    }
    try {
      collision_models_->writePlanningSceneBag(filename, full_planning_scene);
    } catch(...) {
      ROS_INFO_STREAM("Problem writing bag to " << filename);
    }
    return true;
  }


private:
	
  ros::NodeHandle root_handle_, private_handle_;
  planning_environment::CollisionModels *collision_models_;
  planning_environment::PlanningMonitor *planning_monitor_;
  tf::TransformListener tf_;	

  bool use_monitor_;
  bool use_collision_map_;

  ros::ServiceServer get_robot_state_service_;
  ros::ServiceServer get_planning_scene_service_;
  ros::ServiceServer log_planning_scene_service_;

  std::vector<std::string> planning_scene_client_names_;
  ros::ServiceServer register_planning_scene_service_;
  std::vector<actionlib::SimpleActionClient<planning_environment_msgs::SetPlanningSceneAction>* > set_planning_scene_clients_;
};    
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "environment_server");
  //figuring out whether robot_description has been remapped

  ros::AsyncSpinner spinner(4); 
  spinner.start();
  planning_environment::EnvironmentServer environment_monitor;
  ros::waitForShutdown();
  return 0;
}
