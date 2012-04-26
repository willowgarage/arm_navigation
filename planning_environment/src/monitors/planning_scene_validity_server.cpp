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
#include <ros/ros.h>

#include <planning_environment/models/collision_models_interface.h>
#include <planning_environment/models/model_utils.h>

#include <arm_navigation_msgs/GetJointTrajectoryValidity.h>
#include <arm_navigation_msgs/GetStateValidity.h>

namespace planning_environment
{
class PlanningSceneValidityServer
{
public:	
  PlanningSceneValidityServer() :
  private_handle_("~")
  {
    std::string robot_description_name = root_handle_.resolveName("robot_description", true);
    collision_models_interface_ = new planning_environment::CollisionModelsInterface(robot_description_name);

    vis_marker_publisher_ = root_handle_.advertise<visualization_msgs::Marker>("planning_scene_markers", 128);
    vis_marker_array_publisher_ = root_handle_.advertise<visualization_msgs::MarkerArray>("planning_scene_markers_array", 128);

    get_trajectory_validity_service_ = private_handle_.advertiseService("get_trajectory_validity", &PlanningSceneValidityServer::getTrajectoryValidity, this);
    get_state_validity_service_ = private_handle_.advertiseService("get_state_validity", &PlanningSceneValidityServer::getStateValidity, this);
  }
	
  virtual ~PlanningSceneValidityServer()
  {
    delete collision_models_interface_;
  }

  bool getTrajectoryValidity(arm_navigation_msgs::GetJointTrajectoryValidity::Request &req, 
                             arm_navigation_msgs::GetJointTrajectoryValidity::Response &res) 
  {
    collision_models_interface_->bodiesLock();
    if(!collision_models_interface_->isPlanningSceneSet()) {
      res.error_code.val = res.error_code.COLLISION_CHECKING_UNAVAILABLE;
      ROS_WARN_STREAM("Calling getTrajectoryValidity with no planning scene set");
      collision_models_interface_->bodiesUnlock();
      return true;
    }
    collision_models_interface_->resetToStartState(*collision_models_interface_->getPlanningSceneState());
    planning_environment::setRobotStateAndComputeTransforms(req.robot_state,
                                                            *collision_models_interface_->getPlanningSceneState());

    arm_navigation_msgs::Constraints goal_constraints;
    if(req.check_goal_constraints) {
      goal_constraints = req.goal_constraints;
    }
    arm_navigation_msgs::Constraints path_constraints;
    if(req.check_path_constraints) {
      path_constraints = req.path_constraints;
    }
    collision_models_interface_->isJointTrajectoryValid(*collision_models_interface_->getPlanningSceneState(),
                                                        req.trajectory,
                                                        goal_constraints,
                                                        path_constraints,
                                                        res.error_code,
                                                        res.trajectory_error_codes,
                                                        req.check_full_trajectory);
    collision_models_interface_->bodiesUnlock();
    return true;
  }

  bool getStateValidity(arm_navigation_msgs::GetStateValidity::Request &req, 
                        arm_navigation_msgs::GetStateValidity::Response &res) 
  {
    collision_models_interface_->bodiesLock();
    if(!collision_models_interface_->isPlanningSceneSet()) {
      res.error_code.val = res.error_code.COLLISION_CHECKING_UNAVAILABLE;
      ROS_WARN_STREAM("Calling getStateValidity with no planning scene set");
      collision_models_interface_->bodiesUnlock();
      return true;
    }
    collision_models_interface_->resetToStartState(*collision_models_interface_->getPlanningSceneState());
    const planning_models::KinematicModel::JointModelGroup* jmg = NULL;

    if(!req.group_name.empty()) {
      jmg = collision_models_interface_->getKinematicModel()->getModelGroup(req.group_name);
      if(!jmg) {
        ROS_WARN_STREAM("No group name " << req.group_name << " in state validity check");
        res.error_code.val = res.error_code.INVALID_GROUP_NAME;
        collision_models_interface_->bodiesUnlock();
        return true;
      }
      collision_models_interface_->disableCollisionsForNonUpdatedLinks(req.group_name);
    }
    planning_environment::setRobotStateAndComputeTransforms(req.robot_state,
                                                            *collision_models_interface_->getPlanningSceneState());
    arm_navigation_msgs::Constraints goal_constraints;
    if(req.check_goal_constraints) {
      goal_constraints = req.goal_constraints;
    }
    arm_navigation_msgs::Constraints path_constraints;
    if(req.check_path_constraints) {
      path_constraints = req.path_constraints;
    }
    std::vector<std::string> joint_names;
    if(req.check_joint_limits) {
      if(!jmg) {
        joint_names = req.robot_state.joint_state.name;
      } else {
        joint_names = jmg->getJointModelNames();
      }
    }
    collision_models_interface_->isKinematicStateValid(*collision_models_interface_->getPlanningSceneState(),
                                                       joint_names,
                                                       res.error_code,
                                                       goal_constraints,
                                                       path_constraints,
                                                       true);
    if(res.error_code.val == arm_navigation_msgs::ArmNavigationErrorCodes::COLLISION_CONSTRAINTS_VIOLATED) {
      collision_models_interface_->getAllCollisionsForState(*collision_models_interface_->getPlanningSceneState(),
                                                            res.contacts, 
                                                            1);
    }
    collision_models_interface_->bodiesUnlock();
    return true;

  }

  void broadcastPlanningSceneMarkers()
  {
    collision_models_interface_->bodiesLock();
    if(!collision_models_interface_->isPlanningSceneSet()) {
      collision_models_interface_->bodiesUnlock();
      return;
    }
    collision_models_interface_->resetToStartState(*collision_models_interface_->getPlanningSceneState());
    visualization_msgs::MarkerArray arr;
    std_msgs::ColorRGBA col;
    col.a = .9;
    col.r = 0.0;
    col.b = 1.0;
    col.g = 0.0;
    collision_models_interface_->getCollisionMapAsMarkers(arr,
							  col,
							  ros::Duration(0.2));
    col.g = 1.0;
    col.b = 1.0;
    collision_models_interface_->getStaticCollisionObjectMarkers(arr,
								 "",
								 col,
								 ros::Duration(0.2));

    col.r = 0.6;
    col.g = 0.4;
    col.b = 0.3;

    collision_models_interface_->getAttachedCollisionObjectMarkers(*collision_models_interface_->getPlanningSceneState(),
								   arr,
								   "",
								   col,
								   ros::Duration(0.2));
    vis_marker_array_publisher_.publish(arr);
    collision_models_interface_->bodiesUnlock();
  }

	
private:
		
	
  ros::NodeHandle root_handle_, private_handle_;
  planning_environment::CollisionModelsInterface *collision_models_interface_;

  ros::ServiceServer get_trajectory_validity_service_;
  ros::ServiceServer get_state_validity_service_;

  ros::Publisher vis_marker_publisher_;
  ros::Publisher vis_marker_array_publisher_;
};    
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planning_scene_validity_server");
  //figuring out whether robot_description has been remapped

  ros::AsyncSpinner spinner(1); 
  spinner.start();
  ros::NodeHandle nh;
  planning_environment::PlanningSceneValidityServer validity_server;
  ros::Rate r(10.0);
  while(nh.ok()) {
    validity_server.broadcastPlanningSceneMarkers();
    r.sleep();
  }
  ros::waitForShutdown();
  return 0;
}
