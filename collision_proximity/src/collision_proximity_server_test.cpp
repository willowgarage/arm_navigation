/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#include <ros/ros.h>
#include <collision_proximity/collision_proximity_space.h>
#include <planning_environment/models/model_utils.h>
#include <motion_planning_msgs/GetMotionPlan.h>

struct CollisionProximitySpacePlannerInterface
{
  CollisionProximitySpacePlannerInterface(const std::string& robot_description_name)
  {
    cps_ = new collision_proximity::CollisionProximitySpace(robot_description_name);
    
    ros::NodeHandle priv("~");

    motion_planning_service_ = priv.advertiseService("get_distance_aware_plan", &CollisionProximitySpacePlannerInterface::motionPlanCallback, this);

    vis_marker_publisher_ = root_handle_.advertise<visualization_msgs::Marker>("collision_proximity_markers", 128);
    vis_marker_array_publisher_ = root_handle_.advertise<visualization_msgs::MarkerArray>("collision_proximity_markers_array", 128);

  }

  ~CollisionProximitySpacePlannerInterface()
  {
    delete cps_;
  }
  
  bool motionPlanCallback(motion_planning_msgs::GetMotionPlan::Request &req,
                          motion_planning_msgs::GetMotionPlan::Response &res)
  {
    if(!req.motion_plan_request.group_name.empty()) {
      cps_->setupForGroupQueries(req.motion_plan_request.group_name,
                                 req.motion_plan_request.start_state);
    } else {
      return false;
    }
    return true;
  }
  
  void broadcastCollisionMarkers() {
    cps_->getCollisionModelsInterface()->bodiesLock();
    if(!cps_->getCollisionModelsInterface()->isPlanningSceneSet()) {
      cps_->getCollisionModelsInterface()->bodiesUnlock();
      return;
    }
    cps_->getCollisionModelsInterface()->resetToStartState(*cps_->getCollisionModelsInterface()->getPlanningSceneState());
    std::vector<std::string> link_names;
    std::vector<std::string> attached_body_names;
    std::vector<collision_proximity::GradientInfo> gradients;
    cps_->getStateGradients(link_names, attached_body_names, gradients);

    visualization_msgs::MarkerArray arr;
    cps_->getProximityGradientMarkers(link_names,
                                      attached_body_names,
                                      gradients,
                                      "",
                                      arr);
    vis_marker_array_publisher_.publish(arr);
    cps_->getCollisionModelsInterface()->bodiesUnlock();
  }

  ros::NodeHandle root_handle_;
  ros::ServiceServer motion_planning_service_;
  collision_proximity::CollisionProximitySpace* cps_;
  ros::Publisher vis_marker_publisher_;
  ros::Publisher vis_marker_array_publisher_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "collision_proximity_server_test");

  ros::AsyncSpinner spinner(1); 
  spinner.start();
 
  ros::NodeHandle nh;
  std::string robot_description_name = nh.resolveName("robot_description", true);

  CollisionProximitySpacePlannerInterface cps(robot_description_name);

  ros::Rate r(10.0);
  while(nh.ok()) {
    cps.broadcastCollisionMarkers();
    r.sleep();
  }
  ros::waitForShutdown();
  return 0;


}
