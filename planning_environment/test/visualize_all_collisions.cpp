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
#include <planning_environment/monitors/planning_monitor.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <arm_navigation_msgs/GetRobotState.h>
#include <planning_environment/models/model_utils.h>

static const std::string planning_scene_name = "/environment_monitor/get_planning_scene";      
static const std::string robot_state_name = "/environment_monitor/get_robot_state";      
static const std::string vis_topic_name = "all_collision_model_collisions";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "collision_proximity_test");
 
  ros::NodeHandle nh;
  
  std::string robot_description_name = nh.resolveName("robot_description", true);

  planning_environment::CollisionModels cmodel(robot_description_name);

  ros::ServiceClient planning_scene_client = nh.serviceClient<arm_navigation_msgs::GetPlanningScene>(planning_scene_name, true);      
  ros::service::waitForService(planning_scene_name);
  
  ros::ServiceClient robot_state_service = nh.serviceClient<arm_navigation_msgs::GetRobotState>(robot_state_name, true);      
  ros::service::waitForService(robot_state_name);
  
  ros::Publisher vis_marker_publisher = nh.advertise<visualization_msgs::Marker>(vis_topic_name, 128);
  ros::Publisher vis_marker_array_publisher = nh.advertise<visualization_msgs::MarkerArray>(vis_topic_name+"_array", 128);

  arm_navigation_msgs::GetRobotState::Request rob_state_req;
  arm_navigation_msgs::GetRobotState::Response rob_state_res;

  arm_navigation_msgs::GetPlanningScene::Request req;
  arm_navigation_msgs::GetPlanningScene::Response res;

  arm_navigation_msgs::CollisionObject obj1;
  obj1.header.stamp = ros::Time::now();
  obj1.header.frame_id = "odom_combined";
  obj1.id = "wall";
  obj1.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  obj1.shapes.resize(1);
  obj1.shapes[0].type = arm_navigation_msgs::Shape::BOX;
  obj1.shapes[0].dimensions.resize(3);
  obj1.shapes[0].dimensions[0] = 0.05;
  obj1.shapes[0].dimensions[1] = 4.0;
  obj1.shapes[0].dimensions[2] = 4.0;
  obj1.poses.resize(1);
  obj1.poses[0].position.x = .7;
  obj1.poses[0].position.y = 0;
  obj1.poses[0].position.z = 1.0;
  obj1.poses[0].orientation.w = 1.0;

  arm_navigation_msgs::AttachedCollisionObject att_obj;
  att_obj.object = obj1;
  att_obj.object.header.stamp = ros::Time::now();
  att_obj.object.header.frame_id = "r_gripper_r_finger_tip_link";
  att_obj.link_name = "r_gripper_palm_link";
  att_obj.touch_links.push_back("r_gripper_palm_link");
  att_obj.touch_links.push_back("r_gripper_r_finger_link");
  att_obj.touch_links.push_back("r_gripper_l_finger_link");
  att_obj.touch_links.push_back("r_gripper_r_finger_tip_link");
  att_obj.touch_links.push_back("r_gripper_l_finger_tip_link");
  att_obj.touch_links.push_back("r_wrist_roll_link");
  att_obj.touch_links.push_back("r_wrist_flex_link");
  att_obj.touch_links.push_back("r_forearm_link");
  att_obj.touch_links.push_back("r_gripper_motor_accelerometer_link");
  att_obj.object.id = "obj2";
  att_obj.object.shapes[0].type = arm_navigation_msgs::Shape::CYLINDER;
  att_obj.object.shapes[0].dimensions.resize(2);
  att_obj.object.shapes[0].dimensions[0] = .02;
  att_obj.object.shapes[0].dimensions[1] = .25;
  att_obj.object.poses.resize(1);
  att_obj.object.poses[0].position.x = 0.0;
  att_obj.object.poses[0].position.y = 0.0;
  att_obj.object.poses[0].position.z = 0.0;
  att_obj.object.poses[0].orientation.w = 1.0;

  req.planning_scene_diff.collision_objects.push_back(obj1);
  req.planning_scene_diff.attached_collision_objects.push_back(att_obj);

  planning_scene_client.call(req,res);

  planning_models::KinematicState* state = cmodel.setPlanningScene(res.planning_scene);

  if(res.planning_scene.attached_collision_objects[0].object.header.frame_id != "r_gripper_palm_link") {
    ROS_INFO_STREAM("Not in link frame");
  }

  if(state == NULL) {
    ROS_ERROR_STREAM("Problem setting state, exiting");
    ros::shutdown();
    exit(0);
  }
  ros::Rate r(10.0);
  while(nh.ok()) {
    
    robot_state_service.call(rob_state_req,rob_state_res);
    planning_environment::setRobotStateAndComputeTransforms(rob_state_res.robot_state, *state);

    std_msgs::ColorRGBA point_color;
    point_color.a = 1.0;
    point_color.r = 1.0;
    point_color.g = .8;
    point_color.b = 0.04;

    std_msgs::ColorRGBA stat_color;
    stat_color.a = 0.5;
    stat_color.r = 0.1;
    stat_color.g = 0.8;
    stat_color.b = 0.3;

    std_msgs::ColorRGBA attached_color;
    attached_color.a = 0.5;
    attached_color.r = 0.6;
    attached_color.g = 0.4;
    attached_color.b = 0.3;

    visualization_msgs::MarkerArray arr;
    cmodel.getAllCollisionPointMarkers(*state,
                                       arr,
                                       point_color,
                                       ros::Duration(.2));
 
    cmodel.getAllCollisionSpaceObjectMarkers(*state,
                                             arr,
                                             "",
                                             stat_color,
                                             attached_color,
                                             ros::Duration(.2));

    vis_marker_array_publisher.publish(arr);
    ros::spinOnce();
    r.sleep();
  }

  cmodel.revertPlanningScene(state);
  ros::shutdown();
}
  
  


