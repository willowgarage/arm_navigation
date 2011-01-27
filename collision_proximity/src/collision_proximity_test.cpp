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
#include <tf/transform_listener.h>
#include <planning_environment/monitors/planning_monitor.h>
#include <collision_proximity/collision_proximity_space.h>
#include <planning_environment_msgs/GetPlanningScene.h>
#include <planning_environment_msgs/GetRobotState.h>
#include <planning_environment/models/model_utils.h>

static const std::string planning_scene_name = "/environment_server/get_planning_scene";      
static const std::string robot_state_name = "/environment_server/get_robot_state";      

int main(int argc, char** argv)
{
  ros::init(argc, argv, "collision_proximity_test");
 
  ros::NodeHandle nh;
  
  std::string robot_description_name = nh.resolveName("robot_description", true);

  ros::WallTime n1 = ros::WallTime::now();
  collision_proximity::CollisionProximitySpace* cps = new collision_proximity::CollisionProximitySpace(robot_description_name);
  ros::WallTime n2 = ros::WallTime::now();

  ROS_INFO_STREAM("Creation took " << (n2-n1).toSec());

  ros::ServiceClient planning_scene_client = nh.serviceClient<planning_environment_msgs::GetPlanningScene>(planning_scene_name, true);      
  ros::service::waitForService(planning_scene_name);

  ros::ServiceClient robot_state_service = nh.serviceClient<planning_environment_msgs::GetRobotState>(robot_state_name, true);      
  ros::service::waitForService(robot_state_name);

  planning_environment_msgs::GetRobotState::Request rob_state_req;
  planning_environment_msgs::GetRobotState::Response rob_state_res;

  planning_environment_msgs::GetPlanningScene::Request req;
  planning_environment_msgs::GetPlanningScene::Response res;

  mapping_msgs::CollisionObject obj1;
  obj1.header.stamp = ros::Time::now();
  obj1.header.frame_id = "odom_combined";
  obj1.id = "obj1";
  obj1.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
  obj1.shapes.resize(1);
  obj1.shapes[0].type = geometric_shapes_msgs::Shape::BOX;
  obj1.shapes[0].dimensions.resize(3);
  obj1.shapes[0].dimensions[0] = .1;
  obj1.shapes[0].dimensions[1] = .1;
  obj1.shapes[0].dimensions[2] = .75;
  obj1.poses.resize(1);
  obj1.poses[0].position.x = .6;
  obj1.poses[0].position.y = -.6;
  obj1.poses[0].position.z = .375;
  obj1.poses[0].orientation.w = 1.0;

  mapping_msgs::AttachedCollisionObject att_obj;
  att_obj.object = obj1;
  att_obj.object.header.stamp = ros::Time::now();
  att_obj.object.header.frame_id = "r_gripper_palm_link";
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
  att_obj.object.shapes[0].type = geometric_shapes_msgs::Shape::CYLINDER;
  att_obj.object.shapes[0].dimensions.resize(2);
  att_obj.object.shapes[0].dimensions[0] = .025;
  att_obj.object.shapes[0].dimensions[1] = .5;
  att_obj.object.poses.resize(1);
  att_obj.object.poses[0].position.x = .12;
  att_obj.object.poses[0].position.y = 0.0;
  att_obj.object.poses[0].position.z = 0.0;
  att_obj.object.poses[0].orientation.w = 1.0;

  req.collision_object_diffs.push_back(obj1);
  req.attached_collision_object_diffs.push_back(att_obj);

  n1 = ros::WallTime::now();
  planning_scene_client.call(req,res);
  n2 = ros::WallTime::now();

  ROS_INFO_STREAM("Service call took " << (n2-n1).toSec());
  
  n1 = ros::WallTime::now();
  planning_models::KinematicState* state = cps->setupForGroupQueries("right_arm",
                                                                     res.complete_robot_state,
                                                                     res.allowed_collision_matrix,
                                                                     res.transformed_allowed_contacts,
                                                                     res.all_link_padding,
                                                                     res.all_collision_objects,
                                                                     res.all_attached_collision_objects,
                                                                     res.unmasked_collision_map);
  n2 = ros::WallTime::now();
  
  ROS_INFO_STREAM("Setup took "  << (n2-n1).toSec());

  ros::Rate r(10.0);
  std::vector<double> link_distances;
  std::vector<std::vector<double> > distances;
  std::vector<std::vector<btVector3> > gradients;
  std::vector<std::string> link_names;
  std::vector<std::string> attached_body_names;
  std::vector<collision_proximity::CollisionType> collisions;

  cps->visualizeDistanceField();
  
  while(nh.ok()) {
    
    n1 = ros::WallTime::now();
    robot_state_service.call(rob_state_req,rob_state_res);
    n2 = ros::WallTime::now();
    //ROS_INFO_STREAM("Get state req took " << (n2-n1).toSec());

    planning_environment::setRobotStateAndComputeTransforms(rob_state_res.robot_state, *state);

    cps->setCurrentGroupState(*state);
    bool in_collision; 
    cps->getStateCollisions(link_names, attached_body_names, in_collision, collisions);
    cps->visualizeCollisions(link_names, attached_body_names, collisions);
    cps->getStateGradients(link_names, attached_body_names, link_distances, distances, gradients);
    cps->visualizeProximityGradients(link_names, attached_body_names, link_distances, distances, gradients);
    std::vector<std::string> objs = link_names;
    objs.insert(objs.end(), attached_body_names.begin(), attached_body_names.end());
    cps->visualizeObjectSpheres(objs);
    //std::vector<std::string> all_links = cps->->getGroupLinkUnion();
    //cps->visualizeObjectVoxels(all_links);
    //cps->visualizeConvexMeshes(link_names);
    //cps->visualizePaddedTrimeshes(cur_state, link_names);
    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();
  delete cps;

}
