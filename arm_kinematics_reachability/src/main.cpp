/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Sachin Chitta
 */

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <tf/transform_datatypes.h>
#include <planning_environment/models/robot_models.h>
#include <planning_environment/models/model_utils.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <arm_kinematics_reachability/arm_kinematics_reachability.h>


static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_workspace_tests");
  ros::AsyncSpinner spinner(2); 
  spinner.start();

  ros::NodeHandle node_handle("~");
  std::string group_name, root_name;
  node_handle.param<std::string>("group", group_name, std::string());
  node_handle.param<std::string>(group_name+"/root_name", root_name, std::string());
  ros::NodeHandle root_handle;

  /*  ros::ServiceClient set_planning_scene_diff_client;
  ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
  set_planning_scene_diff_client = root_handle.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);
  arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
  arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;  
  */

  /**** WORKSPACE PARAMETERS - These are the only parameters you should need to change ****/
  arm_kinematics_reachability::ArmKinematicsReachability aw;
  kinematics_msgs::WorkspacePoints workspace;
  workspace.position_resolution = 0.05;
  workspace.parameters.workspace_region_shape.type = workspace.parameters.workspace_region_shape.BOX;
  workspace.parameters.workspace_region_shape.dimensions.resize(3);
  workspace.parameters.workspace_region_shape.dimensions[0] = 0.4;
  workspace.parameters.workspace_region_shape.dimensions[1] = 1.5;
  workspace.parameters.workspace_region_shape.dimensions[2] = 0.04;

  workspace.parameters.workspace_region_pose.header.frame_id = root_name;
  workspace.parameters.workspace_region_pose.pose.position.x = 0.6;
  workspace.parameters.workspace_region_pose.pose.position.y = 0.0;
  workspace.parameters.workspace_region_pose.pose.position.z = 0.4;
  workspace.parameters.workspace_region_pose.pose.orientation.w = 1.0;

  /*  planning_scene_req.planning_scene_diff.collision_objects.resize(1);
  planning_scene_req.planning_scene_diff.collision_objects[0].shapes.resize(1);
  planning_scene_req.planning_scene_diff.collision_objects[0].poses.resize(1);
  planning_scene_req.planning_scene_diff.collision_objects[0].id = "dummy_object";

  planning_scene_req.planning_scene_diff.collision_objects[0].header.frame_id = root_name;
  planning_scene_req.planning_scene_diff.collision_objects[0].poses[0].position.x = 100.0;
  planning_scene_req.planning_scene_diff.collision_objects[0].poses[0].position.y = 100.0;
  planning_scene_req.planning_scene_diff.collision_objects[0].poses[0].position.z = 100.0;

  planning_scene_req.planning_scene_diff.collision_objects[0].poses[0].orientation.x = 0.0;
  planning_scene_req.planning_scene_diff.collision_objects[0].poses[0].orientation.y = 0.0;
  planning_scene_req.planning_scene_diff.collision_objects[0].poses[0].orientation.z = 0.0;
  planning_scene_req.planning_scene_diff.collision_objects[0].poses[0].orientation.w = 1.0;

  planning_scene_req.planning_scene_diff.collision_objects[0].shapes[0].type = workspace.parameters.workspace_region_shape.BOX;
  planning_scene_req.planning_scene_diff.collision_objects[0].shapes[0].dimensions.resize(3);
  planning_scene_req.planning_scene_diff.collision_objects[0].shapes[0].dimensions[0] = 0.4;
  planning_scene_req.planning_scene_diff.collision_objects[0].shapes[0].dimensions[1] = 1.5;
  planning_scene_req.planning_scene_diff.collision_objects[0].shapes[0].dimensions[2] = 0.04;
  
  planning_environment::RobotModels robot_model("robot_description");
  planning_models::KinematicState kinematic_state(robot_model.getKinematicModel());
  ros::Time my_time = ros::Time::now();

  kinematic_state.setKinematicStateToDefault();
  planning_environment::convertKinematicStateToRobotState(kinematic_state,
                                                          my_time,
                                                          robot_model.getRobotFrameId(),
                                                          planning_scene_req.planning_scene_diff.robot_state);
  
  if(!set_planning_scene_diff_client.call(planning_scene_req, planning_scene_res)) 
  {
    ROS_ERROR("Can't get planning scene");
    return false;
  }

  */

  //ACTUAL REACHABILITY TESTS

  geometry_msgs::Quaternion quaternion;
  quaternion.w = 1.0;
  workspace.orientations.push_back(quaternion);

  quaternion = tf::createQuaternionMsgFromYaw(M_PI/2.0);
  workspace.orientations.push_back(quaternion);
  /*
  quaternion = tf::createQuaternionMsgFromYaw(M_PI);
  workspace.orientations.push_back(quaternion);
  */
  quaternion = tf::createQuaternionMsgFromYaw(-M_PI/2.0);
  workspace.orientations.push_back(quaternion);

  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,M_PI/2.0,0.0);
  workspace.orientations.push_back(quaternion);

  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,-M_PI/2.0,0.0);
  workspace.orientations.push_back(quaternion);

  // The octants
  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,M_PI/4.0,M_PI/4.0);
  workspace.orientations.push_back(quaternion);

  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,-M_PI/4.0,M_PI/4.0);
  workspace.orientations.push_back(quaternion);

  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,-M_PI/4.0,-M_PI/4.0);
  workspace.orientations.push_back(quaternion);

  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,M_PI/4.0,-M_PI/4.0);
  workspace.orientations.push_back(quaternion);

  /*
  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,M_PI/4.0,3*M_PI/4.0);
  workspace.orientations.push_back(quaternion);

  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,-M_PI/4.0,3*M_PI/4.0);
  workspace.orientations.push_back(quaternion);

  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,-M_PI/4.0,-3*M_PI/4.0);
  workspace.orientations.push_back(quaternion);

  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,M_PI/4.0,-3*M_PI/4.0);
  workspace.orientations.push_back(quaternion);
  */
  while(!aw.isActive())
  {
    sleep(1.0);
    ROS_INFO("Waiting for planning scene to be set");
  }

  aw.computeWorkspace(workspace);
  aw.visualize(workspace,"full");
  //  aw.visualize(workspace,"full");
  //  aw.visualize(workspace,"RPY(0,0,0)",zero_orientation);
  //  aw.getReachableWorkspace(workspace);
  //  aw.visualize(workspace);
  ROS_INFO("Success");

  aw.publishWorkspace(workspace);

  ros::waitForShutdown();

  return(0);
}
