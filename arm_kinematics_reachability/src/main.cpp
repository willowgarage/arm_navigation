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

  /**** WORKSPACE PARAMETERS - These are the parameters you need to change to specify a different 
region in the workspace for which reachability is to be computed****/
  arm_kinematics_reachability::ArmKinematicsReachability aw;
  kinematics_msgs::WorkspacePoints workspace;

/*
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

  // box about countertop planning  docs/user_stories/reachability_study_results/2012-03-01-19-22-11.bag
  workspace.position_resolution = 0.1;
  workspace.parameters.workspace_region_shape.type = workspace.parameters.workspace_region_shape.BOX;
  workspace.parameters.workspace_region_shape.dimensions.resize(3);
  workspace.parameters.workspace_region_shape.dimensions[0] = 1.7;
  workspace.parameters.workspace_region_shape.dimensions[1] = 1.7;
  workspace.parameters.workspace_region_shape.dimensions[2] = 1.5;

  workspace.parameters.workspace_region_pose.header.frame_id = root_name;
  workspace.parameters.workspace_region_pose.pose.position.x = 0.0;
  workspace.parameters.workspace_region_pose.pose.position.y = 0.0;
  workspace.parameters.workspace_region_pose.pose.position.z = 0.2;
  workspace.parameters.workspace_region_pose.pose.orientation.w = 1.0;

  // box about countertop planning  docs/user_stories/reachability_study_results/
  // xy slice 25cm above table
  workspace.position_resolution = 0.05;
  workspace.parameters.workspace_region_shape.type = workspace.parameters.workspace_region_shape.BOX;
  workspace.parameters.workspace_region_shape.dimensions.resize(3);
  workspace.parameters.workspace_region_shape.dimensions[0] = 1.5;
  workspace.parameters.workspace_region_shape.dimensions[1] = 0.635;
  workspace.parameters.workspace_region_shape.dimensions[2] = 0.250;

  workspace.parameters.workspace_region_pose.header.frame_id = root_name;
  workspace.parameters.workspace_region_pose.pose.position.x = 0;
  workspace.parameters.workspace_region_pose.pose.position.y = 0.5842;
  workspace.parameters.workspace_region_pose.pose.position.z = 0.11 + 0.125;
  workspace.parameters.workspace_region_pose.pose.orientation.w = 1.0;


  // box about countertop planning  docs/user_stories/reachability_study_results/
  // yz slice at x = 0, just above the counter
  workspace.position_resolution = 0.05;
  workspace.parameters.workspace_region_shape.type = workspace.parameters.workspace_region_shape.BOX;
  workspace.parameters.workspace_region_shape.dimensions.resize(3);
  workspace.parameters.workspace_region_shape.dimensions[0] = 0.04;
  workspace.parameters.workspace_region_shape.dimensions[1] = 1.1;
  workspace.parameters.workspace_region_shape.dimensions[2] = 0.8;

  workspace.parameters.workspace_region_pose.header.frame_id = root_name;
  workspace.parameters.workspace_region_pose.pose.position.x = 0;
  workspace.parameters.workspace_region_pose.pose.position.y = 0.6;
  workspace.parameters.workspace_region_pose.pose.position.z = 0.11 + 0.4;
  workspace.parameters.workspace_region_pose.pose.orientation.w = 1.0;
*/

/*
  // reaching around the base with ground at z = -30.81cm
  workspace.position_resolution = 0.05;
  workspace.parameters.workspace_region_shape.type = workspace.parameters.workspace_region_shape.BOX;
  workspace.parameters.workspace_region_shape.dimensions.resize(3);
  workspace.parameters.workspace_region_shape.dimensions[0] = 1.2;
  workspace.parameters.workspace_region_shape.dimensions[1] = 1.0;
  workspace.parameters.workspace_region_shape.dimensions[2] = 0.60;

  workspace.parameters.workspace_region_pose.header.frame_id = root_name;
  workspace.parameters.workspace_region_pose.pose.position.x = 0.0;
  workspace.parameters.workspace_region_pose.pose.position.y = -0.5+0.4572/2.0;
  workspace.parameters.workspace_region_pose.pose.position.z = 0.30;
  workspace.parameters.workspace_region_pose.pose.orientation.w = 1.0;


*/
  // reaching on side countertop
  //  <box size="0.635 1.27 0.05"/>
  //  <origin rpy="0.0 0.0 0.0" xyz="0.6604 0 0.88940"/>
  workspace.position_resolution = 0.05;
  workspace.parameters.workspace_region_shape.type = workspace.parameters.workspace_region_shape.BOX;
  workspace.parameters.workspace_region_shape.dimensions.resize(3);
  workspace.parameters.workspace_region_shape.dimensions[0] = 1.27;
  workspace.parameters.workspace_region_shape.dimensions[1] = 0.635;
  workspace.parameters.workspace_region_shape.dimensions[2] = 0.35;

  workspace.parameters.workspace_region_pose.header.frame_id = root_name;
  workspace.parameters.workspace_region_pose.pose.position.x = 0.0;
  workspace.parameters.workspace_region_pose.pose.position.y = -0.6604;
  workspace.parameters.workspace_region_pose.pose.position.z = 0.9144+0.35/2.0;
  workspace.parameters.workspace_region_pose.pose.orientation.w = 1.0;

/*
  // reaching on front countertop
  //  <box size="0.635 1.27 0.05"/>
  //  <origin rpy="0.0 0.0 0.0" xyz="0.6604 0 0.88940"/>
  workspace.position_resolution = 0.05;
  workspace.parameters.workspace_region_shape.type = workspace.parameters.workspace_region_shape.BOX;
  workspace.parameters.workspace_region_shape.dimensions.resize(3);
  workspace.parameters.workspace_region_shape.dimensions[0] = 0.635;
  workspace.parameters.workspace_region_shape.dimensions[1] = 1.27;
  workspace.parameters.workspace_region_shape.dimensions[2] = 0.35;

  workspace.parameters.workspace_region_pose.header.frame_id = root_name;
  workspace.parameters.workspace_region_pose.pose.position.x = 0.6604;
  workspace.parameters.workspace_region_pose.pose.position.y = 0.0;
  workspace.parameters.workspace_region_pose.pose.position.z = 0.9144+0.35/2.0;
  workspace.parameters.workspace_region_pose.pose.orientation.w = 1.0;

  // reaching front bin area
  workspace.position_resolution = 0.05;
  workspace.parameters.workspace_region_shape.type = workspace.parameters.workspace_region_shape.BOX;
  workspace.parameters.workspace_region_shape.dimensions.resize(3);
  workspace.parameters.workspace_region_shape.dimensions[0] = 0.30;
  workspace.parameters.workspace_region_shape.dimensions[1] = 0.45;
  workspace.parameters.workspace_region_shape.dimensions[2] = 0.25;

  workspace.parameters.workspace_region_pose.header.frame_id = root_name;
  workspace.parameters.workspace_region_pose.pose.position.x = 0.15;
  workspace.parameters.workspace_region_pose.pose.position.y = 0.0;
  workspace.parameters.workspace_region_pose.pose.position.z = 0.5-0.075;
  workspace.parameters.workspace_region_pose.pose.orientation.w = 1.0;

  // reaching front bin area
  workspace.position_resolution = 0.02;
  workspace.parameters.workspace_region_shape.type = workspace.parameters.workspace_region_shape.BOX;
  workspace.parameters.workspace_region_shape.dimensions.resize(3);
  workspace.parameters.workspace_region_shape.dimensions[0] = 0.30;
  workspace.parameters.workspace_region_shape.dimensions[1] = 0.45;
  workspace.parameters.workspace_region_shape.dimensions[2] = 0.25;

  workspace.parameters.workspace_region_pose.header.frame_id = root_name;
  workspace.parameters.workspace_region_pose.pose.position.x = 0.15;
  workspace.parameters.workspace_region_pose.pose.position.y = 0.0;
  workspace.parameters.workspace_region_pose.pose.position.z = 0.5-0.075;
  workspace.parameters.workspace_region_pose.pose.orientation.w = 1.0;
*/



  //SET OF ORIENTATIONS TO TEST FOR REACHABILITY

  geometry_msgs::Quaternion quaternion;

  /*
  quaternion.w = 1.0;
  workspace.orientations.push_back(quaternion);

  quaternion = tf::createQuaternionMsgFromYaw(M_PI/2.0);
  workspace.orientations.push_back(quaternion);
  quaternion = tf::createQuaternionMsgFromYaw(M_PI);
  workspace.orientations.push_back(quaternion);
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

  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,M_PI/4.0,3*M_PI/4.0);
  workspace.orientations.push_back(quaternion);

  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,-M_PI/4.0,3*M_PI/4.0);
  workspace.orientations.push_back(quaternion);

  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,-M_PI/4.0,-3*M_PI/4.0);
  workspace.orientations.push_back(quaternion);

  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,M_PI/4.0,-3*M_PI/4.0);
  workspace.orientations.push_back(quaternion);
  */
 // quaternion.w = 1.0;
 // workspace.orientations.push_back(quaternion);

 //  quaternion = tf::createQuaternionMsgFromYaw(M_PI/2.0);
 //  workspace.orientations.push_back(quaternion);
 // 
 //  quaternion = tf::createQuaternionMsgFromYaw(M_PI);
 //  workspace.orientations.push_back(quaternion);
 // 
 //  quaternion = tf::createQuaternionMsgFromYaw(-M_PI/2.0);
 //  workspace.orientations.push_back(quaternion);

  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,M_PI,0.0); //down
  workspace.orientations.push_back(quaternion);
  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,3.0*M_PI/4.0,0.0); //down/forward
  workspace.orientations.push_back(quaternion);
  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,-3.0*M_PI/4.0,0.0); //down/back
  workspace.orientations.push_back(quaternion);
  quaternion = tf::createQuaternionMsgFromRollPitchYaw(3.0*M_PI/4.0,0.0,0.0); //down/right
  workspace.orientations.push_back(quaternion);
  quaternion = tf::createQuaternionMsgFromRollPitchYaw(-3.0*M_PI/4.0,0.0,0.0); //down/left
  workspace.orientations.push_back(quaternion);
  quaternion = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2.0,0.0,0.0); //right
  workspace.orientations.push_back(quaternion);
  quaternion = tf::createQuaternionMsgFromRollPitchYaw(-M_PI/2.0,0.0,0.0); //left
  workspace.orientations.push_back(quaternion);
  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,M_PI/2.0,0.0); //front
  workspace.orientations.push_back(quaternion);
  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,-M_PI/2.0,0.0); //back

  while(!aw.isActive())
  {
    sleep(1.0);
    ROS_INFO("Waiting for planning scene to be set");
  }

  aw.computeWorkspace(workspace);
  //  aw.visualize(workspace,"full");
  aw.visualizeWithArrows(workspace,"full");
  //  aw.visualize(workspace,"RPY(0,0,0)",zero_orientation);
  ROS_INFO("Success");

  aw.publishWorkspace(workspace);

  ros::waitForShutdown();

  return(0);
}
