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
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <planning_environment_msgs/GetPlanningScene.h>
#include <planning_environment_msgs/GetRobotState.h>
#include <planning_environment/models/model_utils.h>

static const std::string planning_scene_name = "/environment_server/get_planning_scene";      
static const std::string robot_state_name = "/environment_server/get_robot_state";      
static const std::string ARM_QUERY_NAME = "/pr2_right_arm_kinematics/get_ik_solver_info";

static const unsigned int TEST_NUM = 10000;

double gen_rand(double min, double max)
{
  int rand_num = rand()%100+1;
  double result = min + (double)((max-min)*rand_num)/101.0;
  return result;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "collision_proximity_test");
 
  ros::NodeHandle nh;

  std::string robot_description_name = nh.resolveName("robot_description", true);

  srand ( time(NULL) ); // initialize random seed
  ros::service::waitForService(ARM_QUERY_NAME);

  ros::ServiceClient query_client = nh.serviceClient<kinematics_msgs::GetKinematicSolverInfo>(ARM_QUERY_NAME);

  kinematics_msgs::GetKinematicSolverInfo::Request request;
  kinematics_msgs::GetKinematicSolverInfo::Response response;

  query_client.call(request, response);
  int num_joints;
  std::vector<double> min_limits, max_limits;
  num_joints = (int) response.kinematic_solver_info.joint_names.size();
  std::vector<std::string> joint_state_names = response.kinematic_solver_info.joint_names;
  std::map<std::string, double> joint_state_map;
  for(int i=0; i< num_joints; i++)
  {
    joint_state_map[joint_state_names[i]] = 0.0;
    min_limits.push_back(response.kinematic_solver_info.limits[i].min_position);
    max_limits.push_back(response.kinematic_solver_info.limits[i].max_position);
  }
  
  std::vector<std::vector<double> > valid_joint_states;
  valid_joint_states.resize(TEST_NUM);
  for(unsigned int i = 0; i < TEST_NUM; i++) {
    std::vector<double> jv(7,0.0);
    for(unsigned int j=0; j < min_limits.size(); j++)
    {
      jv[j] = gen_rand(std::max(min_limits[j],-M_PI),std::min(max_limits[j],M_PI));
    } 
    valid_joint_states[i] = jv;
  }

  collision_proximity::CollisionProximitySpace* cps = new collision_proximity::CollisionProximitySpace(robot_description_name);
  ros::ServiceClient planning_scene_client = nh.serviceClient<planning_environment_msgs::GetPlanningScene>(planning_scene_name, true);      
  ros::service::waitForService(planning_scene_name);

  planning_environment_msgs::GetPlanningScene::Request ps_req;
  planning_environment_msgs::GetPlanningScene::Response ps_res;

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

  ps_req.collision_object_diffs.push_back(obj1);
  ps_req.attached_collision_object_diffs.push_back(att_obj);

  planning_environment_msgs::GetRobotState::Request rob_state_req;
  planning_environment_msgs::GetRobotState::Response rob_state_res;

  ros::Rate slow_wait(1.0);
  while(1) {
    if(!nh.ok()) {
      ros::shutdown();
      exit(0);
    }
    planning_scene_client.call(ps_req,ps_res);
    ROS_INFO_STREAM("Num coll " << ps_res.all_collision_objects.size() << " att " << ps_res.all_attached_collision_objects.size());
    if(ps_res.all_collision_objects.size() > 0
       && ps_res.all_attached_collision_objects.size() > 0) break;
    slow_wait.sleep();
  }

  ROS_INFO("After test");
  
  ros::ServiceClient robot_state_service = nh.serviceClient<planning_environment_msgs::GetRobotState>(robot_state_name, true);      
  ros::service::waitForService(robot_state_name);

  planning_models::KinematicState* state = cps->setupForGroupQueries("right_arm_and_end_effector",
                                                                     ps_res.complete_robot_state,
                                                                     ps_res.allowed_collision_matrix,
                                                                     ps_res.transformed_allowed_contacts,
                                                                     ps_res.all_link_padding,
                                                                     ps_res.all_collision_objects,
                                                                     ps_res.all_attached_collision_objects,
                                                                     ps_res.unmasked_collision_map);

  ros::WallDuration tot_ode, tot_prox;
  ros::WallDuration min_ode(1000.0,1000.0);
  ros::WallDuration min_prox(1000.0, 1000.0);
  ros::WallDuration max_ode;
  ros::WallDuration max_prox;

  std::vector<double> link_distances;
  std::vector<std::vector<double> > distances;
  std::vector<std::vector<btVector3> > gradients;
  std::vector<std::string> link_names;
  std::vector<std::string> attached_body_names;
  std::vector<collision_proximity::CollisionType> collisions;
  unsigned int prox_num_in_collision = 0;
  unsigned int ode_num_in_collision = 0;

  ros::WallTime n1, n2;

  for(unsigned int i = 0; i < TEST_NUM; i++) {
    for(unsigned int j = 0; j < joint_state_names.size(); j++) {
      joint_state_map[joint_state_names[j]] = valid_joint_states[i][j];
    }
    state->setKinematicState(joint_state_map);
    n1 = ros::WallTime::now();
    cps->setCurrentGroupState(*state);
    bool in_prox_collision = cps->isStateInCollision();
    n2 = ros::WallTime::now();
    if(in_prox_collision) {
      prox_num_in_collision++;
    }
    ros::WallDuration prox_dur(n2-n1);
    if(prox_dur > max_prox) {
      max_prox = prox_dur;
    } else if (prox_dur < min_prox) {
      min_prox = prox_dur;
    }
    tot_prox += prox_dur;
    n1 = ros::WallTime::now();
    bool ode_in_collision = cps->getCollisionModels()->isKinematicStateInCollision(*state);
    n2 = ros::WallTime::now();
    if(ode_in_collision) {
      ode_num_in_collision++;
    }
    if(0){//in_prox_collision && !ode_in_collision) {
      ros::Rate r(1.0);
      while(nh.ok()) {
        cps->getStateCollisions(link_names, attached_body_names, in_prox_collision, collisions);
        ROS_INFO("Prox not ode");
        cps->visualizeDistanceField();
        cps->visualizeCollisions(link_names, attached_body_names, collisions);
        cps->visualizeConvexMeshes(cps->getCollisionModels()->getGroupLinkUnion());
        std::vector<std::string> objs = link_names;
        objs.insert(objs.end(), attached_body_names.begin(), attached_body_names.end());
        cps->visualizeObjectSpheres(objs);
        //cps->visualizeVoxelizedLinks(collision_models->getGroupLinkUnion());
        r.sleep();
      }
      exit(0);
    }
    if(0 && !in_prox_collision && ode_in_collision) {
      ros::Rate r(1.0);
      while(nh.ok()) {
        ROS_INFO("Ode not prox");
        cps->visualizeDistanceField();
        cps->getStateCollisions(link_names, attached_body_names, in_prox_collision, collisions);
        cps->visualizeCollisions(link_names, attached_body_names, collisions);
        cps->visualizeConvexMeshes(cps->getCollisionModels()->getGroupLinkUnion());
        std::vector<std::string> objs = link_names;
        objs.insert(objs.end(), attached_body_names.begin(), attached_body_names.end());
        cps->visualizeObjectSpheres(objs);
        r.sleep();
      }
      //cps->visualizeVoxelizedLinks(collision_models->getGroupLinkUnion());
      std::vector<collision_space::EnvironmentModel::AllowedContact> allowed_contacts;
      std::vector<collision_space::EnvironmentModel::Contact> contact;
      cps->getCollisionModels()->getCollisionSpace()->getCollisionContacts(allowed_contacts, contact, 10);   
      for(unsigned int i = 0; i < contact.size(); i++) {
        std::string name1;
        std::string name2;
        if(contact[i].link1 != NULL) {
          if(contact[i].link1_attached_body_index != 0) {
            name1 = contact[i].link1->getAttachedBodyModels()[contact[i].link1_attached_body_index-1]->getName();
          } else {
            name1 = contact[i].link1->getName();
          }
        }
        if(contact[i].link2 != NULL) {
          if(contact[i].link2_attached_body_index != 0) {
            name2 = contact[i].link2->getAttachedBodyModels()[contact[i].link2_attached_body_index-1]->getName();
          } else {
            name2 = contact[i].link2->getName();
          }
        } else if (!contact[i].object_name.empty()) {
          name2 = contact[i].object_name;
        }
        ROS_INFO_STREAM("Contact " << i << " between " << name1 << " and " << name2);
      }
      exit(0);
      if(0) {
        std::vector<double> prox_link_distances;
        std::vector<std::vector<double> > prox_distances;
        std::vector<std::vector<btVector3> > prox_gradients;
        std::vector<std::string> prox_link_names;
        std::vector<std::string> prox_attached_body_names;
        cps->getStateGradients(prox_link_names, prox_attached_body_names, prox_link_distances, prox_distances, prox_gradients);
        ROS_INFO_STREAM("Link size " << prox_link_names.size());
        for(unsigned int i = 0; i < prox_link_names.size(); i++) {
          ROS_INFO_STREAM("Link " << prox_link_names[i] << " closest distance " << prox_link_distances[i]);
        }
        for(unsigned int i = 0; i < prox_attached_body_names.size(); i++) {
          ROS_INFO_STREAM("Attached body names " << prox_attached_body_names[i] << " closest distance " << prox_link_distances[i+prox_link_names.size()]);
        }
        exit(0);
      }
    }
    ros::WallDuration ode_dur(n2-n1);
    if(ode_dur > max_ode) {
      max_ode = ode_dur;
    } else if (ode_dur < min_ode) {
      min_ode = ode_dur;
    }
    tot_ode += ode_dur;
    
  }
//ROS_INFO_STREAM("Setup prox " << tot_prox_setup.toSec()/(TEST_NUM*1.0) << " ode " << tot_ode_setup.toSec()/(TEST_NUM*1.0));
  ROS_INFO_STREAM("Av prox time " << (tot_prox.toSec()/(TEST_NUM*1.0)) << " min " << min_prox.toSec() << " max " << max_prox.toSec()
                  << " percent in coll " << (prox_num_in_collision*1.0)/(TEST_NUM*1.0));
  ROS_INFO_STREAM("Av ode time " << (tot_ode.toSec()/(TEST_NUM*1.0)) << " min " << min_ode.toSec() << " max " << max_ode.toSec()
                  << " percent in coll " << (ode_num_in_collision*1.0)/(TEST_NUM*1.0));

  ros::shutdown();

  delete cps;

}
