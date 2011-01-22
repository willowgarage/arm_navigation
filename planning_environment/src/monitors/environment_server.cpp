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

#include "planning_environment/monitors/environment_server_setup.h"
#include "planning_environment/monitors/planning_monitor.h"

#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <motion_planning_msgs/DisplayTrajectory.h>

#include <planning_environment_msgs/GetRobotState.h>
#include <planning_environment_msgs/GetJointTrajectoryValidity.h>
#include <planning_environment_msgs/GetStateValidity.h>
#include <planning_environment_msgs/GetJointsInGroup.h>
#include <planning_environment_msgs/GetGroupInfo.h>
#include <planning_environment_msgs/GetEnvironmentSafety.h>
#include <planning_environment_msgs/ContactInformation.h>
#include <planning_environment_msgs/GetPlanningScene.h>
#include "planning_environment/monitors/environment_server_setup.h"
#include <valarray>
#include <visualization_msgs/Marker.h>

#include <visualization_msgs/MarkerArray.h>

namespace planning_environment
{
class EnvironmentServer
{
public:	
  EnvironmentServer(EnvironmentServerSetup &setup) : 
    private_handle_("~"),
    setup_(setup)
  {
    planning_monitor_ = setup_.planning_monitor_;

    planning_monitor_->waitForState();
    planning_monitor_->startEnvironmentMonitor();
    planning_monitor_->waitForMap();

    tf_ = &setup_.tf_;

    //get_trajectory_validity_service_ = private_handle_.advertiseService("get_trajectory_validity", &EnvironmentServer::getTrajectoryValidity, this);
    //get_env_safety_service_ = private_handle_.advertiseService("get_environment_safety", &EnvironmentServer::getEnvironmentSafety, this);
    //get_execution_safety_service_ = private_handle_.advertiseService("get_execution_safety", &EnvironmentServer::getExecutionSafety, this);
    //get_joints_in_group_service_ = private_handle_.advertiseService("get_joints_in_group", &EnvironmentServer::getJointsInGroup, this);

    //get_group_info_service_ = private_handle_.advertiseService("get_group_info", &EnvironmentServer::getGroupInfo, this);

    get_robot_state_service_ = private_handle_.advertiseService("get_robot_state", &EnvironmentServer::getRobotState, this);
    //get_state_validity_service_ = private_handle_.advertiseService("get_state_validity", &EnvironmentServer::getStateValidity, this);
    //allowed_contact_regions_publisher_ = private_handle_.advertise<visualization_msgs::MarkerArray>("allowed_contact_regions_array", 128);

    //joint_state_subscriber_ =  root_handle_.subscribe("joint_states", 1, &planning_environment::EnvironmentServer::jointStateCallback, this);

    //vis_marker_publisher_ = root_handle_.advertise<visualization_msgs::Marker>("environment_server_contact_markers", 128);
 
    get_planning_scene_service_ = private_handle_.advertiseService("get_planning_scene", &EnvironmentServer::getPlanningScene, this);
	    
    //planning_monitor_->setOnCollisionContactCallback(boost::bind(&EnvironmentServer::contactFound, this, _1));

    ROS_INFO("Environment server started");
  }
	
  virtual ~EnvironmentServer()
  {
  }
	
private:
		
  /** @brief The ccost and display arguments should be bound by the caller. 
   * This is a callback function that gets called by the planning
   * environment when a collision is found */
//   void contactFound(collision_space::EnvironmentModel::Contact &contact)
//   {
//     static int count = 0;

//     std::string ns_name;
//     planning_environment_msgs::ContactInformation contact_info;
//     contact_info.header.frame_id = planning_monitor_->getWorldFrameId();
//     if(contact.link1 != NULL) {
//       //ROS_INFO_STREAM("Link 1 is " << contact.link2->name);
//       if(contact.link1_attached_body_index == 0) {
//         ns_name += contact.link1->getName()+"+";
//         contact_info.contact_body_1 = contact.link1->getName();
//         contact_info.attached_body_1 = "";
//         contact_info.body_type_1 = planning_environment_msgs::ContactInformation::ROBOT_LINK;
//       } else {
//         if(contact.link1->getAttachedBodyModels().size() < contact.link1_attached_body_index) {
//           ROS_ERROR("Link doesn't have attached body with indicated index");
//         } else {
//           ns_name += contact.link1->getAttachedBodyModels()[contact.link1_attached_body_index-1]->getName()+"+";
//           contact_info.contact_body_1 = contact.link1->getName();
//           contact_info.attached_body_1 = contact.link1->getAttachedBodyModels()[contact.link1_attached_body_index-1]->getName();
//           contact_info.body_type_1 = planning_environment_msgs::ContactInformation::ATTACHED_BODY;
//         }
//       }
//     } 
    
//     if(contact.link2 != NULL) {
//       //ROS_INFO_STREAM("Link 2 is " << contact.link2->name);
//       if(contact.link2_attached_body_index == 0) {
//         ns_name += contact.link2->getName();
//         contact_info.contact_body_2 = contact.link2->getName();
//         contact_info.attached_body_2 = "";
//         contact_info.body_type_2 = planning_environment_msgs::ContactInformation::ROBOT_LINK;
//       } else {
//         if(contact.link2->getAttachedBodyModels().size() < contact.link2_attached_body_index) {
//           ROS_ERROR("Link doesn't have attached body with indicated index");
//         } else {
//           ns_name += contact.link2->getAttachedBodyModels()[contact.link2_attached_body_index-1]->getName();
//           contact_info.contact_body_2 = contact.link2->getName();
//           contact_info.attached_body_2 = contact.link2->getAttachedBodyModels()[contact.link2_attached_body_index-1]->getName();
//           contact_info.body_type_2 = planning_environment_msgs::ContactInformation::ATTACHED_BODY;
//         }
//       }
//     } 
    
//     if(!contact.object_name.empty()) {
//       //ROS_INFO_STREAM("Object is " << contact.object_name);
//       ns_name += contact.object_name;
//       contact_info.contact_body_2 = contact.object_name;
//       contact_info.attached_body_2 = "";
//       contact_info.body_type_2 = planning_environment_msgs::ContactInformation::OBJECT;
//     }
    
//     visualization_msgs::Marker mk;
//     mk.header.stamp = planning_monitor_->lastPoseUpdate();
//     mk.header.frame_id = planning_monitor_->getWorldFrameId();
//     mk.ns = ns_name;
//     mk.id = count++;
//     mk.type = visualization_msgs::Marker::SPHERE;
//     mk.action = visualization_msgs::Marker::ADD;
//     mk.pose.position.x = contact.pos.x();
//     mk.pose.position.y = contact.pos.y();
//     mk.pose.position.z = contact.pos.z();
//     mk.pose.orientation.w = 1.0;
    
//     mk.scale.x = mk.scale.y = mk.scale.z = 0.01;
    
//     mk.color.a = 0.6;
//     mk.color.r = 1.0;
//     mk.color.g = 0.8;
//     mk.color.b = 0.04;
    
//     //mk.lifetime = ros::Duration(30.0);
//     contact_info.position.x = contact.pos.x();
//     contact_info.position.y = contact.pos.y();
//     contact_info.position.z = contact.pos.z();
//     contact_info.depth = contact.depth;
//   //   ROS_INFO("Environment server: Found contact");
// //     ROS_INFO("Position                    : (%f,%f,%f)",contact_info.position.x,contact_info.position.y,contact_info.position.z);
// //     ROS_INFO("Frame id                    : %s",contact_info.header.frame_id.c_str());
// //     ROS_INFO("Depth                       : %f",contact_info.depth);
// //     ROS_INFO("Link 1                      : %s",contact_info.contact_body_1.c_str());
// //     ROS_INFO("Link 2                      : %s",contact_info.contact_body_2.c_str());
// //     ROS_INFO(" ");

//     contact_information_.push_back(contact_info);
//     vis_marker_publisher_.publish(mk);
//   }
	     
//   bool getEnvironmentSafety(planning_environment_msgs::GetEnvironmentSafety::Request &req, 
//                             planning_environment_msgs::GetEnvironmentSafety::Response &res)
//   {
//     planning_monitor_->isEnvironmentSafe(res.error_code);
//     return true;
//   }

  bool getRobotState(planning_environment_msgs::GetRobotState::Request &req, 
                     planning_environment_msgs::GetRobotState::Response &res)
  {
    planning_monitor_->getCurrentRobotState(res.robot_state);
    return true;    
  }

//   bool getJointsInGroup(planning_environment_msgs::GetJointsInGroup::Request &req, 
//                         planning_environment_msgs::GetJointsInGroup::Response &res)
//   {
//     ROS_DEBUG("Looking for joints in group %s",req.group_name.c_str());
//     const planning_models::KinematicModel::JointModelGroup *jg = planning_monitor_->getKinematicModel()->getModelGroup(req.group_name);
//     if(jg)
//     {
//       res.joint_names = jg->getJointModelNames();
//       res.error_code.val = res.error_code.SUCCESS;
//       return true;
//     }
//     res.error_code.val = res.error_code.INVALID_GROUP_NAME;
//     return true;
//   }

//   bool getGroupInfo(planning_environment_msgs::GetGroupInfo::Request &req, 
//                     planning_environment_msgs::GetGroupInfo::Response &res)
//   {
//     if(req.group_name.empty()) {
//       ROS_DEBUG_STREAM("Getting info for all groups");
//     } else {
//       ROS_DEBUG_STREAM("Getting info for group name " << req.group_name);
//     }
//     if(req.group_name.empty()) {
//       res.joint_names = setup_.collision_models_->getGroupJointUnion();
//       res.link_names = setup_.collision_models_->getGroupLinkUnion();
//       res.error_code.val = res.error_code.SUCCESS;
//     } else {
//       const std::map< std::string, std::vector<std::string> > planning_group_joints = setup_.collision_models_->getPlanningGroupJoints();
//       const std::map< std::string, std::vector<std::string> > planning_group_links = setup_.collision_models_->getPlanningGroupLinks();
//       if(planning_group_joints.find(req.group_name) == planning_group_joints.end() ||
//          planning_group_links.find(req.group_name) == planning_group_links.end()) {
//         res.error_code.val = res.error_code.INVALID_GROUP_NAME;
//       } else {
//         res.joint_names = planning_group_joints.find(req.group_name)->second;
//         res.link_names = planning_group_links.find(req.group_name)->second;
//         res.error_code.val = res.error_code.SUCCESS;
//       }
//     }
//     return true;
//   }

//   bool getTrajectoryValidity(planning_environment_msgs::GetJointTrajectoryValidity::Request &req, 
//                              planning_environment_msgs::GetJointTrajectoryValidity::Response &res) 
//   {
//     contact_information_.clear();
//     planning_monitor_->prepareForValidityChecks(req.trajectory.joint_names,
//                                                 req.ordered_collision_operations,
//                                                 req.allowed_contacts,
//                                                 req.path_constraints,
//                                                 req.goal_constraints,
//                                                 req.link_padding,
//                                                 res.error_code);
//     if(res.error_code.val != res.error_code.SUCCESS)
//     {
//       ROS_ERROR("Could not prepare planning_environment");
//       return true;;
//     }
//     int flag = getCheckFlag(req);
//     if(planning_monitor_->isTrajectoryValid(req.trajectory,
//                                             req.robot_state,
//                                             (unsigned int) 0, 
//                                             (unsigned int) req.trajectory.points.size()-1, 
//                                             flag,
//                                             true, 
//                                             res.error_code,
//                                             res.trajectory_error_codes))
//     {
//       res.error_code.val = res.error_code.SUCCESS;
//       ROS_DEBUG("Trajectory is valid");
//     }
//     res.contacts = contact_information_;
//     planning_monitor_->revertToDefaultState();
//     return true;
//   }

  bool getPlanningScene(planning_environment_msgs::GetPlanningScene::Request &req, 
                        planning_environment_msgs::GetPlanningScene::Response &res) 
  {
    planning_monitor_->getCompletePlanningScene(req.group_name,
                                                req.robot_state_diff,
                                                req.goal_constraints,
                                                req.path_constraints,
                                                req.allowed_contacts,
                                                req.ordered_collision_operations,
                                                req.link_padding_diffs,
                                                req.collision_object_diffs,
                                                req.attached_collision_object_diffs,
                                                res.complete_robot_state,
                                                res.transformed_goal_constraints,
                                                res.transformed_path_constraints,
                                                res.allowed_collision_matrix,
                                                res.transformed_allowed_contacts,
                                                res.all_link_padding,
                                                res.all_collision_objects,
                                                res.all_attached_collision_objects,
                                                res.unmasked_collision_map);
    return true;
  }

  // bool getStateValidity(planning_environment_msgs::GetStateValidity::Request &req, 
  //                       planning_environment_msgs::GetStateValidity::Response &res) 
  // {
  //   contact_information_.clear();
  //   std::vector<std::string> names = req.robot_state.joint_state.name;
  //   if(!req.robot_state.multi_dof_joint_state.joint_names.empty()) {
  //     names.insert(names.end(),req.robot_state.multi_dof_joint_state.joint_names.begin(),
  //                  req.robot_state.multi_dof_joint_state.joint_names.end());
  //   }
  //   if(names.empty()) {
  //     planning_monitor_->getKinematicModel()->getJointModelNames(names);
  //   } 
  //   planning_monitor_->prepareForValidityChecks(names,
  //                                               req.ordered_collision_operations,
  //                                               req.allowed_contacts,
  //                                               req.path_constraints,
  //                                               req.goal_constraints,
  //                                               req.link_padding,
  //                                               res.error_code);
  //   if(res.error_code.val != res.error_code.SUCCESS)
  //   {
  //     ROS_ERROR("Could not prepare planning environment");
  //     return true;
  //   }
  //   int flag = getCheckFlag(req);
  //   if(planning_monitor_->isStateValid(req.robot_state,flag,true,res.error_code))
  //   {
  //     res.error_code.val = res.error_code.SUCCESS;
  //   }
  //   res.contacts = contact_information_;
  //   //    visualizeAllowedContactRegions(req.allowed_contacts);
  //   planning_monitor_->revertToDefaultState();
  //   return true;
  // }

  // bool getExecutionSafety(planning_environment_msgs::GetJointTrajectoryValidity::Request &req, 
  //                         planning_environment_msgs::GetJointTrajectoryValidity::Response &res)
  // {
  //   ros::Time start = ros::Time::now();
  //   contact_information_.clear();
  //   planning_monitor_->prepareForValidityChecks(req.trajectory.joint_names,
  //                                               req.ordered_collision_operations,
  //                                               req.allowed_contacts,
  //                                               req.path_constraints,
  //                                               req.goal_constraints,
  //                                               req.link_padding,
  //                                               res.error_code);
  //   if(res.error_code.val != res.error_code.SUCCESS)
  //   {
  //     ROS_ERROR("Could not prepare planning monitor to check validity");
  //     return true;
  //   }
  //   if(!planning_monitor_->isEnvironmentSafe(res.error_code))
  //     return true;

  //   int current_position_index(0);        
  //   // we don't want to check the part of the path that was already executed
  //   current_position_index = planning_monitor_->closestStateOnTrajectory(req.trajectory,
  //                                                                        req.robot_state,
  //                                                                        current_position_index, 
  //                                                                        req.trajectory.points.size() - 1, 
  //                                                                        res.error_code);
  //   if (current_position_index < 0)
  //   {
  //     ROS_WARN("Unable to identify current state in path");
  //     current_position_index = 0;
  //   }
		    
  //   int flag = getCheckFlag(req);
  //   bool valid = false;
  //   valid = planning_monitor_->isTrajectoryValid(req.trajectory,
  //                                                req.robot_state,
  //                                                current_position_index, 
  //                                                req.trajectory.points.size() - 1, 
  //                                                flag,
  //                                                false,
  //                                                res.error_code,res.trajectory_error_codes);

  //   //if(velocity_history_index_ >= velocity_history_.size())
  //   //{		
  //   //  double sum = velocity_history_.sum();
  //   //  if (sum < 1e-3)
  //   //  {
  //   //    ROS_INFO("The total velocity of the robot over the last %d samples is %f. Self-preempting...", (int)velocity_history_.size(), sum);
  //   //    res.error_code.val = res.error_code.JOINTS_NOT_MOVING;
  //   //  }
  //   //}
  //   res.contacts = contact_information_;
  //   planning_monitor_->revertToDefaultState();
  //   return true;
  // }

  // int getCheckFlag(const planning_environment_msgs::GetJointTrajectoryValidity::Request &req)
  // {
  //   int result(0);
  //   if(req.check_collisions)
  //     result = result | planning_environment::PlanningMonitor::COLLISION_TEST;
  //   if(req.check_path_constraints)
  //     result = result | planning_environment::PlanningMonitor::PATH_CONSTRAINTS_TEST;
  //   if(req.check_goal_constraints)
  //     result = result | planning_environment::PlanningMonitor::GOAL_CONSTRAINTS_TEST;
  //   if(req.check_joint_limits)
  //     result = result | planning_environment::PlanningMonitor::JOINT_LIMITS_TEST;
  //   if(req.check_full_trajectory)
  //     result = result | planning_environment::PlanningMonitor::CHECK_FULL_TRAJECTORY;
  //   return result;
  // }

  // int getCheckFlag(const planning_environment_msgs::GetStateValidity::Request &req)
  // {
  //   int result(0);
  //   if(req.check_collisions)
  //     result = result | planning_environment::PlanningMonitor::COLLISION_TEST;
  //   if(req.check_path_constraints)
  //     result = result | planning_environment::PlanningMonitor::PATH_CONSTRAINTS_TEST;
  //   if(req.check_goal_constraints)
  //     result = result | planning_environment::PlanningMonitor::GOAL_CONSTRAINTS_TEST;
  //   if(req.check_joint_limits)
  //     result = result | planning_environment::PlanningMonitor::JOINT_LIMITS_TEST;
  //   return result;
  // }

  // void jointStateCallback(const sensor_msgs::JointStateConstPtr &joint_state)
  // {
  //   velocity_history_[velocity_history_index_ % velocity_history_.size()] = planning_monitor_->getTotalVelocity();
  //   velocity_history_index_++;
  // }  
  
  // void visualizeAllowedContactRegions(const std::vector<motion_planning_msgs::AllowedContactSpecification> &allowed_contacts)
  // {
  //   static int count = 0;
  //   visualization_msgs::MarkerArray mk;
  //   mk.markers.resize(allowed_contacts.size());
  //   for(unsigned int i=0; i < allowed_contacts.size(); i++) 
  //   { 
  //     bool valid_shape = true;
  //     mk.markers[i].header.stamp = ros::Time::now();
  //     mk.markers[i].header.frame_id = allowed_contacts[i].pose_stamped.header.frame_id;
  //     mk.markers[i].ns = allowed_contacts[i].name;
  //     mk.markers[i].id = count++;
  //     if(allowed_contacts[i].shape.type == geometric_shapes_msgs::Shape::SPHERE)
  //     {        
  //       mk.markers[i].type = visualization_msgs::Marker::SPHERE;
  //       if(allowed_contacts[i].shape.dimensions.size() >= 1)
  //         mk.markers[i].scale.x = mk.markers[i].scale.y = mk.markers[i].scale.z = allowed_contacts[i].shape.dimensions[0];
  //       else
  //         valid_shape = false;
  //     }      
  //     else if (allowed_contacts[i].shape.type == geometric_shapes_msgs::Shape::BOX)
  //     {
  //       mk.markers[i].type = visualization_msgs::Marker::CUBE;
  //       if(allowed_contacts[i].shape.dimensions.size() >= 3)
  //       {
  //         mk.markers[i].scale.x = allowed_contacts[i].shape.dimensions[0];
  //         mk.markers[i].scale.y = allowed_contacts[i].shape.dimensions[1];
  //         mk.markers[i].scale.z = allowed_contacts[i].shape.dimensions[2];
  //       }
  //       else
  //         valid_shape = false;
  //     }
  //     else if (allowed_contacts[i].shape.type == geometric_shapes_msgs::Shape::CYLINDER)
  //     {
  //       mk.markers[i].type = visualization_msgs::Marker::CYLINDER;
  //       if(allowed_contacts[i].shape.dimensions.size() >= 2)
  //       {
  //         mk.markers[i].scale.x = allowed_contacts[i].shape.dimensions[0];
  //         mk.markers[i].scale.y = allowed_contacts[i].shape.dimensions[0];
  //         mk.markers[i].scale.z = allowed_contacts[i].shape.dimensions[1];
  //       }
  //       else
  //         valid_shape = false;
  //     }
  //     else
  //     {
  //       mk.markers[i].scale.x = mk.markers[i].scale.y = mk.markers[i].scale.z = 0.01;
  //       valid_shape = false;
  //     }        

  //     mk.markers[i].action = visualization_msgs::Marker::ADD;
  //     mk.markers[i].pose = allowed_contacts[i].pose_stamped.pose;  
  //     if(!valid_shape)
  //     {
  //       mk.markers[i].scale.x = mk.markers[i].scale.y = mk.markers[i].scale.z = 0.01;
  //       mk.markers[i].color.a = 0.5;
  //       mk.markers[i].color.r = 1.0;
  //       mk.markers[i].color.g = 0.04;
  //       mk.markers[i].color.b = 0.04;
  //     }
  //     else
  //     {
  //       mk.markers[i].color.a = 0.5;
  //       mk.markers[i].color.r = 0.04;
  //       mk.markers[i].color.g = 1.0;
  //       mk.markers[i].color.b = 0.04;
  //     }  
  //     //mk.markers[i].lifetime = ros::Duration(30.0);  
  //   }
  //   allowed_contact_regions_publisher_.publish(mk);
  // }

private:
	
  ros::NodeHandle root_handle_, private_handle_;
  EnvironmentServerSetup &setup_;
  planning_environment::PlanningMonitor *planning_monitor_;
  tf::TransformListener *tf_;	

  bool show_collisions_;
  bool allow_valid_collisions_;	
  std::vector<collision_space::EnvironmentModel::AllowedContact> allowed_contacts_;

  ros::Publisher vis_marker_publisher_;
  ros::ServiceServer get_trajectory_validity_service_;
  ros::ServiceServer get_env_safety_service_;
  ros::ServiceServer get_execution_safety_service_;
  ros::ServiceServer set_constraints_service_;
  ros::ServiceServer get_joints_in_group_service_;
  ros::ServiceServer get_group_info_service_;
  ros::ServiceServer get_robot_state_service_;
  ros::ServiceServer get_state_validity_service_;
  ros::Publisher allowed_contact_regions_publisher_;

  ros::ServiceServer get_planning_scene_service_;

  ros::Subscriber joint_state_subscriber_;

  std::vector<planning_environment_msgs::ContactInformation> contact_information_;
};    
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "environment_monitor");
  planning_environment::EnvironmentServerSetup setup;
  ros::AsyncSpinner spinner(4); // Use 2 threads
  spinner.start();
  if (!setup.configure())
  {
    ros::shutdown();
    return 0;
  } 
  planning_environment::EnvironmentServer environment_monitor(setup);
  ros::waitForShutdown();
  return 0;
}
