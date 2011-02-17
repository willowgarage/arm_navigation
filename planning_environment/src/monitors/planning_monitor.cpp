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

/** \author Ioan Sucan, Sachin Chitta */

#include "planning_environment/monitors/planning_monitor.h"
//#include "planning_environment/util/kinematic_state_constraint_evaluator.h"
#include <boost/scoped_ptr.hpp>
//#include <motion_planning_msgs/DisplayTrajectory.h>
#include "planning_environment/models/model_utils.h"

void planning_environment::PlanningMonitor::loadParams(void)
{
  nh_.param<double>("collision_map_safety_timeout", intervalCollisionMap_, 0.0);
  nh_.param<double>("joint_states_safety_timeout", intervalState_, 0.0);
  nh_.param<double>("tf_safety_timeout", intervalPose_, 0.0);
  nh_.param<int>("contacts_to_compute_for_allowable_contacts_test", num_contacts_allowable_contacts_test_, 10);
  nh_.param<int>("contacts_to_compute_for_display", num_contacts_for_display_, 1);

  //display_collision_pose_publisher_ = nh_.advertise<motion_planning_msgs::DisplayTrajectory>("collision_pose", 1);
  //display_state_validity_publisher_ = nh_.advertise<motion_planning_msgs::DisplayTrajectory>("state_validity", 1);
}


bool planning_environment::PlanningMonitor::getCompletePlanningScene(const std::string& group_name,
                                                                     const planning_environment_msgs::PlanningScene& planning_diff,
                                                                     const motion_planning_msgs::OrderedCollisionOperations& ordered_collision_operations,
                                                                     const motion_planning_msgs::Constraints& goal_constraints,
                                                                     const motion_planning_msgs::Constraints& path_constraints,
                                                                     planning_environment_msgs::PlanningScene& planning_scene,
                                                                     motion_planning_msgs::Constraints& transformed_goal_constraints,
                                                                     motion_planning_msgs::Constraints& transformed_path_constraints)
{
  //creating state    
  planning_models::KinematicState set_state(getKinematicModel());    
  //setting state to current values
  setStateValuesFromCurrentValues(set_state);
  //supplementing with state_diff
  setRobotStateAndComputeTransforms(planning_diff.robot_state, set_state);
  
  //now complete robot state is populated
  convertKinematicStateToRobotState(set_state,
                                    last_joint_state_update_,
                                    cm_->getWorldFrameId(),
                                    planning_scene.robot_state);
  
  //this transform 
  btTransform set_world_transform = set_state.getRootTransform();
  
  transformed_goal_constraints = goal_constraints;
  convertConstraintsGivenNewWorldTransform(set_state,
                                           transformed_goal_constraints);
  transformed_path_constraints = path_constraints;
  convertConstraintsGivenNewWorldTransform(set_state,
                                           transformed_path_constraints);

  collision_space::EnvironmentModel::AllowedCollisionMatrix acm = cm_->getCollisionSpace()->getDefaultAllowedCollisionMatrix();
  
  //first we deal with collision object diffs
  cm_->getCollisionSpaceCollisionObjects(planning_scene.collision_objects);
  for(unsigned int i = 0; i < planning_diff.collision_objects.size(); i++) {
    std::string object_name = planning_diff.collision_objects[i].id;
    if(object_name == "all") {
      if(planning_diff.collision_objects[i].operation.operation == mapping_msgs::CollisionObjectOperation::REMOVE) {
        for(unsigned int j = 0; j < planning_scene.collision_objects.size(); i++) {
          acm.removeEntry(planning_scene.collision_objects[j].id);
        }
        planning_scene.collision_objects.clear();
        continue;
      } else {
        ROS_WARN_STREAM("No other operation than remove permitted for all");
        return false;
      }
    }
    bool already_have = false;
    std::vector<mapping_msgs::CollisionObject>::iterator it = planning_scene.collision_objects.begin();
    while(it != planning_scene.collision_objects.end()) {
      if((*it).id == object_name) {
        already_have = true;
        break;
      }
      it++;
    }
    if(planning_diff.collision_objects[i].operation.operation == mapping_msgs::CollisionObjectOperation::REMOVE) {
      if(!already_have) {
        ROS_WARN_STREAM("Diff remove specified for object " << object_name << " which we don't seem to have");
        continue;
      }
      acm.removeEntry(object_name);
      planning_scene.collision_objects.erase(it);
    } else {
      //must be an add
      if(already_have) {
        //if we already have it don't need to add to the matrix
        planning_scene.collision_objects.erase(it);
      } else {
        acm.addEntry(object_name, false);
      }
      planning_scene.collision_objects.push_back(planning_diff.collision_objects[i]);
      convertCollisionObjectToNewWorldFrame(set_state,
                                            planning_scene.collision_objects.back());
    }
  }
  
  //now attached objects
  cm_->getCollisionSpaceAttachedCollisionObjects(planning_scene.attached_collision_objects);
  for(unsigned int i = 0; i < planning_diff.attached_collision_objects.size(); i++) {
    std::string link_name = planning_diff.attached_collision_objects[i].link_name;
    std::string object_name = planning_diff.attached_collision_objects[i].object.id;
    if(planning_diff.attached_collision_objects[i].object.operation.operation == mapping_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT ||
       planning_diff.attached_collision_objects[i].object.operation.operation == mapping_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT) {
      ROS_WARN_STREAM("Object replacement not supported during diff");
    }
    if(link_name == "all") {
      if(planning_diff.attached_collision_objects[i].object.operation.operation == mapping_msgs::CollisionObjectOperation::REMOVE) {
        for(unsigned int j = 0; j < planning_scene.attached_collision_objects.size(); i++) {
          acm.removeEntry(planning_scene.attached_collision_objects[j].object.id);
        }
        planning_scene.attached_collision_objects.clear();
        continue;
      } else {
        ROS_WARN_STREAM("No other operation than remove permitted for all");
        return false;        
      }
    } else {
      if(object_name == "all") {
        if(planning_diff.attached_collision_objects[i].object.operation.operation == mapping_msgs::CollisionObjectOperation::REMOVE) {
          std::vector<mapping_msgs::AttachedCollisionObject>::iterator it = planning_scene.attached_collision_objects.begin();
          while(it != planning_scene.attached_collision_objects.end()) {
            if((*it).link_name == link_name) {
              acm.removeEntry((*it).object.id);
              it == planning_scene.attached_collision_objects.erase(it);
            } else {
              it++;
            }
          }
        } else {
          ROS_WARN_STREAM("No other operation than remove permitted for all");
          return false;
        }
        continue;
      }
      bool already_have = false;
      std::vector<mapping_msgs::AttachedCollisionObject>::iterator it = planning_scene.attached_collision_objects.begin();
      while(it != planning_scene.attached_collision_objects.end()) {
        if((*it).link_name == link_name) {
          if((*it).object.id == object_name) {
            already_have = true;
            break;
          }
        }
        it++;
      }
      if(planning_diff.attached_collision_objects[i].object.operation.operation == mapping_msgs::CollisionObjectOperation::REMOVE) {
        if(!already_have) {
          ROS_WARN_STREAM("Diff remove specified for object " << object_name << " which we don't seem to have");
          return false;
        }
        acm.removeEntry((*it).object.id);
        planning_scene.attached_collision_objects.erase(it);
      } else {
        //must be an add
        if(already_have) {
          planning_scene.attached_collision_objects.erase(it);
        } else {
          acm.addEntry(planning_diff.attached_collision_objects[i].object.id, false);
        }
        acm.changeEntry(planning_diff.attached_collision_objects[i].object.id, planning_diff.attached_collision_objects[i].link_name, true);
        for(unsigned int j = 0; j < planning_diff.attached_collision_objects[i].touch_links.size(); j++) {
          acm.changeEntry(planning_diff.attached_collision_objects[i].object.id, planning_diff.attached_collision_objects[i].touch_links[j], true);
        }
        planning_scene.attached_collision_objects.push_back(planning_diff.attached_collision_objects[i]);
        convertAttachedCollisionObjectToNewWorldFrame(set_state,
                                                      planning_scene.attached_collision_objects.back());
      }
    }
  }

  std::map<std::string, double> cur_link_padding = cm_->getCollisionSpace()->getCurrentLinkPaddingMap();
  for(unsigned int i = 0; i < planning_diff.link_padding.size(); i++) {
    //note - for things like attached objects this might just add them to the mix, but that's fine
    cur_link_padding[planning_diff.link_padding[i].link_name] = planning_diff.link_padding[i].padding;
  }
  convertFromLinkPaddingMapToLinkPaddingVector(cur_link_padding, planning_scene.link_padding);

  //now we need to apply the allowed collision operations to the modified ACM
  std::vector<std::string> o_strings;
  for(unsigned int i = 0; i < planning_scene.collision_objects.size(); i++) {
    o_strings.push_back(planning_scene.collision_objects[i].id);
  }

  std::vector<std::string> a_strings;
  for(unsigned int i = 0; i < planning_scene.attached_collision_objects.size(); i++) {
    a_strings.push_back(planning_scene.attached_collision_objects[i].object.id);
  }

  applyOrderedCollisionOperationsListToACM(ordered_collision_operations,
                                           o_strings,
                                           a_strings,
                                           cm_->getKinematicModel(),
                                           acm);
  
  convertFromACMToACMMsg(acm, planning_scene.allowed_collision_matrix);

  //NOTE - this should be unmasked in collision_space_monitor;
  cm_->getCollisionSpaceCollisionMap(planning_scene.collision_map);
  return true;
}   

void planning_environment::PlanningMonitor::convertAttachedCollisionObjectToNewWorldFrame(const planning_models::KinematicState& state,
                                                                                          mapping_msgs::AttachedCollisionObject& att_obj) const
{
  for(unsigned int i = 0; i < att_obj.object.poses.size(); i++) {
    geometry_msgs::PoseStamped ret_pose = convertPoseGivenWorldTransform(state,
                                                                         att_obj.link_name,
                                                                         att_obj.object.header,
                                                                         att_obj.object.poses[i]);
    if(i == 0) {
      att_obj.object.header = ret_pose.header;
    }
    att_obj.object.poses[i] = ret_pose.pose;
  }
}

void planning_environment::PlanningMonitor::convertCollisionObjectToNewWorldFrame(const planning_models::KinematicState& state,
                                                                                  mapping_msgs::CollisionObject& obj) const
{
  for(unsigned int i = 0; i < obj.poses.size(); i++) {
    geometry_msgs::PoseStamped ret_pose = convertPoseGivenWorldTransform(state,
                                                                         cm_->getWorldFrameId(),
                                                                         obj.header,
                                                                         obj.poses[i]);
    if(i == 0) {
      obj.header = ret_pose.header;
    }
    obj.poses[i] = ret_pose.pose;
  }
}

void planning_environment::PlanningMonitor::convertConstraintsGivenNewWorldTransform(const planning_models::KinematicState& state,
                                                                                     motion_planning_msgs::Constraints& constraints) const {
  std::string fixed_frame = cm_->getWorldFrameId();
  for(unsigned int i = 0; i < constraints.position_constraints.size(); i++) {
    geometry_msgs::PointStamped ps = convertPointGivenWorldTransform(state,
                                                                     fixed_frame,
                                                                     constraints.position_constraints[i].header,
                                                                     constraints.position_constraints[i].target_point_offset);
    constraints.position_constraints[i].header = ps.header;
    constraints.position_constraints[i].target_point_offset = ps.point;

    ps = convertPointGivenWorldTransform(state,
                                         fixed_frame,
                                         constraints.position_constraints[i].header,
                                         constraints.position_constraints[i].position);
    constraints.position_constraints[i].position = ps.point;

    geometry_msgs::QuaternionStamped qs = convertQuaternionGivenWorldTransform(state,
                                                                               fixed_frame,                                     
                                                                               constraints.position_constraints[i].header,
                                                                               constraints.position_constraints[i].constraint_region_orientation);
    constraints.position_constraints[i].constraint_region_orientation = qs.quaternion; 
  }
  
  for(unsigned int i = 0; i < constraints.orientation_constraints.size(); i++) {
    geometry_msgs::QuaternionStamped qs = convertQuaternionGivenWorldTransform(state,
                                                                               fixed_frame,
                                                                               constraints.orientation_constraints[i].header,
                                                                               constraints.orientation_constraints[i].orientation);
    constraints.orientation_constraints[i].header = qs.header;
    constraints.orientation_constraints[i].orientation = qs.quaternion; 
  }

  for(unsigned int i = 0; i < constraints.visibility_constraints.size(); i++) {
    constraints.visibility_constraints[i].target = convertPointGivenWorldTransform(state,
                                                                                   fixed_frame,
                                                                                   constraints.visibility_constraints[i].target.header,
                                                                                   constraints.visibility_constraints[i].target.point);
  }  
}

geometry_msgs::PoseStamped 
planning_environment::PlanningMonitor::convertPoseGivenWorldTransform(const planning_models::KinematicState& state,
                                                                      const std::string& des_frame_id,
                                                                      const std_msgs::Header& header,
                                                                      const geometry_msgs::Pose& pose) const
{
  geometry_msgs::PoseStamped ret_pose;
  ret_pose.header = header;
  ret_pose.pose = pose;

  bool header_is_fixed_frame = (header.frame_id == cm_->getWorldFrameId());
  bool des_is_fixed_frame = (des_frame_id == cm_->getWorldFrameId());

  //Scenario 1(fixed->fixed): if pose is in the world frame and
  //desired is in the world frame, just return
  if(header_is_fixed_frame && des_is_fixed_frame) {
    return ret_pose;
  }
  const planning_models::KinematicState::LinkState* header_link_state = state.getLinkState(header.frame_id);
  const planning_models::KinematicState::LinkState* des_link_state = state.getLinkState(des_frame_id);
  
  bool header_is_robot_frame = (header_link_state != NULL);
  bool des_is_robot_frame = (des_link_state != NULL);

  bool header_is_other_frame = !header_is_fixed_frame && !header_is_robot_frame;
  bool des_is_other_frame = !des_is_fixed_frame && !des_is_robot_frame;
  
  //Scenario 2(*-> other): We can't deal with desired being in a
  //non-fixed frame or relative to the robot
  if(des_is_other_frame) {
    ROS_WARN_STREAM("Shouldn't be transforming into non-fixed non-robot frame " << des_frame_id);
    return ret_pose;
  }

  //Scenario 3 (other->fixed) && 4 (other->robot): we first need to
  //transform into the fixed frame
  if(header_is_other_frame) {
    ros::Time tm;
    std::string err_string;
    if(tf_->canTransform(cm_->getWorldFrameId(), header.frame_id, header.stamp)) {
      tm = header.stamp;
    } else if (tf_->getLatestCommonTime(cm_->getWorldFrameId(), header.frame_id, tm, &err_string) != tf::NO_ERROR) {
      ROS_WARN_STREAM("No transform whatsoever available between " << des_frame_id << " and " << header.frame_id);
      return ret_pose;
    }
    ret_pose.header.stamp = tm;
    geometry_msgs::PoseStamped trans_pose;
    try {
      tf_->transformPose(cm_->getWorldFrameId(), ret_pose, trans_pose);
    } catch(tf::TransformException& ex) {
      ROS_ERROR_STREAM("Unable to transform object from frame " << header.frame_id << " to " << cm_->getWorldFrameId() << " tf error is " << ex.what());
      return ret_pose;
    }
    ret_pose = trans_pose;
    //Scenario 3 (other->fixed): The tf lookup for current state
    //accomplises everything if the desired is the odom combined frame
    if(des_is_fixed_frame) {
      return ret_pose;
    }
  }
  
  //getting tf version of pose
  btTransform bt_pose;
  tf::poseMsgToTF(ret_pose.pose,bt_pose);

  //Scenarios 4(other->robot)/5(fixed->robot): We either started out
  //with a header frame in the fixed frame or converted from a
  //non-robot frame into the fixed frame, and now we need to transform
  //into the desired robot frame given the new world transform
  if(ret_pose.header.frame_id == "odom_combined" && des_is_robot_frame) {
    btTransform trans_bt_pose = des_link_state->getGlobalLinkTransform().inverse()*bt_pose;
    tf::poseTFToMsg(trans_bt_pose,ret_pose.pose);
    ret_pose.header.frame_id = des_link_state->getName();
  } else if(header_is_robot_frame && des_is_fixed_frame) {
    //Scenario 6(robot->fixed): Just need to look up robot transform and pre-multiply
    btTransform trans_bt_pose = header_link_state->getGlobalLinkTransform()*bt_pose;
    tf::poseTFToMsg(trans_bt_pose,ret_pose.pose);
    ret_pose.header.frame_id = cm_->getWorldFrameId();
  } else if(header_is_robot_frame && des_is_robot_frame) {
    //Scenario 7(robot->robot): Completely tf independent
    btTransform trans_bt_pose = des_link_state->getGlobalLinkTransform().inverse()*(header_link_state->getGlobalLinkTransform()*bt_pose);
    tf::poseTFToMsg(trans_bt_pose,ret_pose.pose);
    ret_pose.header.frame_id = des_link_state->getName();
  } else {
    ROS_WARN("Really shouldn't have gotten here");
  }
  return ret_pose;
}

geometry_msgs::PointStamped 
planning_environment::PlanningMonitor::convertPointGivenWorldTransform(const planning_models::KinematicState& state,
                                                                       const std::string& des_frame_id,
                                                                       const std_msgs::Header& header,
                                                                       const geometry_msgs::Point& point) const
{
  geometry_msgs::Pose arg_pose;
  arg_pose.position = point;
  arg_pose.orientation.w = 1.0;
  geometry_msgs::PoseStamped ret_pose = convertPoseGivenWorldTransform(state, 
                                                                       des_frame_id,
                                                                       header,
                                                                       arg_pose);
  geometry_msgs::PointStamped ret_point;
  ret_point.header = ret_pose.header;
  ret_point.point = ret_pose.pose.position;
  return ret_point;
}

geometry_msgs::QuaternionStamped 
planning_environment::PlanningMonitor::convertQuaternionGivenWorldTransform(const planning_models::KinematicState& state,
                                                                            const std::string& des_frame_id,
                                                                            const std_msgs::Header& header,
                                                                            const geometry_msgs::Quaternion& quat) const
{
  geometry_msgs::Pose arg_pose;
  arg_pose.orientation = quat;
  geometry_msgs::PoseStamped ret_pose = convertPoseGivenWorldTransform(state, 
                                                                       des_frame_id,
                                                                       header,
                                                                       arg_pose);
  geometry_msgs::QuaternionStamped ret_quat;
  ret_quat.header = ret_pose.header;
  ret_quat.quaternion = ret_pose.pose.orientation;
  return ret_quat;
}

// bool planning_environment::PlanningMonitor::isEnvironmentSafe(motion_planning_msgs::ArmNavigationErrorCodes &error_code) const
// {

//   if (use_collision_map_ && (!haveMap_ || !isMapUpdated(intervalCollisionMap_)))
//   {
//     ROS_WARN("Environment is not safe for motion: Collision map not updated in the last %f seconds", intervalCollisionMap_);
//     error_code.val = error_code.SENSOR_INFO_STALE;
//     return false;
//   }
  
//   if (!isJointStateUpdated(intervalState_))
//   {
//     ROS_WARN("Environment is not safe for motion: Robot state not updated in the last %f seconds", intervalState_);
//     error_code.val = error_code.ROBOT_STATE_STALE;
//     return false;
//   }  

//   if (!isPoseUpdated(intervalPose_))
//   {
//     ROS_WARN("Environment is not safe for motion: Robot pose not updated in the last %f seconds", intervalPose_);
//     error_code.val = error_code.FRAME_TRANSFORM_FAILURE;
//     return false;
//   }
  
//   error_code.val = error_code.SUCCESS;
//   return true;
// }

// int planning_environment::PlanningMonitor::closestStateOnTrajectory(const trajectory_msgs::JointTrajectory &trajectory, 
//                                                                     motion_planning_msgs::RobotState &robot_state, 
//                                                                     motion_planning_msgs::ArmNavigationErrorCodes &error_code) const
// {
//   return closestStateOnTrajectory(trajectory, robot_state, 0, trajectory.points.size() - 1, error_code);
// }

// int planning_environment::PlanningMonitor::closestStateOnTrajectory(const trajectory_msgs::JointTrajectory &trajectory, 
//                                                                     motion_planning_msgs::RobotState &robot_state, 
//                                                                     unsigned int start, 
//                                                                     unsigned int end, 
//                                                                     motion_planning_msgs::ArmNavigationErrorCodes &error_code) const
// {
//   if (end >= trajectory.points.size())
//     end = trajectory.points.size() - 1;
//   if (start > end)
//   {
//     ROS_ERROR("Invalid start %d and end %d specification",start,end);
//     error_code.val = error_code.INVALID_INDEX;
//     return -1;
//   }

//   if (trajectory.header.frame_id != cm_->getWorldFrameId())
//   {
//     trajectory_msgs::JointTrajectory pathT = trajectory;
//     if (transformTrajectoryToFrame(pathT, robot_state, cm_->getWorldFrameId(), error_code))
//       return closestStateOnTrajectoryAux(pathT, start, end, error_code);
//     else
//     {
//       ROS_ERROR("Could not transform trajectory from %s to %s",trajectory.header.frame_id.c_str(),cm_->getWorldFrameId().c_str());
//       error_code.val = error_code.FRAME_TRANSFORM_FAILURE;
//       return -1;
//     }
//   }
//   else
//     return closestStateOnTrajectoryAux(trajectory, start, end, error_code);  
// }

// int planning_environment::PlanningMonitor::closestStateOnTrajectoryAux(const trajectory_msgs::JointTrajectory &trajectory, 
//                                                                        unsigned int start, 
//                                                                        unsigned int end, 
//                                                                        motion_planning_msgs::ArmNavigationErrorCodes &error_code) const
// {
//   double dist = 0.0;
//   int    pos  = -1;

//   std::map<std::string, double> current_joint_vals = getCurrentJointStateValues();

//   for(unsigned int i = 0; i < trajectory.joint_names.size(); i++) {
//     if(current_joint_vals.find(trajectory.joint_names[i]) == current_joint_vals.end()) {
//       ROS_ERROR("Unknown joint '%s' found on path", trajectory.joint_names[i].c_str());
//       error_code.val = error_code.INVALID_TRAJECTORY;
//       return -1;
//     }
//   }

//   for (unsigned int i = start ; i <= end ; ++i)
//   {
//     double d = 0.0;
//     for (unsigned int j = 0 ; j < trajectory.joint_names.size(); ++j)
//     {
//       double current_joint_position = current_joint_vals.find(trajectory.joint_names[j])->second;
//       double diff = fabs(trajectory.points[i].positions[j] - current_joint_position);
//       d += diff * diff;
//     }
	
//     if (pos < 0 || d < dist)
//     {
//       pos = i;
//       dist = d;
//     }
//   }    
//   return pos;
// }

// bool planning_environment::PlanningMonitor::broadcastCollisions() {
  
//   unsigned int numContacts = num_contacts_for_display_;
//   if(!allowedContacts_.empty()) {
//     numContacts = num_contacts_allowable_contacts_test_;
//   }
//   //this just broadcasts collisions for the most recent collision space calls
//   if(onCollisionContact_) {
//     std::vector<collision_space::EnvironmentModel::Contact> contacts;
//     getEnvironmentModel()->getCollisionContacts(allowedContacts_, contacts, numContacts); 
//     ROS_DEBUG("Callback defined with %u contacts", (unsigned int) contacts.size());
//     for (unsigned int i = 0 ; i < contacts.size() ; ++i) {
//       onCollisionContact_(contacts[i]);
//     } 
//   } 
//   else 
//   {
//     return false;
//   }
//   return true;
// }

// void planning_environment::PlanningMonitor::getChildLinks(const std::vector<std::string> &joints, 
// 							  std::vector<std::string> &link_names)
// {
//   std::set<std::string> links_set;

//   for(unsigned int i=0; i < joints.size(); i++)
//   {
//     const planning_models::KinematicModel::JointModel *joint = getKinematicModel()->getJointModel(joints[i]);
//     if(joint)
//     {
//       if(joint->getChildLinkModel()) {
//         std::vector<const planning_models::KinematicModel::LinkModel*> child_links;
//         getKinematicModel()->getChildLinkModels(joint->getChildLinkModel(), child_links);
//         for(std::vector<const planning_models::KinematicModel::LinkModel*>::iterator it = child_links.begin();
//             it != child_links.end();
//             it++) {
//           if((*it)->getLinkShape() != NULL) {
//             links_set.insert((*it)->getName());
//           }
//         }
//       }
//     }
//   }
//   for(std::set<std::string>::iterator set_iterator = links_set.begin(); set_iterator!= links_set.end(); set_iterator++)
//   {
//     link_names.push_back(*set_iterator);
//   }
// }

// void planning_environment::PlanningMonitor::getOrderedCollisionOperationsForOnlyCollideLinks(const std::vector<std::string> &collision_check_links, 
//                                                                                              const motion_planning_msgs::OrderedCollisionOperations &requested_collision_operations,
//                                                                                              motion_planning_msgs::OrderedCollisionOperations &result_collision_operations)
// {
//   motion_planning_msgs::OrderedCollisionOperations result;
//   motion_planning_msgs::CollisionOperation op;
//   std::vector<motion_planning_msgs::CollisionOperation> self_collisions;

//   //this disables everything and everything
//   op.object1 = op.COLLISION_SET_ALL;
//   op.object2 = op.COLLISION_SET_ALL;
//   op.operation = op.operation = motion_planning_msgs::CollisionOperation::DISABLE;;
//   result.collision_operations.push_back(op);
  
//   //now we need to add bodies attached to these links
//   std::vector<std::string> all_collision_links = collision_check_links;
//   const std::vector<const planning_models::KinematicModel::AttachedBodyModel*> att_vec = getEnvironmentModel()->getAttachedBodies();
//   for(unsigned int i = 0; i < att_vec.size(); i++) 
//   {
//     for(std::vector<std::string>::const_iterator it = collision_check_links.begin();
//         it != collision_check_links.end();
//         it++) 
//     {
//       if(att_vec[i]->getAttachedLinkModel()->getName() == (*it)) {
//         all_collision_links.push_back(att_vec[i]->getName());
//       }
//     }
//   }

//   //this enables collision_check_links with everything
//   for(std::vector<std::string>::const_iterator it = all_collision_links.begin();
//       it != all_collision_links.end();
//       it++) 
//   {
//     op.object1 = (*it);
//     op.object2 = op.COLLISION_SET_ALL;
//     op.operation = motion_planning_msgs::CollisionOperation::ENABLE;
//     result.collision_operations.push_back(op);
//   } 

//   std::vector<std::vector<bool> > current_allowed;
//   std::map<std::string, unsigned int> vec_indices;
//   //this disables collision_check_links with things they are allowed to collide with
//   getEnvironmentModel()->getDefaultAllowedCollisionMatrix(current_allowed, vec_indices);
//   for(std::vector<std::string>::const_iterator it = all_collision_links.begin();
//       it != all_collision_links.end();it++)
//   {
//     std::map<std::string, unsigned int>::iterator map_it = vec_indices.find(*it);
//     if(map_it == vec_indices.end())
//       continue;
//     unsigned int index = map_it->second;
//     for(std::map<std::string, unsigned int>::iterator index_it = vec_indices.begin(); index_it != vec_indices.end(); index_it++)
//     {
//       if(current_allowed[index][index_it->second])
//       {	
//         op.object1 = (*it);
//         op.object2 = index_it->first;
//         op.operation = motion_planning_msgs::CollisionOperation::DISABLE;
//         result.collision_operations.push_back(op);    
//       }
//     }
//   }  

//   //this adds extra requested collision operations
//   for(std::vector<motion_planning_msgs::CollisionOperation>::const_iterator it = requested_collision_operations.collision_operations.begin();
//       it != requested_collision_operations.collision_operations.end();it++)
//   {
//     result.collision_operations.push_back(*it);
//   }  

//   result_collision_operations.collision_operations = result.collision_operations;

// }
