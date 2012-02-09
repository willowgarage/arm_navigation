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
//#include <arm_navigation_msgs/DisplayTrajectory.h>
#include "planning_environment/models/model_utils.h"

void planning_environment::PlanningMonitor::loadParams(void)
{
}

bool planning_environment::PlanningMonitor::getCompletePlanningScene(const arm_navigation_msgs::PlanningScene& planning_diff,
                                                                     const arm_navigation_msgs::OrderedCollisionOperations& ordered_collision_operations,
                                                                     arm_navigation_msgs::PlanningScene& planning_scene) const{
  {
    //indenting because we only need the state in here
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
  }

  //getting full list of tf transforms not associated with the robot's body
  getAllFixedFrameTransforms(planning_scene.fixed_frame_transforms);
  
  //getting all the stuff from the current collision space
  collision_space::EnvironmentModel::AllowedCollisionMatrix acm = cm_->getCollisionSpace()->getDefaultAllowedCollisionMatrix();
  if(planning_diff.allowed_collision_matrix.link_names.size() > 0) {
    acm = convertFromACMMsgToACM(planning_diff.allowed_collision_matrix);
  }

  //first we deal with collision object diffs
  cm_->getCollisionSpaceCollisionObjects(planning_scene.collision_objects);

  for(unsigned int i = 0; i < planning_scene.collision_objects.size(); i++) {
    if(!acm.hasEntry(planning_scene.collision_objects[i].id)) {
      ROS_ERROR_STREAM("Sanity check failing - no entry in acm for collision space object " << planning_scene.collision_objects[i].id);
    } 
  }
  
  cm_->getLastCollisionMap(planning_scene.collision_map);

  //probably haven't gotten another collision map yet after a clear
  if(planning_scene.collision_map.boxes.size() > 0 && !acm.hasEntry(COLLISION_MAP_NAME)) {
    ROS_INFO_STREAM("Adding entry for collision map");
    acm.addEntry(COLLISION_MAP_NAME, false);
  }

  //now attached objects
  cm_->getCollisionSpaceAttachedCollisionObjects(planning_scene.attached_collision_objects);

  for(unsigned int i = 0; i < planning_scene.attached_collision_objects.size(); i++) {
    if(!acm.hasEntry(planning_scene.attached_collision_objects[i].object.id)) {
      ROS_ERROR_STREAM("Sanity check failing - no entry in acm for attached collision space object " << planning_scene.attached_collision_objects[i].object.id);
    } 
  }

  std::map<std::string, double> cur_link_padding = cm_->getCollisionSpace()->getCurrentLinkPaddingMap();

  for(unsigned int i = 0; i < planning_diff.collision_objects.size(); i++) {
    std::string object_name = planning_diff.collision_objects[i].id;
    if(object_name == "all") {
      if(planning_diff.collision_objects[i].operation.operation == arm_navigation_msgs::CollisionObjectOperation::REMOVE) {
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
    std::vector<arm_navigation_msgs::CollisionObject>::iterator it = planning_scene.collision_objects.begin();
    while(it != planning_scene.collision_objects.end()) {
      if((*it).id == object_name) {
        already_have = true;
        break;
      }
      it++;
    }
    if(planning_diff.collision_objects[i].operation.operation == arm_navigation_msgs::CollisionObjectOperation::REMOVE) {
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
    }
  }
  

  for(unsigned int i = 0; i < planning_diff.attached_collision_objects.size(); i++) {
    std::string link_name = planning_diff.attached_collision_objects[i].link_name;
    std::string object_name = planning_diff.attached_collision_objects[i].object.id;
    if(planning_diff.attached_collision_objects[i].object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT ||
       planning_diff.attached_collision_objects[i].object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT) {
      ROS_WARN_STREAM("Object replacement not supported during diff");
    }
    if(link_name == "all") {
      if(planning_diff.attached_collision_objects[i].object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::REMOVE) {
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
        if(planning_diff.attached_collision_objects[i].object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::REMOVE) {
          std::vector<arm_navigation_msgs::AttachedCollisionObject>::iterator it = planning_scene.attached_collision_objects.begin();
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
      std::vector<arm_navigation_msgs::AttachedCollisionObject>::iterator it = planning_scene.attached_collision_objects.begin();
      while(it != planning_scene.attached_collision_objects.end()) {
        if((*it).link_name == link_name) {
          if((*it).object.id == object_name) {
            already_have = true;
            break;
          }
        }
        it++;
      }
      if(planning_diff.attached_collision_objects[i].object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::REMOVE) {
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
        std::vector<std::string> modded_touch_links; 
        for(unsigned int j = 0; j < planning_diff.attached_collision_objects[i].touch_links.size(); j++) {
          if(cm_->getKinematicModel()->getModelGroup(planning_diff.attached_collision_objects[i].touch_links[j])) {
            std::vector<std::string> links = cm_->getKinematicModel()->getModelGroup(planning_diff.attached_collision_objects[i].touch_links[j])->getGroupLinkNames();
            modded_touch_links.insert(modded_touch_links.end(), links.begin(), links.end());
          } else {
            modded_touch_links.push_back(planning_diff.attached_collision_objects[i].touch_links[j]);
          }
        }
        for(unsigned int j = 0; j < modded_touch_links.size(); j++) {
          acm.changeEntry(planning_diff.attached_collision_objects[i].object.id, modded_touch_links[j], true);
        }
        planning_scene.attached_collision_objects.push_back(planning_diff.attached_collision_objects[i]);
      }
    }
  }

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

  planning_scene.allowed_contacts = planning_diff.allowed_contacts;

  return true;
}   

void planning_environment::PlanningMonitor::getAllFixedFrameTransforms(std::vector<geometry_msgs::TransformStamped>& transform_vec) const
{
  transform_vec.clear();

  std::vector<std::string> all_frame_names;
  tf_->getFrameStrings(all_frame_names);
  //TODO - doesn't cope with tf namespaces
  //takes out leading slashes
  for(unsigned int i = 0; i < all_frame_names.size(); i++) {
    if(!all_frame_names[i].empty() && all_frame_names[i][0] == '/') {
      all_frame_names[i].erase(all_frame_names[i].begin());
    }
  }
  //the idea here is that we need to figure out how to take poses from other frames
  //and transform them into the fixed frame. So we want to get the transform
  //that goes from the frame to the identity of the world frame and take the inverse
  for(unsigned int i = 0; i < all_frame_names.size(); i++) {
    if(all_frame_names[i] != cm_->getWorldFrameId() && 
       !cm_->getKinematicModel()->hasLinkModel(all_frame_names[i])) {
      ROS_DEBUG_STREAM("Adding fixed frame transform for frame " << all_frame_names[i]);
      geometry_msgs::PoseStamped ident_pose;
      ident_pose.header.frame_id = cm_->getWorldFrameId();
      ident_pose.pose.orientation.w = 1.0;
      std::string err_string;
      if (tf_->getLatestCommonTime(cm_->getWorldFrameId(), all_frame_names[i], ident_pose.header.stamp, &err_string) != tf::NO_ERROR) {
        ROS_WARN_STREAM("No transform whatsoever available between " << all_frame_names[i] << " and fixed frame" << cm_->getWorldFrameId());
        continue;
      }
      geometry_msgs::PoseStamped trans_pose;
      try {
        tf_->transformPose(all_frame_names[i], ident_pose, trans_pose);
      } catch(tf::TransformException& ex) {
        //just not going to cache this one
	//ROS_WARN_STREAM("Unable to transform object from frame " << all_frame_names[i] << " to fixed frame " 
        //                 << cm_->getWorldFrameId() << " tf error is " << ex.what());
        continue;
      }
      geometry_msgs::TransformStamped f;
      f.header = ident_pose.header;
      f.child_frame_id = all_frame_names[i];
      f.transform.translation.x = trans_pose.pose.position.x;
      f.transform.translation.y = trans_pose.pose.position.y;
      f.transform.translation.z = trans_pose.pose.position.z;
      f.transform.rotation = trans_pose.pose.orientation;
      transform_vec.push_back(f);
    }
  }
}
