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

/** \author Ioan Sucan */

#include "planning_environment/models/robot_models.h"
#include <ros/console.h>

#include <boost/algorithm/string.hpp>
#include <sstream>

void planning_environment::RobotModels::reload(void)
{
    kmodel_.reset();
    urdf_.reset();
    for(std::map<std::string, GroupConfig*>::iterator it = group_config_map_.begin();
        it != group_config_map_.end();
        it++) {
      delete it->second;
    }
    group_config_map_.clear();
    planning_group_joints_.clear();
    planning_group_links_.clear();
    group_joint_union_.clear();
    group_link_union_.clear();

    loadRobot();
}

void planning_environment::RobotModels::loadRobot(void)
{
    std::string content;
    if (nh_.getParam(description_, content))
    {
	urdf_ = boost::shared_ptr<urdf::Model>(new urdf::Model());
	if (urdf_->initString(content))
	{
	    loaded_models_ = true;
            readGroupConfigs();
            bool hasMulti = readMultiDofConfigs();
            if(hasMulti) {
              kmodel_ = boost::shared_ptr<planning_models::KinematicModel>(new planning_models::KinematicModel(*urdf_, planning_group_joints_, multi_dof_configs_));
            } else {
              ROS_WARN("Can't do anything without a root transform");
            }
	}
	else
	{
	    urdf_.reset();
	    ROS_ERROR("Unable to parse URDF description!");
	}
    }
    else
	ROS_ERROR("Robot model '%s' not found! Did you remap 'robot_description'?", description_.c_str());
}

bool planning_environment::RobotModels::readMultiDofConfigs() {
  
  XmlRpc::XmlRpcValue multi_dof_joints;
  
  std::string multi_dof_name = description_ + "_planning/multi_dof_joints";
  
  if(!nh_.hasParam(multi_dof_name)) {
    ROS_WARN_STREAM("No multi dof joints specified, including root to world conversion");
    return false;
  }
 
  nh_.getParam(multi_dof_name, multi_dof_joints);
  
  if(multi_dof_joints.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_WARN("Multi-dof joints is not an array");
    return false;
  }
  
  for(int i = 0; i < multi_dof_joints.size(); i++) {
    if(!multi_dof_joints[i].hasMember("name")) {
      ROS_WARN("Multi dof joint must have name");
      continue;
    }
    if(!multi_dof_joints[i].hasMember("parent_frame_id")) {
      ROS_WARN("Multi dof joint must have parent frame id");
      continue;
    }
    if(!multi_dof_joints[i].hasMember("child_frame_id")) {
      ROS_WARN("Multi dof joint must have child frame id");
      continue;
    }
    if(!multi_dof_joints[i].hasMember("type")) {
      ROS_WARN("Multi dof joint must have a type");
      continue;
    }
    std::string joint_name = multi_dof_joints[i]["name"];
    planning_models::KinematicModel::MultiDofConfig mdc(joint_name);
    for(XmlRpc::XmlRpcValue::iterator it = multi_dof_joints[i].begin();
        it != multi_dof_joints[i].end();
        it++) {
      if(it->first == "parent_frame_id") {
        mdc.parent_frame_id = std::string(it->second);
      } else if(it->first == "child_frame_id") {
        mdc.child_frame_id = std::string(it->second);
      } else if(it->first == "type") {
        mdc.type = std::string(it->second);
      } else if(it->first != "name") {
        mdc.name_equivalents[it->first] = std::string(it->second);
      }
    }
    multi_dof_configs_.push_back(mdc);
  }
  if(multi_dof_configs_.empty()) {
    return false;
  }
  return true;
}

void planning_environment::RobotModels::readGroupConfigs() {
  
  XmlRpc::XmlRpcValue all_groups;
  
  std::string group_name = description_ + "_planning/groups";

  if(!nh_.hasParam(group_name)) {
    ROS_WARN_STREAM("No groups for planning specified in " << group_name);
    return;
  } 
  
  nh_.getParam(group_name, all_groups);
  
  if(all_groups.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_WARN("Groups is not an array");
    return;
  }
  
  if(all_groups.size() == 0) {
    ROS_WARN("No groups in groups");
    return;
  }
  
  for(int i = 0; i < all_groups.size(); i++) {
    if(!all_groups[i].hasMember("name")) {
      ROS_WARN("All groups must have a name");
      continue;
    }
    std::string gname = all_groups[i]["name"];
    GroupConfig* gc = new GroupConfig(gname);
    std::map< std::string, GroupConfig*>::iterator group_iterator = group_config_map_.find(gname);
    if(group_iterator != group_config_map_.end()) {
      ROS_WARN_STREAM("Already have group name " << gname); 
      delete gc;
      continue;
    } 
    group_config_map_[gname] = gc;
    if(all_groups[i].hasMember("joints")) {
      std::string joint_list = std::string(all_groups[i]["joints"]);
      std::stringstream joint_name_stream(joint_list);
      while(joint_name_stream.good() && !joint_name_stream.eof()){
        std::string jname; 
        joint_name_stream >> jname;
        if(jname.size() == 0) continue;
        if (urdf_->getJoint(jname)) {
          group_config_map_[gname]->joint_names_.push_back(jname);
        } else {
          ROS_DEBUG_STREAM("Urdf doesn't have joint " << jname);
        }
      }                                          
    }
    if(all_groups[i].hasMember("links")) {
      std::string link_list = std::string(all_groups[i]["links"]);
      std::stringstream link_name_stream(link_list);
      while(link_name_stream.good() && !link_name_stream.eof()){
        std::string lname; 
        link_name_stream >> lname;
        if(lname.size() == 0) continue;
        if (urdf_->getLink(lname)) {
          group_config_map_[gname]->link_names_.push_back(lname);
          ROS_DEBUG_STREAM("Urdf has link " << lname);
        } else {
          ROS_INFO_STREAM("Urdf doesn't have link " << lname);
        }
      }                                          
    }
    planning_group_joints_[gname] = group_config_map_[gname]->getJointNames();
    for(std::vector<std::string>::iterator it = planning_group_joints_[gname].begin();
        it != planning_group_joints_[gname].end();
        it++) {
      ROS_DEBUG_STREAM("Group " << gname << " joint " << (*it));
    }
    
    planning_group_links_[gname] = group_config_map_[gname]->getLinkNames();
  }

  //want exclusivity in the union
  std::map<std::string, bool> pmap;
  group_joint_union_.clear();
  for(std::map< std::string, std::vector<std::string> >::iterator it1 = planning_group_joints_.begin();
      it1 != planning_group_joints_.end();
      it1++) {
    for(std::vector<std::string>::iterator it2 = it1->second.begin();
        it2 != it1->second.end();
        it2++) {
      pmap[*it2] = true;
    }
  }
  for(std::map<std::string, bool>::iterator it = pmap.begin();
      it != pmap.end();
      it++) {
    group_joint_union_.push_back(it->first);
  }

  pmap.clear();
  group_link_union_.clear();
  for(std::map< std::string, std::vector<std::string> >::iterator it1 = planning_group_links_.begin();
      it1 != planning_group_links_.end();
      it1++) {
    for(std::vector<std::string>::iterator it2 = it1->second.begin();
        it2 != it1->second.end();
        it2++) {
      pmap[*it2] = true;
    }
  }
  for(std::map<std::string, bool>::iterator it = pmap.begin();
      it != pmap.end();
      it++) {
    group_link_union_.push_back(it->first);
  }
}


