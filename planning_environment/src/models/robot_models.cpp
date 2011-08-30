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

planning_environment::RobotModels::RobotModels(const std::string &description) : priv_nh_("~")
{
  description_ = nh_.resolveName(description);
  loaded_models_ = false;
  loadRobotFromParamServer();
}

planning_environment::RobotModels::RobotModels(boost::shared_ptr<urdf::Model> urdf,
                                               planning_models::KinematicModel* kmodel) {
  urdf_ = urdf;
  kmodel_ = kmodel;
  loaded_models_ = true;
}


void planning_environment::RobotModels::reload(void)
{
  urdf_.reset();
  delete kmodel_;
  loadRobotFromParamServer();
}

void planning_environment::RobotModels::loadRobotFromParamServer(void)
{
  std::string content;
  if (nh_.getParam(description_, content))
  {
    urdf_ = boost::shared_ptr<urdf::Model>(new urdf::Model());
    if (urdf_->initString(content))
    {
      loaded_models_ = true;
      std::vector<planning_models::KinematicModel::GroupConfig> group_configs;
      std::vector<planning_models::KinematicModel::MultiDofConfig> multi_dof_configs;
      bool hasMulti = loadMultiDofConfigsFromParamServer(multi_dof_configs);
      loadGroupConfigsFromParamServer(multi_dof_configs, group_configs);
      if(hasMulti) {
        kmodel_ = new planning_models::KinematicModel(*urdf_, group_configs, multi_dof_configs);
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

bool planning_environment::RobotModels::loadMultiDofConfigsFromParamServer(std::vector<planning_models::KinematicModel::MultiDofConfig>& configs) 
{
  configs.clear();
  
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
    configs.push_back(mdc);
  }
  if(configs.empty()) {
    return false;
  }
  return true;
}

void planning_environment::RobotModels::loadGroupConfigsFromParamServer(const std::vector<planning_models::KinematicModel::MultiDofConfig>& multi_dof_configs,
                                                                        std::vector<planning_models::KinematicModel::GroupConfig>& configs) {
  
  configs.clear();

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
    bool already_have = false;
    for(unsigned int j = 0; j < configs.size(); j++) {
      if(configs[j].name_ == gname) {
        ROS_WARN_STREAM("Already have group name " << gname); 
        already_have = true;
        break;
      }
    }
    if(already_have) continue;
    if((all_groups[i].hasMember("base_link") && !all_groups[i].hasMember("tip_link")) ||
       (!all_groups[i].hasMember("base_link") && all_groups[i].hasMember("tip_link"))) {
      ROS_WARN_STREAM("If using chain definition must have both base_link and tip_link defined");
      continue;
    }
    //only need to test one condition
    if(all_groups[i].hasMember("base_link")) {
      configs.push_back(planning_models::KinematicModel::GroupConfig(gname,
                                                                     all_groups[i]["base_link"],
                                                                     all_groups[i]["tip_link"])); 
    } else {
      if(!all_groups[i].hasMember("subgroups") && !all_groups[i].hasMember("joints")) {
        ROS_WARN_STREAM("Group " << gname << " is not a valid chain and thus must have one or more joint or subgroups defined");
        continue;
      }

      std::vector<std::string> subgroups;
      if(all_groups[i].hasMember("subgroups")) {
        XmlRpc::XmlRpcValue subgroups_seq = all_groups[i]["subgroups"];
        if(subgroups_seq.getType() != XmlRpc::XmlRpcValue::TypeArray) {
          ROS_WARN_STREAM("Group " << gname << " subgroups not an array");
          continue;
        }
        for(unsigned int j = 0; j < (unsigned int) subgroups_seq.size(); j++) {
          subgroups.push_back(std::string(subgroups_seq[j]));
        }
      }

      std::vector<std::string> joints;
      if(all_groups[i].hasMember("joints")) {
        XmlRpc::XmlRpcValue joints_seq = all_groups[i]["joints"];
        if(joints_seq.getType() != XmlRpc::XmlRpcValue::TypeArray) {
          ROS_WARN_STREAM("Group " << gname << " joints not an array");
          continue;
        }
        for(unsigned int j = 0; j < (unsigned int) joints_seq.size(); j++) {
          std::string jname = std::string(joints_seq[j]);
          if (urdf_->getJoint(jname)) {
            joints.push_back(jname);
          } else {
            bool have_multi = false;
            for(unsigned int i = 0; i < multi_dof_configs.size(); i++) {
              if(jname == multi_dof_configs[i].name) {
                joints.push_back(jname);
                have_multi = true;
                break;
              }
            }
            if(!have_multi) {
              ROS_WARN_STREAM("Urdf doesn't have joint " << jname << " and no multi-dof joint of that name");
            }
          }
        }
      }
      configs.push_back(planning_models::KinematicModel::GroupConfig(gname,
                                                                     joints,
                                                                     subgroups));
    }
  }
}


