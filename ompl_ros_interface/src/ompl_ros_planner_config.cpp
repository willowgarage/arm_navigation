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

/** \author E. Gil Jones */

#include "ompl_ros_interface/ompl_ros_planner_config.h"
#include <boost/algorithm/string.hpp>
#include <sstream>

const std::string& ompl_ros_interface::PlannerConfig::getName(void)
{
    return config_;
}

bool ompl_ros_interface::PlannerConfig::hasParam(const std::string &param)
{
    return nh_.hasParam(description_ + "/planner_configs/" + config_ + "/" + param);
}

std::string ompl_ros_interface::PlannerConfig::getParamString(const std::string &param, const std::string& def)
{
    std::string value;
    nh_.param(description_ + "/planner_configs/" + config_ + "/" + param, value, def);
    boost::trim(value);
    return value;
}

double ompl_ros_interface::PlannerConfig::getParamDouble(const std::string &param, double def)
{
    double value;
    nh_.param(description_ + "/planner_configs/" + config_ + "/" + param, value, def);
    return value;
}

int ompl_ros_interface::PlannerConfig::getParamInt(const std::string &param, int def)
{
    int value;
    nh_.param(description_ + "/planner_configs/" + config_ + "/" + param, value, def);
    return value;
}

void ompl_ros_interface::PlannerConfigMap::loadPlannerConfigs() {

  ros::NodeHandle nh("~");

  //first we load the group_list
  std::string group_list;
  nh.param(description_ + "/group_list", group_list, std::string(""));
  std::stringstream group_list_stream(group_list);
  std::map<std::string, bool> plan_group_map;
  while (group_list_stream.good() && !group_list_stream.eof())
  {
    std::string g;
    group_list_stream >> g;
    if(g.size() == 0) continue;
    planning_group_names_.push_back(g);
  }

  //now we go through the groups
  XmlRpc::XmlRpcValue all_groups;
  
  if(!nh.hasParam(description_+"/groups")) {
    ROS_WARN("No groups for planning specified");
    return;
  } 
  
  nh.getParam(description_+"/groups", all_groups);
  
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
    std::vector<std::string> configs;
    
    std::string plan_list = std::string(all_groups[i]["planner_configs"]);
    std::stringstream planner_name_stream(plan_list);
    while(planner_name_stream.good() && !planner_name_stream.eof()){
      std::string pname; 
      planner_name_stream >> pname;
      if(pname.size() == 0) continue;
      configs.push_back(pname);
    }
    group_to_planner_string_config_map_[gname] = configs;
  }
}

std::vector< boost::shared_ptr<ompl_ros_interface::PlannerConfig> > ompl_ros_interface::PlannerConfigMap::getGroupPlannersConfig(const std::string &group) const
{
  std::vector< boost::shared_ptr<ompl_ros_interface::PlannerConfig> > ret;
  if(group_to_planner_string_config_map_.find(group) != group_to_planner_string_config_map_.end()) {
    //planners expect their own shared pointers
    for(std::vector<std::string>::const_iterator it = group_to_planner_string_config_map_.find(group)->second.begin();
        it != group_to_planner_string_config_map_.find(group)->second.end();
        it++) {
      ret.push_back(boost::shared_ptr<PlannerConfig>(new ompl_ros_interface::PlannerConfig(description_, (*it))));
    }
  }
  return ret;
}
