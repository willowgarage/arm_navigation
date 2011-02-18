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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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

/** \author E.Gil Jones */

#ifndef OMPL_PLANNER_CONFIG_
#define OMPL_PLANNER_CONFIG_

#include <boost/shared_ptr.hpp>
#include <string>
#include <map>
#include <vector>

#include <ros/ros.h>

namespace ompl_ros_interface
{
/** 
 * @class PlannerConfig
 * @brief A class to define a planner configuration 
 */
class PlannerConfig
{
public:
  /**
   * @brief Default constructor 
   * @param description - the namespace containing the parameters corresponding to the planner
   * @param config - the actual name of the configuration space for this planner, the parameters will be 
read from the ROS parameter server at "description/config".
   */
  PlannerConfig(const std::string &description, const std::string &config) : description_(description), config_(config)
  {
  }
  
  ~PlannerConfig(void)
  {
  }
  
  /**
   * @brief Return the name of the planner configuration parameter
   * @return the string name
   */
  const std::string& getName(void);

  /**
   * @brief Check whether the particular parameter exists on the parameter server 
   * within the namespace defined by the planner configuration
   * @param param - the string name of the parameter to look for
   */
  bool        hasParam(const std::string &param);

  /**
   * @brief Get a string parameter
   * @param param - the string parameter to get
   * @param def (optional) - the return value if the parameter does not exist 
   */
  std::string getParamString(const std::string &param, const std::string &def = "");

  /**
   * @brief Get a double parameter
   * @param param - the name of the parameter to get
   * @param def (optional) - the return value if the parameter does not exist 
   */
  double      getParamDouble(const std::string &param, double def);

  /**
   * @brief Get a integer parameter
   * @param param - the name of the parameter to get
   * @param def (optional) - the return value if the parameter does not exist 
   */
  int         getParamInt(const std::string &param, int def);
  
private:
  
  std::string     description_;
  std::string     config_;
  ros::NodeHandle nh_;	  
};

/** 
 * @class PlannerConfigMap
 * @brief A map from group name to planner configuration names. This class is used internally and 
 * is not intended for external use.
 */
class PlannerConfigMap
{
public:
  
  PlannerConfigMap(std::string description) :
    description_(description)
  {
  }
  
  ~PlannerConfigMap(void)
  {
  }

  void loadPlannerConfigs();

  /** \brief Get the list of planner configurations available for a specific planning group */
  std::vector< boost::shared_ptr<PlannerConfig> > getGroupPlannersConfig(const std::string &group) const;

public:
  std::string description_; /// The namespace in which this group lives
  std::map<std::string, std::vector<std::string> > group_to_planner_string_config_map_; /// A mapping from the group to the list of planners corresponding to the group
  std::vector<std::string> planning_group_names_; /// The names of the planning groups 
};
} //namespace ompl_ros_interface

#endif
