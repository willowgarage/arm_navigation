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

#ifndef OMPL_SEARCH_REQUEST_HANDLER_
#define OMPL_SEARCH_REQUEST_HANDLER_

#include <ompl_ros/ModelKinematic.h>
#include <ompl/extension/kinematic/extension/ik/GAIK.h>
#include <motion_planning_msgs/ConvertToJointConstraint.h>
#include <string>
#include <map>

/** \brief Main namespace */
namespace ompl_search
{    
    
  struct SearchModel
  {
    SearchModel(planning_environment::PlanningMonitor *planningMonitor, const std::string &groupName)
    {
	    mk = new ompl_ros::ModelKinematic(planningMonitor, groupName);
	    mk->configure();
	    gaik = new ompl::kinematic::GAIK(dynamic_cast<ompl::kinematic::SpaceInformationKinematic*>(mk->si));
    }
	
    ~SearchModel(void)
    {
	    delete gaik;
	    delete mk;
    }
	
    ompl_ros::ModelKinematic *mk;
    ompl::kinematic::GAIK    *gaik;
  };
    
  typedef std::map<std::string, SearchModel*> ModelMap;
    

  /** \brief This class represents a basic request to search 
      for a valid state. */
  class SearchRequestHandler
  {
  public:
	
    SearchRequestHandler(void)
      {
      }
	
    ~SearchRequestHandler(void)
      {
      }
	
    /** \brief Check if the request is valid */
    bool isRequestValid(ModelMap &models, motion_planning_msgs::ConvertToJointConstraint::Request &req, const std::string &distance_metric);
	
    /** \brief Find a state in the specified goal region. Return true if state was found */
    bool findState(ModelMap &models, const planning_models::KinematicState *start, motion_planning_msgs::ConvertToJointConstraint::Request &req, motion_planning_msgs::ConvertToJointConstraint::Response &res, const std::string &distance_metric);

  private:

    /** \brief Set up all the data needed by inverse kinematics based on a request */
    void configure(const planning_models::KinematicState *startState, motion_planning_msgs::ConvertToJointConstraint::Request &req, SearchModel *model, const std::string &distance_metric);

    /** \brief Set the workspace bounds based on the request */
    void setWorkspaceBounds(motion_planning_msgs::WorkspaceParameters &params, ompl_ros::ModelKinematic *model);
	
    /** \brief Print the planner setup settings as debug messages */
    void printSettings(ompl::base::SpaceInformation *si);
	
  };
    
} // ompl_planning

#endif
