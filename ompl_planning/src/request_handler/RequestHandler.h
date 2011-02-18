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

#ifndef OMPL_PLANNING_REQUEST_HANDLER_
#define OMPL_PLANNING_REQUEST_HANDLER_

#include "ompl_planning/Model.h"

#include <ros/ros.h>

#include <motion_planning_msgs/GetMotionPlan.h>

#include <boost/bind.hpp>

#include <visualization_msgs/Marker.h>

/** \brief Main namespace */
namespace ompl_planning
{    
   
/** \brief This class represents a basic request to a motion
    planner. */

class RequestHandler
{
public:
	
  RequestHandler()
  {
    onFinishPlan_ = NULL;
    vis_marker_publisher_ = ros::NodeHandle().advertise<visualization_msgs::Marker>("ompl_collisions", 128);
  }
	
  ~RequestHandler(void)
  {
  }
	
  /** \brief Check if the request is valid */
  bool isRequestValid(ModelMap &models, motion_planning_msgs::GetMotionPlan::Request &req, const std::string &distance_metric);

  /** \brief Check and compute a motion plan. Return true if the plan was succesfully computed */
  bool computePlan(ModelMap &models, double stateDelay,
                   motion_planning_msgs::GetMotionPlan::Request &req, 
                   motion_planning_msgs::GetMotionPlan::Response &res, 
                   const std::string &distance_metric);

  /** \brief Enable callback for when a motion plan computation is completed */
  void setOnFinishPlan(const boost::function<void(PlannerSetup*)> &onFinishPlan);

  void contactFound(collision_space::EnvironmentModel::Contact &contact);
       
private:

  struct Solution
  {
    ompl::base::Path *path;
    double            difference;
    bool              approximate;
  };	
	
  /** \brief Set up all the data needed by motion planning based on a request */
  void configure(motion_planning_msgs::GetMotionPlan::Request &req, PlannerSetup *psetup, const std::string &distance_metric);

  /** \brief Compute the actual motion plan. Return true if computed plan was trivial (start state already in goal region) */
  bool callPlanner(PlannerSetup *psetup, const motion_planning_msgs::MotionPlanRequest& request, 
                   motion_planning_msgs::ArmNavigationErrorCodes& error_code, Solution &sol);
	
  /** \brief Set the workspace bounds based on the request */
  void setWorkspaceBounds(motion_planning_msgs::WorkspaceParameters &params, ompl_ros::ModelBase *ompl_model);

  bool checkPathForCollisions(PlannerSetup *psetup,
                              const motion_planning_msgs::MotionPlanRequest &req,
                              motion_planning_msgs::ArmNavigationErrorCodes& error_code,
                              ompl::kinematic::PathKinematic *kpath);
	
  /** \brief Fill the response with solution data */
  void fillResult(PlannerSetup *psetup, double stateDelay,
                  motion_planning_msgs::GetMotionPlan::Response &res, 
                  const Solution &sol);

  /** \brief Fix the input states, if they are not valid */
  bool fixInputStates(PlannerSetup *psetup, double value, unsigned int count);

  /** \brief Print the planner setup settings as debug messages */
  void printSettings(ompl::base::SpaceInformation *si);
	
  /** \brief Send visualization markers */
  void display(PlannerSetup *psetup);
	
  /** \brief Callback for when a plan computation is completed */
  boost::function<void(PlannerSetup*)> onFinishPlan_;
	
  ros::Publisher vis_marker_publisher_;

};
    
} // ompl_planning

#endif
