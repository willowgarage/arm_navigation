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

#ifndef OMPL_PLANNING_PLANNERS_PLANNER_SETUP_
#define OMPL_PLANNING_PLANNERS_PLANNER_SETUP_

#include "ompl_ros/ModelKinematic.h"
#include "ompl_ros/ModelDynamic.h"
#include "ompl_ros/base/GoalDefinitions.h"
#include "ompl_ros/base/ProjectionEvaluators.h"

#include <ompl/base/Planner.h>
#include <ompl/extension/kinematic/PathSmootherKinematic.h>

#include <ros/console.h>
#include <string>
#include <map>

namespace ompl_planning
{    
    class PlannerSetup
    {
    public:
	
	PlannerSetup(void);
	virtual ~PlannerSetup(void);
	
	virtual bool setup(planning_environment::PlanningMonitor *planningMonitor, const std::string &groupName,
			   boost::shared_ptr<planning_environment::RobotModels::PlannerConfig> &options) = 0;
	
	std::string                                                name;       // name of planner
	ompl_ros::ModelBase                                       *ompl_model;       
	ompl::base::Planner                                       *mp;         // pointer to OMPL instance of planner
	int                                                        priority;   // priority of this planner when automatically selecting planners
	ompl::kinematic::PathSmootherKinematic                    *smoother;   // pointer to an OMPL path smoother, if needed

    protected:

	virtual ompl::base::ProjectionEvaluator* getProjectionEvaluator(boost::shared_ptr<planning_environment::RobotModels::PlannerConfig> &options) const;
	
	virtual void preSetup(planning_environment::PlanningMonitor *planningMonitor, const std::string &groupName,
			      boost::shared_ptr<planning_environment::RobotModels::PlannerConfig> &options);
	virtual void postSetup(planning_environment::PlanningMonitor *planningMonitor, const std::string &groupName,
			       boost::shared_ptr<planning_environment::RobotModels::PlannerConfig> &options);
	
    };

    
} // ompl_planning

#endif
    
