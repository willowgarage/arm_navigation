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

#include "ompl_planning/planners/kinematicLazyRRTSetup.h"

ompl_planning::kinematicLazyRRTSetup::kinematicLazyRRTSetup(void) : PlannerSetup()
{
    name = "kinematic::LazyRRT";
    priority = 2;
}

ompl_planning::kinematicLazyRRTSetup::~kinematicLazyRRTSetup(void)
{
}

bool ompl_planning::kinematicLazyRRTSetup::setup(planning_environment::PlanningMonitor *planningMonitor, const std::string &groupName,
						 boost::shared_ptr<PlannerConfig> &options)
{
    preSetup(planningMonitor, groupName, options);
    
    ompl::kinematic::LazyRRT *rrt = new ompl::kinematic::LazyRRT(dynamic_cast<ompl::kinematic::SpaceInformationKinematic*>(ompl_model->si));
    mp                            = rrt;    

    if (options->hasParam("range"))
    {
	rrt->setRange(options->getParamDouble("range", rrt->getRange()));
	ROS_DEBUG("Range is set to %g", rrt->getRange());
    }
    
    if (options->hasParam("goal_bias"))
    {
	rrt->setGoalBias(options->getParamDouble("goal_bias", rrt->getGoalBias()));
	ROS_DEBUG("Goal bias is set to %g", rrt->getGoalBias());
    }
    
    postSetup(planningMonitor, groupName, options);
    
    return true;
}

