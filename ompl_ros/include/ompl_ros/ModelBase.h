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

#ifndef OMPL_ROS_MODEL_BASE_
#define OMPL_ROS_MODEL_BASE_

#include <planning_environment/monitors/planning_monitor.h>
#include <planning_environment/util/kinematic_state_constraint_evaluator.h>
#include <ompl/base/SpaceInformation.h>
#include <string>
#include <map>

namespace ompl_ros
{
    
    /** \brief A class that contains pointers to structures needed in
	planning. The goal is to have multiple of these instances, one
	for each thread. */
    struct EnvironmentDescription
    {
	collision_space::EnvironmentModel                           *collisionSpace;
	planning_models::KinematicModel                             *kmodel;
	
	/** \brief The group instance */
	planning_models::KinematicModel::JointGroup                 *group;
	const planning_environment::KinematicConstraintEvaluatorSet *constraintEvaluator;	
    };
    
    /** \brief The basic definition of a model (a group defined by the planning environment) we are planning for */
    class ModelBase
    {
    public:
	ModelBase(planning_environment::PlanningMonitor *pMonitor, const std::string &gName);
	virtual ~ModelBase(void);
	
	virtual bool configure(void) = 0;
	
	/** \brief Thread safe function that returns the environment description corresponding to the active thread */
	EnvironmentDescription* getEnvironmentDescription(void) const;
	
	/** \brief Clear the created environment descriptions */
	void clearEnvironmentDescriptions(void) const;
	
	/** \brief An instance of a planning monitor that knows about the planning groups */
	planning_environment::PlanningMonitor                      *planningMonitor;

	/** \brief An instance of a kinematic constraint evaluator */
	planning_environment::KinematicConstraintEvaluatorSet       constraintEvaluator;
	
	/** \brief The group name */
	std::string                                                 groupName;
	
	/** \brief The group instance */
	planning_models::KinematicModel::JointGroup                *group;
	
	/** \brief The instance of the space information maintained for this group. si->setup() will need to be called after configure() */
	ompl::base::SpaceInformation                               *si;
	std::map<std::string, ompl::base::StateDistanceEvaluator*>  sde;        // list of available distance evaluators
    };
    
} // ompl_ros

#endif

