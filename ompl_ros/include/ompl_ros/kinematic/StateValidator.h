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

#ifndef OMPL_ROS_KINEMATIC_STATE_VALIDATOR_
#define OMPL_ROS_KINEMATIC_STATE_VALIDATOR_

#include <ompl/base/StateValidityChecker.h>
#include <collision_space/environment.h>

#include "ompl_ros/ModelBase.h"
#include "ompl_ros/kinematic/SpaceInformation.h"

#include "motion_planning_msgs/Constraints.h"

#include <iostream>

namespace ompl_ros
{
    
    class ROSStateValidityPredicateKinematic : public ompl::base::StateValidityChecker
    {
    public:
        ROSStateValidityPredicateKinematic(ROSSpaceInformationKinematic *si, ModelBase *model) : ompl::base::StateValidityChecker(si)
	{
	    model_ = model;
	}
	
	virtual ~ROSStateValidityPredicateKinematic(void)
	{
	}
	
	virtual bool operator()(const ompl::base::State *s) const;
	
	/** \brief Used by the ROS space information to update constraints */
	void setConstraints(const motion_planning_msgs::Constraints &kc);

	/** \brief Used by the ROS space information to update constraints */
	void clearConstraints(void);

	/** \brief Used by the ROS space information to print information */
	void printSettings(std::ostream &out) const;
	
    protected:
	
	bool check(const ompl::base::State *s, collision_space::EnvironmentModel *em, planning_models::KinematicModel::JointGroup *jg,
		   const planning_environment::KinematicConstraintEvaluatorSet *kce) const;
	
	ModelBase *model_;
    };
    
} // ompl_ros

#endif
    
