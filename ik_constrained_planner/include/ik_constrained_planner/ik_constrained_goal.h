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

/** \author Sachin Chitta, Ioan Sucan */

#ifndef IK_CONSTRAINED_GOAL_DEFINITIONS_
#define IK_CONSTRAINED_GOAL_DEFINITIONS_
#include <ompl/base/SpaceInformation.h>
#include <ompl/kinematic/SpaceInformationKinematic.h>

#include <ompl/base/GoalRegion.h>
#include <ompl/base/GoalState.h>
#include <ompl/base/State.h>

#include <motion_planning_msgs/Constraints.h>
#include <motion_planning_msgs/convert_messages.h>
#include <ik_constrained_planner/constraint_evaluators.h>

#include <kinematics_base/kinematics_base.h>

namespace ik_constrained_planner
{    

  class IKConstrainedGoal : public ompl::base::GoalState
  {
    public:
    IKConstrainedGoal(ompl::base::SpaceInformation *si, 
                      const motion_planning_msgs::Constraints &constraint,
                      const double &redundant_joint_value):ompl::base::GoalState(si)
    {
      state = new ompl::base::State(7);
      setup(constraint,redundant_joint_value);
    }
	
    virtual ~IKConstrainedGoal(void)
    {
    }
    
    virtual bool isSatisfied(const ompl::base::State *state, double *dist = NULL) const;    
    virtual double distanceGoal(const ompl::base::State *s) const;
    virtual void print(std::ostream &out = std::cout) const;
	
    protected:
	
    void setup(const motion_planning_msgs::Constraints &goal_constraint,                              
               const double &redundant_joint_value);

  private:
    
    constraint_evaluators::PositionConstraintEvaluator position_constraint_evaluator_;
    constraint_evaluators::OrientationConstraintEvaluator orientation_constraint_evaluator_;
    constraint_evaluators::JointConstraintEvaluator joint_constraint_evaluator_;

  };
}

#endif
