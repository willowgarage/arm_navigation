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

#include "ompl_ros/dynamic/StateValidator.h"

bool ompl_ros::ROSStateValidityPredicateDynamic::operator()(const ompl::base::State *s) const
{
  if (!dsi_->satisfiesBounds(s))
    return false;

  EnvironmentDescription *ed = model_->getEnvironmentDescription();
  return check(s, ed, ed->constraintEvaluator);
}

void ompl_ros::ROSStateValidityPredicateDynamic::setConstraints(const motion_planning_msgs::Constraints &kc)
{
  clearConstraints();
  model_->constraintEvaluator.add(kc.position_constraints);
  model_->constraintEvaluator.add(kc.orientation_constraints);
}

void ompl_ros::ROSStateValidityPredicateDynamic::clearConstraints(void)
{
  model_->constraintEvaluator.clear();
}

void ompl_ros::ROSStateValidityPredicateDynamic::printSettings(std::ostream &out) const
{    
  out << "Path constraints:" << std::endl;
  model_->constraintEvaluator.print(out);
}

bool ompl_ros::ROSStateValidityPredicateDynamic::check(const ompl::base::State *s, 
                                                       EnvironmentDescription* ed, 
                                                       const planning_environment::KinematicConstraintEvaluatorSet *kce) const
{

  std::vector<double> vals(s->values,s->values+ed->group_state->getDimension());
  ed->group_state->setKinematicState(vals);
    
  bool valid = kce->decide(ed->full_state);
  if (valid)
    {
      //ed->collisionSpace->updateRobotModel(ed->full_state);
      //valid = !ed->collisionSpace->isCollision();
    }
    
  return valid;
}
