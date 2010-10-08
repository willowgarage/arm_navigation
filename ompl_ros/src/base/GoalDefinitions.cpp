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

#include "ompl_ros/base/GoalDefinitions.h"
#include <ompl/kinematic/SpaceInformationKinematic.h>
#include <ros/console.h>
#include <climits>

double ompl_ros::GoalToState::distanceGoal(const ompl::base::State *s) const
{
    const double *vals = s->values;
    for (int i = 0 ; i < dim_; ++i)
	if (bounds_[i].first > vals[i])
	    cVals_[i] = stateVals_[i] + bounds_[i].first - vals[i];
	else
	    if (bounds_[i].second < vals[i])
		cVals_[i] = stateVals_[i] + vals[i] - bounds_[i].second;
	    else
		cVals_[i] = stateVals_[i];
    return m_si->distance(compState_, state);
}

const std::vector< std::pair<double, double> >& ompl_ros::GoalToState::getBounds(void) const
{
    return bounds_;
}

void ompl_ros::GoalToState::print(std::ostream &out) const
{
    ompl::base::GoalState::print(out);
    out << "Joint constraints: " << std::endl;
    for (int i = 0 ; i < dim_ ; ++i)
	out << "[" << bounds_[i].first << ", " << bounds_[i].second << "] ";
    out << std::endl;
}
	
void ompl_ros::GoalToState::setup(ModelBase *model, const std::vector<motion_planning_msgs::JointConstraint> &jc) 
{
  planning_models::KinematicState kin_state(model->planningMonitor->getKinematicModel());
  const planning_models::KinematicState::JointStateGroup* group_state = kin_state.getJointStateGroup(model->group->getName());

  dim_ = model->si->getStateDimension();
    
  // get the bounds for the state
  bounds_.resize(dim_);
  for (int i = 0 ; i < dim_ ; ++i)
    bounds_[i] = std::make_pair(model->si->getStateComponent(i).minValue, model->si->getStateComponent(i).maxValue);

  // keep track of the desired joint values
  std::vector<double> desiredValue(dim_);
  std::vector<int> desiredValueCount(dim_, 0);
  const std::map<std::string, unsigned int>& all_group_order = group_state->getKinematicStateIndexMap();
  
  // tighten the bounds based on the constraints
  for (unsigned int i = 0 ; i < jc.size() ; ++i)
    {
      // get the index at which the joint parameters start
      if(group_state->getJointState(jc[i].joint_name)) {
        int idx = all_group_order.find(jc[i].joint_name)->second;
        if (idx >= 0)
        {
          if (bounds_[idx].first < jc[i].position - jc[i].tolerance_below)
            bounds_[idx].first = jc[i].position - jc[i].tolerance_below;
          if (bounds_[idx].second > jc[i].position + jc[i].tolerance_above)
            bounds_[idx].second = jc[i].position + jc[i].tolerance_above;
          desiredValueCount[idx]++;
          desiredValue[idx] = jc[i].position;
        }   
      }
    }
  
  // check if joint bounds are feasible
  for (int i = 0 ; i < dim_ ; ++i)
    if (bounds_[i].first > bounds_[i].second)
      {
        ROS_ERROR("Inconsistent set of joint constraints at state component with index %d", i);
        break;
      }
    
  compState_ = new ompl::base::State(dim_);
  cVals_ = compState_->values;
    
  // compute the middle state
  if (state == NULL)
    state = new ompl::base::State(dim_);
  stateVals_ = state->values;
    
  for (int i = 0 ; i < dim_ ; ++i)
    {
      if (desiredValueCount[i] == 1)
        stateVals_[i] = desiredValue[i];
      else
        {
          stateVals_[i] = (bounds_[i].first + bounds_[i].second) / 2.0;
          if (desiredValueCount[i] > 0)
            ROS_WARN("More than one desired values were specified for joint %d in group %s", i, model->groupName.c_str());
        }
	
      // increase bounds by epsilon so we do not have errors when checking
      bounds_[i].first  -= ompl::STATE_EPSILON;
      bounds_[i].second += ompl::STATE_EPSILON;
    }
    
  threshold = ompl::STATE_EPSILON;	    
}

double ompl_ros::GoalToPosition::distanceGoal(const ompl::base::State *state) const
{
    return evaluateGoalAux(state, NULL);
}

bool ompl_ros::GoalToPosition::isSatisfied(const ompl::base::State *state, double *dist) const
{
    std::vector<bool> decision;
    double d = evaluateGoalAux(state, &decision);
    if (dist)
	*dist = d;
    
    for (unsigned int i = 0 ; i < decision.size() ; ++i)
	if (!decision[i])
	    return false;
    return true;
}

void ompl_ros::GoalToPosition::print(std::ostream &out) const
{
    ompl::base::GoalRegion::print(out);
    out << "Pose constraints:" << std::endl;
    for (unsigned int i = 0 ; i < pce_.size() ; ++i)
	pce_[i]->print(out);
}
	
double ompl_ros::GoalToPosition::evaluateGoalAux(const ompl::base::State *state, std::vector<bool> *decision) const
{
  //TODO - orientation_importance
  double orientation_importance = 1.0;
  EnvironmentDescription *ed = model_->getEnvironmentDescription();
  ROS_INFO("Evaluating goal");
  std::vector<double> vals(state->values,state->values+ed->group_state->getDimension());
  ed->group_state->setKinematicState(vals);

  if (decision)
    decision->resize(pce_.size());
  double distance = 0.0;
  for (unsigned int i = 0 ; i < pce_.size() ; ++i)
    {
      double dPos;
      pce_[i]->evaluate(ed->full_state, dPos);
      if (decision)
        (*decision)[i] = pce_[i]->decide(dPos);
      distance += dPos;
    }
  for (unsigned int i = 0 ; i < oce_.size() ; ++i)
    {
      double dAng;
      oce_[i]->evaluate(ed->full_state, dAng);
      if (decision)
        (*decision)[i] = pce_[i]->decide(dAng);
      distance += orientation_importance * dAng;
    }
    
  return distance;
}
	
double ompl_ros::GoalToMultipleConstraints::distanceGoal(const ompl::base::State *s) const
{
  return gp_.distanceGoal(s) + gs_.distanceGoal(s);
}

bool ompl_ros::GoalToMultipleConstraints::isSatisfied(const ompl::base::State *state, double *dist) const
{
  if (dist)
    {
      double d1, d2;
      bool res1 = gs_.isSatisfied(state, &d1);
      bool res2 = gp_.isSatisfied(state, &d2);
      *dist = d1 + d2;
      return res1 && res2;
    }
  else
    return gs_.isSatisfied(state) && gp_.isSatisfied(state);
}

unsigned int ompl_ros::GoalToMultipleConstraints::maxSampleCount(void) const
{
    return UINT_MAX;
}

void ompl_ros::GoalToMultipleConstraints::sampleGoal(ompl::base::State *s) const
{
  lock_.lock();
  sCore_->sampleNear(s, gs_.state, rho_);
  lock_.unlock();
}

void ompl_ros::GoalToMultipleConstraints::print(std::ostream &out) const
{
  gs_.print(out);
  gp_.print(out);
}

ompl::base::Goal* ompl_ros::computeGoalFromConstraints(ModelBase *model, const motion_planning_msgs::Constraints &kc)
{
  if (kc.joint_constraints.empty() && kc.position_constraints.empty())
    {
      ROS_ERROR("No goal constraints defined. No goal can be selected");
      return NULL;
    }
    
  if (kc.joint_constraints.size() > 0 && kc.position_constraints.empty() && kc.orientation_constraints.empty())
    // we have no pose constraints, only joint constraints
    // we compute the 'middle state' and plan towards it
    return new GoalToState(model, kc.joint_constraints);
    
  if (kc.joint_constraints.empty() && kc.position_constraints.size() > 0 && kc.orientation_constraints.size() > 0)
    // we have no joint constraints, only pose constraints
    // we know nothing of the state we want to get to (in terms of joint angles)
    return new GoalToPosition(model, kc.position_constraints, kc.orientation_constraints);
    
  // if we are doing kinematic planning, we have a special goal type we can use, that may speed things up a bit
  if (dynamic_cast<ompl::kinematic::SpaceInformationKinematic*>(model->si))
    return new GoalToMultipleConstraints(model, kc);
  else
    return new GoalToPosition(model, kc.position_constraints, kc.orientation_constraints);
}
