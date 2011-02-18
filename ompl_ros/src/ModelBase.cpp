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

#include "ompl_ros/ModelBase.h"
#include <boost/thread.hpp>
#include <map>

namespace ompl_ros
{
  static std::map<boost::thread::id, EnvironmentDescription*> ENVS;
  static boost::mutex                                         lockENVS;    
}

ompl_ros::ModelBase::ModelBase(planning_environment::CollisionModelsInterface* cmi, const std::string &gName)
{
  si = NULL;
  groupName = gName;
  collision_models_interface_ = cmi;
  group = collision_models_interface_->getKinematicModel()->getModelGroup(groupName);
  ROS_DEBUG("Create model for group %s", gName.c_str());
}

ompl_ros::ModelBase::~ModelBase(void)
{
  clearEnvironmentDescriptions();
  for (std::map<std::string, ompl::base::StateDistanceEvaluator*>::iterator j = sde.begin(); j != sde.end() ; ++j)
    if (j->second)
      delete j->second;
  if (si->getStateValidityChecker())
    delete si->getStateValidityChecker();
  if (si)
    delete si;
}

ompl_ros::EnvironmentDescription* ompl_ros::ModelBase::getEnvironmentDescription(void) const
{
  boost::thread::id id = boost::this_thread::get_id();
  EnvironmentDescription *result = NULL;
    
  lockENVS.lock();    
  std::map<boost::thread::id, EnvironmentDescription*>::iterator it = ENVS.find(id);
  if (it == ENVS.end())
  {
    if (ENVS.empty())
    {
      result = new EnvironmentDescription();
      result->kmodel = collision_models_interface_->getKinematicModel();
      if(collision_models_interface_->getPlanningSceneState() == NULL) {
        ROS_INFO("Trying to make environment description with no planning scene state");
      } else {
        if(collision_models_interface_->getPlanningSceneState() == NULL) {
          ROS_INFO_STREAM("Null state - going to be problems");
        }
        result->full_state = new planning_models::KinematicState(*collision_models_interface_->getPlanningSceneState());
        result->group_state = result->full_state->getJointStateGroup(group->getName());
      }
      result->constraintEvaluator = &constraintEvaluator;
    }
    else
    {
      ROS_DEBUG("Cloning collision environment (%d total)", (int)ENVS.size() + 1);
      result = new EnvironmentDescription();
      result->kmodel = collision_models_interface_->getKinematicModel();
      if(collision_models_interface_->getPlanningSceneState() == NULL) {
        ROS_INFO("Trying to make environment description with no planning scene state");
      } else {
        result->full_state = new planning_models::KinematicState(*collision_models_interface_->getPlanningSceneState());
        result->group_state = result->full_state->getJointStateGroup(group->getName());
      }
      planning_environment::KinematicConstraintEvaluatorSet *kce = new planning_environment::KinematicConstraintEvaluatorSet();
      kce->add(constraintEvaluator.getPositionConstraints());
      kce->add(constraintEvaluator.getOrientationConstraints());
      kce->add(constraintEvaluator.getJointConstraints());
      result->constraintEvaluator = kce;
    }
    ENVS[id] = result;
  }
  else
    result = it->second;
  lockENVS.unlock();
  return result;
}

void ompl_ros::ModelBase::clearEnvironmentDescriptions(void) const
{    
  lockENVS.lock();    
  for (std::map<boost::thread::id, EnvironmentDescription*>::iterator it = ENVS.begin() ; it != ENVS.end() ; ++it)
  {
    if (it->second->constraintEvaluator != &constraintEvaluator)
    {
      delete it->second->constraintEvaluator;
    }
    delete it->second->full_state;
    delete it->second;
  }
  ENVS.clear();
  lockENVS.unlock();
}
