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

ompl_ros::ModelBase::ModelBase(planning_environment::PlanningMonitor *pMonitor, const std::string &gName)
{
  si = NULL;
  groupName = gName;
  planningMonitor = pMonitor;
  group = planningMonitor->getKinematicModel()->getGroup(groupName);
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
          result->collisionSpace = planningMonitor->getEnvironmentModel();
          result->kmodel = result->collisionSpace->getRobotModel().get();
          result->constraintEvaluator = &constraintEvaluator;
          result->group = group;
        }
      else
        {
          ROS_DEBUG("Cloning collision environment (%d total)", (int)ENVS.size() + 1);
          result = new EnvironmentDescription();
          result->collisionSpace = planningMonitor->getEnvironmentModel()->clone();
          result->kmodel = result->collisionSpace->getRobotModel().get();
          result->group = result->kmodel->getGroup(groupName);
          planning_environment::KinematicConstraintEvaluatorSet *kce = new planning_environment::KinematicConstraintEvaluatorSet();
          kce->add(result->kmodel, constraintEvaluator.getPositionConstraints());
          kce->add(result->kmodel, constraintEvaluator.getOrientationConstraints());
          kce->add(result->kmodel, constraintEvaluator.getJointConstraints());
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
      if (it->second->collisionSpace != planningMonitor->getEnvironmentModel())
        {
          delete it->second->collisionSpace;
          delete it->second->constraintEvaluator;
        }
      delete it->second;
    }
  ENVS.clear();
  lockENVS.unlock();
}
