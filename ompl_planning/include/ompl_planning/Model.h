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

#ifndef OMPL_PLANNING_MODEL_
#define OMPL_PLANNING_MODEL_

#include "ompl_ros/ModelKinematic.h"
#include "ompl_ros/ModelDynamic.h"
#include "ompl_planning/planners/PlannerSetup.h"
#include "ompl_planning/PlannerConfig.h"

#include <boost/shared_ptr.hpp>
#include <string>
#include <map>

namespace ompl_planning
{
class Model
{
public:
	
  Model(planning_environment::CollisionModelsInterface* cmi, const std::string &gName, std::vector< boost::shared_ptr<PlannerConfig> >& cfgs)
  {
    collision_models_interface_ = cmi;
    groupName = gName;
    createMotionPlanningInstances(cfgs);
  }
  
  virtual ~Model(void)
  {
    for (std::map<std::string, PlannerSetup*>::iterator i = planners.begin(); i != planners.end() ; ++i)
      if (i->second)
        delete i->second;
  }
	
  planning_environment::CollisionModelsInterface *collision_models_interface_;
  std::string                            groupName;	
  std::map<std::string, PlannerSetup*>   planners;
	
protected:
	
  /** \brief Instantiate the planners that can be used  */
  void createMotionPlanningInstances(std::vector< boost::shared_ptr<PlannerConfig> >& cfgs);
	
  template<typename _T>
  void add_planner(boost::shared_ptr<PlannerConfig> &options);
	
};
    
typedef std::map<std::string, Model*> ModelMap;
    
/** \brief Create all the instances needed by OMPL using the planning parameters from the ROS server */
void setupPlanningModels(planning_environment::CollisionModelsInterface* cmi, ModelMap &models);

/** \brief Get a list of known models */
std::vector<std::string> knownModels(ModelMap &models);

/** \brief Free all allocated memory */
void destroyPlanningModels(ModelMap &models);
    
} // ompl_planning

#endif
