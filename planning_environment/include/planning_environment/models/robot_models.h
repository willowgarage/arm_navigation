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

#ifndef PLANNING_ENVIRONMENT_MODELS_ROBOT_MODELS_
#define PLANNING_ENVIRONMENT_MODELS_ROBOT_MODELS_

#include <planning_models/kinematic_model.h>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

#include <map>
#include <string>
#include <vector>

namespace planning_environment
{
    
/** \brief A class capable of loading a robot model from the parameter server */
    
class RobotModels
{
public:
		
  RobotModels(const std::string &description);

  RobotModels(boost::shared_ptr<urdf::Model> urdf,
              planning_models::KinematicModel* kmodel);

  virtual ~RobotModels(void)
  {
    delete kmodel_;
  }
       
  /** \brief Return the name of the description */
  const std::string &getDescription(void) const
  {
    return description_;
  }
	
  /** \brief Return the instance of the constructed kinematic model */
  const planning_models::KinematicModel* getKinematicModel(void) const
  {
    return kmodel_;
  }

  /** \brief Return the instance of the parsed robot description */
  const boost::shared_ptr<urdf::Model> &getParsedDescription(void) const
  {
    return urdf_;
  }

  /** \brief Return true if models have been loaded */
  bool loadedModels(void) const
  {
    return loaded_models_;
  }

  /** \brief Reload the robot description and recreate the model */
  virtual void reload(void);
	
  //new functions from the monitors

  /** \brief Return the frame id of the state */
  const std::string& getRobotFrameId(void) const
  {
    return kmodel_->getRoot()->getChildLinkModel()->getName();
  }

  /** \brief Return the world frame id */
  const std::string& getWorldFrameId(void) const
  {
    return kmodel_->getRoot()->getParentFrameId();
  }

  std::string getRobotName(void) const 
  {
    return urdf_->getName();
  }

protected:
	
  void loadRobotFromParamServer(void);

  bool loadMultiDofConfigsFromParamServer(std::vector<planning_models::KinematicModel::MultiDofConfig>& configs);
  void loadGroupConfigsFromParamServer(const std::vector<planning_models::KinematicModel::MultiDofConfig>& multi_dof_configs,
                                       std::vector<planning_models::KinematicModel::GroupConfig>& configs);
	
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;
	  
  std::string description_;
	
  bool loaded_models_;
  planning_models::KinematicModel* kmodel_;
  
  boost::shared_ptr<urdf::Model> urdf_;  
};
    
}

#endif

