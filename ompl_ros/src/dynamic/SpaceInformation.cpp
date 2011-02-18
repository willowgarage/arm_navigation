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

#include "ompl_ros/dynamic/SpaceInformation.h"
#include "ompl_ros/dynamic/StateValidator.h"
#include "ompl_ros/dynamic/extensions/PR2PropagationModels.h"
#include <ros/console.h>

void ompl_ros::ROSSpaceInformationDynamic::configureOMPLSpace(ModelBase *model)
{	   
  kmodel_ = model->collision_models_interface_->getKinematicModel();
  groupName_ = model->groupName;
  propagationModel_ = NULL;
    
  planning_models::KinematicState state(kmodel_);
  const planning_models::KinematicState::JointStateGroup* group_state = state.getJointStateGroup(groupName_);

  const std::map<std::string, unsigned int>& map_index = group_state->getKinematicStateIndexMap();
  /* compute the state space for this group */
  m_stateDimension = map_index.size();
  m_stateComponent.resize(m_stateDimension);

  for (unsigned int i = 0 ; i < group_state->getJointStateVector().size(); ++i)
  {	
    const std::map<std::string, std::pair<double,double> >& bounds = group_state->getJointStateVector()[i]->getAllJointValueBounds();
    for(std::map<std::string, std::pair<double,double> >::const_iterator it = bounds.begin();
        it != bounds.end();
        it++) {
      m_stateComponent[map_index.at(it->first)].minValue = it->second.first;
      m_stateComponent[map_index.at(it->first)].maxValue = it->second.second;
    }

    unsigned int k = map_index.at(group_state->getJointStateVector()[i]->getName());
    
    if (m_stateComponent[i].type == ompl::base::StateComponent::UNKNOWN)
    {
      const planning_models::KinematicModel::RevoluteJointModel *rj = 
        dynamic_cast<const planning_models::KinematicModel::RevoluteJointModel*>(group_state->getJointStateVector()[i]->getJointModel());
      if (rj && rj->continuous_)
        m_stateComponent[k].type = ompl::base::StateComponent::WRAPPING_ANGLE;
      else
        m_stateComponent[k].type = ompl::base::StateComponent::LINEAR;
    }
    
    if (dynamic_cast<const planning_models::KinematicModel::FloatingJointModel*>(group_state->getJointStateVector()[i]->getJointModel()))
    {
      floatingJoints_.push_back(k);
      m_stateComponent[k + 3].type = ompl::base::StateComponent::QUATERNION;
      m_stateComponent[k + 4].type = ompl::base::StateComponent::QUATERNION;
      m_stateComponent[k + 5].type = ompl::base::StateComponent::QUATERNION;
      m_stateComponent[k + 6].type = ompl::base::StateComponent::QUATERNION;
      break;
    }
    
    if (dynamic_cast<const planning_models::KinematicModel::PlanarJointModel*>(group_state->getJointStateVector()[i]->getJointModel()))
    {
      planarJoints_.push_back(k);
      m_stateComponent[k + 2].type = ompl::base::StateComponent::WRAPPING_ANGLE;
      break;		    
    }
  }
    
  // create a backup of this, in case it gets bound by joint constraints
  basicStateComponent_ = m_stateComponent;
  checkBounds();
    
  if (model->groupName == "base")
    propagationModel_ = new PR2BaseModel(this);
    
  assert(propagationModel_);
  propagationModel_->controlDefinition(m_controlComponent, &m_controlDimension, 
                                       &m_minControlDuration, &m_maxControlDuration, &m_resolution);
    
  m_stateForwardPropagator = propagationModel_;
}

void ompl_ros::ROSSpaceInformationDynamic::setPlanningVolume(double x0, double y0, double z0, double x1, double y1, double z1)
{
  for (unsigned int i = 0 ; i < floatingJoints_.size() ; ++i)
    {
      int id = floatingJoints_[i];		
      m_stateComponent[id    ].minValue = x0;
      m_stateComponent[id    ].maxValue = x1;
      m_stateComponent[id + 1].minValue = y0;
      m_stateComponent[id + 1].maxValue = y1;
      m_stateComponent[id + 2].minValue = z0;
      m_stateComponent[id + 2].maxValue = z1;
    }
  checkBounds();    
}

void ompl_ros::ROSSpaceInformationDynamic::setPlanningArea(double x0, double y0, double x1, double y1)
{
  for (unsigned int i = 0 ; i < planarJoints_.size() ; ++i)
    {
      int id = planarJoints_[i];		
      m_stateComponent[id    ].minValue = x0;
      m_stateComponent[id    ].maxValue = x1;
      m_stateComponent[id + 1].minValue = y0;
      m_stateComponent[id + 1].maxValue = y1;
    }
  checkBounds();    
}

void ompl_ros::ROSSpaceInformationDynamic::clearPathConstraints(void)
{
  m_stateComponent = basicStateComponent_;
  ROSStateValidityPredicateDynamic *svp = dynamic_cast<ROSStateValidityPredicateDynamic*>(getStateValidityChecker());
  svp->clearConstraints();
}

void ompl_ros::ROSSpaceInformationDynamic::setPathConstraints(const motion_planning_msgs::Constraints &kc)
{    
  const std::vector<motion_planning_msgs::JointConstraint> &jc = kc.joint_constraints;  
  
  planning_models::KinematicState state(kmodel_);
  const planning_models::KinematicState::JointStateGroup* group_state = state.getJointStateGroup(groupName_);
  
  // tighten the bounds based on the constraints
  for (unsigned int i = 0 ; i < jc.size() ; ++i)
    {
      const std::map<std::string, unsigned int>& all_group_order = group_state->getKinematicStateIndexMap();
      
      if(all_group_order.find(jc[i].joint_name) != all_group_order.end()) {
        int idx = all_group_order.find(jc[i].joint_name)->second;
        if (m_stateComponent[idx].minValue < jc[i].position - jc[i].tolerance_below)
          m_stateComponent[idx].minValue = jc[i].position - jc[i].tolerance_below;
        if (m_stateComponent[idx].maxValue > jc[i].position + jc[i].tolerance_above)
          m_stateComponent[idx].maxValue = jc[i].position + jc[i].tolerance_above;
      }
    }
  checkBounds();
  motion_planning_msgs::Constraints temp_kc = kc;
  temp_kc.joint_constraints.clear();
  ROSStateValidityPredicateDynamic *svp = dynamic_cast<ROSStateValidityPredicateDynamic*>(getStateValidityChecker());
  svp->setConstraints(temp_kc);
}

bool ompl_ros::ROSSpaceInformationDynamic::checkBounds(void)
{
  // check if joint bounds are feasible
  bool valid = true;
  for (unsigned int i = 0 ; i < m_stateDimension ; ++i)
    if (m_stateComponent[i].minValue > m_stateComponent[i].maxValue)
      {
        valid = false;
        ROS_ERROR("Inconsistent set of joint constraints imposed on path at index %d. Sampling will not find any valid states between %f and %f", i,m_stateComponent[i].minValue, m_stateComponent[i].maxValue);
        break;
      }
  return valid;
}
