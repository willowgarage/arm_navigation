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

#include "ik_constrained_planner/ik_state_validator.h"

namespace ik_constrained_planner
{    
bool IKStateValidator::operator()(const ompl::base::State *s) const
{
  int test = planning_environment::PlanningMonitor::COLLISION_TEST;  

  btVector3 tmp_pos(s->values[0],s->values[1],s->values[2]);
  btQuaternion tmp_rot;
  tmp_rot.setRPY(s->values[3],s->values[4],s->values[5]);
  btTransform tmp_transform(tmp_rot,tmp_pos);
  btTransform result = kinematics_planner_tf_*tmp_transform;
  geometry_msgs::Pose pose;
  tf::poseTFToMsg(result,pose);

  std::vector<double> solution;
  std::vector<double> seed;
  seed.resize(space_information_->getStateDimension(),0.0);
  seed[redundant_joint_index_] = s->values[6];

  if(!kinematics_solver_->getPositionIK(pose,seed,solution))
    return false;
  kinematic_state_->setParamsGroup(solution,group_name_);
  bool valid = planning_monitor_->isStateValid(kinematic_state_,test,false);  
  return valid;    
}

void IKStateValidator::printSettings(std::ostream &out) const
{    
  out << "Path constraints:" << std::endl;
}

void IKStateValidator::configure(const std::string &group_name, 
                                 const std::string &redundant_joint_name,                      
                                 const geometry_msgs::Pose &kinematics_planner_offset,
                                 kinematics::KinematicsBase *kinematics_solver)
{
  kinematics_solver_ = kinematics_solver;
  group_name_ = group_name;
  std::vector<std::string> joint_names = kinematics_solver_->getJointNames();
  for(unsigned int i=0; i < joint_names.size(); ++i)
  {
    if(joint_names[i] == redundant_joint_name)
    {
      redundant_joint_index_ = i;
      break;
    }
  }
  btTransform tmp;
  tf::poseMsgToTF(kinematics_planner_offset,tmp);
  kinematics_planner_tf_ = tmp;
}

}
