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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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

#include <ompl_ros_interface/state_transformers/ompl_ros_multi_arm_ik_state_transformer.h>

namespace ompl_ros_interface
{
OmplRosMultiArmIKStateTransformer::OmplRosMultiArmIKStateTransformer(const ompl::base::StateSpacePtr &state_space,
                                                                     const planning_models::KinematicModel::JointModelGroup* physical_joint_model_group) : OmplRosStateTransformer(state_space, physical_joint_model_group), kinematics_loader_("kinematics_base","kinematics::KinematicsBase")

{
  ros::NodeHandle node_handle("~");
  std::string kinematics_solver_name;

  group_name_ = state_space_->getName();
  physical_group_name_ = physical_joint_model_group_->getName();

  XmlRpc::XmlRpcValue arm_list;
  if(!node_handle.getParam(group_name_+"/arm_names", arm_list))
  {
    ROS_ERROR("Could not find parameter %s on param server in namespace %s",(group_name_+"/arm_names").c_str(),node_handle.getNamespace().c_str());
    throw new OMPLROSException();
  }
  ROS_ASSERT(arm_list.getType() == XmlRpc::XmlRpcValue::TypeArray);    
  kinematics_solvers_.resize(arm_list.size());
  for (int32_t j = 0; j < arm_list.size(); ++j) 
  {
    if(arm_list[j].getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_ERROR("Arm names must be of type string");
      throw new OMPLROSException();
    }
    std::string arm_name = static_cast<std::string>(arm_list[j]);
    arm_names_.push_back(arm_name);

    std::string solver_name;
    if(!node_handle.getParam(group_name_+"/"+arm_name+"/kinematics_solver", solver_name))
    {
      ROS_ERROR("Could not find parameter %s on param server in namespace %s",(group_name_+arm_name+"/kinematics_solver").c_str(),node_handle.getNamespace().c_str());
      throw new OMPLROSException();
    }
    kinematics_solver_names_.push_back(solver_name);

    std::string tip_name;
    if(!node_handle.getParam(group_name_+"/"+arm_name+"/tip_name", tip_name))
    {
      ROS_ERROR("Could not find parameter %s on param server in namespace %s",(group_name_+arm_name+"/tip_name").c_str(),node_handle.getNamespace().c_str());
      throw new OMPLROSException();
    }
    end_effector_names_.push_back(tip_name);

    if(!kinematics_loader_.isClassAvailable(solver_name))
    {
      ROS_ERROR("pluginlib does not have the class %s",solver_name.c_str());
      throw new OMPLROSException();
    }
    ROS_DEBUG("Found solver %s",solver_name.c_str());      
    try
    {
      kinematics_solvers_[j] = kinematics_loader_.createClassInstance(solver_name);
    }
    catch(pluginlib::PluginlibException& ex)    //handle the class failing to load
    {
      ROS_ERROR("The plugin failed to load. Error: %s", ex.what());
      throw new OMPLROSException();
    }
    ROS_DEBUG("Loaded solver %s",solver_name.c_str());
    if(!kinematics_solvers_[j]->initialize(arm_name))
    {
      ROS_ERROR("Could not initialize kinematics solver for group %s",arm_name.c_str());
      throw new OMPLROSException();
    }
  }
  ROS_DEBUG("Initialized ompl ros state transformer %s",group_name_.c_str());
}
}
