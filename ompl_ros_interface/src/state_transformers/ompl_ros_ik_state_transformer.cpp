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

#include <ompl_ros_interface/state_transformers/ompl_ros_ik_state_transformer.h>

namespace ompl_ros_interface
{
OmplRosIKStateTransformer::OmplRosIKStateTransformer(const ompl::base::StateSpacePtr &state_space,
                                                     const planning_models::KinematicModel::JointModelGroup* physical_joint_model_group) : OmplRosStateTransformer(state_space, physical_joint_model_group), kinematics_loader_("kinematics_base","kinematics::KinematicsBase")

{
  ros::NodeHandle node_handle("~");
  std::string kinematics_solver_name;
  group_name_ = state_space_->getName();
  physical_group_name_ = physical_joint_model_group_->getName();
  std::string base_name, tip_name;

  if(!node_handle.hasParam(group_name_+"/kinematics_solver"))
  {
    ROS_ERROR("Kinematics solver not defined for group %s in namespace %s",group_name_.c_str(),node_handle.getNamespace().c_str());
    throw new OMPLROSException();
  }
  node_handle.getParam(group_name_+"/kinematics_solver",kinematics_solver_name);

  if(!node_handle.hasParam(group_name_+"/root_name"))
  {
    ROS_ERROR_STREAM("Kinematics solver has no root name " << base_name << " in param ns " << node_handle.getNamespace());
    throw new OMPLROSException();
  }
  node_handle.getParam(group_name_+"/root_name",base_name);

  if(!node_handle.hasParam(group_name_+"/tip_name"))
  {
    ROS_ERROR_STREAM("Kinematics solver has no root name " << tip_name << " in param ns " << node_handle.getNamespace());
    throw new OMPLROSException();
  }
  node_handle.getParam(group_name_+"/tip_name",tip_name);


  ROS_DEBUG("Trying to initialize solver %s",kinematics_solver_name.c_str());
  if(!kinematics_loader_.isClassAvailable(kinematics_solver_name))
  {
    ROS_ERROR("pluginlib does not have the class %s",kinematics_solver_name.c_str());
    throw new OMPLROSException();
  }
  ROS_DEBUG("Found solver %s",kinematics_solver_name.c_str());
  
  try
  {
    kinematics_solver_ = kinematics_loader_.createClassInstance(kinematics_solver_name);
  }
  catch(pluginlib::PluginlibException& ex)    //handle the class failing to load
  {
    ROS_ERROR("The plugin failed to load. Error: %s", ex.what());
    throw new OMPLROSException();
  }
  ROS_DEBUG("Loaded solver %s",kinematics_solver_name.c_str());
  if(!kinematics_solver_->initialize(group_name_,
                                     base_name,
                                     tip_name,
                                     .01))
  {
    ROS_ERROR("Could not initialize kinematics solver for group %s",group_name_.c_str());
    throw new OMPLROSException();
  }
  ROS_DEBUG("Initialized ompl ros state transformer %s",kinematics_solver_name.c_str());
}  
}
