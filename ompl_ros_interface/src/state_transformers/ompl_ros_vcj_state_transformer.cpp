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

#include <ompl_ros_interface/state_transformers/ompl_ros_vcj_state_transformer.h>

namespace ompl_ros_interface
{
OmplRosVCJStateTransformer::OmplRosVCJStateTransformer(const ompl::base::StateSpacePtr &state_space,
                                                       const planning_models::KinematicModel::JointModelGroup* physical_joint_model_group) : OmplRosStateTransformer(state_space, physical_joint_model_group)

{
  ros::NodeHandle node_handle("~");
  group_name_ = state_space_->getName();
  physical_group_name_ = physical_joint_model_group_->getName();
  real_vector_index_ = state_space_->as<ompl::base::CompoundStateSpace>()->getSubSpaceIndex("real_vector");
}  

bool OmplRosVCJStateTransformer::inverseTransform(const ompl::base::State &ompl_state,
                                                  arm_navigation_msgs::RobotState &robot_state)
{
  robot_state = getDefaultState();
  for(unsigned int i=0; i < constrained_joints_.size(); i++)
  {
    robot_state.joint_state.position[constrained_joints_[i].first] = ompl_state.as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(real_vector_index_)->values[mimic_joints_[i]];
    robot_state.joint_state.position[constrained_joints_[i].second] = ompl_state.as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(real_vector_index_)->values[mimic_joints_[i]];
  }
  for(unsigned int i=0; i < robot_state.joint_state.position.size(); i++)
  {
    if(regular_joints_[i] >= 0)
    {
      robot_state.joint_state.position[i] = ompl_state.as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(real_vector_index_)->values[regular_joints_[i]];
    }
  }
  return true;
}

bool OmplRosVCJStateTransformer::forwardTransform(const arm_navigation_msgs::RobotState &robot_state,
                                                  ompl::base::State &ompl_state)
{
  for(unsigned int i=0; i < constrained_joints_.size(); i++)
    ompl_state.as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(real_vector_index_)->values[mimic_joints_[i]] = robot_state.joint_state.position[constrained_joints_[i].first]; 
  for(unsigned int i=0; i < robot_state.joint_state.position.size(); i++)
  {
    if(regular_joints_[i] >= 0)
    {
      ompl_state.as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(real_vector_index_)->values[regular_joints_[i]] = robot_state.joint_state.position[i];
    }
  }
  return true;
}

arm_navigation_msgs::RobotState OmplRosVCJStateTransformer::getDefaultState()
{
  arm_navigation_msgs::RobotState robot_state;
  robot_state.joint_state.name = physical_joint_model_group_->getJointModelNames();
  robot_state.joint_state.position.resize(robot_state.joint_state.name.size());
  return robot_state;
}

bool OmplRosVCJStateTransformer::configureOnRequest(const arm_navigation_msgs::GetMotionPlan::Request &request,
                                                    arm_navigation_msgs::GetMotionPlan::Response &response)
{
  return true;
}

bool OmplRosVCJStateTransformer::initialize()
{
  ros::NodeHandle private_handle("~"+group_name_);
  private_handle.param<std::string>("parent_frame",parent_frame_,"base");
  if(!getConstrainedJoints(group_name_,constrained_joints_))
    return false;
  if(!setConstrainedJoints(constrained_joints_))
    return false;
  return true;
}

bool OmplRosVCJStateTransformer::getConstrainedJoints(const std::string &name, std::vector<std::pair<unsigned int, unsigned int> > &constrained_joints)
{
  //Get mimic joint information
  ros::NodeHandle private_handle("~"+name);
  if (private_handle.hasParam("mimic_joints")) 
  {
    ROS_DEBUG("Finding mimic joints");
    XmlRpc::XmlRpcValue group_list;  
    std::vector<unsigned int> mimic_joints;
    private_handle.getParam("mimic_joints", group_list);
    if(group_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Group list should be of XmlRpc Array type");
      return false;
    } 
    std::vector<std::string> joint_names =  physical_joint_model_group_->getJointModelNames();
    for (unsigned int i = 0; i < (unsigned int) group_list.size(); ++i) 
    {
      if(group_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
      {
        ROS_ERROR("Group names should be strings");
        return false;
      }
      std::string mimic_joint = static_cast<std::string>(group_list[i]);
      ROS_DEBUG("Mimic joint: %s",mimic_joint.c_str());
      int joint_index = -1;
      for(unsigned int j=0; j < joint_names.size(); j++)
      {
        if(mimic_joint == joint_names[j])
        { 
          joint_index = j;
          break;
        }
      }
      if(joint_index < 0)
      {
        return false;
      }
      mimic_joints.push_back(joint_index);
      ROS_DEBUG("Joint index for mimic joint: %d",joint_index);
    }

    for(unsigned int i=0; i < mimic_joints.size(); i+=2)
    {
      if( (i+1) >= mimic_joints.size())
        break;
      std::pair<unsigned int, unsigned int> jc;
      jc.first = mimic_joints[i];
      jc.second = mimic_joints[i+1];
      ROS_INFO("Pushing back %d %d",jc.first,jc.second);
      constrained_joints.push_back(jc);
    }
  } 
  return true;
}

bool OmplRosVCJStateTransformer::setConstrainedJoints(const std::vector<std::pair<unsigned int, unsigned int> > &constrained_joints)
{
  unsigned int joint_counter = 0;
  unsigned int num_mimic_joints = 0;
  regular_joints_.resize(physical_joint_model_group_->getJointModelNames().size(),-1);
  mimic_joints_.resize(constrained_joints_.size(),-1);
  while(joint_counter < physical_joint_model_group_->getJointModelNames().size())
  {
    bool is_mimic_joint = false;
    for(unsigned int j=0; j < constrained_joints.size(); j++)
    {
      if(joint_counter == constrained_joints[j].first || joint_counter == constrained_joints[j].second)
      {
        mimic_joints_[j] = joint_counter-num_mimic_joints;
        regular_joints_[joint_counter] = -1;
        regular_joints_[joint_counter+1] = -1;
        joint_counter += 2;
        num_mimic_joints++;
        is_mimic_joint = true;
        break;
      }
    }
    if(!is_mimic_joint)
    {
      regular_joints_[joint_counter] = joint_counter-num_mimic_joints;
      joint_counter++;
    }
  }
  return true;
}

}
