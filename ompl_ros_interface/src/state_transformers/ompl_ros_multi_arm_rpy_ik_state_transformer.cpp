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

#include <ompl_ros_interface/state_transformers/ompl_ros_multi_arm_rpy_ik_state_transformer.h>

namespace ompl_ros_interface
{

bool OmplRosMultiArmRPYIKStateTransformer::initialize()
{
  srand ( time(NULL) ); // initialize random seed
  seed_states_.resize(arm_names_.size());
  solution_states_.resize(arm_names_.size());
  end_effector_offsets_.resize(arm_names_.size());
  ompl_state_to_robot_state_mappings_.resize(arm_names_.size());
  for(unsigned int i=0; i < arm_names_.size(); i++)
  {
    seed_states_[i].joint_state.name = kinematics_solvers_[i]->getJointNames();
    seed_states_[i].joint_state.position.resize(kinematics_solvers_[i]->getJointNames().size());
    //arm_navigation_msgs::printJointState(seed_state_.joint_state);

    solution_states_[i].joint_state.name = kinematics_solvers_[i]->getJointNames();
    solution_states_[i].joint_state.position.resize(kinematics_solvers_[i]->getJointNames().size());
    //arm_navigation_msgs::printJointState(solution_state_.joint_state);
  }
    
  real_vector_index_ = state_space_->as<ompl::base::CompoundStateSpace>()->getSubSpaceIndex("real_vector");

  if(real_vector_index_ > -1)
  {
    x_index_ = state_space_->as<ompl::base::CompoundStateSpace>()->as<ompl::base::RealVectorStateSpace>(real_vector_index_)->getDimensionIndex("x");
    y_index_ = state_space_->as<ompl::base::CompoundStateSpace>()->as<ompl::base::RealVectorStateSpace>(real_vector_index_)->getDimensionIndex("y");
    z_index_ = state_space_->as<ompl::base::CompoundStateSpace>()->as<ompl::base::RealVectorStateSpace>(real_vector_index_)->getDimensionIndex("z");
    
    pitch_index_ = state_space_->as<ompl::base::CompoundStateSpace>()->as<ompl::base::RealVectorStateSpace>(real_vector_index_)->getDimensionIndex("pitch");
    roll_index_ = state_space_->as<ompl::base::CompoundStateSpace>()->as<ompl::base::RealVectorStateSpace>(real_vector_index_)->getDimensionIndex("roll");
    yaw_index_ = state_space_->as<ompl::base::CompoundStateSpace>()->as<ompl::base::RealVectorStateSpace>(real_vector_index_)->getDimensionIndex("yaw");
  }
  else
  {
    ROS_ERROR("Could not find real vector state space");
    return false;
  }
  /*Map any real joints onto the actual physical joints */
  scoped_state_.reset(new ompl::base::ScopedState<ompl::base::CompoundStateSpace>(state_space_));

  for(unsigned int i=0; i < arm_names_.size(); i++)
  {
    if(!ompl_ros_interface::getOmplStateToRobotStateMapping(*scoped_state_,seed_states_[i],ompl_state_to_robot_state_mappings_[i],false))
    {
      ROS_ERROR("Could not get mapping between ompl state and robot state");
      return false;
    }
  }
  return true;
}

bool OmplRosMultiArmRPYIKStateTransformer::configureOnRequest(const arm_navigation_msgs::GetMotionPlan::Request &request,
                                                              arm_navigation_msgs::GetMotionPlan::Response &response)
{  
  return true;
}

bool OmplRosMultiArmRPYIKStateTransformer::inverseTransform(const ompl::base::State &ompl_state,
                                                            arm_navigation_msgs::RobotState &robot_state)
{
  geometry_msgs::Pose pose;
  tf::Pose pose_tf;
  omplStateToPose(ompl_state,pose);
  tf::poseMsgToTF(pose,pose_tf);

  (*scoped_state_) = ompl_state;
  int error_code;
  //  unsigned int joint_count(0);
  for(unsigned int i=0; i < arm_names_.size(); i++)
  {
    geometry_msgs::Pose end_effector_pose;
    generateRandomState(seed_states_[i]);    
    ompl_ros_interface::omplStateToRobotState(*scoped_state_,ompl_state_to_robot_state_mappings_[i],seed_states_[i]);
    tf::poseTFToMsg(pose_tf*end_effector_offsets_[i],end_effector_pose);

    ROS_DEBUG_STREAM("Inner pose is " << i << "::" << 
                     end_effector_pose.position.x << " " <<
                     end_effector_pose.position.y << " " <<
                     end_effector_pose.position.z << " " <<
                     end_effector_pose.orientation.x << " " << 
                     end_effector_pose.orientation.y << " " << 
                     end_effector_pose.orientation.z << " " << 
                     end_effector_pose.orientation.w);

    if(kinematics_solvers_[i]->searchPositionIK(end_effector_pose,
                                                seed_states_[i].joint_state.position,
                                                1.0,
                                                solution_states_[i].joint_state.position,
                                                error_code))
    {
      for(unsigned int j=0; j < solution_states_[i].joint_state.position.size(); j++)
      {
        robot_state.joint_state.position.push_back(solution_states_[i].joint_state.position[j]);
        robot_state.joint_state.name.push_back(solution_states_[i].joint_state.name[j]);
        //        joint_count++;
      }
      return true;
    }
  }
  return false;
}

bool OmplRosMultiArmRPYIKStateTransformer::forwardTransform(const arm_navigation_msgs::RobotState &joint_state,
                                                    ompl::base::State &ompl_state)
{
  return true;
}

void OmplRosMultiArmRPYIKStateTransformer::omplStateToPose(const ompl::base::State &ompl_state,
                                                           geometry_msgs::Pose &pose)
{
  
  btVector3 tmp_pos(ompl_state.as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(real_vector_index_)->values[x_index_],
                    ompl_state.as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(real_vector_index_)->values[y_index_],
                    ompl_state.as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(real_vector_index_)->values[z_index_]);
  btQuaternion tmp_rot;
  tmp_rot.setRPY(ompl_state.as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(real_vector_index_)->values[roll_index_],
                 ompl_state.as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(real_vector_index_)->values[pitch_index_],
                 ompl_state.as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(real_vector_index_)->values[yaw_index_]);
  btTransform tmp_transform(tmp_rot,tmp_pos);
  tf::poseTFToMsg(tmp_transform,pose);  
}


arm_navigation_msgs::RobotState OmplRosMultiArmRPYIKStateTransformer::getDefaultState()
{
  arm_navigation_msgs::RobotState robot_state;
  if(!kinematics_solvers_.empty())
  {
    for(unsigned int i=0; i < kinematics_solvers_.size(); i++)
    {
      if(kinematics_solvers_[i])
      {
        std::vector<std::string> joint_names = kinematics_solvers_[i]->getJointNames();
        for(unsigned int j=0; j < joint_names.size(); j++)
        {
          robot_state.joint_state.name.push_back(joint_names[j]);
        }
      }
      else
      {
        ROS_ERROR("Kinematics solver not defined");
        return robot_state;
      }
    }
    generateRandomState(robot_state);
  }
  return robot_state;
}

void OmplRosMultiArmRPYIKStateTransformer::generateRandomState(arm_navigation_msgs::RobotState &robot_state)
{
  std::vector<const planning_models::KinematicModel::JointModel*> joint_models = physical_joint_model_group_->getJointModels();
  if(robot_state.joint_state.position.empty())
    robot_state.joint_state.position.resize(robot_state.joint_state.name.size());
  for(unsigned int i=0; i < robot_state.joint_state.name.size(); i++)
  {
    std::pair<double,double> bounds;
    for(unsigned int j=0; j < joint_models.size(); ++j)
    {
      if(robot_state.joint_state.name[i] == joint_models[j]->getName())
      {
        joint_models[j]->getVariableBounds(robot_state.joint_state.name[i],bounds);
        robot_state.joint_state.position[i] = generateRandomNumber(bounds.first,bounds.second);    
      }
    }
  }
}

double OmplRosMultiArmRPYIKStateTransformer::generateRandomNumber(const double &min, const double &max)
{
  int rand_num = rand()%100+1;
  double result = min + (double)((max-min)*rand_num)/101.0;
  return result;
}

void OmplRosMultiArmRPYIKStateTransformer::setEndEffectorOffsets(const std::vector<tf::Pose> &end_effector_offsets)
{
  end_effector_offsets_ = end_effector_offsets;
}


}
