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

#include <ompl_ros_interface/state_transformers/ompl_ros_rpy_ik_state_transformer.h>

namespace ompl_ros_interface
{

bool OmplRosRPYIKStateTransformer::initialize()
{
  seed_state_.joint_state.name = kinematics_solver_->getJointNames();
  seed_state_.joint_state.position.resize(kinematics_solver_->getJointNames().size());
  //  motion_planning_msgs::printJointState(seed_state_.joint_state);

  solution_state_.joint_state.name = kinematics_solver_->getJointNames();
  solution_state_.joint_state.position.resize(kinematics_solver_->getJointNames().size());
  //  motion_planning_msgs::printJointState(solution_state_.joint_state);

  real_vector_index_ = state_manifold_->as<ompl::base::CompoundStateManifold>()->getSubManifoldIndex("real_vector");

  if(real_vector_index_ > -1)
  {
    x_index_ = state_manifold_->as<ompl::base::CompoundStateManifold>()->as<ompl::base::RealVectorStateManifold>(real_vector_index_)->getDimensionIndex("x");
    y_index_ = state_manifold_->as<ompl::base::CompoundStateManifold>()->as<ompl::base::RealVectorStateManifold>(real_vector_index_)->getDimensionIndex("y");
    z_index_ = state_manifold_->as<ompl::base::CompoundStateManifold>()->as<ompl::base::RealVectorStateManifold>(real_vector_index_)->getDimensionIndex("z");
    
    pitch_index_ = state_manifold_->as<ompl::base::CompoundStateManifold>()->as<ompl::base::RealVectorStateManifold>(real_vector_index_)->getDimensionIndex("pitch");
    roll_index_ = state_manifold_->as<ompl::base::CompoundStateManifold>()->as<ompl::base::RealVectorStateManifold>(real_vector_index_)->getDimensionIndex("roll");
    yaw_index_ = state_manifold_->as<ompl::base::CompoundStateManifold>()->as<ompl::base::RealVectorStateManifold>(real_vector_index_)->getDimensionIndex("yaw");
  }
  else
  {
    ROS_ERROR("Could not find real vector manifold");
    return false;
  }
  /*Map any real joints onto the actual physical joints */
  scoped_state_.reset(new ompl::base::ScopedState<ompl::base::CompoundStateManifold>(state_manifold_));
  if(!ompl_ros_interface::getOmplStateToRobotStateMapping(*scoped_state_,seed_state_,ompl_state_to_robot_state_mapping_,false))
  {
    ROS_ERROR("Could not get mapping between ompl state and robot state");
    return false;
  }
  return true;
}

bool OmplRosRPYIKStateTransformer::configureOnRequest(const motion_planning_msgs::GetMotionPlan::Request &request,
                                                      motion_planning_msgs::GetMotionPlan::Response &response)
{  

  return true;
}

bool OmplRosRPYIKStateTransformer::inverseTransform(const ompl::base::State &ompl_state,
                                                    motion_planning_msgs::RobotState &robot_state)
{
  geometry_msgs::Pose pose;
  omplStateToPose(ompl_state,pose);
  (*scoped_state_) = ompl_state;
  ompl_ros_interface::omplStateToRobotState(*scoped_state_,ompl_state_to_robot_state_mapping_,seed_state_);
  int error_code;

  ROS_DEBUG_STREAM("Inner pose is " <<
                   pose.position.x << " " <<
                   pose.position.y << " " <<
                   pose.position.z << " " <<
                   pose.orientation.x << " " << 
                   pose.orientation.y << " " << 
                   pose.orientation.z << " " << 
                   pose.orientation.w);

  if(kinematics_solver_->getPositionIK(pose,
                                       seed_state_.joint_state.position,
                                       solution_state_.joint_state.position,
				       error_code))
  {
    robot_state.joint_state = solution_state_.joint_state;
    return true;
  }
  return false;
}

bool OmplRosRPYIKStateTransformer::forwardTransform(const motion_planning_msgs::RobotState &joint_state,
                                                    ompl::base::State &ompl_state)
{
  return true;
}

void OmplRosRPYIKStateTransformer::omplStateToPose(const ompl::base::State &ompl_state,
                                                   geometry_msgs::Pose &pose)
{
  
  btVector3 tmp_pos(ompl_state.as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateManifold::StateType>(real_vector_index_)->values[x_index_],
                    ompl_state.as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateManifold::StateType>(real_vector_index_)->values[y_index_],
                    ompl_state.as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateManifold::StateType>(real_vector_index_)->values[z_index_]);
  btQuaternion tmp_rot;
  tmp_rot.setRPY(ompl_state.as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateManifold::StateType>(real_vector_index_)->values[roll_index_],
                 ompl_state.as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateManifold::StateType>(real_vector_index_)->values[pitch_index_],
                 ompl_state.as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateManifold::StateType>(real_vector_index_)->values[yaw_index_]);
  btTransform tmp_transform(tmp_rot,tmp_pos);
  tf::poseTFToMsg(tmp_transform,pose);  
}


motion_planning_msgs::RobotState  OmplRosRPYIKStateTransformer::getDefaultState()
{
  motion_planning_msgs::RobotState robot_state;
  if(kinematics_solver_)
  {
    robot_state.joint_state.name = kinematics_solver_->getJointNames();
    robot_state.joint_state.position.resize(kinematics_solver_->getJointNames().size());
  }
  else
  {
    ROS_ERROR("Kinematics solver not defined");
  }
  return robot_state;
}

}
