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

#include <ompl_ros_interface/planners/ompl_ros_multi_arm_rpy_ik_task_space_planner.h>
#include <planning_environment/models/model_utils.h>

namespace ompl_ros_interface
{
bool OmplRosMultiArmRPYIKTaskSpacePlanner::initializeStateValidityChecker(ompl_ros_interface::OmplRosStateValidityCheckerPtr &state_validity_checker)
{
  state_validity_checker.reset(new ompl_ros_interface::OmplRosTaskSpaceValidityChecker(planner_->getSpaceInformation().get(),
                                                                                       collision_models_interface_,
                                                                                       planning_frame_id_));
  boost::shared_ptr<ompl_ros_interface::OmplRosStateTransformer> state_transformer;
  state_transformer.reset(new ompl_ros_interface::OmplRosMultiArmRPYIKStateTransformer(state_space_, physical_joint_group_));
  if(!state_transformer->initialize())
    return false;
  if(!(dynamic_cast<ompl_ros_interface::OmplRosTaskSpaceValidityChecker*>(state_validity_checker.get()))->setStateTransformer(state_transformer))
    return false;
  state_transformer_ = state_transformer;

  arm_names_ = (dynamic_cast<ompl_ros_interface::OmplRosMultiArmRPYIKStateTransformer*>(state_transformer_.get()))->getArmNames();
  end_effector_names_ = (dynamic_cast<ompl_ros_interface::OmplRosMultiArmRPYIKStateTransformer*>(state_transformer_.get()))->getEndEffectorNames();

  if(!node_handle_.hasParam(state_space_->getName()+"/object_name"))
  {
      ROS_ERROR("Could not find object name %s for multi arm planner in state space %s",object_name_.c_str(),state_space_->getName().c_str());
      return false;
  }
  node_handle_.getParam(state_space_->getName()+"/object_name",object_name_);
  original_real_vector_bounds_.reset(new ompl::base::RealVectorBounds(state_space_->as<ompl::base::CompoundStateSpace>()->getSubSpace("real_vector")->as<ompl::base::RealVectorStateSpace>()->getBounds()));
  return true;
}

arm_navigation_msgs::RobotTrajectory OmplRosMultiArmRPYIKTaskSpacePlanner::getSolutionPath()
{
  arm_navigation_msgs::RobotTrajectory robot_trajectory;  
  ompl::geometric::PathGeometric path = planner_->getSolutionPath();
  path.interpolate();
  unsigned int num_points = path.states.size();
  ROS_DEBUG("Path has %d waypoints",(int)path.states.size());
  for(unsigned int i=0; i < num_points; i++)
  {
    arm_navigation_msgs::RobotState robot_state;
    trajectory_msgs::JointTrajectoryPoint joint_trajectory_point;
    arm_navigation_msgs::MultiDOFJointTrajectoryPoint multi_dof_joint_trajectory_point;

    if(!state_transformer_->inverseTransform(*(path.states[i]),
                                             robot_state))
    {
      ROS_ERROR("Could not transform solution waypoint");
      std::stringstream string_stream;
      state_space_->printState(path.states[i],string_stream);
      ROS_ERROR("State: %d %s",i,string_stream.str().c_str());
    }

    if(i==0)
    {
      robot_trajectory.joint_trajectory.joint_names = robot_state.joint_state.name;
      robot_trajectory.joint_trajectory.header.stamp = robot_state.joint_state.header.stamp;
      robot_trajectory.joint_trajectory.header.frame_id = robot_state.joint_state.header.frame_id;

      robot_trajectory.multi_dof_joint_trajectory.stamp = ros::Duration(robot_state.multi_dof_joint_state.stamp.toSec());
      robot_trajectory.multi_dof_joint_trajectory.joint_names = robot_state.multi_dof_joint_state.joint_names;
      robot_trajectory.multi_dof_joint_trajectory.frame_ids = robot_state.multi_dof_joint_state.frame_ids;
      robot_trajectory.multi_dof_joint_trajectory.child_frame_ids = robot_state.multi_dof_joint_state.child_frame_ids;
    }
    //    arm_navigation_msgs::printJointState(robot_state.joint_state);
    arm_navigation_msgs::robotStateToRobotTrajectoryPoint(robot_state,
                                                           joint_trajectory_point,
                                                           multi_dof_joint_trajectory_point);
    if(!robot_state.joint_state.name.empty())
      robot_trajectory.joint_trajectory.points.push_back(joint_trajectory_point);
    if(!robot_state.multi_dof_joint_state.joint_names.empty())
      robot_trajectory.multi_dof_joint_trajectory.points.push_back(multi_dof_joint_trajectory_point);
  }
  return robot_trajectory;
}

bool OmplRosMultiArmRPYIKTaskSpacePlanner::setStart(arm_navigation_msgs::GetMotionPlan::Request &request,
                                                    arm_navigation_msgs::GetMotionPlan::Response &response)
{
  //Use the path constraints to set component bounds first
  arm_navigation_msgs::ArmNavigationErrorCodes error_code;
  state_space_->as<ompl::base::CompoundStateSpace>()->getSubSpace("real_vector")->as<ompl::base::RealVectorStateSpace>()->setBounds(*original_real_vector_bounds_);
  arm_navigation_msgs::Constraints tmp_constraints = request.motion_plan_request.path_constraints;
  collision_models_interface_->convertConstraintsGivenNewWorldTransform(*collision_models_interface_->getPlanningSceneState(),
                                                                        tmp_constraints,
                                                                        state_transformer_->getFrame());

  arm_navigation_msgs::PositionConstraint position_constraint;
  arm_navigation_msgs::OrientationConstraint orientation_constraint;
  if(!getObjectConstraints(tmp_constraints,position_constraint,orientation_constraint,false))
  {
    ROS_ERROR("Could not get object constraints for setting start state");
    return false;
  }
  if(!position_constraint.header.frame_id.empty())
  {
    if(!positionConstraintToOmplStateBounds(position_constraint,state_space_))
    {
      ROS_ERROR("Could not convert position constraint");
      return false;
    }
  }

  if(!orientation_constraint.header.frame_id.empty())
  {
    if(!orientationConstraintToOmplStateBounds(orientation_constraint,state_space_))
    {
      ROS_ERROR("Could not convert orientation constraint");
      return false;
    }
  }

  if(!getEndEffectorOffsets(request.motion_plan_request.start_state,end_effector_offsets_))
  {
    ROS_ERROR("Could not get end effector offsets from the planning request.");
    return false;
  }

  (dynamic_cast<ompl_ros_interface::OmplRosMultiArmRPYIKStateTransformer*>(state_transformer_.get()))->setEndEffectorOffsets(end_effector_offsets_);

  // Now, set the start state - first from the current state but then overwrite with what's in the request
  ompl::base::ScopedState<ompl::base::CompoundStateSpace> start(state_space_);
  
  //everything should be in the current planning state
  arm_navigation_msgs::RobotState current_state;
  planning_environment::convertKinematicStateToRobotState(*collision_models_interface_->getPlanningSceneState(),
                                                          ros::Time::now(),
                                                          collision_models_interface_->getWorldFrameId(),
                                                          current_state);  
  ompl_ros_interface::robotStateToOmplState(current_state,start,false);
  geometry_msgs::PoseStamped object_pose = getObjectPose(request.motion_plan_request.start_state);
  ROS_DEBUG("Setting start");
  poseStampedToOmplState(object_pose,start,false);

  // Check the start state validity
  ompl_ros_interface::OmplRosTaskSpaceValidityChecker *my_checker = dynamic_cast<ompl_ros_interface::OmplRosTaskSpaceValidityChecker*>(state_validity_checker_.get());  
  if(!my_checker->isStateValid(start.get()))
  {
    response.error_code = my_checker->getLastErrorCode();
    if(response.error_code.val == response.error_code.PATH_CONSTRAINTS_VIOLATED)
      response.error_code.val = response.error_code.START_STATE_VIOLATES_PATH_CONSTRAINTS;
    else if(response.error_code.val == response.error_code.COLLISION_CONSTRAINTS_VIOLATED)
      response.error_code.val = response.error_code.START_STATE_IN_COLLISION;
    ROS_ERROR_STREAM("Start state is invalid with code " << response.error_code.val);
    return false;
  }
  planner_->getProblemDefinition()->clearStartStates(); 
  planner_->addStartState(start);

  std::stringstream string_stream;
  state_space_->printState(start.get(),string_stream);
  ROS_INFO("Ompl Start State: %s",string_stream.str().c_str());


  ROS_DEBUG("Setting start state successful");
  return true;
}

bool OmplRosMultiArmRPYIKTaskSpacePlanner::getEndEffectorOffsets(const arm_navigation_msgs::RobotState &robot_state, 
                                                                 std::vector<tf::Pose> &end_effector_offsets)
{
  // Look for poses that match the end effector names and have the object frame as origin
  end_effector_offsets.resize(end_effector_names_.size());
  for(unsigned int i=0; i < end_effector_names_.size(); i++)
  {
    geometry_msgs::Pose end_effector_offset_msg;
    if(!getEndEffectorOffset(end_effector_names_[i],object_name_,robot_state,end_effector_offset_msg))
    {
      ROS_WARN("Could not get pose for %s with respect to object %s",end_effector_names_[i].c_str(),object_name_.c_str());
      return false;
    }
    tf::poseMsgToTF(end_effector_offset_msg,end_effector_offsets[i]);
  }
  return true;
}

bool OmplRosMultiArmRPYIKTaskSpacePlanner::getEndEffectorOffset(const std::string &end_effector_name,
                                                                const std::string &object_name,
                                                                const arm_navigation_msgs::RobotState &robot_state,
                                                                geometry_msgs::Pose &offset,
                                                                const bool &need_both_constraints)
{
  if(robot_state.multi_dof_joint_state.child_frame_ids.size() != robot_state.multi_dof_joint_state.frame_ids.size())
  {    
    ROS_ERROR("Robot state must have same number of child frame ids as frame ids");
    return false;
  }
  if(robot_state.multi_dof_joint_state.poses.size() != robot_state.multi_dof_joint_state.frame_ids.size())
  {    
    ROS_ERROR("Robot state must have same number of poses as frame ids");
    return false;
  }
  for(unsigned int i=0; i < robot_state.multi_dof_joint_state.child_frame_ids.size(); i++)
  {
    if(robot_state.multi_dof_joint_state.child_frame_ids[i] == end_effector_name && robot_state.multi_dof_joint_state.frame_ids[i] == object_name)
    {
      offset = robot_state.multi_dof_joint_state.poses[i];
      return true;
    }
  }
  return false;
}

bool OmplRosMultiArmRPYIKTaskSpacePlanner::constraintsToOmplState(const arm_navigation_msgs::Constraints &constraints, 
                                                                  ompl::base::ScopedState<ompl::base::CompoundStateSpace> &goal)
{
  // Transform the constraints
  arm_navigation_msgs::ArmNavigationErrorCodes error_code;
  arm_navigation_msgs::Constraints tmp_constraints = constraints;
  collision_models_interface_->convertConstraintsGivenNewWorldTransform(*collision_models_interface_->getPlanningSceneState(),
                                                                        tmp_constraints,
                                                                        state_transformer_->getFrame());
  // Set all physical constraints that directly map onto the joints
  if(!ompl_ros_interface::constraintsToOmplState(tmp_constraints,goal,false))
    return false;

  // Set RPY, position constraints
  arm_navigation_msgs::PositionConstraint position_constraint;
  arm_navigation_msgs::OrientationConstraint orientation_constraint;
  if(!getObjectConstraints(tmp_constraints,position_constraint,orientation_constraint,true)) {
    ROS_WARN("Goal constraints probably don't have a position and orientation constraint");
    return false;
  }
  geometry_msgs::PoseStamped desired_pose = arm_navigation_msgs::poseConstraintsToPoseStamped(position_constraint,
                                                                                               orientation_constraint);
  btQuaternion orientation;
  double roll,pitch,yaw;
  tf::quaternionMsgToTF(desired_pose.pose.orientation,orientation);
  btMatrix3x3 rotation(orientation);
  rotation.getRPY(roll,pitch,yaw);

  ROS_INFO("Setting goal: %f %f %f, %f %f %f",
           desired_pose.pose.position.x,
           desired_pose.pose.position.y,
           desired_pose.pose.position.z,
           roll,
           pitch,
           yaw);
  ROS_INFO("Setting goal (quaternion): %f %f %f %f",
           desired_pose.pose.orientation.x,
           desired_pose.pose.orientation.y,
           desired_pose.pose.orientation.z,
           desired_pose.pose.orientation.w);

  if(!poseStampedToOmplState(desired_pose,goal))
    return false;

  std::stringstream string_stream;
  state_space_->printState(goal.get(),string_stream);
  ROS_INFO("Ompl Goal State: %s",string_stream.str().c_str());

  return true;
}

bool OmplRosMultiArmRPYIKTaskSpacePlanner::positionConstraintToOmplStateBounds(const arm_navigation_msgs::PositionConstraint &position_constraint,
                                                                               ompl::base::StateSpacePtr &goal)
{
  int real_vector_index = state_space_->as<ompl::base::CompoundStateSpace>()->getSubSpaceIndex("real_vector");
  int x_index = state_space_->as<ompl::base::CompoundStateSpace>()->as<ompl::base::RealVectorStateSpace>(real_vector_index)->getDimensionIndex("x");
  int y_index = state_space_->as<ompl::base::CompoundStateSpace>()->as<ompl::base::RealVectorStateSpace>(real_vector_index)->getDimensionIndex("y");
  int z_index = state_space_->as<ompl::base::CompoundStateSpace>()->as<ompl::base::RealVectorStateSpace>(real_vector_index)->getDimensionIndex("z");
    
  ompl::base::RealVectorBounds real_vector_bounds = state_space_->as<ompl::base::CompoundStateSpace>()->getSubSpace("real_vector")->as<ompl::base::RealVectorStateSpace>()->getBounds();

  real_vector_bounds.low[x_index] = position_constraint.position.x-position_constraint.constraint_region_shape.dimensions[0]/2.0;
  real_vector_bounds.low[y_index] = position_constraint.position.y-position_constraint.constraint_region_shape.dimensions[1]/2.0;
  real_vector_bounds.low[z_index] = position_constraint.position.z-position_constraint.constraint_region_shape.dimensions[2]/2.0;
  
  real_vector_bounds.high[x_index] = position_constraint.position.x+position_constraint.constraint_region_shape.dimensions[0]/2.0;
  real_vector_bounds.high[y_index] = position_constraint.position.y+position_constraint.constraint_region_shape.dimensions[1]/2.0;
  real_vector_bounds.high[z_index] = position_constraint.position.z+position_constraint.constraint_region_shape.dimensions[2]/2.0;

  return true;
}

bool OmplRosMultiArmRPYIKTaskSpacePlanner::orientationConstraintToOmplStateBounds(const arm_navigation_msgs::OrientationConstraint &orientation_constraint,
                                                                                  ompl::base::StateSpacePtr &goal)
{
  int real_vector_index = state_space_->as<ompl::base::CompoundStateSpace>()->getSubSpaceIndex("real_vector");
    
  int roll_index = state_space_->as<ompl::base::CompoundStateSpace>()->as<ompl::base::RealVectorStateSpace>(real_vector_index)->getDimensionIndex("roll");
  int pitch_index = state_space_->as<ompl::base::CompoundStateSpace>()->as<ompl::base::RealVectorStateSpace>(real_vector_index)->getDimensionIndex("pitch");
  int yaw_index = state_space_->as<ompl::base::CompoundStateSpace>()->as<ompl::base::RealVectorStateSpace>(real_vector_index)->getDimensionIndex("yaw");

  ompl::base::RealVectorBounds real_vector_bounds = state_space_->as<ompl::base::CompoundStateSpace>()->getSubSpace("real_vector")->as<ompl::base::RealVectorStateSpace>()->getBounds();

  btQuaternion orientation;
  double roll,pitch,yaw;
  tf::quaternionMsgToTF(orientation_constraint.orientation,orientation);
  btMatrix3x3 rotation(orientation);
  rotation.getRPY(roll,pitch,yaw);
  
  double min_roll = roll - 0.75*orientation_constraint.absolute_roll_tolerance;
  double max_roll = roll + 0.75*orientation_constraint.absolute_roll_tolerance;
  if(fabs(orientation_constraint.absolute_roll_tolerance-M_PI) <= 0.001)
  {
    ROS_DEBUG("Roll is unconstrained");
    min_roll = -M_PI;
    max_roll = M_PI;
  }
  double min_pitch = pitch - 0.75*orientation_constraint.absolute_pitch_tolerance;
  double max_pitch = pitch + 0.75*orientation_constraint.absolute_pitch_tolerance;
  if(fabs(orientation_constraint.absolute_pitch_tolerance-M_PI) <= 0.001)
  {
    ROS_DEBUG("Pitch is unconstrained");
    min_pitch = -M_PI;
    max_pitch = M_PI;
  }    
  double min_yaw = yaw - 0.75*orientation_constraint.absolute_yaw_tolerance;
  double max_yaw = yaw + 0.75*orientation_constraint.absolute_yaw_tolerance;    
  if(fabs(orientation_constraint.absolute_yaw_tolerance-M_PI) <= 0.001)
  {
    ROS_DEBUG("Yaw is unconstrained");
    min_yaw = -M_PI;
    max_yaw = M_PI;
  }  
  real_vector_bounds.low[roll_index] = min_roll;
  real_vector_bounds.low[yaw_index] = min_yaw;
  real_vector_bounds.low[pitch_index] = min_pitch;
  real_vector_bounds.high[roll_index] = max_roll;
  real_vector_bounds.high[yaw_index] = max_yaw;
  real_vector_bounds.high[pitch_index] = max_pitch;
  ROS_DEBUG("Resetting constraints");

  ROS_DEBUG("Roll constraint: [%f, %f]",min_roll,max_roll);
  ROS_DEBUG("Pitch constraint: [%f, %f]",min_pitch,max_pitch);
  ROS_DEBUG("Yaw constraint: [%f, %f]",min_yaw,max_yaw);

  state_space_->as<ompl::base::CompoundStateSpace>()->getSubSpace("real_vector")->as<ompl::base::RealVectorStateSpace>()->setBounds(real_vector_bounds);
  return true;
}


bool OmplRosMultiArmRPYIKTaskSpacePlanner::getObjectConstraints(const arm_navigation_msgs::Constraints &constraints,
                                                                arm_navigation_msgs::PositionConstraint &position_constraint,
                                                                arm_navigation_msgs::OrientationConstraint &orientation_constraint,
                                                                const bool &need_both_constraints)
{
  int position_index = -1;
  int orientation_index = -1;
  for(unsigned int i=0; i < constraints.position_constraints.size(); i++)
  {
    if(constraints.position_constraints[i].link_name == object_name_)
    {
      position_index = i;
      break;
    }
  }
  for(unsigned int i=0; i < constraints.orientation_constraints.size(); i++)
  {
    if(constraints.orientation_constraints[i].link_name == object_name_)
    {
      orientation_index = i;
      break;
    }
  }
  if((position_index < 0 || orientation_index < 0) && need_both_constraints)
  {
    ROS_ERROR("Need at least one position and orientation constraint to be specified in the message");
    return false;
  }
  if(position_index >= 0)
    position_constraint = constraints.position_constraints[position_index];
  if(orientation_index >= 0)
    orientation_constraint = constraints.orientation_constraints[orientation_index];
  return true;
}

bool OmplRosMultiArmRPYIKTaskSpacePlanner::poseStampedToOmplState(const geometry_msgs::PoseStamped &desired_pose, 
                                                                  ompl::base::ScopedState<ompl::base::CompoundStateSpace> &goal,
                                                                  const bool &return_if_outside_constraints)
{
  btTransform desired_pose_tf;
  double roll, pitch, yaw, x, y, z;
  tf::poseMsgToTF(desired_pose.pose,desired_pose_tf);
  desired_pose_tf.getBasis().getRPY(roll,pitch,yaw);
  x = desired_pose.pose.position.x;
  y = desired_pose.pose.position.y;
  z = desired_pose.pose.position.z;

  int real_vector_index = state_space_->as<ompl::base::CompoundStateSpace>()->getSubSpaceIndex("real_vector");
  int x_index = state_space_->as<ompl::base::CompoundStateSpace>()->as<ompl::base::RealVectorStateSpace>(real_vector_index)->getDimensionIndex("x");
  int y_index = state_space_->as<ompl::base::CompoundStateSpace>()->as<ompl::base::RealVectorStateSpace>(real_vector_index)->getDimensionIndex("y");
  int z_index = state_space_->as<ompl::base::CompoundStateSpace>()->as<ompl::base::RealVectorStateSpace>(real_vector_index)->getDimensionIndex("z");
    
  int roll_index = state_space_->as<ompl::base::CompoundStateSpace>()->as<ompl::base::RealVectorStateSpace>(real_vector_index)->getDimensionIndex("roll");
  int pitch_index = state_space_->as<ompl::base::CompoundStateSpace>()->as<ompl::base::RealVectorStateSpace>(real_vector_index)->getDimensionIndex("pitch");
  int yaw_index = state_space_->as<ompl::base::CompoundStateSpace>()->as<ompl::base::RealVectorStateSpace>(real_vector_index)->getDimensionIndex("yaw");


  double min_value,max_value;
  min_value = state_space_->as<ompl::base::CompoundStateSpace>()->as<ompl::base::RealVectorStateSpace>(real_vector_index)->getBounds().low[roll_index];
  max_value = state_space_->as<ompl::base::CompoundStateSpace>()->as<ompl::base::RealVectorStateSpace>(real_vector_index)->getBounds().high[roll_index];
  if(!checkAndCorrectForWrapAround(roll,min_value,max_value) && return_if_outside_constraints)
  {
    ROS_ERROR("Roll %f does not lie within constraint bounds [%f,%f]",roll,min_value,max_value);
    return false;
  }
  else
    ROS_DEBUG("Roll : %f [%f %f]",roll,min_value,max_value);

  min_value = state_space_->as<ompl::base::CompoundStateSpace>()->as<ompl::base::RealVectorStateSpace>(real_vector_index)->getBounds().low[pitch_index];
  max_value = state_space_->as<ompl::base::CompoundStateSpace>()->as<ompl::base::RealVectorStateSpace>(real_vector_index)->getBounds().high[pitch_index];
  if(!checkAndCorrectForWrapAround(pitch,min_value,max_value)  && return_if_outside_constraints)
  {
    ROS_ERROR("Pitch %f does not lie within constraint bounds [%f,%f]",pitch,min_value,max_value);
    return false;
  }
  else
    ROS_DEBUG("Pitch : %f [%f %f]",pitch,min_value,max_value);

  min_value = state_space_->as<ompl::base::CompoundStateSpace>()->as<ompl::base::RealVectorStateSpace>(real_vector_index)->getBounds().low[yaw_index];
  max_value = state_space_->as<ompl::base::CompoundStateSpace>()->as<ompl::base::RealVectorStateSpace>(real_vector_index)->getBounds().high[yaw_index];
  if(!checkAndCorrectForWrapAround(yaw,min_value,max_value)  && return_if_outside_constraints)
  {
    ROS_ERROR("Yaw %f does not lie within constraint bounds [%f,%f]",yaw,min_value,max_value);
    return false;
  }
  else
    ROS_DEBUG("Yaw : %f [%f %f]",yaw,min_value,max_value);

  goal->as<ompl::base::RealVectorStateSpace::StateType>(real_vector_index)->values[x_index] = x;
  goal->as<ompl::base::RealVectorStateSpace::StateType>(real_vector_index)->values[y_index] = y;
  goal->as<ompl::base::RealVectorStateSpace::StateType>(real_vector_index)->values[z_index] = z;

  goal->as<ompl::base::RealVectorStateSpace::StateType>(real_vector_index)->values[roll_index] = roll;
  goal->as<ompl::base::RealVectorStateSpace::StateType>(real_vector_index)->values[pitch_index] = pitch;
  goal->as<ompl::base::RealVectorStateSpace::StateType>(real_vector_index)->values[yaw_index] = yaw;
  return true;
}

bool OmplRosMultiArmRPYIKTaskSpacePlanner::checkAndCorrectForWrapAround(double &angle, 
                                                                        const double &min_v, 
                                                                        const double &max_v)
{
  if(angle >= min_v && angle <= max_v)
    return true;
  double new_angle = angle - 2*M_PI;
  if(new_angle >= min_v && new_angle <= max_v)
  {
    angle = new_angle;
    return true;
  }
  new_angle = angle + 2*M_PI;
  if(new_angle >= min_v && new_angle <= max_v)
  {
    angle = new_angle;
    return true;
  }
  return false;
}

geometry_msgs::PoseStamped OmplRosMultiArmRPYIKTaskSpacePlanner::getObjectPose(const arm_navigation_msgs::RobotState &robot_state)
{
  // 1. First check if the request contains a start state for the end effector pose
  // 2. Otherwise set the start pose to the current pose
  for(unsigned int i=0; i < robot_state.multi_dof_joint_state.child_frame_ids.size(); i++)
  {
    if(robot_state.multi_dof_joint_state.child_frame_ids[i] == object_name_)
    {
      geometry_msgs::PoseStamped desired_pose;
      desired_pose.pose = robot_state.multi_dof_joint_state.poses[i];
      desired_pose.header.stamp = ros::Time();
      desired_pose.header.frame_id = robot_state.multi_dof_joint_state.frame_ids[i];
      if(!collision_models_interface_->convertPoseGivenWorldTransform(*collision_models_interface_->getPlanningSceneState(),
                                                                      state_transformer_->getFrame(),
                                                                      desired_pose.header,
                                                                      desired_pose.pose,
                                                                      desired_pose)) {
        ROS_WARN_STREAM("getObjectPose has problems transforming pose into frame " << state_transformer_->getFrame());
      }
      return desired_pose;
    }
  }

  planning_environment::setRobotStateAndComputeTransforms(robot_state, *collision_models_interface_->getPlanningSceneState());
  std::vector<tf::Pose> end_effector_poses;
  end_effector_poses.resize(end_effector_names_.size());

  for(unsigned int i=0; i < end_effector_names_.size(); i++)
  {
    end_effector_poses[i] = collision_models_interface_->getPlanningSceneState()->getLinkState(end_effector_names_[i])->getGlobalLinkTransform();
    //Store candidate object pose in same variable
    end_effector_poses[i] = end_effector_poses[i] * end_effector_offsets_[i].inverse();
    ROS_INFO("Object pose inferred from end effector %d (%s): %f %f %f, %f %f %f %f",i,end_effector_names_[i].c_str(),
	     end_effector_poses[i].getOrigin().x(),
	     end_effector_poses[i].getOrigin().y(),
	     end_effector_poses[i].getOrigin().z(),
	     end_effector_poses[i].getRotation().x(),
	     end_effector_poses[i].getRotation().y(),
	     end_effector_poses[i].getRotation().z(),
	     end_effector_poses[i].getRotation().w());
  }
  
  geometry_msgs::PoseStamped desired_pose;
  if(end_effector_names_.size() == 1)
  {
    tf::poseTFToMsg(end_effector_poses[0],desired_pose.pose);
  }
  else
  {
    tf::Quaternion quaternion_0 = end_effector_poses[0].getRotation();
    tf::Quaternion quaternion_1 = end_effector_poses[1].getRotation();
    //For now, just averaging the first two poses
    tf::Quaternion mean_quaternion = quaternion_0.slerp(quaternion_1,0.5);
    tf::Pose desired_pose_tf = btTransform(mean_quaternion, (end_effector_poses[0].getOrigin() + end_effector_poses[1].getOrigin())/2.0);
    tf::poseTFToMsg(desired_pose_tf,desired_pose.pose);
  }

  desired_pose.header.stamp = ros::Time();
  desired_pose.header.frame_id = collision_models_interface_->getWorldFrameId();
  if(!collision_models_interface_->convertPoseGivenWorldTransform(*collision_models_interface_->getPlanningSceneState(),
                                                                  state_transformer_->getFrame(),
                                                                  desired_pose.header,
                                                                  desired_pose.pose,
                                                                  desired_pose)) 
  {
      ROS_WARN_STREAM("getObjectPose has problems transforming pose into frame " << state_transformer_->getFrame());
  }
  btQuaternion orientation;
  double roll,pitch,yaw;
  tf::quaternionMsgToTF(desired_pose.pose.orientation,orientation);
  btMatrix3x3 rotation(orientation);
  rotation.getRPY(roll,pitch,yaw);
  
  ROS_INFO("Object pose in frame %s: %f %f %f, %f %f %f",state_transformer_->getFrame().c_str(), desired_pose.pose.position.x,desired_pose.pose.position.y,desired_pose.pose.position.z,roll,pitch,yaw);
  ROS_INFO_STREAM("Quaternion is " 
                   << orientation.x() << " " 
                   << orientation.y() << " " 
                     << orientation.z() << " " 
                   << orientation.w()); 
  return desired_pose;
}
                                     
}
