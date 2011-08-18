/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *
 *  \author Sachin Chitta
 *********************************************************************/

#include <move_arm/move_group.h>

namespace move_arm
{

MoveGroup::MoveGroup(const std::string &group_name, planning_environment::CollisionModelsInterface *collision_models_interface) : private_handle_("~"),group_name_(group_name)
{
  if(!collision_models_interface)
    collision_models_interface_ = new planning_environment::CollisionModelsInterface("robot_description");
  else
    collision_models_interface_ = collision_models_interface;

  ik_solver_initialized_ = false;
}	

bool MoveGroup::getConfigurationParams(std::vector<std::string> &arm_names,
                                       std::vector<std::string> &kinematics_solver_names,
                                       std::vector<std::string> &end_effector_link_names)
{
  if(arm_names.empty())
  {
    XmlRpc::XmlRpcValue arm_list;
    if(!private_handle_.getParam(group_name_+"/arms", arm_list))
    {
      ROS_ERROR("Could not find arms on param server");
      return false;
    }
    if(arm_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Arm list should be of XmlRpc Array type");
      return false;
    } 
    for (int32_t i = 0; i < arm_list.size(); ++i) 
    {
      if(arm_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
      {
        ROS_ERROR("Arm names should be strings");
        return false;
      }
      arm_names.push_back(static_cast<std::string>(arm_list[i]));
      ROS_DEBUG("Adding group: %s",arm_names.back().c_str());
    }
  }

  for(unsigned int i=0; i < arm_names.size(); i++)
  {
    std::string kinematics_solver_name;
    if(!private_handle_.hasParam(group_name_+"/"+arm_names[i]+"/kinematics_solver"))
    {
      ROS_ERROR("Kinematics solver not defined for group %s in namespace %s",arm_names[i].c_str(),private_handle_.getNamespace().c_str());
      continue;
    }
    private_handle_.getParam(group_name_+"/"+arm_names[i]+"/kinematics_solver",kinematics_solver_name);
    kinematics_solver_names.push_back(kinematics_solver_name);

    std::string tip_name;
    if(!private_handle_.hasParam(group_name_+"/"+arm_names[i]+"/tip_name"))
    {
      ROS_ERROR("End effector not defined for group %s in namespace %s",arm_names[i].c_str(),private_handle_.getNamespace().c_str());
      continue;
    }
    private_handle_.getParam(group_name_+"/"+arm_names[i]+"/tip_name",tip_name);
    end_effector_link_names.push_back(tip_name);
  }
  num_arms_ = arm_names.size();
  return true;
};

bool MoveGroup::initialize()
{
  if(!initializeControllerInterface())
  {
    ROS_ERROR("Could not initialize controller interface");
    return false;
  }
  if (group_name_.empty())
  {
    ROS_ERROR("No 'group' parameter specified. Without the name of the group of joints to plan for, action cannot start");
    return false;
  }
  const planning_models::KinematicModel::JointModelGroup* joint_model_group = collision_models_interface_->getKinematicModel()->getModelGroup(group_name_);
  physical_group_name_ = group_name_;
  if(joint_model_group == NULL) 
  {
    std::string physical_group_name;
    ROS_WARN_STREAM("No joint group " << group_name_);
    private_handle_.param<std::string>(group_name_+"/physical_group",physical_group_name,group_name_);
    joint_model_group = collision_models_interface_->getKinematicModel()->getModelGroup(physical_group_name);
    if(joint_model_group == NULL)
    {
      ROS_ERROR("No physical joint group %s exists",physical_group_name.c_str());
      return false;
    }
    physical_group_name_ = physical_group_name;
  }
  else
  {
    arm_names_.push_back(group_name_);
  }
  getConfigurationParams(arm_names_,kinematics_solver_names_,end_effector_link_names_);
  group_joint_names_ = joint_model_group->getJointModelNames();
  group_link_names_ = joint_model_group->getGroupLinkNames();

  try
  {  
    kinematics_solver_ = new arm_kinematics_constraint_aware::MultiArmKinematicsConstraintAware(arm_names_,kinematics_solver_names_,end_effector_link_names_,collision_models_interface_);
  }
  catch (MultiArmKinematicsException &e)
  {
    ROS_ERROR("Could not initialize kinematics solver");
    return true;
  }
  ik_solver_initialized_ = true;
  return true;
}
	
bool MoveGroup::createJointGoalFromPoseGoal(arm_navigation_msgs::GetMotionPlan::Request &request,
                                          arm_navigation_msgs::GetMotionPlan::Response &response,
                                          planning_models::KinematicState *kinematic_state)
{
  arm_navigation_msgs::Constraints constraints = arm_navigation_msgs::getPoseConstraintsForGroup(end_effector_names_,request.motion_plan_request.goal_constraints);
  unsigned int num_constraints = constraints.position_constraints.size();
  arm_navigation_msgs::ArmNavigationErrorCodes error_code;
  if(num_constraints != constraints.orientation_constraints.size())
  {
    ROS_ERROR("Number of position and orientation constraints should be the same");
    return false;
  }
  arm_navigation_msgs::printConstraints(constraints,false,true,true);
  std::vector<geometry_msgs::Pose> poses;
  std::vector<std::vector<double> > solutions;
  std::vector<int> error_codes;
  if(!arm_navigation_msgs::constraintsToPoseVector(constraints,poses))
    return false;
  if(!kinematics_solver_->searchConstraintAwarePositionIK(poses,ik_allowed_time_,solutions,error_codes))
    return false;
  request.motion_plan_request.goal_constraints.joint_constraints = arm_navigation_msgs::getJointConstraints(solutions,kinematics_solver_->getJointNamesByGroup(),default_joint_tolerance_);

  std::map<string, double> joint_values;
  arm_navigation_msgs::getJointValueMap(request.motion_plan_request.goal_constraints.joint_constraints,joint_values);

  kinematic_state->setKinematicState(joint_values);
  if(!collision_models_interface_->isKinematicStateValid(*kinematic_state,
                                                         group_joint_names_,
                                                         error_code,
                                                         original_goal_constraints_,
                                                         request.motion_plan_request.path_constraints,
                                                         true))
  {
    ROS_INFO("IK returned joint state for goal that doesn't seem to be valid");
    if(error_code.val == error_code.GOAL_CONSTRAINTS_VIOLATED) {
      ROS_WARN("IK solution doesn't obey goal constraints");
    } else if(error_code.val == error_code.COLLISION_CONSTRAINTS_VIOLATED) {
      ROS_WARN("IK solution in collision");
    } else {
      ROS_WARN_STREAM("Some other problem with ik solution " << error_code.val);
    }
  }
  arm_navigation_msgs::clearPoseConstraintsForGroup(request.motion_plan_request.goal_constraints,end_effector_names_);
  return true;
}

bool MoveGroup::checkState(arm_navigation_msgs::Constraints &goal_constraints,
                         arm_navigation_msgs::Constraints &path_constraints,
                         const planning_models::KinematicState *kinematic_state,
                         arm_navigation_msgs::ArmNavigationErrorCodes &error_code)
{
  if(!collision_models_interface_->isKinematicStateValid(*kinematic_state,
                                                         group_joint_names_,
                                                         error_code,
                                                         goal_constraints,
                                                         path_constraints,
                                                         true))
  {
    return false;
  } 
  return true;
}

bool MoveGroup::checkRequest(arm_navigation_msgs::GetMotionPlan::Request &request,
                             arm_navigation_msgs::GetMotionPlan::Response &response,
                             planning_models::KinematicState *kinematic_state,
                             bool check_start_state)
{
  if(kinematic_state == NULL) 
  {
    ROS_ERROR("Can't do pre-planning checks without planning state");
    return false;
  }
  if(!checkStartState(request.motion_plan_request.goal_constraints,
                      request.motion_plan_request.path_constraints,
                      kinematic_state,
                      response.error_code) && check_start_state)
    return false;
  if(!checkGoalState(request,response,kinematic_state))
    return false;
  return true;
}
                                

bool MoveGroup::checkStartState(arm_navigation_msgs::Constraints &goal_constraints,
                                arm_navigation_msgs::Constraints &path_constraints,
                                const planning_models::KinematicState *kinematic_state,
                                arm_navigation_msgs::ArmNavigationErrorCodes &error_code)
{
  arm_navigation_msgs::Constraints empty_goal_constraints;
  if(!checkState(goal_constraints,path_constraints,kinematic_state,error_code))
  {
    if(error_code.val == error_code.COLLISION_CONSTRAINTS_VIOLATED) 
    {
      error_code.val = error_code.START_STATE_IN_COLLISION;
      //TODO      visualizeCollisions(kinematic_state);
    } 
    else if (error_code.val == error_code.PATH_CONSTRAINTS_VIOLATED) 
      error_code.val = error_code.START_STATE_VIOLATES_PATH_CONSTRAINTS;
    else if (error_code.val == error_code.JOINT_LIMITS_VIOLATED) 
      error_code.val = error_code.JOINT_LIMITS_VIOLATED;
    ROS_ERROR("Start state invalid with error_code: %s",arm_navigation_msgs::armNavigationErrorCodeToString(error_code).c_str());
    return false;
  }
  return true;
}

bool MoveGroup::checkGoalState(arm_navigation_msgs::GetMotionPlan::Request &request,
                               arm_navigation_msgs::GetMotionPlan::Response &response,
                               planning_models::KinematicState *kinematic_state)
{
  if(!arm_navigation_msgs::hasPoseGoal(request)) 
  {
    arm_navigation_msgs::RobotState empty_state;
    empty_state.joint_state = arm_navigation_msgs::jointConstraintsToJointState(request.motion_plan_request.goal_constraints.joint_constraints);
    planning_environment::setRobotStateAndComputeTransforms(empty_state,*kinematic_state);
    if(!checkState(original_goal_constraints_,
                   request.motion_plan_request.path_constraints,
                   kinematic_state,
                   response.error_code))
    {
      if(response.error_code.val == response.error_code.JOINT_LIMITS_VIOLATED) 
      {
        ROS_ERROR("Will not plan to requested joint goal since it violates joint limits constraints");
        response.error_code.val = response.error_code.JOINT_LIMITS_VIOLATED;
      } 
      else if(response.error_code.val == response.error_code.COLLISION_CONSTRAINTS_VIOLATED) 
      {
        ROS_ERROR("Will not plan to requested joint goal since it is in collision");
        response.error_code.val = response.error_code.GOAL_IN_COLLISION;
      } 
      else if(response.error_code.val == response.error_code.GOAL_CONSTRAINTS_VIOLATED) 
      {
        ROS_ERROR("Will not plan to requested joint goal since it violates goal constraints");
        response.error_code.val = response.error_code.GOAL_VIOLATES_PATH_CONSTRAINTS;
      } 
      else if(response.error_code.val == response.error_code.PATH_CONSTRAINTS_VIOLATED) 
      {
        ROS_ERROR("Will not plan to requested joint goal since it violates path constraints");
        response.error_code.val = response.error_code.GOAL_VIOLATES_PATH_CONSTRAINTS;
      } 
      else 
      {
        ROS_INFO_STREAM("Will not plan to request joint goal due to error code " << response.error_code.val);
      }
      return false;
    }
  }
  return true;
}

bool MoveGroup::runIKOnRequest(arm_navigation_msgs::GetMotionPlan::Request &request,  
                               arm_navigation_msgs::GetMotionPlan::Response &response,
                               planning_models::KinematicState *kinematic_state)
{
  ROS_DEBUG("Planning to a pose goal");
  if(!ik_solver_initialized_)
  {
    ROS_ERROR("IK solver not initialized");
    return false;
  }
  original_goal_constraints_ = request.motion_plan_request.goal_constraints;
  if(!createJointGoalFromPoseGoal(request,response,kinematic_state))
  {
    ROS_ERROR("Aborting pose goal since IK failed");
    response.error_code.val = response.error_code.NO_IK_SOLUTION;
    return false;
  }
  return true;
}

bool MoveGroup::initializeControllerInterface()
{
  std::string controller_action_name;
  private_handle_.param<std::string>(group_name_+"/controller_action_name", controller_action_name, "action");
  ROS_INFO("Connecting to controller using action: %s",controller_action_name.c_str());
  controller_action_client_ = new JointExecutorActionClient(controller_action_name);
  if(!controller_action_client_) 
  {
    ROS_ERROR("Controller action client hasn't been initialized yet");
    return false;
  }
  while(!controller_action_client_->waitForActionServerToStart(ros::Duration(1.0)))
  {
    ROS_INFO("Waiting for the joint_trajectory_action server to come up.");
    if(!root_handle_.ok()) 
    {
      return false;
    }
  }
  ROS_INFO("Connected to the controller");
  return true;
}

bool MoveGroup::stopTrajectory()
{
  if (controller_goal_handle_.isExpired())
    ROS_ERROR("Expired goal handle.  controller_status = %d", controller_status_);
  else
    controller_goal_handle_.cancel();
  return true;
}

bool MoveGroup::sendTrajectory(trajectory_msgs::JointTrajectory &current_trajectory)
{
  current_trajectory.header.stamp = ros::Time::now()+ros::Duration(0.2);
  control_msgs::FollowJointTrajectoryGoal goal;  
  goal.trajectory = current_trajectory;
  controller_goal_handle_ = controller_action_client_->sendGoal(goal,boost::bind(&MoveGroup::controllerTransitionCallback, this, _1));
  controller_status_ = QUEUED;
  arm_navigation_msgs::printJointTrajectory(goal.trajectory);
  return true;
}

void MoveGroup::controllerTransitionCallback(JointExecutorActionClient::GoalHandle gh) 
{   
  if(gh != controller_goal_handle_)
    return;
  actionlib::CommState comm_state = gh.getCommState();    
  switch( comm_state.state_)
  {
  case actionlib::CommState::WAITING_FOR_GOAL_ACK:
  case actionlib::CommState::PENDING:
  case actionlib::CommState::RECALLING:
    controller_status_ = QUEUED;
    return;
  case actionlib:: CommState::ACTIVE:
  case actionlib::CommState::PREEMPTING:
    controller_status_ = ACTIVE;
    return;
  case actionlib::CommState::DONE:
    {
      switch(gh.getTerminalState().state_)
      {
      case actionlib::TerminalState::RECALLED:
      case actionlib::TerminalState::REJECTED:
      case actionlib::TerminalState::PREEMPTED:
      case actionlib::TerminalState::ABORTED:
      case actionlib::TerminalState::LOST:
        {
          ROS_INFO("Trajectory controller status came back as failed");
          controller_status_ = FAILED;
          controller_goal_handle_.reset();
          return;
        }
      case actionlib::TerminalState::SUCCEEDED:
        {
          controller_goal_handle_.reset();
          controller_status_ = SUCCESS;	  
          return;
        }
      default:
        ROS_ERROR("Unknown terminal state [%u]. This is a bug in ActionClient", gh.getTerminalState().state_);
      }
    }
  default:
    break;
  }
} 

bool MoveGroup::isControllerDone(arm_navigation_msgs::ArmNavigationErrorCodes& error_code)
{      
  if (controller_status_ == SUCCESS)
  {
    error_code.val = error_code.SUCCESS;
    return true;
  } 
  else if(controller_status_ == FAILED)
  {
    error_code.val = error_code.TRAJECTORY_CONTROLLER_FAILED;
    return true;
  } 
  else 
  {
    return false;
  }
}

}
