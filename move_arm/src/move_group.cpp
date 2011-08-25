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

MoveGroup::MoveGroup(const std::string &group_name, planning_environment::CollisionModels *collision_models) : private_handle_("~"),group_name_(group_name),commanded_frame_name_(std::string("")),default_joint_tolerance_(0.04),controller_connected_(false)
{
  collision_models_ = collision_models;
  ik_solver_initialized_ = false;
}	

MoveGroup::~MoveGroup()
{
}

bool MoveGroup::getConfigurationParams(std::vector<std::string> &arm_names,
                                       std::vector<std::string> &kinematics_solver_names,
                                       std::vector<std::string> &end_effector_link_names)
{
  if(arm_names.empty())
  {
    if(arm_config_map_.find(physical_group_name_) == arm_config_map_.end())
    {
      ROS_ERROR("Could not find arm %s",physical_group_name_.c_str());
      return false;
    }
    arm_names = arm_config_map_[physical_group_name_].subgroups_;
    if(arm_names.empty())
    {
      arm_names.push_back(physical_group_name_);
    }
  }

  for(unsigned int i=0; i < arm_names.size(); i++)
  {
    std::string kinematics_solver_name;
    if(!private_handle_.hasParam(arm_names[i]+"/kinematics_solver"))
    {
      ROS_ERROR("Kinematics solver not defined for group %s in namespace %s",(arm_names[i]).c_str(),private_handle_.getNamespace().c_str());
      continue;
    }
    private_handle_.getParam(arm_names[i]+"/kinematics_solver",kinematics_solver_name);
    kinematics_solver_names.push_back(kinematics_solver_name);
    if(arm_config_map_.find(arm_names[i]) == arm_config_map_.end())
    {
      ROS_ERROR("Could not find arm %s",arm_names[i].c_str());
      return false;
    }
    std::string tip_name = arm_config_map_[arm_names[i]].tip_link_;
    end_effector_link_names.push_back(tip_name);
  }
  num_arms_ = arm_names.size();
  return true;
};

bool MoveGroup::initialize()
{
  if (group_name_.empty())
  {
    ROS_ERROR("No 'group' parameter specified. Without the name of the group of joints to plan for, action cannot start");
    return false;
  }
  const planning_models::KinematicModel::JointModelGroup* joint_model_group = collision_models_->getKinematicModel()->getModelGroup(group_name_);
  physical_group_name_ = group_name_;
  if(joint_model_group == NULL) 
  {
    std::string physical_group_name;
    ROS_WARN_STREAM("No joint group " << group_name_);
    private_handle_.param<std::string>(group_name_+"/physical_group",physical_group_name,group_name_);
    private_handle_.param<std::string>(group_name_+"/commanded_frame_name",commanded_frame_name_,"two_arms_objects");
    joint_model_group = collision_models_->getKinematicModel()->getModelGroup(physical_group_name);
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

  arm_config_map_ = collision_models_->getKinematicModel()->getJointModelGroupConfigMap();
  getConfigurationParams(arm_names_,kinematics_solver_names_,end_effector_link_names_);
  group_joint_names_ = joint_model_group->getJointModelNames();

  ik_solver_initialized_ = true;
  try
  {  
    kinematics_solver_ = new arm_kinematics_constraint_aware::MultiArmKinematicsConstraintAware(arm_names_,kinematics_solver_names_,end_effector_link_names_,collision_models_);
  }
  catch (MultiArmKinematicsException &e)
  {
    ROS_WARN("Could not initialize kinematics solver fro group %s",group_name_.c_str());
    ik_solver_initialized_ = false;
  }

  if(!initializeControllerInterface())
  {
    ROS_ERROR("Could not initialize controller interface");
    return false;
  }

  std::string trajectory_filter_service_name;
  private_handle_.param<std::string>(group_name_+"trajectory_filter",trajectory_filter_service_name,TRAJECTORY_FILTER);
  ros::service::waitForService(trajectory_filter_service_name);
  filter_trajectory_client_ = root_handle_.serviceClient<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>(trajectory_filter_service_name);      

  return true;
}

arm_navigation_msgs::ArmNavigationErrorCodes MoveGroup::kinematicsErrorCodeToArmNavigationErrorCode(const int& error_code)
{
  arm_navigation_msgs::ArmNavigationErrorCodes ec;
  switch(error_code)
  {
  case kinematics::SUCCESS:
    ec.val = ec.SUCCESS;
    break;
  case kinematics::TIMED_OUT:
    ec.val = ec.TIMED_OUT;
    break;
  case kinematics::NO_IK_SOLUTION:
    ec.val = ec.NO_IK_SOLUTION;
    break;
  case kinematics::FRAME_TRANSFORM_FAILURE:
    ec.val = ec.FRAME_TRANSFORM_FAILURE;
    break;
  case kinematics::IK_LINK_INVALID:
    ec.val = ec.INVALID_LINK_NAME;
    break;
  case kinematics::IK_LINK_IN_COLLISION:
    ec.val = ec.IK_LINK_IN_COLLISION;
    break;
  case kinematics::STATE_IN_COLLISION:
    ec.val = ec.KINEMATICS_STATE_IN_COLLISION;
    break;
  case kinematics::INVALID_LINK_NAME:
    ec.val = ec.INVALID_LINK_NAME;
    break;
    //  case kinematics::GOAL_CONSTRAINTS_VIOLATED:
    //    ec.val = ec.GOAL_CONSTRAINTS_VIOLATED;
    //    break;
  case kinematics::INACTIVE:
    ec.val = ec.NO_IK_SOLUTION;
    break;
  default:
    ec.val = ec.PLANNING_FAILED;
    break;
  }
  return ec;
}

bool MoveGroup::createJointGoalFromPoseGoal(arm_navigation_msgs::GetMotionPlan::Request &request,
                                            arm_navigation_msgs::GetMotionPlan::Response &response,
                                            planning_models::KinematicState *kinematic_state,
                                            double &ik_allowed_time)
{
  arm_navigation_msgs::Constraints constraints = arm_navigation_msgs::getPoseConstraintsForGroup(end_effector_link_names_,request.motion_plan_request.goal_constraints);
  
  unsigned int num_constraints = constraints.position_constraints.size();
  ROS_DEBUG("Num constraints: %d",num_constraints);
  arm_navigation_msgs::ArmNavigationErrorCodes error_code;
  if(num_constraints != constraints.orientation_constraints.size())
  {
    ROS_ERROR("Number of position and orientation constraints should be the same");
    return false;
  }
  if(!collision_models_->convertConstraintsGivenNewWorldTransform(*kinematic_state,constraints,kinematics_solver_->getBaseFrame()))
  {
    response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
    return false;
  }
  arm_navigation_msgs::printConstraints(constraints,true,true,true);
  std::vector<geometry_msgs::Pose> poses;
  std::vector<std::vector<double> > solutions;
  std::vector<int> error_codes;
  solutions.resize(num_arms_);
  error_codes.resize(num_arms_);
  if(!arm_navigation_msgs::constraintsToPoseVector(constraints,poses))
    return false;
  if(!kinematics_solver_->searchConstraintAwarePositionIK(poses,ik_allowed_time,kinematic_state,solutions,error_codes))
  {
    for(unsigned int i=0; i < error_codes.size(); i++)
      ROS_INFO("Error code: %s",arm_navigation_msgs::armNavigationErrorCodeToString(kinematicsErrorCodeToArmNavigationErrorCode(error_codes[i])).c_str());
    return false;
  }
  else
    ROS_DEBUG("Found IK solution");
  request.motion_plan_request.goal_constraints.joint_constraints = arm_navigation_msgs::getJointConstraints(solutions,kinematics_solver_->getJointNamesByGroup(),default_joint_tolerance_);

  std::map<string, double> joint_values;
  arm_navigation_msgs::getJointValueMap(request.motion_plan_request.goal_constraints.joint_constraints,joint_values);

  kinematic_state->setKinematicState(joint_values);
  if(!collision_models_->isKinematicStateValid(*kinematic_state,
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
  arm_navigation_msgs::clearPoseConstraintsForGroup(request.motion_plan_request.goal_constraints,end_effector_link_names_);
  arm_navigation_msgs::printConstraints(request.motion_plan_request.goal_constraints);
  return true;
}

bool MoveGroup::checkState(arm_navigation_msgs::Constraints &goal_constraints,
                           arm_navigation_msgs::Constraints &path_constraints,
                           const planning_models::KinematicState *kinematic_state,
                           arm_navigation_msgs::ArmNavigationErrorCodes &error_code)
{
  if(!collision_models_->isKinematicStateValid(*kinematic_state,
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
  arm_navigation_msgs::Constraints empty_goal_constraints;
  if(!checkStartState(empty_goal_constraints,
                      request.motion_plan_request.path_constraints,
                      kinematic_state,
                      response.error_code) && check_start_state)
    return false;
  if(!checkGoalState(request,response,kinematic_state))
    return false;

  if(!commanded_frame_name_.empty())
  {
    ROS_DEBUG("Commanded frame name: %s",commanded_frame_name_.c_str());
    std::vector<std::string> commanded_link_names;
    commanded_link_names.push_back(commanded_frame_name_);
    arm_navigation_msgs::Constraints constraints = arm_navigation_msgs::getPoseConstraintsForGroup(commanded_link_names,request.motion_plan_request.goal_constraints);
    ROS_DEBUG("Pose constraints for group %s",commanded_frame_name_.c_str());
    arm_navigation_msgs::printConstraints(constraints);
    constraints = arm_navigation_msgs::transformPoseConstraintsToEndEffectorConstraints(commanded_frame_name_,constraints,request.motion_plan_request.start_state,end_effector_link_names_);
    ROS_DEBUG("Transformed pose constraints for group %s",commanded_frame_name_.c_str());
    arm_navigation_msgs::printConstraints(constraints);
    original_goal_constraints_= constraints;
  }
  return true;
}
                                

bool MoveGroup::checkStartState(arm_navigation_msgs::Constraints &goal_constraints,
                                arm_navigation_msgs::Constraints &path_constraints,
                                const planning_models::KinematicState *kinematic_state,
                                arm_navigation_msgs::ArmNavigationErrorCodes &error_code)
{
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
                               planning_models::KinematicState *kinematic_state,
                               double &ik_allowed_time)
{
  ROS_DEBUG("Planning to a pose goal");
  if(!ik_solver_initialized_)
  {
    ROS_ERROR("IK solver not initialized");
    return false;
  }
  original_goal_constraints_ = request.motion_plan_request.goal_constraints;
  if(!createJointGoalFromPoseGoal(request,response,kinematic_state,ik_allowed_time))
  {
    ROS_ERROR("Aborting pose goal since IK failed");
    response.error_code.val = response.error_code.NO_IK_SOLUTION;
    return false;
  }
  return true;
}

bool MoveGroup::initializeControllerInterface()
{
  private_handle_.param<std::string>(group_name_+"/controller_action_name", controller_action_name_, "action");
  ROS_INFO("Connecting to controller using action: %s",controller_action_name_.c_str());
  controller_action_client_ = new JointExecutorActionClient(controller_action_name_);
  if(!controller_action_client_) 
  {
    ROS_ERROR("Controller action client hasn't been initialized yet");
    return false;
  }
  if(!connectToControllerServer(5))
    return false;
  return true;
}

bool MoveGroup::connectToControllerServer(const unsigned int &num_tries)
{
  controller_connected_ = false;
  unsigned int counter = 0;
  while(true)
  {
    if(counter++ > num_tries)
      break;
    if(controller_action_client_->waitForActionServerToStart(ros::Duration(1.0)))
    {
      controller_connected_ = true;
      break;
    }
    if(!root_handle_.ok()) 
      return false;
    ROS_INFO("Waiting for the joint_trajectory_action server to come up.");
  }
  if(!controller_connected_)
    ROS_WARN("Could not connect to controller. Will try to connect online");
  else
    ROS_INFO("Connected to the controller: %s",controller_action_name_.c_str());
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
  if(!controller_connected_)
  {
    if(!connectToControllerServer(1))
      return false;
  }
  if(!controller_connected_)
    return false;

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

/*bool MoveGroup::correctForJointLimits(arm_navigation_msgs::PlanningScene &planning_scene)
{
  const planning_models::KinematicModel::JointModelGroup* joint_model_group = collision_models_interface_->getKinematicModel()->getModelGroup(goal->group_name);
  const std::vector<std::string>& joint_names = joint_model_group->getJointModelNames();
  ROS_INFO_STREAM("Group name " << goal->group_name);
  if(state.areJointsWithinBounds(joint_names)) 
  {
    return;
  }
  trajectory_msgs::JointTrajectory traj;
  traj.joint_names = joint_names;
  traj.header.stamp = ros::Time::now();
  traj.header.frame_id = collision_models_interface_->getWorldFrameId();
  traj.points.resize(1);
  traj.points[0].positions.resize(joint_names.size());
  traj.points[0].velocities.resize(joint_names.size());
  traj.points[0].time_from_start = ros::Duration(.4);
  
  std::map<std::string, double> joint_values;
  state.getKinematicStateValues(joint_values);
  
  for(unsigned int j = 0; j < joint_names.size(); j++) 
  {
    if(!state.isJointWithinBounds(joint_names[j])) {
      std::pair<double, double> bounds; 
      state.getJointState(joint_names[j])->getJointModel()->getVariableBounds(joint_names[j], bounds);
      ROS_INFO_STREAM("Joint " << joint_names[j] << " out of bounds. " <<
                      " value: " << state.getJointState(joint_names[j])->getJointStateValues()[0] << 
                      " low: " << bounds.first << " high: " << bounds.second);
      if(joint_values[joint_names[j]] < bounds.first) {
        traj.points[0].positions[j] = bounds.first+JOINT_BOUNDS_MARGIN;
        ROS_INFO_STREAM("Setting joint " << joint_names[j] << " inside lower bound " << traj.points[0].positions[j]);
      } else {
        traj.points[0].positions[j] = bounds.second-JOINT_BOUNDS_MARGIN;
        ROS_INFO_STREAM("Setting joint " << joint_names[j] << " inside upper bound " << traj.points[0].positions[j]);
      }
    } else {
      traj.points[0].positions[j] = joint_values[joint_names[j]];
    }
  }
  }*/

}
