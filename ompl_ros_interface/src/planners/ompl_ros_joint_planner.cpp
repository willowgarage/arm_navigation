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

#include <ompl_ros_interface/planners/ompl_ros_joint_planner.h>

namespace ompl_ros_interface
{

bool OmplRosJointPlanner::initializePlanningManifold(ompl::base::StateManifoldPtr &state_manifold)
{
  //Setup the corresponding ompl manifold
  state_manifold = ompl_ros_interface::jointGroupToOmplStateManifoldPtr(physical_joint_group_,
                                                                        ompl_state_to_kinematic_state_mapping_,
                                                                        kinematic_state_to_ompl_state_mapping_);
  if(!state_manifold)
  {
    ROS_ERROR("Could not set up the ompl state manifold from group %s",group_name_.c_str());
    return false;
  }
  std::string physical_group_name = physical_joint_group_->getName();
  if(node_handle_.hasParam(physical_group_name+"/tip_name"))
  {
    node_handle_.getParam(physical_group_name+"/tip_name",end_effector_name_);
    ROS_DEBUG("Group: %s, End effector: %s",physical_group_name.c_str(),end_effector_name_.c_str());
    if(node_handle_.hasParam(physical_group_name+"/kinematics_solver"))
    {
      node_handle_.getParam(physical_group_name+"/kinematics_solver",kinematics_solver_name_);
      ROS_DEBUG("Kinematics solver: %s",kinematics_solver_name_.c_str());
      ROS_DEBUG("Created new ik sampler: %s",kinematics_solver_name_.c_str());
      if(!ik_sampler_.initialize(state_manifold_,kinematics_solver_name_,physical_group_name,end_effector_name_,collision_models_interface_))
      {
        ROS_ERROR("Could not set IK sampler for pose goal");
      }
      else
        ik_sampler_available_ = true;
    }
  }

  return true;
}

bool OmplRosJointPlanner::isRequestValid(motion_planning_msgs::GetMotionPlan::Request &request,
                                          motion_planning_msgs::GetMotionPlan::Response &response)
{
  if(request.motion_plan_request.group_name != group_name_)
  {
    ROS_ERROR("Invalid group name: %s",request.motion_plan_request.group_name.c_str());
    response.error_code.val = response.error_code.INVALID_GROUP_NAME;
    return false;
  }
  for (unsigned int i = 0 ; i < request.motion_plan_request.goal_constraints.position_constraints.size() ; ++i)
  {
    if (!(request.motion_plan_request.goal_constraints.position_constraints[i].link_name == end_effector_name_))
    {
      response.error_code.val = motion_planning_msgs::ArmNavigationErrorCodes::INVALID_LINK_NAME;
      ROS_ERROR("Cartesian goals for link %s are the only ones that can be processed", end_effector_name_.c_str());
      return false;      
    }
  }
  for (unsigned int i = 0 ; i < request.motion_plan_request.goal_constraints.orientation_constraints.size() ; ++i)
  {
    if (!(request.motion_plan_request.goal_constraints.orientation_constraints[i].link_name == end_effector_name_))
    {
      response.error_code.val = motion_planning_msgs::ArmNavigationErrorCodes::INVALID_LINK_NAME;
      ROS_ERROR("Cartesian goals for link %s are the only ones that can be processed", end_effector_name_.c_str());
      return false;      
    }
  }
  if(request.motion_plan_request.allowed_planning_time.toSec() <= 0.0)
  {
    response.error_code.val = motion_planning_msgs::ArmNavigationErrorCodes::INVALID_TIMEOUT;
    ROS_ERROR("Request does not specify correct allowed planning time %f",request.motion_plan_request.allowed_planning_time.toSec());
    return false;
  }
  return true;
}

bool OmplRosJointPlanner::setGoal(motion_planning_msgs::GetMotionPlan::Request &request,
                                  motion_planning_msgs::GetMotionPlan::Response &response)
{

  if(!request.motion_plan_request.goal_constraints.joint_constraints.empty() 
     && request.motion_plan_request.goal_constraints.position_constraints.empty() 
     && request.motion_plan_request.goal_constraints.orientation_constraints.empty())
  {
    ROS_DEBUG("Joint space goal");
    return setJointGoal(request,response);
  }
  else if(!request.motion_plan_request.goal_constraints.position_constraints.empty()
          && !request.motion_plan_request.goal_constraints.orientation_constraints.empty())
  {
    ROS_DEBUG("Pose goal");
    return setPoseGoal(request,response);
  }
  else 
  {
    ROS_ERROR("Cannot handle request");
    return false;
  }
}

bool OmplRosJointPlanner::setStart(motion_planning_msgs::GetMotionPlan::Request &request,
                                    motion_planning_msgs::GetMotionPlan::Response &response)
{
  ompl::base::ScopedState<ompl::base::CompoundStateManifold> start(state_manifold_);
  ROS_DEBUG("Start");
  if(!ompl_ros_interface::kinematicStateGroupToOmplState(physical_joint_state_group_,
                                                         kinematic_state_to_ompl_state_mapping_,
                                                         start))
  {
    ROS_ERROR("Could not set start state");
    return false;
  }

  if(!ompl_ros_interface::robotStateToOmplState(request.motion_plan_request.start_state,start,false))
  {
    ROS_ERROR("Could not set start state");
    return false;
  }

  ompl_ros_interface::OmplRosJointStateValidityChecker *my_checker = dynamic_cast<ompl_ros_interface::OmplRosJointStateValidityChecker*>(state_validity_checker_.get());  
  if(!my_checker->isStateValid(start.get()))
  {
    response.error_code = my_checker->getLastErrorCode();
    if(response.error_code.val == response.error_code.PATH_CONSTRAINTS_VIOLATED)
      response.error_code.val = response.error_code.START_STATE_VIOLATES_PATH_CONSTRAINTS;
    else if(response.error_code.val == response.error_code.COLLISION_CONSTRAINTS_VIOLATED)
      response.error_code.val = response.error_code.START_STATE_IN_COLLISION;
    ROS_ERROR("Start state is invalid. Reason: %s",motion_planning_msgs::armNavigationErrorCodeToString(response.error_code).c_str());
    return false;
  }
  planner_->getProblemDefinition()->clearStartStates(); 
  planner_->addStartState(start);
  return true;
}

bool OmplRosJointPlanner::setJointGoal(motion_planning_msgs::GetMotionPlan::Request &request,
                                        motion_planning_msgs::GetMotionPlan::Response &response)
{
  ompl::base::ScopedState<ompl::base::CompoundStateManifold> goal(state_manifold_);
  ompl::base::GoalPtr goal_states(new ompl::base::GoalStates(planner_->getSpaceInformation()));
  unsigned int dimension = physical_joint_state_group_->getDimension();
  unsigned int num_goals = request.motion_plan_request.goal_constraints.joint_constraints.size()/dimension;
  if(!(num_goals*dimension == request.motion_plan_request.goal_constraints.joint_constraints.size()))
  {
    if(request.motion_plan_request.goal_constraints.joint_constraints.size() < dimension)
    {
      response.error_code.val = response.error_code.PLANNING_FAILED;
      ROS_ERROR("Joint space goal specification did not specify goal for all joints in group");	
      return false;
    }
    else
      num_goals = 1;
  }
  for(unsigned int i=0; i < num_goals; i++)
  {
    std::vector<motion_planning_msgs::JointConstraint> joint_constraints;
    for(unsigned int j=0; j < dimension; j++)
      joint_constraints.push_back(request.motion_plan_request.goal_constraints.joint_constraints[i*dimension+j]);
    if(!ompl_ros_interface::jointConstraintsToOmplState(joint_constraints,goal))
    {
      response.error_code.val = response.error_code.PLANNING_FAILED;
      ROS_ERROR("Could not convert joint space constraints to ompl state");	
      return false;
    }
    goal_states->as<ompl::base::GoalStates>()->addState(goal.get());
  }
  ompl_ros_interface::OmplRosJointStateValidityChecker *my_checker = dynamic_cast<ompl_ros_interface::OmplRosJointStateValidityChecker*>(state_validity_checker_.get());  
  if(num_goals == 1 && !my_checker->isStateValid(goal.get()))
  {
    response.error_code = my_checker->getLastErrorCode();
    if(response.error_code.val == response.error_code.PATH_CONSTRAINTS_VIOLATED)
      response.error_code.val = response.error_code.GOAL_VIOLATES_PATH_CONSTRAINTS;
    else if(response.error_code.val == response.error_code.COLLISION_CONSTRAINTS_VIOLATED)
      response.error_code.val = response.error_code.GOAL_IN_COLLISION;
    ROS_ERROR("Joint space goal is invalid. Reason: %s",motion_planning_msgs::armNavigationErrorCodeToString(response.error_code).c_str());
    return false;
  }  
  planner_->setGoal(goal_states);    
  return true;
}

bool OmplRosJointPlanner::setPoseGoal(motion_planning_msgs::GetMotionPlan::Request &request,
                                       motion_planning_msgs::GetMotionPlan::Response &response)
{
  if(!ik_sampler_available_)
  {
    ROS_ERROR("Cannot solve for pose goals since an ik sampler has not been defined");
    response.error_code.val = response.error_code.PLANNING_FAILED;
    return false;
  }
  ik_sampler_.configureOnRequest(request,response,100);
  ompl::base::GoalPtr goal;
  goal.reset(new ompl::base::GoalLazySamples(planner_->getSpaceInformation(),boost::bind(&OmplRosIKSampler::sampleGoals,&ik_sampler_,_1,_2)));
  planner_->setGoal(goal);
  return true;
}

bool OmplRosJointPlanner::initializeStateValidityChecker(ompl_ros_interface::OmplRosStateValidityCheckerPtr &state_validity_checker)
{
  state_validity_checker.reset(new ompl_ros_interface::OmplRosJointStateValidityChecker(planner_->getSpaceInformation().get(),
                                                                                        collision_models_interface_,
                                                                                        ompl_state_to_kinematic_state_mapping_));
    return true;
}

motion_planning_msgs::RobotTrajectory OmplRosJointPlanner::getSolutionPath()
{
  motion_planning_msgs::RobotTrajectory robot_trajectory;
  ompl::geometric::PathGeometric solution = planner_->getSolutionPath();
  omplPathGeometricToRobotTrajectory(solution,robot_trajectory);
  return robot_trajectory;
}


}
