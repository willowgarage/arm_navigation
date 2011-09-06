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

#include <ompl_ros_interface/planners/ompl_ros_vcj_planner.h>
#include <planning_environment/models/model_utils.h>

namespace ompl_ros_interface
{

bool OmplRosVCJPlanner::initializeStateValidityChecker(ompl_ros_interface::OmplRosStateValidityCheckerPtr &state_validity_checker)
{
  state_validity_checker.reset(new ompl_ros_interface::OmplRosTaskSpaceValidityChecker(planner_->getSpaceInformation().get(),
                                                                                       collision_models_interface_,
                                                                                       planning_frame_id_));
  boost::shared_ptr<ompl_ros_interface::OmplRosStateTransformer> state_transformer;
  state_transformer.reset(new ompl_ros_interface::OmplRosVCJStateTransformer(state_space_, physical_joint_group_));
  if(!state_transformer->initialize())
    return false;
  if(!(dynamic_cast<ompl_ros_interface::OmplRosTaskSpaceValidityChecker*>(state_validity_checker.get()))->setStateTransformer(state_transformer))
    return false;
  state_transformer_ = state_transformer;
  return true;
}

bool OmplRosVCJPlanner::setGoal(arm_navigation_msgs::GetMotionPlan::Request &request,
                                arm_navigation_msgs::GetMotionPlan::Response &response)
{
  ompl::base::ScopedState<ompl::base::CompoundStateSpace> goal(state_space_);
  ompl::base::GoalPtr goal_states(new ompl::base::GoalStates(planner_->getSpaceInformation()));
  ROS_DEBUG("Setting my goal");
  if(!ompl_ros_interface::constraintsToOmplState(request.motion_plan_request.goal_constraints,goal,false))
  {
    response.error_code.val = response.error_code.PLANNING_FAILED;
    ROS_WARN("Problem converting constraints to ompl state");
    return false;
  }
  goal_states->as<ompl::base::GoalStates>()->addState(goal.get());
  ompl_ros_interface::OmplRosTaskSpaceValidityChecker *my_checker = dynamic_cast<ompl_ros_interface::OmplRosTaskSpaceValidityChecker*>(state_validity_checker_.get());  
  if(!my_checker->isStateValid(goal.get()))
  {
    response.error_code = my_checker->getLastErrorCode();
    if(response.error_code.val == response.error_code.PATH_CONSTRAINTS_VIOLATED)
      response.error_code.val = response.error_code.GOAL_VIOLATES_PATH_CONSTRAINTS;
    else if(response.error_code.val == response.error_code.COLLISION_CONSTRAINTS_VIOLATED)
      response.error_code.val = response.error_code.GOAL_IN_COLLISION;
    ROS_ERROR("Goal state is invalid. Reason: %s",arm_navigation_msgs::armNavigationErrorCodeToString(response.error_code).c_str());
    return false;
  }  
  planner_->setGoal(goal_states);    
  ROS_DEBUG("Setting goal state successful");
  return true;
}

arm_navigation_msgs::RobotTrajectory OmplRosVCJPlanner::getSolutionPath()
{
  arm_navigation_msgs::RobotTrajectory robot_trajectory;
  
  ompl::geometric::PathGeometric path = planner_->getSolutionPath();
  path.interpolate();
  unsigned int num_points = path.states.size();
  ROS_INFO("Path has %d waypoints",(int)path.states.size());
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
  ROS_INFO("Solution size      : %d",(int) robot_trajectory.joint_trajectory.points.size());
  ROS_INFO("Solution state size: %d",(int) robot_trajectory.joint_trajectory.points[0].positions.size());
  return robot_trajectory;
}

}
