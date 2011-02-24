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

#include <ompl_ros_interface/planners/ompl_ros_task_space_planner.h>

namespace ompl_ros_interface
{
bool OmplRosTaskSpacePlanner::initializePlanningManifold(ompl::base::StateManifoldPtr &state_manifold)
{
  if(!node_handle_.hasParam(group_name_))
  {
    ROS_ERROR("Could not find description of planning manifold %s",group_name_.c_str());
    return false;
  }
  if(!node_handle_.getParam(group_name_+"/parent_frame", planning_frame_id_))
  {
    ROS_ERROR("Could not find parent frame for group %ss",group_name_.c_str());
    return false;
  }

  XmlRpc::XmlRpcValue manifold_list;
  std::vector<std::string> manifold_names;
  if(!node_handle_.getParam(group_name_+"/manifolds", manifold_list))
  {
    ROS_ERROR("Could not find parameter %s on param server",(group_name_+"/manifolds").c_str());
    return false;
  }
  if(manifold_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Manifold list should be of XmlRpc Array type");
    return false;
  }
  int real_vector_index = -1;
  state_manifold.reset(new ompl::base::CompoundStateManifold());
  state_manifold->setName(group_name_);
  for (unsigned int i = 0; i < (unsigned int) manifold_list.size(); ++i) 
  {
    if(manifold_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_ERROR("Manifold names should be strings");
      return false;
    }
    manifold_names.push_back(static_cast<std::string>(manifold_list[i]));
    ROS_INFO("Adding manifold: %s",manifold_names.back().c_str());

    if(collision_models_interface_->getKinematicModel()->getJointModel(manifold_names.back()))
    {
      addToOmplStateManifold(collision_models_interface_->getKinematicModel(),
                             manifold_names.back(),
                             state_manifold);
      continue;
    }

    if(!getManifoldFromParamServer(ros::NodeHandle(node_handle_,group_name_+"/"+manifold_names.back()),
                                   manifold_names.back(),
                                   state_manifold,
                                   real_vector_index))
      return false;
  }
  if(!state_manifold)
  {
    ROS_ERROR("Could not set up the ompl state manifold from group %s",group_name_.c_str());
    return false;
  }
  return true;
}

bool OmplRosTaskSpacePlanner::getManifoldFromParamServer(const ros::NodeHandle &node_handle,
                                                         const std::string &manifold_name,
                                                         ompl::base::StateManifoldPtr& state_manifold,
                                                         int& real_vector_index)
{
  std::string type;
  if(!node_handle.hasParam("type"))
  {
    ROS_ERROR("Could not find type for manifold %s",manifold_name.c_str());
    return false;
  }
  node_handle.getParam("type",type);
  if(type == "Revolute" || type == "Linear")
  {
    if(real_vector_index < 0)
    {
      real_vector_index = state_manifold->as<ompl::base::CompoundStateManifold>()->getSubManifoldCount();
      ompl::base::RealVectorStateManifold *manifold = new ompl::base::RealVectorStateManifold(0);
      manifold->setName("real_vector");
      state_manifold->as<ompl::base::CompoundStateManifold>()->addSubManifold(ompl::base::StateManifoldPtr(manifold),1.0);
    }
    ompl::base::StateManifoldPtr real_vector_manifold = state_manifold->as<ompl::base::CompoundStateManifold>()->getSubManifold("real_vector");
    double min_value, max_value;
    node_handle.param("min",min_value,-M_PI);
    node_handle.param("max",max_value,M_PI);
    real_vector_manifold->as<ompl::base::RealVectorStateManifold>()->addDimension(manifold_name,min_value,max_value);    
  }
  else if(type == "Planar")
  {
    ompl::base::SE2StateManifold *manifold = new ompl::base::SE2StateManifold();
    manifold->setName(manifold_name);
    if(!node_handle.hasParam("x/min") || !node_handle.hasParam("x/max") || 
       !node_handle.hasParam("y/min") || !node_handle.hasParam("y/max"))
    {
      ROS_ERROR("Could not find bounds for planar manifold %s",manifold_name.c_str());
    }
    ompl::base::RealVectorBounds real_vector_bounds(2);
    node_handle.getParam("x/min",real_vector_bounds.low[0]);
    node_handle.getParam("x/max",real_vector_bounds.high[0]);
    node_handle.getParam("y/min",real_vector_bounds.low[1]);
    node_handle.getParam("y/max",real_vector_bounds.high[1]);
    manifold->setBounds(real_vector_bounds);

    state_manifold->as<ompl::base::CompoundStateManifold>()->addSubManifold(ompl::base::StateManifoldPtr(manifold), 1.0);
  } 
  else if(type == "Floating")
  {
    ompl::base::SE3StateManifold *manifold = new ompl::base::SE3StateManifold();
    manifold->setName(manifold_name);
    if(!node_handle.hasParam("x/min") || !node_handle.hasParam("x/max") || 
       !node_handle.hasParam("y/min") || !node_handle.hasParam("y/max") ||
       !node_handle.hasParam("z/min") || !node_handle.hasParam("z/max"))
    {
      ROS_ERROR("Could not find bounds for floating manifold %s",manifold_name.c_str());
    }
    ompl::base::RealVectorBounds real_vector_bounds(3);
    node_handle.getParam("x/min",real_vector_bounds.low[0]);
    node_handle.getParam("x/max",real_vector_bounds.high[0]);

    node_handle.getParam("y/min",real_vector_bounds.low[1]);
    node_handle.getParam("y/max",real_vector_bounds.high[1]);

    node_handle.getParam("z/min",real_vector_bounds.low[2]);
    node_handle.getParam("z/max",real_vector_bounds.high[2]);

    manifold->setBounds(real_vector_bounds);
    state_manifold->as<ompl::base::CompoundStateManifold>()->addSubManifold(ompl::base::StateManifoldPtr(manifold), 1.0);
  } 
  else if(type == "Continuous")
  {
    ompl::base::SO2StateManifold *manifold = new ompl::base::SO2StateManifold();
    manifold->setName(manifold_name);
    state_manifold->as<ompl::base::CompoundStateManifold>()->addSubManifold(ompl::base::StateManifoldPtr(manifold), 1.0);
  } 
  else
  {
    ROS_ERROR("Unknown joint type: %s",type.c_str());
    return false;
  }
  return true;
}

bool OmplRosTaskSpacePlanner::isRequestValid(motion_planning_msgs::GetMotionPlan::Request &request,
                                             motion_planning_msgs::GetMotionPlan::Response &response)
{
  if(request.motion_plan_request.group_name != group_name_)
  {
    ROS_ERROR("Invalid group name: %s",request.motion_plan_request.group_name.c_str());
    response.error_code.val = response.error_code.INVALID_GROUP_NAME;
    return false;
  }

  if(!collision_models_interface_->convertConstraintsGivenNewWorldTransform(*collision_models_interface_->getPlanningSceneState(),
                                                                            request.motion_plan_request.goal_constraints))
  {
    response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
    return false;
  }
  if(!collision_models_interface_->convertConstraintsGivenNewWorldTransform(*collision_models_interface_->getPlanningSceneState(),
                                                                            request.motion_plan_request.path_constraints))
  {
    response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
    return false;
  }
  if(request.motion_plan_request.allowed_planning_time.toSec() <= 0.0)
  {
    response.error_code.val = motion_planning_msgs::ArmNavigationErrorCodes::INVALID_TIMEOUT;
    ROS_ERROR("Request does not specify correct allowed planning time %f",request.motion_plan_request.allowed_planning_time.toSec());
    return false;
  }
  return true;
}

bool OmplRosTaskSpacePlanner::setGoal(motion_planning_msgs::GetMotionPlan::Request &request,
                                      motion_planning_msgs::GetMotionPlan::Response &response)
{
  ompl::base::ScopedState<ompl::base::CompoundStateManifold> goal(state_manifold_);
  ompl::base::GoalPtr goal_states(new ompl::base::GoalStates(planner_->getSpaceInformation()));
  ROS_DEBUG("Setting my goal");
  if(!constraintsToOmplState(request.motion_plan_request.goal_constraints,goal))
  {
    response.error_code.val = response.error_code.PLANNING_FAILED;
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
    ROS_ERROR("Goal state is invalid. Reason: %s",motion_planning_msgs::armNavigationErrorCodeToString(response.error_code).c_str());
    return false;
  }  
  planner_->setGoal(goal_states);    
  ROS_DEBUG("Setting goal state successful");
  return true;
}

bool OmplRosTaskSpacePlanner::constraintsToOmplState(const motion_planning_msgs::Constraints &constraints, 
                                                     ompl::base::ScopedState<ompl::base::CompoundStateManifold> &goal)
{
  return ompl_ros_interface::constraintsToOmplState(constraints,goal);
}

bool OmplRosTaskSpacePlanner::setStart(motion_planning_msgs::GetMotionPlan::Request &request,
                                       motion_planning_msgs::GetMotionPlan::Response &response)
{
  ompl::base::ScopedState<ompl::base::CompoundStateManifold> start(state_manifold_);
  ompl_ros_interface::robotStateToOmplState(request.motion_plan_request.start_state,start,false);
  ompl_ros_interface::OmplRosTaskSpaceValidityChecker *my_checker = dynamic_cast<ompl_ros_interface::OmplRosTaskSpaceValidityChecker*>(state_validity_checker_.get());  
  if(!my_checker->isStateValid(start.get()))
  {
    response.error_code = my_checker->getLastErrorCode();
    if(response.error_code.val == response.error_code.PATH_CONSTRAINTS_VIOLATED)
      response.error_code.val = response.error_code.START_STATE_VIOLATES_PATH_CONSTRAINTS;
    else if(response.error_code.val == response.error_code.COLLISION_CONSTRAINTS_VIOLATED)
      response.error_code.val = response.error_code.START_STATE_IN_COLLISION;
    return false;
  }
  planner_->getProblemDefinition()->clearStartStates(); 
  planner_->addStartState(start);
  return true;
}

bool OmplRosTaskSpacePlanner::initializeStateValidityChecker(ompl_ros_interface::OmplRosStateValidityCheckerPtr &state_validity_checker)
{
  state_validity_checker.reset(new ompl_ros_interface::OmplRosTaskSpaceValidityChecker(planner_->getSpaceInformation().get(),
                                                                                       collision_models_interface_,
                                                                                       planning_frame_id_));
  return true;
}


}
