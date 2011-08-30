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

#include <ompl_ros_interface/ompl_ros_planning_group.h>

namespace ompl_ros_interface
{
bool OmplRosPlanningGroup::initialize(const ros::NodeHandle &node_handle,
                                      const std::string &group_name,
                                      const std::string &planner_config_name,
                                      planning_environment::PlanningMonitor *planning_monitor)
{
  planning_monitor_    = planning_monitor;
  group_name_          = group_name;
  node_handle_         = node_handle;
  planner_config_name_ = planner_config_name;
    
  if(!initializePhysicalGroup())
    return false;

  if(!initializePlanningStateSpace(state_space_))
    return false;

  double longest_valid_segment_fraction;
  node_handle_.param(group_name_+"/longest_valid_segment_fraction",longest_valid_segment_fraction,0.005);
  state_space_->setLongestValidSegmentFraction(longest_valid_segment_fraction);

  //Setup the projection evaluator for this group
  if(!initializeProjectionEvaluator())
  {
    ROS_ERROR("Could not setup the projection evaluator");
    return false;
  }

  planner_.reset(new ompl::geometric::SimpleSetup(state_space_));
  
  if(!initializePlanner())
    return false;

  if(!initializeStateValidityChecker(state_validity_checker_))
    return false;

  planner_->setStateValidityChecker(static_cast<ompl::base::StateValidityCheckerPtr> (state_validity_checker_));
  planner_->setPlanner(ompl_planner_);

  return true;
};

bool OmplRosPlanningGroup::initializePhysicalGroup()
{
  std::string physical_group_name;
  if(!planning_monitor_->getKinematicModel()->hasModelGroup(group_name_))
  {
    ROS_INFO("Joint group %s is an abstract group",group_name_.c_str());
    if(!node_handle_.hasParam(group_name_+"/physical_group"))
    {
      ROS_ERROR("No physical group specified for %s",group_name_.c_str());
      return false;
    }
    else
      node_handle_.getParam(group_name_+"/physical_group",physical_group_name);
  }
  else
    physical_group_name = group_name_;

  //Setup the actual (physical) groups
  physical_joint_group_ = planning_monitor_->getKinematicModel()->getModelGroup(physical_group_name);
  return true;
}


bool OmplRosPlanningGroup::initializeProjectionEvaluator()
{
  std::string projection_evaluator;
  if(!node_handle_.hasParam(group_name_+"/projection_evaluator"))
  {
    ROS_ERROR("Projection evaluator not defined for group %s",group_name_.c_str());
    return false;
  }
  node_handle_.getParam(group_name_+"/projection_evaluator",projection_evaluator);
  ompl::base::ProjectionEvaluatorPtr ompl_projection_evaluator;
  ompl_projection_evaluator.reset(new ompl_ros_interface::OmplRosProjectionEvaluator(state_space_.get(),
                                                                                     projection_evaluator));
  state_space_->registerDefaultProjection(ompl_projection_evaluator);
  return true;
}

bool OmplRosPlanningGroup::initializePlanner()
{
  planner_config_.reset(new ompl_ros_interface::PlannerConfig(node_handle_.getNamespace(),planner_config_name_));
  std::string planner_type = planner_config_->getParamString("type");
  if(planner_type == "kinematic::RRT")
    return initializeRRTPlanner();
  else if(planner_type == "kinematic::RRTConnect")
    return initializeRRTConnectPlanner();
  else if(planner_type == "kinematic::pRRT")
    return initializepRRTPlanner();
  else if(planner_type == "kinematic::LazyRRT")
    return initializeLazyRRTPlanner();
  else if(planner_type == "kinematic::EST")
    return initializeESTPlanner();
  else if(planner_type == "kinematic::SBL")
    return initializeSBLPlanner();
  else if(planner_type == "kinematic::pSBL")
    return initializepSBLPlanner();
  else if(planner_type == "kinematic::KPIECE")
    return initializeKPIECEPlanner();
  else if(planner_type == "kinematic::LBKPIECE")
    return initializeLBKPIECEPlanner();
	else
  {
    ROS_WARN("Unknown planner type: %s", planner_type.c_str());
    return false;
  }
}

bool OmplRosPlanningGroup::initializeRRTPlanner()
{
  ompl_planner_.reset(new ompl::geometric::RRT(planner_->getSpaceInformation()));
  ompl::geometric::RRT* new_planner = dynamic_cast<ompl::geometric::RRT*>(ompl_planner_.get());
  if (planner_config_->hasParam("goal_bias"))
  {
    new_planner->setGoalBias(planner_config_->getParamDouble("goal_bias",new_planner->getGoalBias()));
    ROS_DEBUG("RRTPlanner::Goal bias is set to %g", new_planner->getGoalBias());
  }  
  return true;
}

bool OmplRosPlanningGroup::initializeRRTConnectPlanner()
{
  ompl_planner_.reset(new ompl::geometric::RRTConnect(planner_->getSpaceInformation()));
  ompl::geometric::RRTConnect* new_planner = dynamic_cast<ompl::geometric::RRTConnect*>(ompl_planner_.get());
  if (planner_config_->hasParam("range"))
  {
    new_planner->setRange(planner_config_->getParamDouble("range",new_planner->getRange()));
    ROS_DEBUG("RRTConnectPlanner::Range is set to %g", new_planner->getRange());
  }  
  return true;
}

bool OmplRosPlanningGroup::initializepRRTPlanner()
{
  ompl_planner_.reset(new ompl::geometric::pRRT(planner_->getSpaceInformation()));
  ompl::geometric::pRRT* new_planner = dynamic_cast<ompl::geometric::pRRT*>(ompl_planner_.get());
  if (planner_config_->hasParam("range"))
  {
    new_planner->setRange(planner_config_->getParamDouble("range",new_planner->getRange()));
    ROS_DEBUG("pRRTPlanner::Range is set to %g", new_planner->getRange());
  }  
  if (planner_config_->hasParam("goal_bias"))
  {
    new_planner->setGoalBias(planner_config_->getParamDouble("goal_bias",new_planner->getGoalBias()));
    ROS_DEBUG("pRRTPlanner::Goal bias is set to %g", new_planner->getGoalBias());
  }  
  if (planner_config_->hasParam("thread_count"))
  {
    new_planner->setThreadCount(planner_config_->getParamDouble("thread_count",new_planner->getThreadCount()));
    ROS_DEBUG("pRRTPlanner::Thread count is set to %d", (int) new_planner->getThreadCount());
  }  
  return true;
}

bool OmplRosPlanningGroup::initializeLazyRRTPlanner()
{
  ompl_planner_.reset(new ompl::geometric::LazyRRT(planner_->getSpaceInformation()));
  ompl::geometric::LazyRRT* new_planner = dynamic_cast<ompl::geometric::LazyRRT*>(ompl_planner_.get());
  if (planner_config_->hasParam("range"))
  {
    new_planner->setRange(planner_config_->getParamDouble("range",new_planner->getRange()));
    ROS_DEBUG("LazyRRTPlanner::Range is set to %g", new_planner->getRange());
  }  
  if (planner_config_->hasParam("goal_bias"))
  {
    new_planner->setGoalBias(planner_config_->getParamDouble("goal_bias",new_planner->getGoalBias()));
    ROS_DEBUG("LazyRRTPlanner::Goal bias is set to %g", new_planner->getGoalBias());
  }  
  return true;
}

bool OmplRosPlanningGroup::initializeESTPlanner()
{
  ompl_planner_.reset(new ompl::geometric::EST(planner_->getSpaceInformation()));
  ompl::geometric::EST* new_planner = dynamic_cast<ompl::geometric::EST*>(ompl_planner_.get());
  if (planner_config_->hasParam("range"))
  {
    new_planner->setRange(planner_config_->getParamDouble("range",new_planner->getRange()));
    ROS_DEBUG("ESTPlanner::Range is set to %g", new_planner->getRange());
  }  
  if (planner_config_->hasParam("goal_bias"))
  {
    new_planner->setGoalBias(planner_config_->getParamDouble("goal_bias",new_planner->getGoalBias()));
    ROS_DEBUG("ESTPlanner::Goal bias is set to %g", new_planner->getGoalBias());
  }  
  return true;
}

bool OmplRosPlanningGroup::initializeSBLPlanner()
{
  ompl_planner_.reset(new ompl::geometric::SBL(planner_->getSpaceInformation()));
  ompl::geometric::SBL* new_planner = dynamic_cast<ompl::geometric::SBL*>(ompl_planner_.get());
  if (planner_config_->hasParam("range"))
  {
    new_planner->setRange(planner_config_->getParamDouble("range",new_planner->getRange()));
    ROS_DEBUG("SBLPlanner::Range is set to %g", new_planner->getRange());
  }  
  return true;
}

bool OmplRosPlanningGroup::initializepSBLPlanner()
{
  ompl_planner_.reset(new ompl::geometric::pSBL(planner_->getSpaceInformation()));
  ompl::geometric::pSBL* new_planner = dynamic_cast<ompl::geometric::pSBL*>(ompl_planner_.get());
  if (planner_config_->hasParam("range"))
  {
    new_planner->setRange(planner_config_->getParamDouble("range",new_planner->getRange()));
    ROS_DEBUG("pSBLPlanner::Range is set to %g", new_planner->getRange());
  }  
  if (planner_config_->hasParam("thread_count"))
  {
    new_planner->setThreadCount(planner_config_->getParamDouble("thread_count",new_planner->getThreadCount()));
    ROS_DEBUG("pSBLPlanner::Thread count is set to %d", (int) new_planner->getThreadCount());
  }  
  return true;
}

bool OmplRosPlanningGroup::initializeKPIECEPlanner()
{
  ompl_planner_.reset(new ompl::geometric::KPIECE1(planner_->getSpaceInformation()));
  ompl::geometric::KPIECE1* new_planner = dynamic_cast<ompl::geometric::KPIECE1*>(ompl_planner_.get());
  if (planner_config_->hasParam("range"))
  {
    new_planner->setRange(planner_config_->getParamDouble("range",new_planner->getRange()));
    ROS_DEBUG("KPIECEPlanner::Range is set to %g", new_planner->getRange());
  }  
  if (planner_config_->hasParam("goal_bias"))
  {
    new_planner->setGoalBias(planner_config_->getParamDouble("goal_bias",new_planner->getGoalBias()));
    ROS_DEBUG("KPIECEPlanner::Goal bias is set to %g", new_planner->getGoalBias());
  }  
  return true;
}

bool OmplRosPlanningGroup::initializeLBKPIECEPlanner()
{
  ompl_planner_.reset(new ompl::geometric::LBKPIECE1(planner_->getSpaceInformation()));
  ompl::geometric::LBKPIECE1* new_planner = dynamic_cast<ompl::geometric::LBKPIECE1*>(ompl_planner_.get());
  if (planner_config_->hasParam("range"))
  {
    new_planner->setRange(planner_config_->getParamDouble("range",new_planner->getRange()));
    ROS_DEBUG("LBKPIECEPlanner::Range is set to %g", new_planner->getRange());
  }  
  return true;
}

bool OmplRosPlanningGroup::transformConstraints(motion_planning_msgs::GetMotionPlan::Request &request, 
                                                motion_planning_msgs::GetMotionPlan::Response &response)
{
  if(!planning_monitor_->transformConstraintsToFrame(request.motion_plan_request.goal_constraints, 
                                                     planning_monitor_->getWorldFrameId(),
                                                     response.error_code))
    return false;


  if(!planning_monitor_->transformConstraintsToFrame(request.motion_plan_request.goal_constraints, 
                                                     planning_monitor_->getWorldFrameId(),
                                                     response.error_code))
    return false;
 
  return true;
}

bool OmplRosPlanningGroup::omplPathGeometricToRobotTrajectory(const ompl::geometric::PathGeometric &path, 
                                                              motion_planning_msgs::RobotTrajectory &robot_trajectory)
{
  if(!ompl_ros_interface::jointStateGroupToRobotTrajectory(physical_joint_state_group_,robot_trajectory))
    return false;
  if(!ompl_ros_interface::omplPathGeometricToRobotTrajectory(path,state_space_,robot_trajectory))
    return false;
  return true;
}

bool OmplRosPlanningGroup::computePlan(motion_planning_msgs::GetMotionPlan::Request &request, 
                                       motion_planning_msgs::GetMotionPlan::Response &response)
{
  planner_->clear();
  boost::scoped_ptr<planning_models::KinematicState> kinematic_state;
  kinematic_state.reset(new planning_models::KinematicState(planning_monitor_->getKinematicModel()));
  physical_joint_state_group_ = kinematic_state->getJointStateGroup(physical_joint_group_->getName());
  if(!physical_joint_state_group_)
  {
    ROS_ERROR("Could not find physical joint state group");
    response.error_code.val = motion_planning_msgs::ArmNavigationErrorCodes::PLANNING_FAILED;
    return finish(false);
  }
  if (!isRequestValid(request,response))
    return finish(false);

  if(!configurePlanningMonitor(request,response,kinematic_state.get()))
    return finish(false);
  
  if(!transformConstraints(request,response))
    return finish(false);
  
  if(!setStartAndGoalStates(request,response))
    return finish(false);
  
  bool solved = planner_->solve(request.motion_plan_request.allowed_planning_time.toSec());
  
  if(solved)
  {
    ROS_DEBUG("Found solution for request in %f seconds",planner_->getLastPlanComputationTime());
    response.planning_time = ros::Duration(planner_->getLastPlanComputationTime());
    planner_->getPathSimplifier()->reduceVertices(planner_->getSolutionPath());
    planner_->getPathSimplifier()->collapseCloseVertices(planner_->getSolutionPath());
    
    try
    {
      response.trajectory = getSolutionPath();
      response.error_code.val = motion_planning_msgs::ArmNavigationErrorCodes::SUCCESS;
      return finish(true);
    }
    catch(...)
    {
      ROS_ERROR("Could not find solution");
      response.error_code.val = motion_planning_msgs::ArmNavigationErrorCodes::PLANNING_FAILED;
      return finish(false);
    }
  }
  else
  {
    ROS_ERROR("Could not find solution for request");
    response.error_code.val = motion_planning_msgs::ArmNavigationErrorCodes::PLANNING_FAILED;
    return finish(false);
  }  
}


bool OmplRosPlanningGroup::finish(const bool &result)
{
  planning_monitor_->revertToDefaultState();
  return result;
}

bool OmplRosPlanningGroup::configurePlanningMonitor(motion_planning_msgs::GetMotionPlan::Request &request,
                                                    motion_planning_msgs::GetMotionPlan::Response &response,
                                                    planning_models::KinematicState *kinematic_state)
{
  if(!planning_monitor_->prepareForValidityChecks(physical_joint_group_->getJointModelNames(),
                                                  request.motion_plan_request.ordered_collision_operations,
                                                  request.motion_plan_request.allowed_contacts,
                                                  request.motion_plan_request.path_constraints,
                                                  request.motion_plan_request.goal_constraints,
                                                  request.motion_plan_request.link_padding,
                                                  response.error_code))
  {
    ROS_ERROR("Could not configure planning monitor");
    return false;
  }

  /* set the pose of the whole robot */
  planning_monitor_->setRobotStateAndComputeTransforms(request.motion_plan_request.start_state, *kinematic_state);
  planning_monitor_->getEnvironmentModel()->updateRobotModel(kinematic_state);

  /* set the kinematic state for the state validator */
  ompl_ros_interface::OmplRosStateValidityChecker *my_checker = dynamic_cast<ompl_ros_interface::OmplRosStateValidityChecker*>(state_validity_checker_.get());
  my_checker->configureOnRequest(kinematic_state,
                                 physical_joint_state_group_,
                                 request);
  return true;
}

bool OmplRosPlanningGroup::setStartAndGoalStates(motion_planning_msgs::GetMotionPlan::Request &request,
                                                 motion_planning_msgs::GetMotionPlan::Response &response)
{
  if(!setStart(request,response))
    return false;
  if(!setGoal(request,response))
    return false;
  return true;
}

}
