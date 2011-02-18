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
*   * Neither the name of the Willow Garage nor the names of its
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

/** \author Ioan Sucan */

#include "RequestHandler.h"
#include <ros/console.h>
#include <sstream>
#include <cstdlib>
#include <motion_planning_msgs/convert_messages.h>

void ompl_planning::RequestHandler::setOnFinishPlan(const boost::function<void(PlannerSetup*)> &onFinishPlan)
{
  onFinishPlan_ = onFinishPlan;
}

bool ompl_planning::RequestHandler::isRequestValid(ModelMap &models, motion_planning_msgs::GetMotionPlan::Request &req, const std::string &distance_metric)
{   
  ModelMap::const_iterator pos = models.find(req.motion_plan_request.group_name);
    
  if (pos == models.end())
  {
    ROS_ERROR("Cannot plan for '%s'. Model does not exist", req.motion_plan_request.group_name.c_str());
    return false;
  }
    
  /* find the model */
  Model *m = pos->second;
    
  /* if the user did not specify a planner, use the first available one */
  if (req.motion_plan_request.planner_id.empty())
    for (std::map<std::string, PlannerSetup*>::const_iterator it = m->planners.begin() ; it != m->planners.end() ; ++it)
      if ((req.motion_plan_request.goal_constraints.position_constraints.empty() && req.motion_plan_request.goal_constraints.orientation_constraints.empty() && (it->second->mp->getType() & ompl::base::PLAN_TO_GOAL_STATE) != 0) ||
          (!req.motion_plan_request.goal_constraints.position_constraints.empty() && !req.motion_plan_request.goal_constraints.orientation_constraints.empty() && (it->second->mp->getType() & ompl::base::PLAN_TO_GOAL_REGION) != 0))
      {
        if (req.motion_plan_request.planner_id.empty())
          req.motion_plan_request.planner_id = it->first;
        else
          if (m->planners[req.motion_plan_request.planner_id]->priority < it->second->priority ||
              (m->planners[req.motion_plan_request.planner_id]->priority == it->second->priority && rand() % 2 == 1))
            req.motion_plan_request.planner_id = it->first;
      }
    
  /* check if desired planner exists */
  std::map<std::string, PlannerSetup*>::iterator plannerIt = m->planners.end();
  for (std::map<std::string, PlannerSetup*>::iterator it = m->planners.begin() ; it != m->planners.end() ; ++it)
    if (it->first.find(req.motion_plan_request.planner_id) != std::string::npos)
    {
      req.motion_plan_request.planner_id = it->first;
      plannerIt = it;
      break;
    }
    
  if (plannerIt == m->planners.end())
  {
    ROS_ERROR("Motion planner not found: '%s'", req.motion_plan_request.planner_id.c_str());
    return false;
  }
    
  PlannerSetup *psetup = plannerIt->second;
    
  /* check if the desired distance metric is defined */
  if (psetup->ompl_model->sde.find(distance_metric) == psetup->ompl_model->sde.end())
  {
    ROS_ERROR("Distance evaluator not found: '%s'", distance_metric.c_str());
    return false;
  }

 
  // // check headers
  // for (unsigned int i = 0 ; i < req.motion_plan_request.goal_constraints.position_constraints.size() ; ++i)
  //   if (!m->planningMonitor->getTransformListener()->frameExists(req.motion_plan_request.goal_constraints.position_constraints[i].header.frame_id))
  //   {
  //     ROS_ERROR("Frame '%s' is not defined for goal position constraint message %u", req.motion_plan_request.goal_constraints.position_constraints[i].header.frame_id.c_str(), i);
  //     return false;
  //   }

  // for (unsigned int i = 0 ; i < req.motion_plan_request.goal_constraints.orientation_constraints.size() ; ++i)
  //   if (!m->planningMonitor->getTransformListener()->frameExists(req.motion_plan_request.goal_constraints.orientation_constraints[i].header.frame_id))
  //   {
  //     ROS_ERROR("Frame '%s' is not defined for goal pose constraint message %u", req.motion_plan_request.goal_constraints.orientation_constraints[i].header.frame_id.c_str(), i);
  //     return false;
  //   }
    
  // /*  for (unsigned int i = 0 ; i < req.motion_plan_request.goal_constraints.joint_constraints.size() ; ++i)
  //     if (!m->planningMonitor->getTransformListener()->frameExists(req.motion_plan_request.goal_constraints.joint_constraints[i].header.frame_id))
  //     {
  //     ROS_ERROR("Frame '%s' is not defined for goal joint constraint message %u", req.motion_plan_request.goal_constraints.joint_constraints[i].header.frame_id.c_str(), i);
  //     return false;
  //     }
  // */
  // for (unsigned int i = 0 ; i < req.motion_plan_request.path_constraints.position_constraints.size() ; ++i)
  //   if (!m->planningMonitor->getTransformListener()->frameExists(req.motion_plan_request.path_constraints.position_constraints[i].header.frame_id))
  //   {
  //     ROS_ERROR("Frame '%s' is not defined for path pose constraint message %u", req.motion_plan_request.path_constraints.position_constraints[i].header.frame_id.c_str(), i);
  //     return false;
  //   }

  // for (unsigned int i = 0 ; i < req.motion_plan_request.path_constraints.orientation_constraints.size() ; ++i)
  //   if (!m->planningMonitor->getTransformListener()->frameExists(req.motion_plan_request.path_constraints.orientation_constraints[i].header.frame_id))
  //   {
  //     ROS_ERROR("Frame '%s' is not defined for path pose constraint message %u", req.motion_plan_request.path_constraints.orientation_constraints[i].header.frame_id.c_str(), i);
  //     return false;
  //   }

    
  /*  for (unsigned int i = 0 ; i < req.motion_plan_request.path_constraints.joint_constraints.size() ; ++i)
      if (!m->planningMonitor->getTransformListener()->frameExists(req.motion_plan_request.path_constraints.joint_constraints[i].header.frame_id))
      {
      ROS_ERROR("Frame '%s' is not defined for path joint constraint message %u", req.motion_plan_request.path_constraints.joint_constraints[i].header.frame_id.c_str(), i);
      return false;
      }
  */
  return true;
}
void ompl_planning::RequestHandler::setWorkspaceBounds(motion_planning_msgs::WorkspaceParameters &workspace_parameters, ompl_ros::ModelBase *ompl_model)
{
  //if(workspace_parameters.workspace_region_pose.header.frame_id != ompl_model->collision_models_interface_->getWorldFrameId()) {
  //  ROS_WARN_STREAM("Workspace parameters not in world frame.  Can't transform");
  // }
  //TODO - this currently only works when the workspace is setup as a box
  if(workspace_parameters.workspace_region_shape.type == geometric_shapes_msgs::Shape::BOX && workspace_parameters.workspace_region_shape.dimensions.size() == 3)
  {
    // only area or volume should go through
    if (ompl_ros::ROSSpaceInformationKinematic *s = dynamic_cast<ompl_ros::ROSSpaceInformationKinematic*>(ompl_model->si))
    {
      double min_x = workspace_parameters.workspace_region_pose.pose.position.x-workspace_parameters.workspace_region_shape.dimensions[0]/2.0;
      double min_y = workspace_parameters.workspace_region_pose.pose.position.y-workspace_parameters.workspace_region_shape.dimensions[1]/2.0;
      double min_z = workspace_parameters.workspace_region_pose.pose.position.z-workspace_parameters.workspace_region_shape.dimensions[2]/2.0;
      
      double max_x = workspace_parameters.workspace_region_pose.pose.position.x+workspace_parameters.workspace_region_shape.dimensions[0]/2.0;
      double max_y = workspace_parameters.workspace_region_pose.pose.position.y+workspace_parameters.workspace_region_shape.dimensions[1]/2.0;
      double max_z = workspace_parameters.workspace_region_pose.pose.position.z+workspace_parameters.workspace_region_shape.dimensions[2]/2.0;
      s->setPlanningArea(min_x,min_y,max_x,max_y);
      s->setPlanningVolume(min_x,min_y,min_z,max_x,max_y,max_z);
    }
  }
  else
    ROS_DEBUG("No workspace bounding volume was set");
}

void ompl_planning::RequestHandler::configure(motion_planning_msgs::GetMotionPlan::Request &req, PlannerSetup *psetup, const std::string &distance_metric)
{
  motion_planning_msgs::ArmNavigationErrorCodes error_code;
  /* clear memory */
  psetup->ompl_model->si->clearGoal();           // goal definitions
  psetup->ompl_model->si->clearStartStates();    // start states
  psetup->ompl_model->clearEnvironmentDescriptions();

  setWorkspaceBounds(req.motion_plan_request.workspace_parameters, psetup->ompl_model);

  //doing planning monitor setup
  sensor_msgs::JointState joint_state = motion_planning_msgs::jointConstraintsToJointState(req.motion_plan_request.goal_constraints.joint_constraints);
  if(!psetup->ompl_model->collision_models_interface_->isPlanningSceneSet()) {
    ROS_WARN_STREAM("Planning scene not set.  That's bad");
    return;
  }
    
  //planning_models::KinematicState* state = psetup->ompl_model->getEnvironmentDescription()->full_state;
  planning_models::KinematicState::JointStateGroup* group_state = psetup->ompl_model->getEnvironmentDescription()->group_state;

  if (ompl_ros::ROSSpaceInformationKinematic *s = dynamic_cast<ompl_ros::ROSSpaceInformationKinematic*>(psetup->ompl_model->si))
    s->setPathConstraints(req.motion_plan_request.path_constraints);
  psetup->ompl_model->si->setStateDistanceEvaluator(psetup->ompl_model->sde[distance_metric]);
    
  //planning_models::KinematicModel::JointGroup* joint_group = psetup->ompl_model->planningMonitor->getKinematicModel()->getGroup(psetup->ompl_model->group->name);
  //if(joint_group == NULL) {
  //  ROS_ERROR_STREAM("No kinematic group for " <<psetup->ompl_model->group);
  //  return;
  //}

  /* set the starting state */
  const unsigned int dim = psetup->ompl_model->si->getStateDimension();
  ompl::base::State *start = new ompl::base::State(dim);

  std::vector<double> start_vals;
  group_state->getKinematicStateValues(start_vals);
  for(unsigned int i = 0; i < start_vals.size(); i++) {
    ROS_DEBUG_STREAM("Start states before normalization " << i << " val " << start_vals[i]);
  }

  for(unsigned int i = 0; i < group_state->getJointStateVector().size(); i++) {
    const planning_models::KinematicModel::RevoluteJointModel *rj = 
      dynamic_cast<const planning_models::KinematicModel::RevoluteJointModel*>(group_state->getJointStateVector()[i]->getJointModel());
    if (rj && rj->continuous_) {
      while(start_vals[i] < M_PI) {
        start_vals[i] += 2.0*M_PI;
      }
      while(start_vals[i] > M_PI) {
        start_vals[i] -= 2.0*M_PI;
      }
    }
  }

  for(unsigned int i = 0; i < start_vals.size(); i++) {
    ROS_DEBUG_STREAM("Start states after normalization " << i << " val " << start_vals[i]);
  }
  if(start_vals.size() != dim) {
    ROS_ERROR_STREAM("Kinematic model group has dimension " << start_vals.size() << " and not dimension " << dim);
    return;
  }
  std::map<std::string, double > vals;
  group_state->getKinematicStateValues(vals);

  for(unsigned int i = 0; i < start_vals.size(); i++) {
    start->values[i] = start_vals[i];
  }
    
  psetup->ompl_model->si->addStartState(start);

  psetup->ompl_model->si->setGoal(computeGoalFromConstraints(psetup->ompl_model, req.motion_plan_request.goal_constraints));
    
  /* fix invalid input states, if we have any */
  //fixInputStates(psetup, 0.02, 50);
  //fixInputStates(psetup, 0.05, 50);
    
  /* print some information */
  printSettings(psetup->ompl_model->si);
}

void ompl_planning::RequestHandler::printSettings(ompl::base::SpaceInformation *si)
{
  std::stringstream ss;
  si->printSettings(ss);
  ROS_DEBUG("%s", ss.str().c_str());
}

bool ompl_planning::RequestHandler::fixInputStates(PlannerSetup *psetup, double value, unsigned int count)
{
  /* add bounds for automatic state fixing (in case a state is invalid) */
  std::vector<double> rhoStart(psetup->ompl_model->si->getStateDimension());
  for (unsigned int i = 0 ; i < rhoStart.size() ; ++i)
  {
    const ompl::base::StateComponent &sc = psetup->ompl_model->si->getStateComponent(i);
    rhoStart[i] = (sc.maxValue - sc.minValue) * value;
  }
  std::vector<double> rhoGoal(rhoStart);
    
  // in case we have large bounds, we may have a larger area to sample,
  // so we increase it if we can
  ompl_ros::GoalToState *gs = dynamic_cast<ompl_ros::GoalToState*>(psetup->ompl_model->si->getGoal());
  if (gs)
  {
    std::vector< std::pair<double, double> > bounds = gs->getBounds();
    for (unsigned int i = 0 ; i < rhoGoal.size() ; ++i)
    {
      double dif = bounds[i].second - bounds[i].first;
      if (dif > rhoGoal[i])
        rhoGoal[i] = dif;
    }
  }
    
  bool result = psetup->ompl_model->si->fixInvalidInputStates(rhoStart, rhoGoal, count);
  return result;
}

bool ompl_planning::RequestHandler::computePlan(ModelMap &models, double stateDelay,
                                                motion_planning_msgs::GetMotionPlan::Request &req, 
                                                motion_planning_msgs::GetMotionPlan::Response &res, 
                                                const std::string &distance_metric)
{
  if (!isRequestValid(models, req, distance_metric))
    return false;
    
  // find the data we need 
  Model *m = models[req.motion_plan_request.group_name];
    
  // get the planner setup
  PlannerSetup *psetup = m->planners[req.motion_plan_request.planner_id];
    
  ROS_INFO("Selected motion planner: '%s', with priority %d", req.motion_plan_request.planner_id.c_str(), psetup->priority);
    
  // configure the planner
  configure(req, psetup, distance_metric);
    
  /* compute actual motion plan */
  Solution sol;
  sol.path = NULL;
  sol.difference = 0.0;
  sol.approximate = false;
  callPlanner(psetup, req.motion_plan_request, res.error_code, sol);

  /* copy the solution to the result */
  if (sol.path)
  {
    fillResult(psetup, stateDelay, res, sol);
    res.robot_state = req.motion_plan_request.start_state;
    delete sol.path;
    if (!sol.approximate)
    {
      psetup->priority++;
      if (psetup->priority > (int)m->planners.size())
        psetup->priority = m->planners.size();
    }
  }
    
  if (!sol.path || sol.approximate)
  {
    psetup->priority--;
    if (psetup->priority < -(int)m->planners.size())
      psetup->priority = -m->planners.size();
  }
  psetup->ompl_model->si->clearGoal();
  psetup->ompl_model->si->clearStartStates();
  psetup->ompl_model->clearEnvironmentDescriptions();

  ROS_DEBUG("New motion priority for  '%s' is %d", req.motion_plan_request.planner_id.c_str(), psetup->priority);
  return true;
}

void ompl_planning::RequestHandler::fillResult(PlannerSetup *psetup, double stateDelay,
                                               motion_planning_msgs::GetMotionPlan::Response &res, 
                                               const Solution &sol)
{   
  planning_models::KinematicState state(psetup->ompl_model->collision_models_interface_->getKinematicModel());

  ompl::kinematic::PathKinematic *kpath = dynamic_cast<ompl::kinematic::PathKinematic*>(sol.path);
  if (kpath)
  {
    res.trajectory.joint_trajectory.points.resize(kpath->states.size());
    const std::map<std::string, unsigned int>& map_order = state.getJointStateGroup(psetup->ompl_model->groupName)->getKinematicStateIndexMap();
    res.trajectory.joint_trajectory.joint_names.resize(map_order.size());
    for(std::map<std::string, unsigned int>::const_iterator it = map_order.begin();
        it != map_order.end();
        it++) {
      res.trajectory.joint_trajectory.joint_names[it->second] = it->first;
    }
    const unsigned int dim = psetup->ompl_model->si->getStateDimension();
    for (unsigned int i = 0 ; i < kpath->states.size() ; ++i)
    {
      res.trajectory.joint_trajectory.points[i].time_from_start = ros::Duration(i * stateDelay);
      res.trajectory.joint_trajectory.points[i].positions.resize(dim);
      for(std::map<std::string, unsigned int>::const_iterator it = map_order.begin();
          it != map_order.end();
          it++) {
        res.trajectory.joint_trajectory.points[i].positions[it->second] = kpath->states[i]->values[it->second];
      }
    }
  }
  assert(kpath);    
}

bool ompl_planning::RequestHandler::callPlanner(PlannerSetup *psetup, 
                                                const motion_planning_msgs::MotionPlanRequest &req, 
                                                motion_planning_msgs::ArmNavigationErrorCodes& error_code,
                                                Solution &sol)
{
  if (req.num_planning_attempts <= 0)
  {
    ROS_ERROR("Motion plan cannot be computed %d times", req.num_planning_attempts);
    error_code.val = error_code.PLANNING_FAILED;
  }
    
  if (dynamic_cast<ompl::base::GoalRegion*>(psetup->ompl_model->si->getGoal()))
    ROS_DEBUG("Goal threshold is %g", dynamic_cast<ompl::base::GoalRegion*>(psetup->ompl_model->si->getGoal())->threshold);
    
  unsigned int t_index = 0;
  double t_distance = 0.0;
  bool result = psetup->mp->isTrivial(&t_index, &t_distance);
    
  bool all_ok = true;

  if (result)
  {
    ROS_INFO("Solution already achieved");
    sol.difference = t_distance;
    sol.approximate = false;
	
    /* we want to maintain the invariant that a path will
       at least consist of start & goal states, so we copy
       the start state twice */
    ompl::kinematic::PathKinematic *kpath = new ompl::kinematic::PathKinematic(psetup->ompl_model->si);
    ompl::base::State *s0 = new ompl::base::State(psetup->ompl_model->si->getStateDimension());
    ompl::base::State *s1 = new ompl::base::State(psetup->ompl_model->si->getStateDimension());
    psetup->ompl_model->si->copyState(s0, psetup->ompl_model->si->getStartState(t_index));
    psetup->ompl_model->si->copyState(s1, psetup->ompl_model->si->getStartState(t_index));
    kpath->states.push_back(s0);
    kpath->states.push_back(s1);
    sol.path = kpath;
  }
  else
  {		
    /* do the planning */
    sol.path = NULL;
    sol.difference = 0.0;
    double totalTime = 0.0;
    ompl::base::Goal *goal = psetup->ompl_model->si->getGoal();
	
    //psetup->ompl_model->planningMonitor->setOnCollisionContactCallback(boost::bind(&ompl_planning::RequestHandler::contactFound, this, _1));

    for (int i = 0 ; i < req.num_planning_attempts ; ++i)
    {
      ros::WallTime startTime = ros::WallTime::now();
      bool ok = psetup->mp->solve(req.allowed_planning_time.toSec()); 
      double tsolve = (ros::WallTime::now() - startTime).toSec();	
      ROS_INFO("%s Motion planner spent %g seconds", (ok ? "[Success]" : "[Failure]"), tsolve);
      totalTime += tsolve;
	    
      if (ok)
      {
        ompl::kinematic::PathKinematic *path = dynamic_cast<ompl::kinematic::PathKinematic*>(goal->getSolutionPath());
        if(path) {
          if(!checkPathForCollisions(psetup, req, error_code, path)) {
            ROS_INFO("Path out of planner bad");
            all_ok = false;
          }
        }
        /* do path smoothing, if we are doing kinematic planning */
        if (psetup->smoother)
        {
          if (path)
          {
            ros::WallTime startTime = ros::WallTime::now();
            psetup->smoother->smoothMax(path);
            if(!checkPathForCollisions(psetup, req, error_code, path)) {
              ROS_INFO("Path out of smoother bad");
              all_ok = false;
            }
            double tsmooth = (ros::WallTime::now() - startTime).toSec();
            ROS_DEBUG("          Smoother spent %g seconds (%g seconds in total)", tsmooth, tsmooth + tsolve);
            dynamic_cast<ompl_ros::ROSSpaceInformationKinematic*>(psetup->ompl_model->si)->interpolatePath(path, 1.0);
            if(!checkPathForCollisions(psetup, req, error_code, path)) {
              ROS_INFO("Path out of interpolator bad");
              all_ok = false;
            }
          }
        }
		
        if (sol.path == NULL || sol.difference > goal->getDifference() || 
            (sol.path && sol.difference == goal->getDifference() && sol.path->length() > goal->getSolutionPath()->length()))
        {
          if (sol.path)
            delete sol.path;
          sol.path = goal->getSolutionPath();
          sol.difference = goal->getDifference();
          sol.approximate = goal->isApproximate();
          goal->forgetSolutionPath();
          ROS_DEBUG("          Obtained better solution: distance is %f", sol.difference);
        }
      } else {
        ROS_INFO("Actually not ok");
        all_ok = false;
        error_code.val = error_code.PLANNING_FAILED;
      }

      if (onFinishPlan_)
        onFinishPlan_(psetup);
	    
      psetup->mp->clear();	    
    }
	
    ROS_INFO("Total planning time: %g; Average planning time: %g", totalTime, (totalTime / (double)req.num_planning_attempts));
  }
  if(all_ok) {
    ROS_INFO("Ompl says ok");
  } else {
    ROS_INFO("Ompl says not ok");
  }
  //psetup->ompl_model->planningMonitor->setOnCollisionContactCallback(NULL);
  return result;
}


bool ompl_planning::RequestHandler::checkPathForCollisions(PlannerSetup *psetup,
                                                           const motion_planning_msgs::MotionPlanRequest& req,
                                                           motion_planning_msgs::ArmNavigationErrorCodes& error_code,
                                                           ompl::kinematic::PathKinematic *kpath)
{
  
  std::vector<motion_planning_msgs::ArmNavigationErrorCodes> trajectory_error_codes;

  planning_models::KinematicState state(psetup->ompl_model->collision_models_interface_->getKinematicModel());   
  trajectory_msgs::JointTrajectory joint_trajectory;
  
  joint_trajectory.points.resize(kpath->states.size());
  const std::map<std::string, unsigned int>& map_order = state.getJointStateGroup(psetup->ompl_model->groupName)->getKinematicStateIndexMap();
  joint_trajectory.joint_names.resize(map_order.size());
  for(std::map<std::string, unsigned int>::const_iterator it = map_order.begin();
      it != map_order.end();
      it++) {
    joint_trajectory.joint_names[it->second] = it->first;
  }

  const unsigned int dim = psetup->ompl_model->si->getStateDimension();
  for (unsigned int i = 0 ; i < kpath->states.size() ; ++i)
  {
    joint_trajectory.points[i].time_from_start = ros::Duration(i * .01);
    joint_trajectory.points[i].positions.resize(dim);
    for(std::map<std::string, unsigned int>::const_iterator it = map_order.begin();
        it != map_order.end();
        it++) {
      joint_trajectory.points[i].positions[it->second] = kpath->states[i]->values[it->second];
    }
  }

  if(!psetup->ompl_model->collision_models_interface_->isTrajectoryValid(*psetup->ompl_model->getEnvironmentDescription()->full_state,
                                                                         joint_trajectory,
                                                                         req.goal_constraints,
                                                                         req.path_constraints,
                                                                         error_code,
                                                                         trajectory_error_codes, false)) {
    if(error_code.val == error_code.JOINT_LIMITS_VIOLATED) {
      ROS_INFO("Joint limits violated");
    } else if(error_code.val == error_code.COLLISION_CONSTRAINTS_VIOLATED) {
      ROS_INFO("Trajectory in collision");
    } else if(error_code.val == error_code.PATH_CONSTRAINTS_VIOLATED) {
      ROS_INFO("Trajectory violates path constraints");
    } else if(error_code.val == error_code.GOAL_CONSTRAINTS_VIOLATED) {
      ROS_INFO("Trajectory violates goal constraints");
    } else {
      ROS_INFO_STREAM("Something else wrong with path: " << error_code.val);
    }
    return false;
  }
  return true;
  }

/** \brief The ccost and display arguments should be bound by the caller. This is a callback function that gets called by the planning
 * environment when a collision is found */
/*
void ompl_planning::RequestHandler::contactFound(collision_space::EnvironmentModel::Contact &contact)
{

  static int count = 0;
  
  std::string ns_name;
  if(contact.link1 != NULL) {
    //ROS_INFO_STREAM("Link 1 is " << contact.link2->name);
    if(contact.link1_attached_body_index == 0) {
      ns_name += contact.link1->getName()+"+";
    } else {
      if(contact.link1->getAttachedBodyModels().size() < contact.link1_attached_body_index) {
        ROS_ERROR("Link doesn't have attached body with indicated index");
      } else {
        ns_name += contact.link1->getAttachedBodyModels()[contact.link1_attached_body_index-1]->getName()+"+";
      }
    }
  } 
  
  if(contact.link2 != NULL) {
    //ROS_INFO_STREAM("Link 2 is " << contact.link2->name);
    if(contact.link2_attached_body_index == 0) {
      ns_name += contact.link2->getName();
    } else {
      if(contact.link2->getAttachedBodyModels().size() < contact.link2_attached_body_index) {
        ROS_ERROR("Link doesn't have attached body with indicated index");
      } else {
        ns_name += contact.link2->getAttachedBodyModels()[contact.link2_attached_body_index-1]->getName();
      }
    }
  } 
  
  if(!contact.object_name.empty()) {
    //ROS_INFO_STREAM("Object is " << contact.object_name);
    ns_name += contact.object_name;
  }
  
  visualization_msgs::Marker mk;
  mk.header.stamp = ros::Time::now();
  mk.header.frame_id = "odom_combined";
  mk.ns = ns_name;
  mk.id = count++;
  mk.type = visualization_msgs::Marker::SPHERE;
  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.position.x = contact.pos.x();
  mk.pose.position.y = contact.pos.y();
  mk.pose.position.z = contact.pos.z();
  mk.pose.orientation.w = 1.0;
  
  mk.scale.x = mk.scale.y = mk.scale.z = 0.01;
  
  mk.color.a = 0.6;
  mk.color.r = 0.04;
  mk.color.g = 1.0;
  mk.color.b = 0.04;
  
  //mk.lifetime = ros::Duration(30.0);
  
  vis_marker_publisher_.publish(mk);
}
*/
