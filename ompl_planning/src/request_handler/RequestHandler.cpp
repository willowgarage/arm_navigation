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
    
  // check headers
  for (unsigned int i = 0 ; i < req.motion_plan_request.goal_constraints.position_constraints.size() ; ++i)
    if (!m->planningMonitor->getTransformListener()->frameExists(req.motion_plan_request.goal_constraints.position_constraints[i].header.frame_id))
      {
        ROS_ERROR("Frame '%s' is not defined for goal position constraint message %u", req.motion_plan_request.goal_constraints.position_constraints[i].header.frame_id.c_str(), i);
        return false;
      }

  for (unsigned int i = 0 ; i < req.motion_plan_request.goal_constraints.orientation_constraints.size() ; ++i)
    if (!m->planningMonitor->getTransformListener()->frameExists(req.motion_plan_request.goal_constraints.orientation_constraints[i].header.frame_id))
      {
        ROS_ERROR("Frame '%s' is not defined for goal pose constraint message %u", req.motion_plan_request.goal_constraints.orientation_constraints[i].header.frame_id.c_str(), i);
        return false;
      }
    
  /*  for (unsigned int i = 0 ; i < req.motion_plan_request.goal_constraints.joint_constraints.size() ; ++i)
    if (!m->planningMonitor->getTransformListener()->frameExists(req.motion_plan_request.goal_constraints.joint_constraints[i].header.frame_id))
      {
        ROS_ERROR("Frame '%s' is not defined for goal joint constraint message %u", req.motion_plan_request.goal_constraints.joint_constraints[i].header.frame_id.c_str(), i);
        return false;
      }
  */
  for (unsigned int i = 0 ; i < req.motion_plan_request.path_constraints.position_constraints.size() ; ++i)
    if (!m->planningMonitor->getTransformListener()->frameExists(req.motion_plan_request.path_constraints.position_constraints[i].header.frame_id))
      {
        ROS_ERROR("Frame '%s' is not defined for path pose constraint message %u", req.motion_plan_request.path_constraints.position_constraints[i].header.frame_id.c_str(), i);
        return false;
      }

  for (unsigned int i = 0 ; i < req.motion_plan_request.path_constraints.orientation_constraints.size() ; ++i)
    if (!m->planningMonitor->getTransformListener()->frameExists(req.motion_plan_request.path_constraints.orientation_constraints[i].header.frame_id))
      {
        ROS_ERROR("Frame '%s' is not defined for path pose constraint message %u", req.motion_plan_request.path_constraints.orientation_constraints[i].header.frame_id.c_str(), i);
        return false;
      }

    
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
  /* set the workspace volume for planning */
  if (ompl_model->planningMonitor->getTransformListener()->frameExists(workspace_parameters.workspace_region_pose.header.frame_id))
    {
      bool err = false;
      try
        {
          ompl_model->planningMonitor->getTransformListener()->transformPose(ompl_model->planningMonitor->getFrameId(), workspace_parameters.workspace_region_pose, workspace_parameters.workspace_region_pose);
        }
      catch(...)
        {
          err = true;
        }
      if (err)
        ROS_ERROR("Unable to transform workspace bounds to planning frame");
      else
        {	    
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
        }
    }
  else
    ROS_DEBUG("No workspace bounding volume was set");
}

void ompl_planning::RequestHandler::configure(const planning_models::KinematicState *startState, motion_planning_msgs::GetMotionPlan::Request &req, PlannerSetup *psetup, const std::string &distance_metric)
{
    motion_planning_msgs::ArmNavigationErrorCodes error_code;
    /* clear memory */
    psetup->ompl_model->si->clearGoal();           // goal definitions
    psetup->ompl_model->si->clearStartStates();    // start states
    
    // clear clones of environments 
    psetup->ompl_model->clearEnvironmentDescriptions();

    /* before configuring, we may need to update bounds on the state space + other path constraints */
    psetup->ompl_model->planningMonitor->transformConstraintsToFrame(req.motion_plan_request.path_constraints, psetup->ompl_model->planningMonitor->getFrameId(),error_code);
    if (ompl_ros::ROSSpaceInformationKinematic *s = dynamic_cast<ompl_ros::ROSSpaceInformationKinematic*>(psetup->ompl_model->si))
  	s->setPathConstraints(req.motion_plan_request.path_constraints);
    if (ompl_ros::ROSSpaceInformationDynamic *s = dynamic_cast<ompl_ros::ROSSpaceInformationDynamic*>(psetup->ompl_model->si))
	  s->setPathConstraints(req.motion_plan_request.path_constraints);
    setWorkspaceBounds(req.motion_plan_request.workspace_parameters, psetup->ompl_model);
    psetup->ompl_model->si->setStateDistanceEvaluator(psetup->ompl_model->sde[distance_metric]);
    
    /* set the starting state */
    const unsigned int dim = psetup->ompl_model->si->getStateDimension();
    ompl::base::State *start = new ompl::base::State(dim);
    
    /* set the pose of the whole robot */
    ompl_ros::EnvironmentDescription *ed = psetup->ompl_model->getEnvironmentDescription();
    ed->kmodel->computeTransforms(startState->getParams());
    ed->collisionSpace->updateRobotModel();
    
    /* extract the components needed for the start state of the desired group */
    startState->copyParamsGroup(start->values, psetup->ompl_model->group);
    
    psetup->ompl_model->si->addStartState(start);
    
    /* add goal state */
    psetup->ompl_model->planningMonitor->transformConstraintsToFrame(req.motion_plan_request.goal_constraints, psetup->ompl_model->planningMonitor->getFrameId(),error_code);
    psetup->ompl_model->si->setGoal(computeGoalFromConstraints(psetup->ompl_model, req.motion_plan_request.goal_constraints));

    /* fix invalid input states, if we have any */
    fixInputStates(psetup, 0.02, 50);
    fixInputStates(psetup, 0.05, 50);
    
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

bool ompl_planning::RequestHandler::computePlan(ModelMap &models, const planning_models::KinematicState *start, double stateDelay,
                                                motion_planning_msgs::GetMotionPlan::Request &req, motion_planning_msgs::GetMotionPlan::Response &res, const std::string &distance_metric)
{
  if (!isRequestValid(models, req, distance_metric))
    return false;
    
  // find the data we need 
  Model *m = models[req.motion_plan_request.group_name];
    
  // get the planner setup
  PlannerSetup *psetup = m->planners[req.motion_plan_request.planner_id];
    
  ROS_INFO("Selected motion planner: '%s', with priority %d", req.motion_plan_request.planner_id.c_str(), psetup->priority);
    
  m->planningMonitor->getEnvironmentModel()->lock();
  m->planningMonitor->getKinematicModel()->lock();

  // configure the planner
  configure(start, req, psetup, distance_metric);
    
  /* compute actual motion plan */
  Solution sol;
  sol.path = NULL;
  sol.difference = 0.0;
  sol.approximate = false;
  callPlanner(psetup, req.motion_plan_request.num_planning_attempts, req.motion_plan_request.allowed_planning_time.toSec(), sol);
    
  m->planningMonitor->getEnvironmentModel()->unlock();
  m->planningMonitor->getKinematicModel()->unlock();

  psetup->ompl_model->si->clearGoal();
  psetup->ompl_model->si->clearStartStates();

  /* copy the solution to the result */
  if (sol.path)
    {
      fillResult(psetup, start, stateDelay, res, sol);
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
  ROS_DEBUG("New motion priority for  '%s' is %d", req.motion_plan_request.planner_id.c_str(), psetup->priority);
  return true;
}

void ompl_planning::RequestHandler::fillResult(PlannerSetup *psetup, const planning_models::KinematicState *start, double stateDelay,
                                               motion_planning_msgs::GetMotionPlan::Response &res, const Solution &sol)
{   
  std::vector<const planning_models::KinematicModel::Joint*> joints;
  psetup->ompl_model->planningMonitor->getKinematicModel()->getJoints(joints);
  /*    res.path.start_state.resize(joints.size());
        for (unsigned int i = 0 ; i < joints.size() ; ++i)
        {
        res.path.start_state[i].header = res.path.header;
        res.path.start_state[i].joint_name = joints[i]->name;
        start->copyParamsJoint(res.path.start_state[i].value, joints[i]->name);
        }*/
    
  ompl::kinematic::PathKinematic *kpath = dynamic_cast<ompl::kinematic::PathKinematic*>(sol.path);
  if (kpath)
    {
      res.trajectory.joint_trajectory.points.resize(kpath->states.size());
      res.trajectory.joint_trajectory.joint_names = psetup->ompl_model->group->jointNames;
	
      const unsigned int dim = psetup->ompl_model->si->getStateDimension();
      for (unsigned int i = 0 ; i < kpath->states.size() ; ++i)
        {
          res.trajectory.joint_trajectory.points[i].time_from_start = ros::Duration(i * stateDelay);
          res.trajectory.joint_trajectory.points[i].positions.resize(dim);
          for (unsigned int j = 0 ; j < dim ; ++j)
            res.trajectory.joint_trajectory.points[i].positions[j] = kpath->states[i]->values[j];
        }
    }
    
  ompl::dynamic::PathDynamic *dpath = dynamic_cast<ompl::dynamic::PathDynamic*>(sol.path);
  if (dpath)
    {
      res.trajectory.joint_trajectory.points.resize(dpath->states.size());
      res.trajectory.joint_trajectory.joint_names = psetup->ompl_model->group->jointNames;
	
      const unsigned int dim = psetup->ompl_model->si->getStateDimension();
      for (unsigned int i = 0 ; i < dpath->states.size() ; ++i)
        {
          res.trajectory.joint_trajectory.points[i].time_from_start = ros::Duration(i * stateDelay);
          res.trajectory.joint_trajectory.points[i].positions.resize(dim);
          for (unsigned int j = 0 ; j < dim ; ++j)
            res.trajectory.joint_trajectory.points[i].positions[j] = kpath->states[i]->values[j];
        }
    }
  assert(kpath || dpath);    
}

bool ompl_planning::RequestHandler::callPlanner(PlannerSetup *psetup, int times, double allowed_time, Solution &sol)
{
  if (times <= 0)
    {
      ROS_ERROR("Motion plan cannot be computed %d times", times);
      return false;
    }
    
  if (dynamic_cast<ompl::base::GoalRegion*>(psetup->ompl_model->si->getGoal()))
    ROS_DEBUG("Goal threshold is %g", dynamic_cast<ompl::base::GoalRegion*>(psetup->ompl_model->si->getGoal())->threshold);
    
  unsigned int t_index = 0;
  double t_distance = 0.0;
  bool result = psetup->mp->isTrivial(&t_index, &t_distance);
    
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
	
      for (int i = 0 ; i < times ; ++i)
        {
          ros::WallTime startTime = ros::WallTime::now();
          bool ok = psetup->mp->solve(allowed_time); 
          double tsolve = (ros::WallTime::now() - startTime).toSec();	
          ROS_DEBUG("%s Motion planner spent %g seconds", (ok ? "[Success]" : "[Failure]"), tsolve);
          totalTime += tsolve;
	    

          if (ok)
            {
              /* do path smoothing, if we are doing kinematic planning */
              if (psetup->smoother)
                {
                  ompl::kinematic::PathKinematic *path = dynamic_cast<ompl::kinematic::PathKinematic*>(goal->getSolutionPath());
                  if (path)
                    {
                      ros::WallTime startTime = ros::WallTime::now();
                      psetup->smoother->smoothMax(path);
                      double tsmooth = (ros::WallTime::now() - startTime).toSec();
                      ROS_DEBUG("          Smoother spent %g seconds (%g seconds in total)", tsmooth, tsmooth + tsolve);
                      dynamic_cast<ompl_ros::ROSSpaceInformationKinematic*>(psetup->ompl_model->si)->interpolatePath(path, 0.3);
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
            }

          if (onFinishPlan_)
            onFinishPlan_(psetup);
	    
          psetup->mp->clear();	    
        }
	
      ROS_DEBUG("Total planning time: %g; Average planning time: %g", totalTime, (totalTime / (double)times));
    }
  return result;
}

