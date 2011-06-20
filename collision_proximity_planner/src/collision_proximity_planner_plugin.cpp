/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

/** \author Sachin Chitta*/

#include <collision_proximity_planner/collision_proximity_planner_plugin.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(collision_proximity_planner,CollisionProximityPlannerPlugin,collision_proximity_planner::CollisionProximityPlannerPlugin, motion_planning_state_refinement::MotionPlanningStateRefinement)

namespace collision_proximity_planner 
{
CollisionProximityPlannerPlugin::CollisionProximityPlannerPlugin()
{
}

bool CollisionProximityPlannerPlugin::setGroupName(const std::string &group_name)
{
  return(planner_.initialize(group_name));
}

bool CollisionProximityPlannerPlugin::refineState(const motion_planning_msgs::RobotState &robot_state,
                                                  const motion_planning_msgs::Constraints &constraints,
                                                  motion_planning_msgs::RobotState &group_state)
{
  motion_planning_msgs::RobotTrajectory robot_trajectory;
  motion_planning_msgs::RobotState current_state = robot_state;
  planner_.fillInGroupState(current_state,group_state);
  bool solution_found = planner_.findPathToFreeState(current_state,robot_trajectory);
  if(!robot_trajectory.joint_trajectory.points.empty())
  {
    group_state.joint_state.header = robot_trajectory.joint_trajectory.header;
    group_state.joint_state.position = robot_trajectory.joint_trajectory.points.back().positions;
    group_state.joint_state.name = robot_trajectory.joint_trajectory.joint_names;
  }
  return solution_found;
}

bool CollisionProximityPlannerPlugin::setRobotState(const motion_planning_msgs::RobotState &robot_state)
{
  return planner_.setRobotState(robot_state);
}

bool CollisionProximityPlannerPlugin::setGroupState(const motion_planning_msgs::RobotState &group_state)
{
  return planner_.setGroupState(group_state);
}

bool CollisionProximityPlannerPlugin::setConstraints(const motion_planning_msgs::Constraints &constraints)
{
  return true;
}

bool CollisionProximityPlannerPlugin::refineState(motion_planning_msgs::RobotState &group_state,
                                                  motion_planning_msgs::RobotTrajectory &robot_trajectory)
{
  return planner_.refineState(group_state,robot_trajectory);
}

void CollisionProximityPlannerPlugin::getStateGradient(const motion_planning_msgs::RobotState &group_state,
                                                       double &distance,
                                                       motion_planning_msgs::RobotState &gradient)
{
  planner_.getStateGradient(group_state,distance,gradient);
}
 
void CollisionProximityPlannerPlugin::clear()
{
  planner_.clear();
}

}
