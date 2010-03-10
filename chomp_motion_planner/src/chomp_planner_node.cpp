/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

/** \author Mrinal Kalakrishnan */

#include <chomp_motion_planner/chomp_planner_node.h>
#include <chomp_motion_planner/chomp_trajectory.h>
#include <chomp_motion_planner/chomp_utils.h>
#include <chomp_motion_planner/chomp_parameters.h>
#include <chomp_motion_planner/chomp_optimizer.h>
#include <kdl/jntarray.hpp>
#include <angles/angles.h>
#include <visualization_msgs/MarkerArray.h>

#include <map>
#include <vector>
#include <string>

using namespace std;

namespace chomp
{

  ChompPlannerNode::ChompPlannerNode(ros::NodeHandle node_handle) : node_handle_(node_handle)
{

}

bool ChompPlannerNode::init()
{
  // load in some default parameters
  node_handle_.param("trajectory_duration", trajectory_duration_, 3.0);
  node_handle_.param("trajectory_discretization", trajectory_discretization_, 0.03);

  // build the robot model
  if (!chomp_robot_model_.init())
    return false;

  // load chomp parameters:
  chomp_parameters_.initFromNodeHandle();

  double max_radius_clearance = chomp_robot_model_.getMaxRadiusClearance();

  // initialize the collision space
  if (!chomp_collision_space_.init(max_radius_clearance))
    return false;

  // initialize the visualization publisher:
  vis_marker_array_publisher_ = root_handle_.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );
  vis_marker_publisher_ = root_handle_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

  // advertise the planning service
  plan_kinematic_path_service_ = root_handle_.advertiseService("chomp_planner_longrange/plan_path", &ChompPlannerNode::planKinematicPath, this);

  ROS_INFO("Initalized CHOMP planning service...");

  return true;
}

ChompPlannerNode::~ChompPlannerNode()
{
}

int ChompPlannerNode::run()
{
  ros::spin();
  return 0;
}

bool ChompPlannerNode::planKinematicPath(motion_planning_msgs::GetMotionPlan::Request &req, motion_planning_msgs::GetMotionPlan::Response &res)
{
  if (!(req.motion_plan_request.goal_constraints.position_constraints.empty() && req.motion_plan_request.goal_constraints.orientation_constraints.empty()))
  {
    ROS_ERROR("CHOMP cannot handle pose contraints yet.");
    return false;
  }

  sensor_msgs::JointState joint_goal_chomp = motion_planning_msgs::jointConstraintsToJointState(req.motion_plan_request.goal_constraints.joint_constraints);
  ROS_INFO("Chomp goal");

  if(joint_goal_chomp.name.size() != joint_goal_chomp.position.size())
  {
    ROS_ERROR("Invalid chomp goal");
    return false;
  }

  for(unsigned int i=0; i<joint_goal_chomp.name.size(); i++)
  {
    ROS_INFO("%s %f",joint_goal_chomp.name[i].c_str(),joint_goal_chomp.position[i]);
  }

  ros::WallTime start_time = ros::WallTime::now();
  ROS_INFO("Received planning request...");
  // get the planning group:
  const ChompRobotModel::ChompPlanningGroup* group = chomp_robot_model_.getPlanningGroup(req.motion_plan_request.group_name);

  if (group==NULL)
  {
    ROS_ERROR("Could not load planning group %s", req.motion_plan_request.group_name.c_str());
    return false;
  }

  ChompTrajectory trajectory(&chomp_robot_model_, trajectory_duration_, trajectory_discretization_);

  // set the start state:
  chomp_robot_model_.jointStateToArray(req.motion_plan_request.start_state.joint_state, trajectory.getTrajectoryPoint(0));

  // set the goal state equal to start state, and override the joints specified in the goal
  // joint constraints
  int goal_index = trajectory.getNumPoints()-1;
  trajectory.getTrajectoryPoint(goal_index) = trajectory.getTrajectoryPoint(0);
  chomp_robot_model_.jointStateToArray(motion_planning_msgs::jointConstraintsToJointState(req.motion_plan_request.goal_constraints.joint_constraints), trajectory.getTrajectoryPoint(goal_index));

  // fix the goal to move the shortest angular distance for wrap-around joints:
  for (int i=0; i<group->num_joints_; i++)
  {
    if (group->chomp_joints_[i].wrap_around_)
    {
      int kdl_index = group->chomp_joints_[i].kdl_joint_index_;
      double start = trajectory(0, kdl_index);
      double end = trajectory(goal_index, kdl_index);
      trajectory(goal_index, kdl_index) = start + angles::shortest_angular_distance(start, end);
    }
  }

  // fill in an initial quintic spline trajectory
  trajectory.fillInMinJerk();

  // set the max planning time:
  chomp_parameters_.setPlanningTimeLimit(req.motion_plan_request.allowed_planning_time.toSec());

  // optimize!
  ChompOptimizer optimizer(&trajectory, &chomp_robot_model_, group, &chomp_parameters_,
      vis_marker_array_publisher_, vis_marker_publisher_, &chomp_collision_space_);
  optimizer.optimize();

  // assume that the trajectory is now optimized, fill in the output structure:

  std::vector<double> velocity_limits(group->num_joints_, std::numeric_limits<double>::max());

  // fill in joint names:
  res.trajectory.joint_trajectory.joint_names.resize(group->num_joints_);
  for (int i=0; i<group->num_joints_; i++)
  {
    res.trajectory.joint_trajectory.joint_names[i] = group->chomp_joints_[i].joint_name_;
    // try to retrieve the joint limits:
    if (joint_velocity_limits_.find(res.trajectory.joint_trajectory.joint_names[i])==joint_velocity_limits_.end())
    {
      node_handle_.param("joint_velocity_limits/"+res.trajectory.joint_trajectory.joint_names[i], joint_velocity_limits_[res.trajectory.joint_trajectory.joint_names[i]], std::numeric_limits<double>::max());
    }
    velocity_limits[i] = joint_velocity_limits_[res.trajectory.joint_trajectory.joint_names[i]];
  }

  res.trajectory.joint_trajectory.header = req.motion_plan_request.start_state.joint_state.header; // @TODO this is probably a hack

  // fill in the entire trajectory
  res.trajectory.joint_trajectory.points.resize(trajectory.getNumPoints());
  for (int i=0; i<=goal_index; i++)
  {
    res.trajectory.joint_trajectory.points[i].positions.resize(group->num_joints_);
    for (int j=0; j<group->num_joints_; j++)
    {
      int kdl_joint_index = chomp_robot_model_.urdfNameToKdlNumber(res.trajectory.joint_trajectory.joint_names[j]);
      res.trajectory.joint_trajectory.points[i].positions[j] = trajectory(i, kdl_joint_index);
    }
    if (i==0)
      res.trajectory.joint_trajectory.points[i].time_from_start = ros::Duration(0.0);
    else
    {
      double duration = trajectory.getDiscretization();
      // check with all the joints if this duration is ok, else push it up
      for (int j=0; j<group->num_joints_; j++)
      {
        double d = fabs(res.trajectory.joint_trajectory.points[i].positions[j] - res.trajectory.joint_trajectory.points[i-1].positions[j]) / velocity_limits[j];
        if (d > duration)
          duration = d;
      }
      res.trajectory.joint_trajectory.points[i].time_from_start = res.trajectory.joint_trajectory.points[i-1].time_from_start + ros::Duration(duration);
    }
  }

  ROS_INFO("Serviced planning request in %f wall-seconds, trajectory duration is %f", (ros::WallTime::now() - start_time).toSec(), res.trajectory.joint_trajectory.points[goal_index].time_from_start.toSec());
  return true;
}


} // namespace chomp

int main(int argc, char** argv)
{
  ros::init(argc, argv, "chomp_planner_node");
  ros::NodeHandle node_handle("~");
  chomp::ChompPlannerNode chomp_planner_node(node_handle);
  if (!chomp_planner_node.init())
    return 1;
  return chomp_planner_node.run();
}
