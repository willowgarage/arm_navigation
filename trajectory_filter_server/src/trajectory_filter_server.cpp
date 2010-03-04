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

/** \author Mrinal Kalakrishnan, Sachin Chitta */

#include <trajectory_filter_server/trajectory_filter_server.h>

namespace trajectory_filter
{
TrajectoryFilterServer::TrajectoryFilterServer() : priv_handle_("~"),filter_chain_("motion_planning_msgs::JointTrajectoryWithLimits")                                                                 
{
 filter_trajectory_service_ = priv_handle_.advertiseService("filter_trajectory", &TrajectoryFilterServer::filter, this);
}

TrajectoryFilterServer::~TrajectoryFilterServer()
{
}

bool TrajectoryFilterServer::init()
{
  if (!filter_chain_.configure("filter_chain",priv_handle_))
    return false;
  return true;
}

bool TrajectoryFilterServer::filter(motion_planning_msgs::FilterJointTrajectory::Request &req,
                                    motion_planning_msgs::FilterJointTrajectory::Response &resp)
{
  ROS_INFO("TrajectoryFilter::Got trajectory with %d points and %d joints",(int)req.filter_request.trajectory.points.size(),(int)req.filter_request.trajectory.joint_names.size());
  // first convert the input into a "WaypointTrajWithLimits" message
  motion_planning_msgs::JointTrajectoryWithLimits trajectory_in;
  motion_planning_msgs::JointTrajectoryWithLimits trajectory_out;
  jointTrajectoryToJointTrajectoryWithLimits(req.filter_request.trajectory, trajectory_in);

  // run the filters on it:
  if (!filter_chain_.update(trajectory_in, trajectory_out))
  {
    ROS_ERROR("Filter chain failed to process trajectory");
    return false;
  }
  resp.trajectory = trajectory_out.trajectory;
  resp.error_code.val = resp.error_code.SUCCESS;
  return true;
}

void TrajectoryFilterServer::jointTrajectoryToJointTrajectoryWithLimits(const trajectory_msgs::JointTrajectory& joint_traj, motion_planning_msgs::JointTrajectoryWithLimits& waypoint_traj)
{
  waypoint_traj.trajectory = joint_traj;
  int size = joint_traj.points.size();
  int num_joints = joint_traj.joint_names.size();
  for (int i=0; i<size; ++i)
  {
    waypoint_traj.trajectory.points[i].velocities.resize(num_joints, 0.0);
    waypoint_traj.trajectory.points[i].accelerations.resize(num_joints, 0.0);
  }
  waypoint_traj.limits.resize(num_joints);
  for (int i=0; i<num_joints; ++i)
  {
    std::map<std::string, motion_planning_msgs::JointLimits>::const_iterator limit_it = joint_limits_.find(joint_traj.joint_names[i]);
    motion_planning_msgs::JointLimits limits;
    if (limit_it == joint_limits_.end())
    {
      // load the limits from the param server
      priv_handle_.param("joint_limits/"+joint_traj.joint_names[i]+"/min_position", limits.min_position, -std::numeric_limits<double>::max());
      priv_handle_.param("joint_limits/"+joint_traj.joint_names[i]+"/max_position", limits.max_position, std::numeric_limits<double>::max());
      priv_handle_.param("joint_limits/"+joint_traj.joint_names[i]+"/max_velocity", limits.max_velocity, std::numeric_limits<double>::max());
      priv_handle_.param("joint_limits/"+joint_traj.joint_names[i]+"/max_acceleration", limits.max_acceleration, std::numeric_limits<double>::max());
      bool boolean;
      priv_handle_.param("joint_limits/"+joint_traj.joint_names[i]+"/has_position_limits", boolean, false);
      limits.has_position_limits = boolean?1:0;
      priv_handle_.param("joint_limits/"+joint_traj.joint_names[i]+"/has_velocity_limits", boolean, false);
      limits.has_velocity_limits = boolean?1:0;
      priv_handle_.param("joint_limits/"+joint_traj.joint_names[i]+"/has_acceleration_limits", boolean, false);
      limits.has_acceleration_limits = boolean?1:0;
      joint_limits_.insert(make_pair(joint_traj.joint_names[i], limits));
    }
    else
    {
      limits = limit_it->second;
    }
    waypoint_traj.limits[i] = limits;
  }
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_filter_server");
  trajectory_filter::TrajectoryFilterServer traj_filter_server;
  if(traj_filter_server.init())
  {
    ROS_INFO("Started trajectory filter");
    ros::spin();  
  }
  else
    ROS_ERROR("Could not start trajectory filter node");

  return 0;
}
