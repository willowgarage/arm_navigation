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

namespace trajectory_filter_server
{
TrajectoryFilterServer::TrajectoryFilterServer() : private_handle_("~"),
                                                   filter_chain_("motion_planning_msgs::FilterJointTrajectoryRequest"),
                                                   filter_constraints_chain_("motion_planning_msgs::FilterJointTrajectoryWithConstraintsRequest")
{
  std::string service_type_string;
  private_handle_.param<std::string>("service_type",service_type_string,"FilterJointTrajectory");

  if(service_type_string == std::string("FilterJointTrajectory"))
    service_type_ = FILTER_JOINT_TRAJECTORY;
  else if(service_type_string == std::string("FilterJointTrajectoryWithConstraints"))
    service_type_ = FILTER_JOINT_TRAJECTORY_WITH_CONSTRAINTS;
  else
    service_type_ = FILTER_JOINT_TRAJECTORY;
  
  if(service_type_ == FILTER_JOINT_TRAJECTORY)
    filter_service_ = private_handle_.advertiseService("filter_trajectory", &TrajectoryFilterServer::filter, this);
  if(service_type_ == FILTER_JOINT_TRAJECTORY_WITH_CONSTRAINTS)
    filter_constraints_service_ = private_handle_.advertiseService("filter_trajectory_with_constraints", &TrajectoryFilterServer::filterConstraints, this);
}

TrajectoryFilterServer::~TrajectoryFilterServer()
{
}

bool TrajectoryFilterServer::init()
{
  if(service_type_ == FILTER_JOINT_TRAJECTORY)
    if (!filter_chain_.configure("filter_chain",private_handle_))
      return false;
  if(service_type_ == FILTER_JOINT_TRAJECTORY_WITH_CONSTRAINTS)
    if (!filter_constraints_chain_.configure("filter_chain",private_handle_))
      return false;

  return true;
}

bool TrajectoryFilterServer::filter(motion_planning_msgs::FilterJointTrajectory::Request &req,
                                    motion_planning_msgs::FilterJointTrajectory::Response &resp)
{
  ROS_DEBUG("TrajectoryFilter::Got trajectory with %d points and %d joints",
            (int)req.filter_request.trajectory.points.size(),
            (int)req.filter_request.trajectory.joint_names.size());
  motion_planning_msgs::FilterJointTrajectoryRequest filter_response;
  getLimits(req.filter_request.trajectory,req.filter_request.limits);
  if (!filter_chain_.update(req.filter_request, filter_response))
  {
    ROS_ERROR("Filter chain failed to process trajectory");
    return false;
  }
  resp.trajectory = filter_response.trajectory;
  resp.error_code.val = resp.error_code.SUCCESS;
  return true;
}

bool TrajectoryFilterServer::filterConstraints(motion_planning_msgs::FilterJointTrajectoryWithConstraints::Request &req,
                                               motion_planning_msgs::FilterJointTrajectoryWithConstraints::Response &resp)
{
  ROS_DEBUG("TrajectoryFilter::Got trajectory with %d points and %d joints",
            (int)req.filter_request.trajectory.points.size(),
            (int)req.filter_request.trajectory.joint_names.size());
  motion_planning_msgs::FilterJointTrajectoryWithConstraintsRequest filter_response;
  getLimits(req.filter_request.trajectory,req.filter_request.limits);
  if (!filter_constraints_chain_.update(req.filter_request, filter_response))
  {
    ROS_ERROR("Filter chain failed to process trajectory");
    return false;
  }
  resp.trajectory = filter_response.trajectory;
  resp.error_code.val = resp.error_code.SUCCESS;
  return true;
}

void TrajectoryFilterServer::getLimits(const trajectory_msgs::JointTrajectory& trajectory, 
                                       std::vector<motion_planning_msgs::JointLimits>& limits_out)
{
  int num_joints = trajectory.joint_names.size();
  limits_out.resize(num_joints);
  for (int i=0; i<num_joints; ++i)
  {
    std::map<std::string, motion_planning_msgs::JointLimits>::const_iterator limit_it = joint_limits_.find(trajectory.joint_names[i]);
    motion_planning_msgs::JointLimits limits;
    if (limit_it == joint_limits_.end())
    {
      // load the limits from the param server
      private_handle_.param("joint_limits/"+trajectory.joint_names[i]+"/min_position", limits.min_position, -std::numeric_limits<double>::max());
      private_handle_.param("joint_limits/"+trajectory.joint_names[i]+"/max_position", limits.max_position, std::numeric_limits<double>::max());
      private_handle_.param("joint_limits/"+trajectory.joint_names[i]+"/max_velocity", limits.max_velocity, std::numeric_limits<double>::max());
      private_handle_.param("joint_limits/"+trajectory.joint_names[i]+"/max_acceleration", limits.max_acceleration, std::numeric_limits<double>::max());
      bool boolean;
      private_handle_.param("joint_limits/"+trajectory.joint_names[i]+"/has_position_limits", boolean, false);
      limits.has_position_limits = boolean?1:0;
      private_handle_.param("joint_limits/"+trajectory.joint_names[i]+"/has_velocity_limits", boolean, false);
      limits.has_velocity_limits = boolean?1:0;
      private_handle_.param("joint_limits/"+trajectory.joint_names[i]+"/has_acceleration_limits", boolean, false);
      limits.has_acceleration_limits = boolean?1:0;
      joint_limits_.insert(make_pair(trajectory.joint_names[i], limits));
    }
    else
    {
      limits = limit_it->second;
    }
    limits_out[i] = limits;
  }
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_filter_server");
  trajectory_filter_server::TrajectoryFilterServer traj_filter_server;
  if(traj_filter_server.init())
  {
    ROS_INFO("Started trajectory filter server");
    ros::spin();  
  }
  else
  {
    ROS_ERROR("Could not start trajectory filter server");
    ROS_ERROR("Check your filters.yaml file to make sure it is configured correctly");
    ROS_ERROR("Also check the ROS parameter: service_type in your launch file");
    ROS_ERROR("The message type in the service request must match the type that the filters are acting on");
  }
  return 0;
}
