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
                                                   filter_chain_("motion_planning_msgs::FilterJointTrajectory::Request"),
                                                   filter_constraints_chain_("motion_planning_msgs::FilterJointTrajectoryWithConstraints::Request")
{
  std::string service_type_string;
  private_handle_.param<std::string>("service_type",service_type_string,"FilterJointTrajectory");
  private_handle_.param<bool>("use_safety_limits",use_safety_limits_,true);

  if(service_type_string == std::string("FilterJointTrajectory"))
    service_type_ = FILTER_JOINT_TRAJECTORY;
  else if(service_type_string == std::string("FilterJointTrajectoryWithConstraints"))
  {
    ROS_INFO("Filtering joint trajectories with constraints");
    service_type_ = FILTER_JOINT_TRAJECTORY_WITH_CONSTRAINTS;
  }
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

  if(!loadURDF())
    return false;

  return true;
}

bool TrajectoryFilterServer::filter(motion_planning_msgs::FilterJointTrajectory::Request &req,
                                    motion_planning_msgs::FilterJointTrajectory::Response &resp)
{
  ROS_DEBUG("TrajectoryFilter::Got trajectory with %d points and %d joints",
            (int)req.trajectory.points.size(),
            (int)req.trajectory.joint_names.size());
  getLimits(req.trajectory,req.limits);
  motion_planning_msgs::FilterJointTrajectory::Request chain_response;
  if (!filter_chain_.update(req,chain_response))
  {
    ROS_ERROR("Filter chain failed to process trajectory");
    return false;
  }
  resp.trajectory = chain_response.trajectory;
  resp.error_code.val = resp.error_code.SUCCESS;
  return true;
}

bool TrajectoryFilterServer::filterConstraints(motion_planning_msgs::FilterJointTrajectoryWithConstraints::Request &req,
                                               motion_planning_msgs::FilterJointTrajectoryWithConstraints::Response &resp)
{
  ROS_DEBUG("TrajectoryFilter::Got trajectory with %d points and %d joints",
            (int)req.trajectory.points.size(),
            (int)req.trajectory.joint_names.size());
  motion_planning_msgs::FilterJointTrajectoryWithConstraints::Request filter_response;
  getLimits(req.trajectory,req.limits);
  if (!filter_constraints_chain_.update(req,filter_response))
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
      limits.joint_name = trajectory.joint_names[i];
      limits.has_position_limits = false;
      limits.has_velocity_limits = false;
      limits.has_acceleration_limits = false;
      // First load the limits from the urdf
      std::map<std::string, boost::shared_ptr<urdf::Joint> >::const_iterator urdf_joint_iterator = urdf_model_.joints_.find(trajectory.joint_names[i]);
      if(urdf_joint_iterator != urdf_model_.joints_.end())
      {
        boost::shared_ptr<urdf::Joint> urdf_joint = urdf_joint_iterator->second;
        if(use_safety_limits_ && urdf_joint->safety)
        {
          limits.min_position = urdf_joint->safety->soft_lower_limit;
          limits.max_position = urdf_joint->safety->soft_upper_limit;
        }
        else
        {
          limits.min_position = urdf_joint->limits->lower;
          limits.max_position = urdf_joint->limits->upper;          
        }
        limits.max_velocity = urdf_joint->limits->velocity;
        limits.has_velocity_limits = true;

        if(urdf_joint->type == urdf::Joint::CONTINUOUS)
          limits.has_position_limits = false;
        else
          limits.has_position_limits = true;

        limits.has_acceleration_limits = false;
      }
          
      // Now, try to load the limits from the param server if they exist
      private_handle_.getParam("joint_limits/"+trajectory.joint_names[i]+"/min_position",limits.min_position);
      private_handle_.getParam("joint_limits/"+trajectory.joint_names[i]+"/max_position",limits.max_position);
      private_handle_.getParam("joint_limits/"+trajectory.joint_names[i]+"/max_velocity",limits.max_velocity);
      private_handle_.param("joint_limits/"+trajectory.joint_names[i]+"/max_acceleration", 
                            limits.max_acceleration, 
                            std::numeric_limits<double>::max());
      bool boolean;
      if(private_handle_.getParam("joint_limits/"+trajectory.joint_names[i]+"/has_position_limits", boolean))
        limits.has_position_limits = boolean?1:0;
      if(private_handle_.getParam("joint_limits/"+trajectory.joint_names[i]+"/has_velocity_limits", boolean))
        limits.has_velocity_limits = boolean?1:0;
      if(private_handle_.getParam("joint_limits/"+trajectory.joint_names[i]+"/has_acceleration_limits", boolean))
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

bool TrajectoryFilterServer::loadURDF()
{
  std::string robot_desc_string;
  root_handle_.param("robot_description", robot_desc_string, std::string());  
  if (!urdf_model_.initString(robot_desc_string)){
    ROS_ERROR("Failed to parse urdf string");
    return false;
  }
  ROS_INFO("Successfully parsed urdf file");
  return true;
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_filter_server");

  ros::AsyncSpinner spinner(1); 
  spinner.start();   

  trajectory_filter_server::TrajectoryFilterServer traj_filter_server;
  if(traj_filter_server.init())
  {
    ROS_INFO("Started trajectory filter server");
    ros::waitForShutdown();
  }
  else
  {
    ROS_ERROR("Could not start trajectory filter server");
    ROS_ERROR("Check your filters.yaml file to make sure it is configured correctly");
    ROS_ERROR("Also check the ROS parameter: service_type in your launch file");
    ROS_ERROR("The message type in the service request must match the type that the filters are acting on");
    ros::shutdown();
  }
  return 0;
}
