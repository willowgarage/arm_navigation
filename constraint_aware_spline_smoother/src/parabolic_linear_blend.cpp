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

/** \author Ken Anderson */

#include <constraint_aware_spline_smoother/KunzStilman/Trajectory.h>
#include <constraint_aware_spline_smoother/parabolic_linear_blend.h>
#include <arm_navigation_msgs/FilterJointTrajectoryWithConstraints.h>
#include <arm_navigation_msgs/JointLimits.h>
#include <Eigen/Core>

using namespace constraint_aware_spline_smoother;

const double	DEFAULT_VEL_MAX=1.0;
const double	DEFAULT_ACCEL_MAX=1.0;
const double	ROUNDING_THRESHOLD = 0.01;
const int			NUM_OUTPUT_POINTS = 100;	// TODO - make this a parameter


template <typename T>
bool ParabolicLinearBlendSmoother<T>::smooth(const T& trajectory_in,
                                   T& trajectory_out) const
{
  std::list<Eigen::VectorXd> path;
  Eigen::VectorXd vmax;	// velocity
  Eigen::VectorXd amax;	// acceleration
  double smin = 0.03;		// minimum waypoint separation

  // Convert to expected form
  vmax.resize(trajectory_in.request.limits.size());
  amax.resize(trajectory_in.request.limits.size());

  // limits
  for(unsigned int i=0; i<trajectory_in.request.limits.size(); i++)
  {
    if( trajectory_in.request.limits[i].has_velocity_limits )
    {
      vmax[i] = trajectory_in.request.limits[i].max_velocity;
    }
    else
    {
      vmax[i] = DEFAULT_VEL_MAX;
    }
    if( trajectory_in.request.limits[i].has_acceleration_limits )
    {
      amax[i] = trajectory_in.request.limits[i].max_acceleration;
    }
    else
    {
      amax[i] = DEFAULT_ACCEL_MAX;
    }
  }


  // trajectory points
  for(unsigned int i=0; i<trajectory_in.request.trajectory.points.size(); i++)
  {
    Eigen::VectorXd point;
    point.resize(trajectory_in.request.trajectory.points[i].positions.size());
    for(unsigned int j=0; j<trajectory_in.request.trajectory.points[i].positions.size(); j++)
    {
      point[j]=trajectory_in.request.trajectory.points[i].positions[j];
    }
    std::vector<double> positions=trajectory_in.request.trajectory.points[i].positions;
    path.push_back(point);
  }

  ParabolicBlend::Trajectory blend_trajectory(path,vmax,amax,smin);

  // Convert back
  trajectory_out = trajectory_in;
  unsigned int num_joints = trajectory_in.request.trajectory.joint_names.size();
  unsigned int num_points = NUM_OUTPUT_POINTS;
  double duration = blend_trajectory.getDuration();
  double discretization = duration / num_points;

  // add on the new trjectory points
  trajectory_out.request.trajectory.points.clear();
  for (unsigned int i=0; i<=num_points; ++i)
  {
   double time_from_start = discretization * i;
    if(i==0) time_from_start=0.0;	//safety check.

    Eigen::VectorXd positions = blend_trajectory.getPosition( time_from_start );
    Eigen::VectorXd velocities = blend_trajectory.getVelocity( time_from_start );

    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start = ros::Duration(time_from_start);
    point.positions.resize(num_joints);
    point.velocities.resize(num_joints);
    for (unsigned int j=0; j<num_joints; ++j)
    {
      point.positions[j] = positions[j];
      point.velocities[j] = velocities[j];
    }

    trajectory_out.request.trajectory.points.push_back(point);	// TODO - can make this faster by initializing the size
  }

  return true;
}


PLUGINLIB_DECLARE_CLASS(constraint_aware_spline_smoother,
                        ParabolicLinearBlendFilterJointTrajectoryWithConstraints,
                        constraint_aware_spline_smoother::ParabolicLinearBlendSmoother<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>,
                        filters::FilterBase<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>)
