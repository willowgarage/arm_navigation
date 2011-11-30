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

#include <constraint_aware_spline_smoother/parabolic_blend_fast.h>
#include <arm_navigation_msgs/FilterJointTrajectoryWithConstraints.h>
#include <arm_navigation_msgs/JointLimits.h>

using namespace constraint_aware_spline_smoother;

const double DEFAULT_VEL_MAX=1.0;
const double DEFAULT_ACCEL_MAX=1.0;
const double ROUNDING_THRESHOLD = 0.01;


template <typename T>
ParabolicBlendFastSmoother<T>::ParabolicBlendFastSmoother()
: max_iterations_(100),
  max_time_change_per_it_(0.1)
{}

template <typename T>
ParabolicBlendFastSmoother<T>::~ParabolicBlendFastSmoother()
{}

template <typename T>
bool ParabolicBlendFastSmoother<T>::configure()
{
  if (!spline_smoother::SplineSmoother<T>::getParam("max_iterations", max_iterations_))
  {
    ROS_WARN("Spline smoother, \"%s\", params has no attribute max_iterations.",
            spline_smoother::SplineSmoother<T>::getName().c_str());
  }
  ROS_DEBUG("Using a max_iterations value of %d",max_iterations_);

  if (!spline_smoother::SplineSmoother<T>::getParam("max_time_change_per_it", max_time_change_per_it_))
  {
    ROS_WARN("Spline smoother, \"%s\", params has no attribute max_time_change_per_it.",
            spline_smoother::SplineSmoother<T>::getName().c_str());
  }
  ROS_DEBUG("Using a max_time_change_per_it value of %f",max_time_change_per_it_);

 return true;
}

template <typename T>
void ParabolicBlendFastSmoother<T>::PrintPoint(const trajectory_msgs::JointTrajectoryPoint& point, unsigned int i) const
{
    ROS_ERROR("time [%i]=%f",i,point.time_from_start.toSec());
    if(point.positions.size() >= 7 )
    {
      ROS_ERROR("pos  [%i]=%f %f %f %f %f %f %f",i,
        point.positions[0],point.positions[1],point.positions[2],point.positions[3],point.positions[4],point.positions[5],point.positions[6]);
    }
    if(point.velocities.size() >= 7 )
    {
      ROS_ERROR(" vel [%i]=%f %f %f %f %f %f %f",i,
        point.velocities[0],point.velocities[1],point.velocities[2],point.velocities[3],point.velocities[4],point.velocities[5],point.velocities[6]);
    }
    if(point.accelerations.size() >= 7 )
    {
      ROS_ERROR("  acc[%i]=%f %f %f %f %f %f %f",i,
        point.accelerations[0],point.accelerations[1],point.accelerations[2],point.accelerations[3],point.accelerations[4],point.accelerations[5],point.accelerations[6]);
    }
}

template <typename T>
void ParabolicBlendFastSmoother<T>::PrintStats(const T& trajectory) const
{
   ROS_ERROR("jointNames=%s %s %s %s %s %s %s",
   trajectory.limits[0].joint_name.c_str(),trajectory.limits[1].joint_name.c_str(),trajectory.limits[2].joint_name.c_str(),
   trajectory.limits[3].joint_name.c_str(),trajectory.limits[4].joint_name.c_str(),trajectory.limits[5].joint_name.c_str(),
   trajectory.limits[6].joint_name.c_str());
 ROS_ERROR("maxVelocities=%f %f %f %f %f %f %f",
   trajectory.limits[0].max_velocity,trajectory.limits[1].max_velocity,trajectory.limits[2].max_velocity,
   trajectory.limits[3].max_velocity,trajectory.limits[4].max_velocity,trajectory.limits[5].max_velocity,
   trajectory.limits[6].max_velocity);
 ROS_ERROR("maxAccelerations=%f %f %f %f %f %f %f",
   trajectory.limits[0].max_acceleration,trajectory.limits[1].max_acceleration,trajectory.limits[2].max_acceleration,
   trajectory.limits[3].max_acceleration,trajectory.limits[4].max_acceleration,trajectory.limits[5].max_acceleration,
   trajectory.limits[6].max_acceleration);
  // for every point in time:
  for (unsigned int i=0; i<trajectory.trajectory.points.size(); ++i)
  {
    const trajectory_msgs::JointTrajectoryPoint& point = trajectory.trajectory.points[i];
    PrintPoint(point, i);
  }
}

// Applies velocity
template <typename T>
void ParabolicBlendFastSmoother<T>::ApplyVelocityConstraints(T& trajectory, std::vector<double> &time_diff) const
{
  //we double the number of points by adding a midpoint between each point.
  const unsigned int num_points = trajectory.trajectory.points.size();
  const unsigned int num_joints = trajectory.trajectory.joint_names.size();

  // Initial values
  for (unsigned int i=0; i<num_points; ++i)
  {
    trajectory_msgs::JointTrajectoryPoint& point = trajectory.trajectory.points[i];
    point.velocities.resize(num_joints);
    point.accelerations.resize(num_joints);
  }

  for (unsigned int i=0; i<num_points-1; ++i)
  {
    trajectory_msgs::JointTrajectoryPoint& point1 = trajectory.trajectory.points[i];
    trajectory_msgs::JointTrajectoryPoint& point2 = trajectory.trajectory.points[i+1];

    // Get velocity min/max
    for (unsigned int j=0; j<num_joints; ++j)
    {
      double v_max = 1.0;

      if( trajectory.limits[j].has_velocity_limits )
      {
        v_max = trajectory.limits[j].max_velocity;
      }
      double a_max = 1.0;
      if( trajectory.limits[j].has_velocity_limits )
      {
        a_max = trajectory.limits[j].max_acceleration;
      }

      const double d1 = point1.positions[j];
      const double d2 = point2.positions[j];
      const double t_min = std::abs(d2-d1) / v_max;

      if( t_min > time_diff[i] )
      {
        time_diff[i] = t_min;
      }
    }
  }
}

// Iteratively expand t1 interval by a constant factor until within acceleration constraint
// In the future we may want to solve to quadratic equation to get the exact timing interval.
// To do this, the solveQuadratic function below is a start
template <typename T>
double ParabolicBlendFastSmoother<T>::findT1( const double d1, const double d2, double t1, const double t2, const double a_max) const
{
  const double mult_factor = 1.01;
  double v1 = (d1)/t1;
  double v2 = (d2)/t2;
  double a = (v2-v1)/(t1+t2);

  while( std::abs( a ) > a_max )
  {
    v1 = (d1)/t1;
    v2 = (d2)/t2;
    a = (v2-v1)/(t1+t2);
    t1 *= mult_factor;
  }

  return t1;
}

template <typename T>
double ParabolicBlendFastSmoother<T>::findT2( const double d1, const double d2, const double t1, double t2, const double a_max) const
{
  const double mult_factor = 1.01;
  double v1 = (d1)/t1;
  double v2 = (d2)/t2;
  double a = (v2-v1)/(t1+t2);

  while( std::abs( a ) > a_max )
  {
    v1 = (d1)/t1;
    v2 = (d2)/t2;
    a = (v2-v1)/(t1+t2);
    t2 *= mult_factor;
  }

  return t2;
}
/*
template <typename T>
double ParabolicBlendFastSmoother<T>::solveQuadratic(
    const double d1, const double d2, const double t1, const double t2, const double a_max ) const
{
  double v2 = d2/t2;
  double a = a_max;
  double discriminant = (a*t2-v2)*(a*t2-v2) - 4*a*(d1);
  double sol = 99999;

  // Grab the minimum positive solution
  if( discriminant > 0.0 )
  {
    double sol1 = (-(a*t2+v2) + std::sqrt(discriminant) ) / (2*a);
    if( sol1 > 0 && sol1 < sol)
      sol = sol1;

    double sol2 = (-(a*t2+v2) - std::sqrt(discriminant) ) / (2*a);
    if( sol2 > 0 && sol2 < sol)
      sol = sol2;
  }

  a = -a_max;
  discriminant = (a*t2-v2)*(a*t2-v2) - 4*a*(d1);
  if( discriminant > 0.0 )
  {
    double sol1 = (-(a*t2+v2) + std::sqrt(discriminant) ) / (2*a);
    if( sol1 > 0 && sol1 < sol)
      sol = sol1;
    double sol2 = (-(a*t2+v2) - std::sqrt(discriminant) ) / (2*a);
    if( sol2 > 0 && sol2 < sol)
      sol = sol2;
  }

  return sol;

  //double t1_new = sol;
  //return std::max( t*1.05, t_new );
}
*/

// Takes the time differences, and updates the values in the trajectory.
template <typename T>
void UpdateTrajectory(T& trajectory, const std::vector<double>& time_diffs )
{
  double time_sum = 0.0;
  unsigned int num_joints = trajectory.trajectory.joint_names.size();
  const unsigned int num_points = trajectory.trajectory.points.size();

	// Error check
	if(time_diffs.size() < 1)
		return;

  // Times
  trajectory.trajectory.points[0].time_from_start = ros::Duration(time_diffs[0]);
  for (unsigned int i=1; i<num_points; ++i)
  {
    time_sum += time_diffs[i-1];
    trajectory.trajectory.points[i].time_from_start = ros::Duration(time_sum);
  }

  // Velocities
  for (unsigned int j=0; j<num_joints; ++j)
  {
    trajectory.trajectory.points[num_points-1].velocities[j] = 0.0;
  }
  for (unsigned int i=0; i<num_points-1; ++i)
  {
    trajectory_msgs::JointTrajectoryPoint& point1 = trajectory.trajectory.points[i];
    trajectory_msgs::JointTrajectoryPoint& point2 = trajectory.trajectory.points[i+1];
    for (unsigned int j=0; j<num_joints; ++j)
    {
      const double d1 = point1.positions[j];
      const double d2 = point2.positions[j];
      const double & t1 = time_diffs[i];
      const double v1 = (d2-d1)/(t1);
      point1.velocities[j]= v1;
    }
  }

  // Accelerations
  for (unsigned int i=0; i<num_points; ++i)
  {
    for (unsigned int j=0; j<num_joints; ++j)
    {
      double v1;
      double v2;
      double t1;
      double t2;

      if(i==0)
      {
        v1 = 0.0;
        v2 = trajectory.trajectory.points[i].velocities[j];
        t1 = time_diffs[i];
        t2 = time_diffs[i];
      }
      else if(i < num_points-1)
      {
        v1 = trajectory.trajectory.points[i-1].velocities[j];
        v2 = trajectory.trajectory.points[i].velocities[j];
        t1 = time_diffs[i-1];
        t2 = time_diffs[i];
      }
      else
      {
        v1 = trajectory.trajectory.points[i-1].velocities[j];
        v2 = 0.0;
        t1 = time_diffs[i-1];
        t2 = time_diffs[i-1];
      }

      const double a = (v2-v1)/(t1+t2);
      trajectory.trajectory.points[i].accelerations[j] = a;
    }
  }
}


// Applies Acceleration constraints
template <typename T>
void ParabolicBlendFastSmoother<T>::ApplyAccelerationConstraints(const T& trajectory, std::vector<double> & time_diff) const
{
  const unsigned int num_points = trajectory.trajectory.points.size();
  const unsigned int num_joints = trajectory.trajectory.joint_names.size();
  int num_updates = 0;
  int iteration= 0;
  bool backwards = false;

  do
  {
    num_updates = 0;
    iteration++;
    // Loop forwards, then backwards
    for( int count=0; count<2; count++)
    {
      ROS_ERROR("ApplyAcceleration: Iteration %i backwards=%i", iteration, backwards);

      for (unsigned int i=0; i<num_points-1; ++i)
      {
        unsigned int index = i;
        if(backwards)
        {
          index = (num_points-1)-i;
        }

        double d1;
        double d2;
        double d3;
        double t1;
        double t2;
        double v1;
        double v2;
        double a;

        // Get acceleration min/max
        for (unsigned int j=0; j<num_joints; ++j)
        {
          double a_max = 1.0;
          if( trajectory.limits[j].has_velocity_limits )
          {
            a_max = trajectory.limits[j].max_acceleration;
          }

          if( index <= 0 )
          {	// First point
            d2 = trajectory.trajectory.points[index].positions[j];
            d1 = d2;
            d3 = trajectory.trajectory.points[index+1].positions[j];
            t1 = time_diff[0];
            t2 = t1;
            ROS_ASSERT(!backwards);
          }
          else if( index < num_points-1 )
          {	// Intermediate Points
            d1 = trajectory.trajectory.points[index-1].positions[j];
            d2 = trajectory.trajectory.points[index].positions[j];
            d3 = trajectory.trajectory.points[index+1].positions[j];
            t1 = time_diff[index-1];
            t2 = time_diff[index];
          }
          else
          {	// Last Point - there are only numpoints-1 time intervals.
            d1 = trajectory.trajectory.points[index-1].positions[j];
            d2 = trajectory.trajectory.points[index].positions[j];
            d3 = d2;
            t1 = time_diff[index-1];
            t2 = t1;
            ROS_ASSERT(backwards);
          }

          v1 = (d2-d1)/t1;
          v2 = (d3-d2)/t2;
          a = (v2-v1)/(t1+t2);

          if( std::abs( a ) > a_max + ROUNDING_THRESHOLD )
          {
            if(!backwards)
            {
              t2 = std::min( t2+max_time_change_per_it_, findT2( d2-d1, d3-d2, t1, t2, a_max) );
              time_diff[index] = t2;
            }
            else
            {
              t1 = std::min( t1+max_time_change_per_it_, findT1( d2-d1, d3-d2, t1, t2, a_max) );
              time_diff[index-1] = t1;
            }
            num_updates++;

            v1 = (d2-d1)/t1;
            v2 = (d3-d2)/t2;
            a = (v2-v1)/(t1+t2);
          }
        }
      }
      backwards = !backwards;
    }
  } while(num_updates > 0 && iteration < max_iterations_);
}

template <typename T>
bool ParabolicBlendFastSmoother<T>::smooth(const T& trajectory_in,
                                   T& trajectory_out) const
{
  bool success = true;
  trajectory_out = trajectory_in;	//copy
  const unsigned int num_points = trajectory_out.trajectory.points.size();
  std::vector<double> time_diff(num_points,0.0);	// the time difference between adjacent points

  //ROS_ERROR("Initial Trajectory");
  //PrintStats(trajectory_in);

  ApplyVelocityConstraints(trajectory_out, time_diff);
  //ROS_ERROR("Velocity Trajectory");//FIXME-remove
  //UpdateTrajectory(trajectory_out, time_diff);
  //PrintStats(trajectory_out);

  ApplyAccelerationConstraints(trajectory_out, time_diff);
  ROS_ERROR("Acceleration Trajectory");//FIXME-remove
  UpdateTrajectory(trajectory_out, time_diff);
  PrintStats(trajectory_out);

  return success;
}


PLUGINLIB_REGISTER_CLASS(ParabolicBlendFastFilterJointTrajectoryWithConstraints,
                         constraint_aware_spline_smoother::ParabolicBlendFastSmoother<arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Request>,
                         filters::FilterBase<arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Request>)
