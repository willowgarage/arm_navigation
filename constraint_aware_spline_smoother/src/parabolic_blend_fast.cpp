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
#include <list>
#include <Eigen/Core>

using namespace constraint_aware_spline_smoother;

const double DEFAULT_VEL_MAX=1.0;
const double DEFAULT_ACCEL_MAX=1.0;
const double ROUNDING_THRESHOLD = 0.01;


void PrintPoint(const trajectory_msgs::JointTrajectoryPoint& point, unsigned int i)
{
    ROS_ERROR("time [%i]=%f",i,point.time_from_start.toSec());
    ROS_ERROR("pos  [%i]=%f %f %f %f %f %f %f",i,
      point.positions[0],point.positions[1],point.positions[2],point.positions[3],point.positions[4],point.positions[5],point.positions[6]);
    ROS_ERROR(" vel [%i]=%f %f %f %f %f %f %f",i,
      point.velocities[0],point.velocities[1],point.velocities[2],point.velocities[3],point.velocities[4],point.velocities[5],point.velocities[6]);
    ROS_ERROR("  acc[%i]=%f %f %f %f %f %f %f",i,
      point.accelerations[0],point.accelerations[1],point.accelerations[2],point.accelerations[3],point.accelerations[4],point.accelerations[5],point.accelerations[6]);
}

// FIXME-remove
template <typename T>
void PrintStats(const T& trajectory)
{
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

// Applies velocity/acceleration constraints
// and returns a modified the range of possible time values for point2
void CalculateRange(
    const trajectory_msgs::JointTrajectoryPoint& point1,
    const trajectory_msgs::JointTrajectoryPoint& point2,
    const std::vector<arm_navigation_msgs::JointLimits>& joint_limits,
    ValueRange& possible_times)
{
  unsigned int num_joints = point1.positions.size();

 // Calculate range of times that can fulfill acceleration and velocity constraints
 for (unsigned int j=0; j<num_joints; ++j)
 {
   double v_max = 1.0;
   if( joint_limits[j].has_velocity_limits )
   {
     v_max = joint_limits[j].max_velocity;
   }
   double a_max = 1.0;
   if( joint_limits[j].has_velocity_limits )
   {
     a_max = joint_limits[j].max_acceleration;
   }

   const double d1 = point1.positions[j];
   const double d2 = point2.positions[j];
   const double v1 = point1.velocities[j];
   const double t1 = point1.time_from_start.toSec();
   double a;
   double d = d2 - d1;
   double v2;

   // Find point (d2,t) on the two parabolas (one for positive acceleration, one for negative)
   // There are four possible points-- two for each parabola.
   // But not all four points necessarily exist.
   a = a_max;
   double t_low, t_high;

   if( ( v1*v1 + 2*a_max*d ) >= 0.0 )
   { // positive acceleration constraints apply
     // intersection with maximum positive acceleration parabola
     t_high = ((-v1) + sqrt(v1*v1+2*a_max*d))/a_max;
     //ROS_ERROR("[%i][%i] Positive acceleration constraint t_high=%f",i,j,t_high);
     v2 = v1 + a*t_high;
     if( std::abs(v2) > v_max )
     {	// intersection with maximum positive velocity line
       //ROS_ERROR("  Positive velocity constraint t_high=%f",t_high);
       t_high = ( d2 - (v_max-v1)*(v1-v_max)/(2*a_max) ) / (v_max);
     }

     // intersection with maximum positive acceleration parabola
     t_low = ((-v1) - sqrt(v1*v1+2*a_max*d))/a_max;
     //ROS_ERROR("[%i][%i] Positive acceleration constraint t_low=%f",i,j,t_low);
     v2 = v1 + a*t_low;
     if( std::abs(v2) > v_max )
     {	// intersection with maximum negative velocity line
       t_low = ( d2 + (v_max+v1)*(v1+v_max)/(2*a_max) ) / (-v_max);
       //ROS_ERROR("  Positive velocity constraint t_low=%f",t_low);
     }

     // Add to constraint range
     possible_times.RemoveRange(t1+t_low,t1+t_high);
     //possible_times.Print();	//FIXME- remove
   }

   if( ( v1*v1 + 2*(-a_max)*d ) >= 0.0 )
   { // negative acceleration constraints apply
     // intersection with maximum negative acceleration parabola
     t_low = ((-v1) + sqrt(v1*v1+2*(-a_max)*d))/(-a_max);
     //ROS_ERROR("[%i][%i] Negative acceleration constraint t_low=%f",i,j,t_low);
     v2 = v1 + a*t_low;
     if( std::abs(v2) > v_max )
     {	// intersection with maximum negative velocity line
       t_low = ( d2 - (v_max-v1)*(v1-v_max)/(2*(-a_max)) ) / (v_max);
       //ROS_ERROR("  Negative velocity constraint t_low=%f",t_low);
     }

     // intersection with maximum negative acceleration parabola
     t_high = ((-v1) - sqrt(v1*v1+2*(-a_max)*d))/(-a_max);
     //ROS_ERROR("[%i][%i] Negative acceleration constraint t_high=%f",i,j,t_high);
     v2 = v1 + a*t_high;
     if( std::abs(v2) > v_max )
     { // intersection with maximum positive velocity line
       t_high = ( d2 + (v_max+v1)*(v1+v_max)/(2*(-a_max)) ) / (-v_max);
       //ROS_ERROR("  Negative velocity constraint t_high=%f",t_high);
     }

     // Add to constraint range
     possible_times.RemoveRange(t1+t_low,t1+t_high);
     //possible_times.Print();	//FIXME- remove
   }
 }
}


// Calculates the correct velocity and acceleration of a point's joint,
// after the time of the second point has been set.
// Should be called for each joint.
void CalculateJointValues(
    const trajectory_msgs::JointTrajectoryPoint& point1,
    const trajectory_msgs::JointTrajectoryPoint& point2,
    const std::vector<arm_navigation_msgs::JointLimits>& joint_limits,
    const unsigned int joint_index,
    double& velocity,
    double& acceleration )
{
  const unsigned int j = joint_index;

  // Find the correct velocity & acceleration
  double v_max = 1.0;
  if( joint_limits[j].has_velocity_limits )
  {
    v_max = joint_limits[j].max_velocity;
  }
  double a_max = 1.0;
  if( joint_limits[j].has_velocity_limits )
  {
    a_max = joint_limits[j].max_acceleration;
  }

  const double t1 = point1.time_from_start.toSec();
  const double t2 = point2.time_from_start.toSec();
  const double d1 = point1.positions[j];
  const double d2 = point2.positions[j];
  const double v1 = point1.velocities[j];

  const double t = t2 - t1;
  const double d = d2 - d1;
  double a = a_max + 500; 	// (invalid acceleration)
  double v2 = v_max + 500;	// (invalid velocity)

  // First solution
  const double v2_high = ( d + std::abs(d-v1*t) ) / t;
  const double a_high = (v2_high - v1 ) / t;

  // Second solution
  const double v2_low = ( d - std::abs(d-v1*t) ) / t;
  const double a_low = ( v2_low - v1 ) / t;

  // Select the best
  if( std::abs(v2_high) < v_max + ROUNDING_THRESHOLD )
  {
    if( std::abs(a_high) < a_max + ROUNDING_THRESHOLD )
    {
      a = a_high;
      v2 = v2_high;
      //ROS_ERROR("  Recalculated solution 1 [%i][%i]: v2=%f a=%f", i,j,v2,a);
    }
  }

  if( std::abs(v2_low) < v_max + ROUNDING_THRESHOLD )
  {
    if( std::abs(a_low) < a_max + ROUNDING_THRESHOLD )
    {
      //ROS_ERROR("  Recalculated solution 2 [%i][%i]: v2=%f a=%f", i,j,v2_low,a_low);
      // choose the one with higher velocity
      if( std::abs(v2_low) > std::abs(v2) )
      {
        a = a_low;
        v2 = v2_low;
      }
    }
  }

  acceleration = a;
  velocity = v2;

  if( std::abs(a) > a_max + ROUNDING_THRESHOLD )
  {
    ROS_ERROR("************** INVALID acceleration: j=%i a=%f *****************",j,a);
    ROS_ERROR(" d=%f t=%f v1=%f ",d,t,v1);
    ROS_ERROR(" v2_high=%f a_high=%f, v2_low=%f a_low=%f", v2_high, (v2_high-v1) / t, v2_low, (v2_low-v1)/t);
  }
  ROS_ASSERT( std::abs(a) <= a_max + ROUNDING_THRESHOLD );
  ROS_ASSERT( std::abs(v2) <= v_max + ROUNDING_THRESHOLD );
}

template <typename T>
bool ParabolicBlendFastSmoother<T>::smooth(const T& trajectory_in,
                                   T& trajectory_out) const
{
  bool success = true;
  const unsigned int num_points = trajectory_in.trajectory.points.size();
  const unsigned int num_joints = trajectory_in.trajectory.joint_names.size();
  const unsigned int center = num_points/2;
  trajectory_out = trajectory_in;	//copy

  // Init
  for (unsigned int i=0; i<num_points; ++i)
  {
    trajectory_msgs::JointTrajectoryPoint& point = trajectory_out.trajectory.points[i];
    point.velocities.resize(num_joints);
    point.accelerations.resize(num_joints);
    for (unsigned int j=0; j<num_joints; ++j)
    {
      point.velocities[j] = 0.0;
      point.accelerations[j] = 0.0;
    }
  }

  //FIXME-remove
  ROS_ERROR("Initial Trajectory");
  PrintStats(trajectory_out);

  // Change the time intervals and set the velocity and accelerations values
  for (unsigned int i=0; i<center; ++i)
  {
    trajectory_msgs::JointTrajectoryPoint& point1 = trajectory_out.trajectory.points[i];
    trajectory_msgs::JointTrajectoryPoint& point2 = trajectory_out.trajectory.points[i+1];
    ROS_ERROR("starting to apply constraints");

    // Find the intersection of the ranges of all feasible times for each joint.
    ValueRange possible_times(point1.time_from_start.toSec());
    CalculateRange(point1,point2,trajectory_out.limits,possible_times);

    // Choose the first valid time
    point2.time_from_start = ros::Duration( possible_times.GetFirstValidTime() );
    ROS_ERROR("[%i] Possible Times:",i);
    possible_times.Print();	//FIXME-- remove

    // re-calculate velocity and acceleration with the new time value
    for( unsigned int j=0; j<num_joints; ++j)
    {
      double velocity, acceleration;
      CalculateJointValues( point1, point2, trajectory_out.limits, j, velocity, acceleration );
      point1.accelerations[j] = acceleration;
      point2.velocities[j] = velocity;
    }
    PrintPoint( point1, i);
    PrintPoint( point2, i+1);
  }

  // Change the time intervals and set the velocity and accelerations values
  // Backwards direction
  for (unsigned int i=num_points-1; i>=center+2; --i)
  {
    trajectory_msgs::JointTrajectoryPoint& point1 = trajectory_out.trajectory.points[i];
    trajectory_msgs::JointTrajectoryPoint& point2 = trajectory_out.trajectory.points[i-1];
    ROS_ERROR("starting to apply constraints");

    // Find the intersection of the ranges of all feasible times for each joint.
    ValueRange possible_times(point1.time_from_start.toSec());
    CalculateRange(point1,point2,trajectory_out.limits,possible_times);

    // Choose the first valid time
    point2.time_from_start = ros::Duration( possible_times.GetFirstValidTime() );
    ROS_ERROR("[%i] Possible Times:",i);
    possible_times.Print();	//FIXME-- remove

    // re-calculate velocity and acceleration with the new time value
    for( unsigned int j=0; j<num_joints; ++j)
    {
      double velocity, acceleration;
      CalculateJointValues( point1, point2, trajectory_out.limits, j, velocity, acceleration );
      point2.accelerations[j] = (acceleration);
      point2.velocities[j] = (velocity);
    }
    PrintPoint( point1, i);
    PrintPoint( point2, i-1);
  }

  // Invert the second half
  if( center < num_points )
  {
    const double t_total =
        trajectory_out.trajectory.points[center].time_from_start.toSec() +
        trajectory_out.trajectory.points[center+1].time_from_start.toSec();
    for (unsigned int i=center+1; i<num_points; ++i)
    {
      trajectory_msgs::JointTrajectoryPoint& point1 = trajectory_out.trajectory.points[i];
      point1.time_from_start = ros::Duration( t_total - point1.time_from_start.toSec() );
      for (unsigned int j=0; j<num_joints; ++j)
      {
        point1.velocities[j] = (-point1.velocities[j]);
        if(i<num_points-1)
        {
          point1.accelerations[j] = (point1.accelerations[j]);
        }
      }
    }
  }

  ROS_ERROR("Acceration/Velocity-Constrained Trajectory");
  PrintStats(trajectory_out);

  return success;
}

//////////////////////////////////////////
////////////// ValueRange ////////////////
//////////////////////////////////////////

ValueRange::ValueRange(double start, double end)
{ ValueRangeType entry;
  entry.start = start;
  entry.end = end;
  possible_times_.insert(possible_times_.begin(),entry);
}

double ValueRange::GetFirstValidTime()
{
 ROS_ASSERT(!possible_times_.empty());
 return possible_times_.begin()->start;
}

void ValueRange::RemoveRange(double start,double end)
{
  for (std::list<ValueRangeType>::iterator it = possible_times_.begin(); it != possible_times_.end(); /* Careful with copy paste*/)
  {
    if( start <= it->start && end >= it->end)
    {	// delete.  Leave pointer where it is.
      //ROS_ERROR("RemoveRange() - delete (%f,%f)",start,end);
      it = possible_times_.erase(it);
    }
    else if( start <= it->start && end > it->start && end <= it->end )
    {	// update range entry
      //ROS_ERROR("RemoveRange() - update start (%f,%f)",start,end);
      it->start = end;
      it++;
    }
    else if( start >= it->start && start < it->end && end >= it->end )
    {	// update range entry
      //ROS_ERROR("RemoveRange() - update end (%f,%f)",start,end);
      it->end = start;
      it++;
    }
    else if( start > it->start && end < it->end )
    {	// split the entry
      //ROS_ERROR("RemoveRange() - split (%f,%f)",start,end);
      ValueRangeType entry;
      entry.start = end;
      entry.end = it->end;
      it->end = start;

      it++;
      possible_times_.insert(it,entry);
    }
    else
    {
      //ignore
      //ROS_ERROR("RemoveRange() - ignore (%f,%f)",start,end);
      it++;
    }
  }
}

void ValueRange::Print()
{
  std::stringstream stream;
  stream << "ValueRange: ";
  for (std::list<ValueRangeType>::iterator it = possible_times_.begin(); it != possible_times_.end(); ++it)
  {
    stream << "[" << it->start << "," << it->end << "] ";
  }
  ROS_ERROR("%s",stream.str().c_str());
}


PLUGINLIB_REGISTER_CLASS(ParabolicBlendFastFilterJointTrajectoryWithConstraints,
                         constraint_aware_spline_smoother::ParabolicBlendFastSmoother<arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Request>,
                         filters::FilterBase<arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Request>)
