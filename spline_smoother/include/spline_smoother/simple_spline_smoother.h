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

/** \author Kaijen Hsiao, Sachin Chitta */

#ifndef SIMPLE_SPLINE_SMOOTHER_H_
#define SIMPLE_SPLINE_SMOOTHER_H_

#include <spline_smoother/spline_smoother.h>
#include <spline_smoother/spline_smoother_utils.h>

namespace spline_smoother
{

/**
 * \brief
 */
template <typename T>
class SimpleSplineSmoother: public SplineSmoother<T>
{
public:
  SimpleSplineSmoother();
  virtual ~SimpleSplineSmoother();

  virtual bool smooth(const T& trajectory_in, 
                      T& trajectory_out) const;
};

template <typename T>
SimpleSplineSmoother<T>::SimpleSplineSmoother()
{
}

template <typename T>
SimpleSplineSmoother<T>::~SimpleSplineSmoother()
{
}
  
template <typename T>
bool SimpleSplineSmoother<T>::smooth(const T& trajectory_in, 
                                     T& trajectory_out) const
{
  double min_segment_time = 0.01;
  unsigned int num_points = trajectory_in.trajectory.points.size();
  unsigned int num_joints = trajectory_in.trajectory.joint_names.size();
  trajectory_out = trajectory_in;

  if (!checkTrajectoryConsistency(trajectory_out))
    return false;

  ros::Duration first_point(0.05);
  trajectory_out.trajectory.points[0].time_from_start = first_point;

  for(unsigned int i=0; i < num_points; i++)
  {
    double segment_time = 0.0;
    for(unsigned int j=0; j < num_joints; j++)
    {
      double joint_diff = fabs(trajectory_in.trajectory.points[i+1].positions[j] - trajectory_in.trajectory.points[i].positions[j]);
      segment_time = std::max<double>(joint_diff/trajectory_in.limits[j].max_velocity,segment_time);
    }
    trajectory_out.trajectory.points[i+1].time_from_start = trajectory_in.trajectory.points[i].time_from_start + ros::Duration(segment_time);
  }
  
  for(unsigned int i=0; i < num_points; i++)
    trajectory_out.trajectory.points[i].velocities.resize(num_joints);
  
  for(unsigned int j=0; j < num_joints; j++)
  {
    trajectory_out.trajectory.points.front().velocities[j] = 0.0;
    trajectory_out.trajectory.points.back().velocities[j] = 0.0;
  }
  
  for(unsigned int i=1; i < num_points-1; i++)
  {
    for(unsigned int j=0; j < num_joints; j++)
    {
      double first_diff  = trajectory_in.trajectory.points[i].positions[j]-trajectory_in.trajectory.points[i-1].positions[j];
      double second_diff = trajectory_in.trajectory.points[i+1].positions[j]-trajectory_in.trajectory.points[i].positions[j];
      if( (first_diff > 0 && second_diff < 0) || (first_diff < 0 && second_diff > 0))
      {
        trajectory_out.trajectory.points[i].velocities[j] = 0.0;
      }
      else
      {
        double first_vel = first_diff/(trajectory_in.trajectory.points[i].time_from_start-trajectory_in.trajectory.points[i-1].time_from_start).toSec();
        double second_vel = second_diff/(trajectory_in.trajectory.points[i+1].time_from_start-trajectory_in.trajectory.points[i].time_from_start).toSec();
        trajectory_out.trajectory.points[i].velocities[j] = (first_vel+second_vel)/2.0;
      }
    }
  }
  
  std::vector<double> segment_times;
  segment_times.resize(num_points-1);

  for(unsigned int i=1; i < num_points; i++)
  {
    segment_times[i-1] = (trajectory_out.trajectory.points[i].time_from_start - trajectory_in.trajectory.points[i-1].time_from_start).toSec();
    if(segment_times[i-1] < min_segment_time)
      segment_times[i-1] = min_segment_time;
    for(unsigned int j=0; j < num_joints; j++)
    {
      double accn = fabs(trajectory_out.trajectory.points[i].velocities[j] - trajectory_out.trajectory.points[i-1].velocities[j]);
      if(accn > trajectory_in.limits[j].max_acceleration)
      {
        try
        {          
          segment_times[i-1] = accn/trajectory_in.limits[j].max_acceleration;
        }
        catch(...)
        {
          ROS_ERROR("Should not be in here");
        }
      }
    }
    trajectory_out.trajectory.points[i].time_from_start = trajectory_out.trajectory.points[i-1].time_from_start + ros::Duration(segment_times[i-1]);
  }
  return true;
}

}


#endif /* SIMPLE_SPLINE_SMOOTHER_H_ */
