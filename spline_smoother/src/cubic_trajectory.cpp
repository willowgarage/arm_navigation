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

/** \author Sachin Chitta */

#include <spline_smoother/cubic_trajectory.h>

namespace spline_smoother
{
  CubicTrajectory::CubicTrajectory()
  {
    apply_limits_ = true;
  }

  double CubicTrajectory::calculateMinimumTime(const trajectory_msgs::JointTrajectoryPoint &start, 
                                               const trajectory_msgs::JointTrajectoryPoint &end, 
                                               const std::vector<motion_planning_msgs::JointLimits> &limits)
  {
    double minJointTime(MAX_ALLOWABLE_TIME);
    double segmentTime(0);
    int num_joints = (int) start.positions.size();

    for(int i = 0; i < num_joints; i++)
    {
      minJointTime = minSegmentTime(start.positions[i],end.positions[i],start.velocities[i],end.velocities[i],limits[i]);
      if(segmentTime < minJointTime)
        segmentTime = minJointTime;
    }
    return segmentTime;
  }

  double CubicTrajectory::minSegmentTime(const double &q0, 
                                         const double &q1, 
                                         const double &v0, 
                                         const double &v1, 
                                         const motion_planning_msgs::JointLimits &limit)
  {
    double t1(MAX_ALLOWABLE_TIME), t2(MAX_ALLOWABLE_TIME), result(MAX_ALLOWABLE_TIME);
    double dq = jointDiff(q0,q1,limit);
    double vmax = limit.max_velocity;

    /*    double v(0.0);
    if(dq > 0)
      v = vmax;
    else
      v = -vmax;

    double a = 3.0*(v0+v1)*v - 3.0* (v0+v1)*v0 + pow((2.0*v0+v1),2.0);
    double b = -6.0*dq*v + 6.0 * v0 *dq - 6.0*dq*(2.0*v0+v1);
    double c = 9.0 * pow(dq,2);

    if (fabs(a) > EPS_TRAJECTORY)
    {
      if((pow(b,2)-4.0*a*c) >= 0)
      {
        t1 = (-b + sqrt(pow(b,2)-4.0*a*c))/(2.0*a);
        t2 = (-b - sqrt(pow(b,2)-4.0*a*c))/(2.0*a);
      }
    }
    else
    {
      t1 = -c/b;
      t2 = t1;
    }

    if(t1 < 0)
      t1 = MAX_ALLOWABLE_TIME;

    if(t2 < 0)
      t2 = MAX_ALLOWABLE_TIME;

    result = std::min(t1,t2);
    return result;
    */
    if( q0 == q1 && fabs(v0-v1) == 0.0)
    {
      result = 0.0;
      return result;
    }

    double dt_result = 0.01;
    result = dt_result;
    bool done = false;
    if(limit.has_acceleration_limits)
      ROS_DEBUG("Checking acceleration limits");
    double a0(0.0),a1(0.0),a2(0.0),a3(0.0),max_velocity_time(0.0);

    while(!done && result > EPS_TRAJECTORY)
    {
      ROS_DEBUG("Time: %f",result);
      a0 = q0;
      a1 = v0;
      a2 = (3*(q1-q0)-(2*v0+v1)*result)/(result*result);
      a3 = (2*(q0-q1)+(v0+v1)*result)/(result*result*result);
      if(a3 != 0.0)
      {
        max_velocity_time = -2*a2/(6*a3);
        if(max_velocity_time >= 0.0)
	      {
          double max_velocity = a1+2*a2*max_velocity_time+3*a3*max_velocity_time*max_velocity_time;
          if(fabs(max_velocity) <= limit.max_velocity)
            done = true;
          else
            done = false;
	      }
        else
          done = true;
      }
      if(limit.has_acceleration_limits)
        if(fabs(2*a2) <= limit.max_acceleration && fabs(2*a2+6*a3*result) <= limit.max_acceleration)
          done = true;
        else
          done = false;

      if(!done)
        result += dt_result;
    }       
    return result;
  }

  bool CubicTrajectory::parameterize(const motion_planning_msgs::JointTrajectoryWithLimits& trajectory_in, 
                                     spline_smoother::SplineTrajectory& spline)
  {
    int num_traj = trajectory_in.trajectory.points.size();
    int num_joints = trajectory_in.trajectory.joint_names.size();
    spline.names = trajectory_in.trajectory.joint_names;
    spline.segments.resize(num_traj-1);

    for(int i=0; i<num_joints; i++)
    {
      if(!trajectory_in.limits[i].has_velocity_limits)
      {
        ROS_ERROR("Trying to apply velocity limits without supplying them. Set velocity_limits in the message and set has_velocity_limits flag to true.");
        return false;
      }
    }
    for (int i=1; i< num_traj; ++i)
    {
      spline.segments[i-1].joints.resize(num_joints);

      for(int j =0; j < num_joints; j++)
        spline.segments[i-1].joints[j].coefficients.resize(4);
      double dT = (trajectory_in.trajectory.points[i].time_from_start - trajectory_in.trajectory.points[i-1].time_from_start).toSec();
      if(apply_limits_)
      {
      double dTMin = calculateMinimumTime(trajectory_in.trajectory.points[i],trajectory_in.trajectory.points[i-1],trajectory_in.limits);
      if(dTMin > dT) // if minimum time required to satisfy limits is greater than time available, stretch this segment
        dT = dTMin;      
      }
      spline.segments[i-1].duration = ros::Duration(dT);
      for(int j=0; j<num_joints; j++)
      {
        double diff = jointDiff(trajectory_in.trajectory.points[i-1].positions[j],trajectory_in.trajectory.points[i].positions[j],trajectory_in.limits[j]);
        spline.segments[i-1].joints[j].coefficients[0] = trajectory_in.trajectory.points[i-1].positions[j];
        spline.segments[i-1].joints[j].coefficients[1] = trajectory_in.trajectory.points[i-1].velocities[j];
        spline.segments[i-1].joints[j].coefficients[2] = (3*diff-(2*trajectory_in.trajectory.points[i-1].velocities[j]+trajectory_in.trajectory.points[i].velocities[j])*spline.segments[i-1].duration.toSec())/(spline.segments[i-1].duration.toSec()*spline.segments[i-1].duration.toSec());;
        spline.segments[i-1].joints[j].coefficients[3] = (-2*diff+(trajectory_in.trajectory.points[i-1].velocities[j]+trajectory_in.trajectory.points[i].velocities[j])*spline.segments[i-1].duration.toSec())/pow(spline.segments[i-1].duration.toSec(),3);
      }
    }
    return true;
  }
}
