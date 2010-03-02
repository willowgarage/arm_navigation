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
    double dq = jointDiff(q0,q1,limit);
    double vmax = limit.max_velocity;
    if( q0 == q1 && fabs(v0-v1) == 0.0)
    {
      return 0.0;
    }
    dq = q1-q0;
    double v = vmax;
    double solution;
    std::vector<double> solution_vec;

    double a1 = 3*(v0+v1)*v - 3* (v0+v1)*v0 + (2*v0+v1)*(2*v0+v1);
    double b1 = -6*dq*v + 6 * v0 *dq - 6*dq*(2*v0+v1);
    double c1 = 9*dq*dq;

    double a2 = 3*(v0+v1)*v + 3* (v0+v1)*v0 - (2*v0+v1)*(2*v0+v1);
    double b2 = -6*dq*v - 6 * v0 *dq + 6*dq*(2*v0+v1);
    double c2 = -9*dq*dq;
    if(quadSolve(a1,b1,c1,solution))
      solution_vec.push_back(solution);
    if(quadSolve(a2,b2,c2,solution))
      solution_vec.push_back(solution);

    if(limit.has_acceleration_limits)
    {
      double amax = limit.max_acceleration;
      double a3 = amax/2.0;
      double b3 = 2*v0+v1;
      double c3 = -3*dq;
      if(quadSolve(a3,b3,c3,solution))
        solution_vec.push_back(solution);
      double a4 = amax/2.0;
      double b4 = -(2*v0+v1);
      double c4 = 3*dq;
      if(quadSolve(a4,b4,c4,solution))
        solution_vec.push_back(solution);

      double a5 = amax;
      double b5 = (-2*v0-4*v1);
      double c5 = 6*dq;
      if(quadSolve(a5,b5,c5,solution))
        solution_vec.push_back(solution);

      double a6 = amax;
      double b6 = (2*v0+4*v1);
      double c6 = -6*dq;
      if(quadSolve(a6,b6,c6,solution))
        solution_vec.push_back(solution);
    }
    std::vector<double> positive_durations;
    for(unsigned int i=0; i < solution_vec.size(); i++)
    {
      if(solution_vec[i] > 0)
        positive_durations.push_back(solution_vec[i]);        
    }
    std::sort(positive_durations.begin(),positive_durations.end());
    solution = positive_durations.back();
    return solution;
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
      double dTMin = calculateMinimumTime(trajectory_in.trajectory.points[i-1],trajectory_in.trajectory.points[i],trajectory_in.limits);
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

bool CubicTrajectory::quadSolve(const double &a, const double &b, const double &c, double &solution)
{
  double t1(0.0), t2(0.0);
  if (fabs(a) > 0.0)
  {
    double discriminant = b*b-4*a*c;
    if (discriminant > 0)
    {
      t1 = (-b + sqrt(discriminant))/(2*a);
      t2 = (-b - sqrt(discriminant))/(2*a);
      ROS_DEBUG("t1:%f t2:%f",t1,t2);
      solution = std::max(t1,t2);
      ROS_DEBUG("Solution: %f",solution);
      return true;
    }
    else
      return false;
  }
  else
  {
    if(fabs(b) == 0.0)
      return false;
    t1 = -c/b;
    t2 = t1;
    solution = t1;
    ROS_DEBUG("Solution: %f",solution);
    return true;
  }
}

}
