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

#include <gtest/gtest.h>
#include <arm_navigation_msgs/JointTrajectoryWithLimits.h>
#include <spline_smoother/cubic_parameterized_trajectory.h>

double gen_rand(double min, double max)
{
  int rand_num = rand()%100+1;
  double result = min + (double)((max-min)*rand_num)/101.0;
  return result;
}

TEST(TestCubicParameterizedTrajectory, TestCubicParameterizedTrajectoryWithWrapAround)
{
  spline_smoother::CubicParameterizedTrajectory traj;

  // create the input:
  int length = 2;
  int joints = 1;

  arm_navigation_msgs::JointTrajectoryWithLimits wpt;
  wpt.trajectory.points.resize(length);
  wpt.trajectory.joint_names.resize(joints);
  wpt.trajectory.joint_names[0] = std::string("test0");

  wpt.limits.resize(joints);
  wpt.limits[0].max_velocity = 0.5;
  wpt.limits[0].has_velocity_limits = 1;

  //  wpt.limits[1].max_velocity = 0.25;
  //  wpt.limits[1].has_velocity_limits = 1;

  for (int i=0; i<length; i++)
  {
    wpt.trajectory.points[i].positions.resize(joints);
    wpt.trajectory.points[i].velocities.resize(joints);
    wpt.trajectory.points[i].accelerations.resize(joints);
    for(int j=0; j<joints; j++)
    {
      wpt.trajectory.points[i].positions[j] = 0.0;
      wpt.trajectory.points[i].velocities[j] = 0.0;
      wpt.trajectory.points[i].accelerations[j] = 0.0;
    }
    wpt.trajectory.points[i].time_from_start = ros::Duration(0.0);
  }

  wpt.trajectory.points[0].positions[0] = M_PI;
  wpt.trajectory.points[1].positions[0] = -M_PI;

  spline_smoother::SplineTrajectory spline;
  bool success = traj.parameterize(wpt.trajectory,wpt.limits,spline);

  double total_time;
  bool ss = spline_smoother::getTotalTime(spline,total_time);
  EXPECT_TRUE(ss);

  trajectory_msgs::JointTrajectory wpt_out;
  std::vector<double> times_out;
  times_out.resize(length);
  times_out[0] = 0.0;
  times_out[1] = total_time;
  spline_smoother::sampleSplineTrajectory(spline,times_out,wpt_out);
  EXPECT_NEAR(wpt_out.points[0].positions[0], M_PI, 1e-5);
  EXPECT_NEAR(wpt_out.points[1].positions[0], -M_PI, 1e-5);
}


TEST(TestCubicParameterizedTrajectory, TestCubicParameterizedTrajectory)
{
  spline_smoother::CubicParameterizedTrajectory traj;

  // create the input:
  int length = 4;
  int joints = 2;

  arm_navigation_msgs::JointTrajectoryWithLimits wpt;
  wpt.trajectory.points.resize(length);
  wpt.trajectory.joint_names.resize(joints);
  wpt.trajectory.joint_names[0] = std::string("test0");
  wpt.trajectory.joint_names[1] = std::string("test1");

  wpt.limits.resize(joints);
  wpt.limits[0].max_velocity = 0.5;
  wpt.limits[0].has_velocity_limits = 1;

  wpt.limits[1].max_velocity = 0.25;
  wpt.limits[1].has_velocity_limits = 1;

  for (int i=0; i<length; i++)
  {
    wpt.trajectory.points[i].positions.resize(joints);
    wpt.trajectory.points[i].velocities.resize(joints);
    wpt.trajectory.points[i].accelerations.resize(joints);
    for(int j=0; j<joints; j++)
    {
      wpt.trajectory.points[i].positions[j] = i+j;
      wpt.trajectory.points[i].velocities[j] = 0.0;
      wpt.trajectory.points[i].accelerations[j] = 0.0;
    }
    wpt.trajectory.points[i].time_from_start = ros::Duration(0.0);
  }

  spline_smoother::SplineTrajectory spline;
  bool success = traj.parameterize(wpt.trajectory,wpt.limits,spline);

  double total_time;
  bool ss = spline_smoother::getTotalTime(spline,total_time);
  EXPECT_TRUE(ss);

  double dT = 0.01;
  int sample_length = (int) (total_time/dT);

  std::vector<double> times;
  times.resize(sample_length);
  for (int i=0; i<sample_length; i++)
  {
    times[i] = dT*i;
  }

  EXPECT_TRUE(success);

  trajectory_msgs::JointTrajectory wpt_out;
  std::vector<double> times_out;
  times_out.resize(length);
  for (int i=0; i<length; i++)
  {
    times_out[i] = i;
  }
  spline_smoother::sampleSplineTrajectory(spline,times_out,wpt_out);
  EXPECT_NEAR(wpt_out.points[0].positions[0], 0.0, 1e-5);
  EXPECT_NEAR(wpt_out.points[1].positions[0], 0.0740741, 1e-5);
  EXPECT_NEAR(wpt_out.points[2].positions[0], 0.259259, 1e-5);
  EXPECT_NEAR(wpt_out.points[3].positions[0], 0.5, 1e-5);

  EXPECT_NEAR(wpt_out.points[0].positions[1], 1.0, 1e-5);
  EXPECT_NEAR(wpt_out.points[1].positions[1], 1.0740741, 1e-5);
  EXPECT_NEAR(wpt_out.points[2].positions[1], 1.259259, 1e-5);
  EXPECT_NEAR(wpt_out.points[3].positions[1], 1.5, 1e-5);
}

TEST(TestCubicParameterizedTrajectory, TestWithAccelerationLimits1)
{
  spline_smoother::CubicParameterizedTrajectory traj;

  // create the input:
  int length = 2;
  int joints = 1;

  arm_navigation_msgs::JointTrajectoryWithLimits wpt;
  wpt.trajectory.points.resize(length);
  wpt.trajectory.joint_names.resize(joints);
  wpt.trajectory.joint_names[0] = std::string("test0");

  wpt.limits.resize(joints);
  wpt.limits[0].max_velocity = 0.2;
  wpt.limits[0].has_velocity_limits = 1;

  wpt.limits[0].max_acceleration = 0.1;
  wpt.limits[0].has_acceleration_limits = 1;

  for (int i=0; i<length; i++)
  {
    wpt.trajectory.points[i].positions.resize(joints);
    wpt.trajectory.points[i].velocities.resize(joints);
    wpt.trajectory.points[i].accelerations.resize(joints);
    wpt.trajectory.points[i].time_from_start = ros::Duration(0.0);
  }
  wpt.trajectory.points[1].positions[0] = 1.0;
  spline_smoother::SplineTrajectory spline;
  bool success = traj.parameterize(wpt.trajectory,wpt.limits,spline);
  EXPECT_TRUE(success);

  double total_time;
  bool ss = spline_smoother::getTotalTime(spline,total_time);
  EXPECT_TRUE(ss);
  EXPECT_NEAR(total_time,7.74597,1e-5);

  trajectory_msgs::JointTrajectory wpt_out;
  std::vector<double> times_out;
  times_out.push_back(total_time);
  spline_smoother::sampleSplineTrajectory(spline,times_out,wpt_out);

  EXPECT_NEAR(wpt.trajectory.points[1].positions[0],wpt_out.points[0].positions[0],1e-5);
  EXPECT_NEAR(wpt.trajectory.points[1].velocities[0],wpt_out.points[0].velocities[0],1e-5);
}

TEST(TestCubicParameterizedTrajectory, TestWithAccelerationLimits2)
{
  spline_smoother::CubicParameterizedTrajectory traj;
  srand ( time(NULL) ); // initialize random seed: 

  // create the input:
  int length = 2;
  int joints = 1;

  arm_navigation_msgs::JointTrajectoryWithLimits wpt;
  wpt.trajectory.points.resize(length);
  wpt.trajectory.joint_names.resize(joints);
  wpt.trajectory.joint_names[0] = std::string("test0");

  wpt.limits.resize(joints);
  wpt.limits[0].max_velocity = 0.2;
  wpt.limits[0].has_velocity_limits = 1;

  wpt.limits[0].max_acceleration = 0.1;
  wpt.limits[0].has_acceleration_limits = 1;

  for (int i=0; i<length; i++)
  {
    wpt.trajectory.points[i].positions.resize(joints);
    wpt.trajectory.points[i].velocities.resize(joints);
    wpt.trajectory.points[i].accelerations.resize(joints);
    wpt.trajectory.points[i].time_from_start = ros::Duration(0.0);
  }
  wpt.trajectory.points[1].positions[0] = 1.0;
  wpt.trajectory.points[1].velocities[0] = 0.0;
  spline_smoother::SplineTrajectory spline;
  bool success = traj.parameterize(wpt.trajectory,wpt.limits,spline);
  EXPECT_TRUE(success);

  double total_time;
  bool ss = spline_smoother::getTotalTime(spline,total_time);
  EXPECT_TRUE(ss);
  //  EXPECT_NEAR(total_time,12.717798,1e-5);

  ROS_INFO("Next test");
  wpt.trajectory.points[1].velocities[0] = 0.0;
  success = traj.parameterize(wpt.trajectory,wpt.limits,spline);
  EXPECT_TRUE(success);
  ss = spline_smoother::getTotalTime(spline,total_time);
  EXPECT_TRUE(ss);
  //  EXPECT_NEAR(total_time,7.5,1e-5);

  trajectory_msgs::JointTrajectory wpt_out;
  std::vector<double> times_out;
  times_out.push_back(total_time);
  spline_smoother::sampleSplineTrajectory(spline,times_out,wpt_out);

  //  EXPECT_NEAR(wpt.trajectory.points[1].positions[0],wpt_out.points[0].positions[0],1e-5);
  //  EXPECT_NEAR(wpt.trajectory.points[1].velocities[0],wpt_out.points[0].velocities[0],1e-5);

  wpt.trajectory.points[0].positions[0] = -0.000720;
  wpt.trajectory.points[1].positions[0] = -0.0000080;
  wpt.trajectory.points[0].velocities[0] = .000028;
  wpt.trajectory.points[1].velocities[0] = 0.000034;
  wpt.limits[0].max_velocity = 1.0;
  wpt.limits[0].max_acceleration = 0.5;

  success = traj.parameterize(wpt.trajectory,wpt.limits,spline);
  EXPECT_TRUE(success);
  ss = spline_smoother::getTotalTime(spline,total_time);
  EXPECT_TRUE(ss);
  EXPECT_NEAR(total_time,0.092254,1e-3);

    double eps = 1e-2;
  for(unsigned int i=0; i < 2000; i++)
  {
    wpt.trajectory.points[0].positions[0] = gen_rand(-100.0,100.0);
    wpt.trajectory.points[1].positions[0] = gen_rand(-100.0,100.0);
    wpt.limits[0].max_velocity = fabs(gen_rand(-100.0,100.0));
    wpt.limits[0].max_acceleration = fabs(gen_rand(-100.0,100.0));
    //    wpt.trajectory.points[0].velocities[0] = gen_rand(-wpt.limits[0].max_velocity,wpt.limits[0].max_velocity);
    //    wpt.trajectory.points[1].velocities[0] = gen_rand(-wpt.limits[0].max_velocity,wpt.limits[0].max_velocity);
    wpt.trajectory.points[0].velocities[0] = 0.0;
    wpt.trajectory.points[1].velocities[0] = 0.0;
    if(wpt.trajectory.points[0].positions[0] == wpt.trajectory.points[1].positions[0])
      continue;
    success = traj.parameterize(wpt.trajectory,wpt.limits,spline);
    EXPECT_TRUE(success);
    ss = spline_smoother::getTotalTime(spline,total_time);
    std::vector<double> test_times;
    double dT = 0.0;
    while(dT < total_time)
    {
      test_times.push_back(dT);
      dT += 0.01;
    }
    test_times.push_back(total_time);
    trajectory_msgs::JointTrajectory test_trajectory;
    spline_smoother::sampleSplineTrajectory(spline,test_times,test_trajectory);
    for(unsigned int i=0; i < test_trajectory.points.size(); i++)
    {
      double vel_error = fabs(test_trajectory.points[i].velocities[0]) - wpt.limits[0].max_velocity;
      double acc_error = fabs(test_trajectory.points[i].accelerations[0]) - wpt.limits[0].max_acceleration;
      if(!(vel_error <= eps) || isnan(vel_error))
      {
        ROS_INFO("error: %f %f",vel_error,acc_error);      
        ROS_INFO("positions: %f, %f, velocities: %f, %f",wpt.trajectory.points[0].positions[0],
                 wpt.trajectory.points[1].positions[0],
                 wpt.trajectory.points[0].velocities[0],
                 wpt.trajectory.points[1].velocities[0]);
        ROS_INFO("Limits: %f, %f",wpt.limits[0].max_velocity,wpt.limits[0].max_acceleration);
        ROS_INFO(" ");
        ROS_INFO(" ");
        ROS_INFO(" ");
      }
      if(!(acc_error <= eps))
        ROS_INFO("error: %f %f",vel_error,acc_error);      

      EXPECT_TRUE(vel_error <= eps);
      EXPECT_TRUE(acc_error <= eps);      
    }
  }
  
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
