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
#include <spline_smoother/lspb_trajectory.h>


double gen_rand(double min, double max)
{
  int rand_num = rand()%100+1;
  double result = min + (double)((max-min)*rand_num)/101.0;
  return result;
}

TEST(TestLSPBTrajectory, TestLSPBTrajectory)
{
  srand(time(NULL)); // initialize random seed: 
  spline_smoother::LSPBTrajectory traj;

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

  wpt.limits[0].max_acceleration = 0.25;
  wpt.limits[0].has_acceleration_limits = 1;

  wpt.limits[1].max_acceleration = 0.5;
  wpt.limits[1].has_acceleration_limits = 1;

  for (int i=0; i<length; i++)
  {
    wpt.trajectory.points[i].positions.resize(joints);
    wpt.trajectory.points[i].velocities.resize(joints);
    wpt.trajectory.points[i].accelerations.resize(joints);
    for(int j=0; j<joints; j++)
    {
      wpt.trajectory.points[i].positions[j] = i+j+gen_rand(0.0,1.0);
      wpt.trajectory.points[i].velocities[j] = 0.0;
      wpt.trajectory.points[i].accelerations[j] = 0.0;
    }
    wpt.trajectory.points[i].time_from_start = ros::Duration(i);
  }

   wpt.trajectory.points[0].positions[0] = 1.693069;
   wpt.trajectory.points[0].positions[1] = 2.910891;

   wpt.trajectory.points[1].positions[0] = 2.782178;
   wpt.trajectory.points[1].positions[1] = 3.594059;

  FILE* f = fopen("test_lspb_original.txt","w");
  for(int i=0; i<length; i++)
  {
    fprintf(f,"%f ",wpt.trajectory.points[i].time_from_start.toSec());
    for(int j=0; j<joints; j++)
    {
      fprintf(f,"%f ",wpt.trajectory.points[i].positions[j]);
    }
    for(int j=0; j<joints; j++)
    {
      fprintf(f,"%f ",wpt.trajectory.points[i].velocities[j]);
    }
    fprintf(f,"\n");
  }
  fclose(f);

  spline_smoother::LSPBTrajectoryMsg spline;
  bool success = traj.parameterize(wpt.trajectory,wpt.limits,spline);
  EXPECT_TRUE(success);
  // traj->writeSpline(spline,"test_lspb_spline.txt");

  trajectory_msgs::JointTrajectory wpt_out;
  int num_seg = spline.segments.size();
  std::vector<double> knot_times;
  knot_times.resize(num_seg+1);
  knot_times[0] = 0.0;
  for(int i=1; i < (int) knot_times.size(); i++)
  {
    knot_times[i] = knot_times[i-1]+spline.segments[i-1].duration.toSec();
  }
  bool ss = spline_smoother::sampleSplineTrajectory(spline,knot_times,wpt_out);

  EXPECT_TRUE(ss);

  EXPECT_NEAR(wpt.trajectory.points[0].positions[0],wpt_out.points[0].positions[0],1e-5);
  EXPECT_NEAR(wpt.trajectory.points[0].positions[1],wpt_out.points[0].positions[1],1e-5);
  EXPECT_NEAR(wpt.trajectory.points[1].positions[0],wpt_out.points[1].positions[0],1e-5);
  EXPECT_NEAR(wpt.trajectory.points[1].positions[1],wpt_out.points[1].positions[1],1e-5);


  double total_time;
  bool st = spline_smoother::getTotalTime(spline,total_time);
  EXPECT_TRUE(st);
  double dT = 0.01;
  int sample_length = (int) (total_time/dT);
  std::vector<double> times;
  times.resize(sample_length+1);
  for (int i=0; i<sample_length; i++)
  {
    times[i] = dT*i;
  }
  times[sample_length] = total_time;
  bool sw = spline_smoother::write(spline,times,"test_lspb.txt");
  EXPECT_TRUE(sw);

}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
