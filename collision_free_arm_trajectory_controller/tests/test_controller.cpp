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

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> JointExecutorActionClient;

void spinThread()
{
  ros::spin();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_controller");
  boost::thread spin_thread(&spinThread);
  JointExecutorActionClient *traj_action_client_ = new JointExecutorActionClient("collision_free_arm_trajectory_action_right_arm");

  while(!traj_action_client_->waitForServer(ros::Duration(1.0))){
    ROS_INFO("Waiting for the joint_trajectory_action action server to come up");
    if(!ros::ok()) {
      exit(0);
    }
  }

  pr2_controllers_msgs::JointTrajectoryGoal goal;
  goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
  goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
  goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
  goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
  goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
  goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
  goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

  goal.trajectory.points.resize(1);
  for(unsigned int i=0; i < 7; i++)
    goal.trajectory.points[0].positions.push_back(0.0);
  goal.trajectory.points[0].positions[0] = 0.0;
  goal.trajectory.points[0].positions[0] = -1.57/2.0;
  goal.trajectory.points[0].time_from_start = ros::Duration(0.0);

  traj_action_client_->sendGoal(goal);
  ROS_INFO("Sent goal");

  while(!traj_action_client_->getState().isDone() && ros::ok())
  {
    ros::Duration(0.1).sleep();
  }
  return 0;
}
