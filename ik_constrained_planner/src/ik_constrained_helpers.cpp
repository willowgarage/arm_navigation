/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
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

/** \author Sachin Chitta, Ioan Sucan */

#include <ik_constrained_planner/ik_constrained_helpers.h>

namespace ik_constrained_planner
{

bool computeRedundancyFromConstraints(const motion_planning_msgs::Constraints &goal_constraint,
                                      geometry_msgs::PoseStamped &kinematics_planner_frame,
                                      kinematics::KinematicsBase *kinematics_solver,
                                      const std::string &redundant_joint_name,
                                      double &redundancy)
{
  geometry_msgs::PoseStamped desired_pose = motion_planning_msgs::poseConstraintsToPoseStamped(goal_constraint.position_constraints[0],
                                                                                               goal_constraint.orientation_constraints[0]);
  btTransform desired_pose_tf;
  btTransform kinematics_planner_frame_tf;
  tf::poseMsgToTF(desired_pose.pose,desired_pose_tf);
  tf::poseMsgToTF(kinematics_planner_frame.pose,kinematics_planner_frame_tf);

  // Now convert to kinematics frame and perform kinematics
  desired_pose_tf = kinematics_planner_frame_tf * desired_pose_tf;

  unsigned int redundant_joint_index(0);
  std::vector<std::string> joint_names = kinematics_solver->getJointNames();
  for(unsigned int i=0; i < joint_names.size(); ++i)
  {
    if(joint_names[i] == redundant_joint_name)
    {
      redundant_joint_index = i;
      break;
    }
  }
  geometry_msgs::Pose pose;
  tf::poseTFToMsg(desired_pose_tf,pose);
  std::vector<double> solution;
  std::vector<double> seed;
  seed.resize(7,0.0);
  solution.resize(7,0.0);
  double timeout = 2.0;
  ROS_DEBUG("Kinematics request: Goal");
  ROS_DEBUG("Position          : %f %f %f",pose.position.x,pose.position.y,pose.position.z);
  ROS_DEBUG("Orientation       : %f %f %f %f",pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
  if(!kinematics_solver->searchPositionIK(pose,seed,timeout,solution))
    return false;

  redundancy = solution[redundant_joint_index];
  return true;
}

}
