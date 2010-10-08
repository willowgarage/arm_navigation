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

/** \author Ioan Sucan */

#include "ik_constrained_planner/ik_state_validator.h"

namespace ik_constrained_planner
{    
bool IKStateValidator::operator()(const ompl::base::State *s) const
{
  //  double c_x = 0.6;
  //  double c_y = -0.45;

  //  int test = planning_environment::PlanningMonitor::COLLISION_TEST;  

  btVector3 tmp_pos(s->values[0],s->values[1],s->values[2]);
  btQuaternion tmp_rot;
  tmp_rot.setRPY(s->values[3],s->values[4],s->values[5]);
  btTransform tmp_transform(tmp_rot,tmp_pos);
  btTransform result = kinematics_planner_tf_*tmp_transform;
  geometry_msgs::Pose pose;
  tf::poseTFToMsg(result,pose);
  ROS_DEBUG("Checking ik for pose: %f %f %f, %f %f %f %f",
           pose.position.x,
           pose.position.y,
           pose.position.z,
           pose.orientation.x,
           pose.orientation.y,
           pose.orientation.z,
           pose.orientation.w);
  std::vector<double> solution;
  std::vector<double> seed;
  seed.resize(space_information_->getStateDimension(),0.0);
  seed[redundant_joint_index_] = s->values[6];

  if(!kinematics_solver_->getPositionIK(pose,seed,solution))
    {
      ROS_DEBUG("Could not find IK pose");
    return false;
    }
  ROS_DEBUG("IK Solution: %f %f %f %f %f %f %f",solution[0],solution[1],solution[2],solution[3],solution[4],solution[5],solution[6]);
  
  std::map<std::string, double> joint_map_values;
  for(unsigned int i=0; i < kinematics_solver_->getJointNames().size(); i++)
  {
    joint_map_values[kinematics_solver_->getJointNames()[i]] = solution[i];
  }
  group_state_->setKinematicState(joint_map_values);
  planning_monitor_->getEnvironmentModel()->updateRobotModel(kinematic_state_);

  bool valid = (!planning_monitor_->getEnvironmentModel()->isCollision() &&  !planning_monitor_->getEnvironmentModel()->isSelfCollision());

  if(!valid)
    ROS_DEBUG("State is in collision");

//   motion_planning_msgs::DisplayTrajectory d_path;
//   ros::NodeHandle root_handle;
//   if(valid)
//   {
//     ROS_INFO("Collision check was false for group %s",group_name_.c_str());
//     double cyl_distance = sqrt((pose.position.x-c_x)*(pose.position.x-c_x)+(pose.position.y-c_y)*(pose.position.y-c_y));
//     double cyl_radius = 0.1;
//     if(cyl_distance < cyl_radius)
//     {
//       ROS_ERROR("This is not right");
//       d_path.model_id = group_name_;
//       ros::ServiceClient get_state_client = root_handle.serviceClient<planning_environment_msgs::GetRobotState>("/environment_server_right_arm/get_robot_state");
//       planning_environment_msgs::GetRobotState::Request req;
//       planning_environment_msgs::GetRobotState::Response res;
//       if(get_state_client.call(req,res))
//         d_path.robot_state = res.robot_state;
//       else
//         ROS_ERROR("IKStateValidator:: Service call to get robot state failed on %s",
//                   get_state_client.getService().c_str());
//       d_path.trajectory.joint_trajectory.points.resize(1);
//       d_path.trajectory.joint_trajectory.points[0].positions = solution;
//       d_path.trajectory.joint_trajectory.joint_names = kinematics_solver_->getJointNames();
//       display_path_publisher_.publish(d_path);
//       ros::Duration(10.0).sleep();
//     }
//   }
//   else
//     ROS_INFO("Collision check was true for group %s",group_name_.c_str());
    
  return valid;    
}

void IKStateValidator::printSettings(std::ostream &out) const
{    
  out << "Path constraints:" << std::endl;
}

void IKStateValidator::configure(const std::string &group_name, 
                                 const std::string &redundant_joint_name,                      
                                 const geometry_msgs::Pose &kinematics_planner_offset,
                                 planning_models::KinematicState* kinematic_state,
                                 kinematics::KinematicsBase *kinematics_solver)
{
  kinematics_solver_ = kinematics_solver;
  group_name_ = group_name;
  kinematic_state_ = kinematic_state;
  group_state_ = kinematic_state_->getJointStateGroup(group_name_);
  std::vector<std::string> joint_names = kinematics_solver_->getJointNames();
  for(unsigned int i=0; i < joint_names.size(); ++i)
  {
    if(joint_names[i] == redundant_joint_name)
    {
      redundant_joint_index_ = i;
      break;
    }
  }
  btTransform tmp;
  tf::poseMsgToTF(kinematics_planner_offset,tmp);
  kinematics_planner_tf_ = tmp;

  ros::NodeHandle root_handle;
  //Set up mechanism for displaying paths
  display_path_publisher_ = root_handle.advertise<motion_planning_msgs::DisplayTrajectory>("ik_constrained_planner_debug", 1, true);  
  // end mechanism for displaying paths
}

}
