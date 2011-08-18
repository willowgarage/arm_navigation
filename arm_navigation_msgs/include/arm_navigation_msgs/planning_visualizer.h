/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *
 *  \author Sachin Chitta
 *********************************************************************/

#ifndef ARM_NAVIGATION_PLANNING_VISUALIZER_H_
#define ARM_NAVIGATION_PLANNING_VISUALIZER_H_

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <arm_navigation_msgs/RobotState.h>
#include <arm_navigation_msgs/DisplayTrajectory.h>
#include <arm_navigation_msgs/GetMotionPlan.h>

namespace arm_navigation_msgs
{ 

class PlanningVisualizer
{
public:
  PlanningVisualizer(const std::string &planning_topic_name,
                     const std::string &marker_topic_name)
  {
    display_trajectory_publisher_ = root_handle_.advertise<arm_navigation_msgs::DisplayTrajectory>(planning_topic_name, 1, true);
    marker_publisher_ = root_handle_.advertise<arm_navigation_msgs::DisplayTrajectory>(marker_topic_name, 1, true);
  }

  void visualize(const trajectory_msgs::JointTrajectory &trajectory,
                 const arm_navigation_msgs::RobotState &robot_state)
  {
    arm_navigation_msgs::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory.joint_trajectory = trajectory;
    display_trajectory.robot_state = robot_state;
    display_trajectory_publisher_.publish(display_trajectory);
  }

  void visualize(const sensor_msgs::JointState &joint_state,
                 const arm_navigation_msgs::RobotState &robot_state)
  {
    arm_navigation_msgs::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory.joint_trajectory.points.push_back(arm_navigation_msgs::jointStateToJointTrajectoryPoint(joint_state));
    display_trajectory.robot_state = robot_state;
    display_trajectory_publisher_.publish(display_trajectory);
  }

  void visualize(const trajectory_msgs::JointTrajectory &trajectory,
                 const planning_models::KinematicState *kinematic_state)
  {
    arm_navigation_msgs::RobotState robot_state;
    planning_environment::convertKinematicStateToRobotState(*kinematic_state,
                                                            trajectory.header.stamp,
                                                            trajectory.header.frame_id,
                                                            robot_state);
    visualize(trajectory,robot_state);
  }

  void visualize(const sensor_msgs::JointState &joint_state,
                 const planning_models::KinematicState *kinematic_state)
  {
    arm_navigation_msgs::RobotState robot_state;
    planning_environment::convertKinematicStateToRobotState(*kinematic_state,
                                                            joint_state.header.stamp,
                                                            joint_state.header.frame_id,
                                                            robot_state);
    visualize(joint_state,robot_state);
  }

  void visualize(arm_navigation_msgs::GetMotionPlan::Request &request,
                 const planning_models::KinematicState *kinematic_state)
  {
    arm_navigation_msgs::RobotState robot_state;
    trajectory_msgs::JointTrajectory joint_trajectory = arm_navigation_msgs::jointConstraintsToJointTrajectory(request.motion_plan_request.goal_constraints.joint_constraints);
    planning_environment::convertKinematicStateToRobotState(*kinematic_state,
                                                            joint_trajectory.header.stamp,
                                                            joint_trajectory.header.frame_id,
                                                            robot_state);
    visualize(joint_trajectory,robot_state);
  }

  void visualize(const visualization_msgs::MarkerArray &marker_array)
  {
    marker_publisher_.publish(marker_array);
  }

  void visualize(const std::vector<arm_navigation_msgs::AllowedContactSpecification> &allowed_contacts)
  {
    visualization_msgs::MarkerArray mk;
    mk.markers.resize(allowed_contacts.size());
    for(unsigned int i=0; i < allowed_contacts.size(); i++) 
    { 
      bool valid_shape = true;
      mk.markers[i].header.stamp = ros::Time::now();
      mk.markers[i].header.frame_id = allowed_contacts[i].pose_stamped.header.frame_id;
      mk.markers[i].ns = root_handle_.getNamespace()+allowed_contacts[i].name;
      mk.markers[i].id = marker_counter_++;
      if(allowed_contacts[i].shape.type == arm_navigation_msgs::Shape::SPHERE)
      {        
        mk.markers[i].type = visualization_msgs::Marker::SPHERE;
        if(allowed_contacts[i].shape.dimensions.size() >= 1)
          mk.markers[i].scale.x = mk.markers[i].scale.y = mk.markers[i].scale.z = allowed_contacts[i].shape.dimensions[0];
        else
          valid_shape = false;
      }      
      else if (allowed_contacts[i].shape.type == arm_navigation_msgs::Shape::BOX)
      {
        mk.markers[i].type = visualization_msgs::Marker::CUBE;
        if(allowed_contacts[i].shape.dimensions.size() >= 3)
        {
          mk.markers[i].scale.x = allowed_contacts[i].shape.dimensions[0];
          mk.markers[i].scale.y = allowed_contacts[i].shape.dimensions[1];
          mk.markers[i].scale.z = allowed_contacts[i].shape.dimensions[2];
        }
        else
          valid_shape = false;
      }
      else if (allowed_contacts[i].shape.type == arm_navigation_msgs::Shape::CYLINDER)
      {
        mk.markers[i].type = visualization_msgs::Marker::CYLINDER;
        if(allowed_contacts[i].shape.dimensions.size() >= 2)
        {
          mk.markers[i].scale.x = allowed_contacts[i].shape.dimensions[0];
          mk.markers[i].scale.y = allowed_contacts[i].shape.dimensions[0];
          mk.markers[i].scale.z = allowed_contacts[i].shape.dimensions[1];
        }
        else
          valid_shape = false;
      }
      else
      {
        mk.markers[i].scale.x = mk.markers[i].scale.y = mk.markers[i].scale.z = 0.01;
        valid_shape = false;
      }        
      mk.markers[i].action = visualization_msgs::Marker::ADD;
      mk.markers[i].pose = allowed_contacts[i].pose_stamped.pose;  
      if(!valid_shape)
      {
        mk.markers[i].scale.x = mk.markers[i].scale.y = mk.markers[i].scale.z = 0.01;
        mk.markers[i].color.a = 0.3;
        mk.markers[i].color.r = 1.0;
        mk.markers[i].color.g = 0.04;
        mk.markers[i].color.b = 0.04;
      }
      else
      {
        mk.markers[i].color.a = 1.0;
        mk.markers[i].color.r = 0.04;
        mk.markers[i].color.g = 1.0;
        mk.markers[i].color.b = 0.04;
      }  
    }
    marker_publisher_.publish(mk);
  }

private:
  ros::NodeHandle root_handle_;
  ros::Publisher display_trajectory_publisher_, marker_publisher_;
  unsigned int marker_counter_;

};

}

#endif
