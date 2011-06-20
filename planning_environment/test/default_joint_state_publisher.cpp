/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

/** \author E. Gil Jones */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <planning_models/kinematic_state.h>
#include <planning_environment/models/robot_models.h>
#include <tf/transform_broadcaster.h>

static const std::string JOINT_STATES_TOPIC = "/joint_states";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "default_joint_state_publisher");

  ROS_INFO("got info");
 
  ros::NodeHandle nh;

  //for odom combined
  tf::TransformBroadcaster odom_broadcaster;

  std::vector<geometry_msgs::TransformStamped> trans_vector;
  
  geometry_msgs::TransformStamped odom_ident;
  odom_ident.header.frame_id = "odom_combined";
  odom_ident.child_frame_id = "base_footprint";
  odom_ident.transform.rotation.w = 1.0;

  trans_vector.push_back(odom_ident);

  geometry_msgs::TransformStamped map_to_odom;
  map_to_odom.header.frame_id = "map";
  map_to_odom.child_frame_id = "odom_combined";  
  map_to_odom.transform.translation.x = -3.0;
  map_to_odom.transform.rotation.w = 1.0;

  trans_vector.push_back(map_to_odom);

  geometry_msgs::TransformStamped map_to_stapler;
  map_to_stapler.header.frame_id = "map";
  map_to_stapler.child_frame_id = "map_to_stapler";  
  map_to_stapler.transform.translation.x = 1.0;
  map_to_stapler.transform.translation.y = -3.0;
  map_to_stapler.transform.rotation.w = 1.0;

  trans_vector.push_back(map_to_stapler);

  std::string robot_description_name = nh.resolveName("robot_description", true);
  
  ros::WallRate h(10.0);

  while(nh.ok() && !nh.hasParam(robot_description_name)) {
    ros::spinOnce();
    h.sleep();
  }

  ROS_INFO_STREAM("Got description");

  planning_environment::RobotModels rmodel(robot_description_name);

  ROS_INFO_STREAM("Made models");

  ros::Publisher joint_state_publisher = nh.advertise<sensor_msgs::JointState>(JOINT_STATES_TOPIC, 1);

  planning_models::KinematicState state(rmodel.getKinematicModel());

  state.setKinematicStateToDefault();

  std::map<std::string, double> joint_state_map;
  state.getKinematicStateValues(joint_state_map);

  sensor_msgs::JointState joint_state;
  joint_state.name.resize(joint_state_map.size());
  joint_state.position.resize(joint_state_map.size());
  joint_state.velocity.resize(joint_state_map.size());
  unsigned int i = 0;
  for(std::map<std::string, double>::iterator it = joint_state_map.begin(); it != joint_state_map.end(); it++, i++) {
    joint_state.name[i] = it->first;
    joint_state.position[i] = it->second;
    joint_state.velocity[i] = 0.0;
  }

  ros::WallRate r(100.0);
  while(nh.ok()) {
    ros::Time ts = ros::Time::now();
    
    joint_state.header.stamp = ts;
    joint_state_publisher.publish(joint_state);
    for(unsigned int i = 0; i < trans_vector.size(); i++) {
      trans_vector[i].header.stamp = ts;
    }
    odom_broadcaster.sendTransform(trans_vector);
    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();
}
  
  


