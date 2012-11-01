/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MOTION_PLANNING_RVIZ_PLUGIN_PLANNING_DISPLAY_H
#define MOTION_PLANNING_RVIZ_PLUGIN_PLANNING_DISPLAY_H

#include "rviz/display.h"

#include <arm_navigation_msgs/DisplayTrajectory.h>
#include <std_msgs/Bool.h>

#include <ros/ros.h>

#include <map>

namespace Ogre
{
class Entity;
class SceneNode;
}

namespace planning_models
{
class KinematicModel;
}

namespace planning_environment
{
class RobotModels;
}

namespace rviz
{
class Robot;
class BoolProperty;
class FloatProperty;
class StringProperty;
class RosTopicProperty;
}

namespace motion_planning_rviz_plugin
{

/**
 * \class PlanningDisplay
 * \brief
 */
class PlanningDisplay : public rviz::Display
{
public:
  PlanningDisplay();
  virtual ~PlanningDisplay();

  virtual void onInitialize();

  /**
   * \brief Set the robot description parameter
   * @param description_param The ROS parameter name which contains the robot xml description
   */
  void changedRobotDescription(  );

  /**
   * \brief Set the topic to listen on for the JointPath message
   * @param topic The ROS topic
   */
  void changedTopic(  );

  /**
   * \brief Set the amount of time each state should display for
   * @param time The length of time, in seconds
   */
  void changedStateDisplayTime(  );

  /**
   * \brief Set whether the visual mesh representation should be displayed
   * @param visible
   */
  void changedVisualVisible(  );

  /**
   * \brief Set whether the collision representation should be displayed
   * @param visible
   */
  void changedCollisionVisible(  );

  void changedAlpha();
  void changedLoopDisplay();
        
  virtual void update(float wall_dt, float ros_dt);

  // Overrides from Display
  virtual void fixedFrameChanged();

protected:
  /**
   * \brief Subscribes to any ROS topics we need to subscribe to
   */
  void subscribe();
  /**
   * \brief Unsubscribes from all ROS topics we're currently subscribed to
   */
  void unsubscribe();

  /**
   * \brief Advertises any ROS topics
   */
  void advertise();

  /**
   * \brief Unadvertises all ROS topics that we have advertised
   */
  void unadvertise();

  /**
   * \brief Loads a URDF from our #description_param_
   */
  void load();

  /**
   * \brief ROS callback for an incoming kinematic path message
   */
  void incomingJointPath(const arm_navigation_msgs::DisplayTrajectory::ConstPtr& msg);

  /**
   * \brief Uses libTF to set the robot's position, given the target frame and the planning frame
   */
  void calculateRobotPosition();

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  std::string description_param_;             ///< ROS parameter that contains the robot xml description

  rviz::Robot* robot_;                              ///< Handles actually drawing the robot

  ros::Subscriber sub_;
  std::string kinematic_path_topic_;
  planning_environment::RobotModels* env_models_;
  const planning_models::KinematicModel* kinematic_model_;
  arm_navigation_msgs::DisplayTrajectory::ConstPtr incoming_kinematic_path_message_;
  arm_navigation_msgs::DisplayTrajectory::ConstPtr displaying_kinematic_path_message_;
  bool new_kinematic_path_;
  bool animating_path_;
  int current_state_;
  bool loop_display_;
  float state_display_time_;
  float current_state_time_;
  float alpha_;

  rviz::BoolProperty* visual_enabled_property_;
  rviz::BoolProperty* collision_enabled_property_;
  rviz::FloatProperty* state_display_time_property_;
  rviz::StringProperty* robot_description_property_;
  rviz::RosTopicProperty* topic_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::BoolProperty* loop_display_property_;

  ros::Publisher state_publisher_;

};

} // namespace motion_planning_rviz_plugin

 #endif


