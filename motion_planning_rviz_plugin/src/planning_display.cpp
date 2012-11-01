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

#include "planning_display.h"

#include "rviz/robot/robot.h"
#include "rviz/robot/link_updater.h"
#include "rviz/properties/property.h"

#include <rviz/visualization_manager.h>

#include <rviz/properties/property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>

#include <urdf/model.h>

#include <tf/transform_listener.h>
#include <planning_environment/models/robot_models.h>
#include <planning_models/kinematic_state.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

namespace motion_planning_rviz_plugin
{

class PlanningLinkUpdater : public rviz::LinkUpdater
{
public:
  PlanningLinkUpdater(const planning_models::KinematicState* state)
    : kinematic_state_(state)
  {}

  virtual bool getLinkTransforms(const std::string& link_name, Ogre::Vector3& visual_position, Ogre::Quaternion& visual_orientation,
                                 Ogre::Vector3& collision_position, Ogre::Quaternion& collision_orientation) const
  {

    const planning_models::KinematicState::LinkState* link_state = kinematic_state_->getLinkState( link_name );

    if ( !link_state )
    {
      return false;
    }

    tf::Vector3 robot_visual_position = link_state->getGlobalLinkTransform().getOrigin();
    tf::Quaternion robot_visual_orientation = link_state->getGlobalLinkTransform().getRotation();
    visual_position = Ogre::Vector3( robot_visual_position.getX(), robot_visual_position.getY(), robot_visual_position.getZ() );
    visual_orientation = Ogre::Quaternion( robot_visual_orientation.getW(), robot_visual_orientation.getX(), robot_visual_orientation.getY(), robot_visual_orientation.getZ() );

    tf::Vector3 robot_collision_position = link_state->getGlobalLinkTransform().getOrigin();
    tf::Quaternion robot_collision_orientation = link_state->getGlobalLinkTransform().getRotation();
    collision_position = Ogre::Vector3( robot_collision_position.getX(), robot_collision_position.getY(), robot_collision_position.getZ() );
    collision_orientation = Ogre::Quaternion( robot_collision_orientation.getW(), robot_collision_orientation.getX(), robot_collision_orientation.getY(), robot_collision_orientation.getZ() );

    return true;
  }

private:
  const planning_models::KinematicState* kinematic_state_;
};

PlanningDisplay::PlanningDisplay():
  Display(), 
  env_models_(NULL), 
  kinematic_model_(NULL),
  new_kinematic_path_(false), 
  animating_path_(false), 
  state_display_time_(0.05f)
{
  visual_enabled_property_ = new rviz::BoolProperty ("Visual Enabled", true, "", this,
                                                     SLOT (changedVisualVisible()), this);
  
  
  collision_enabled_property_ = new rviz::BoolProperty ("Collision Enabled", false, "", this,
                                                        SLOT (changedCollisionVisible()), this);

  state_display_time_property_ = new rviz::FloatProperty("State Display Time", 0.05f, "", this,
                                                         SLOT(changedStateDisplayTime()), this);

  state_display_time_property_->setMin(0.0001);

  loop_display_property_ = new rviz::BoolProperty("Loop Display", false, "", this,
                                                  SLOT(changedLoopDisplay), this);
  
  alpha_property_ = new rviz::FloatProperty ("Alpha", 1.0f, "", this,
                                             SLOT(changedAlpha()), this);
  
  robot_description_property_ = new rviz::StringProperty("Robot Description", "robot_description", "", this,
                                                         SLOT(changedRobotDescription()), this);
  
  topic_property_ = new rviz::RosTopicProperty("Topic", "", ros::message_traits::datatype<arm_navigation_msgs::DisplayTrajectory>(), "", this, 
                                               SLOT(changedTopic()), this);
}

void PlanningDisplay::onInitialize()
{
  robot_ = new rviz::Robot(scene_node_, context_, "Planning Robot", this);  
}


PlanningDisplay::~PlanningDisplay()
{
  unsubscribe();

  delete env_models_;
  delete robot_;
}

void PlanningDisplay::changedRobotDescription()
{
  description_param_ = robot_description_property_->getStdString();

  if (isEnabled())
    load();
}

void  PlanningDisplay::changedLoopDisplay()
{
  loop_display_ = loop_display_property_->getBool();
}

void PlanningDisplay::changedAlpha()
{
  alpha_ = alpha_property_->getFloat();
  robot_->setAlpha(alpha_);
}

void PlanningDisplay::changedTopic()
{
  unsubscribe();
  unadvertise();
  kinematic_path_topic_ = topic_property_->getStdString();
  subscribe();
  advertise();
}

void PlanningDisplay::changedStateDisplayTime()
{
  state_display_time_ = state_display_time_property_->getFloat();
}

void PlanningDisplay::changedVisualVisible()
{
  robot_->setVisualVisible(visual_enabled_property_->getBool());
}

void PlanningDisplay::changedCollisionVisible()
{
  robot_->setCollisionVisible(collision_enabled_property_->getBool());
}

void PlanningDisplay::load()
{
  std::string content;
  if (!update_nh_.getParam(description_param_, content))
  {
    std::string loc;
    if (update_nh_.searchParam(description_param_, loc))
    {
      update_nh_.getParam(loc, content);
    }
  }

  TiXmlDocument doc;
  doc.Parse(content.c_str());
  if (!doc.RootElement())
  {
    return;
  }

  urdf::Model descr;
  descr.initXml(doc.RootElement());
  robot_->load( descr);

  delete env_models_;
  env_models_ = new planning_environment::RobotModels(description_param_);
  kinematic_model_ = env_models_->getKinematicModel();

  planning_models::KinematicState state(kinematic_model_);
  state.setKinematicStateToDefault();

  //robot_->update(PlanningLinkUpdater(&state));
}

void PlanningDisplay::onEnable()
{
  subscribe();
  advertise();
  load();
  robot_->setVisible(true);
}

void PlanningDisplay::onDisable()
{
  unsubscribe();
  unadvertise();
  robot_->setVisible(false);
}

void PlanningDisplay::subscribe()
{
  if (!isEnabled())
  {
    return;
  }

  if (!kinematic_path_topic_.empty())
  {
    sub_ = update_nh_.subscribe(kinematic_path_topic_, 2, &PlanningDisplay::incomingJointPath, this);
  }

}

void PlanningDisplay::unsubscribe()
{
  sub_.shutdown();
}

void PlanningDisplay::advertise()
{
  if (!isEnabled())
  {
    return;
  }
  state_publisher_ = update_nh_.advertise<std_msgs::Bool>(kinematic_path_topic_+std::string("state"), 1);
}

void PlanningDisplay::unadvertise()
{
  state_publisher_.shutdown();
}


void PlanningDisplay::update(float wall_dt, float ros_dt)
{
  if (!kinematic_model_)
    return;

  if (!animating_path_ && !new_kinematic_path_ && loop_display_ && displaying_kinematic_path_message_)
  {
      new_kinematic_path_ = true;
      incoming_kinematic_path_message_ = displaying_kinematic_path_message_;
  }
  
  planning_models::KinematicState state(kinematic_model_);

  if (!animating_path_ && new_kinematic_path_)
  {
    displaying_kinematic_path_message_ = incoming_kinematic_path_message_;

    animating_path_ = true;
    new_kinematic_path_ = false;
    current_state_ = -1;
    current_state_time_ = state_display_time_ + 1.0f;

    for(unsigned int i = 0; i < displaying_kinematic_path_message_->robot_state.multi_dof_joint_state.joint_names.size(); i++) {
      planning_models::KinematicState::JointState* js = state.getJointState(displaying_kinematic_path_message_->robot_state.multi_dof_joint_state.joint_names[i]);
      if(!js) continue;
      if(displaying_kinematic_path_message_->robot_state.multi_dof_joint_state.frame_ids[i] != js->getParentFrameId() ||
         displaying_kinematic_path_message_->robot_state.multi_dof_joint_state.child_frame_ids[i] != js->getChildFrameId()) {
        ROS_WARN_STREAM("Robot state msg has bad multi_dof transform");
      } else {
        tf::StampedTransform transf;
        tf::poseMsgToTF(displaying_kinematic_path_message_->robot_state.multi_dof_joint_state.poses[i], transf);
        js->setJointStateValues(transf);
      }
    }

    std::map<std::string, double> joint_state_map;
    for (unsigned int i = 0 ; i < displaying_kinematic_path_message_->robot_state.joint_state.name.size(); ++i)
    {
      joint_state_map[displaying_kinematic_path_message_->robot_state.joint_state.name[i]] = displaying_kinematic_path_message_->robot_state.joint_state.position[i];
    }
    //overwriting with vals in first value in trajectory
    for(unsigned int i = 0; i < displaying_kinematic_path_message_->trajectory.joint_trajectory.joint_names.size(); i++) {
      joint_state_map[displaying_kinematic_path_message_->trajectory.joint_trajectory.joint_names[i]] = displaying_kinematic_path_message_->trajectory.joint_trajectory.points[0].positions[i];
    }
    state.setKinematicState(joint_state_map);
    robot_->update(PlanningLinkUpdater(&state));
  }

  if (animating_path_)
  {

    for(unsigned int i = 0; i < displaying_kinematic_path_message_->robot_state.multi_dof_joint_state.joint_names.size(); i++) {
      planning_models::KinematicState::JointState* js = state.getJointState(displaying_kinematic_path_message_->robot_state.multi_dof_joint_state.joint_names[i]);
      if(!js) continue;
      if(displaying_kinematic_path_message_->robot_state.multi_dof_joint_state.frame_ids[i] != js->getParentFrameId() ||
         displaying_kinematic_path_message_->robot_state.multi_dof_joint_state.child_frame_ids[i] != js->getChildFrameId()) {
        ROS_WARN_STREAM("Robot state msg has bad multi_dof transform");
      } else {
        tf::StampedTransform transf;
        tf::poseMsgToTF(displaying_kinematic_path_message_->robot_state.multi_dof_joint_state.poses[i], transf);
        js->setJointStateValues(transf);
      }
    }
    std::map<std::string, double> joint_state_map;
    for (unsigned int i = 0 ; i < displaying_kinematic_path_message_->robot_state.joint_state.name.size(); ++i)
    {
      joint_state_map[displaying_kinematic_path_message_->robot_state.joint_state.name[i]] = displaying_kinematic_path_message_->robot_state.joint_state.position[i];
    }
    if (current_state_time_ > state_display_time_)
    {
      ++current_state_;

      calculateRobotPosition();

      if ((size_t) current_state_ < displaying_kinematic_path_message_->trajectory.joint_trajectory.points.size())
      {
        for(unsigned int i = 0; i < displaying_kinematic_path_message_->trajectory.joint_trajectory.joint_names.size(); i++) {
          joint_state_map[displaying_kinematic_path_message_->trajectory.joint_trajectory.joint_names[i]] = displaying_kinematic_path_message_->trajectory.joint_trajectory.points[current_state_].positions[i];
        }

        state.setKinematicState(joint_state_map);
	bool updKstate = false;	
	for(unsigned int i = 0; i < displaying_kinematic_path_message_->trajectory.multi_dof_joint_trajectory.joint_names.size(); i++) {
	    planning_models::KinematicState::JointState* js = state.getJointState(displaying_kinematic_path_message_->trajectory.multi_dof_joint_trajectory.joint_names[i]);
	    if(!js) continue;
	    if(displaying_kinematic_path_message_->trajectory.multi_dof_joint_trajectory.frame_ids[i] != js->getParentFrameId() ||
	       displaying_kinematic_path_message_->trajectory.multi_dof_joint_trajectory.child_frame_ids[i] != js->getChildFrameId()) {
		ROS_WARN_STREAM("Robot state msg has bad multi_dof transform");
	    } else {
		updKstate = true;	
		tf::StampedTransform transf;
		tf::poseMsgToTF(displaying_kinematic_path_message_->trajectory.multi_dof_joint_trajectory.points[current_state_].poses[i], transf);
		js->setJointStateValues(transf);
	    }
	}
	if (updKstate)
	    state.updateKinematicLinks();
	
        robot_->update(PlanningLinkUpdater(&state));
      }
      else
      {
        animating_path_ = false;
        std_msgs::Bool done;
        done.data = !animating_path_;        
        state_publisher_.publish(done);
      }

      current_state_time_ = 0.0f;
    }
    current_state_time_ += wall_dt;
  }
}

void PlanningDisplay::calculateRobotPosition()
{
  if (!displaying_kinematic_path_message_)
  {
    return;
  }

  tf::Stamped<tf::Pose> pose(tf::Transform(tf::Quaternion(0, 0, 0, 1.0), tf::Vector3(0, 0, 0)), displaying_kinematic_path_message_->trajectory.joint_trajectory.header.stamp,
                             displaying_kinematic_path_message_->trajectory.joint_trajectory.header.frame_id);

  if (context_->getTFClient()->canTransform(fixed_frame_.toStdString(), displaying_kinematic_path_message_->trajectory.joint_trajectory.header.frame_id, displaying_kinematic_path_message_->trajectory.joint_trajectory.header.stamp))
  {
    try
    {
      context_->getTFClient()->transformPose(fixed_frame_.toStdString(), pose, pose);
    }
    catch (tf::TransformException& e)
    {
      ROS_ERROR( "Error transforming from frame '%s' to frame '%s'", pose.frame_id_.c_str(), fixed_frame_.toStdString().c_str() );
    }
  }

  Ogre::Vector3 position(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z());

  double yaw, pitch, roll;
  pose.getBasis().getEulerYPR(yaw, pitch, roll);
  Ogre::Matrix3 orientation;
  orientation.FromEulerAnglesYXZ(Ogre::Radian(yaw), Ogre::Radian(pitch), Ogre::Radian(roll));

  robot_->setPosition(position);
  robot_->setOrientation(orientation);
}

void PlanningDisplay::incomingJointPath(const arm_navigation_msgs::DisplayTrajectory::ConstPtr& msg)
{
  incoming_kinematic_path_message_ = msg;
  new_kinematic_path_ = true;
}

void PlanningDisplay::fixedFrameChanged()
{
  calculateRobotPosition();
}


} // namespace motion_planning_rviz_plugin


