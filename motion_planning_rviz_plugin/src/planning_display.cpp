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
#include "rviz/common.h"
#include "rviz/visualization_manager.h"
#include "rviz/robot/robot.h"
#include "rviz/robot/link_updater.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"

#include <urdf/model.h>

#include <ogre_tools/axes.h>

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
                                 Ogre::Vector3& collision_position, Ogre::Quaternion& collision_orientation, bool& apply_offset_transforms) const
  {
    apply_offset_transforms = false;

    const planning_models::KinematicState::LinkState* link_state = kinematic_state_->getLinkState( link_name );

    if ( !link_state )
    {
      return false;
    }

    btVector3 robot_visual_position = link_state->getGlobalLinkTransform().getOrigin();
    btQuaternion robot_visual_orientation = link_state->getGlobalLinkTransform().getRotation();
    visual_position = Ogre::Vector3( robot_visual_position.getX(), robot_visual_position.getY(), robot_visual_position.getZ() );
    visual_orientation = Ogre::Quaternion( robot_visual_orientation.getW(), robot_visual_orientation.getX(), robot_visual_orientation.getY(), robot_visual_orientation.getZ() );
    rviz::robotToOgre( visual_position );
    rviz::robotToOgre( visual_orientation );

    btVector3 robot_collision_position = link_state->getGlobalLinkTransform().getOrigin();
    btQuaternion robot_collision_orientation = link_state->getGlobalLinkTransform().getRotation();
    collision_position = Ogre::Vector3( robot_collision_position.getX(), robot_collision_position.getY(), robot_collision_position.getZ() );
    collision_orientation = Ogre::Quaternion( robot_collision_orientation.getW(), robot_collision_orientation.getX(), robot_collision_orientation.getY(), robot_collision_orientation.getZ() );
    rviz::robotToOgre( collision_position );
    rviz::robotToOgre( collision_orientation );

    return true;
  }

private:
  const planning_models::KinematicState* kinematic_state_;
};

PlanningDisplay::PlanningDisplay(const std::string& name, rviz::VisualizationManager* manager) :
  Display(name, manager), env_models_(NULL), new_kinematic_path_(false), animating_path_(false), state_display_time_(0.05f)
{
  robot_ = new rviz::Robot(vis_manager_, "Planning Robot " + name_);

  setVisualVisible(false);
  setCollisionVisible(true);

  setLoopDisplay(false);
  setAlpha(0.6f);
}

PlanningDisplay::~PlanningDisplay()
{
  unsubscribe();

  delete env_models_;
  delete robot_;
}

void PlanningDisplay::initialize(const std::string& description_param, const std::string& kinematic_path_topic)
{
  setRobotDescription(description_param);
  setTopic(kinematic_path_topic);
}

void PlanningDisplay::setRobotDescription(const std::string& description_param)
{
  description_param_ = description_param;

  propertyChanged(robot_description_property_);

  if (isEnabled())
  {
    load();
    causeRender();
  }
}

void  PlanningDisplay::setLoopDisplay(bool loop_display)
{
    loop_display_ = loop_display;
    propertyChanged(loop_display_property_);
}

void PlanningDisplay::setAlpha(float alpha)
{
  alpha_ = alpha;

  robot_->setAlpha(alpha_);

  propertyChanged(alpha_property_);
}

void PlanningDisplay::setTopic(const std::string& topic)
{
  unsubscribe();
  unadvertise();
  kinematic_path_topic_ = topic;
  subscribe();
  advertise();

  propertyChanged(topic_property_);
}

void PlanningDisplay::setStateDisplayTime(float time)
{
  state_display_time_ = time;

  propertyChanged(state_display_time_property_);

  causeRender();
}

void PlanningDisplay::setVisualVisible(bool visible)
{
  robot_->setVisualVisible(visible);

  propertyChanged(visual_enabled_property_);

  causeRender();
}

void PlanningDisplay::setCollisionVisible(bool visible)
{
  robot_->setCollisionVisible(visible);

  propertyChanged(collision_enabled_property_);

  causeRender();
}

bool PlanningDisplay::isVisualVisible()
{
  return robot_->isVisualVisible();
}

bool PlanningDisplay::isCollisionVisible()
{
  return robot_->isCollisionVisible();
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
  robot_->load(doc.RootElement(), descr);

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
        std::map<std::string, double> vals;
        for(unsigned int i = 0; i < displaying_kinematic_path_message_->trajectory.joint_trajectory.joint_names.size(); i++) {
          joint_state_map[displaying_kinematic_path_message_->trajectory.joint_trajectory.joint_names[i]] = displaying_kinematic_path_message_->trajectory.joint_trajectory.points[current_state_].positions[i];
        }
        state.setKinematicState(joint_state_map);
        robot_->update(PlanningLinkUpdater(&state));
        causeRender();
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

  tf::Stamped<tf::Pose> pose(btTransform(btQuaternion(0, 0, 0), btVector3(0, 0, 0)), displaying_kinematic_path_message_->trajectory.joint_trajectory.header.stamp,
                             displaying_kinematic_path_message_->trajectory.joint_trajectory.header.frame_id);

  if (vis_manager_->getTFClient()->canTransform(target_frame_, displaying_kinematic_path_message_->trajectory.joint_trajectory.header.frame_id, displaying_kinematic_path_message_->trajectory.joint_trajectory.header.stamp))
  {
    try
    {
      vis_manager_->getTFClient()->transformPose(target_frame_, pose, pose);
    }
    catch (tf::TransformException& e)
    {
      ROS_ERROR( "Error transforming from frame '%s' to frame '%s'", pose.frame_id_.c_str(), target_frame_.c_str() );
    }
  }

  Ogre::Vector3 position(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z());
  rviz::robotToOgre(position);

  btScalar yaw, pitch, roll;
  pose.getBasis().getEulerYPR(yaw, pitch, roll);
  Ogre::Matrix3 orientation;
  orientation.FromEulerAnglesYXZ(Ogre::Radian(yaw), Ogre::Radian(pitch), Ogre::Radian(roll));

  robot_->setPosition(position);
  robot_->setOrientation(orientation);
}

void PlanningDisplay::incomingJointPath(const motion_planning_msgs::DisplayTrajectory::ConstPtr& msg)
{
  incoming_kinematic_path_message_ = msg;
  new_kinematic_path_ = true;
}

void PlanningDisplay::targetFrameChanged()
{
  calculateRobotPosition();
}

void PlanningDisplay::createProperties()
{
  visual_enabled_property_ = property_manager_->createProperty<rviz::BoolProperty> ("Visual Enabled", property_prefix_,
                                                                              boost::bind(&PlanningDisplay::isVisualVisible, this),
                                                                              boost::bind(&PlanningDisplay::setVisualVisible, this, _1), parent_category_, this);
  collision_enabled_property_ = property_manager_->createProperty<rviz::BoolProperty> ("Collision Enabled", property_prefix_,
                                                                                 boost::bind(&PlanningDisplay::isCollisionVisible, this),
                                                                                 boost::bind(&PlanningDisplay::setCollisionVisible, this, _1), parent_category_, this);
  state_display_time_property_ = property_manager_->createProperty<rviz::FloatProperty> ("State Display Time", property_prefix_,
                                                                                   boost::bind(&PlanningDisplay::getStateDisplayTime, this),
                                                                                   boost::bind(&PlanningDisplay::setStateDisplayTime, this, _1), parent_category_,
                                                                                   this);
  rviz::FloatPropertyPtr float_prop = state_display_time_property_.lock();
  float_prop->setMin(0.0001);

  loop_display_property_ = property_manager_->createProperty<rviz::BoolProperty>("Loop Display", property_prefix_, boost::bind(&PlanningDisplay::getLoopDisplay, this),
                                                                                 boost::bind(&PlanningDisplay::setLoopDisplay, this, _1), parent_category_, this);
  
  alpha_property_ = property_manager_->createProperty<rviz::FloatProperty> ("Alpha", property_prefix_, boost::bind(&PlanningDisplay::getAlpha, this),
                                                                      boost::bind(&PlanningDisplay::setAlpha, this, _1), parent_category_, this);

  robot_description_property_ = property_manager_->createProperty<rviz::StringProperty> ("Robot Description", property_prefix_,
                                                                                   boost::bind(&PlanningDisplay::getRobotDescription, this),
                                                                                   boost::bind(&PlanningDisplay::setRobotDescription, this, _1), parent_category_,
                                                                                   this);
  topic_property_ = property_manager_->createProperty<rviz::ROSTopicStringProperty> ("Topic", property_prefix_, boost::bind(&PlanningDisplay::getTopic, this),
                                                                               boost::bind(&PlanningDisplay::setTopic, this, _1), parent_category_, this);
  rviz::ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(motion_planning_msgs::DisplayTrajectory::__s_getDataType());

  robot_->setPropertyManager(property_manager_, parent_category_);
}

} // namespace motion_planning_rviz_plugin


