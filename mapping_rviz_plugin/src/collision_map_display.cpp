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
 *
 *
 */

#include "collision_map_display.h"
#include "rviz/visualization_manager.h"
#include "rviz/frame_manager.h"

#include <tf/transform_listener.h>

#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreBillboardSet.h>

#include <rviz/ogre_helpers/point_cloud.h>
#include <rviz/properties/property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/color_property.h>

namespace mapping_rviz_plugin
{

CollisionMapDisplay::CollisionMapDisplay()
  : Display()
  , color_(0.1f, 1.0f, 0.0f)
  , render_operation_(collision_render_ops::CBoxes)
  , override_color_(false)
  , tf_filter_(NULL)
{
  override_color_property_ = new rviz::BoolProperty ("Override Color", false, "", this, SLOT (changedOverrideColor() ), this);

  color_property_ = new rviz::ColorProperty ("Color", QColor(255, 0, 0), "", this, 
                                             SLOT (changedColor() ), this);
  
  render_operation_property_ = new rviz::EnumProperty ("Render Operation", "", "", this, 
                                                       SLOT( changedRenderOperation() ), this);
  render_operation_property_->addOption("Boxes", collision_render_ops::CBoxes);
  render_operation_property_->addOption("Points", collision_render_ops::CPoints);

  alpha_property_ = new rviz::FloatProperty ("Alpha", 1.0f, "", this,
                                             SLOT( changedAlpha() ), this);
  
  point_size_property_ = new rviz::FloatProperty ("Point Size", 0.01f, "", this,
                                                  SLOT( changedPointSize() ), this);
  
  topic_property_ = new rviz::RosTopicProperty("Topic", "", ros::message_traits::datatype<arm_navigation_msgs::CollisionMap>(), "", this,
                                               SLOT(changedTopic()), this);
}

void CollisionMapDisplay::onInitialize() 
{
  tf_filter_ = new tf::MessageFilter<arm_navigation_msgs::CollisionMap>(*context_->getTFClient(), "", 2, update_nh_);

  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  static int count = 0;
  std::stringstream ss;
  ss << "Collision Map" << count++;

  cloud_ = new rviz::PointCloud();
  alpha_ = 1.0f;
  cloud_->setAlpha(alpha_);
  scene_node_->attachObject(cloud_);

  tf_filter_->connectInput(sub_);
  tf_filter_->registerCallback(boost::bind(&CollisionMapDisplay::incomingMessage, this, _1));
}

CollisionMapDisplay::~CollisionMapDisplay()
{
  unsubscribe();
  clear();

  delete tf_filter_;
  delete cloud_;
}

void CollisionMapDisplay::clear()
{
  cloud_->clear();
}

void CollisionMapDisplay::changedTopic()
{
  unsubscribe();
  topic_ = topic_property_->getStdString();
  subscribe();
}

void CollisionMapDisplay::changedColor(void)
{
  color_ = rviz::Color(color_property_->getColor().redF(),
                       color_property_->getColor().greenF(),
                       color_property_->getColor().blueF());
  
  processMessage(current_message_);
}

void CollisionMapDisplay::changedOverrideColor(void)
{
  override_color_ = override_color_property_->getBool();
  
  processMessage(current_message_);
}

void CollisionMapDisplay::changedRenderOperation(void)
{
  render_operation_ = render_operation_property_->getOptionInt();
  
  if(render_operation_ == collision_render_ops::CPoints) {
    cloud_->setRenderMode(rviz::PointCloud::RM_SPHERES);
  } else {
    cloud_->setRenderMode(rviz::PointCloud::RM_BOXES);
  }

  processMessage(current_message_);
}

void CollisionMapDisplay::changedPointSize(void)
{
  point_size_ = point_size_property_->getFloat();
  cloud_->setDimensions(point_size_, point_size_, point_size_);
}

void CollisionMapDisplay::changedAlpha()
{
  alpha_ = alpha_property_->getFloat();
  cloud_->setAlpha(alpha_);
  processMessage(current_message_);
}

void CollisionMapDisplay::subscribe()
{
  if (!isEnabled())
    return;

  if (!topic_.empty())
  {
    sub_.subscribe(update_nh_, topic_, 1);
  }
}

void CollisionMapDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void CollisionMapDisplay::onEnable()
{
  scene_node_->setVisible(true);
  subscribe();
}

void CollisionMapDisplay::onDisable()
{
  unsubscribe();
  clear();
  scene_node_->setVisible(false);
}

void CollisionMapDisplay::fixedFrameChanged()
{
  tf_filter_->setTargetFrame(fixed_frame_.toStdString());
  clear();
}

void CollisionMapDisplay::update(float wall_dt, float ros_dt)
{
}

void CollisionMapDisplay::processMessage(const arm_navigation_msgs::CollisionMap::ConstPtr& msg)
{
  clear();

  if (!msg)
  {
    return;
  }

  if(msg->boxes.size() == 0) return;

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), fixed_frame_.toStdString().c_str() );
  }

  scene_node_->setPosition( position );
  scene_node_->setOrientation( orientation );

  Ogre::ColourValue color;

  unsigned int num_boxes = msg->boxes.size();
  ROS_DEBUG("Collision map contains %d boxes.", num_boxes); 

  typedef std::vector<rviz::PointCloud::Point> V_Point;
  V_Point points;
  points.resize(num_boxes);
  //use first box extents

  cloud_->setDimensions(msg->boxes[0].extents.x,
                        msg->boxes[0].extents.y,
                        msg->boxes[0].extents.z);
  for (uint32_t i = 0; i < num_boxes; i++)
  {
    rviz::PointCloud::Point & current_point = points[i];
    
    current_point.position.x = msg->boxes[i].center.x;
    current_point.position.y = msg->boxes[i].center.y;
    current_point.position.z = msg->boxes[i].center.z;
    current_point.color = Ogre::ColourValue(color_.r_, color_.g_, color_.b_, alpha_);
  }
  cloud_->clear();
  
  if (!points.empty())
  {
    cloud_->addPoints(&points.front(), points.size());
  }
}

void CollisionMapDisplay::incomingMessage(const arm_navigation_msgs::CollisionMap::ConstPtr& message)
{
  processMessage(message);
}

void CollisionMapDisplay::reset()
{
  clear();
}


} // namespace mapping_rviz_plugin
