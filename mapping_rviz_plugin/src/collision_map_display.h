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
 * $Id$
 *
 */

#ifndef RVIZ_COLLISION_MAP_DISPLAY_H_
#define RVIZ_COLLISION_MAP_DISPLAY_H_

#include "rviz/display.h"
#include "rviz/helpers/color.h"

#include <boost/thread/mutex.hpp>

#include <boost/shared_ptr.hpp>

#include <arm_navigation_msgs/OrientedBoundingBox.h>
#include <arm_navigation_msgs/CollisionMap.h>

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

namespace rviz
{
class PointCloud;
class ColorProperty;
class RosTopicProperty;
class BoolProperty;
class EnumProperty;
class FloatProperty;
}

namespace Ogre
{
class SceneNode;
class ManualObject;
}

namespace mapping_rviz_plugin
{

namespace collision_render_ops
{
enum CollisionRenderOp
{
  CBoxes, CPoints, CCount,
};
}
typedef collision_render_ops::CollisionRenderOp CollisionRenderOp;

/**
 * \class CollisionMapDisplay
 * \brief Displays a collision_map::CollisionMap message
 */
class CollisionMapDisplay : public rviz::Display
{
public:
  CollisionMapDisplay();

  virtual ~CollisionMapDisplay();

  virtual void onInitialize();

  void changedTopic();

  void changedColor();
  void changedOverrideColor();

  void changedRenderOperation();
  void changedPointSize();
  void changedAlpha();


  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();

protected:
  void subscribe();
  void unsubscribe();
  void clear();
  void incomingMessage(const arm_navigation_msgs::CollisionMap::ConstPtr& message);
  void processMessage(const arm_navigation_msgs::CollisionMap::ConstPtr& message);

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();
  virtual void fixedFrameChanged();

  std::string topic_;
  rviz::Color color_;
  int render_operation_;
  bool override_color_;
  float point_size_;
  float alpha_;

  Ogre::SceneNode* scene_node_;
  rviz::PointCloud* cloud_;

  arm_navigation_msgs::CollisionMap::ConstPtr current_message_;
  message_filters::Subscriber<arm_navigation_msgs::CollisionMap> sub_;
  tf::MessageFilter<arm_navigation_msgs::CollisionMap>* tf_filter_;

  rviz::ColorProperty* color_property_;
  rviz::RosTopicProperty* topic_property_;
  rviz::BoolProperty* override_color_property_;
  rviz::EnumProperty* render_operation_property_;
  rviz::FloatProperty* point_size_property_;
  rviz::FloatProperty* alpha_property_;
};

} // namespace mapping_rviz_plugin

#endif /* RVIZ_COLLISION_MAP_DISPLAY_H_ */
