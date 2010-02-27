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

/** \author Mrinal Kalakrishnan */

#include <chomp_motion_planner/chomp_collision_space.h>
#include <sstream>

namespace chomp
{

ChompCollisionSpace::ChompCollisionSpace():
  distance_field_(NULL),node_handle_("~"),collision_map_subscriber_(node_handle_,"collision_map",1)
{
}

ChompCollisionSpace::~ChompCollisionSpace()
{
  if (distance_field_)
    delete distance_field_;
  delete collision_map_filter_;
}

void ChompCollisionSpace::collisionMapCallback(const mapping_msgs::CollisionMapConstPtr& collision_map)
{
  // @TODO transform the collision map to the required frame!!
  if (mutex_.try_lock())
  {
    ros::WallTime start = ros::WallTime::now();
    distance_field_->reset();
    ROS_INFO("Reset prop distance_field in %f sec", (ros::WallTime::now() - start).toSec());
    start = ros::WallTime::now();
    distance_field_->addPointsToField(cuboid_points_);
    distance_field_->addCollisionMapToField(*collision_map);
    mutex_.unlock();
    ROS_INFO("Updated prop distance_field in %f sec", (ros::WallTime::now() - start).toSec());

    distance_field_->visualize(0.895*max_expansion_, 0.9*max_expansion_, collision_map->header.frame_id, collision_map->header.stamp);

  }
  else
  {
    ROS_INFO("Skipped collision map update because planning is in progress.");
  }
}

bool ChompCollisionSpace::init(double max_radius_clearance)
{
  double size_x, size_y, size_z;
  double origin_x, origin_y, origin_z;
  double resolution;

  node_handle_.param("reference_frame", reference_frame_, std::string("base_link"));
  node_handle_.param("collision_space/size_x", size_x, 2.0);
  node_handle_.param("collision_space/size_y", size_y, 3.0);
  node_handle_.param("collision_space/size_z", size_z, 4.0);
  node_handle_.param("collision_space/origin_x", origin_x, 0.1);
  node_handle_.param("collision_space/origin_y", origin_y, -1.5);
  node_handle_.param("collision_space/origin_z", origin_z, -2.0);
  node_handle_.param("collision_space/resolution", resolution, 0.02);
  node_handle_.param("collision_space/field_bias_x", field_bias_x_, 0.0);
  node_handle_.param("collision_space/field_bias_y", field_bias_y_, 0.0);
  node_handle_.param("collision_space/field_bias_z", field_bias_z_, 0.0);
  resolution_ = resolution;
  max_expansion_ = max_radius_clearance;

  initCollisionCuboids();

  distance_field_ = new distance_field::PropagationDistanceField(size_x, size_y, size_z, resolution, origin_x, origin_y, origin_z, max_expansion_);

  collision_map_filter_ = new tf::MessageFilter<mapping_msgs::CollisionMap>(collision_map_subscriber_,tf_,reference_frame_,1);
  collision_map_filter_->registerCallback(boost::bind(&ChompCollisionSpace::collisionMapCallback, this, _1));

  ROS_INFO("Initialized chomp collision space in %s reference frame with %f expansion radius.", reference_frame_.c_str(), max_expansion_);
  return true;
}

static std::string intToString(int i)
{
  std::ostringstream oss;
  oss << i;
  return oss.str();
}

void ChompCollisionSpace::initCollisionCuboids()
{
  int index=1;
  while (node_handle_.hasParam(std::string("collision_space/cuboids/cuboid")+intToString(index)+"/size_x"))
  {
    addCollisionCuboid(std::string("collision_space/cuboids/cuboid")+intToString(index));
    index++;
  }

}

void ChompCollisionSpace::addCollisionCuboid(const std::string param_name)
{
  double size_x, size_y, size_z;
  double origin_x, origin_y, origin_z;
  if (!node_handle_.getParam(param_name+"/size_x", size_x))
    return;
  if (!node_handle_.getParam(param_name+"/size_y", size_y))
    return;
  if (!node_handle_.getParam(param_name+"/size_z", size_z))
    return;
  if (!node_handle_.getParam(param_name+"/origin_x", origin_x))
    return;
  if (!node_handle_.getParam(param_name+"/origin_y", origin_y))
    return;
  if (!node_handle_.getParam(param_name+"/origin_z", origin_z))
    return;

  if (size_x<0 || size_y<0 || size_z<0)
    return;

  // add points:
  int num_points=0;
  for (double x=origin_x; x<=origin_x+size_x; x+=resolution_)
    for (double y=origin_y; y<=origin_y+size_y; y+=resolution_)
      for (double z=origin_z; z<=origin_z+size_z; z+=resolution_)
      {
        cuboid_points_.push_back(btVector3(x,y,z));
        ++num_points;
      }
  ROS_INFO("Added %d points for collision cuboid %s", num_points, param_name.c_str());
}

}
