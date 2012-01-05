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

/** \author Ioan Sucan, E. Gil Jones */
#include <boost/bind.hpp>
#include <climits>
#include <sstream>

#include <planning_environment/monitors/collision_space_monitor.h>
#include <planning_environment/monitors/monitor_utils.h>

namespace planning_environment
{
    
static inline double maxCoord(const geometry_msgs::Point32 &point)
{
  return std::max(std::max(point.x, point.y), point.z);
}
}

void planning_environment::CollisionSpaceMonitor::setupCSM(void)
{
  envMonitorStarted_ = false;

  collisionMapFilter_ = NULL;
  collisionMapUpdateFilter_ = NULL;
  collisionObjectFilter_ = NULL;
    
  collisionMapSubscriber_ = NULL;
  collisionMapUpdateSubscriber_ = NULL;
  collisionObjectSubscriber_ = NULL;
    
  have_map_ = false;
  use_collision_map_ = false;

  nh_.param<double>("pointcloud_padd", pointcloud_padd_, 0.00);
}

void planning_environment::CollisionSpaceMonitor::startEnvironmentMonitor(void)
{
  if (envMonitorStarted_)
    return;

  if(use_collision_map_) {
    collisionMapSubscriber_ = new message_filters::Subscriber<arm_navigation_msgs::CollisionMap>(root_handle_, "collision_map_occ", 1);
    collisionMapFilter_ = new tf::MessageFilter<arm_navigation_msgs::CollisionMap>(*collisionMapSubscriber_, *tf_, cm_->getWorldFrameId(), 1);
    collisionMapFilter_->registerCallback(boost::bind(&CollisionSpaceMonitor::collisionMapCallback, this, _1));
    ROS_INFO("Listening to collision_map using message notifier with target frame %s", collisionMapFilter_->getTargetFramesString().c_str());
    
    collisionMapUpdateSubscriber_ = new message_filters::Subscriber<arm_navigation_msgs::CollisionMap>(root_handle_, "collision_map_update", 1024);
    collisionMapUpdateFilter_ = new tf::MessageFilter<arm_navigation_msgs::CollisionMap>(*collisionMapUpdateSubscriber_, *tf_, cm_->getWorldFrameId(), 1);
    collisionMapUpdateFilter_->registerCallback(boost::bind(&CollisionSpaceMonitor::collisionMapUpdateCallback, this, _1));
    ROS_DEBUG("Listening to collision_map_update using message notifier with target frame %s", collisionMapUpdateFilter_->getTargetFramesString().c_str());
  }

  collisionObjectSubscriber_ = new message_filters::Subscriber<arm_navigation_msgs::CollisionObject>(root_handle_, "collision_object", 1024);
  collisionObjectFilter_ = new tf::MessageFilter<arm_navigation_msgs::CollisionObject>(*collisionObjectSubscriber_, *tf_, cm_->getWorldFrameId(), 1024);
  collisionObjectFilter_->registerCallback(boost::bind(&CollisionSpaceMonitor::collisionObjectCallback, this, _1));
  ROS_DEBUG("Listening to object_in_map using message notifier with target frame %s", collisionObjectFilter_->getTargetFramesString().c_str());
  
  //using regular message filter as there's no header
  attachedCollisionObjectSubscriber_ = new message_filters::Subscriber<arm_navigation_msgs::AttachedCollisionObject>(root_handle_, "attached_collision_object", 1024);	
  attachedCollisionObjectSubscriber_->registerCallback(boost::bind(&CollisionSpaceMonitor::attachObjectCallback, this, _1));    

  envMonitorStarted_ = true;
}

void planning_environment::CollisionSpaceMonitor::setUseCollisionMap(bool use_collision_map) {
  if(use_collision_map_ == use_collision_map) return;
  
  use_collision_map_ = use_collision_map;

  if(!envMonitorStarted_) return;

  if(use_collision_map_) {
    collisionMapSubscriber_ = new message_filters::Subscriber<arm_navigation_msgs::CollisionMap>(root_handle_, "collision_map", 1);
    collisionMapFilter_ = new tf::MessageFilter<arm_navigation_msgs::CollisionMap>(*collisionMapSubscriber_, *tf_, cm_->getWorldFrameId(), 1);
    collisionMapFilter_->registerCallback(boost::bind(&CollisionSpaceMonitor::collisionMapCallback, this, _1));
    ROS_DEBUG("Listening to collision_map using message notifier with target frame %s", collisionMapFilter_->getTargetFramesString().c_str());
    
    collisionMapUpdateSubscriber_ = new message_filters::Subscriber<arm_navigation_msgs::CollisionMap>(root_handle_, "collision_map_update", 1);
    collisionMapUpdateFilter_ = new tf::MessageFilter<arm_navigation_msgs::CollisionMap>(*collisionMapUpdateSubscriber_, *tf_, cm_->getWorldFrameId(), 1);
    collisionMapUpdateFilter_->registerCallback(boost::bind(&CollisionSpaceMonitor::collisionMapUpdateCallback, this, _1));
    ROS_DEBUG("Listening to collision_map_update using message notifier with target frame %s", collisionMapUpdateFilter_->getTargetFramesString().c_str());
  } else {
    if(collisionMapUpdateFilter_) {
      delete collisionMapUpdateFilter_;
      collisionMapUpdateFilter_ = NULL;
    }
    if(collisionMapUpdateSubscriber_) {
      delete collisionMapUpdateSubscriber_;
      collisionMapUpdateSubscriber_ = NULL;
    }
    if(collisionMapFilter_) {
      delete collisionMapFilter_;
      collisionMapFilter_ = NULL;
    }
    
    if(collisionMapSubscriber_) {
      delete collisionMapSubscriber_;
      collisionMapSubscriber_ = NULL;
    }
  }
}
  
void planning_environment::CollisionSpaceMonitor::stopEnvironmentMonitor(void)
{
  if (!envMonitorStarted_)
    return;
    
  if(collisionMapUpdateFilter_) {
    delete collisionMapUpdateFilter_;
    collisionMapUpdateFilter_ = NULL;
  }
  if(collisionMapUpdateSubscriber_) {
    delete collisionMapUpdateSubscriber_;
    collisionMapUpdateSubscriber_ = NULL;
  }
  if(collisionMapFilter_) {
    delete collisionMapFilter_;
    collisionMapFilter_ = NULL;
  }
  
  if(collisionMapSubscriber_) {
    delete collisionMapSubscriber_;
    collisionMapSubscriber_ = NULL;
  }
   
  if(collisionObjectFilter_) {
    delete collisionObjectFilter_;
    collisionObjectFilter_ = NULL;
  }

  if(collisionObjectSubscriber_) {
    delete collisionObjectSubscriber_;
    collisionObjectSubscriber_ = NULL;
  }
  
  if(attachedCollisionObjectSubscriber_) {
    delete attachedCollisionObjectSubscriber_;
    attachedCollisionObjectSubscriber_ = NULL;
  }

  ROS_DEBUG("Environment state is no longer being monitored");

  envMonitorStarted_ = false;
}

bool planning_environment::CollisionSpaceMonitor::isMapUpdated(double sec) const
{
  if(sec < 1e-5) 
  {
    return false;
  }

  //2. it hasn't yet been a full second interval but we've updated
  if(last_map_update_ > ros::TIME_MIN && ros::Time::now() < ros::Time(sec)) 
  {
    return true;
  }

  //3. Been longer than sec interval, so we check that the update has happened in the indicated interval
  if (last_map_update_ < ros::Time::now()-ros::Duration(sec)) 
  {
    return false;
  }

  return true;
}

void planning_environment::CollisionSpaceMonitor::waitForMap(void) const
{
  if(!use_collision_map_) {
    ROS_INFO("Not subscribing to map so not waiting");
    return;
  }
  int s = 0;
  while (nh_.ok() && !haveMap())
  {
    if (s == 0)
      ROS_INFO("Waiting for map ...");
    s = (s + 1) % 40;
    ros::spinOnce();
    ros::Duration().fromSec(0.05).sleep();
  }
  if (haveMap())
    ROS_INFO("Map received!");
}

void planning_environment::CollisionSpaceMonitor::collisionMapUpdateCallback(const arm_navigation_msgs::CollisionMapConstPtr &collisionMap)
{
  if (collisionMap->boxes.size() > 0)
    updateCollisionSpace(collisionMap, false);
}

void planning_environment::CollisionSpaceMonitor::collisionMapCallback(const arm_navigation_msgs::CollisionMapConstPtr &collisionMap)
{
  updateCollisionSpace(collisionMap, true);
}

void planning_environment::CollisionSpaceMonitor::collisionMapAsSpheres(const arm_navigation_msgs::CollisionMapConstPtr &collisionMap,
                                                                        std::vector<shapes::Shape*> &spheres, std::vector<tf::Transform> &poses)
{
  // we want to make sure the frame the robot model is kept in is the same as the frame of the collisionMap
  bool transform = collisionMap->header.frame_id != cm_->getWorldFrameId();
  const int n = collisionMap->boxes.size();
    
  spheres.resize(n);
  poses.resize(n);
    
  if (transform)
  {
    std::string target = cm_->getWorldFrameId();
    bool err = false;
	
    //#pragma omp parallel for
    for (int i = 0 ; i < n ; ++i)
    {
      geometry_msgs::PointStamped psi;
      psi.header  = collisionMap->header;
      psi.point.x = collisionMap->boxes[i].center.x;
      psi.point.y = collisionMap->boxes[i].center.y;
      psi.point.z = collisionMap->boxes[i].center.z;
	    
      geometry_msgs::PointStamped pso;
      try
      {
        tf_->transformPoint(target, psi, pso);
      }
      catch(...)
      {
        err = true;
        pso = psi;
      }
	    
      poses[i].setIdentity();
      poses[i].setOrigin(tf::Vector3(pso.point.x, pso.point.y, pso.point.z));
      spheres[i] = new shapes::Sphere(maxCoord(collisionMap->boxes[i].extents) * 0.867 + pointcloud_padd_);
    }
	
    if (err)
      ROS_ERROR("Some errors encountered in transforming the collision map to frame '%s' from frame '%s'", target.c_str(), collisionMap->header.frame_id.c_str());
  }
  else
  {

    //#pragma omp parallel for
    for (int i = 0 ; i < n ; ++i)
    {
      poses[i].setIdentity();
      poses[i].setOrigin(tf::Vector3(collisionMap->boxes[i].center.x, collisionMap->boxes[i].center.y, collisionMap->boxes[i].center.z));
      spheres[i] = new shapes::Sphere(maxCoord(collisionMap->boxes[i].extents) * 0.867 + pointcloud_padd_);
    }
  }
}

void planning_environment::CollisionSpaceMonitor::collisionMapAsBoxes(const arm_navigation_msgs::CollisionMapConstPtr &collision_map,
                                                                      std::vector<shapes::Shape*> &boxes, std::vector<tf::Transform> &poses)
{
  collisionMapAsBoxes(*collision_map, boxes, poses);
}

void planning_environment::CollisionSpaceMonitor::collisionMapAsBoxes(const arm_navigation_msgs::CollisionMap& collision_map,
                                                                      std::vector<shapes::Shape*> &boxes, std::vector<tf::Transform> &poses)
{

  // we want to make sure the frame the robot model is kept in is the same as the frame of the collision_map
  bool transform = collision_map.header.frame_id != cm_->getWorldFrameId();
  const int n = collision_map.boxes.size();
    
  double pd = 2.0 * pointcloud_padd_;
    
  boxes.resize(n);
  poses.resize(n);
    
  if (transform)
  {
    std::string target = cm_->getWorldFrameId();
    bool err = false;
	
    //#pragma omp parallel for
    for (int i = 0 ; i < n ; ++i)
    {
      geometry_msgs::PointStamped psi;
      psi.header  = collision_map.header;
      psi.point.x = collision_map.boxes[i].center.x;
      psi.point.y = collision_map.boxes[i].center.y;
      psi.point.z = collision_map.boxes[i].center.z;
	    
      geometry_msgs::PointStamped pso;
      try
      {
        tf_->transformPoint(target, psi, pso);
      }
      catch(...)
      {
        err = true;
        pso = psi;
      }
	    
      poses[i].setRotation(tf::Quaternion(tf::Vector3(collision_map.boxes[i].axis.x, collision_map.boxes[i].axis.y, collision_map.boxes[i].axis.z), collision_map.boxes[i].angle));
      poses[i].setOrigin(tf::Vector3(pso.point.x, pso.point.y, pso.point.z));
      boxes[i] = new shapes::Box(collision_map.boxes[i].extents.x + pd, collision_map.boxes[i].extents.y + pd, collision_map.boxes[i].extents.z + pd);
    }
	
    if (err)
      ROS_ERROR("Some errors encountered in transforming the collision map to frame '%s' from frame '%s'", target.c_str(), collision_map.header.frame_id.c_str());
  }
  else
  {

    //#pragma omp parallel for
    for (int i = 0 ; i < n ; ++i)
    {
      poses[i].setRotation(tf::Quaternion(tf::Vector3(collision_map.boxes[i].axis.x, collision_map.boxes[i].axis.y, collision_map.boxes[i].axis.z), collision_map.boxes[i].angle));
      poses[i].setOrigin(tf::Vector3(collision_map.boxes[i].center.x, collision_map.boxes[i].center.y, collision_map.boxes[i].center.z));
      boxes[i] = new shapes::Box(collision_map.boxes[i].extents.x + pd, collision_map.boxes[i].extents.y + pd, collision_map.boxes[i].extents.z + pd);
    }
  }
}

void planning_environment::CollisionSpaceMonitor::updateCollisionSpace(const arm_navigation_msgs::CollisionMapConstPtr &collision_map, bool clear)
{ 
  std::vector<shapes::Shape*> shapes;
  std::vector<tf::Transform> poses;
  
  collisionMapAsBoxes(*collision_map, shapes, poses);
  //not masking here
  cm_->setCollisionMap(shapes, poses, false);
  last_map_update_ = collision_map->header.stamp;
  have_map_ = true;
}


void planning_environment::CollisionSpaceMonitor::collisionObjectCallback(const arm_navigation_msgs::CollisionObjectConstPtr &collision_object)
{
  processCollisionObjectMsg(collision_object, *tf_, cm_);
}

bool planning_environment::CollisionSpaceMonitor::attachObjectCallback(const arm_navigation_msgs::AttachedCollisionObjectConstPtr &attached_object)
{
  return processAttachedCollisionObjectMsg(attached_object, *tf_, cm_);
}


