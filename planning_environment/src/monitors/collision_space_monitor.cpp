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

#include "planning_environment/monitors/collision_space_monitor.h"
#include "planning_environment/util/construct_object.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometric_shapes/shape_operations.h>
#include <boost/bind.hpp>
#include <climits>
#include <sstream>

static const std::string GET_COLLISION_OBJECTS_SERVICE_NAME = "get_collision_objects";
static const std::string GET_CURRENT_ALLOWED_COLLISION_MATRIX_SERVICE_NAME = "get_current_allowed_collision_matrix";
static const std::string SET_ALLOWED_COLLISIONS_SERVICE_NAME="set_allowed_collisions";
static const std::string REVERT_ALLOWED_COLLISIONS_SERVICE_NAME="revert_allowed_collisions";

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
  onBeforeMapUpdate_ = NULL;
  onAfterMapUpdate_  = NULL;
  onCollisionObjectUpdate_ = NULL;
  onAfterAttachCollisionObject_ = NULL;

  collisionMapFilter_ = NULL;
  collisionMapUpdateFilter_ = NULL;
  collisionObjectFilter_ = NULL;
    
  collisionMapSubscriber_ = NULL;
  collisionMapUpdateSubscriber_ = NULL;
  collisionObjectSubscriber_ = NULL;
    
  haveMap_ = false;
  use_collision_map_ = false;

  collisionSpace_ = cm_->getODECollisionModel().get();
  collisionSpace_->clearObjects("points");
  nh_.param<double>("pointcloud_padd", pointcloud_padd_, 0.02);
  nh_.param<double>("object_scale", scale_, 1.0);
  nh_.param<double>("object_padd", padd_, 0.02);

}

void planning_environment::CollisionSpaceMonitor::startEnvironmentMonitor(void)
{
  if (envMonitorStarted_)
    return;

  if(use_collision_map_) {
    collisionMapSubscriber_ = new message_filters::Subscriber<mapping_msgs::CollisionMap>(root_handle_, "collision_map", 1);
    collisionMapFilter_ = new tf::MessageFilter<mapping_msgs::CollisionMap>(*collisionMapSubscriber_, *tf_, getWorldFrameId(), 1);
    collisionMapFilter_->registerCallback(boost::bind(&CollisionSpaceMonitor::collisionMapCallback, this, _1));
    ROS_DEBUG("Listening to collision_map using message notifier with target frame %s", collisionMapFilter_->getTargetFramesString().c_str());
    
    collisionMapUpdateSubscriber_ = new message_filters::Subscriber<mapping_msgs::CollisionMap>(root_handle_, "collision_map_update", 1024);
    collisionMapUpdateFilter_ = new tf::MessageFilter<mapping_msgs::CollisionMap>(*collisionMapUpdateSubscriber_, *tf_, getWorldFrameId(), 1);
    collisionMapUpdateFilter_->registerCallback(boost::bind(&CollisionSpaceMonitor::collisionMapUpdateCallback, this, _1));
    ROS_DEBUG("Listening to collision_map_update using message notifier with target frame %s", collisionMapUpdateFilter_->getTargetFramesString().c_str());
  }

  collisionObjectSubscriber_ = new message_filters::Subscriber<mapping_msgs::CollisionObject>(root_handle_, "collision_object", 1024);
  collisionObjectFilter_ = new tf::MessageFilter<mapping_msgs::CollisionObject>(*collisionObjectSubscriber_, *tf_, getWorldFrameId(), 1024);
  collisionObjectFilter_->registerCallback(boost::bind(&CollisionSpaceMonitor::collisionObjectCallback, this, _1));
  ROS_DEBUG("Listening to object_in_map using message notifier with target frame %s", collisionMapUpdateFilter_->getTargetFramesString().c_str());
  
  //using regular message filter as there's no header
  attachedCollisionObjectSubscriber_ = new message_filters::Subscriber<mapping_msgs::AttachedCollisionObject>(root_handle_, "attached_collision_object", 1024);	
  attachedCollisionObjectSubscriber_->registerCallback(boost::bind(&CollisionSpaceMonitor::attachObjectCallback, this, _1));    

  advertiseServices();

  envMonitorStarted_ = true;
}

void planning_environment::CollisionSpaceMonitor::advertiseServices() {
  KinematicModelStateMonitor::advertiseServices();
  get_objects_service_ = ros::NodeHandle("~").advertiseService(GET_COLLISION_OBJECTS_SERVICE_NAME, &CollisionSpaceMonitor::getObjectsService, this);
  get_current_collision_map_service_ = ros::NodeHandle("~").advertiseService(GET_CURRENT_ALLOWED_COLLISION_MATRIX_SERVICE_NAME, &CollisionSpaceMonitor::getCurrentAllowedCollisionsService, this);
  set_allowed_collisions_service_ = ros::NodeHandle("~").advertiseService(SET_ALLOWED_COLLISIONS_SERVICE_NAME, &CollisionSpaceMonitor::setAllowedCollisionsService, this);
  revert_allowed_collisions_service_ = ros::NodeHandle("~").advertiseService(REVERT_ALLOWED_COLLISIONS_SERVICE_NAME, &CollisionSpaceMonitor::revertAllowedCollisionMatrixToDefaultService, this);  
}

void planning_environment::CollisionSpaceMonitor::setUseCollisionMap(bool use_collision_map) {
  if(use_collision_map_ == use_collision_map) return;
  
  use_collision_map_ = use_collision_map;

  if(!envMonitorStarted_) return;

  if(use_collision_map_) {
    collisionMapSubscriber_ = new message_filters::Subscriber<mapping_msgs::CollisionMap>(root_handle_, "collision_map", 1);
    collisionMapFilter_ = new tf::MessageFilter<mapping_msgs::CollisionMap>(*collisionMapSubscriber_, *tf_, getWorldFrameId(), 1);
    collisionMapFilter_->registerCallback(boost::bind(&CollisionSpaceMonitor::collisionMapCallback, this, _1));
    ROS_DEBUG("Listening to collision_map using message notifier with target frame %s", collisionMapFilter_->getTargetFramesString().c_str());
    
    collisionMapUpdateSubscriber_ = new message_filters::Subscriber<mapping_msgs::CollisionMap>(root_handle_, "collision_map_update", 1);
    collisionMapUpdateFilter_ = new tf::MessageFilter<mapping_msgs::CollisionMap>(*collisionMapUpdateSubscriber_, *tf_, getWorldFrameId(), 1);
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

  for (std::map<std::string, KnownObject*>::iterator it = collisionObjects_.begin() ; it != collisionObjects_.end() ; ++it)
    delete it->second;

  for(unsigned int i = 0; i < last_collision_map_shapes_.size(); i++) {
    delete last_collision_map_shapes_[i];
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
  if(lastMapUpdate_ > ros::TIME_MIN && ros::Time::now() < ros::Time(sec)) 
  {
    return true;
  }

  //3. Been longer than sec interval, so we check that the update has happened in the indicated interval
  if (lastMapUpdate_ < ros::Time::now()-ros::Duration(sec)) 
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

void planning_environment::CollisionSpaceMonitor::collisionMapUpdateCallback(const mapping_msgs::CollisionMapConstPtr &collisionMap)
{
  if (collisionMap->boxes.size() > 0)
    updateCollisionSpace(collisionMap, false);
}

void planning_environment::CollisionSpaceMonitor::collisionMapCallback(const mapping_msgs::CollisionMapConstPtr &collisionMap)
{
  updateCollisionSpace(collisionMap, true);
}

void planning_environment::CollisionSpaceMonitor::collisionMapAsSpheres(const mapping_msgs::CollisionMapConstPtr &collisionMap,
                                                                        std::vector<shapes::Shape*> &spheres, std::vector<btTransform> &poses)
{
  // we want to make sure the frame the robot model is kept in is the same as the frame of the collisionMap
  bool transform = collisionMap->header.frame_id != getWorldFrameId();
  const int n = collisionMap->boxes.size();
    
  spheres.resize(n);
  poses.resize(n);
    
  if (transform)
  {
    std::string target = getWorldFrameId();
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
      poses[i].setOrigin(btVector3(pso.point.x, pso.point.y, pso.point.z));
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
      poses[i].setOrigin(btVector3(collisionMap->boxes[i].center.x, collisionMap->boxes[i].center.y, collisionMap->boxes[i].center.z));
      spheres[i] = new shapes::Sphere(maxCoord(collisionMap->boxes[i].extents) * 0.867 + pointcloud_padd_);
    }
  }
}

void planning_environment::CollisionSpaceMonitor::collisionMapAsBoxes(const mapping_msgs::CollisionMapConstPtr &collisionMap,
                                                                      std::vector<shapes::Shape*> &boxes, std::vector<btTransform> &poses)
{
  collisionMapAsBoxes(*collisionMap, boxes, poses);
}

void planning_environment::CollisionSpaceMonitor::collisionMapAsBoxes(const mapping_msgs::CollisionMap& collisionMap,
                                                                      std::vector<shapes::Shape*> &boxes, std::vector<btTransform> &poses)
{

  // we want to make sure the frame the robot model is kept in is the same as the frame of the collisionMap
  bool transform = collisionMap.header.frame_id != getWorldFrameId();
  const int n = collisionMap.boxes.size();
    
  double pd = 2.0 * pointcloud_padd_;
    
  boxes.resize(n);
  poses.resize(n);
    
  if (transform)
  {
    std::string target = getWorldFrameId();
    bool err = false;
	
    //#pragma omp parallel for
    for (int i = 0 ; i < n ; ++i)
    {
      geometry_msgs::PointStamped psi;
      psi.header  = collisionMap.header;
      psi.point.x = collisionMap.boxes[i].center.x;
      psi.point.y = collisionMap.boxes[i].center.y;
      psi.point.z = collisionMap.boxes[i].center.z;
	    
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
	    
      poses[i].setRotation(btQuaternion(btVector3(collisionMap.boxes[i].axis.x, collisionMap.boxes[i].axis.y, collisionMap.boxes[i].axis.z), collisionMap.boxes[i].angle));
      poses[i].setOrigin(btVector3(pso.point.x, pso.point.y, pso.point.z));
      boxes[i] = new shapes::Box(collisionMap.boxes[i].extents.x + pd, collisionMap.boxes[i].extents.y + pd, collisionMap.boxes[i].extents.z + pd);
    }
	
    if (err)
      ROS_ERROR("Some errors encountered in transforming the collision map to frame '%s' from frame '%s'", target.c_str(), collisionMap.header.frame_id.c_str());
  }
  else
  {

    //#pragma omp parallel for
    for (int i = 0 ; i < n ; ++i)
    {
      poses[i].setRotation(btQuaternion(btVector3(collisionMap.boxes[i].axis.x, collisionMap.boxes[i].axis.y, collisionMap.boxes[i].axis.z), collisionMap.boxes[i].angle));
      poses[i].setOrigin(btVector3(collisionMap.boxes[i].center.x, collisionMap.boxes[i].center.y, collisionMap.boxes[i].center.z));
      boxes[i] = new shapes::Box(collisionMap.boxes[i].extents.x + pd, collisionMap.boxes[i].extents.y + pd, collisionMap.boxes[i].extents.z + pd);
    }
  }
}

void planning_environment::CollisionSpaceMonitor::updateStaticObjectBodies(const mapping_msgs::CollisionObjectConstPtr &collisionObject) {
  collision_objects_lock_.lock();
  if (collisionObject->operation.operation == mapping_msgs::CollisionObjectOperation::ADD) {
    KnownObject* kb = new KnownObject();
    for(unsigned int i = 0; i < collisionObject->shapes.size(); i++) {
      // add the object to the map
      shapes::Shape *shape = planning_environment::constructObject(collisionObject->shapes[i]);
      if (shape) {
        bool err = false;
        geometry_msgs::PoseStamped psi;
        geometry_msgs::PoseStamped pso;
        psi.pose = collisionObject->poses[i];
        psi.header = collisionObject->header;
        try {
          tf_->transformPose(getWorldFrameId(), psi, pso);
        } catch(tf::TransformException& ex) {
          ROS_ERROR("Unable to transform object '%s' in frame '%s' to frame '%s' Exception: %s", collisionObject->id.c_str(), collisionObject->header.frame_id.c_str(), getWorldFrameId().c_str(), ex.what());          err = true;
        }
	
        if (!err) {
          btTransform pose;
          tf::poseMsgToTF(pso.pose, pose);
          bodies::Body* body = bodies::createBodyFromShape(shape);
          body->setScale(scale_);
          body->setPadding(padd_);
          kb->addBodyWithPose(body, pose);
        }
        delete shape;
      }
    }
    //if we already have this object
    if(collisionObjects_.find(collisionObject->id) != collisionObjects_.end()) {
      delete collisionObjects_[collisionObject->id];
      collisionObjects_.erase(collisionObject->id);
    }
    collisionObjects_[collisionObject->id] = kb;
    ROS_INFO("Added object '%s' with to list of known objects", collisionObject->id.c_str());
  } else {
    if(collisionObject->id == "all") {
      for(std::map<std::string, KnownObject*>::iterator it = collisionObjects_.begin();
          it != collisionObjects_.end();
          it++) {
        delete it->second;
      }
      collisionObjects_.clear();
    } else if (collisionObjects_.find(collisionObject->id) != collisionObjects_.end()) {
      delete collisionObjects_[collisionObject->id];
      collisionObjects_.erase(collisionObject->id);
    }  else
      ROS_WARN("Object '%s' is not in list of known objects", collisionObject->id.c_str());
  }
  collision_objects_lock_.unlock();
  setCollisionSpace();
}

void planning_environment::CollisionSpaceMonitor::maskCollisionMapForCollisionObjects(std::vector<shapes::Shape*>& shapes,
                                                                                      std::vector<btTransform>& poses,
                                                                                      std::vector<bool>& mask) 
{
  //no sensor pos for now
  btVector3 sensor_pos(0, 0, 0);

  if(shapes.size() != poses.size()) {
    ROS_WARN("Unequal number shapes and poses");
    return;
  }
  if(shapes.size() != mask.size()) {
    ROS_WARN("Unequal number shapes and mask");
    return;
  }

  std::vector<shapes::Shape*>::iterator shapeit = shapes.begin();
  std::vector<btTransform>::iterator poseit = poses.begin();
  std::vector<bool>::iterator maskit = mask.begin();

  collision_objects_lock_.lock();
  while(shapeit != shapes.end()) {
    btVector3 pt = (*poseit).getOrigin();
    btVector3 dir(sensor_pos - pt);
    dir.normalize();
    bool inside = false;
    for (std::map<std::string, KnownObject*>::iterator it = collisionObjects_.begin() ; !inside && it != collisionObjects_.end() ; ++it) {
      KnownObject* ko = it->second;
      for (unsigned int j = 0 ; !inside && j < ko->bodies.size(); ++j) {
        if(ko->bodies.size() != ko->bspheres.size() ||
           ko->bodies.size() != ko->rsquares.size()) {
          ROS_ERROR_STREAM("Bad sizes " << ko->bodies.size() << " " << ko->bspheres.size()
                           << " " << ko->rsquares.size());
          continue;
        }
        if (ko->bspheres[j].center.distance2(pt) < ko->rsquares[j]) {
          if(ko->bodies[j]->containsPoint(pt)) {
            inside = true;
          }
          //if (ko->bodies[j]->intersectsRay(pt, dir)) {
          //  inside = true;
          //}
        }
      }
    }
    *maskit = !inside;
    shapeit++;
    poseit++;
    maskit++;
  }
  collision_objects_lock_.unlock();
}

void planning_environment::CollisionSpaceMonitor::updateCollisionSpace(const mapping_msgs::CollisionMapConstPtr &collisionMap, bool clear)
{ 
  collision_map_lock_.lock();
  for(unsigned int i = 0; i < last_collision_map_shapes_.size(); i++) {
    delete last_collision_map_shapes_[i];
  }
  last_collision_map_shapes_.clear();
  last_collision_map_poses_.clear();
  last_collision_map_object_mask_.clear();

  collisionMapAsBoxes(*collisionMap, last_collision_map_shapes_, last_collision_map_poses_);
  last_collision_map_object_mask_.resize(last_collision_map_shapes_.size(), true);
  lastMapUpdate_ = collisionMap->header.stamp;
  collision_map_lock_.unlock();
  haveMap_ = true;
}

void planning_environment::CollisionSpaceMonitor::setCollisionSpace() {

  if(haveMap_) {
    getEnvironmentModel()->lock();
    collision_map_lock_.lock();


    ros::WallTime startTime = ros::WallTime::now();

    maskCollisionMapForCollisionObjects(last_collision_map_shapes_, last_collision_map_poses_, last_collision_map_object_mask_);

    std::vector<shapes::Shape*> unmasked_shapes;
    std::vector<btTransform> unmasked_poses;

    for(unsigned int i = 0; i < last_collision_map_shapes_.size(); i++) {
      if(last_collision_map_object_mask_[i]) {
        unmasked_shapes.push_back(shapes::cloneShape(last_collision_map_shapes_[i]));
        unmasked_poses.push_back(last_collision_map_poses_[i]);
      }
    }

    ROS_DEBUG_STREAM("Before filter we had " << last_collision_map_shapes_.size() << " points and now we have " << unmasked_shapes.size() << " points.");

    collision_map_lock_.unlock();
    
    getEnvironmentModel()->clearObjects("points");
    if(unmasked_shapes.size() == 0) {
      ROS_DEBUG("There don't appear to be any points to set in the collision map");
    }
    getEnvironmentModel()->addObjects("points", unmasked_shapes, unmasked_poses);
    double tupd = (ros::WallTime::now() - startTime).toSec();
    ROS_DEBUG("Updated map model in %f seconds", tupd); 
    getEnvironmentModel()->unlock();
  }   
}

void planning_environment::CollisionSpaceMonitor::collisionObjectCallback(const mapping_msgs::CollisionObjectConstPtr &collisionObject)
{
  if (collisionObject->operation.operation == mapping_msgs::CollisionObjectOperation::ADD)
  {
    std::vector<shapes::Shape*> shapes;
    std::vector<btTransform> poses;
    bool shapes_ok = true;
    for(unsigned int i = 0; i < collisionObject->shapes.size(); i++) {

      // add the object to the map
      shapes::Shape *shape = constructObject(collisionObject->shapes[i]);
      if (shape)
      {
        bool err = false;
        geometry_msgs::PoseStamped psi;
        geometry_msgs::PoseStamped pso;
        psi.pose = collisionObject->poses[i];
        
        psi.header = collisionObject->header;
        try
        {
          tf_->transformPose(getWorldFrameId(), psi, pso);
        }
        catch(...)
        {
          err = true;
        }
        
        if (err) {
          ROS_ERROR("Unable to transform object in ns %s num %d in frame %s to frame %s", collisionObject->id.c_str(), i, collisionObject->header.frame_id.c_str(), getWorldFrameId().c_str());
          shapes_ok = false;
        }
        else
        {
          btTransform pose;
          tf::poseMsgToTF(pso.pose, pose);
          ROS_DEBUG_STREAM("Object is at " << pose.getOrigin().x() << " " << pose.getOrigin().y());
          shapes.push_back(shape);
          poses.push_back(pose);
        }
      }
    }
    if(shapes_ok) {
      getEnvironmentModel()->lock();
      getEnvironmentModel()->clearObjects(collisionObject->id);
      getEnvironmentModel()->addObjects(collisionObject->id, shapes, poses);
      getEnvironmentModel()->unlock();
      ROS_INFO("Added %u object to namespace %s in collision space", (unsigned int)shapes.size(),collisionObject->id.c_str());
    } else {
      ROS_INFO_STREAM("Some shape in object " << collisionObject->id << " not ok, so not adding shape to collision space");
    }
  }
  else
  {
    // remove the object from the map
    getEnvironmentModel()->lock();
    if(collisionObject->id == "all") {
      ROS_INFO("Clearing all collision objects");
      getEnvironmentModel()->clearObjects();
    } else {
      getEnvironmentModel()->clearObjects(collisionObject->id);
    }
    getEnvironmentModel()->unlock();
    ROS_INFO("Removed object '%s' from collision space", collisionObject->id.c_str());
  }

  updateStaticObjectBodies(collisionObject);
    
  if (onCollisionObjectUpdate_)
    onCollisionObjectUpdate_(collisionObject);

}

bool planning_environment::CollisionSpaceMonitor::attachObjectCallback(const mapping_msgs::AttachedCollisionObjectConstPtr &attached_object)
{
  ROS_INFO("Calling attached object callback");

  bool result = true;

  getEnvironmentModel()->lock();

  if(attached_object->link_name == "all") { //attached_object->REMOVE_ALL_ATTACHED_OBJECTS) {
    if(attached_object->object.operation.operation != mapping_msgs::CollisionObjectOperation::REMOVE) {
      ROS_WARN("Can't perform any action for all attached bodies except remove");
      getEnvironmentModel()->unlock();
      return false;
    } 
    //waits for exclusive lock
    getKinematicModel()->clearAllAttachedBodyModels();
    getEnvironmentModel()->updateAttachedBodies();
    if(onAfterAttachCollisionObject_ != NULL) {
      onAfterAttachCollisionObject_(attached_object); 
    }
    ROS_INFO("All attached bodies cleared");
    getEnvironmentModel()->unlock();
    return true;
  }

  const planning_models::KinematicModel::LinkModel *link = getKinematicModel()->getLinkModel(attached_object->link_name);
  
  if(!link) {
    ROS_WARN_STREAM("Can't find link " << attached_object->link_name << " indicated by attach message");
    getEnvironmentModel()->unlock();
    return false;
  }
  //if there are no objects in the map, clear everything
  unsigned int n = attached_object->object.shapes.size();

  const mapping_msgs::CollisionObject& obj = attached_object->object;
      
  if(n == 0) {
    if(obj.operation.operation == mapping_msgs::CollisionObjectOperation::REMOVE) {
      //will wait for exclusive lock
      getKinematicModel()->clearLinkAttachedBodyModels(link->getName());
      getEnvironmentModel()->updateAttachedBodies();

      ROS_INFO_STREAM("Should have cleared all objects attached to " << link->getName());
      // call the event, if needed
      if (onAfterAttachCollisionObject_ != NULL) {
        onAfterAttachCollisionObject_(attached_object); 
      }
      getEnvironmentModel()->unlock();
      return true;
    } else if (obj.operation.operation == mapping_msgs::CollisionObjectOperation::ADD){
      ROS_INFO("Remove must also be specified to delete all attached bodies");
      getEnvironmentModel()->unlock();
      return false;
    } 
  } 
    
  bool delAtEnd = false;
  bool addAtEnd = false;
  
  const planning_models::KinematicModel::AttachedBodyModel* att = NULL;
  for (unsigned int i = 0 ; i < link->getAttachedBodyModels().size() ; ++i) {
    if(link->getAttachedBodyModels()[i]->getName() == obj.id) {
      att = link->getAttachedBodyModels()[i];
      break;
    }
  }
    
  std::vector<btTransform> link_frame_poses;
  std::vector<shapes::Shape*> shapes;
  
  if(obj.operation.operation == mapping_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT) {
    if(att == NULL) {
      ROS_WARN_STREAM("No attached object found for detach with id " << obj.id); 
    } else {
      getEnvironmentModel()->clearObjects(obj.id);
      std::vector<shapes::Shape*> shapes_temp;
      for(unsigned int i = 0; i < att->getShapes().size(); i++) {
        shapes_temp.push_back(shapes::cloneShape(att->getShapes()[i]));
      }
      //need state to get current locations
      planning_models::KinematicState state(getKinematicModel());
      setStateValuesFromCurrentValues(state);
      const planning_models::KinematicState::AttachedBodyState* att_state = state.getAttachedBodyState(att->getName());
      getEnvironmentModel()->addObjects(obj.id, shapes_temp, att_state->getGlobalCollisionBodyTransforms());
      ROS_INFO_STREAM("Adding in attached object as regular object " << obj.id);
      delAtEnd = true;
    }
  } else if(obj.operation.operation == mapping_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT) {
    const collision_space::EnvironmentObjects *eo = getEnvironmentModel()->getObjects();
    std::vector<std::string> ns = eo->getNamespaces();
    bool found = false;
    for (unsigned int i = 0 ; i < ns.size() ; ++i) {
      if(ns[i] == obj.id) {
        found = true;
        const collision_space::EnvironmentObjects::NamespaceObjects &no = eo->getObjects(ns[i]);
        for(unsigned int j = 0; j < no.shape.size(); j++) {
          geometry_msgs::PoseStamped p1;
          geometry_msgs::PoseStamped p2;
          tf::poseTFToMsg(no.shapePose[j], p1.pose);
	  std::string err_string;
	  ros::Time tm;
	  if (tf_->getLatestCommonTime(attached_object->link_name, getWorldFrameId(), tm, &err_string) == tf::NO_ERROR) {
	    p1.header.stamp = tm;
	    p1.header.frame_id = getWorldFrameId();
	    tf_->transformPose(attached_object->link_name, p1, p2);
	  } else {
	    ROS_WARN_STREAM("No common time between " << attached_object->link_name << " and " << getWorldFrameId());
	  }
	  btTransform bt;
          tf::poseMsgToTF(p2.pose, bt);
          link_frame_poses.push_back(bt);
          shapes.push_back(shapes::cloneShape(no.shape[j]));
        }
        getEnvironmentModel()->clearObjects(obj.id);
        ROS_INFO_STREAM("Clearing object " << obj.id << " before attach");
        addAtEnd = true;
        break;
      }
    }
    if(!found) {
      ROS_WARN_STREAM("Couldn't find object " << obj.id << " for attach and remove");
    }
  } else if(obj.operation.operation == mapping_msgs::CollisionObjectOperation::REMOVE) {
    delAtEnd = true;
  } else {
    //must be a straight add
    for(unsigned int i = 0; i < obj.shapes.size(); i++) {
      geometry_msgs::PoseStamped pose;
      geometry_msgs::PoseStamped poseP;
      pose.pose = obj.poses[i];
      pose.header = obj.header;
      bool err = false;
      std::string err_string;
      ros::Time tm;
      if (tf_->getLatestCommonTime(attached_object->link_name, pose.header.frame_id, tm, &err_string) == tf::NO_ERROR) {
	pose.header.stamp = tm;
	try {
	  tf_->transformPose(attached_object->link_name, pose, poseP);
	} catch(tf::TransformException& ex) {
	  err = true;
	  ROS_ERROR("Unable to transform object to be attached from frame %s to frame %s. TF said: %s", obj.header.frame_id.c_str(), attached_object->link_name.c_str(),ex.what());
	} 
      } else {
	ROS_WARN_STREAM("No common time between " << attached_object->link_name << " and " << pose.header.frame_id);
	err = true;
      }
      if (err) {
        continue;
      }
      
      shapes::Shape *shape = constructObject(obj.shapes[i]);
      if (!shape) {
        continue;
      }
      btTransform bt;
      tf::poseMsgToTF(poseP.pose, bt);
      link_frame_poses.push_back(bt);
      shapes.push_back(shape);
      addAtEnd = true;
      //need to get rid of current entry first
      if(att != NULL) {
        delAtEnd = true;
      }
    }
  }
  
  bool hasCur = (att != NULL);
  //add at end by default

  if(delAtEnd) {
    //bool found = false;
    if(!hasCur) {
      ROS_WARN("Can't delete what we don't have");
    } else {
      //will get exclusive lock
      getKinematicModel()->clearLinkAttachedBodyModel(link->getName(), obj.id);
      ROS_INFO_STREAM("Removing attached body associated with " << obj.id);
    }
  }
  if(addAtEnd) {
    //attached object should be allowed to touch attached link
    std::vector<std::string> touch_links = attached_object->touch_links;
    if(find(touch_links.begin(),touch_links.end(), link->getName()) == touch_links.end()) {
      touch_links.push_back(link->getName());
    } 

    planning_models::KinematicModel::AttachedBodyModel* ab = 
      new planning_models::KinematicModel::AttachedBodyModel(link, obj.id,
                                                             link_frame_poses,
                                                             touch_links,
                                                             shapes);
    getKinematicModel()->addAttachedBodyModel(link->getName(),
                                              ab);
    ROS_INFO_STREAM("Trying to add attached body id " << obj.id << " to link " << link->getName());
  }
  
  getEnvironmentModel()->updateAttachedBodies();
  // call the event, if needed
  if (onAfterAttachCollisionObject_ != NULL)
    onAfterAttachCollisionObject_(attached_object); 
  
  getEnvironmentModel()->unlock();    
  return result;
}

bool planning_environment::CollisionSpaceMonitor::computeAllowedContact(const motion_planning_msgs::AllowedContactSpecification &allowed_contact,
                                                                        collision_space::EnvironmentModel::AllowedContact &allowedContact) const
{
  shapes::Shape *shape = constructObject(allowed_contact.shape);
  if (shape)
  {
    boost::shared_ptr<bodies::Body> body(bodies::createBodyFromShape(shape));
    geometry_msgs::PoseStamped pose;
    tf_->transformPose(getWorldFrameId(), allowed_contact.pose_stamped, pose);
    btTransform tr;
    tf::poseMsgToTF(pose.pose, tr);
    body->setPose(tr);
    allowedContact.bound = body;
    allowedContact.links = allowed_contact.link_names;
    allowedContact.depth = allowed_contact.penetration_depth;
    delete shape;
    return true;
  }
  else
    return false;
}

void planning_environment::CollisionSpaceMonitor::recoverCollisionMap(mapping_msgs::CollisionMap &cmap)
{   
  recoverCollisionMap(getEnvironmentModel(), cmap);
}

void planning_environment::CollisionSpaceMonitor::recoverCollisionMap(const collision_space::EnvironmentModel *env, mapping_msgs::CollisionMap &cmap)
{
  getEnvironmentModel()->lock();

  cmap.header.frame_id = getWorldFrameId();
  cmap.header.stamp = ros::Time::now();
  cmap.boxes.clear();
    
  const collision_space::EnvironmentObjects::NamespaceObjects &no = env->getObjects()->getObjects("points");
  const unsigned int n = no.shape.size();
  //ROS_INFO_STREAM("Recovering " << no.shape.size() << " points");
  //double pd = pointcloud_padd_ * 2.0;    
  for (unsigned int i = 0 ; i < n ; ++i)
    if (no.shape[i]->type == shapes::BOX)
    {
      const shapes::Box* box = static_cast<const shapes::Box*>(no.shape[i]);
      mapping_msgs::OrientedBoundingBox obb;
      obb.extents.x = box->size[0];
      obb.extents.y = box->size[1];
      obb.extents.z = box->size[2];
      //if(i == 0) {
      //  ROS_INFO_STREAM("Sizes " << obb.extents.x << " " << obb.extents.y << " " << obb.extents.z);
      //}
      const btVector3 &c = no.shapePose[i].getOrigin();
      obb.center.x = c.x();
      obb.center.y = c.y();
      obb.center.z = c.z();
      const btQuaternion q = no.shapePose[i].getRotation();
      obb.angle = q.getAngle();
      const btVector3 axis = q.getAxis();
      obb.axis.x = axis.x();
      obb.axis.y = axis.y();
      obb.axis.z = axis.z();
      cmap.boxes.push_back(obb);
    }
  getEnvironmentModel()->unlock();
}

bool planning_environment::CollisionSpaceMonitor::getObjectsService(planning_environment_msgs::GetCollisionObjects::Request &req,
                                                                    planning_environment_msgs::GetCollisionObjects::Response &res) {
  getEnvironmentModel()->lock();

  recoverCollisionObjects(getEnvironmentModel(), res.collision_objects);
  if(req.include_points) {
    recoverCollisionMap(getEnvironmentModel(),res.points);
  }
  recoverAttachedCollisionObjects(getEnvironmentModel(), res.attached_collision_objects);

  getEnvironmentModel()->unlock();
  return true;
}
                                                                   
void planning_environment::CollisionSpaceMonitor::recoverCollisionObjects(std::vector<mapping_msgs::CollisionObject> &omap)
{
  recoverCollisionObjects(getEnvironmentModel(), omap);
}

void planning_environment::CollisionSpaceMonitor::recoverCollisionObjects(const collision_space::EnvironmentModel *env, std::vector<mapping_msgs::CollisionObject> &omap)
{
  getEnvironmentModel()->lock();
  omap.clear();
  const collision_space::EnvironmentObjects *eo = env->getObjects();
  std::vector<std::string> ns = eo->getNamespaces();
  for (unsigned int i = 0 ; i < ns.size() ; ++i)
  {
    if (ns[i] == "points")
      continue;
    const collision_space::EnvironmentObjects::NamespaceObjects &no = eo->getObjects(ns[i]);
    const unsigned int n = no.shape.size();

    mapping_msgs::CollisionObject o;
    o.header.frame_id = getWorldFrameId();
    o.header.stamp = lastJointStateUpdate();
    o.id = ns[i];
    o.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
    for (unsigned int j = 0 ; j < n ; ++j) {
      geometric_shapes_msgs::Shape obj;
      if (constructObjectMsg(no.shape[j], obj)) {
        geometry_msgs::Pose pose;
        tf::poseTFToMsg(no.shapePose[j], pose);
        o.shapes.push_back(obj);
        o.poses.push_back(pose);
      }
    }
    omap.push_back(o);
  }
  getEnvironmentModel()->unlock();
}

void planning_environment::CollisionSpaceMonitor::recoverAttachedCollisionObjects(std::vector<mapping_msgs::AttachedCollisionObject> &avec) 
{
  recoverAttachedCollisionObjects(getEnvironmentModel(), avec);
}

void planning_environment::CollisionSpaceMonitor::recoverAttachedCollisionObjects(const collision_space::EnvironmentModel *env, std::vector<mapping_msgs::AttachedCollisionObject> &avec) 
{
  getEnvironmentModel()->lock();

  planning_models::KinematicState state(getKinematicModel());    
  setStateValuesFromCurrentValues(state);
  getEnvironmentModel()->updateRobotModel(&state);
  
  avec.clear();

  const std::vector<const planning_models::KinematicModel::AttachedBodyModel*>& att_vec = env->getAttachedBodies();
  for(unsigned int i = 0; i < att_vec.size(); i++) 
  {
    mapping_msgs::AttachedCollisionObject ao;
    ao.object.header.frame_id = getWorldFrameId();
    ao.object.header.stamp = ros::Time::now();
    ao.link_name = att_vec[i]->getAttachedLinkModel()->getName();
    double attached_padd = env->getCurrentLinkPadding("attached");
    for(unsigned int j = 0; j < att_vec[i]->getShapes().size(); j++) {
      geometric_shapes_msgs::Shape shape;
      constructObjectMsg(att_vec[i]->getShapes()[j], shape, attached_padd);
      geometry_msgs::Pose pose;
      planning_models::KinematicState::AttachedBodyState* att_state = state.getAttachedBodyState(att_vec[i]->getName());
      if(att_state == NULL) {
        ROS_WARN_STREAM("No attached body state for attached body model " << att_vec[i]->getName());
      }
      tf::poseTFToMsg(att_state->getGlobalCollisionBodyTransforms()[j], pose);
      ao.object.shapes.push_back(shape);
      ao.object.poses.push_back(pose);
    }
    ao.touch_links = att_vec[i]->getTouchLinks();
    ao.object.id = att_vec[i]->getName();
    avec.push_back(ao);
  }
  getEnvironmentModel()->unlock();
}

bool planning_environment::CollisionSpaceMonitor::getCurrentAllowedCollisionsService(planning_environment_msgs::GetAllowedCollisionMatrix::Request& req,
                                                                                     planning_environment_msgs::GetAllowedCollisionMatrix::Response& res) {
  std::vector<std::vector<bool> > matrix;
  std::map<std::string, unsigned int> ind;
  
  getEnvironmentModel()->lock();
  getEnvironmentModel()->getCurrentAllowedCollisionMatrix(matrix, ind);
  getEnvironmentModel()->unlock();

  //printAllowedCollisionMatrix(matrix, ind);

  unsigned int i = 0;
  res.matrix.link_names.resize(ind.size());
  res.matrix.entries.resize(ind.size());
  for(std::map<std::string, unsigned int>::iterator it = ind.begin();
      it != ind.end();
      it++,i++) {
    res.matrix.link_names[i] = it->first;
    res.matrix.entries[i].enabled.resize(matrix[it->second].size());
    unsigned int j = 0;
    for(std::map<std::string, unsigned int>::iterator it2 = ind.begin();
        it2 != ind.end();
        it2++,j++) {
      res.matrix.entries[i].enabled[j] = matrix[it->second][it2->second];
    }
  }
  return true;
}

bool planning_environment::CollisionSpaceMonitor::setAllowedCollisionsService(planning_environment_msgs::SetAllowedCollisions::Request& req,
                                                                              planning_environment_msgs::SetAllowedCollisions::Response& res)
{
  getEnvironmentModel()->lock();
  bool ok = applyOrderedCollisionOperationsToCollisionSpace(req.ord);
  if(!ok) {
    ROS_WARN("Can't apply ordered collision operations");
    getEnvironmentModel()->unlock();
    return false;
  }
  planning_environment_msgs::GetAllowedCollisionMatrix::Request greq;
  planning_environment_msgs::GetAllowedCollisionMatrix::Response gres;
  getCurrentAllowedCollisionsService(greq,gres);
  res.matrix = gres.matrix;
  getEnvironmentModel()->unlock();
  return true;
}

bool planning_environment::CollisionSpaceMonitor::revertAllowedCollisionMatrixToDefaultService(std_srvs::Empty::Request& req,
                                                                                               std_srvs::Empty::Response& res) {
  getEnvironmentModel()->lock();
  revertAllowedCollisionToDefault();
  getEnvironmentModel()->unlock();
  return true;
}

bool planning_environment::CollisionSpaceMonitor::applyOrderedCollisionOperationsToCollisionSpace(const motion_planning_msgs::OrderedCollisionOperations &ord, bool print) {

  //getting the default set of enabled collisions
  std::vector<std::vector<bool> > curAllowed;
  std::map<std::string, unsigned int> vecIndices;
  getEnvironmentModel()->lock();
  getEnvironmentModel()->getDefaultAllowedCollisionMatrix(curAllowed, vecIndices);
  getEnvironmentModel()->unlock();

  motion_planning_msgs::OrderedCollisionOperations expanded;
  expandOrderedCollisionOperations(ord, expanded);

  //std::cout << "Default:\n";
  //printAllowedCollisionMatrix(curAllowed, vecIndices);

  bool ok = applyOrderedCollisionOperationsToMatrix(expanded, curAllowed, vecIndices);

  if(print) {
    printAllowedCollisionMatrix(curAllowed, vecIndices);
  }

  getEnvironmentModel()->lock();
  getEnvironmentModel()->setAllowedCollisionMatrix(curAllowed, vecIndices);
  getEnvironmentModel()->unlock();
  return ok;
}

bool planning_environment::CollisionSpaceMonitor::applyOrderedCollisionOperationsToMatrix(const motion_planning_msgs::OrderedCollisionOperations &ord,
                                                                                          std::vector<std::vector<bool> > &curAllowed,
                                                                                          std::map<std::string, unsigned int> &vecIndices) {
  size_t all_size = curAllowed.size();

  for(size_t i = 0; i < ord.collision_operations.size(); i++) {
    
    bool allowed = (ord.collision_operations[i].operation == motion_planning_msgs::CollisionOperation::DISABLE);
    
    if(ord.collision_operations[i].object1 == ord.collision_operations[i].COLLISION_SET_ALL &&
       ord.collision_operations[i].object2 == ord.collision_operations[i].COLLISION_SET_ALL) {
      //first case - both all
      for(size_t j = 0; j < all_size; j++) {
        for(size_t k = 0; k < all_size; k++) {
          curAllowed[j][k] = allowed;
        }
      }
    } else if(ord.collision_operations[i].object1 == ord.collision_operations[i].COLLISION_SET_ALL ||
              ord.collision_operations[i].object2 == ord.collision_operations[i].COLLISION_SET_ALL) {
      //second case - only one all
      std::string other;
      if(ord.collision_operations[i].object1 == ord.collision_operations[i].COLLISION_SET_ALL) {
        other = ord.collision_operations[i].object2;
      } else {
        other = ord.collision_operations[i].object1;
      }
      if(vecIndices.find(other) == vecIndices.end()) {
        continue;
      }
      unsigned int obj_ind = vecIndices.find(other)->second;
      for(size_t j = 0; j < all_size; j++) {
        curAllowed[obj_ind][j] = allowed;
        curAllowed[j][obj_ind] = allowed;
      }
    } else {
      //third case - must be both objects
      if(vecIndices.find(ord.collision_operations[i].object1) == vecIndices.end() ||
         vecIndices.find(ord.collision_operations[i].object2) == vecIndices.end()) {
        continue;
      }
      unsigned int obj1ind = vecIndices.find(ord.collision_operations[i].object1)->second;
      unsigned int obj2ind = vecIndices.find(ord.collision_operations[i].object2)->second;
      curAllowed[obj1ind][obj2ind] = allowed;
      curAllowed[obj2ind][obj1ind] = allowed;
    }
  }
  return true;
}
                                      

void planning_environment::CollisionSpaceMonitor::revertAllowedCollisionToDefault() {

  getEnvironmentModel()->lock();
  getEnvironmentModel()->revertAllowedCollisionMatrix();
  getEnvironmentModel()->unlock();
}

void planning_environment::CollisionSpaceMonitor::applyLinkPaddingToCollisionSpace(const std::vector<motion_planning_msgs::LinkPadding>& link_padding) {
  if(link_padding.empty()) return;

  const std::map<std::string, std::vector<std::string > >& group_link_map = rm_->getPlanningGroupLinks();
  std::map<std::string, double> link_padding_map;

  for(std::vector<motion_planning_msgs::LinkPadding>::const_iterator it = link_padding.begin();
      it != link_padding.end();
      it++) {
    std::vector<std::string> svec1;
    if(group_link_map.find((*it).link_name) != group_link_map.end()) {
      svec1 = group_link_map.find((*it).link_name)->second;
    } else {
      svec1.push_back((*it).link_name);
    }
    for(std::vector<std::string>::iterator stit1 = svec1.begin();
        stit1 != svec1.end();
        stit1++) {
      link_padding_map[(*stit1)] = (*it).padding;
    }
  }

  getEnvironmentModel()->lock();
  getEnvironmentModel()->setRobotLinkPadding(link_padding_map);  
  getEnvironmentModel()->unlock();

}

void planning_environment::CollisionSpaceMonitor::revertCollisionSpacePaddingToDefault() {
  getEnvironmentModel()->lock();
  getEnvironmentModel()->revertRobotLinkPadding();
  getEnvironmentModel()->unlock();
}

bool planning_environment::CollisionSpaceMonitor::expandOrderedCollisionOperations(const motion_planning_msgs::OrderedCollisionOperations &ord,
                                                                                   motion_planning_msgs::OrderedCollisionOperations &ex) {
  getEnvironmentModel()->lock();

  std::vector<std::string> o_strings;    
  std::vector<mapping_msgs::CollisionObject> ovec;
  recoverCollisionObjects(getEnvironmentModel(), ovec);
  for(std::vector<mapping_msgs::CollisionObject>::iterator it = ovec.begin();
      it != ovec.end();
      it++) {
    o_strings.push_back((*it).id);
  }
  //this doesn't get returned with the objects
  o_strings.push_back("points");

  getKinematicModel()->sharedLock();  
  std::vector<std::string> a_strings;
  const std::vector<const planning_models::KinematicModel::AttachedBodyModel*>& att_vec = getEnvironmentModel()->getAttachedBodies();
  for(unsigned int i = 0; i < att_vec.size(); i++) 
  {
    a_strings.push_back(att_vec[i]->getName());
  }
  getKinematicModel()->sharedUnlock();

  const std::map<std::string, std::vector<std::string > >& group_link_map = rm_->getPlanningGroupLinks();

  ex = ord;
  std::vector<motion_planning_msgs::CollisionOperation>::iterator it = ex.collision_operations.begin();
  while(it != ex.collision_operations.end()) {
    std::vector<std::string> svec1;
    std::vector<std::string> svec2;
    bool special1 = false;
    bool special2 = false;
    if((*it).object1 == (*it).COLLISION_SET_OBJECTS) {
      svec1 = o_strings;
      special1 = true;
    }
    if((*it).object2 == (*it).COLLISION_SET_OBJECTS) {
      svec2 = o_strings;
      special2 = true;
    }
    if((*it).object1 == (*it).COLLISION_SET_ATTACHED_OBJECTS) {
      svec1 = a_strings;
      if(a_strings.empty()) {
        ROS_DEBUG_STREAM("No attached bodies");
      } else {
        ROS_DEBUG_STREAM("Setting first vec to all attached bodies, first entry " << a_strings[0]);
      }
      special1 = true;
    }
    if((*it).object2 == (*it).COLLISION_SET_ATTACHED_OBJECTS) {
      svec2 = a_strings;
      if(a_strings.empty()) {
        ROS_DEBUG_STREAM("No attached bodies");
      } else {
        ROS_DEBUG_STREAM("Setting second vec to all attached bodies, first entry " << a_strings[0]);
      }
      special2 = true;
    }
    if(group_link_map.find((*it).object1) != group_link_map.end()) {
      svec1 = group_link_map.find((*it).object1)->second;
      special1 = true;
    }
    if(group_link_map.find((*it).object2) != group_link_map.end()) {
      svec2 = group_link_map.find((*it).object2)->second;
      special2 = true;
    }
    if(!special1) {
      svec1.push_back((*it).object1);
    }
    if(!special2) {
      svec2.push_back((*it).object2);
    }

    int32_t op = (*it).operation;
      
    //we erase the original operation
    it = ex.collision_operations.erase(it);

    //EGJ 05/24/2010 Actually adding in attached bodies by default
    //addAttachedCollisionObjects(svec1);
    //addAttachedCollisionObjects(svec2);

    //now we need to add all the new pairwise constraints
    for(std::vector<std::string>::iterator stit1 = svec1.begin();
        stit1 != svec1.end();
        stit1++) {
      for(std::vector<std::string>::iterator stit2 = svec2.begin();
          stit2 != svec2.end();
          stit2++) {
        motion_planning_msgs::CollisionOperation coll;
        coll.object1 = (*stit1);
        coll.object2 = (*stit2);
        coll.operation = op;
        it = ex.collision_operations.insert(it,coll);
        it++;
      }
    }
  }
  for(it = ex.collision_operations.begin(); it != ex.collision_operations.end();it++) {
    ROS_DEBUG_STREAM("Expanded coll " << (*it).object1 << " " << (*it).object2 << " op " << (*it).operation);
  }
  getEnvironmentModel()->unlock();
  return true;
}


void planning_environment::CollisionSpaceMonitor::printAllowedCollisionMatrix(const std::vector<std::vector<bool> > &curAllowed,
                                                                              const std::map<std::string, unsigned int> &vecIndices) const {
  size_t all_size = curAllowed.size();
  for(unsigned int i = 0; i < vecIndices.size(); i++) {
    std::string n;
    for(std::map<std::string, unsigned int>::const_iterator it = vecIndices.begin();
        it != vecIndices.end();
        it++) {
      if(it->second == i) {
        n = it->first; 
      }
    }
    if(n.empty()) {
      ROS_WARN_STREAM("Can't find index " << i << " in vecIndex");
      return;
    }
    std::cout << std::setw(40) << n;
    std::cout << " | ";
    for(size_t j = 0; j < all_size; j++) {
      std::cout << std::setw(3) << curAllowed[i][j];
    }
    std::cout << std::endl;
  }
}

void planning_environment::CollisionSpaceMonitor::addAttachedCollisionObjects(std::vector<std::string>& svec) const {

  getKinematicModel()->sharedLock();

  std::vector<std::string>::iterator stit = svec.begin();
  while(stit != svec.end()) {
    const std::vector<const planning_models::KinematicModel::AttachedBodyModel*> att_vec = getEnvironmentModel()->getAttachedBodies(*stit);
    //these get inserted after the link
    stit++;
    for(std::vector<const planning_models::KinematicModel::AttachedBodyModel*>::const_iterator ait = att_vec.begin();
        ait != att_vec.end();
        ait++) {
      ROS_DEBUG_STREAM("Adding attached collision object " << (*ait)->getName() << " to list");
      stit = svec.insert(stit,(*ait)->getName());
    }
  }

  getKinematicModel()->sharedUnlock();
}
