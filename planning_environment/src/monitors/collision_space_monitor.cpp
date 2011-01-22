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
    
  have_map_ = false;
  use_collision_map_ = false;

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
    collisionMapFilter_ = new tf::MessageFilter<mapping_msgs::CollisionMap>(*collisionMapSubscriber_, *tf_, cm_->getWorldFrameId(), 1);
    collisionMapFilter_->registerCallback(boost::bind(&CollisionSpaceMonitor::collisionMapCallback, this, _1));
    ROS_DEBUG("Listening to collision_map using message notifier with target frame %s", collisionMapFilter_->getTargetFramesString().c_str());
    
    collisionMapUpdateSubscriber_ = new message_filters::Subscriber<mapping_msgs::CollisionMap>(root_handle_, "collision_map_update", 1024);
    collisionMapUpdateFilter_ = new tf::MessageFilter<mapping_msgs::CollisionMap>(*collisionMapUpdateSubscriber_, *tf_, cm_->getWorldFrameId(), 1);
    collisionMapUpdateFilter_->registerCallback(boost::bind(&CollisionSpaceMonitor::collisionMapUpdateCallback, this, _1));
    ROS_DEBUG("Listening to collision_map_update using message notifier with target frame %s", collisionMapUpdateFilter_->getTargetFramesString().c_str());
  }

  collisionObjectSubscriber_ = new message_filters::Subscriber<mapping_msgs::CollisionObject>(root_handle_, "collision_object", 1024);
  collisionObjectFilter_ = new tf::MessageFilter<mapping_msgs::CollisionObject>(*collisionObjectSubscriber_, *tf_, cm_->getWorldFrameId(), 1024);
  collisionObjectFilter_->registerCallback(boost::bind(&CollisionSpaceMonitor::collisionObjectCallback, this, _1));
  ROS_DEBUG("Listening to object_in_map using message notifier with target frame %s", collisionObjectFilter_->getTargetFramesString().c_str());
  
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
    collisionMapFilter_ = new tf::MessageFilter<mapping_msgs::CollisionMap>(*collisionMapSubscriber_, *tf_, cm_->getWorldFrameId(), 1);
    collisionMapFilter_->registerCallback(boost::bind(&CollisionSpaceMonitor::collisionMapCallback, this, _1));
    ROS_DEBUG("Listening to collision_map using message notifier with target frame %s", collisionMapFilter_->getTargetFramesString().c_str());
    
    collisionMapUpdateSubscriber_ = new message_filters::Subscriber<mapping_msgs::CollisionMap>(root_handle_, "collision_map_update", 1);
    collisionMapUpdateFilter_ = new tf::MessageFilter<mapping_msgs::CollisionMap>(*collisionMapUpdateSubscriber_, *tf_, cm_->getWorldFrameId(), 1);
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

void planning_environment::CollisionSpaceMonitor::collisionMapAsBoxes(const mapping_msgs::CollisionMapConstPtr &collision_map,
                                                                      std::vector<shapes::Shape*> &boxes, std::vector<btTransform> &poses)
{
  collisionMapAsBoxes(*collision_map, boxes, poses);
}

void planning_environment::CollisionSpaceMonitor::collisionMapAsBoxes(const mapping_msgs::CollisionMap& collision_map,
                                                                      std::vector<shapes::Shape*> &boxes, std::vector<btTransform> &poses)
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
	    
      poses[i].setRotation(btQuaternion(btVector3(collision_map.boxes[i].axis.x, collision_map.boxes[i].axis.y, collision_map.boxes[i].axis.z), collision_map.boxes[i].angle));
      poses[i].setOrigin(btVector3(pso.point.x, pso.point.y, pso.point.z));
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
      poses[i].setRotation(btQuaternion(btVector3(collision_map.boxes[i].axis.x, collision_map.boxes[i].axis.y, collision_map.boxes[i].axis.z), collision_map.boxes[i].angle));
      poses[i].setOrigin(btVector3(collision_map.boxes[i].center.x, collision_map.boxes[i].center.y, collision_map.boxes[i].center.z));
      boxes[i] = new shapes::Box(collision_map.boxes[i].extents.x + pd, collision_map.boxes[i].extents.y + pd, collision_map.boxes[i].extents.z + pd);
    }
  }
}

void planning_environment::CollisionSpaceMonitor::updateCollisionSpace(const mapping_msgs::CollisionMapConstPtr &collision_map, bool clear)
{ 
  std::vector<shapes::Shape*> shapes;
  std::vector<btTransform> poses;
  
  collisionMapAsBoxes(*collision_map, shapes, poses);
  cm_->setCollisionMap(shapes, poses, true);
  last_map_update_ = collision_map->header.stamp;
  have_map_ = true;
}

bool planning_environment::CollisionSpaceMonitor::createAndPoseShapes(const std::vector<geometric_shapes_msgs::Shape>& orig_shapes,
                                                                      const std::vector<geometry_msgs::Pose>& orig_poses,
                                                                      const std_msgs::Header& header, 
                                                                      const std::string& frame_to,
                                                                      std::vector<shapes::Shape*>& conv_shapes,
                                                                      std::vector<btTransform>& conv_poses)
{
  conv_shapes.clear();
  conv_poses.clear();
  bool shapes_ok = true;
  for(unsigned int i = 0; i < orig_shapes.size(); i++) {
    shapes::Shape *shape = constructObject(orig_shapes[i]);
    if(shape == NULL) {
      shapes_ok = false;
      break;
    }
    conv_shapes.push_back(shape);
    std::string err_string;
    ros::Time tm;
    geometry_msgs::PoseStamped temp_pose;
    temp_pose.pose = orig_poses[i];
    temp_pose.header = header;
    geometry_msgs::PoseStamped trans_pose;
    if (tf_->getLatestCommonTime(frame_to, temp_pose.header.frame_id, tm, &err_string) == tf::NO_ERROR) {
      temp_pose.header.stamp = tm;
      try {
        tf_->transformPose(frame_to, temp_pose, trans_pose);
      } catch(tf::TransformException& ex) {
        ROS_ERROR_STREAM("Unable to transform object from frame " << temp_pose.header.frame_id << " to " << frame_to << " error is " << ex.what());
        shapes_ok = false;
        break;
      }
      btTransform pose;
      tf::poseMsgToTF(trans_pose.pose, pose);
      ROS_DEBUG_STREAM("Object is at " << pose.getOrigin().x() << " " << pose.getOrigin().y());
      conv_poses.push_back(pose);
    } else {
      ROS_ERROR_STREAM("Something wrong with tf");
      shapes_ok = false;
      break;
    }
  }
  if(!shapes_ok) {
    for(unsigned int i=0; i < conv_shapes.size(); i++) {
      delete conv_shapes[i];
    }
    conv_shapes.clear();
    conv_poses.clear();
    return false;
  }
  return true;

}

void planning_environment::CollisionSpaceMonitor::collisionObjectCallback(const mapping_msgs::CollisionObjectConstPtr &collision_object)
{
  if (collision_object->operation.operation == mapping_msgs::CollisionObjectOperation::ADD) {
    std::vector<shapes::Shape*> shapes;
    std::vector<btTransform> poses;
    bool shapes_ok = createAndPoseShapes(collision_object->shapes,
                                         collision_object->poses,
                                         collision_object->header,
                                         cm_->getWorldFrameId(),
                                         shapes,
                                         poses);
    if(!shapes_ok) {
      ROS_INFO_STREAM("Not adding attached object " << collision_object->id 
                      << " to collision space because something's wrong with the shapes");
    } else {
      cm_->addStaticObject(collision_object->id,
                           shapes,
                           poses);
      ROS_INFO("Added %u object to namespace %s in collision space", (unsigned int)shapes.size(),collision_object->id.c_str()); 
    } 
  } else {
    if(collision_object->id == "all") {
      ROS_INFO("Clearing all collision objects");
      cm_->deleteAllStaticObjects();
    } else {
      cm_->deleteStaticObject(collision_object->id);
      ROS_INFO("Removed object '%s' from collision space", collision_object->id.c_str());
    }
  }

  if (onCollisionObjectUpdate_)
    onCollisionObjectUpdate_(collision_object);
  
}

bool planning_environment::CollisionSpaceMonitor::attachObjectCallback(const mapping_msgs::AttachedCollisionObjectConstPtr &attached_object)
{
  ROS_INFO("Calling attached object callback");

  if(attached_object->link_name == "all") { //attached_object->REMOVE_ALL_ATTACHED_OBJECTS) {
    if(attached_object->object.operation.operation != mapping_msgs::CollisionObjectOperation::REMOVE) {
      ROS_WARN("Can't perform any action for all attached bodies except remove");
      return false;
    } 
    cm_->deleteAllAttachedObjects();
    if(onAfterAttachCollisionObject_ != NULL) {
      onAfterAttachCollisionObject_(attached_object); 
    }
  }

  //if there are no objects in the map, clear everything
  unsigned int n = attached_object->object.shapes.size();

  const mapping_msgs::CollisionObject& obj = attached_object->object;
      
  if(n == 0) {
    if(obj.operation.operation == mapping_msgs::CollisionObjectOperation::REMOVE) {
      cm_->deleteAllAttachedObjects();
      return true;
    } else if (obj.operation.operation == mapping_msgs::CollisionObjectOperation::ADD){
      ROS_INFO("Remove must also be specified to delete all attached bodies");
      return false;
    } 
  } 
    
  if(obj.operation.operation == mapping_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT) {
    cm_->convertAttachedObjectToStaticObject(obj.id,
                                             attached_object->link_name);
    ROS_INFO_STREAM("Adding in attached object as regular object " << obj.id);
  } else if(obj.operation.operation == mapping_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT) {
    cm_->convertStaticObjectToAttachedObject(obj.id,
                                             attached_object->link_name,
                                             attached_object->touch_links);
    ROS_INFO_STREAM("Adding in static object as attached object " << obj.id);
  } else if(obj.operation.operation == mapping_msgs::CollisionObjectOperation::REMOVE) {
    cm_->deleteAttachedObject(obj.id,
                              attached_object->link_name);
  } else {
    std::vector<shapes::Shape*> shapes;
    std::vector<btTransform> poses;
    bool shapes_ok = createAndPoseShapes(obj.shapes,
                                         obj.poses,
                                         obj.header,
                                         attached_object->link_name,
                                         shapes,
                                         poses);
    if(!shapes_ok) {
      for(unsigned int i = 0; i < shapes.size(); i++) {
        delete shapes[i];
      }
      ROS_INFO_STREAM("Not adding attached object " << obj.id
                      << " to collision space because something's wrong with the shapes");
      return true;
    }
    cm_->addAttachedObject(obj.id,
                           attached_object->link_name,
                           shapes,
                           poses,
                           attached_object->touch_links);
  }
  return true;
}

bool planning_environment::CollisionSpaceMonitor::getObjectsService(planning_environment_msgs::GetCollisionObjects::Request &req,
                                                                    planning_environment_msgs::GetCollisionObjects::Response &res) {
  cm_->getCollisionSpaceCollisionObjects(res.collision_objects);
  if(req.include_points) {
    cm_->getCollisionSpaceCollisionMap(res.points);
  }
  cm_->getCollisionSpaceAttachedCollisionObjects(res.attached_collision_objects);
  return true;
}
                                                                   

bool planning_environment::CollisionSpaceMonitor::getCurrentAllowedCollisionsService(planning_environment_msgs::GetAllowedCollisionMatrix::Request& req,
                                                                                     planning_environment_msgs::GetAllowedCollisionMatrix::Response& res) {
  cm_->getCollisionSpaceAllowedCollisions(res.matrix);
  return true;
}

bool planning_environment::CollisionSpaceMonitor::setAllowedCollisionsService(planning_environment_msgs::SetAllowedCollisions::Request& req,
                                                                              planning_environment_msgs::SetAllowedCollisions::Response& res)
{
  bool ok = cm_->applyOrderedCollisionOperationsToCollisionSpace(req.ord);
  if(!ok) {
    ROS_WARN("Can't apply ordered collision operations");
    return false;
  }
  planning_environment_msgs::GetAllowedCollisionMatrix::Request greq;
  planning_environment_msgs::GetAllowedCollisionMatrix::Response gres;
  getCurrentAllowedCollisionsService(greq,gres);
  res.matrix = gres.matrix;
  return true;
}

bool planning_environment::CollisionSpaceMonitor::revertAllowedCollisionMatrixToDefaultService(std_srvs::Empty::Request& req,
                                                                                               std_srvs::Empty::Response& res) {
  cm_->revertAllowedCollisionToDefault();
  return true;
}


