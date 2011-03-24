/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2011, Willow Garage, Inc.
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

#include <planning_environment/monitors/monitor_utils.h>
#include <planning_environment/util/construct_object.h>
#include <robot_self_filter/self_mask.h>
#include <pcl_ros/transforms.h>

bool planning_environment::getLatestIdentityTransform(const std::string& to_frame,
                                                      const std::string& from_frame,
                                                      tf::TransformListener& tf,
                                                      btTransform& pose) 
{
  geometry_msgs::PoseStamped temp_pose;
  temp_pose.pose.orientation.w = 1.0;
  temp_pose.header.frame_id = from_frame;
  geometry_msgs::PoseStamped trans_pose;
  ros::Time tm;
  std::string err_string;
  if (tf.getLatestCommonTime(to_frame, temp_pose.header.frame_id, tm, &err_string) == tf::NO_ERROR) {
    temp_pose.header.stamp = tm;
    try {
      tf.transformPose(to_frame, temp_pose, trans_pose);
    } catch(tf::TransformException& ex) {
      ROS_ERROR_STREAM("Unable to transform object from frame " << temp_pose.header.frame_id << " to " << to_frame << " error is " << ex.what());
      return false;
    }
  } else {
    ROS_ERROR_STREAM("No latest time for transforming " << from_frame << " to " << to_frame);
    return false;
  }
  tf::poseMsgToTF(trans_pose.pose, pose);
  return true;
}
                                                
bool planning_environment::createAndPoseShapes(tf::TransformListener& tf,
                                               const std::vector<geometric_shapes_msgs::Shape>& orig_shapes,
                                               const std::vector<geometry_msgs::Pose>& orig_poses,
                                               const std_msgs::Header& header, 
                                               const std::string& frame_to,
                                               std::vector<shapes::Shape*>& conv_shapes,
                                               std::vector<btTransform>& conv_poses)
{
  conv_shapes.clear();
  conv_poses.clear();
  bool shapes_ok = true;
  btTransform ident_trans;
  if(!getLatestIdentityTransform(frame_to, header.frame_id, tf, ident_trans)) {
    ROS_WARN_STREAM("Can't get identity pose to transform shapes");
    return false;
  }
  for(unsigned int i = 0; i < orig_shapes.size(); i++) {
    shapes::Shape *shape = constructObject(orig_shapes[i]);
    if(shape == NULL) {
      shapes_ok = false;
      break;
    }
    conv_shapes.push_back(shape);
    btTransform shape_pose;
    tf::poseMsgToTF(orig_poses[i], shape_pose);
    conv_poses.push_back(ident_trans*shape_pose);
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

bool planning_environment::processCollisionObjectMsg(const mapping_msgs::CollisionObjectConstPtr &collision_object,
                                                     tf::TransformListener& tf,
                                                     planning_environment::CollisionModels* cm)
{
  if (collision_object->operation.operation == mapping_msgs::CollisionObjectOperation::ADD) {
    std::vector<shapes::Shape*> shapes;
    std::vector<btTransform> poses;
    bool shapes_ok = createAndPoseShapes(tf,
                                         collision_object->shapes,
                                         collision_object->poses,
                                         collision_object->header,
                                         cm->getWorldFrameId(),
                                         shapes,
                                         poses);
    if(!shapes_ok) {
      ROS_INFO_STREAM("Not adding attached object " << collision_object->id 
                      << " to collision space because something's wrong with the shapes");
      return false;
    } else {
      double padding = cm->getDefaultObjectPadding();
      if(collision_object->padding < 0.0) {
        padding = 0.0;
      } else if(collision_object->padding > 0.0) {
        padding = collision_object->padding;
      }
      cm->addStaticObject(collision_object->id,
                          shapes,
                          poses,
                          padding);
      ROS_INFO("Added %u object to namespace %s in collision space", (unsigned int)shapes.size(),collision_object->id.c_str()); 
    } 
  } else {
    if(collision_object->id == "all") {
      ROS_INFO("Clearing all collision objects");
      cm->deleteAllStaticObjects();
    } else {
      cm->deleteStaticObject(collision_object->id);
      ROS_INFO("Removed object '%s' from collision space", collision_object->id.c_str());
    }
  }  
  return true;
}

bool planning_environment::processAttachedCollisionObjectMsg(const mapping_msgs::AttachedCollisionObjectConstPtr &attached_object,
                                                             tf::TransformListener& tf,
                                                             planning_environment::CollisionModels* cm)
{
  
  if(attached_object->link_name == "all") { //attached_object->REMOVE_ALL_ATTACHED_OBJECTS) {
    if(attached_object->object.operation.operation != mapping_msgs::CollisionObjectOperation::REMOVE) {
      ROS_WARN("Can't perform any action for all attached bodies except remove");
      return false;
    } 
    cm->deleteAllAttachedObjects();
  }

  //if there are no objects in the map, clear everything
  unsigned int n = attached_object->object.shapes.size();

  const mapping_msgs::CollisionObject& obj = attached_object->object;
      
  if(n == 0) {
    if(obj.operation.operation == mapping_msgs::CollisionObjectOperation::REMOVE) {
      cm->deleteAllAttachedObjects();
      return true;
    } else if (obj.operation.operation == mapping_msgs::CollisionObjectOperation::ADD){
      ROS_INFO("Remove must also be specified to delete all attached bodies");
      return false;
    } 
  } 
  
  if(obj.operation.operation == mapping_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT
     || obj.operation.operation == mapping_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT) {
    //getting link pose in the world frame
    btTransform link_pose;
    if(!getLatestIdentityTransform(cm->getWorldFrameId(), attached_object->link_name, tf, link_pose)) {
      ROS_WARN_STREAM("Can't get transform for link " << attached_object->link_name);
      return false;
    }
    if(obj.operation.operation == mapping_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT) {
      cm->convertAttachedObjectToStaticObject(obj.id,
                                              attached_object->link_name,
                                              link_pose);
      ROS_INFO_STREAM("Adding in attached object as regular object " << obj.id);
    } else {
      cm->convertStaticObjectToAttachedObject(obj.id,
                                              attached_object->link_name,
                                              link_pose,
                                              attached_object->touch_links);
      ROS_INFO_STREAM("Adding in static object as attached object " << obj.id);
    }
  } else if(obj.operation.operation == mapping_msgs::CollisionObjectOperation::REMOVE) {
    cm->deleteAttachedObject(obj.id,
                             attached_object->link_name);
  } else {
    std::vector<shapes::Shape*> shapes;
    std::vector<btTransform> poses;
    bool shapes_ok = createAndPoseShapes(tf,
                                         obj.shapes,
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
    double padding = cm->getDefaultObjectPadding();
    if(attached_object->object.padding < 0.0) {
      padding = 0.0;
    } else if(attached_object->object.padding > 0.0){
      padding = attached_object->object.padding;
    }
    cm->addAttachedObject(obj.id,
                          attached_object->link_name,
                          shapes,
                          poses,
                          attached_object->touch_links,
                          padding);
  }
  return true;
}

void planning_environment::updateAttachedObjectBodyPoses(planning_environment::CollisionModels* cm,
                                                         planning_models::KinematicState& state,
                                                         tf::TransformListener& tf)
{
  cm->bodiesLock();
  
  const std::map<std::string, std::map<std::string, bodies::BodyVector*> >& link_att_objects = cm->getLinkAttachedObjects();
  
  if(link_att_objects.empty()) {
    cm->bodiesUnlock();
    return;
  }

  //this gets all the attached bodies in their correct current positions according to tf
  geometry_msgs::PoseStamped ps;
  ps.pose.orientation.w = 1.0;
  for(std::map<std::string, std::map<std::string, bodies::BodyVector*> >::const_iterator it = link_att_objects.begin();
      it != link_att_objects.end();
      it++) {
    ps.header.frame_id = it->first;
    std::string es;
    if (tf.getLatestCommonTime(cm->getWorldFrameId(), it->first, ps.header.stamp, &es) != tf::NO_ERROR) {
      ROS_INFO_STREAM("Problem transforming into fixed frame from " << it->first << ".  Error string " << es);
      continue;
    }
    geometry_msgs::PoseStamped psout;
    tf.transformPose(cm->getWorldFrameId(), ps, psout);
    btTransform link_trans;
    tf::poseMsgToTF(psout.pose, link_trans);
    state.updateKinematicStateWithLinkAt(it->first, link_trans);
    cm->updateAttachedBodyPosesForLink(state, it->first);
  }
  cm->bodiesUnlock();
}

//assumes that the point is in the world frame
//and that state has been set
int planning_environment::computeAttachedObjectPointMask(const planning_environment::CollisionModels* cm,
                                                         const btVector3 &pt, 
                                                         const btVector3 &sensor_pos)
{
  cm->bodiesLock();
  const std::map<std::string, std::map<std::string, bodies::BodyVector*> > link_att_objects = cm->getLinkAttachedObjects();

  btVector3 dir(sensor_pos - pt);
  dir.normalize();
  
  for(std::map<std::string, std::map<std::string, bodies::BodyVector*> >::const_iterator it = link_att_objects.begin();
      it != link_att_objects.end();
      it++) {
    for(std::map<std::string, bodies::BodyVector*>::const_iterator it2 = it->second.begin();
        it2 != it->second.end();
        it2++) {
      for(unsigned int k = 0; k < it2->second->getSize(); k++) {
        //ROS_INFO_STREAM("Sphere distance " << it2->second->getBoundingSphere(k).center.distance2(pt)
        //                << " squared " << it2->second->getBoundingSphereRadiusSquared(k));
        if(it2->second->getPaddedBoundingSphere(k).center.distance2(pt) < it2->second->getPaddedBoundingSphereRadiusSquared(k)) {
          if(it2->second->getPaddedBody(k)->containsPoint(pt)) {
	    cm->bodiesUnlock();
	    return robot_self_filter::INSIDE;
	  }
        }
        if(it2->second->getPaddedBody(k)->intersectsRay(pt, dir)) {
          cm->bodiesUnlock();
          return robot_self_filter::SHADOW;
        }
      }
    }
  }
  cm->bodiesUnlock();
  return robot_self_filter::OUTSIDE;
}

bool planning_environment::configureForAttachedBodyMask(planning_models::KinematicState& state,
                                                        planning_environment::CollisionModels* cm,
                                                        tf::TransformListener& tf,
                                                        const std::string& sensor_frame,
                                                        const ros::Time& sensor_time,
                                                        btVector3& sensor_pos)
{
  state.setKinematicStateToDefault();

  cm->bodiesLock();
  
  const std::map<std::string, std::map<std::string, bodies::BodyVector*> >& link_att_objects = cm->getLinkAttachedObjects();
  
  if(link_att_objects.empty()) {
    cm->bodiesUnlock();
    return false;
  }

  planning_environment::updateAttachedObjectBodyPoses(cm,
                                                      state,
                                                      tf);
  
  sensor_pos.setValue(0.0,0.0,0.0);

  // compute the origin of the sensor in the frame of the cloud
  if (!sensor_frame.empty()) {
    std::string err;
    try {
      tf::StampedTransform transf;
      tf.lookupTransform(cm->getWorldFrameId(), sensor_frame, sensor_time, transf);
      sensor_pos = transf.getOrigin();
    } catch(tf::TransformException& ex) {
      ROS_ERROR("Unable to lookup transform from %s to %s. Exception: %s", sensor_frame.c_str(), cm->getWorldFrameId().c_str(), ex.what());
      sensor_pos.setValue(0, 0, 0);
    }
  } 
  cm->bodiesUnlock();
  return true;
}
                                                         
bool planning_environment::computeAttachedObjectPointCloudMask(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud,
                                                               const std::string& sensor_frame,
                                                               planning_environment::CollisionModels* cm,
                                                               tf::TransformListener& tf,
                                                               std::vector<int> &mask)
{

  // compute mask for cloud
  int n = pcl_cloud.points.size();
  mask.resize(n, robot_self_filter::OUTSIDE);

  //state lock before body lock
  planning_models::KinematicState state(cm->getKinematicModel());
  
  btVector3 sensor_pos;
  
  planning_environment::configureForAttachedBodyMask(state,
                                                     cm,
                                                     tf,
                                                     sensor_frame,
                                                     pcl_cloud.header.stamp,
                                                     sensor_pos);

  // transform pointcloud into fixed frame, if needed
  if (cm->getWorldFrameId() != pcl_cloud.header.frame_id) {
    pcl::PointCloud<pcl::PointXYZ> trans_cloud = pcl_cloud;
    pcl_ros::transformPointCloud(cm->getWorldFrameId(), pcl_cloud, trans_cloud,tf);
    for (int i = 0 ; i < n ; ++i) {
      btVector3 pt = btVector3(trans_cloud.points[i].x, trans_cloud.points[i].y, trans_cloud.points[i].z);
      mask[i] = computeAttachedObjectPointMask(cm, pt, sensor_pos);
    }
  } else {
    for (int i = 0 ; i < n ; ++i) {
      btVector3 pt = btVector3(pcl_cloud.points[i].x, pcl_cloud.points[i].y, pcl_cloud.points[i].z);
      mask[i] = computeAttachedObjectPointMask(cm, pt, sensor_pos);
    }
  }
  return true;
}
