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
    if (tf.getLatestCommonTime(frame_to, temp_pose.header.frame_id, tm, &err_string) == tf::NO_ERROR) {
      temp_pose.header.stamp = tm;
      try {
        tf.transformPose(frame_to, temp_pose, trans_pose);
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
      cm->addStaticObject(collision_object->id,
                           shapes,
                           poses);
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
                                                             CollisionModels* cm)
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
    
  if(obj.operation.operation == mapping_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT) {
    cm->convertAttachedObjectToStaticObject(obj.id,
                                            attached_object->link_name);
    ROS_INFO_STREAM("Adding in attached object as regular object " << obj.id);
  } else if(obj.operation.operation == mapping_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT) {
    cm->convertStaticObjectToAttachedObject(obj.id,
                                            attached_object->link_name,
                                            attached_object->touch_links);
    ROS_INFO_STREAM("Adding in static object as attached object " << obj.id);
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
    cm->addAttachedObject(obj.id,
                           attached_object->link_name,
                          shapes,
                          poses,
                          attached_object->touch_links);
  }
  return true;
}

