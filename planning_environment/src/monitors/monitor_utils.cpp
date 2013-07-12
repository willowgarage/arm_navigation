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
#include <angles/angles.h>

bool planning_environment::getLatestIdentityTransform(const std::string& to_frame,
                                                      const std::string& from_frame,
                                                      tf::TransformListener& tf,
                                                      tf::Transform& pose) 
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
                                               const std::vector<arm_navigation_msgs::Shape>& orig_shapes,
                                               const std::vector<geometry_msgs::Pose>& orig_poses,
                                               const std_msgs::Header& header, 
                                               const std::string& frame_to,
                                               std::vector<shapes::Shape*>& conv_shapes,
                                               std::vector<tf::Transform>& conv_poses)
{
  conv_shapes.clear();
  conv_poses.clear();
  bool shapes_ok = true;
  tf::Transform ident_trans;
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
    tf::Transform shape_pose;
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

bool planning_environment::processCollisionObjectMsg(const arm_navigation_msgs::CollisionObjectConstPtr &collision_object,
                                                     tf::TransformListener& tf,
                                                     planning_environment::CollisionModels* cm)
{
  if (collision_object->operation.operation == arm_navigation_msgs::CollisionObjectOperation::ADD) {
    std::vector<shapes::Shape*> shapes;
    std::vector<tf::Transform> poses;
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
      cm->deleteAllStaticObjects();
      ROS_INFO("Cleared all collision objects");
    } else {
      cm->deleteStaticObject(collision_object->id);
      ROS_INFO("Removed object '%s' from collision space", collision_object->id.c_str());
    }
  }  
  return true;
}

bool planning_environment::processAttachedCollisionObjectMsg(const arm_navigation_msgs::AttachedCollisionObjectConstPtr &attached_object,
                                                             tf::TransformListener& tf,
                                                             planning_environment::CollisionModels* cm)
{
  if(attached_object->link_name == "all") { //attached_object->REMOVE_ALL_ATTACHED_OBJECTS) {
    if(attached_object->object.operation.operation != arm_navigation_msgs::CollisionObjectOperation::REMOVE) {
      ROS_WARN("Can't perform any action for all attached bodies except remove");
      return false;
    } 
    cm->deleteAllAttachedObjects();
    ROS_INFO_STREAM("Cleared all attached objects");
    return true;
  }

  //if there are no objects in the map, clear everything
  unsigned int n = attached_object->object.shapes.size();

  const arm_navigation_msgs::CollisionObject& obj = attached_object->object;
      
  if(n == 0) {
    if(obj.operation.operation == arm_navigation_msgs::CollisionObjectOperation::REMOVE) {
      cm->deleteAllAttachedObjects(attached_object->link_name);
      return true;
    } else if (obj.operation.operation == arm_navigation_msgs::CollisionObjectOperation::ADD){
      ROS_INFO("Remove must also be specified to delete all attached bodies");
      return false;
    } 
  } 
  
  if(obj.operation.operation == arm_navigation_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT
     || obj.operation.operation == arm_navigation_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT) {
    //getting link pose in the world frame
    tf::Transform link_pose;
    if(!getLatestIdentityTransform(cm->getWorldFrameId(), attached_object->link_name, tf, link_pose)) {
      ROS_WARN_STREAM("Can't get transform for link " << attached_object->link_name);
      return false;
    }
    if(obj.operation.operation == arm_navigation_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT) {
      cm->convertAttachedObjectToStaticObject(obj.id,
                                              attached_object->link_name,
                                              link_pose);
    } else {
      cm->convertStaticObjectToAttachedObject(obj.id,
                                              attached_object->link_name,
                                              link_pose,
                                              attached_object->touch_links);
    }
  } else if(obj.operation.operation == arm_navigation_msgs::CollisionObjectOperation::REMOVE) {
    cm->deleteAttachedObject(obj.id,
                             attached_object->link_name);
  } else {
    std::vector<shapes::Shape*> shapes;
    std::vector<tf::Transform> poses;
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
      ROS_WARN_STREAM("Not adding attached object " << obj.id
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
    tf::Transform link_trans;
    tf::poseMsgToTF(psout.pose, link_trans);
    state.updateKinematicStateWithLinkAt(it->first, link_trans);
    cm->updateAttachedBodyPosesForLink(state, it->first);
  }
  cm->bodiesUnlock();
}

//assumes that the point is in the world frame
//and that state has been set
int planning_environment::computeAttachedObjectPointMask(const planning_environment::CollisionModels* cm,
                                                         const tf::Vector3 &pt, 
                                                         const tf::Vector3 &sensor_pos)
{
  cm->bodiesLock();
  const std::map<std::string, std::map<std::string, bodies::BodyVector*> > link_att_objects = cm->getLinkAttachedObjects();

  tf::Vector3 dir(sensor_pos - pt);
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
                                                        tf::Vector3& sensor_pos)
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
  
  tf::Vector3 sensor_pos;
  
  planning_environment::configureForAttachedBodyMask(state,
                                                     cm,
                                                     tf,
                                                     sensor_frame,
                                                     ros::Time(pcl_cloud.header.stamp),
                                                     sensor_pos);

  // transform pointcloud into fixed frame, if needed
  if (cm->getWorldFrameId() != pcl_cloud.header.frame_id) {
    pcl::PointCloud<pcl::PointXYZ> trans_cloud = pcl_cloud;
    pcl_ros::transformPointCloud(cm->getWorldFrameId(), pcl_cloud, trans_cloud,tf);
    for (int i = 0 ; i < n ; ++i) {
      tf::Vector3 pt = tf::Vector3(trans_cloud.points[i].x, trans_cloud.points[i].y, trans_cloud.points[i].z);
      mask[i] = computeAttachedObjectPointMask(cm, pt, sensor_pos);
    }
  } else {
    for (int i = 0 ; i < n ; ++i) {
      tf::Vector3 pt = tf::Vector3(pcl_cloud.points[i].x, pcl_cloud.points[i].y, pcl_cloud.points[i].z);
      mask[i] = computeAttachedObjectPointMask(cm, pt, sensor_pos);
    }
  }
  return true;
}

int planning_environment::closestStateOnTrajectory(const boost::shared_ptr<urdf::Model> &model,
                                                   const trajectory_msgs::JointTrajectory &trajectory, 
                                                   const sensor_msgs::JointState &joint_state, 
                                                   unsigned int start, 
                                                   unsigned int end)
{
  double dist = 0.0;
  int    pos  = -1;
  
  std::map<std::string, double> current_state_map;
  std::map<std::string, bool> continuous;
  for(unsigned int i = 0; i < joint_state.name.size(); i++) {
    current_state_map[joint_state.name[i]] = joint_state.position[i];
  }

  for(unsigned int j = 0; j < trajectory.joint_names.size(); j++) {
    std::string name = trajectory.joint_names[j];
    boost::shared_ptr<const urdf::Joint> joint = model->getJoint(name);
    if (joint.get() == NULL)
    {
      ROS_ERROR("Joint name %s not found in urdf model", name.c_str());
      return false;
    }
    if (joint->type == urdf::Joint::CONTINUOUS) {
      continuous[name] = true;
    } else {
      continuous[name] = false;
    }

  }

  for (unsigned int i = start ; i <= end ; ++i)
  {
    double d = 0.0;
    for(unsigned int j = 0; j < trajectory.joint_names.size(); j++) {
      double diff; 
      if(!continuous[trajectory.joint_names[j]]) {
        diff = fabs(trajectory.points[i].positions[j] - current_state_map[trajectory.joint_names[j]]);
      } else {
        diff = angles::shortest_angular_distance(trajectory.points[i].positions[j],current_state_map[trajectory.joint_names[j]]);
      }
      d += diff * diff;
    }
	
    if (pos < 0 || d < dist)
    {
      pos = i;
      dist = d;
    }
  }    

  // if(pos == 0) {
  //   for(unsigned int i = 0; i < joint_state.name.size(); i++) {
  //     ROS_INFO_STREAM("Current state for joint " << joint_state.name[i] << " is " << joint_state.position[i]);
  //   }
  //   for(unsigned int j = 0; j < trajectory.joint_names.size(); j++) {
  //     ROS_INFO_STREAM("Trajectory zero has " << trajectory.points[0].positions[j] << " for joint " << trajectory.joint_names[j]);
  //   }
  // }

  return pos;
}

bool planning_environment::removeCompletedTrajectory(const boost::shared_ptr<urdf::Model> &model,
                                                     const trajectory_msgs::JointTrajectory &trajectory_in, 
                                                     const sensor_msgs::JointState& current_state, 
                                                     trajectory_msgs::JointTrajectory &trajectory_out, 
                                                     bool zero_vel_acc)
{

  trajectory_out = trajectory_in;
  trajectory_out.points.clear();

  if(trajectory_in.points.empty())
  {
    ROS_WARN("No points in input trajectory");
    return true;
  }
        
  int current_position_index = 0;        
  //Get closest state in given trajectory
  current_position_index = closestStateOnTrajectory(model,
                                                    trajectory_in, 
                                                    current_state, 
                                                    current_position_index, 
                                                    trajectory_in.points.size() - 1);
  if (current_position_index < 0)
  {
    ROS_ERROR("Unable to identify current state in trajectory");
    return false;
  } else {
    ROS_DEBUG_STREAM("Closest state is " << current_position_index << " of " << trajectory_in.points.size());
  }
    
  // Start one ahead of returned closest state index to make sure first trajectory point is not behind current state
  for(unsigned int i = current_position_index+1; i < trajectory_in.points.size(); ++i)
  {
    trajectory_out.points.push_back(trajectory_in.points[i]);
  }

  if(trajectory_out.points.empty())
  {
    ROS_DEBUG("No points in output trajectory");
    return false;
  }	

  ros::Duration first_time = trajectory_out.points[0].time_from_start;

  if(first_time < ros::Duration(.1)) {
    first_time = ros::Duration(0.0);
  } else {
    first_time -= ros::Duration(.1);
  }

  for(unsigned int i=0; i < trajectory_out.points.size(); ++i)
  {
    if(trajectory_out.points[i].time_from_start > first_time) {
      trajectory_out.points[i].time_from_start -= first_time;
    } else {
      ROS_INFO_STREAM("Not enough time in time from start for trajectory point " << i);
    }
  }

  if(zero_vel_acc) {
    for(unsigned int i=0; i < trajectory_out.joint_names.size(); ++i) {
      for(unsigned int j=0; j < trajectory_out.points.size(); ++j) {
        trajectory_out.points[j].velocities[i] = 0;
        trajectory_out.points[j].accelerations[i] = 0;
      }
    }
  }
  return true;
}

