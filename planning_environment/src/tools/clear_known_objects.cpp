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

/** \author Ioan Sucan */

/**

   @b ClearKnownObjects is a node that removes known objects from a
   collision map.

**/

#include <ros/ros.h>
#include "planning_environment/models/collision_models.h"
#include "planning_environment/util/construct_object.h"

#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <arm_navigation_msgs/AttachedCollisionObject.h>
#include <arm_navigation_msgs/CollisionObject.h>

class ClearKnownObjects
{
public:

  ClearKnownObjects(void): nh_("~")
  {    
    cm_ = new planning_environment::CollisionModels("robot_description");
    ROS_INFO("Starting");
    if (cm_->loadedModels())
    {
      fixed_frame_ = cm_->getWorldFrameId();

      nh_.param<std::string>("sensor_frame", sensor_frame_, std::string());
      nh_.param<bool>("filter_static_objects", filter_static_objects_, false);

      cloudPublisher_ = root_handle_.advertise<sensor_msgs::PointCloud>("cloud_out", 1);	    
      collisionObjectSubscriber_ = new message_filters::Subscriber<arm_navigation_msgs::CollisionObject>(root_handle_, "collision_object", 1024);
      collisionObjectFilter_ = new tf::MessageFilter<arm_navigation_msgs::CollisionObject>(*collisionObjectSubscriber_, *tf_, cm_->getWorldFrameId(), 1024);
      collisionObjectFilter_->registerCallback(boost::bind(&CollisionSpaceMonitor::collisionObjectCallback, this, _1));
      ROS_DEBUG("Listening to object_in_map using message notifier with target frame %s", collisionObjectFilter_->getTargetFramesString().c_str());
      
      //using regular message filter as there's no header
      attachedCollisionObjectSubscriber_ = new message_filters::Subscriber<arm_navigation_msgs::AttachedCollisionObject>(root_handle_, "attached_collision_object", 1024);	
      attachedCollisionObjectSubscriber_->registerCallback(boost::bind(&CollisionSpaceMonitor::attachObjectCallback, this, _1));    



      kmsm_->setOnAfterAttachCollisionObjectCallback(boost::bind(&ClearKnownObjects::attachObjectEvent, this, _1));
      if(filter_static_objects_) {
        kmsm_->setOnAfterCollisionObjectCallback(boost::bind(&ClearKnownObjects::objectUpdateEvent, this, _1));
      }            

      cloudSubscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud>(root_handle_, "cloud_in", 1);
      cloudFilter_ = new tf::MessageFilter<sensor_msgs::PointCloud>(*cloudSubscriber_, tf_, fixed_frame_, 1);
      cloudFilter_->registerCallback(boost::bind(&ClearKnownObjects::cloudCallback, this, _1));
    }
    else
    {
      kmsm_ = NULL;
      cloudFilter_ = NULL;
      cloudSubscriber_ = NULL;
    }
  }

  ~ClearKnownObjects(void)
  {
    ROS_INFO("Destructor getting called");
    if (cloudFilter_)
      delete cloudFilter_;
    if (cloudSubscriber_)
      delete cloudSubscriber_;
    if (kmsm_)
      delete kmsm_;
    if (rm_)
      delete rm_;
    for (std::map<std::string, KnownObject*>::iterator it = attached_objects_.begin(); it != attached_objects_.end(); it++) {
      delete it->second;
    }
    for (std::map<std::string, KnownObject*>::iterator it = collisionObjects_.begin() ; it != collisionObjects_.end() ; ++it)
      delete it->second;
  }
    
  void run(void)
  {   
    if (rm_->loadedModels())
    {
      kmsm_->waitForState();
    }
  }
    
private:

  struct KnownObject
  {
    KnownObject(void)
    {
    }

    ~KnownObject(void) {
      deleteBodies();
    }

    void deleteBodies() {
      for(unsigned int i = 0; i < bodies.size(); i++) {
        delete bodies[i];
      }
      bodies.clear();
    }

    void addBodyWithPose(bodies::Body* body, const btTransform &pose) {
      body->setPose(pose);
      bodies::BoundingSphere bsphere;
      body->computeBoundingSphere(bsphere);
      bodies.push_back(body);
      double rsquare = bsphere.radius * bsphere.radius;
      bspheres.push_back(bsphere);
      rsquares.push_back(rsquare);
    }
      
    void usePose(const unsigned int i, const btTransform &pose) {
      if(i >= bodies.size()) {
        ROS_WARN("Not enough bodies");
        return;
      }
      bodies[i]->setPose(pose);
      bodies[i]->computeBoundingSphere(bspheres[i]);
      rsquares[i] = bspheres[i].radius*bspheres[i].radius;
    }
	
    std::vector<bodies::Body*> bodies;
    std::vector<bodies::BoundingSphere>  bspheres;
    std::vector<double>                  rsquares;
  };
    
  void computeMask(const sensor_msgs::PointCloud &cloud, std::vector<int> &mask)
  {
    planning_models::KinematicState state(kmsm_->getKinematicModel());

    kmsm_->setKinematicStateToTime(cloud.header.stamp, state);

    updateObjects_.lock();

    // check if we have attached bodies
    if (attached_.size() > 0) {
      // update the poses for the attached bodies
      if (fixed_frame_ != kmsm_->getWorldFrameId()) {
        std::string errStr;
        ros::Time tm;
        if (!tf_.getLatestCommonTime(kmsm_->getWorldFrameId(), fixed_frame_, tm, &errStr) == tf::NO_ERROR) {
          ROS_ERROR("Unable to transform attached body from frame '%s' to frame '%s'", kmsm_->getWorldFrameId().c_str(), fixed_frame_.c_str());		
          if (!errStr.empty())
            ROS_ERROR("TF said: %s", errStr.c_str());
        } else {
          bool error = false;
          for (unsigned int i = 0 ; i < attached_.size() ; ++i) {
            const planning_models::KinematicState::AttachedBodyState* att_state = state.getAttachedBodyState(attached_[i]->getName());
            for(unsigned int j = 0; j < att_state->getGlobalCollisionBodyTransforms().size(); j++) {
              tf::Stamped<tf::Pose> pose(att_state->getGlobalCollisionBodyTransforms()[j], tm, kmsm_->getWorldFrameId());
              tf::Stamped<tf::Pose> poseOut;
              try {
                tf_.transformPose(fixed_frame_, pose, poseOut);
                attached_objects_[attached_[i]->getName()]->usePose(j,poseOut);
              } catch(...) {
                error = true;
              }
            }
            if (error)
              ROS_ERROR("Errors encountered when transforming attached bodies from frame '%s' to frame '%s'", kmsm_->getWorldFrameId().c_str(), fixed_frame_.c_str());
          }
        }
      } else {
        for (unsigned int i = 0 ; i < attached_.size() ; ++i) {
          const planning_models::KinematicState::AttachedBodyState* att_state = state.getAttachedBodyState(attached_[i]->getName());
          for(unsigned int j = 0; j < att_state->getGlobalCollisionBodyTransforms().size(); j++) {
            attached_objects_[attached_[i]->getName()]->usePose(j,att_state->getGlobalCollisionBodyTransforms()[j]);
          }
        }
      }
    }
	
    // transform pointcloud into fixed frame, if needed
    sensor_msgs::PointCloud temp;
    const sensor_msgs::PointCloud *cloudTransf = &cloud;
    if (fixed_frame_ != cloud.header.frame_id) {
      tf_.transformPointCloud(fixed_frame_, cloud, temp);
      cloudTransf = &temp;
    }
	
    btVector3 sensor_pos(0, 0, 0);
      
    // compute the origin of the sensor in the frame of the cloud
    if (!sensor_frame_.empty()) {
      ros::Time tm;
      std::string err;
      if (tf_.getLatestCommonTime(sensor_frame_.c_str(), fixed_frame_.c_str(), tm, &err) == tf::NO_ERROR) {
        try {
          tf::StampedTransform transf;
          tf_.lookupTransform(fixed_frame_, sensor_frame_, tm, transf);
          sensor_pos = transf.getOrigin();
        } catch(tf::TransformException& ex) {
          ROS_ERROR("Unable to lookup transform from %s to %s. Exception: %s", sensor_frame_.c_str(), fixed_frame_.c_str(), ex.what());
          sensor_pos.setValue(0, 0, 0);
        }
      } else {
        ROS_WARN("No common time between %s and %s", sensor_frame_.c_str(), fixed_frame_.c_str());
        sensor_pos.setValue(0, 0, 0);
      }
    }
      
    // compute mask for cloud
    int n = cloud.points.size();
    mask.resize(n);
      
    //#pragma omp parallel for
    for (int i = 0 ; i < n ; ++i) {
      btVector3 pt = btVector3(cloudTransf->points[i].x, cloudTransf->points[i].y, cloudTransf->points[i].z);
      btVector3 dir(sensor_pos - pt);
      dir.normalize();
      int out = 1;
	    
      for(std::map<std::string, KnownObject*>::iterator it = attached_objects_.begin();
          out && it != attached_objects_.end();
          it++) {
        for(unsigned int k = 0; out && k < it->second->bodies.size(); k++) {
          if (it->second->bspheres[k].center.distance2(pt) < it->second->rsquares[k]) {
            if (it->second->bodies[k]->containsPoint(pt) || it->second->bodies[k]->intersectsRay(pt, dir)) {
              out = 0;
            }
          }
        }
      }
      for (std::map<std::string, KnownObject*>::iterator it = collisionObjects_.begin() ; out && it != collisionObjects_.end() ; ++it) {
        ROS_INFO("Have known objects");
        KnownObject* ko = it->second;
        for (unsigned int j = 0 ; out && j < ko->bodies.size(); ++j) {
          if(ko->bodies.size() != ko->bspheres.size() ||
             ko->bodies.size() != ko->rsquares.size()) {
            ROS_ERROR_STREAM("Bad sizes " << ko->bodies.size() << " " << ko->bspheres.size()
                             << " " << ko->rsquares.size());
            continue;
          }
          if (ko->bspheres[j].center.distance2(pt) < ko->rsquares[j]) {
            if(ko->bodies[j]->containsPoint(pt)) {
              out = 0;
            }
            if (ko->bodies[j]->intersectsRay(pt, dir)) {
              out = 0;
            }
          }
        }
      }
      mask[i] = out;
    }
  }
    
  void cloudCallback(const sensor_msgs::PointCloudConstPtr &cloud)
  {
    ROS_DEBUG("Got pointcloud that is %f seconds old", (ros::Time::now() - cloud->header.stamp).toSec());
	
    std::vector<int> mask;
    bool filter = false;
	
    if  (attached_objects_.size() > 0 || collisionObjects_.size() > 0)
    {
      computeMask(*cloud, mask);
      filter = true;
    }
	
    if (filter)
    {
      // publish new cloud
      const unsigned int np = cloud->points.size();
      sensor_msgs::PointCloud data_out;
	    
      // fill in output data with points that are NOT in the known objects
      data_out.header = cloud->header;	  
	    
      data_out.points.resize(0);
      data_out.points.reserve(np);
	    
      data_out.channels.resize(cloud->channels.size());
      for (unsigned int i = 0 ; i < data_out.channels.size() ; ++i)
      {
        ROS_ASSERT(cloud->channels[i].values.size() == cloud->points.size());
        data_out.channels[i].name = cloud->channels[i].name;
        data_out.channels[i].values.reserve(cloud->channels[i].values.size());
      }
	    
      for (unsigned int i = 0 ; i < np ; ++i)
        if (mask[i])
        {
          data_out.points.push_back(cloud->points[i]);
          for (unsigned int j = 0 ; j < data_out.channels.size() ; ++j)
            data_out.channels[j].values.push_back(cloud->channels[j].values[i]);
        }

      ROS_DEBUG("Published filtered cloud (%d points out of %d)", (int)data_out.points.size(), (int)cloud->points.size());
      cloudPublisher_.publish(data_out);
    }
    else
    {
      cloudPublisher_.publish(*cloud);
      ROS_DEBUG("Republished unchanged cloud");
    }
    updateObjects_.unlock();
  }
       
  void objectUpdateEvent(const arm_navigation_msgs::CollisionObjectConstPtr &collisionObject) {
    updateObjects_.lock();
    if (collisionObject->operation.operation == arm_navigation_msgs::CollisionObjectOperation::ADD) {
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
            tf_.transformPose(fixed_frame_, psi, pso);
          } catch(tf::TransformException& ex) {
            ROS_ERROR("Unable to transform object '%s' in frame '%s' to frame '%s' Exception: %s", collisionObject->id.c_str(), collisionObject->header.frame_id.c_str(), fixed_frame_.c_str(), ex.what());
            err = true;
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
    updateObjects_.unlock();
  }

  void attachObjectEvent(const arm_navigation_msgs::AttachedCollisionObjectConstPtr& attachedObject) {
    ROS_INFO_STREAM("Calling attach object for object " << attachedObject->object.id << " operation " << attachedObject->object.operation.operation);
    updateObjects_.lock();
    if(attachedObject->object.operation.operation != arm_navigation_msgs::CollisionObjectOperation::REMOVE) {
      //assumes stuff is up to date
      if(attachedObject->object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT) {
        if(attached_objects_.find(attachedObject->object.id) == attached_objects_.end()) {
          ROS_WARN_STREAM("No attached body " << attachedObject->object.id << " to detach");
        } else {
          if(filter_static_objects_) {
            if(collisionObjects_.find(attachedObject->object.id) != collisionObjects_.end()) {
              ROS_WARN("Already have object in objects in map");
              delete collisionObjects_[attachedObject->object.id];
            }
            collisionObjects_[attachedObject->object.id] = attached_objects_[attachedObject->object.id];
          }
          attached_objects_.erase(attachedObject->object.id);
          ROS_INFO_STREAM("Clear objects adding attached object " << attachedObject->object.id << " back to objects");
        }
      } else if(attachedObject->object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT) {
        if(filter_static_objects_) {
          if(collisionObjects_.find(attachedObject->object.id) == collisionObjects_.end()) {
            ROS_WARN_STREAM("No object " << attachedObject->object.id << " to attach");
          } else {
            ROS_INFO_STREAM("Adding object " << attachedObject->object.id << " to attached objects");
            delete collisionObjects_[attachedObject->object.id];
            collisionObjects_.erase(attachedObject->object.id);
          }
        }
        //rest of this handled by following code
      }
    }
    //rest of stuff can just happen
    attached_.clear();
    const std::vector<planning_models::KinematicModel::LinkModel*>& links = kmsm_->getKinematicModel()->getLinkModels();
    for (unsigned int i = 0 ; i < links.size() ; ++i) {
      for (unsigned int j = 0 ; j < links[i]->getAttachedBodyModels().size() ; ++j) {
        attached_.push_back(links[i]->getAttachedBodyModels()[j]);
      }
    }
    for(std::map<std::string, KnownObject*>::iterator it = attached_objects_.begin();
        it != attached_objects_.end();
        it++) {
      delete it->second;
    }
    attached_objects_.clear();
    for (unsigned int i = 0; i < attached_.size(); ++i) {
      KnownObject* kb = new KnownObject();
      for(unsigned int j = 0; j < attached_[i]->getShapes().size(); j++) {
        bodies::Body* body = bodies::createBodyFromShape(attached_[i]->getShapes()[j]);
        body->setScale(scale_);
        body->setPadding(padd_);
        btTransform ident;
        ident.setIdentity();
        kb->addBodyWithPose(body, ident);
      }
      attached_objects_[attached_[i]->getName()] = kb;
    }
    updateObjects_.unlock();
  }
    
  ros::NodeHandle nh_;
  ros::NodeHandle root_handle_;
  tf::TransformListener tf_;
  planning_environment::CollisionModels *rm_;
  planning_environment::CollisionSpaceMonitor *kmsm_;

  message_filters::Subscriber<sensor_msgs::PointCloud> *cloudSubscriber_;
  tf::MessageFilter<sensor_msgs::PointCloud> *cloudFilter_;

  std::string fixed_frame_;
  boost::mutex updateObjects_;
  ros::Publisher cloudPublisher_;    

  std::string sensor_frame_;
  double scale_;
  double padd_;
  std::vector<const planning_models::KinematicModel::AttachedBodyModel*>  attached_;
  std::map<std::string, KnownObject*> attached_objects_;
  std::map<std::string, KnownObject*> collisionObjects_;
  bool filter_static_objects_;
};

   
int main(int argc, char **argv)
{
  ros::init(argc, argv, "clear_known_objects");

  ros::AsyncSpinner spinner(2); // Use 2 threads
    spinner.start();

    ClearKnownObjects cko;
    cko.run();

    ros::waitForShutdown();
    
    return 0;
}
