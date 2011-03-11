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
#include <planning_environment/models/collision_models.h>
#include <planning_environment/util/construct_object.h>
#include <planning_environment/monitors/monitor_utils.h>

#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <mapping_msgs/AttachedCollisionObject.h>

class FilterAttachedObjects
{
public:

  FilterAttachedObjects(void): priv_handle_("~")
  {    
    cm_ = new planning_environment::CollisionModels("robot_description");
    fixed_frame_ = cm_->getWorldFrameId();
    
    priv_handle_.param<std::string>("sensor_frame", sensor_frame_, std::string());
    
    cloud_publisher_ = root_handle_.advertise<sensor_msgs::PointCloud>("cloud_out", 1);	    
    attached_collision_object_subscriber_ = new message_filters::Subscriber<mapping_msgs::AttachedCollisionObject>(root_handle_, "attached_collision_object", 1024);	
    attached_collision_object_subscriber_->registerCallback(boost::bind(&FilterAttachedObjects::attachedObjectCallback, this, _1));    
    
    cloud_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud>(root_handle_, "cloud_in", 1);
    cloud_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud>(*cloud_subscriber_, tf_, fixed_frame_, 1);
    cloud_filter_->registerCallback(boost::bind(&FilterAttachedObjects::cloudCallback, this, _1));

    attached_color_.a = 0.5;
    attached_color_.r = 0.6;
    attached_color_.g = 0.4;
    attached_color_.b = 0.3;

    vis_marker_publisher_ = root_handle_.advertise<visualization_msgs::Marker>("filter_attached", 128);
    vis_marker_array_publisher_ = root_handle_.advertise<visualization_msgs::MarkerArray>(std::string("filter_attached")+"_array", 128);
    
  }

  ~FilterAttachedObjects(void)
  {
    delete cloud_filter_;
    delete cloud_subscriber_;
    delete attached_collision_object_subscriber_;
    delete cm_;
  }
    
  bool computeMask(const sensor_msgs::PointCloud &cloud, std::vector<int> &mask)
  {
    // compute mask for cloud
    int n = cloud.points.size();
    mask.resize(n, 1);

    cm_->bodiesLock();
    
    const std::map<std::string, std::map<std::string, bodies::BodyVector*> >& link_att_objects = cm_->getLinkAttachedObjects();

    if(link_att_objects.empty()) {
      cm_->bodiesUnlock();
      return false;
    }

    planning_models::KinematicState state(cm_->getKinematicModel());
    state.setKinematicStateToDefault();

    //this gets all the attached bodies in their correct current positions according to tf
    geometry_msgs::PoseStamped ps;
    ps.pose.orientation.w = 1.0;
    for(std::map<std::string, std::map<std::string, bodies::BodyVector*> >::const_iterator it = link_att_objects.begin();
        it != link_att_objects.end();
        it++) {
      ps.header.frame_id = it->first;
      std::string es;
      if (tf_.getLatestCommonTime(fixed_frame_, it->first, ps.header.stamp, &es) != tf::NO_ERROR) {
        ROS_INFO_STREAM("Problem transforming into fixed frame from " << it->first << ".  Error string " << es);
        continue;
      }
      geometry_msgs::PoseStamped psout;
      tf_.transformPose(fixed_frame_, ps, psout);
      btTransform link_trans;
      tf::poseMsgToTF(psout.pose, link_trans);
      state.updateKinematicStateWithLinkAt(it->first, link_trans);
      cm_->updateAttachedBodyPosesForLink(state, it->first);
    }

    visualization_msgs::MarkerArray arr;
    cm_->getAttachedCollisionObjectMarkers(state,
                                           arr,
                                           "filter_attached",
                                           attached_color_,
                                           ros::Duration(.2));
	
    vis_marker_array_publisher_.publish(arr);

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
            
    //#pragma omp parallel for
    unsigned int filter_count = 0;    

    for (int i = 0 ; i < n ; ++i) {
      btVector3 pt = btVector3(cloudTransf->points[i].x, cloudTransf->points[i].y, cloudTransf->points[i].z);
      btVector3 dir(sensor_pos - pt);
      dir.normalize();
      int out = 1;

      for(std::map<std::string, std::map<std::string, bodies::BodyVector*> >::const_iterator it = link_att_objects.begin();
          out && it != link_att_objects.end();
          it++) {
        for(std::map<std::string, bodies::BodyVector*>::const_iterator it2 = it->second.begin();
            out && it2 != it->second.end();
            it2++) {
          for(unsigned int k = 0; out && k < it2->second->getSize(); k++) {
            //ROS_INFO_STREAM("Sphere distance " << it2->second->getBoundingSphere(k).center.distance2(pt)
            //                << " squared " << it2->second->getBoundingSphereRadiusSquared(k));
            if(it2->second->getBoundingSphere(k).center.distance2(pt) < it2->second->getBoundingSphereRadiusSquared(k)) {
              if(it2->second->getBody(k)->containsPoint(pt) || it2->second->getBody(k)->intersectsRay(pt, dir)) {
                out = 0;
                filter_count++;
              }
            }
          }
        }
      }
      mask[i] = out;
    }
    cm_->bodiesUnlock();
    return true;
  }
    
  void cloudCallback(const sensor_msgs::PointCloudConstPtr &cloud)
  {
    ROS_DEBUG("Got pointcloud that is %f seconds old", (ros::Time::now() - cloud->header.stamp).toSec());
	
    std::vector<int> mask;
    if(computeMask(*cloud, mask)) {
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
      cloud_publisher_.publish(data_out);
    }
    else
    {
      cloud_publisher_.publish(*cloud);
      ROS_DEBUG("Republished unchanged cloud");
    }
  }
       
  void attachedObjectCallback(const mapping_msgs::AttachedCollisionObjectConstPtr& attached_object) {
    planning_environment::processAttachedCollisionObjectMsg(attached_object, tf_, cm_);
  }
    
  ros::NodeHandle priv_handle_;
  ros::NodeHandle root_handle_;
  tf::TransformListener tf_;
  planning_environment::CollisionModels *cm_;

  message_filters::Subscriber<sensor_msgs::PointCloud> *cloud_subscriber_;
  tf::MessageFilter<sensor_msgs::PointCloud> *cloud_filter_;

  message_filters::Subscriber<mapping_msgs::AttachedCollisionObject> *attached_collision_object_subscriber_;

  std::string fixed_frame_;
  ros::Publisher cloud_publisher_;    

  std::string sensor_frame_;

  ros::Publisher vis_marker_publisher_;
  ros::Publisher vis_marker_array_publisher_;
  std_msgs::ColorRGBA attached_color_;
};

   
int main(int argc, char **argv)
{
  ros::init(argc, argv, "clear_known_objects");

  ros::AsyncSpinner spinner(2); // Use 2 threads
  spinner.start();
  
  FilterAttachedObjects cko;
  ros::waitForShutdown();
  
  return 0;
}
