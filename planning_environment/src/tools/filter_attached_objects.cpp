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
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <arm_navigation_msgs/AttachedCollisionObject.h>
#include <robot_self_filter/self_mask.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

class FilterAttachedObjects
{
public:

  FilterAttachedObjects(void): priv_handle_("~")
  {    
    cm_ = new planning_environment::CollisionModels("robot_description");
    priv_handle_.param<std::string>("sensor_frame", sensor_frame_, std::string());
    
    cloud_publisher_ = root_handle_.advertise<sensor_msgs::PointCloud2>("cloud_out", 1);	    
    cloud_publisher_shadow_ = root_handle_.advertise<sensor_msgs::PointCloud2>("cloud_out_shadow", 1);	    
    attached_collision_object_subscriber_ = new message_filters::Subscriber<arm_navigation_msgs::AttachedCollisionObject>(root_handle_, "attached_collision_object", 1024);	
    attached_collision_object_subscriber_->registerCallback(boost::bind(&FilterAttachedObjects::attachedObjectCallback, this, _1));    

    collision_object_subscriber_ = new message_filters::Subscriber<arm_navigation_msgs::CollisionObject>(root_handle_, "collision_object", 1024);	
    collision_object_subscriber_->registerCallback(boost::bind(&FilterAttachedObjects::objectCallback, this, _1));    
    
    cloud_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(root_handle_, "cloud_in", 1);
    cloud_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(*cloud_subscriber_, tf_, cm_->getWorldFrameId(), 1);
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
    delete collision_object_subscriber_;
    delete cm_;
  }
        
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud)
  {
    ROS_DEBUG("Got pointcloud that is %f seconds old", (ros::Time::now() - cloud->header.stamp).toSec());
    
    {
      planning_models::KinematicState state(cm_->getKinematicModel());
      state.setKinematicStateToDefault();

      visualization_msgs::MarkerArray arr;      
      planning_environment::updateAttachedObjectBodyPoses(cm_,
                                                          state,
                                                          tf_);
      
      cm_->getAttachedCollisionObjectMarkers(state,
                                             arr,
                                             "filter_attached",
                                             attached_color_,
                                             ros::Duration(.2));

      std_msgs::ColorRGBA static_color;
      static_color.a = 0.5;
      static_color.r = 0.0;
      static_color.g = 1.0;
      static_color.b = 0.3;
      
      cm_->getStaticCollisionObjectMarkers(arr,
                                           "filter_attached",
                                           static_color,
                                           ros::Duration(.2));

      vis_marker_array_publisher_.publish(arr);
    }
      
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*cloud, pcl_cloud);

    std::vector<int> mask;

    if(planning_environment::computeAttachedObjectPointCloudMask(pcl_cloud, 
                                                                 sensor_frame_,
                                                                 cm_,
                                                                 tf_,
                                                                 mask)) {
      // publish new cloud
      const unsigned int np = pcl_cloud.size();
            
      pcl::PointCloud<pcl::PointXYZ> inside_masked_cloud;
      pcl::PointCloud<pcl::PointXYZ> shadow_cloud;
      
      inside_masked_cloud.header = pcl_cloud.header;
      shadow_cloud.header = pcl_cloud.header;
      
      inside_masked_cloud.points.reserve(np);
      shadow_cloud.points.reserve(np);

      for (unsigned int i = 0; i < np; ++i) {
        if(mask[i] != robot_self_filter::INSIDE) {
          inside_masked_cloud.points.push_back(pcl_cloud.points[i]);
        } 
        if(mask[i] == robot_self_filter::SHADOW) {
          shadow_cloud.points.push_back(pcl_cloud.points[i]);
        }
      }
      sensor_msgs::PointCloud2 out;
      pcl::toROSMsg(inside_masked_cloud, out);
      cloud_publisher_.publish(out);
      
      sensor_msgs::PointCloud2 out_shadow;
      pcl::toROSMsg(shadow_cloud, out_shadow);
      cloud_publisher_shadow_.publish(out_shadow);
    } else {
      sensor_msgs::PointCloud2 out;
      pcl::toROSMsg(pcl_cloud, out);
      cloud_publisher_.publish(out);

      pcl::PointCloud<pcl::PointXYZ> empty_cloud;
      empty_cloud.header = pcl_cloud.header;
      sensor_msgs::PointCloud2 out_shadow;
      pcl::toROSMsg(empty_cloud, out_shadow);
      cloud_publisher_shadow_.publish(out_shadow);
    }
  }
       
  void objectCallback(const arm_navigation_msgs::CollisionObjectConstPtr& object) {
    planning_environment::processCollisionObjectMsg(object, tf_, cm_);
  }
  
  void attachedObjectCallback(const arm_navigation_msgs::AttachedCollisionObjectConstPtr& attached_object) {
    planning_environment::processAttachedCollisionObjectMsg(attached_object, tf_, cm_);
  }
    
  ros::NodeHandle priv_handle_;
  ros::NodeHandle root_handle_;
  tf::TransformListener tf_;
  planning_environment::CollisionModels *cm_;

  message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_subscriber_;
  tf::MessageFilter<sensor_msgs::PointCloud2> *cloud_filter_;

  message_filters::Subscriber<arm_navigation_msgs::AttachedCollisionObject> *attached_collision_object_subscriber_;
  message_filters::Subscriber<arm_navigation_msgs::CollisionObject> *collision_object_subscriber_;

  ros::Publisher cloud_publisher_;    
  ros::Publisher cloud_publisher_shadow_;    

  std::string sensor_frame_;

  ros::Publisher vis_marker_publisher_;
  ros::Publisher vis_marker_array_publisher_;
  std_msgs::ColorRGBA attached_color_;
};

   
int main(int argc, char **argv)
{
  ros::init(argc, argv, "clear_known_objects");

  ros::AsyncSpinner spinner(1); // Use 2 threads
  spinner.start();
  
  FilterAttachedObjects cko;
  ros::waitForShutdown();
  
  return 0;
}
