/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *
 *  \author Adam Harmat, Kai M. Wurm
 *********************************************************************/

#include "collider/collider.h"
#include <planning_environment/monitors/monitor_utils.h>


Collider::Collider(): root_handle_(""), pruning_counter_(0), transparent_freespace_(false) {
   
  ros::NodeHandle priv("~");

  cm_ = new planning_environment::CollisionModels("robot_description");
  fixed_frame_ = cm_->getWorldFrameId();

  attached_color_.a = 0.5;
  attached_color_.r = 0.6;
  attached_color_.g = 0.4;
  attached_color_.b = 0.3;
  
  priv.param<double>("resolution", resolution_, 0.1);
  priv.param<double>("max_range", max_range_, -1.0); // default: full beam length
  priv.param<int>("pruning_period", pruning_period_, 5);

  priv.param<bool>("publish_static_over_dynamic_map", publish_over_dynamic_map_, false);

  // bounds of collision map in fixed frame
  priv.param<double>("dimension_x", workspace_box_.dimensionX, 0.0);
  priv.param<double>("dimension_y", workspace_box_.dimensionY, 0.0);
  priv.param<double>("dimension_z", workspace_box_.dimensionZ, 0.0);

  // origin of collision map in the fixed frame
  priv.param<double>("origin_x", workspace_box_.originX, 0.0);
  priv.param<double>("origin_y", workspace_box_.originY, 0.0);
  priv.param<double>("origin_z", workspace_box_.originZ, 0.0);

  // compute some useful values
  workspace_box_.real_minX = -workspace_box_.dimensionX + workspace_box_.originX;
  workspace_box_.real_maxX =  workspace_box_.dimensionX + workspace_box_.originX;
  workspace_box_.real_minY = -workspace_box_.dimensionY + workspace_box_.originY;
  workspace_box_.real_maxY =  workspace_box_.dimensionY + workspace_box_.originY;
  workspace_box_.real_minZ = -workspace_box_.dimensionZ + workspace_box_.originZ;
  workspace_box_.real_maxZ =  workspace_box_.dimensionZ + workspace_box_.originZ;

  // stereo cam params for sensor cone:
  priv.param<int>("camera_stereo_offset_left", camera_stereo_offset_left_, 128);
  priv.param<int>("camera_stereo_offset_right", camera_stereo_offset_right_, 0);


  collision_octree_ = new OcTreeType(resolution_);
  double probHit = 0.7;
  double probMiss = 0.4;
  double thresMin = 0.12;
  double thresMax = 0.97;
  priv.param("sensor_model_hit", probHit, probHit);
  priv.param("sensor_model_miss", probMiss, probMiss);
  priv.param("sensor_model_min", thresMin, thresMin);
  priv.param("sensor_model_max", thresMax, thresMax);
  collision_octree_->setProbHit(probHit);
  collision_octree_->setProbMiss(probMiss);
  collision_octree_->setClampingThresMin(thresMin);
  collision_octree_->setClampingThresMax(thresMax);

  marker_pub_ = root_handle_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  octomap_visualization_pub_ = root_handle_.advertise<visualization_msgs::Marker>("occupied_cells", 10);
  octomap_visualization_free_pub_ = root_handle_.advertise<visualization_msgs::Marker>("free_cells", 10);
  octomap_visualization_attached_pub_ = root_handle_.advertise<visualization_msgs::Marker>("attached_objects", 10);
  octomap_visualization_attached_array_pub_ = root_handle_.advertise<visualization_msgs::MarkerArray>("attached_objects_array", 10);
  cmap_publisher_ = root_handle_.advertise<mapping_msgs::CollisionMap>("collision_map_out", 1, true);
  static_map_publisher_ = root_handle_.advertise<mapping_msgs::CollisionMap>("collision_map_occ_static", 1);
  pointcloud_publisher_ = root_handle_.advertise<sensor_msgs::PointCloud2>("point_cloud_out", 1, true);
  
  color_occupied_.r = 0;
  color_occupied_.g = 0;
  color_occupied_.b = 1.0;
  color_occupied_.a = 0.5;

  color_free_.r = 0;
  color_free_.g = 1.0;
  color_free_.b = 0.0;
  color_free_.a = 0.5;

  // create a self mask with links: (see self_see_filter.h)
  double default_padding, default_scale;
  priv.param<double> ("self_see_default_padding", default_padding, .01);
  priv.param<double> ("self_see_default_scale", default_scale, 1.0);
  priv.param<double> ("min_sensor_dist", self_filter_min_dist_, 0.05);

  std::vector<robot_self_filter::LinkInfo> links;
  if (!priv.hasParam ("self_see_links"))
  {
    ROS_WARN ("No links specified for self filtering.");
  }
  else
  {
    XmlRpc::XmlRpcValue ssl_vals;;

    priv.getParam ("self_see_links", ssl_vals);
    if (ssl_vals.getType () != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_WARN ("Self see links need to be an array");
    }
    else
    {
      if (ssl_vals.size () == 0)
      {
        ROS_WARN ("No values in self see links array");
      }
      else
      {
        for (int i = 0; i < ssl_vals.size (); ++i)
        {
          robot_self_filter::LinkInfo li;

          if (ssl_vals[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
          {
            ROS_WARN ("Self see links entry %d is not a structure.  Stopping processing of self see links", i);
            break;
          }
          if (!ssl_vals[i].hasMember ("name"))
          {
            ROS_WARN ("Self see links entry %d has no name.  Stopping processing of self see links", i);
            break;
          }
          li.name = std::string (ssl_vals[i]["name"]);
          if (!ssl_vals[i].hasMember ("padding"))
          {
            ROS_DEBUG ("Self see links entry %d has no padding.  Assuming default padding of %g", i, default_padding);
            li.padding = default_padding;
          }
          else
          {
            li.padding = ssl_vals[i]["padding"];
          }
          if (!ssl_vals[i].hasMember ("scale"))
          {
            ROS_DEBUG ("Self see links entry %d has no scale.  Assuming default scale of %g", i, default_scale);
            li.scale = default_scale;
          }
          else
          {
            li.scale = ssl_vals[i]["scale"];
          }
          links.push_back (li);
        }
      }
    }
  }


  robot_mask_right_ = new robot_self_filter::SelfMask(tf_, links);
  robot_mask_left_ = new robot_self_filter::SelfMask(tf_, links);
  ROS_INFO_STREAM("Robot self mask initialized with " << links.size() << " links");



  cam_model_initialized_ = false;
  camera_info_subscriber_ = new ros::Subscriber;
  std::string camera_info_topic;
  priv.getParam("camera_info_topic", camera_info_topic);
  (*camera_info_subscriber_) = root_handle_.subscribe( camera_info_topic , 10,
                                                       &Collider::cameraInfoCallback, this);



  if(!priv.hasParam("cloud_sources")) {
    //ROS_WARN("No sensor sources specified, please set config correctly (e.g., config/sources.yaml)");
    //return;
  } 
  else 
  {
    XmlRpc::XmlRpcValue cloud_sources;
    priv.getParam("cloud_sources", cloud_sources);

    if(cloud_sources.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_WARN("Cloud sources needs to be an array");
    } 
    else 
    {
      for(int i = 0; i < cloud_sources.size(); ++i)
      {
        CloudInfo cinfo;
        if(cloud_sources[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) 
        {
          ROS_WARN("Cloud source entry %d is not a structure.  Stopping processing of cloud sources",i);
          break;
        }

        if(!cloud_sources[i].hasMember("name")) 
        {
          ROS_WARN("Cloud sources entry %d has no name.  Stopping processing of cloud sources",i);
          break;
        } 

        if(cloud_sources[i].hasMember("raw_name"))
        {
          cinfo.raw_cloud_name_ = std::string(cloud_sources[i]["raw_name"]);

          if (!cloud_sources[i].hasMember("sensor_stereo_other_frame")){
            ROS_WARN("Cloud sources entry %d has a raw_name but no sensor_stereo_other_frame.  Stopping processing of cloud sources",i);
            break;
          } else{
            cinfo.sensor_stereo_other_frame_ = std::string(cloud_sources[i]["sensor_stereo_other_frame"]);
          }

        }

        if(!cloud_sources[i].hasMember("sensor_frame")) 
        {
          ROS_WARN("Cloud sources entry %d has no sensor_frame.  Stopping processing of cloud sources",i);
          break;
        } 

        cinfo.cloud_name_ = std::string(cloud_sources[i]["name"]);
        cinfo.sensor_frame_ = std::string(cloud_sources[i]["sensor_frame"]);

        if(!cloud_sources[i].hasMember("frame_subsample")) 
        {
          ROS_DEBUG("Cloud sources entry %d has no frame subsample.  Assuming default subsample of %g",i,1.0);
          cinfo.frame_subsample_ = 1.0;;
        } 
        else 
        {
          cinfo.frame_subsample_ = cloud_sources[i]["frame_subsample"];
        }

        if(!cloud_sources[i].hasMember("point_subsample")) 
        {
          ROS_DEBUG("Cloud sources entry %d has no point subsample.  Assuming default subsample of %g",i,1.0);
          cinfo.point_subsample_ = 1.0;;
        } 
        else 
        {
          cinfo.point_subsample_ = cloud_sources[i]["point_subsample"];
        }

        if(cloud_sources[i].hasMember("dynamic_buffer_size"))
          cinfo.dynamic_buffer_size_ = static_cast<unsigned int>(int(cloud_sources[i]["dynamic_buffer_size"]));
	
        if(cloud_sources[i].hasMember("static_buffer_size"))
          cinfo.static_buffer_size_ = static_cast<unsigned int>(int(cloud_sources[i]["static_buffer_size"]));
        
        if(cloud_sources[i].hasMember("dynamic_buffer_duration"))
          cinfo.dynamic_buffer_duration_ = ros::Duration(int(cloud_sources[i]["dynamic_buffer_duration"]));
        
        if(cloud_sources[i].hasMember("static_buffer_duration"))
          cinfo.static_buffer_duration_ = ros::Duration(int(cloud_sources[i]["static_buffer_duration"]));
        
        if(cloud_sources[i].hasMember("dynamic_publish"))
          cinfo.dynamic_publish_ = cloud_sources[i]["dynamic_publish"];
        
        if(cloud_sources[i].hasMember("static_publish"))
          cinfo.static_publish_ = cloud_sources[i]["static_publish"];
        
        if(cloud_sources[i].hasMember("dynamic_until_static_publish")) {
          cinfo.dynamic_until_static_publish_ = cloud_sources[i]["dynamic_until_static_publish"];
        }

        if(cloud_sources_.find(cinfo.cloud_name_) != cloud_sources_.end()) 
        {
          ROS_WARN_STREAM("Already have a cloud defined with name " << cinfo.cloud_name_ << ", using prior values");
        } 
        else 
        {
          message_filters::Subscriber<sensor_msgs::PointCloud2>* sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>();
          sub->subscribe(root_handle_, cinfo.cloud_name_, 3);
          sub_filtered_clouds_.push_back(sub);
          if(cinfo.raw_cloud_name_ != "") {
            message_filters::Subscriber<sensor_msgs::PointCloud2>* sub_raw = new message_filters::Subscriber<sensor_msgs::PointCloud2>();
            sub_raw->subscribe(root_handle_,cinfo.raw_cloud_name_, 3);
            message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> >* sync =
              new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> >(3);
            sub_raw_clouds_.push_back(sub_raw);            
            sync->connectInput (*sub, *sub_raw);
            sync->registerCallback(bind (&Collider::cloudCombinedCallback, this, _1, _2, cinfo.cloud_name_));
            sync_vector_.push_back(sync);
            ROS_INFO_STREAM("Adding synced callback for cloud " << cinfo.cloud_name_ << " raw " << cinfo.raw_cloud_name_);
          } else {
            sub->registerCallback(boost::bind(&Collider::cloudCallback, this, _1, cinfo.cloud_name_));
            ROS_INFO_STREAM("Adding callback for " << cinfo.cloud_name_);
          }
          cloud_sources_[cinfo.cloud_name_] = cinfo;
        }
      }
    }
  }

  attached_collision_object_subscriber_ = new message_filters::Subscriber<mapping_msgs::AttachedCollisionObject>(root_handle_, "attached_collision_object", 1024);	
  attached_collision_object_subscriber_->registerCallback(boost::bind(&Collider::attachedObjectCallback, this, _1));    

  reset_service_ = priv.advertiseService("reset", &Collider::reset, this);
  dummy_reset_service_ = priv.advertiseService("dummy_reset", &Collider::dummyReset, this);

  action_server_.reset( new actionlib::SimpleActionServer<collision_environment_msgs::MakeStaticCollisionMapAction>(root_handle_, "make_static_collision_map", boost::bind(&Collider::makeStaticCollisionMap, this, _1), false));
  action_server_->start();

  // queries on the map:
  get_octomap_service_ = root_handle_.advertiseService("octomap_binary", &Collider::octomapSrv, this);
  occupancy_point_service_ = root_handle_.advertiseService("occupancy_point", &Collider::occupancyPointSrv, this);
  occupancy_bbx_service_ = root_handle_.advertiseService("occupancy_in_bbx", &Collider::occupancyBBXSrv, this);
  occupancy_bbx_size_service_ = root_handle_.advertiseService("occupancy_in_bbx_size", &Collider::occupancyBBXSizeSrv, this);

}


Collider::~Collider() {

  delete collision_octree_;
  delete robot_mask_right_;
  delete robot_mask_left_;
  delete attached_collision_object_subscriber_;
  
  for(unsigned int i = 0; i < sub_filtered_clouds_.size(); ++i){
    delete sub_filtered_clouds_[i];
  }
  for(unsigned int i = 0; i < sub_raw_clouds_.size(); ++i) {
    delete sub_raw_clouds_[i];
  }
  for(unsigned int i = 0; i < sync_vector_.size(); ++i) {
    delete sync_vector_[i];
  }
}

void Collider::attachedObjectCallback(const mapping_msgs::AttachedCollisionObjectConstPtr& attached_object) {
  planning_environment::processAttachedCollisionObjectMsg(attached_object, tf_, cm_);
}

void Collider::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &cam_info){
  ROS_DEBUG("Got camera info: %d x %d\n", cam_info->height, cam_info->width);
  cam_model_.fromCameraInfo(*cam_info);
  cam_size_ = cam_model_.fullResolution();
  cam_model_initialized_ = true;
  delete camera_info_subscriber_;
}

void Collider::cloudCombinedCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud, 
                            const sensor_msgs::PointCloud2::ConstPtr &cloud_raw,
                            const std::string topic_name) {  

  CloudInfo settings =  cloud_sources_[topic_name];

  ros::WallTime begin_cb = ros::WallTime::now();

  if (!cam_model_initialized_) {
    ROS_INFO ("ERROR: camera model not initialized.");
    return;
  }

  // Use old callback with cloud (filtered)
  //  cloudCallback (cloud, "full_cloud_filtered");
  // For clearing use cloud_raw to check projections
  // (cloud_raw.{width,height})

  // transform pointcloud from sensor frame to fixed_frame_
  if (!tf_.waitForTransform(fixed_frame_, cloud_raw->header.frame_id, cloud->header.stamp, ros::Duration(1.0))) {
    ROS_WARN_STREAM( "Timed out waiting for transform from " << cloud_raw->header.frame_id
                     << " to " << fixed_frame_ << ", quitting callback");
    return;
  }

  // transform pointcloud from sensor frame to fixed_frame_
  if (!tf_.waitForTransform(fixed_frame_, cloud->header.frame_id, cloud->header.stamp, ros::Duration(1.0))) {
    ROS_WARN_STREAM( "Timed out waiting for transform from " << cloud->header.frame_id
                     << " to " << fixed_frame_ << ", quitting callback");
    return;
  }


  tf::StampedTransform trans;
  tf_.lookupTransform (fixed_frame_, cloud->header.frame_id, cloud->header.stamp, trans);
  tf::Transform to_world = trans;

  ros::WallTime begin_transform = ros::WallTime::now();
  Eigen::Matrix4f eigen_transform;
  sensor_msgs::PointCloud2 transformed_cloud;
  // //  cturtle:
//   pcl::transformAsMatrix (to_world, eigen_transform);
//   pcl::transformPointCloud (eigen_transform, *cloud, transformed_cloud);
  // //  dturtle:
  pcl_ros::transformAsMatrix (to_world, eigen_transform);
  pcl_ros::transformPointCloud (eigen_transform, *cloud, transformed_cloud);

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg (transformed_cloud, pcl_cloud);
  
  pcl_cloud.header.frame_id = cm_->getWorldFrameId();
  pcl_cloud.header.stamp = cloud->header.stamp;

  std::vector<int> inside_mask;
  //filtering out attached object inside points
  if(planning_environment::computeAttachedObjectPointCloudMask(pcl_cloud,
                                                               cm_->getWorldFrameId(),
                                                               cm_,
                                                               tf_,
                                                               inside_mask)) {
    pcl::PointCloud<pcl::PointXYZ>::iterator it = pcl_cloud.points.begin();
    unsigned int i = 0;
    unsigned int count = 0;
    while(it != pcl_cloud.points.end()) {
      if(inside_mask[i++] == robot_self_filter::INSIDE) {
        it = pcl_cloud.points.erase(it);
	count++;
      } else {
        it++;
      }
    }
    ROS_DEBUG_STREAM("Filtering " << count << " points");
  }

  {
    planning_models::KinematicState state(cm_->getKinematicModel());
    state.setKinematicStateToDefault();
    
    planning_environment::updateAttachedObjectBodyPoses(cm_,
                                                        state,
                                                        tf_);
    
    visualization_msgs::MarkerArray arr;
    cm_->getAttachedCollisionObjectMarkers(state,
                                           arr,
                                           "filter_attached",
                                           attached_color_,
                                           ros::Duration(.2));
    
    octomap_visualization_attached_array_pub_.publish(arr);
  }
  
  std_msgs::Header global_header = cloud->header;
  global_header.frame_id = fixed_frame_;

  // pcl::transformPointCloud (fixed_frame_, *cloud, transformed_cloud, tf_);
  // pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  // pcl::fromROSMsg (transformed_cloud, pcl_cloud);


  pcl::PointCloud<pcl::PointXYZ> pcl_cloud_raw;
  pcl::fromROSMsg (*cloud_raw, pcl_cloud_raw);

  // copy data to octomap pointcloud
  octomap::Pointcloud octo_pointcloud;
  octomap::pointcloudPCLToOctomap(pcl_cloud, octo_pointcloud);
  double elapsed_transform = (ros::WallTime::now() - begin_transform).toSec();


  octomap::point3d sensor_origin = getSensorOrigin(cloud->header);
  tf::Point sensor_origin_tf = octomap::pointOctomapToTf(sensor_origin);

  // integrate pointcloud into map
  ros::WallTime begin_insert = ros::WallTime::now();
  // TODO prune map or not?
  collision_octree_->insertScan(octo_pointcloud, sensor_origin, max_range_, false);
  double elapsed_insert = (ros::WallTime::now() - begin_insert).toSec();


  // remove outdated occupied nodes -----
  ros::WallTime begin_degrade = ros::WallTime::now();
  degradeOutdatedRaw(cloud->header, sensor_origin_tf, settings.sensor_stereo_other_frame_, pcl_cloud_raw);

  double elapsed_degrade = (ros::WallTime::now() - begin_degrade).toSec();


  // get all occupied nodes from map
  octomap::point3d_list node_centers;
  ros::WallTime begin_get_occupied = ros::WallTime::now();

  std::vector<geometry_msgs::Point> pointlist;
  getOccupiedPoints(pointlist);

  //collision_octree_->getOccupied(node_centers);
  double elapsed_get_occupied = (ros::WallTime::now() - begin_get_occupied).toSec();


  // publish occupied cells
  ros::WallTime begin_send = ros::WallTime::now();
  if (settings.dynamic_publish_ && cmap_publisher_.getNumSubscribers() > 0) {
    publishCollisionMap(pointlist, global_header, cmap_publisher_);
  }
  if (pointcloud_publisher_.getNumSubscribers() > 0) {
    publishPointCloud(pointlist, global_header, pointcloud_publisher_);
  }
  if (octomap_visualization_pub_.getNumSubscribers() > 0){
    publishMarkerArray(pointlist, global_header, color_occupied_, octomap_visualization_pub_);
  }
  if (octomap_visualization_free_pub_.getNumSubscribers() > 0){
	  std::vector<geometry_msgs::Point> freelist;
	  getFreePoints(freelist);
	  publishMarkerArray(freelist, global_header, color_free_, octomap_visualization_free_pub_);
  }
  double elapsed_send = (ros::WallTime::now() - begin_send).toSec();

  double total_elapsed = (ros::WallTime::now() - begin_cb).toSec();
  ROS_DEBUG("Total cloudCombinedCB %d pts: %f (transf: %f, update: %f, clear: %f, get occ: %f, send map: %f)",
		  int(pcl_cloud.size()), total_elapsed, elapsed_transform, elapsed_insert, elapsed_degrade, elapsed_get_occupied, elapsed_send);

}


void Collider::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud, const std::string topic_name) {

  ros::WallTime begin_cb = ros::WallTime::now();
  CloudInfo settings =  cloud_sources_[topic_name];
  
  // transform pointcloud from sensor frame to fixed_frame_
  if (!tf_.waitForTransform(fixed_frame_, cloud->header.frame_id, cloud->header.stamp, ros::Duration(1.0))) {
    ROS_WARN_STREAM( "Timed out waiting for transform from " << cloud->header.frame_id
                     << " to " << fixed_frame_ << ", quitting callback");
    return;
  }

  // sensor_msgs::PointCloud2 transformed_cloud;
  // pcl::transformPointCloud (fixed_frame_, *cloud, transformed_cloud, tf_);
  // // tf::StampedTransform transform;
  // // tf_.lookupTransform (fixed_frame_, cloud->header.frame_id, cloud->header.stamp, transform);
  // // pcl::transformPointCloud (*cloud, transformed_cloud, transform);
  // pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  // pcl::fromROSMsg (transformed_cloud, pcl_cloud);



  tf::StampedTransform trans;
  tf_.lookupTransform (fixed_frame_, cloud->header.frame_id,  cloud->header.stamp, trans);
  tf::Transform to_world = trans;
  Eigen::Matrix4f eigen_transform;
  sensor_msgs::PointCloud2 transformed_cloud;
  // //  cturtle:
//   pcl::transformAsMatrix (to_world, eigen_transform);
//   pcl::transformPointCloud (eigen_transform, *cloud, transformed_cloud);
  // //  dturtle:
  pcl_ros::transformAsMatrix (to_world, eigen_transform);
  pcl_ros::transformPointCloud (eigen_transform, *cloud, transformed_cloud);

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg (transformed_cloud, pcl_cloud);
  transformed_cloud.header.frame_id = fixed_frame_;

  std::vector<int> inside_mask;
  //filtering out attached object inside points
  if(planning_environment::computeAttachedObjectPointCloudMask(pcl_cloud,
                                                               cm_->getWorldFrameId(),
                                                               cm_,
                                                               tf_,
                                                               inside_mask)) {
    pcl::PointCloud<pcl::PointXYZ>::iterator it = pcl_cloud.points.begin();
    unsigned int i = 0;
    while(it != pcl_cloud.end()) {
      if(inside_mask[i++] == robot_self_filter::INSIDE) {
        it = pcl_cloud.points.erase(it);
      } else {
        it++;
      }
    }
  }

  {
    planning_models::KinematicState state(cm_->getKinematicModel());
    state.setKinematicStateToDefault();
    
    planning_environment::updateAttachedObjectBodyPoses(cm_,
                                                        state,
                                                        tf_);
    
    visualization_msgs::MarkerArray arr;
    cm_->getAttachedCollisionObjectMarkers(state,
                                           arr,
                                           "filter_attached",
                                           attached_color_,
                                           ros::Duration(.2));
    
    octomap_visualization_attached_array_pub_.publish(arr);
  }

  // copy data to octomap pointcloud
  octomap::Pointcloud octo_pointcloud;
  octomap::pointcloudPCLToOctomap(pcl_cloud, octo_pointcloud);


  // // retrieve sensor pose from tf
  // tf::StampedTransform trans;
  // tf_.lookupTransform (fixed_frame_, settings.sensor_frame_, cloud->header.stamp, trans);
  // tf::Vector3 orig = trans.getOrigin();
  // octomap::pose6d  sensor_pose(orig.x(), orig.y(), orig.z(), 0, 0, 0);
  // octomap::point3d sensor_origin(orig.x(), orig.y(), orig.z());

  octomap::point3d sensor_origin = getSensorOrigin(cloud->header);
  octomap::pose6d  sensor_pose(sensor_origin.x(), sensor_origin.y(), sensor_origin.z(), 0, 0, 0);
/* 
 fprintf(stderr, "sensor origin: %.2f , %.2f , %.2f\n (frame: %s)", 
	sensor_origin.x(), sensor_origin.y(), sensor_origin.z(),
	cloud->header.frame_id.c_str());
*/
  // integrate pointcloud into map
  ros::WallTime begin_insert = ros::WallTime::now();
  collision_octree_->insertScanNaive(octo_pointcloud, sensor_origin, max_range_, false);

  double elapsed_insert = (ros::WallTime::now() - begin_insert).toSec();


  // remove outdated occupied nodes
  ros::WallTime begin_degrade = ros::WallTime::now();
  //degradeOutdatedRaycasting(cloud->header, sensor_origin, *collision_octree_);
  double elapsed_degrade = (ros::WallTime::now() - begin_degrade).toSec();


  // get all occupied nodes from map
  octomap::point3d_list node_centers;
  ros::WallTime begin_get_occupied = ros::WallTime::now();
  std::vector<geometry_msgs::Point> pointlist;
  getOccupiedPoints(pointlist);
  double elapsed_get_occupied = (ros::WallTime::now() - begin_get_occupied).toSec();


  // publish occupied cells
  ros::WallTime begin_send = ros::WallTime::now();
  if (settings.dynamic_publish_ && cmap_publisher_.getNumSubscribers() > 0) {
    publishCollisionMap(pointlist, transformed_cloud.header, cmap_publisher_);
  }
  if (pointcloud_publisher_.getNumSubscribers() > 0) {
    publishPointCloud(pointlist, transformed_cloud.header, pointcloud_publisher_);
  }
  if (octomap_visualization_pub_.getNumSubscribers() > 0){
    publishMarkerArray(pointlist, transformed_cloud.header, color_occupied_, octomap_visualization_pub_);
  }

  double elapsed_send = (ros::WallTime::now() - begin_send).toSec();

  double total_elapsed = (ros::WallTime::now() - begin_cb).toSec();
  ROS_DEBUG("Total cloudCB: %f (Update %d pnts: %f, clearing: %f, get occupied: %f, send map: %f)",
            total_elapsed, int(pcl_cloud.size()), elapsed_insert, elapsed_degrade, elapsed_get_occupied, elapsed_send);

}


void Collider::degradeOutdatedRaw(const std_msgs::Header& sensor_header, const tf::Point& sensor_origin,
                                  const std::string& other_stereo_frame, const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud_raw) {

  tf::StampedTransform trans;
  // tf_.lookupTransform (fixed_frame_, sensor_header.frame_id, sensor_header.stamp, trans);
  // tf::Transform to_world = trans;
  tf_.lookupTransform (sensor_header.frame_id, fixed_frame_, sensor_header.stamp, trans);
  tf::Transform to_sensor = trans;

  robot_mask_right_->assumeFrame(fixed_frame_, sensor_header.stamp, sensor_header.frame_id, self_filter_min_dist_);
  robot_mask_left_->assumeFrame(fixed_frame_, sensor_header.stamp, other_stereo_frame, self_filter_min_dist_);

  btVector3 sensor_pos_right, sensor_pos_left;
  planning_models::KinematicState state(cm_->getKinematicModel());
  planning_environment::configureForAttachedBodyMask(state, 
                                                     cm_,
                                                     tf_, 
                                                     sensor_header.frame_id,
                                                     sensor_header.stamp,
                                                     sensor_pos_right);

  planning_environment::configureForAttachedBodyMask(state, 
                                                     cm_,
                                                     tf_, 
                                                     other_stereo_frame,
                                                     sensor_header.stamp,
                                                     sensor_pos_left);
  
  octomap::point3d min;
  octomap::point3d max;
  computeBBX(sensor_header, min, max);
  unsigned query_time = time(NULL);
  unsigned max_update_time = 3;
  for(OcTreeType::leaf_bbx_iterator it = collision_octree_->begin_leafs_bbx(min,max),
      end=collision_octree_->end_leafs_bbx(); it!= end; ++it)
  {
    if (collision_octree_->isNodeOccupied(*it) &&
        ((query_time - it->getTimestamp()) > max_update_time))
    {
      tf::Point pos(it.getX(), it.getY(), it.getZ());
      tf::Point posRel = to_sensor(pos);
      cv::Point2d uv = cam_model_.project3dToPixel(cv::Point3d(posRel.x(), posRel.y(), posRel.z()));

      // ignore point if not in sensor cone
      if (!inSensorCone(uv))
        continue;

      // ignore point if it is occluded in the map
      if (isOccludedRaw(uv, pos.distance(sensor_origin), pcl_cloud_raw))
        continue;

      // ignore point if it is in the shadow of the robot or attached object
      if (robot_mask_right_->getMaskIntersection(pos) == robot_self_filter::SHADOW
          || robot_mask_left_->getMaskIntersection(pos) == robot_self_filter::SHADOW
          || planning_environment::computeAttachedObjectPointMask(cm_, pos, sensor_pos_right) == robot_self_filter::SHADOW
          || planning_environment::computeAttachedObjectPointMask(cm_, pos, sensor_pos_left) == robot_self_filter::SHADOW)
      {
        continue;
      }

      // otherwise: degrade node
      collision_octree_->integrateMissNoTime(&*it);

    }
  }

//  // query map for occupied leafs within a given BBX and time frame
//  std::list<std::pair<octomap::point3d, octomap::OcTreeNodeStamped*> > nodes;
//  tree.getOccupiedNodesUpdateTimeBBX(nodes, 3, time(NULL), min, max);
//
//
//
//  std::list<std::pair<octomap::point3d, octomap::OcTreeNodeStamped*> >::iterator it = nodes.begin();
//  for ( ; it != nodes.end(); ++it) {
//
//
//    // ignore point if not in sensor cone
//    if (!inSensorCone(to_sensor, it->first))
//      continue;
//
//    // ignore point if it is occluded in the map
//    if (isOccludedRaw(to_sensor, sensor_origin, it->first, pcl_cloud_raw))
//        continue;
//
//    // ignore point if it is in the shadow of the robot:
//    if (robot_mask_right_->getMaskIntersection(octomap::pointOctomapToTf(it->first)) == robot_self_filter::SHADOW
//        || robot_mask_left_->getMaskIntersection(octomap::pointOctomapToTf(it->first)) == robot_self_filter::SHADOW
//        || planning_environment::computeAttachedObjectPointMask(cm_, octomap::pointOctomapToTf(it->first), sensor_pos_right) == robot_self_filter::SHADOW
//        || planning_environment::computeAttachedObjectPointMask(cm_, octomap::pointOctomapToTf(it->first), sensor_pos_left) == robot_self_filter::SHADOW)
//    {
//      continue;
//    }
//
//    // degrade node
//    collision_octree_->integrateMissNoTime(it->second);
//  }
/*  std_msgs::Header marker_header = sensor_header;
  marker_header.frame_id = fixed_frame_;
  publishMarkerArray(shadow_voxels, marker_header, octomap_debug_pub_);*/

}



void Collider::degradeOutdatedRaycasting(const std_msgs::Header& sensor_header, const octomap::point3d& sensor_origin,
                                         octomap::OcTreeStamped& tree) {
  tf::StampedTransform trans;
  tf_.lookupTransform (sensor_header.frame_id, fixed_frame_, sensor_header.stamp, trans);
  tf::Transform to_sensor = trans;

  // compute bbx from sensor cone
  octomap::point3d min;
  octomap::point3d max;
  computeBBX(sensor_header, min, max);

  unsigned query_time = time(NULL);
  unsigned max_update_time = 1;
  for(OcTreeType::leaf_bbx_iterator it = collision_octree_->begin_leafs_bbx(min,max),
      end=collision_octree_->end_leafs_bbx(); it!= end; ++it)
  {
    if (collision_octree_->isNodeOccupied(*it) &&
        ((query_time - it->getTimestamp()) > max_update_time))
    {
      tf::Point pos(it.getX(), it.getY(), it.getZ());
      tf::Point posRel = to_sensor(pos);
      cv::Point2d uv = cam_model_.project3dToPixel(cv::Point3d(posRel.x(), posRel.y(), posRel.z()));

      // ignore point if not in sensor cone
      if (!inSensorCone(uv))
        continue;

      // ignore point if it is occluded in the map
      if (isOccludedMap(sensor_origin, it.getCoordinate()))
        continue;

      // otherwise: degrade node
      collision_octree_->integrateMissNoTime(&*it);

    }
  }
}


octomap::point3d Collider::getSensorOrigin(const std_msgs::Header& sensor_header) {

  geometry_msgs::PointStamped stamped_in;
  geometry_msgs::PointStamped stamped_out;
  stamped_in.header = sensor_header;

// HACK: laser origin
  if (sensor_header.frame_id == "base_footprint") {
   stamped_in.header.frame_id = "laser_tilt_link"; 
  }

  geometry_msgs::Point p;
  p.x=p.y=p.z=0;
  tf_.transformPoint(fixed_frame_, stamped_in, stamped_out);
  octomap::point3d retval (stamped_out.point.x, stamped_out.point.y, stamped_out.point.z);

  return retval;
}


void Collider::computeBBX(const std_msgs::Header& sensor_header, octomap::point3d& bbx_min, octomap::point3d& bbx_max) {

  std::string sensor_frame = sensor_header.frame_id;

  //  transform sensor FOV 
  geometry_msgs::PointStamped stamped_in;
  geometry_msgs::PointStamped stamped_out;
  stamped_in.header = sensor_header;
  stamped_in.header.frame_id = sensor_frame;

  // get max 3d points from camera at 0.5m and 5m.
  geometry_msgs::Point p[8];

  // define min/max 2d points
  cv::Point2d uv [4];
  uv[0].x = camera_stereo_offset_left_;
  uv[0].y = 0;
  uv[1].x = cam_size_.width + camera_stereo_offset_right_;
  uv[1].y = 0;
  uv[2].x = cam_size_.width + camera_stereo_offset_right_;
  uv[2].y = cam_size_.height;
  uv[3].x = camera_stereo_offset_left_;
  uv[3].y = cam_size_.height;

  // transform to 3d space
  cv::Point3d xyz [4];
  for (int i=0;i<4;i++) {
	xyz[i] = cam_model_.projectPixelTo3dRay(uv[i]);
    cv::Point3d xyz_05 = xyz[i] * 0.5;
    xyz[i] *= 5.; // 5meters
    p[i].x = xyz[i].x;
    p[i].y = xyz[i].y;
    p[i].z = xyz[i].z;
    p[i+4].x = xyz_05.x;
    p[i+4].y = xyz_05.y;
    p[i+4].z = xyz_05.z;
  }

  // transform to world coodinates and find axis-aligned bbx
  bbx_min.x() = bbx_min.y() = bbx_min.z() = 1e6;
  bbx_max.x() = bbx_max.y() = bbx_max.z() = -1e6;
  for (int i=0; i<8; i++) {
    stamped_in.point = p[i];
    tf_.transformPoint(fixed_frame_, stamped_in, stamped_out);
    p[i].x = stamped_out.point.x;
    p[i].y = stamped_out.point.y;
    p[i].z = stamped_out.point.z;
    if (p[i].x < bbx_min.x()) bbx_min.x() = p[i].x;
    if (p[i].y < bbx_min.y()) bbx_min.y() = p[i].y;
    if (p[i].z < bbx_min.z()) bbx_min.z() = p[i].z;
    if (p[i].x > bbx_max.x()) bbx_max.x() = p[i].x;
    if (p[i].y > bbx_max.y()) bbx_max.y() = p[i].y;
    if (p[i].z > bbx_max.z()) bbx_max.z() = p[i].z;
  }


  // // visualize axis-aligned querying bbx
  visualization_msgs::Marker bbx;
  bbx.header.frame_id = fixed_frame_;
  bbx.header.stamp = ros::Time::now();
  bbx.ns = "collider";
  bbx.id = 1;
  bbx.action = visualization_msgs::Marker::ADD;
  bbx.type = visualization_msgs::Marker::CUBE;
  bbx.pose.orientation.w = 1.0;
  bbx.pose.position.x = (bbx_min.x() + bbx_max.x()) / 2.;
  bbx.pose.position.y = (bbx_min.y() + bbx_max.y()) / 2.;
  bbx.pose.position.z = (bbx_min.z() + bbx_max.z()) / 2.;
  bbx.scale.x = bbx_max.x()-bbx_min.x();
  bbx.scale.y = bbx_max.y()-bbx_min.y();
  bbx.scale.z = bbx_max.z()-bbx_min.z();
  bbx.color.g = 1;
  bbx.color.a = 0.3;
  marker_pub_.publish(bbx);


  // visualize sensor cone
  visualization_msgs::Marker bbx_points;
  bbx_points.header.frame_id = fixed_frame_; 
  bbx_points.header.stamp = ros::Time::now();  
  bbx_points.ns = "collider";
  bbx_points.id = 2;
  bbx_points.action = visualization_msgs::Marker::ADD;
  bbx_points.type = visualization_msgs::Marker::LINE_STRIP;
  bbx_points.pose.orientation.w = 1.0;
  bbx_points.scale.x = 0.02;
  bbx_points.scale.y = 0.02;
  bbx_points.color.g = 1;
  bbx_points.color.a = 0.3;
  bbx_points.points.push_back(p[0]);
  bbx_points.points.push_back(p[1]);
  bbx_points.points.push_back(p[2]);
  bbx_points.points.push_back(p[3]);
  bbx_points.points.push_back(p[0]);
  bbx_points.points.push_back(p[4]);
  bbx_points.points.push_back(p[5]);
  bbx_points.points.push_back(p[6]);
  bbx_points.points.push_back(p[7]);
  bbx_points.points.push_back(p[4]);
  bbx_points.points.push_back(p[7]);
  bbx_points.points.push_back(p[3]);
  bbx_points.points.push_back(p[2]);
  bbx_points.points.push_back(p[6]);
  bbx_points.points.push_back(p[5]);
  bbx_points.points.push_back(p[1]);
  marker_pub_.publish(bbx_points);
}


bool Collider::inSensorCone(const cv::Point2d& uv) const {
  // Check if projected 2D coordinate in pixel range.
  // This check is a little more restrictive than it should be by using
  // 1 pixel less to account for rounding / discretization errors.
  // Otherwise points on the corner are accounted to be in the sensor cone.
  return ( (uv.x > camera_stereo_offset_left_+1)
           && (uv.x < cam_size_.width + camera_stereo_offset_right_ - 2)
           && (uv.y > 1)
           && (uv.y < cam_size_.height-2) );
}



bool Collider::isOccludedRaw(const cv::Point2d& uv, double range, const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud_raw) {

  // out of image range?
  if ((uv.x < 0) || (uv.y < 0) || (uv.x > cam_size_.width) || (uv.y > cam_size_.height))
	  return false;
  
  double sensor_range = pcl_cloud_raw(uv.x, uv.y).z;

  return (sensor_range < range);
}


bool Collider::isOccludedMap(const octomap::point3d& sensor_origin, const octomap::point3d& p) const {

  octomap::point3d direction (p-sensor_origin);
  octomap::point3d obstacle;
  double range = direction.norm() - resolution_;

  if (collision_octree_->castRay(sensor_origin, direction, obstacle, true, range)) {
    // fprintf(stderr, "<%.2f , %.2f , %.2f> -> <%.2f , %.2f , %.2f> // obs at: <%.2f , %.2f , %.2f>, range: %.2f\n", 
    //         sensor_origin.x(), sensor_origin.y(), sensor_origin.z(),
    //         p.x(), p.y(), p.z(), 
    //         obstacle.x(), obstacle.y(), obstacle.z(), (obstacle-p).norm());
    return true;
  }
  return false;
}



// publish map  ----------------------------------------------------------------------

void Collider::publishCollisionMap(const std::vector<geometry_msgs::Point>& pointlist,
                                   const std_msgs::Header &header, ros::Publisher &pub) {
  if(pointlist.size() <= 1)
    return;
  
  mapping_msgs::CollisionMap cmap;
  cmap.header = header;

  mapping_msgs::OrientedBoundingBox box;
  box.extents.x = box.extents.y = box.extents.z = collision_octree_->getResolution();
  box.axis.x = box.axis.y = 0.0; box.axis.z = 1.0;
  box.angle = 0.0;
  cmap.boxes.reserve(pointlist.size());

  for (std::vector<geometry_msgs::Point>::const_iterator it = pointlist.begin(); it != pointlist.end(); ++it) {
    box.center.x = it->x;
    box.center.y = it->y;
    box.center.z = it->z;
    cmap.boxes.push_back(box);
  }
  pub.publish(cmap);
}


void Collider::publishPointCloud(const std::vector<geometry_msgs::Point>& pointlist,
                                 const std_msgs::Header &header, ros::Publisher &pub) {

  if(pointlist.size() <= 1)
    return;

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl_cloud.points.reserve (pointlist.size ());

  for (std::vector<geometry_msgs::Point>::const_iterator it = pointlist.begin(); it != pointlist.end(); ++it) {
    pcl::PointXYZ point;
    point.x = it->x;
    point.y = it->y;
    point.z = it->z;
    pcl_cloud.points.push_back (point);
  } 

  sensor_msgs::PointCloud2 cloud;
  pcl::toROSMsg (pcl_cloud, cloud);
  cloud.header = header;
  pub.publish (cloud);    

}

void Collider::publishMarkerArray(const std::vector<geometry_msgs::Point>& pointlist,
                                 const std_msgs::Header &header, const std_msgs::ColorRGBA& color,
                                 ros::Publisher &pub) {

  if(pointlist.size() <= 1)
    return;

  visualization_msgs::Marker occupiedCellsVis;
  occupiedCellsVis.header = header;
  occupiedCellsVis.ns = "map";
  occupiedCellsVis.id = 0;
  occupiedCellsVis.type = visualization_msgs::Marker::CUBE_LIST;
  occupiedCellsVis.scale.x = collision_octree_->getResolution();
  occupiedCellsVis.scale.y = collision_octree_->getResolution();
  occupiedCellsVis.scale.z = collision_octree_->getResolution();
  occupiedCellsVis.color = color;


  occupiedCellsVis.points = pointlist;

  pub.publish (occupiedCellsVis);

}



// action server  --------------------------------------------------------------------

bool Collider::reset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  delete collision_octree_;
  collision_octree_ = new OcTreeType(resolution_);
  return true;
}


bool Collider::dummyReset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  // Dummy function that doesn't actually reset anything. Needed because some
  // parts of the grasping pipeline want to reset the collision map
  // on occasion, but with the octomap we don't want other nodes resetting the map sporadically.
  return true;
}


void Collider::makeStaticCollisionMap(const collision_environment_msgs::MakeStaticCollisionMapGoalConstPtr& goal) {

  std_msgs::Header head;
  head.stamp = ros::Time::now();
  head.frame_id = fixed_frame_;

  ROS_INFO("Making static collision map");

  //we first publish whatever we got
  std::vector<geometry_msgs::Point> pointlist;
  getOccupiedPoints(pointlist);
  if (pointcloud_publisher_.getNumSubscribers() > 0) {
    publishPointCloud(pointlist, head, pointcloud_publisher_);
  }
  
  if(publish_over_dynamic_map_)
  {
    if (cmap_publisher_.getNumSubscribers() > 0) {
      publishCollisionMap(pointlist, head, cmap_publisher_);
    }
    // loop through cloud_source_map, set dynamic publish to false on all sources to preserve behavior of older collision_map_self_occ
    for(std::map<std::string,CloudInfo>::iterator it = cloud_sources_.begin(); 
        it != cloud_sources_.end();
        it++) 
    {
      it->second.dynamic_publish_ = false;	
    }	
    ROS_INFO("Should stop publishing dynamic map");
  } else {
    publishCollisionMap(pointlist, head, static_map_publisher_);
  }
  action_server_->setSucceeded();
}

bool Collider::octomapSrv(octomap_ros::GetOctomap::Request  &req, octomap_ros::GetOctomap::Response &res){
	ROS_DEBUG("Sending map data on service request");

	res.map.header.frame_id = fixed_frame_;
	res.map.header.stamp = ros::Time::now();
	octomap::octomapMapToMsg(*collision_octree_, res.map);

	return true;
}

bool Collider::occupancyPointSrv(collision_environment_msgs::OccupancyPointQuery::Request &req,
                                  collision_environment_msgs::OccupancyPointQuery::Response &res){

  octomap::OcTreeNodeStamped* node = collision_octree_->search(req.point.x, req.point.y, req.point.z);
  if (node){
    if (collision_octree_->isNodeOccupied(node))
      res.occupancy=collision_environment_msgs::OccupancyPointQueryResponse::OCCUPIED;
    else
      res.occupancy=collision_environment_msgs::OccupancyPointQueryResponse::FREE;
  } else{
    res.occupancy = collision_environment_msgs::OccupancyPointQueryResponse::UNKNOWN;
  }

	return true;
}

bool Collider::occupancyBBXSrv(collision_environment_msgs::OccupancyBBXQuery::Request &req,
                                collision_environment_msgs::OccupancyBBXQuery::Response &res){

  OcTreeType::leaf_bbx_iterator it = collision_octree_->begin_leafs_bbx(octomap::pointMsgToOctomap(req.min),
                                                                    octomap::pointMsgToOctomap(req.max));
  OcTreeType::leaf_bbx_iterator end = collision_octree_->end_leafs_bbx();

  geometry_msgs::Point pt;
  for(; it!= end; ++it){
    pt.x = it.getX();
    pt.y = it.getY();
    pt.z = it.getZ();

    if (collision_octree_->isNodeOccupied(*it)){
      res.occupied.push_back(pt);
    } else {
      res.free.push_back(pt);
    }
  }
  res.resolution = collision_octree_->getResolution();

  return true;
}

bool Collider::occupancyBBXSizeSrv(collision_environment_msgs::OccupancyBBXSizeQuery::Request &req,
                                collision_environment_msgs::OccupancyBBXSizeQuery::Response &res){

  octomap::point3d center = octomap::pointMsgToOctomap(req.center);
  octomap::point3d size = octomap::pointMsgToOctomap(req.size);
  OcTreeType::leaf_bbx_iterator it = collision_octree_->begin_leafs_bbx(center - (size*0.5), center + (size*0.5));
  OcTreeType::leaf_bbx_iterator end = collision_octree_->end_leafs_bbx();

  geometry_msgs::Point pt;
  for(; it!= end; ++it){
    pt.x = it.getX();
    pt.y = it.getY();
    pt.z = it.getZ();

    if (collision_octree_->isNodeOccupied(*it)){
      res.occupied.push_back(pt);
    } else {
      res.free.push_back(pt);
    }
  }
  res.resolution = collision_octree_->getResolution();

  return true;
}

