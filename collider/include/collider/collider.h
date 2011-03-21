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

#include <ros/ros.h>

#include <std_srvs/Empty.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <mapping_msgs/CollisionMap.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
//#include <pcl_tf/transforms.h> // cturtle
#include <pcl_ros/transforms.h>  // dturtle

#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <mapping_msgs/AttachedCollisionObject.h>
#include <collision_environment_msgs/MakeStaticCollisionMapAction.h>
#include <collision_environment_msgs/OccupancyPointQuery.h>
#include <collision_environment_msgs/OccupancyBBXQuery.h>
#include <collision_environment_msgs/OccupancyBBXSizeQuery.h>
#include <actionlib/server/simple_action_server.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <robot_self_filter/self_mask.h>
#include <planning_environment/models/collision_models.h>

#include <vector>
#include <algorithm>

#include <octomap/OcTreeStamped.h>
#include <octomap_ros/OctomapROS.h>
#include <octomap_ros/OctomapBinary.h>
#include <octomap_ros/GetOctomap.h>


class Collider {

 public:

  Collider();
  ~Collider();

 public:

  typedef octomap::OcTreeStamped OcTreeType;
  
  /// Struct to hold config options for subscribed PointClouds
  struct CloudInfo 
  {
    CloudInfo(): cloud_name_(""), raw_cloud_name_(""), frame_subsample_(1.0),
                 point_subsample_(1.0), sensor_frame_(""), sensor_stereo_other_frame_(""), counter_(0),
                 dynamic_buffer_size_(1), static_buffer_size_(1), 
                 dynamic_buffer_duration_(0), static_buffer_duration_(0),
                 dynamic_publish_(true), static_publish_(true), 
                 dynamic_until_static_publish_(true)
                 
    {
    };
    std::string cloud_name_;
    std::string raw_cloud_name_;
    int frame_subsample_; 
    int point_subsample_; 
    std::string sensor_frame_;
    std::string sensor_stereo_other_frame_;
    unsigned int counter_;
    
    unsigned int dynamic_buffer_size_, static_buffer_size_;
    ros::Duration dynamic_buffer_duration_, static_buffer_duration_;

    // Settings for publishing map data on the standard publisher.
    // Note that if static_publish is set to false, static map data WILL still be published on the dedicated
    // static map topic when a map is acquired, but it WON'T publish on the main publisher, which publishes
    // a union of all the maps in the node's buffer.  If dynamic_until_static_publish is set to true then 
    // the dynamic map will be published on the main topic until a static map from that source is created, 
    // at which point dynamic behavior will revert to that specified by dynamic_publish;
    bool dynamic_publish_, static_publish_, dynamic_until_static_publish_;  
  };

  // parameters & precomputed values for the box that represents the collision map in the fixed frame
  struct BoxInfo
  {    
    double dimensionX, dimensionY, dimensionZ;
    double originX, originY, originZ;
    double real_minX, real_minY, real_minZ;
    double real_maxX, real_maxY, real_maxZ;
  };


 protected:

  // msg callbacks
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud, const std::string topic_name);
  void cloudCombinedCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud,
                    const sensor_msgs::PointCloud2::ConstPtr &cloud_raw, const std::string topic_name);
  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &cam_info);

  void attachedObjectCallback(const mapping_msgs::AttachedCollisionObjectConstPtr& attached_object);

  // obstacle cleaning
  void degradeOutdatedRaycasting(const std_msgs::Header& sensor_header, const octomap::point3d& sensor_origin, octomap::OcTreeStamped& tree);
  void computeBBX(const std_msgs::Header& sensor_header, octomap::point3d& bbx_min, octomap::point3d& bbx_max);
  bool inSensorCone(const cv::Point2d& uv) const;
  bool isOccludedMap(const octomap::point3d& sensor_origin, const octomap::point3d& p) const;
  octomap::point3d getSensorOrigin(const std_msgs::Header& sensor_header);
  void degradeOutdatedRaw(const std_msgs::Header& sensor_header, const tf::Point& sensor_origin,
                          const std::string& other_stereo_frame,
                          const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud_raw);
  bool isOccludedRaw (const cv::Point2d& uv, double range, const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud_raw);

  //const tf::Transform& transform, 

  // publish map
  void publishCollisionMap(const std::vector<geometry_msgs::Point>& pointlist, const std_msgs::Header &header, ros::Publisher &pub);
  void publishPointCloud(const std::vector<geometry_msgs::Point>& pointlist, const std_msgs::Header &header, ros::Publisher &pub);
  void publishMarkerArray(const std::vector<geometry_msgs::Point>& pointlist, const std_msgs::Header &header, const std_msgs::ColorRGBA& color, ros::Publisher &pub);

  // action server
  bool reset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  void makeStaticCollisionMap(const collision_environment_msgs::MakeStaticCollisionMapGoalConstPtr& goal);
  bool dummyReset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  // occupancy queries:
  bool octomapSrv(octomap_ros::GetOctomap::Request  &req, octomap_ros::GetOctomap::Response &res);
  bool occupancyPointSrv(collision_environment_msgs::OccupancyPointQuery::Request &req, collision_environment_msgs::OccupancyPointQuery::Response &res);
  bool occupancyBBXSrv(collision_environment_msgs::OccupancyBBXQuery::Request &req, collision_environment_msgs::OccupancyBBXQuery::Response &res);
  bool occupancyBBXSizeSrv(collision_environment_msgs::OccupancyBBXSizeQuery::Request &req, collision_environment_msgs::OccupancyBBXSizeQuery::Response &res);



 protected:

  tf::TransformListener tf_;
  std::vector<message_filters::Subscriber<sensor_msgs::PointCloud2>* > message_filter_subscribers_;
  std::vector<tf::MessageFilter<sensor_msgs::PointCloud2>* > message_filters_;

  planning_environment::CollisionModels *cm_;
  message_filters::Subscriber<mapping_msgs::AttachedCollisionObject> *attached_collision_object_subscriber_;

  ros::NodeHandle root_handle_;

  ros::Publisher cmap_publisher_, static_map_publisher_, pointcloud_publisher_, marker_pub_,
                  octomap_visualization_pub_, octomap_visualization_free_pub_;

  ros::Publisher octomap_visualization_attached_pub_, octomap_visualization_attached_array_pub_;
  std_msgs::ColorRGBA attached_color_;

  ros::ServiceServer reset_service_, dummy_reset_service_, transparent_service_,
                      get_octomap_service_, occupancy_point_service_,
                      occupancy_bbx_service_, occupancy_bbx_size_service_;

  ros::Subscriber*  camera_info_subscriber_;
  
  OcTreeType*	collision_octree_;
  std::map<std::string,CloudInfo> 		cloud_sources_;
  robot_self_filter::SelfMask* robot_mask_right_;
  robot_self_filter::SelfMask* robot_mask_left_;

  image_geometry::PinholeCameraModel cam_model_;
  bool cam_model_initialized_;

  bool publish_over_dynamic_map_;

  std::string fixed_frame_;
  double resolution_, max_range_, self_filter_min_dist_;
  int pruning_period_, pruning_counter_;
  bool transparent_freespace_;
  int camera_stereo_offset_left_;
  int camera_stereo_offset_right_;
  std_msgs::ColorRGBA color_free_, color_occupied_;
  cv::Size cam_size_;

  BoxInfo workspace_box_;

  inline void getOccupiedPoints(std::vector<geometry_msgs::Point>& pointlist) const{
	  pointlist.reserve(collision_octree_->size() / 2.0);
	  for (OcTreeType::iterator it = collision_octree_->begin(),
			  end = collision_octree_->end(); it != end; ++it){
		  if (collision_octree_->isNodeOccupied(*it)){
			  geometry_msgs::Point p;
			  p.x = it.getX();
			  p.y = it.getY();
			  p.z = it.getZ();
			  pointlist.push_back(p);
		  }
	  }
  }

  inline void getFreePoints(std::vector<geometry_msgs::Point>& pointlist) const{
	  pointlist.reserve(collision_octree_->size() / 2.0);
	  for (OcTreeType::iterator it = collision_octree_->begin(),
			  end = collision_octree_->end(); it != end; ++it){
		  if (!collision_octree_->isNodeOccupied(*it)){
			  geometry_msgs::Point p;
			  p.x = it.getX();
			  p.y = it.getY();
			  p.z = it.getZ();
			  pointlist.push_back(p);
			  if (it.getSize() != collision_octree_->getResolution())
				  ROS_WARN("%f", it.getSize());

		  }
	  }
  }

  boost::shared_ptr<actionlib::SimpleActionServer<collision_environment_msgs::MakeStaticCollisionMapAction> > action_server_;	

  // Time synchronized, dual input (raw+filtered)
  std::vector<message_filters::Subscriber<sensor_msgs::PointCloud2>* > sub_filtered_clouds_;
  std::vector<message_filters::Subscriber<sensor_msgs::PointCloud2>* > sub_raw_clouds_;
  //message_filters::Subscriber<sensor_msgs::PointCloud2> sub_filtered_;
  //message_filters::Subscriber<sensor_msgs::PointCloud2> sub_raw_;
  //message_filters::Synchronizer<message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> > sync_;
  std::vector<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> >* > sync_vector_;
};

