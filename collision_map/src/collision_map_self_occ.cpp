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

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud.h>
#include <arm_navigation_msgs/CollisionMap.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
//#include <robot_self_filter/self_see_filter.h>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <vector>
#include <algorithm>
#include <set>
#include <iterator>
#include <cstdlib>
#include <arm_navigation_msgs/MakeStaticCollisionMapAction.h>
#include <actionlib/server/simple_action_server.h>

struct CloudInfo 
{
  CloudInfo(): cloud_name_(""), frame_subsample_(1.0),
               point_subsample_(1.0), sensor_frame_(""), counter_(0), 
	       dynamic_buffer_size_(1), static_buffer_size_(1), 
	       dynamic_buffer_duration_(0), static_buffer_duration_(0),
	       dynamic_publish_(true), static_publish_(true), 
               dynamic_until_static_publish_(true)
  {
  };
  std::string cloud_name_;
  int frame_subsample_; //take every nth frame (1 is no discards)
  int point_subsample_; //take every nth point from the frames you do process
  std::string sensor_frame_;
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

class CollisionMapperOcc
{
public:
  
  CollisionMapperOcc(void): root_handle_(""), making_static_collision_map_(false), disregard_first_message_(false)
  {
    static_map_goal_ = NULL;
    
    ros::NodeHandle priv("~");
    
    // a frame that does not move with the robot
    priv.param<std::string>("fixed_frame", fixedFrame_, "odom");

    // a frame that moves with the robot
    priv.param<std::string>("robot_frame", robotFrame_, "base_link");

    // bounds of collision map in robot frame
    priv.param<double>("dimension_x", bi_.dimensionX, 1.0);
    priv.param<double>("dimension_y", bi_.dimensionY, 1.5);
    priv.param<double>("dimension_z", bi_.dimensionZ, 2.0);

    // origin of collision map in the robot frame
    priv.param<double>("origin_x", bi_.originX, 1.1);
    priv.param<double>("origin_y", bi_.originY, 0.0);
    priv.param<double>("origin_z", bi_.originZ, 0.0);

    // sensor frame
    //priv.param<std::string>("sensor_frame", bi_.sensor_frame, std::string());
	
    // resolution
    priv.param<double>("resolution", bi_.resolution, 0.015);
	
    ROS_INFO("Maintaining occlusion map in frame '%s', with origin at (%f, %f, %f) and dimension (%f, %f, %f), resolution of %f; "
             "sensor is in frame '%s', fixed fame is '%s'.",
             robotFrame_.c_str(), bi_.originX, bi_.originY, bi_.originZ, bi_.dimensionX, bi_.dimensionY, bi_.dimensionZ, 
             bi_.resolution, bi_.sensor_frame.c_str(), fixedFrame_.c_str());
        
    priv.param<bool>("publish_occlusion", publishOcclusion_, false);
    priv.param<bool>("publish_static_over_dynamic_map", publish_over_dynamic_map_, false);


    // compute some useful values
    bi_.real_minX = -bi_.dimensionX + bi_.originX;
    bi_.real_maxX =  bi_.dimensionX + bi_.originX;
    bi_.real_minY = -bi_.dimensionY + bi_.originY;
    bi_.real_maxY =  bi_.dimensionY + bi_.originY;
    bi_.real_minZ = -bi_.dimensionZ + bi_.originZ;
    bi_.real_maxZ =  bi_.dimensionZ + bi_.originZ;	

    //self_filter_ = new filters::SelfFilter<sensor_msgs::PointCloud>(priv);

    // advertise our topics: full map and updates
    cmapPublisher_ = root_handle_.advertise<arm_navigation_msgs::CollisionMap>("collision_map_occ", 1, true);
    cmapUpdPublisher_ = root_handle_.advertise<arm_navigation_msgs::CollisionMap>("collision_map_occ_update", 1);
    static_map_publisher_ = root_handle_.advertise<arm_navigation_msgs::CollisionMap>("collision_map_occ_static", 1);
        
    if(!priv.hasParam("cloud_sources")) {
      ROS_WARN("No links specified for self filtering.");
    } else {
      XmlRpc::XmlRpcValue cloud_vals;
      priv.getParam("cloud_sources", cloud_vals);
      
      if(cloud_vals.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_WARN("Cloud sources needs to be an array");
      } else {
        for(int i = 0; i < cloud_vals.size(); i++) {
          CloudInfo cps;
          if(cloud_vals[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_WARN("Cloud source entry %d is not a structure.  Stopping processing of cloud sources",i);
            break;
          }
          if(!cloud_vals[i].hasMember("name")) {
            ROS_WARN("Cloud sources entry %d has no name.  Stopping processing of cloud sources",i);
            break;
          } 
          if(!cloud_vals[i].hasMember("sensor_frame")) {
            ROS_WARN("Cloud sources entry %d has no sensor_frame.  Stopping processing of cloud sources",i);
            break;
          } 
          cps.cloud_name_ = std::string(cloud_vals[i]["name"]);
          cps.sensor_frame_ = std::string(cloud_vals[i]["sensor_frame"]);
          if(!cloud_vals[i].hasMember("frame_subsample")) {
            ROS_DEBUG("Cloud sources entry %d has no frame subsample.  Assuming default subsample of %g",i,1.0);
            cps.frame_subsample_ = 1.0;;
          } else {
            cps.frame_subsample_ = cloud_vals[i]["frame_subsample"];
          }
          if(!cloud_vals[i].hasMember("point_subsample")) {
            cps.point_subsample_ = 1.0;;
          } else {
            cps.point_subsample_ = cloud_vals[i]["point_subsample"];
          }
	
	  if(cloud_vals[i].hasMember("dynamic_buffer_size"))
	    cps.dynamic_buffer_size_ = static_cast<unsigned int>(int(cloud_vals[i]["dynamic_buffer_size"]));
	
  	  if(cloud_vals[i].hasMember("static_buffer_size"))
	    cps.static_buffer_size_ = static_cast<unsigned int>(int(cloud_vals[i]["static_buffer_size"]));

	  if(cloud_vals[i].hasMember("dynamic_buffer_duration"))
	    cps.dynamic_buffer_duration_ = ros::Duration(int(cloud_vals[i]["dynamic_buffer_duration"]));

	  if(cloud_vals[i].hasMember("static_buffer_duration"))
	    cps.static_buffer_duration_ = ros::Duration(int(cloud_vals[i]["static_buffer_duration"]));

	  if(cloud_vals[i].hasMember("dynamic_publish"))
	    cps.dynamic_publish_ = cloud_vals[i]["dynamic_publish"];

	  if(cloud_vals[i].hasMember("static_publish"))
	    cps.static_publish_ = cloud_vals[i]["static_publish"];

          if(cloud_vals[i].hasMember("dynamic_until_static_publish")) {
	    cps.dynamic_until_static_publish_ = cloud_vals[i]["dynamic_until_static_publish"];
          }

          if(cloud_source_map_.find(cps.cloud_name_) != cloud_source_map_.end()) {
            ROS_WARN_STREAM("Already have a cloud defined with name " << cps.cloud_name_);
          } else {
            cloud_source_map_[cps.cloud_name_] = cps;
            mn_cloud_tf_sub_vector_.push_back(new message_filters::Subscriber<sensor_msgs::PointCloud>(root_handle_, cps.cloud_name_, 1));
            mn_cloud_tf_fil_vector_.push_back(new tf::MessageFilter<sensor_msgs::PointCloud>(*(mn_cloud_tf_sub_vector_.back()), tf_, "", 1));
            mn_cloud_tf_fil_vector_.back()->registerCallback(boost::bind(&CollisionMapperOcc::cloudCallback, this, _1, cps.cloud_name_));
            // if (publishOcclusion_) {
            //   std::string name = std::string("collision_map_occ_occlusion_")+cps.cloud_name_;
            //   occPublisherMap_[cps.cloud_name_] = root_handle_.advertise<arm_navigation_msgs::CollisionMap>(name, 1);
            // }
            static_map_published_[cps.cloud_name_] = false;
            ROS_INFO_STREAM("Source name " << cps.cloud_name_);
          }
        }
      }
    }
    // create a message notifier (and enable subscription) for both the full map and for the updates
    //mnCloud_ = new tf::MessageNotifier<sensor_msgs::PointCloud>(tf_, boost::bind(&CollisionMapperOcc::cloudCallback, this, _1), "cloud_in", "", 1);
    //mnCloudIncremental_ = new tf::MessageNotifier<sensor_msgs::PointCloud>(tf_, boost::bind(&CollisionMapperOcc::cloudIncrementalCallback, this, _1), "cloud_incremental_in", "", 1);
    
    // configure the self mask and the message notifier
    //sm_ = self_filter_->getSelfMask();
    std::vector<std::string> frames;
    //sm_->getLinkNames(frames);
    if (std::find(frames.begin(), frames.end(), robotFrame_) == frames.end()) {
      frames.push_back(robotFrame_);
    }
    for(unsigned int i = 0; i < mn_cloud_tf_fil_vector_.size(); i++) {
      mn_cloud_tf_fil_vector_[i]->setTargetFrames(frames);
    }
    //mnCloudIncremental_->setTargetFrame(frames);
    resetService_ = priv.advertiseService("reset", &CollisionMapperOcc::reset, this);

    action_server_.reset(new actionlib::SimpleActionServer<arm_navigation_msgs::MakeStaticCollisionMapAction>(root_handle_, "make_static_collision_map", 
                                                                                                                    boost::bind(&CollisionMapperOcc::makeStaticCollisionMap, this, _1)));
  }
  
  ~CollisionMapperOcc(void)
  {

    for(std::map<std::string, std::list<StampedCMap*> >::iterator it = currentMaps_.begin(); it != currentMaps_.end(); it++)
    {
      for(std::list<StampedCMap*>::iterator itbuff = it->second.begin(); itbuff != it->second.end(); itbuff++)
      {
        delete (*itbuff);
      }
    }

    for(std::map<std::string, StampedCMap*>::iterator it = tempMaps_.begin();
        it != tempMaps_.end();
        it++) {
      delete it->second;
    }

    for(unsigned int i = 0; i < mn_cloud_tf_sub_vector_.size(); i++) {
      delete mn_cloud_tf_sub_vector_[i];
    }
    for(unsigned int i = 0; i < mn_cloud_tf_fil_vector_.size(); i++) {
      delete mn_cloud_tf_fil_vector_[i];
    }
    //delete self_filter_;
    if(static_map_goal_) delete static_map_goal_;
    //delete mnCloudIncremental_;
  }

  void run(void)
  {
    //if (bi_.sensor_frame.empty())
    //  ROS_ERROR("No sensor frame specified. Cannot perform raytracing");
    //else
    ros::spin();
  }
        
private:
  
  struct CollisionPoint
  {
    CollisionPoint(void) {}
    CollisionPoint(int _x, int _y, int _z) : x(_x), y(_y), z(_z) {}
	
    int x, y, z;
  };

  // define an order on points
  struct CollisionPointOrder
  {
    bool operator()(const CollisionPoint &a, const CollisionPoint &b) const
    {
      if (a.x < b.x)
        return true;
      if (a.x > b.x)
        return false;
      if (a.y < b.y)
        return true;
      if (a.y > b.y)
        return false;
      return a.z < b.z;
    }
  };
    
  typedef std::set<CollisionPoint, CollisionPointOrder> CMap;

  struct StampedCMap
  {
    std::string frame_id;
    ros::Time stamp;
    CMap cmap;
  };
  

  // parameters & precomputed values for the box that represents the collision map
  // around the robot
  struct BoxInfo
  {    
    double dimensionX, dimensionY, dimensionZ;
    double originX, originY, originZ;
    std::string sensor_frame;
    double resolution;
    double real_minX, real_minY, real_minZ;
    double real_maxX, real_maxY, real_maxZ;
  };
			
  void cloudIncrementalCallback(const sensor_msgs::PointCloudConstPtr &cloud)
  {
    if (!mapProcessing_.try_lock())
      return;

    ros::WallTime tm = ros::WallTime::now();
                
    ROS_DEBUG("Got pointcloud update that is %f seconds old", (ros::Time::now() - cloud->header.stamp).toSec());
	
    sensor_msgs::PointCloud out;
    //    if(point_subsample_ > 1) { 
    //       sensor_msgs::PointCloud sub;
    //       sub.header = (*cloud).header;
    //       for(unsigned int i = 0; i < (*cloud).points.size(); i += point_subsample_) {
    //         sub.points.push_back((*cloud).points[i]);
    //       }
    //       tf_.transformPointCloud(robotFrame_, sub, out);
    //     } else {
    // transform the pointcloud to the robot frame
    // since we need the points in this frame (around the robot)
    // to compute the collision map
    tf_.transformPointCloud(robotFrame_, *cloud, out);


    CMap obstacles;
    constructCollisionMap(out, obstacles);

    CMap diff;
    //set_difference(obstacles.begin(), obstacles.end(), currentMap_.begin(), currentMap_.end(),
    //               std::inserter(diff, diff.begin()), CollisionPointOrder());
    mapProcessing_.unlock();
	
    if (!diff.empty())
      publishCollisionMap(diff, out.header.frame_id, out.header.stamp, cmapUpdPublisher_);
  }

  void subsampleCloudPoints(const sensor_msgs::PointCloud &msg, sensor_msgs::PointCloud& myMsg, int subsampleNum) {
    myMsg.points.clear();
    myMsg.header = msg.header;
    
    //myMsg.channels.clear();        
    for(int i = 0; i < (int)msg.points.size(); i += subsampleNum) {
      myMsg.points.push_back(msg.points[i]);
      //if((*msg).channels.size() < i) {
      //  myMsg.channels.push_back((*msg).channels[i]);
      //}
    }
  }

  void updateBuffer(std::list<StampedCMap*> &buffer, const unsigned int buffer_size, const ros::Duration buffer_duration, const std::string sensor_frame)
  {
    if(buffer_size > 1)
    {
      while(buffer.size() > buffer_size)
      {
        ROS_DEBUG_STREAM("Deleting old map in frame " << sensor_frame << " total buffer size: " << buffer.size());
        delete buffer.back();
        buffer.pop_back();
      }
    }

    if(buffer_duration > ros::Duration(0))
    {
      ros::Time min_time = buffer.front()->stamp - buffer_duration;

      while(buffer.back()->stamp < min_time)
      {
        ROS_DEBUG_STREAM("Deleting old map in frame " << sensor_frame << " which is too old by: " << min_time - buffer.back()->stamp << " seconds" );
        delete buffer.back();
        buffer.pop_back();
      }
    }

    // Keep only most recent
    if(buffer_size <= 1 && buffer_duration <= ros::Duration(0))
    {
      ROS_DEBUG_STREAM("Keeping only most recent map in frame " << sensor_frame );
      while(buffer.size() > 1)
      {
        delete buffer.back();
        buffer.pop_back();
      }
    }

  }

  void composeMapUnion(std::map<std::string, std::list<StampedCMap*> > &maps, CMap &uni)
  {
    //composing the union of all the sensor maps
    for(std::map<std::string, std::list<StampedCMap*> >::iterator it = maps.begin(); it != maps.end(); it++)
    {
      int processed = 0;
      for(std::list<StampedCMap*>::iterator itbuff = it->second.begin(); itbuff != it->second.end(); itbuff++)
      {
        set_union(uni.begin(), uni.end(), (*itbuff)->cmap.begin(), (*itbuff)->cmap.end(), std::inserter(uni, uni.begin()), CollisionPointOrder());
        processed++;
      }

      ROS_DEBUG_STREAM("The buffer for sensor frame " << it->first << " had " << processed << " maps");
    }

  }
    
  void cloudCallback(const sensor_msgs::PointCloudConstPtr &cloud, const std::string topic_name)
  {
    CloudInfo settings =  cloud_source_map_[topic_name];

    if(!making_static_collision_map_ &&  !settings.dynamic_publish_ && (!settings.dynamic_until_static_publish_ || static_map_published_[topic_name])) {
      return;
    }

    //sensor_msgs::PointCloud sf_out;
    //self_filter_->updateWithSensorFrame(*cloud, sf_out, settings.sensor_frame_);
    //ROS_DEBUG_STREAM("Size before self filter " << (*cloud).points.size() << " after " << sf_out.points.size());
    
    ros::WallTime tm = ros::WallTime::now();

    sensor_msgs::PointCloud subCloud;

    ROS_DEBUG("Got pointcloud that is %f seconds old", (ros::Time::now() - cloud->header.stamp).toSec());
    ROS_DEBUG("Received %u data points.",(unsigned int)(*cloud).points.size ());
    subsampleCloudPoints(*cloud, subCloud, settings.point_subsample_);
    ROS_DEBUG("After subsampling we have %u data points.",(unsigned int)subCloud.points.size ());


    boost::recursive_mutex::scoped_lock lock(mapProcessing_);

    CMap obstacles;

    sensor_msgs::PointCloud transCloud;
    
    tf_.transformPointCloud(robotFrame_, subCloud, transCloud);
	
    // transform the pointcloud to the robot frame
    // since we need the points in this frame (around the robot)
    // to compute the collision map
    constructCollisionMap(transCloud, obstacles);

    if(making_static_collision_map_ && topic_name == static_map_goal_->cloud_source) {
      if(disregard_first_message_) {
        disregard_first_message_ = false;
      } else {
        //making new collision map for this message
        std::string map_name = topic_name+"_static_temp";
        StampedCMap* static_map;
        if(tempMaps_.find(map_name) != tempMaps_.end()) {
          static_map = tempMaps_[map_name];
        } else {
          static_map = new StampedCMap();
	  static_map->frame_id = transCloud.header.frame_id;
	  static_map->stamp = transCloud.header.stamp;
          tempMaps_[map_name] = static_map;
        } 
        updateMap(&static_map->cmap, obstacles, transCloud.header.frame_id, transCloud.header.stamp, settings.sensor_frame_, settings.cloud_name_);
        if(++cloud_count_ == static_map_goal_->number_of_clouds) {

	    ROS_DEBUG("Publishing static map");
	    publishCollisionMap(static_map->cmap, transCloud.header.frame_id, transCloud.header.stamp, static_map_publisher_);

	  if(!settings.static_publish_)
  	  {
	    delete static_map;
	  }
	  else
   	  {
	    ROS_DEBUG("Saving static map for inclusion in future dynamic maps");
	    currentMaps_[topic_name+"_static"].push_front(static_map);

	    updateBuffer(currentMaps_[topic_name+"_static"], settings.static_buffer_size_, settings.static_buffer_duration_, topic_name+"_static");

            if(settings.dynamic_until_static_publish_) {
              //now if we are supposed to stop publishing dynamic maps get ride of the dynamic map for this topic
              std::list<StampedCMap*>& dyn_list = currentMaps_[topic_name+"_dynamic"];
              for(std::list<StampedCMap*>::iterator it = dyn_list.begin(); it != dyn_list.end(); it++) {
                delete (*it);
              }
              currentMaps_.erase(topic_name+"_dynamic");
            }

            CMap uni;
            composeMapUnion(currentMaps_, uni);

            publishCollisionMap(uni, transCloud.header.frame_id, transCloud.header.stamp, cmapPublisher_);
	  }
          
	  tempMaps_.erase(map_name);
	  making_static_collision_map_ = false;
          static_map_published_[topic_name] = true;
        }
      }
    }

    if(settings.dynamic_publish_ || (settings.dynamic_until_static_publish_ && !static_map_published_[topic_name]))
    {

      StampedCMap* current_map;

      // try to transform the previous map(s) (if it exists) to the new frame
      if(settings.dynamic_buffer_size_ == 1 && currentMaps_.find(topic_name+"_dynamic") != currentMaps_.end()) {
        current_map = currentMaps_[topic_name+"_dynamic"].front();
//      if (!current_map->cmap.empty()) 
//      {
//	if (!transformMap(current_map->cmap, current_map->header, transCloud.header))  // Need to transform entire buffer, not just current_map!
//  	{
//        current_map->cmap.clear();
//  	} 
//      current_map->header = transCloud.header;
//      }
      } else {
        current_map = new StampedCMap();
        current_map->frame_id = transCloud.header.frame_id;
	current_map->stamp = transCloud.header.stamp;
        currentMaps_[topic_name+"_dynamic"].push_front(current_map);
      }

      // update map
      updateMap(&current_map->cmap, obstacles, transCloud.header.frame_id, transCloud.header.stamp, settings.sensor_frame_, settings.cloud_name_);
      updateBuffer(currentMaps_[topic_name+"_dynamic"], settings.dynamic_buffer_size_, settings.dynamic_buffer_duration_, topic_name+"_dynamic");

      CMap uni;
      composeMapUnion(currentMaps_, uni);

      publishCollisionMap(uni, transCloud.header.frame_id, transCloud.header.stamp, cmapPublisher_);

    } 

    double sec = (ros::WallTime::now() - tm).toSec();
    ROS_DEBUG("Updated collision map took %g seconds",sec);

  }



  void updateMap(CMap* currentMap, CMap &obstacles, 
		 std::string &frame_id,
		 ros::Time &stamp,
		 std::string to_frame_id, 
		 std::string cloud_name)
  {
    if (currentMap->empty())
    {
      *currentMap = obstacles;
    }
    else
    {
      CMap diff;
	    
      // find the points from the old map that are no longer visible
      set_difference(currentMap->begin(), currentMap->end(), obstacles.begin(), obstacles.end(),
                     std::inserter(diff, diff.begin()), CollisionPointOrder());
	    
      // the current map will at least contain the new info
      *currentMap = obstacles;

      // find out which of these points are now occluded 
      //sm_->assumeFrame(header, to_frame_id, 0.05);
	    
      // OpenMP need an int as the lookup variable, but for set,
      // this is not possible, so we copy to a vector
      int n = diff.size();
      std::vector<CollisionPoint> pts(n);
      std::copy(diff.begin(), diff.end(), pts.begin());

      //unsigned int count = 0;

      // // add points occluded by self
      // if (publishOcclusion_)
      // {
      //   CMap keep;
		
      //   //#pragma omp parallel for
      //   for (int i = 0 ; i < n ; ++i)
      //   {
      //     tf::Vector3 p(((double)pts[i].x - 0.5) * bi_.resolution + bi_.originX,
      //                 ((double)pts[i].y - 0.5) * bi_.resolution + bi_.originY,
      //                 ((double)pts[i].z - 0.5) * bi_.resolution + bi_.originZ);
      //     if (sm_->getMaskIntersection(p) == robot_self_filter::SHADOW)
      //     {
      //       //#pragma omp critical
      //       {
      //         keep.insert(pts[i]);
      //         count++;
      //         currentMap->insert(pts[i]);
      //       }
      //     }
      //   }
      //   ROS_DEBUG("Occlusion topic %s num points %d", cloud_name.c_str(), count);
      //   publishCollisionMap(keep, header, occPublisherMap_[cloud_name]);
      // }
      // else
      // {
      //   //#pragma omp parallel for
      //   for (int i = 0 ; i < n ; ++i)
      //   {
      //     tf::Vector3 p(((double)pts[i].x - 0.5) * bi_.resolution + bi_.originX,
      //                 ((double)pts[i].y - 0.5) * bi_.resolution + bi_.originY,
      //                 ((double)pts[i].z - 0.5) * bi_.resolution + bi_.originZ);
      //     if (sm_->getMaskIntersection(p) == robot_self_filter::SHADOW)
      //     {
      //       //#pragma omp critical
      //       {
      //         currentMap->insert(pts[i]);
      //       }
      //     }		
      //   }
		
      // }
	    
    }
  }
    
  bool reset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    boost::recursive_mutex::scoped_lock lock(mapProcessing_);

    for(std::map<std::string, std::list<StampedCMap*> >::iterator it = currentMaps_.begin(); it != currentMaps_.end(); it++)
    {
      for(std::list<StampedCMap*>::iterator itbuff = it->second.begin(); itbuff != it->second.end(); itbuff++)
      {
        delete (*itbuff);
      }
    }

    for(std::map<std::string, StampedCMap*>::iterator it = tempMaps_.begin();
        it != tempMaps_.end();
        it++) {
      delete it->second;
    }

    currentMaps_.clear();
    tempMaps_.clear();

    CMap uni;
    composeMapUnion(currentMaps_, uni);

    publishCollisionMap(uni, robotFrame_, ros::Time::now(), cmapPublisher_);

    return true;
  }
    
  bool transformMap(CMap &map, 
                    const std::string &from_frame_id,
                    const ros::Time &from_stamp,
                    const std::string &to_frame_id,
                    const ros::Time &to_stamp)
  {
    tf::StampedTransform transf;
    std::string err;
    ros::Time tm;
    ROS_DEBUG("Transforming old map");
    ROS_DEBUG_STREAM("From id: " << from_frame_id <<" at time " << from_stamp << " to id: " << to_frame_id << " at time " << to_stamp);
    if(from_frame_id == to_frame_id)
    {
      if(from_frame_id == fixedFrame_)
      {
        ROS_DEBUG("Nothing to transform, returning");
        return true;
      }
      try {
        tf_.lookupTransform(to_frame_id, to_stamp, from_frame_id, from_stamp, fixedFrame_, transf);
        ROS_DEBUG("Got transform!");
      } catch(...) {
        ROS_WARN("Unable to transform previous collision map into new frame");
        return false;
      }
    }
    else if (tf_.getLatestCommonTime(to_frame_id, from_frame_id, tm, &err) == tf::NO_ERROR) {
      try {
        tf_.lookupTransform(to_frame_id, tm, from_frame_id, tm, fixedFrame_, transf);
      } catch(...) {
        ROS_WARN("Unable to transform previous collision map into new frame");
        return false;
      }
    } else {
      ROS_WARN_STREAM("No common time between " << to_frame_id << " and " << from_frame_id);
    }

    float disp = transf.getOrigin().length();
    float angle = transf.getRotation().getAngle();

    ROS_DEBUG_STREAM("Old frame displaced by " << disp << " and angle " << angle << " to get new frame");

   
    // copy data to temporary location
    const int n = map.size();
    std::vector<CollisionPoint> pts(n);
    std::copy(map.begin(), map.end(), pts.begin());
    map.clear();
	
    //#pragma omp parallel for
    for (int i = 0 ; i < n ; ++i)
    {
      tf::Vector3 p(((double)pts[i].x - 0.5) * bi_.resolution + bi_.originX,
                  ((double)pts[i].y - 0.5) * bi_.resolution + bi_.originY,
                  ((double)pts[i].z - 0.5) * bi_.resolution + bi_.originZ);
      p = transf * p;
      if (p.x() > bi_.real_minX && p.x() < bi_.real_maxX && p.y() > bi_.real_minY && p.y() < bi_.real_maxY && p.z() > bi_.real_minZ && p.z() < bi_.real_maxZ)
      {
        CollisionPoint c((int)(0.5 + (p.x() - bi_.originX) / bi_.resolution),
                         (int)(0.5 + (p.y() - bi_.originY) / bi_.resolution),
                         (int)(0.5 + (p.z() - bi_.originZ) / bi_.resolution));
        //#pragma omp critical
        {
          map.insert(c);
        }
		
      }
    }
	
    return true;

  }

  /** Construct an axis-aligned collision map from a point cloud assumed to be in the robot frame */
  void constructCollisionMap(const sensor_msgs::PointCloud &cloud, CMap &map)
  {
    const unsigned int n = cloud.points.size();
    CollisionPoint c;
	
    for (unsigned int i = 0 ; i < n ; ++i)
    {
      const geometry_msgs::Point32 &p = cloud.points[i];
      if (p.x > bi_.real_minX && p.x < bi_.real_maxX && p.y > bi_.real_minY && p.y < bi_.real_maxY && p.z > bi_.real_minZ && p.z < bi_.real_maxZ)
      {
        c.x = (int)(0.5 + (p.x - bi_.originX) / bi_.resolution);
        c.y = (int)(0.5 + (p.y - bi_.originY) / bi_.resolution);
        c.z = (int)(0.5 + (p.z - bi_.originZ) / bi_.resolution);
        map.insert(c);
      }
    }
  }
    
  void publishCollisionMap(const CMap &map, 
                           const std::string &frame_id,
                           const ros::Time &stamp,
                           ros::Publisher &pub) const
  {
    arm_navigation_msgs::CollisionMap cmap;
    cmap.header.frame_id = frame_id;
    cmap.header.stamp = stamp;
    const unsigned int ms = map.size();
	
    for (CMap::const_iterator it = map.begin() ; it != map.end() ; ++it)
    {
      const CollisionPoint &cp = *it;
      arm_navigation_msgs::OrientedBoundingBox box;
      box.extents.x = box.extents.y = box.extents.z = bi_.resolution;
      box.axis.x = box.axis.y = 0.0; box.axis.z = 1.0;
      box.angle = 0.0;
      box.center.x = cp.x * bi_.resolution + bi_.originX;
      box.center.y = cp.y * bi_.resolution + bi_.originY;
      box.center.z = cp.z * bi_.resolution + bi_.originZ;
      cmap.boxes.push_back(box);
    } 
    pub.publish(cmap);
    
    ROS_DEBUG("Published collision map with %u boxes", ms);
  }

  void makeStaticCollisionMap(const arm_navigation_msgs::MakeStaticCollisionMapGoalConstPtr& goal) {
    
    if(cloud_source_map_.find(goal->cloud_source) == cloud_source_map_.end())
    {
      ROS_ERROR_STREAM("Could not make static map for source: "<<goal->cloud_source);
      ROS_ERROR_STREAM("Currently making maps from the following sources:");

      for(std::map<std::string,CloudInfo>::iterator it = cloud_source_map_.begin(); 
	  it != cloud_source_map_.end();
          it++) 
      {
        ROS_ERROR_STREAM(it->first);
      }	
      action_server_->setAborted();
      return;
    }

    static_map_goal_ = new arm_navigation_msgs::MakeStaticCollisionMapGoal(*goal);
    cloud_count_ = 0;
    making_static_collision_map_ = true;
    disregard_first_message_ = true;

    ros::Rate r(10);
    while(making_static_collision_map_) {
      if(action_server_->isPreemptRequested()) {
        making_static_collision_map_ = false;
	// Clean up tempMaps before breaking
	std::string map_name = static_map_goal_->cloud_source + "_static_temp";
	if(tempMaps_.find(map_name) != tempMaps_.end())
	{
	  delete tempMaps_[map_name];  
    	  tempMaps_.erase(map_name);
    	}

	break;
      }      
      r.sleep();
    }

    if(publish_over_dynamic_map_)
    {
     // loop through cloud_source_map, set dynamic publish to false on all sources to preserve behavior of older collision_map_self_occ
      for(std::map<std::string,CloudInfo>::iterator it = cloud_source_map_.begin(); 
	  it != cloud_source_map_.end();
          it++) 
      {
        it->second.dynamic_publish_ = false;	
      }	
    }

    delete static_map_goal_;
    if(action_server_->isPreemptRequested()) {
      action_server_->setPreempted();
    } else {
      action_server_->setSucceeded();
    }
  }

  boost::recursive_mutex                                  mapProcessing_;
  
  tf::TransformListener                         tf_;
  //robot_self_filter::SelfMask                  *sm_;
  //filters::SelfFilter<sensor_msgs::PointCloud> *self_filter_;
  std::vector<message_filters::Subscriber<sensor_msgs::PointCloud>* > mn_cloud_tf_sub_vector_;
  std::vector<tf::MessageFilter<sensor_msgs::PointCloud>* > mn_cloud_tf_fil_vector_;
  //tf::MessageNotifier<sensor_msgs::PointCloud> *mnCloudIncremental_;
  ros::NodeHandle                               root_handle_;
  ros::Publisher                                cmapPublisher_;
  ros::Publisher                                cmapUpdPublisher_;
  ros::Publisher static_map_publisher_;           
  std::map<std::string, ros::Publisher>         occPublisherMap_;
  ros::ServiceServer                            resetService_;
  bool                                          publishOcclusion_;
    
  arm_navigation_msgs::MakeStaticCollisionMapGoal *static_map_goal_;
  bool making_static_collision_map_;
  bool publish_over_dynamic_map_;
  bool disregard_first_message_;
  int cloud_count_;


  std::map<std::string, std::list<StampedCMap*> >                  currentMaps_;  //indexed by frame_ids
  std::map<std::string, StampedCMap*>                  			tempMaps_;  //indexed by frame_ids_static_save
    
  BoxInfo                                       bi_;
  std::string                                   fixedFrame_;
  std::string                                   robotFrame_;

  std::map<std::string,CloudInfo> cloud_source_map_;
  std::map<std::string,bool> static_map_published_;
  boost::shared_ptr<actionlib::SimpleActionServer<arm_navigation_msgs::MakeStaticCollisionMapAction> > action_server_;	
  
  ros::ServiceServer get_settings_server_;
  ros::ServiceServer set_settings_server_;
  

  
};  

void spinThread()
{
  ros::spin();
}
  
int main (int argc, char** argv)
{
  ros::init(argc, argv, "collision_map_self_occ");

  boost::thread spin_thread(&spinThread);

  CollisionMapperOcc cm;
  
  while(ros::ok()) {
    ros::Duration(0.1).sleep();    
  }

  ros::shutdown();
  spin_thread.join();
    
  return 0;
}
