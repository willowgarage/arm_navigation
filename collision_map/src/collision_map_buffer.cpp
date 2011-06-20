/*
 * Copyright (c) 2008 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

/**
   @mainpage

   @htmlinclude manifest.html

   \author Radu Bogdan Rusu

   @b collision_map_buffer is a node providing a map of the occupied space around the robot as discretized boxes (center,
   dimension, orientation) useful for collision detection.

**/

// ROS core
#include <ros/ros.h>
// ROS messages
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Core>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/transforms.h>

#include <boost/thread/mutex.hpp>

#include <mapping_msgs/OrientedBoundingBox.h>
#include <mapping_msgs/CollisionMap.h>
#include <tabletop_srvs/RecordStaticMapTrigger.h>
#include <tabletop_srvs/SubtractObjectFromCollisionMap.h>

#include <tf/transform_listener.h>
#include <sys/time.h>

using namespace std;
using namespace sensor_msgs;
using namespace mapping_msgs;
using namespace tabletop_srvs;
using namespace visualization_msgs;

static const unsigned int MAX_CLOUD_SOURCES = 3;

struct CloudProcSettings {
  string cloud_name_;
  int frame_subsample_; //take every nth frame (1 is no discards)
  int point_subsample_; //take every nth point from the frames you do process
};

struct Leaf
{
  int i_, j_, k_;
  int nr_points_;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
compareLeaf (const Leaf &l1, const Leaf &l2)
{
  if (l1.i_ < l2.i_)
    return (true);
  else if (l1.i_ > l2.i_)
    return (false);
  else if (l1.j_ < l2.j_)
    return (true);
  else if (l1.j_ > l2.j_)
    return (false);
  else if (l1.k_ < l2.k_)
    return (true);
  else
    return (false);
}

class CollisionMapperBuffer
{
protected:
  ros::NodeHandle& node_;

public:

  // ROS messages
  sensor_msgs::PointCloud cloud_;
  CollisionMap final_collision_map_;

  // Internal map representations
  vector<Leaf> static_leaves_, cur_leaves_, final_leaves_;
  list<std::pair<vector<Leaf>, ros::Time> > decaying_maps_;

  // TF
  tf::TransformListener tf_;

  // Parameters
  geometry_msgs::Point leaf_width_, robot_max_;
  int min_nr_points_;
  string end_effector_frame_l_, end_effector_frame_r_;
  string shared_frame_;

  // Mutices
  boost::mutex static_map_lock_, object_subtract_lock_, cloud_frame_lock_, m_lock_;

  // The time for the buffer window
  double window_time_;
  bool acquire_static_map_;
  ros::Time acquire_static_map_time_;

  // Internal parameters
  string cloud_frame_;
  geometry_msgs::PoseStamped gripper_orientation_link_;
  geometry_msgs::Point32 min_object_b_, max_object_b_;
  bool subtract_object_;
  int m_id_;

  CloudProcSettings cloud_source_[MAX_CLOUD_SOURCES];
  unsigned int cloud_counter_[MAX_CLOUD_SOURCES];
  unsigned int num_actual_sources_;

  ros::Subscriber cloud_subscriber_[MAX_CLOUD_SOURCES];
  ros::Publisher collision_map_publisher_,visualization_publisher_;

  ros::ServiceServer record_static_map_service_;
  ros::ServiceServer subtract_object_service_;

 

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  CollisionMapperBuffer (ros::NodeHandle& anode) : node_ (anode)
  {
    ros::NodeHandle priv("~");

    priv.param("cloud_source/zero/name", cloud_source_[0].cloud_name_, string("/full_laser_cloud"));
    priv.param("cloud_source/zero/frame_subsample", cloud_source_[0].frame_subsample_, 1);
    priv.param("cloud_source/zero/point_subsample", cloud_source_[0].point_subsample_,1);
    num_actual_sources_ = 1;
    if(priv.hasParam ("cloud_source/one")) 
    {
      priv.param("cloud_source/one/name", cloud_source_[1].cloud_name_, string("/full_stereo_cloud"));
      priv.param("cloud_source/one/frame_subsample", cloud_source_[1].frame_subsample_, 4);
      priv.param("cloud_source/one/point_subsample", cloud_source_[1].point_subsample_,4);
      num_actual_sources_ = 2;
      if(priv.hasParam("cloud_source/two")) 
      {
        priv.param("cloud_source/two/name", cloud_source_[2].cloud_name_, string("/full_bonus_cloud"));
        priv.param("cloud_source/two/frame_subsample", cloud_source_[2].frame_subsample_, 4);
        priv.param("cloud_source/two/point_subsample", cloud_source_[2].point_subsample_,4);
        num_actual_sources_ = 3;
      }
    } 
//     priv.param("cloud_source_1", cloud_source_[1].cloud_name_, string("/full_stereo_cloud"));
//     priv.param("cloud_0_frame_subsample", 
//     priv.param("cloud_1_frame_subsample", cloud_source_[1].frame_subsample_, 1);
//     priv.param("cloud_0_point_subsample", cloud_source_[0].point_subsample_, 4);
//     priv.param("cloud_1_point_subsample", cloud_source_[1].point_subsample_, 4);
    priv.param ("leaf_width_x", leaf_width_.x, 0.02);       // 2cm diameter by default
    priv.param ("leaf_width_y", leaf_width_.y, 0.02);       // 2cm diameter by default
    priv.param ("leaf_width_z", leaf_width_.z, 0.02);       // 2cm diameter by default

    priv.param ("robot_max_x", robot_max_.x, 1.5);           // 1.5m radius by default
    priv.param ("robot_max_y", robot_max_.y, 1.5);           // 1.5m radius by default
    priv.param ("robot_max_z", robot_max_.z, 1.5);           // 1.5m radius by default

    priv.param ("min_nr_points", min_nr_points_, 1);         // Need at least 1 point per box to consider it "occupied"

    ROS_INFO ("Using a default leaf of size: %g,%g,%g.", leaf_width_.x, leaf_width_.y, leaf_width_.z);
    ROS_INFO ("Using a maximum bounding box around the robot of size: %g,%g,%g.", robot_max_.x, robot_max_.y, robot_max_.z);

    priv.param ("window_time", window_time_, 5.0);             // Use 5 seconds of scans by default
    priv.param ("end_effector_frame_l", end_effector_frame_l_, string ("r_gripper_l_finger_tip_link"));     // The frame of the end effector (used for object subtraction)
    priv.param ("end_effector_frame_r", end_effector_frame_r_, string ("r_gripper_r_finger_tip_link"));     // The frame of the end effector (used for object subtraction)

    priv.param ("shared_frame", shared_frame_, string("base_link")); //all points get projected into this frame

    // Square the limits (simplified point distances below)
    robot_max_.x = robot_max_.x * robot_max_.x;
    robot_max_.y = robot_max_.y * robot_max_.y;
    robot_max_.z = robot_max_.z * robot_max_.z;

    acquire_static_map_ = false;                        // Do not acquire a static map

    //  string cloud_topic ("/full_cloud");

    //       std::vector<ros::master::TopicInfo> t_list;
    //       bool topic_found = false;
    //       ros::master::getTopics (t_list);
    //       for (vector<ros::master::TopicInfo>::iterator it = t_list.begin (); it != t_list.end (); it++)
    //       {
    //         if (it->name == cloud_topic)
    //         {
    //           topic_found = true;
    //           break;
    //         }
    //       }
    //       if (!topic_found)
    //         ROS_WARN ("Trying to subscribe to %s, but the topic doesn't exist!", cloud_topic.c_str ());

    ROS_DEBUG("Cloud 0 name %s 1 name %s", cloud_source_[0].cloud_name_.c_str(),cloud_source_[1].cloud_name_.c_str());

    cloud_subscriber_[0] = node_.subscribe (cloud_source_[0].cloud_name_, 1, &CollisionMapperBuffer::cloudCb0, this);
    cloud_counter_[0] = 0;
    
    if(num_actual_sources_ > 1) {
      cloud_subscriber_[1] = node_.subscribe (cloud_source_[1].cloud_name_, 1, &CollisionMapperBuffer::cloudCb1, this);
      cloud_counter_[1] = 0;
    }

    if(num_actual_sources_ > 2) {
      cloud_subscriber_[2] = node_.subscribe (cloud_source_[2].cloud_name_, 1, &CollisionMapperBuffer::cloudCb2, this);
      cloud_counter_[2] = 0;
    }

    collision_map_publisher_ = node_.advertise<CollisionMap> ("collision_map_buffer", 1);

    record_static_map_service_ = node_.advertiseService ("record_static_map", &CollisionMapperBuffer::getStaticMap, this);
    subtract_object_service_ = node_.advertiseService ("subtract_object", &CollisionMapperBuffer::subtractObject, this);

    // Gripper orientation/position
    gripper_orientation_link_.pose.orientation.x = 0.0;
    gripper_orientation_link_.pose.orientation.y = 0.0;
    gripper_orientation_link_.pose.orientation.z = 0.0;
    gripper_orientation_link_.pose.orientation.w = 1.0;
    subtract_object_ = false;

    m_id_ = 0;
    visualization_publisher_ = node_.advertise<visualization_msgs::Marker>("visualization_marker", 100);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  virtual ~CollisionMapperBuffer () { }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void
  updateParametersFromServer ()
  {
    ros::NodeHandle priv("~");



    if (priv.hasParam ("leaf_width_x")) priv.getParam ("leaf_width_x", leaf_width_.x);
    if (priv.hasParam ("leaf_width_y")) priv.getParam ("leaf_width_y", leaf_width_.y);
    if (priv.hasParam ("leaf_width_z")) priv.getParam ("leaf_width_z", leaf_width_.z);

    if (priv.hasParam ("window_time")) priv.getParam ("window_time", window_time_);

    if (priv.hasParam ("robot_max_x"))
    {
      double rx;
      priv.getParam ("robot_max_x", rx);
      robot_max_.x = rx * rx;
    }
    if (priv.hasParam ("robot_max_y"))
    {
      double ry;
      priv.getParam ("robot_max_y", ry);
      robot_max_.y = ry * ry;
    }
    if (priv.hasParam ("robot_max_z"))
    {
      double rz;
      priv.getParam ("robot_max_z", rz);
      robot_max_.z = rz * rz;
    }

    if (priv.hasParam ("min_nr_points")) priv.getParam ("min_nr_points", min_nr_points_);
  }


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Compute a CollisionMap from a Leaf vector.
   * \param leaves the Leaf vector
   * \param cmap the resultant collision map
   */
  void
  computeCollisionMapFromLeaves (vector<Leaf> *leaves, CollisionMap &cmap)
  {
    cmap.boxes.resize (leaves->size ());
    // Second pass: go over all leaves and add them to the map
    int nr_c = 0;
    for (unsigned int cl = 0; cl < leaves->size (); cl++)
    {
      if (leaves->at (cl).nr_points_ >= min_nr_points_)
      {
        cmap.boxes[nr_c].extents.x = leaf_width_.x / 2.0;
        cmap.boxes[nr_c].extents.y = leaf_width_.y / 2.0;
        cmap.boxes[nr_c].extents.z = leaf_width_.z / 2.0;
        cmap.boxes[nr_c].center.x = (leaves->at (cl).i_ + 1) * leaf_width_.x - cmap.boxes[nr_c].extents.x; // + minB.x;
        cmap.boxes[nr_c].center.y = (leaves->at (cl).j_ + 1) * leaf_width_.y - cmap.boxes[nr_c].extents.y; // + minB.y;
        cmap.boxes[nr_c].center.z = (leaves->at (cl).k_ + 1) * leaf_width_.z - cmap.boxes[nr_c].extents.z; // + minB.z;
        cmap.boxes[nr_c].axis.x = cmap.boxes[nr_c].axis.y = cmap.boxes[nr_c].axis.z = 0.0;
        cmap.boxes[nr_c].angle = 0.0;
        nr_c++;
      }
    }
    cmap.boxes.resize (nr_c);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Compute a Leaf vector from an unorganized PointCloud
   * \param points the PointCloud message
   * \param leaves the resultant Leaf vector
   * \param centers a resultant PointCloud message containing the centers of the leaves
   */
  void
  computeLeaves (sensor_msgs::PointCloud *points, vector<Leaf> &leaves, sensor_msgs::PointCloud &centers)
  {
    geometry_msgs::PointStamped base_origin, torso_lift_origin;
    base_origin.point.x = base_origin.point.y = base_origin.point.z = 0.0;
    base_origin.header.frame_id = "torso_lift_link";
    base_origin.header.stamp = ros::Time ();

    try
    {
      tf_.transformPoint (points->header.frame_id, base_origin, torso_lift_origin);
      //ROS_INFO ("Robot 'origin' is : %g,%g,%g", torso_lift_origin.point.x, torso_lift_origin.point.y, torso_lift_origin.point.z);
    }
    catch (tf::ConnectivityException)
    {
      ROS_ERROR ("TF not running or wrong TF frame specified! Defaulting to 0,0,0.");
      torso_lift_origin = base_origin;
    }
    // Get a set of point indices that respect our bounding limits around the robot
    vector<int> indices (cloud_.points.size ());
    int nr_p = 0;

    geometry_msgs::Point32 minP, maxP;
    minP.x = minP.y = minP.z = FLT_MAX;
    maxP.x = maxP.y = maxP.z = FLT_MIN;
    double distance_sqr_x, distance_sqr_y, distance_sqr_z;
    for (unsigned int i = 0; i < points->points.size (); i++)
    {
      // We split the "distance" on all 3 dimensions to allow greater flexibility
      distance_sqr_x = fabs ((points->points[i].x - torso_lift_origin.point.x) * (points->points[i].x - torso_lift_origin.point.x));
      distance_sqr_y = fabs ((points->points[i].y - torso_lift_origin.point.y) * (points->points[i].y - torso_lift_origin.point.y));
      distance_sqr_z = fabs ((points->points[i].z - torso_lift_origin.point.z) * (points->points[i].z - torso_lift_origin.point.z));

      // If the point is within the bounds, use it for minP/maxP calculations
      if (distance_sqr_x < robot_max_.x && distance_sqr_y < robot_max_.y && distance_sqr_z < robot_max_.z)
      {
        minP.x = (points->points[i].x < minP.x) ? points->points[i].x : minP.x;
        minP.y = (points->points[i].y < minP.y) ? points->points[i].y : minP.y;
        minP.z = (points->points[i].z < minP.z) ? points->points[i].z : minP.z;

        maxP.x = (points->points[i].x > maxP.x) ? points->points[i].x : maxP.x;
        maxP.y = (points->points[i].y > maxP.y) ? points->points[i].y : maxP.y;
        maxP.z = (points->points[i].z > maxP.z) ? points->points[i].z : maxP.z;
        indices[nr_p] = i;
        nr_p++;
      }
    }
    indices.resize (nr_p);

    // Compute the minimum and maximum bounding box values
    geometry_msgs::Point32 minB, maxB, divB;

    minB.x = (int)(floor (minP.x / leaf_width_.x));
    maxB.x = (int)(floor (maxP.x / leaf_width_.x));

    minB.y = (int)(floor (minP.y / leaf_width_.y));
    maxB.y = (int)(floor (maxP.y / leaf_width_.y));

    minB.z = (int)(floor (minP.z / leaf_width_.z));
    maxB.z = (int)(floor (maxP.z / leaf_width_.z));

    // Compute the number of divisions needed along all axis
    divB.x = maxB.x - minB.x + 1;
    divB.y = maxB.y - minB.y + 1;
    divB.z = maxB.z - minB.z + 1;

    // Allocate the space needed (+ extra)
    if (leaves.capacity () < divB.x * divB.y * divB.z)
      leaves.reserve (divB.x * divB.y * divB.z);

    leaves.resize (divB.x * divB.y * divB.z);

    for (unsigned int cl = 0; cl < leaves.size (); cl++)
    {
      if (leaves[cl].nr_points_ > 0)
        leaves[cl].i_ = leaves[cl].j_ = leaves[cl].k_ = leaves[cl].nr_points_ = 0;
    }

    // Return a point cloud message containing the centers of the leaves as well
    centers.header = points->header;
    centers.points.resize (indices.size ());
    float extents[3];
    extents[0] = leaf_width_.x / 2.0;
    extents[1] = leaf_width_.y / 2.0;
    extents[2] = leaf_width_.z / 2.0;

    // First pass: go over all points and count them into the right leaf
    int i = 0, j = 0, k = 0;
    for (unsigned int cp = 0; cp < indices.size (); cp++)
    {
      i = (int)(floor (points->points[indices.at (cp)].x / leaf_width_.x));
      j = (int)(floor (points->points[indices.at (cp)].y / leaf_width_.y));
      k = (int)(floor (points->points[indices.at (cp)].z / leaf_width_.z));

      int idx = ( (k - minB.z) * divB.y * divB.x ) + ( (j - minB.y) * divB.x ) + (i - minB.x);
      leaves[idx].i_ = i;
      leaves[idx].j_ = j;
      leaves[idx].k_ = k;
      leaves[idx].nr_points_++;

      // Get the point
      centers.points[cp].x = (i + 1) * leaf_width_.x - extents[0];
      centers.points[cp].y = (j + 1) * leaf_width_.y - extents[1];
      centers.points[cp].z = (k + 1) * leaf_width_.z - extents[2];
    }

    sort (leaves.begin (), leaves.end (), compareLeaf);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void
  sendMarker (geometry_msgs::Point32 pt, const std::string &frame_id, double radius = 0.02)
  {
    Marker mk;
    mk.header.stamp = ros::Time::now();

    mk.header.frame_id = frame_id;

    mk.ns = "collision_map_buffer";
    mk.id = ++m_id_;
    mk.type = Marker::SPHERE;
    mk.action = Marker::ADD;
    mk.pose.position.x = pt.x;
    mk.pose.position.y = pt.y;
    mk.pose.position.z = pt.z;

    mk.pose.orientation.w = 1.0;
    mk.scale.x = mk.scale.y = mk.scale.z = radius * 2.0;

    mk.color.a = 1.0;
    mk.color.r = 1.0;
    mk.color.g = 0.04;
    mk.color.b = 0.04;

    visualization_publisher_.publish(mk);
  }


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Obtain the position of the end effector (center of the two fingers) in the required target frame.
   * \param tgt_frame the target TF frame
   * \param stamp the time stamp
   * \param center the resultant center position
   */
  inline bool
  getEndEffectorPosition (string tgt_frame, ros::Time stamp, geometry_msgs::Point32 &center)
  {
    geometry_msgs::PointStamped src, tgt;
    src.header.frame_id = end_effector_frame_l_;
    src.header.stamp    = stamp;

    src.point.x = src.point.y = src.point.z = 0.0;
    try
    {
      tf_.transformPoint (tgt_frame, src, tgt);
    }
    catch (tf::ConnectivityException)
    {
      ROS_ERROR ("TF not running or wrong TF end_effector_frame specified!");
      return (false);
    }
    catch (tf::ExtrapolationException)
    {
      ROS_ERROR("Extrapolation exception from %s to %s.", tgt_frame.c_str(), src.header.frame_id.c_str());
    }

    center.x = tgt.point.x; center.y = tgt.point.y; center.z = tgt.point.z;

    src.header.frame_id = end_effector_frame_r_;
    try
    {
      tf_.transformPoint (tgt_frame, src, tgt);
    }
    catch (tf::ConnectivityException)
    {
      ROS_ERROR ("TF not running or wrong TF end_effector_frame specified!");
      return (false);
    }
    catch (tf::ExtrapolationException)
    {
      ROS_ERROR("Extrapolation exception from %s to %s.", tgt_frame.c_str(), src.header.frame_id.c_str());
    }

    center.x += tgt.point.x; center.y += tgt.point.y; center.z += tgt.point.z;
    center.x /= 2.0;         center.y /= 2.0;         center.z /= 2.0;
    return (true);

  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Prune the leaves inside a given bounding box
   * \param leaves the set of leaves to prune data from
   * \param points the point cloud representing the centers of the leaves
   * \param center a point in the center of the gripper
   * \param source_frame the TF frame in which the points are represented
   * \param target_frame the TF frame in which the bounds are represented
   * \param min_b the minimum bounds of the box
   * \param max_b the maximum bounds of the box
   */
  void pruneLeaves (vector<Leaf> &leaves, sensor_msgs::PointCloud *points, geometry_msgs::Point32 *center, string source_frame, string target_frame,
                    geometry_msgs::Point32 *min_b, geometry_msgs::Point32 *max_b)
  {
    sensor_msgs::PointCloud points_tgt;

    // Transform the entire cloud at once
    try
    {
      tf_.transformPointCloud (target_frame, *points, points_tgt);
    }
    catch (tf::ConnectivityException)
    {
      ROS_ERROR ("TF not running or wrong TF end_effector_frame specified!");
      return;
    }
    catch (tf::ExtrapolationException)
    {
      ROS_ERROR("Extrapolation exception from %s to %s.", target_frame.c_str(), points->header.frame_id.c_str());
    }

    vector<int> object_indices (points_tgt.points.size ());
    int nr_p = 0;
    // Check and mark point indices in the bounds of the objects
    for (unsigned int i = 0; i < points_tgt.points.size (); i++)
    {
      if (points_tgt.points[i].x > min_b->x &&
          points_tgt.points[i].x < max_b->x &&
          points_tgt.points[i].y > min_b->y &&
          points_tgt.points[i].y < max_b->y &&
          points_tgt.points[i].z > min_b->z &&
          points_tgt.points[i].z < max_b->z)
      {
        object_indices[nr_p] = i;
        nr_p++;
      }
    }
    object_indices.resize (nr_p);

    // Copy the indices from object_indices into a temporary cloud
    sensor_msgs::PointCloud object_points;
    object_points.header = points_tgt.header;
    object_points.points.resize (object_indices.size ());
    for (unsigned int i = 0; i < object_indices.size (); i++)
    {
      object_points.points[i].x = points_tgt.points[object_indices.at (i)].x;
      object_points.points[i].y = points_tgt.points[object_indices.at (i)].y;
      object_points.points[i].z = points_tgt.points[object_indices.at (i)].z;
    }

   //  geometry_msgs::PointStamped ee_local, ee_global;      // Transform the end effector position in global (source frame)
//     ee_local.point.x = ee_local.point.y = ee_local.point.z = 0.0;
//     ee_local.header.frame_id = target_frame;
//     ee_local.header.stamp = points->header.stamp;

    // Transform the points back into the source frrame
    sensor_msgs::PointCloud points_src;
    try
    {
      tf_.transformPointCloud (source_frame, object_points, points_src);
      //tf_.transformPoint (source_frame, ee_local, ee_global);
    }
    catch (tf::ConnectivityException)
    {
      ROS_ERROR ("TF not running or wrong TF end_effector_frame specified!");
      return;
    }

    //ROS_DEBUG ("End effector position is: [%f, %f, %f].", ee_global.point.x, ee_global.point.y, ee_global.point.z);

    // Compute the leaves
    vector<Leaf> object_leaves;
    computeLeaves (&points_src, object_leaves, object_points);

    // Go over the leaves and subtract the ones on the object
    vector<Leaf> model_difference;
    set_difference (leaves.begin (), leaves.end (), object_leaves.begin (), object_leaves.end (),
                    inserter (model_difference, model_difference.begin ()), compareLeaf);
    leaves = model_difference;
  }

  double determineDecayingMapDuration() const {

    ros::Time min_time = ros::TIME_MAX;
    ros::Time max_time = ros::TIME_MIN;

    for( list<std::pair<vector<Leaf>, ros::Time> >::const_iterator dec_it = decaying_maps_.begin();
         dec_it != decaying_maps_.end();
         dec_it++) {
      if(dec_it->second < min_time) {
        min_time = dec_it->second;
      }
      if(dec_it->second > max_time) {
        max_time = dec_it->second;
      }
    }
    ros::Duration dur = max_time-min_time;
    return dur.toSec();
  }

  void discardOutOfTimeWindowScans() {
    ros::Time max_time = ros::TIME_MIN;
    
    for( list<std::pair<vector<Leaf>, ros::Time> >::const_iterator dec_cit = decaying_maps_.begin();
         dec_cit != decaying_maps_.end();
         dec_cit++) {
      if(dec_cit->second > max_time) {
        max_time = dec_cit->second;
      }
    }
    
    ros::Time windowTime(max_time.toSec()-window_time_);

    list<std::pair<vector<Leaf>, ros::Time> >::iterator dec_it = decaying_maps_.begin();
    while(dec_it != decaying_maps_.end()) {
      if(dec_it->second < windowTime) {
        dec_it = decaying_maps_.erase(dec_it);
      } else {
        dec_it++;
      }
    }
  }

  void subsampleCloudPoints(const sensor_msgs::PointCloudConstPtr &msg, sensor_msgs::PointCloud& myMsg, unsigned int subsampleNum) {
    myMsg.points.clear();
    myMsg.header = (*msg).header;

    sensor_msgs::PointCloud points_tgt;

    // Transform the entire cloud at once
    try
    {
      tf_.transformPointCloud (shared_frame_, (*msg), points_tgt);
    }
    catch (tf::ConnectivityException)
    {
      ROS_ERROR ("TF not running or wrong TF end_effector_frame specified!");
      return;
    }
    catch (tf::ExtrapolationException)
    {
      ROS_ERROR("Extrapolation exception.");
    }
    myMsg.header.frame_id=shared_frame_;
    //myMsg.channels.clear();        
    for(unsigned int i = 0; i < points_tgt.points.size(); i += subsampleNum) {
      myMsg.points.push_back(points_tgt.points[i]);
      //if((*msg).channels.size() < i) {
      //  myMsg.channels.push_back((*msg).channels[i]);
      //}
    }
  }
  

  //this will be unnecessary once messages carry some meta_information
  void cloudCb0 (const sensor_msgs::PointCloudConstPtr &msg) 
  {
    //here we do the frame subsampling, where we process the first frame
    if(cloud_counter_[0]++ % cloud_source_[0].frame_subsample_ == 0) 
    {
      cloudCb(msg, cloud_source_[0]);
    }
  }

  void cloudCb1 (const sensor_msgs::PointCloudConstPtr &msg) 
  {
    //here we do the frame subsampling, where we process the first frame
    if(cloud_counter_[1]++ % cloud_source_[1].frame_subsample_ == 0) 
    {
      cloudCb(msg, cloud_source_[1]);
    }
  }

  void cloudCb2 (const sensor_msgs::PointCloudConstPtr &msg) 
  {
    //here we do the frame subsampling, where we process the first frame
    if(cloud_counter_[2]++ % cloud_source_[2].frame_subsample_ == 0) 
    {
      cloudCb(msg, cloud_source_[2]);
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Pointcloud callback function */
  void cloudCb (const sensor_msgs::PointCloudConstPtr &msg, const CloudProcSettings& settings)
  {
    ros::Time t1, t2, t3;
    t1 = ros::Time::now ();
    ROS_DEBUG ("Received %u data points.",(*msg).points.size ());
    subsampleCloudPoints(msg, cloud_,settings.point_subsample_);
    ROS_DEBUG ("After subsampling we have %u data points.",cloud_.points.size ());

    // Get the new parameters from the server
    //m_lock_.lock ();
    //updateParametersFromServer ();
    //m_lock_.unlock ();

    //timeval t1, t2;

    double time_spent;

    // Get the position of the end effector
    geometry_msgs::Point32 ee_center;
    //if (!getEndEffectorPosition (cloud_.header.frame_id, cloud_.header.stamp, ee_center))
    //  return;
    //       sendMarker (ee_center, cloud_.header.frame_id);

    // Copy the header (and implicitly the frame_id)
    final_collision_map_.header = cloud_.header;

    // Static map acquisition has been triggered via the service call
    if (acquire_static_map_)
    {
      // Do not compute any collision maps until we receive a cloud with a higher timestamp
      if (cloud_.header.stamp < acquire_static_map_time_)
        return;

      // Compute the static collision map
      //gettimeofday (&t1, NULL);


      // We do not subtract anything when we compute the static map
      sensor_msgs::PointCloud centers;
      computeLeaves (&cloud_, static_leaves_, centers);

      // Clear the static map flag
      static_map_lock_.lock ();
      acquire_static_map_ = false;
      static_map_lock_.unlock ();

      //gettimeofday (&t2, NULL);
      t2 = ros::Time::now ();
      //time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
      time_spent = (t2 - t1).toSec ();
      ROS_DEBUG ("Static collision map computed in %g seconds. Number of boxes: %u.", time_spent, (unsigned int)static_leaves_.size ());

    }
    else
    {
      vector<Leaf> model_reunion;
      // Rotate N maps in the queue
      //gettimeofday (&t1, NULL);
      //t1 = ros::Time::now ();

      // Compute the leaves for the current dataset
      sensor_msgs::PointCloud centers;
      m_lock_.lock ();
      computeLeaves (&cloud_, cur_leaves_, centers);
      m_lock_.unlock ();

      // Check the points against the object bounds
      object_subtract_lock_.lock ();
      
      pruneLeaves (cur_leaves_, &centers, &ee_center, cloud_.header.frame_id, end_effector_frame_l_, &min_object_b_, &max_object_b_);
      object_subtract_lock_.unlock ();
      
      std::pair<vector<Leaf>, ros::Time> cur_leaves_with_time(cur_leaves_,(*msg).header.stamp);

      // Push the current leaves onto the queue
      decaying_maps_.push_back (cur_leaves_with_time);

      // If we have window_size maps, combine them together
      ROS_DEBUG("Duration before %g",determineDecayingMapDuration());
      //if(determineDecayingMapDuration() > window_time_) {
      final_leaves_.clear ();
      list<std::pair<vector<Leaf>, ros::Time> >::const_iterator it = decaying_maps_.begin ();
      for ( ; it != decaying_maps_.end (); ++it)
      {
        // Assume the models are sorted
        set_union (final_leaves_.begin (), final_leaves_.end (), (*it).first.begin (), (*it).first.end (),
                   inserter (model_reunion, model_reunion.begin ()), compareLeaf);
        final_leaves_ = model_reunion;
        model_reunion.clear ();
      }
      discardOutOfTimeWindowScans();
      ROS_DEBUG("Duration after %g",determineDecayingMapDuration());
      //}
      //else
      //  final_leaves_ = cur_leaves_;

      // Include the static map in the reunion
      set_union (final_leaves_.begin (), final_leaves_.end (), static_leaves_.begin (), static_leaves_.end (),
                 inserter (model_reunion, model_reunion.begin ()), compareLeaf);

      computeCollisionMapFromLeaves (&model_reunion, final_collision_map_);

      //gettimeofday (&t2, NULL);
      t2 = ros::Time::now ();
      //time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
      time_spent = (t2 - t1).toSec ();
      ROS_DEBUG ("Collision map with %u boxes computed in %g seconds. Total maps in the queue %d.",
                 (unsigned int)final_collision_map_.boxes.size (), time_spent, (int)decaying_maps_.size ());

      collision_map_publisher_.publish(final_collision_map_);
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief CollisionMapBuffer "record_static_map" service callback */
  bool getStaticMap (RecordStaticMapTrigger::Request &req, RecordStaticMapTrigger::Response &resp)
  {
    static_map_lock_.lock ();
    acquire_static_map_      = true;
    acquire_static_map_time_ = req.map_time;
    static_map_lock_.unlock ();

    ROS_INFO ("Got a request to compute a new static map at %f.", acquire_static_map_time_.toSec ());

    // Wait until the scan is ready, sleep for 10ms
    ros::Duration tictoc (0, 10000000);
    while (acquire_static_map_)
    {
      tictoc.sleep ();
    }

    resp.status = 0;      // success (!)

    return (true);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief CollisionMapBuffer "subtract_object" service callback */
  bool subtractObject (SubtractObjectFromCollisionMap::Request &req, SubtractObjectFromCollisionMap::Response &resp)
  {
    ROS_INFO ("Got request to subtract object.");
    geometry_msgs::Point32 center;
    center.x = (req.object.min_bound.x + req.object.max_bound.x) / 2.0;
    center.y = (req.object.min_bound.y + req.object.max_bound.y) / 2.0;
    center.z = (req.object.min_bound.z + req.object.max_bound.z) / 2.0;

    object_subtract_lock_.lock ();

    min_object_b_.x = req.object.min_bound.x - center.x;
    min_object_b_.y = req.object.min_bound.y - center.y;
    min_object_b_.z = req.object.min_bound.z - center.z;

    max_object_b_.x = req.object.max_bound.x - center.x;
    max_object_b_.y = req.object.max_bound.y - center.y;
    max_object_b_.z = req.object.max_bound.z - center.z;

    subtract_object_ = true;

    object_subtract_lock_.unlock ();

    resp.status = 0;      // success (!)

    return (true);
  }
};

/* ---[ */
int main (int argc, char** argv)
{
  ros::init (argc, argv, "collision_map_buffer");
  ros::NodeHandle ros_node("");
  CollisionMapperBuffer p (ros_node);

  //   RecordStaticMapTrigger::Request req;
  //   RecordStaticMapTrigger::Response resp;
  //   req.map_time = ros::Time::now ();
  //   ros::service::call ("record_static_map", req, resp);

  // Wait until the scan is ready, sleep for 1s
  ros::Duration tictoc (10.0, 0);
  tictoc.sleep ();

  // Box example: 22.2 cm x 10.5 cm x 5.8 cm
  /*  SubtractObjectFromCollisionMap::Request req;
      req.object.min_bound.x = req.object.min_bound.y = req.object.min_bound.z = 0.0;
      req.object.max_bound.z = 0.35; //0.222;
      req.object.max_bound.x = 0.105 * 2;
      req.object.max_bound.y = 0.058 * 4;
      SubtractObjectFromCollisionMap::Response resp;
      ros::service::call ("~subtract_object", req, resp);
  */
  ros::spin ();

  return (0);
}
/* ]--- */

