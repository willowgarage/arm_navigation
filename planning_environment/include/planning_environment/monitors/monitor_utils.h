/*********************************************************************
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
*********************************************************************/

/** \author E. Gil Jones */

#ifndef _MONITOR_UTILS_H_
#define _MONITOR_UTILS_H_

#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <arm_navigation_msgs/Shape.h>
#include <geometric_shapes/shape_operations.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/AttachedCollisionObject.h>
#include <planning_environment/models/collision_models.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace planning_environment 
{

bool getLatestIdentityTransform(const std::string& to_frame,
                                const std::string& from_frame,
                                tf::TransformListener& tf,
                                tf::Transform& pose); 

bool createAndPoseShapes(tf::TransformListener& tf, 
                         const std::vector<arm_navigation_msgs::Shape>& orig_shapes,
                         const std::vector<geometry_msgs::Pose>& orig_poses,
                         const std_msgs::Header& header, 
                         const std::string& frame_to,
                         std::vector<shapes::Shape*>& conv_shapes,
                         std::vector<tf::Transform>& conv_poses);

bool processCollisionObjectMsg(const arm_navigation_msgs::CollisionObjectConstPtr &collision_object,
                               tf::TransformListener& tf,
                               CollisionModels* cm);

bool processAttachedCollisionObjectMsg(const arm_navigation_msgs::AttachedCollisionObjectConstPtr &attached_object,
                                       tf::TransformListener& tf,
                                       CollisionModels* cm);

void updateAttachedObjectBodyPoses(planning_environment::CollisionModels* cm,
                                   planning_models::KinematicState& state,
                                   tf::TransformListener& tf);

bool computeAttachedObjectPointCloudMask(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud,
                                         const std::string& sensor_frame,
                                         CollisionModels* cm,
                                         tf::TransformListener& tf,
                                         std::vector<int> &mask);

void updateAttachedObjectBodyPoses(planning_environment::CollisionModels* cm,
                                   planning_models::KinematicState& state,
                                   tf::TransformListener& tf);

bool configureForAttachedBodyMask(planning_models::KinematicState& state,
                                  planning_environment::CollisionModels* cm,
                                  tf::TransformListener& tf,
                                  const std::string& sensor_frame,
                                  const ros::Time& sensor_time,
                                  tf::Vector3& sensor_pos);

int computeAttachedObjectPointMask(const planning_environment::CollisionModels* cm,
                                   const tf::Vector3 &pt, 
                                   const tf::Vector3 &sensor_pos);


int closestStateOnTrajectory(const boost::shared_ptr<urdf::Model> &model,
                             const trajectory_msgs::JointTrajectory &trajectory, 
                             const sensor_msgs::JointState &joint_state, 
                             unsigned int start, 
                             unsigned int end);

bool removeCompletedTrajectory(const boost::shared_ptr<urdf::Model> &model,
                               const trajectory_msgs::JointTrajectory &trajectory_in, 
                               const sensor_msgs::JointState& current_state, 
                               trajectory_msgs::JointTrajectory &trajectory_out, 
                               bool zero_vel_acc);
}
#endif
