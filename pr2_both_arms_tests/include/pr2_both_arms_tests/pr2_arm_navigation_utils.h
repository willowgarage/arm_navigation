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
 *   * Neither the name of Willow Garage nor the names of its
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
 *
 *********************************************************************/

/* \author: Sachin Chitta */

#include <ros/ros.h>
#include <arm_navigation_msgs/MakeStaticCollisionMapAction.h>
#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/Shape.h>
#include <pr2_msgs/SetPeriodicCmd.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_datatypes.h>


namespace arm_navigation_utils
{

static const std::string MAKE_STATIC_COLLISION_MAP_ACTION_NAME = "make_static_collision_map";
static const std::string TILT_LASER_PERIODIC_COMMAND_NAME = "laser_tilt_controller/set_periodic_cmd";

class ArmNavigationUtils
{
public:

  ArmNavigationUtils() :
    root_nh_(""),
    make_static_collision_map_client_(root_nh_, MAKE_STATIC_COLLISION_MAP_ACTION_NAME, true)
  {
    while ( !ros::service::waitForService(TILT_LASER_PERIODIC_COMMAND_NAME, ros::Duration(2.0)) && 
            root_nh_.ok() ) 
    {
      ROS_INFO("Waiting for tilt laser periodic command service to come up");
    }
    tilt_laser_service_ = root_nh_.serviceClient<pr2_msgs::SetPeriodicCmd>(TILT_LASER_PERIODIC_COMMAND_NAME);
    
    while(!make_static_collision_map_client_.waitForServer(ros::Duration(2.0))
          && root_nh_.ok()){
      ROS_INFO("Waiting for the make static collision map action to come up");
    }

    //cribbed from motion planning laser settings
    laser_slow_period_ = 5;
    laser_slow_amplitude_ = 1.02;
    laser_slow_offset_ = .31;
    
    //cribbed from head cart tilt_laser_launch
    laser_fast_period_ = 2;
    laser_fast_amplitude_ = .6;
    laser_fast_offset_ = .25;	    
  }
  ~ArmNavigationUtils(){}

  bool setLaserScan(bool fast) 
  {
    pr2_msgs::SetPeriodicCmd::Request req;
    pr2_msgs::SetPeriodicCmd::Response res;
    
    req.command.profile = "linear";
    if(fast) 
    {
      req.command.period = laser_fast_period_;
      req.command.amplitude = laser_fast_amplitude_;
      req.command.offset = laser_fast_offset_;
    } 
    else 
    {
      req.command.period = laser_slow_period_;
      req.command.amplitude = laser_slow_amplitude_;
      req.command.offset = laser_slow_offset_;
    }
    
    if(!tilt_laser_service_.call(req,res))
    {
      ROS_ERROR("Tilt laser service call failed.\n");
      return false;
    }
    return true;
  }

  bool takeStaticMap()
  {
    arm_navigation_msgs::MakeStaticCollisionMapGoal static_map_goal;
    static_map_goal.cloud_source = "full_cloud_filtered";
    static_map_goal.number_of_clouds = 2;
  
    make_static_collision_map_client_.sendGoal(static_map_goal);
    if(!setLaserScan(false))
    {
      ROS_ERROR("Could not take laser scan");
      return false;
    }
    if ( !make_static_collision_map_client_.waitForResult(ros::Duration(30.0)) )
    {
      ROS_ERROR("Collision map was not formed in allowed time");
      return false;
    }      
    if(make_static_collision_map_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) 
    {
      ROS_ERROR("Some non-success state was reached for static collision map.  Proceed with caution");
      return false;
    }       
    ROS_INFO("Successful taking static map");
    ros::Duration(1.0).sleep();
    return true;
  }
  
private:
  //! The private node handle
  ros::NodeHandle root_nh_;
  //! Client for changing laser scan speed
  ros::ServiceClient tilt_laser_service_;

  // Parameters for slow laser movement (high-res scanning)
  double laser_slow_period_;
  double laser_slow_amplitude_;
  double laser_slow_offset_;

  // Parameters for fast lsaer movement (low-res scanning)
  double laser_fast_period_;
  double laser_fast_amplitude_;
  double laser_fast_offset_;

  //! Action client for static map
  actionlib::SimpleActionClient<arm_navigation_msgs::MakeStaticCollisionMapAction>   make_static_collision_map_client_;
};

}
