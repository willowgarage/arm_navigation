/*********************************************************************
 *
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
 *
 *  \author Sachin Chitta
 *********************************************************************/

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <boost/thread.hpp>
#include <sys/time.h>

#include <arm_navigation_msgs/OrientedBoundingBox.h>
#include <arm_navigation_msgs/CollisionMap.h>

#include <gtest/gtest.h>

static const std::string COLLISION_MAP_TOPIC = "/collision_map_occ";
static const std::string COLLISION_MAP_FRAME = "/odom";

static const double MIN_Z_THRESHOLD = -0.1;
static const double MAX_Z_THRESHOLD = 0.1;

static const int TEST_NUM_MSGS = 2;

namespace collision_map
{
  class CollisionMapTest
  {
    public:
      ros::NodeHandle node_;
      ros::NodeHandle private_handle_;
      bool result_;
      int num_msgs_;
      int test_num_msgs_;
      tf::TransformListener tf_;
      std::string collision_map_frame_;
      double min_z_threshold_,max_z_threshold_;
      tf::MessageFilter<arm_navigation_msgs::CollisionMap>* cloud_notifier_;
      message_filters::Subscriber<arm_navigation_msgs::CollisionMap>* cloud_subscriber_;

      bool done_;

      CollisionMapTest(): private_handle_("~")
      {
        done_ = false;
        num_msgs_ = 0;
        result_ = true;

        //  ROS_INFO("Private handle: %s, %s",private_handle_.getName(), private_handle_.getNamespace());

        private_handle_.param<double>("min_z_threshold",min_z_threshold_,MIN_Z_THRESHOLD);
        private_handle_.param<double>("max_z_threshold",max_z_threshold_,MAX_Z_THRESHOLD);
        private_handle_.param<int>("test_num_collision_msgs",test_num_msgs_,TEST_NUM_MSGS);

        private_handle_.param<std::string>("collision_map_frame",collision_map_frame_,COLLISION_MAP_FRAME);

        cloud_subscriber_ = new message_filters::Subscriber<arm_navigation_msgs::CollisionMap>(node_,COLLISION_MAP_TOPIC,50);
        cloud_notifier_ = new tf::MessageFilter<arm_navigation_msgs::CollisionMap>(*cloud_subscriber_,tf_,collision_map_frame_,50);
        cloud_notifier_->registerCallback(boost::bind(&CollisionMapTest::collisionCallback,this,_1));

      }

      void collisionCallback(const arm_navigation_msgs::CollisionMapConstPtr& msg)
      {        
        num_msgs_++;
        if(num_msgs_ < test_num_msgs_)
        {
          ROS_INFO("Got collision map update %d",num_msgs_);
          ROS_INFO("Waiting for %d updates before running test\n",test_num_msgs_);
          return;
        }
        ROS_INFO("Got collision map update %d, Running test.",num_msgs_);

        arm_navigation_msgs::CollisionMap map = *msg;
        for(int i=0; i < (int) map.boxes.size(); i++)
        {
          if(map.boxes[i].center.z > max_z_threshold_ || map.boxes[i].center.z < min_z_threshold_)
          {
            result_ = false;
            break;
          }
        }
        EXPECT_TRUE(result_);
        done_ = true;
      }      
};
}

void spinThread()
{
  ros::spin();
}

TEST(CollisionMapTest, collisionMapGroundPlaneTest)
{
  boost::thread spin_thread(&spinThread);
  collision_map::CollisionMapTest cmap;
  ros::Duration dd(1.0);   
  ros::NodeHandle nh;

  while (nh.ok() && !cmap.done_)
  {
    dd.sleep();
  }
  ros::shutdown();
  spin_thread.join();
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init (argc, argv, "collision_map_test");
  return RUN_ALL_TESTS();
}
