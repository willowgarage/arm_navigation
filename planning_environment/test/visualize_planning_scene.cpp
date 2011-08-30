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

#include <ros/ros.h>
#include <planning_environment/monitors/planning_monitor.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
#include <arm_navigation_msgs/GetRobotState.h>
#include <planning_environment/models/model_utils.h>
#include <ros/package.h>

static const std::string vis_topic_name = "planning_scene";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualize_planning_scene");
 
  ros::NodeHandle nh;
  
  ros::Publisher vis_marker_publisher = nh.advertise<visualization_msgs::Marker>(vis_topic_name, 128);
  ros::Publisher vis_marker_array_publisher = nh.advertise<visualization_msgs::MarkerArray>(vis_topic_name+"_array", 128);

  std::string robot_description_name = nh.resolveName("robot_description", true);

  planning_environment::CollisionModels cmodel(robot_description_name);

  arm_navigation_msgs::PlanningScene planning_scene;

  cmodel.readPlanningSceneBag(ros::package::getPath("planning_environment")+"/test_table.bag",
                              planning_scene);
  planning_models::KinematicState* state = cmodel.setPlanningScene(planning_scene);
  
  std::map<std::string, double> vals;
  state->getKinematicStateValues(vals);
  
  ROS_INFO_STREAM("Value " << vals["floating_trans_x"]);

                
  ros::Rate r(10.0);
  while(nh.ok()) {

    std_msgs::ColorRGBA point_color;
    point_color.a = 1.0;
    point_color.r = 1.0;
    point_color.g = .8;
    point_color.b = 0.04;

    std_msgs::ColorRGBA stat_color;
    stat_color.a = 0.5;
    stat_color.r = 0.1;
    stat_color.g = 0.8;
    stat_color.b = 0.3;

    std_msgs::ColorRGBA attached_color;
    attached_color.a = 0.5;
    attached_color.r = 0.6;
    attached_color.g = 0.4;
    attached_color.b = 0.3;

    visualization_msgs::MarkerArray arr;
    /*
    cmodel.getAllCollisionPointMarkers(*state,
                                       arr,
                                       point_color,
                                       ros::Duration(.2));
    */
    cmodel.getAllCollisionSpaceObjectMarkers(*state, 
                                             arr,
                                             "",
                                             stat_color,
                                             attached_color,
                                             ros::Duration(.2));

    cmodel.getRobotPaddedMarkersGivenState(*state, arr, stat_color, "robot", ros::Duration(.2));

    vis_marker_array_publisher.publish(arr);
    ros::spinOnce();
    r.sleep();
  }

  cmodel.revertPlanningScene(state);
  ros::shutdown();
}
  
  


