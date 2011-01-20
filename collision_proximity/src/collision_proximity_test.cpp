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
#include <tf/transform_listener.h>
#include <planning_environment/monitors/planning_monitor.h>
#include <collision_proximity/collision_proximity_space.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "collision_proximity_test");
 
  ros::NodeHandle nh;
  
  tf::TransformListener tf;

  std::string robot_description_name = nh.resolveName("robot_description", true);
  
  // monitor robot
  planning_environment::CollisionModels* collision_models = new planning_environment::CollisionModels(robot_description_name);
  planning_environment::PlanningMonitor* planning_monitor = new planning_environment::PlanningMonitor(collision_models, &tf);
  planning_monitor->setUseCollisionMap(true);
  if (!collision_models->loadedModels())
    return false;
  planning_monitor->startEnvironmentMonitor();

  ros::WallTime n1 = ros::WallTime::now();
  collision_proximity::CollisionProximitySpace* cps = new collision_proximity::CollisionProximitySpace(planning_monitor);
  ros::WallTime n2 = ros::WallTime::now();

  ROS_INFO_STREAM("Creation took " << (n2-n1).toSec());

  ros::Rate r(10.0);
  
  unsigned int count_max = 50;
  unsigned int count = 0;

  while(nh.ok()) {
    if(count%count_max == 0) {
      
      motion_planning_msgs::RobotState rob_state;
      n1 = ros::WallTime::now();
      if(count != 0) {
        cps->revertAfterGroupQueries();
      }
      cps->setupForGroupQueries("right_arm_and_end_effector", rob_state);
      n2 = ros::WallTime::now();
      ROS_INFO_STREAM("Distance field preparation took " << (n2-n1).toSec());
      //cps->visualizeDistanceField();
    }
    count++;
    //collision_proximity::CollisionProximitySpace::ProximityInfo prox;
    //cps->getEnvironmentProximity(pubs, state, prox);
    //n2 = ros::WallTime::now();
    //ROS_INFO_STREAM("Proximity query took " << (n2-n1).toSec()/1000.0);
    //ROS_INFO_STREAM("Distance is " << prox.proximity);

    //n1 = ros::WallTime::now();
    //bool coll = cps->getEnvironmentCollision(pubs);
    //n2 = ros::WallTime::now();
    //ROS_INFO_STREAM("Collision check took " << (n2-n1).toSec());
    std::vector<double> link_distances;
    std::vector<std::vector<double> > distances;
    std::vector<std::vector<btVector3> > gradients;
    std::vector<std::string> link_names;
    std::vector<std::string> attached_body_names;
    std::vector<collision_proximity::CollisionType> collisions;
    {
      planning_models::KinematicState cur_state(planning_monitor->getKinematicModel());
      planning_monitor->setStateValuesFromCurrentValues(cur_state);
      cps->setCurrentGroupState(cur_state);
      bool in_collision; 
      cps->getStateCollisions(link_names, attached_body_names, in_collision, collisions);
      cps->visualizeCollisions(link_names, attached_body_names, collisions);
      cps->getStateGradients(link_names, attached_body_names, link_distances, distances, gradients);
      cps->visualizeProximityGradients(link_names, attached_body_names, link_distances, distances, gradients);
    }      
    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();

  delete collision_models;
  delete planning_monitor;
  delete cps;

}
