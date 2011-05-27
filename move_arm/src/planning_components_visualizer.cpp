/*
 * Copyright (c) 2011, Willow Garage, Inc.
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
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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
 */

// Author: E. Gil Jones

#include <termios.h>
#include <signal.h>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <math.h>

#include <ros/ros.h>

#include <planning_environment/models/collision_models.h>
#include <planning_environment_msgs/PlanningScene.h>
#include <planning_environment_msgs/GetPlanningScene.h>
#include <planning_environment/models/model_utils.h>
#include <rosgraph_msgs/Clock.h>
#include <tf/transform_broadcaster.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <actionlib/client/simple_action_client.h>
#include <motion_planning_msgs/GetMotionPlan.h>
#include <planning_environment_msgs/GetStateValidity.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <motion_planning_msgs/FilterJointTrajectoryWithConstraints.h>
#include <motion_planning_msgs/convert_messages.h>
#include <move_arm/move_arm_warehouse_reader.h>

#include <ncurses.h>

static const std::string VIS_TOPIC_NAME = "planning_components_visualizer";

//in 100 hz ticks
static const unsigned int CONTROL_SPEED = 10;

static const double BASE_TRANS_SPEED=.3;
static const double BASE_ROT_SPEED = .15;

static const double HAND_TRANS_SPEED=.05;
static const double HAND_ROT_SPEED = .15;

static const std::string GET_PLANNING_SCENE_NAME = "/environment_server/get_planning_scene";

class PlanningComponentsVisualizer {

public:

  struct GroupCollection {
    GroupCollection() {
      group_state_ = NULL;
      good_ik_solution_ = false;
    }

    std::string name_;
    std::string ik_link_name_;
    ros::ServiceClient coll_aware_ik_service_;
    ros::ServiceClient non_coll_aware_ik_service_;
    bool good_ik_solution_;
    planning_models::KinematicState* group_state_;
  };

  PlanningComponentsVisualizer() {

    cm_ = new planning_environment::CollisionModels("robot_description");
    vis_marker_publisher_ = nh_.advertise<visualization_msgs::Marker>(VIS_TOPIC_NAME, 128);
    vis_marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(VIS_TOPIC_NAME+"_array", 128);
    
    while(!ros::service::waitForService(GET_PLANNING_SCENE_NAME, ros::Duration(1.0))) {
      ROS_INFO_STREAM("Waiting for planning scene service " << GET_PLANNING_SCENE_NAME);
    }
    get_planning_scene_client_ = nh_.serviceClient<planning_environment_msgs::GetPlanningScene>(GET_PLANNING_SCENE_NAME);

    const std::map<std::string, planning_models::KinematicModel::GroupConfig>& group_config_map = cm_->getKinematicModel()->getJointModelGroupConfigMap();

    for(std::map<std::string, planning_models::KinematicModel::GroupConfig>::const_iterator it = group_config_map.begin();
        it != group_config_map.end();
        it++) {
      if(!it->second.base_link_.empty()) {
        group_collection_map_[it->first].name_ = it->first;
        group_collection_map_[it->first].ik_link_name_ = it->second.tip_link_;
        std::string ik_service_name = "/"+cm_->getKinematicModel()->getRobotName()+"_"+it->first+"_kinematics/";
        std::string coll_aware_name = ik_service_name+"get_constraint_aware_ik";
        std::string non_coll_aware_name = ik_service_name+"get_ik";
        while(!ros::service::waitForService(coll_aware_name, ros::Duration(1.0))) {
          ROS_INFO_STREAM("Waiting for service " << coll_aware_name);
        }
        while(!ros::service::waitForService(non_coll_aware_name, ros::Duration(1.0))) {
          ROS_INFO_STREAM("Waiting for service " << non_coll_aware_name);
        }
        group_collection_map_[it->first].coll_aware_ik_service_ = 
          nh_.serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>(coll_aware_name, true);
        group_collection_map_[it->first].non_coll_aware_ik_service_ = 
          nh_.serviceClient<kinematics_msgs::GetPositionIK>(non_coll_aware_name, true);
      }
    }
    robot_state_ = new planning_models::KinematicState(cm_->getKinematicModel());
    robot_state_->setKinematicStateToDefault();

    sendPlanningScene();

    ROS_INFO_STREAM("Initialized");
    
  }

  ~PlanningComponentsVisualizer() { 
    deleteKinematicStates();
    delete robot_state_;
    delete cm_;
  }

  void sendPlanningScene() {
    lock_.lock();

    planning_environment_msgs::GetPlanningScene::Request planning_scene_req;
    planning_environment_msgs::GetPlanningScene::Response planning_scene_res;

    planning_environment::convertKinematicStateToRobotState(*robot_state_,
                                                            ros::Time::now(),
                                                            cm_->getWorldFrameId(), 
                                                            planning_scene_req.planning_scene_diff.robot_state);
    deleteKinematicStates();
    if(robot_state_ != NULL) {
      cm_->revertPlanningScene(robot_state_);
      robot_state_ = NULL;
    }
    
    if(!get_planning_scene_client_.call(planning_scene_req, planning_scene_res)) {
      ROS_WARN("Can't get planning scene");
      lock_.unlock();
      return;
    }

    robot_state_ = cm_->setPlanningScene(planning_scene_res.planning_scene);
    if(robot_state_ == NULL) {
      ROS_ERROR("Something wrong with planning scene");
      lock_.unlock();
      return;
    }
    lock_.unlock();
  }

  void selectPlanningGroup() {
    while(1) {
      echo();
      deleteKinematicStates();
      unsigned int num = 0;
      clear();
      std::vector<std::string> names;
      for(std::map<std::string, GroupCollection>::iterator it = group_collection_map_.begin();
          it != group_collection_map_.end();
          it++, num++) {
        names.push_back(it->first);
        printw("%d) %s\n", num, it->first.c_str());
      }
      printw("Enter a number to select the group\n");
      refresh();
      char str[80];
      getstr(str); 
      unsigned int entry;
      std::stringstream ss(str);
      ss >> entry;
      lock_.lock();
      current_group_name_ = names[entry]; 
      group_collection_map_[current_group_name_].group_state_ = new planning_models::KinematicState(*robot_state_);
      moveEndEffectorMarkers(0.0,0.0,0.0,0.0,0.0,0.0,false);
      lock_.unlock();
      movePlanningGroup();
      lock_.lock();
      current_group_name_ = "";
      deleteKinematicStates();
      lock_.unlock();
    }
  }

  void movePlanningGroup() {
    clear();
    noecho();
    bool coll_aware = false;
    bool quit = false;
    bool print = true;
    while(!quit) {
      if(print) {
        clear();
        printw("Examining current group %s\n", current_group_name_.c_str());
        printw("Type 'q' to return to group selection\n");
        printw("Type 'c' to toggle collision-aware/non-collision aware ik\n");
        printw("Type 'i/k' for end effector forward/back\n");
        printw("Type 'j/l' for end effector left/right\n");
        printw("Type 'h/n' for end effector up/down\n");
        printw("Type 'r/f' for end effector pitch up/down\n"); 
        printw("Type 'd/g' for end effector yaw up/down\n");
        printw("Type 'v/b' for end effector roll up/down\n");
        printw("Current using");
        if(coll_aware) {
          printw(" collision-aware ");
        } else {
          printw(" non-collision-aware ");
        }
        printw("ik\n");
        refresh();
        print = false;
      }
      char c = getch();
      switch(c) {
      case 'q':
        quit = true;
        break;
      case 'c':
        coll_aware = !coll_aware;
        print = true;
        break;
      case 'i':
        moveEndEffectorMarkers(HAND_TRANS_SPEED, 0.0, 0.0,
                                    0.0,0.0,0.0, coll_aware);
        break;
      case 'k':
        moveEndEffectorMarkers(-HAND_TRANS_SPEED, 0.0, 0.0,
                                    0.0,0.0,0.0, coll_aware);
        break;
      case 'j':
        moveEndEffectorMarkers(0.0, HAND_TRANS_SPEED, 0.0,
                                    0.0,0.0,0.0, coll_aware);
        break;
      case 'l':
        moveEndEffectorMarkers(0.0, -HAND_TRANS_SPEED, 0.0,
                                    0.0,0.0,0.0, coll_aware);
        break;
      case 'h':
        moveEndEffectorMarkers(0.0, 0.0, HAND_TRANS_SPEED,
                                    0.0,0.0,0.0, coll_aware);
        break;
      case 'n':
        moveEndEffectorMarkers(0.0, 0.0, -HAND_TRANS_SPEED,
                                    0.0,0.0,0.0, coll_aware);
        break;
      case 'r':
        moveEndEffectorMarkers(0.0, 0.0, 0.0,
                               0.0, HAND_ROT_SPEED, 0.0, coll_aware);
        break;
      case 'f':
        moveEndEffectorMarkers(0.0, 0.0, 0.0,
                                    0.0, -HAND_ROT_SPEED, 0.0, coll_aware);
        break;
      case 'd':
        moveEndEffectorMarkers(0.0, 0.0, 0.0,
                                    0.0, 0.0, HAND_ROT_SPEED, coll_aware);
        break;
      case 'g':
        moveEndEffectorMarkers(0.0, 0.0, 0.0,
                                    0.0, 0.0, -HAND_ROT_SPEED, coll_aware);
        break;
      case 'v':
        moveEndEffectorMarkers(0.0, 0.0, 0.0,
                                    HAND_ROT_SPEED, 0.0, 0.0, coll_aware);
        break;
      case 'b':
        moveEndEffectorMarkers(0.0, 0.0, 0.0,
                                    -HAND_ROT_SPEED, 0.0, 0.0, coll_aware);
        break;
      }
    }
  }

  void moveEndEffectorMarkers(double vx, double vy, double vz,
                              double vr, double vp, double vw,
                              bool coll_aware = true)

  {
    lock_.lock();
    GroupCollection& gc = group_collection_map_[current_group_name_];
    btTransform cur = gc.group_state_->getLinkState(gc.ik_link_name_)->getGlobalLinkTransform();
    double mult = CONTROL_SPEED/100.0;
    cur.setOrigin(btVector3(cur.getOrigin().x()+(vx*mult), cur.getOrigin().y()+(vy*mult), cur.getOrigin().z()+(vz*mult))); 
    btScalar roll, pitch, yaw;
    cur.getBasis().getRPY(roll,pitch,yaw);
    roll += vr*mult;
    pitch += vp*mult;
    yaw += vw*mult;
    if(roll > 2*M_PI) {
      roll -= 2*M_PI;
    } else if (roll < -2*M_PI) {
      roll += 2*M_PI;
    }
    if(pitch > 2*M_PI) {
      pitch -= 2*M_PI;
    } else if (pitch < -2*M_PI) {
      pitch += 2*M_PI;
    }
    cur.getBasis().setRPY(roll,pitch,yaw);
    if(!gc.group_state_->updateKinematicStateWithLinkAt(gc.ik_link_name_, cur)) {
      ROS_INFO("Problem");
    }
    if(solveIKForEndEffectorPose(gc, coll_aware)) {
      gc.good_ik_solution_ = true;
    } else {
      gc.good_ik_solution_ = false;
    }
    lock_.unlock();
  }

  bool solveIKForEndEffectorPose(PlanningComponentsVisualizer::GroupCollection& gc, bool coll_aware = true, 
                                 bool constrain_pitch_and_roll = false, double change_redundancy = 0.0)
  {
    kinematics_msgs::PositionIKRequest ik_request;
    ik_request.ik_link_name = gc.ik_link_name_;
    ik_request.pose_stamped.header.frame_id = cm_->getWorldFrameId();
    ik_request.pose_stamped.header.stamp = ros::Time::now();
    tf::poseTFToMsg(gc.group_state_->getLinkState(gc.ik_link_name_)->getGlobalLinkTransform(), ik_request.pose_stamped.pose);
    planning_environment::convertKinematicStateToRobotState(*gc.group_state_, ros::Time::now(), 
                                                            cm_->getWorldFrameId(), ik_request.robot_state);
    ik_request.ik_seed_state = ik_request.robot_state;
    // if(change_redundancy != 0.0) {
    //   for(unsigned int i = 0; i < ik_request.ik_seed_state.joint_state.name.size(); i++) {
    //     if(ik_request.ik_seed_state.joint_state.name[i] == redundancy_joint_) {
    //       ik_request.ik_seed_state.joint_state.position[i] += change_redundancy;
    //     }
    //   }
    // }
    std::map<std::string, double> joint_values;
    if(coll_aware) {
      kinematics_msgs::GetConstraintAwarePositionIK::Request ik_req;
      kinematics_msgs::GetConstraintAwarePositionIK::Response ik_res;
      ik_req.ik_request = ik_request;
      ik_req.timeout = ros::Duration(1.0);
      if(!gc.coll_aware_ik_service_.call(ik_req, ik_res)) {
        ROS_INFO("Problem with ik service call");
        return false;
      }
      if(ik_res.error_code.val != ik_res.error_code.SUCCESS) {
        ROS_INFO_STREAM("Call yields bad error code " << ik_res.error_code.val);
        return false;
      }
      for(unsigned int i = 0; i < ik_res.solution.joint_state.name.size(); i++) {
        joint_values[ik_res.solution.joint_state.name[i]] = ik_res.solution.joint_state.position[i];
      }
    } else {
      kinematics_msgs::GetPositionIK::Request ik_req;
      kinematics_msgs::GetPositionIK::Response ik_res;
      ik_req.ik_request = ik_request;
      ik_req.timeout = ros::Duration(1.0);
      if(!gc.non_coll_aware_ik_service_.call(ik_req, ik_res)) {
        ROS_INFO("Problem with ik service call");
        return false;
      }
      if(ik_res.error_code.val != ik_res.error_code.SUCCESS) {
        ROS_DEBUG_STREAM("Call yields bad error code " << ik_res.error_code.val);
        return false;
      }
      for(unsigned int i = 0; i < ik_res.solution.joint_state.name.size(); i++) {
        joint_values[ik_res.solution.joint_state.name[i]] = ik_res.solution.joint_state.position[i];
      }
    }
    lock_.lock();
    gc.group_state_->setKinematicState(joint_values);
    lock_.unlock();
    return true;
  }

  void sendMarkers() {
    lock_.lock();
    if(!current_group_name_.empty()) {
      std_msgs::ColorRGBA group_color;
      group_color.a = 1.0;
      group_color.r = 0.0;
      group_color.g = .8;
      group_color.b = 0.04;

      std_msgs::ColorRGBA updated_color;
      updated_color.a = 1.0;
      updated_color.r = 1.0;
      updated_color.g = 0.0;
      updated_color.b = 1.0;

      std_msgs::ColorRGBA bad_color;
      bad_color.a = 1.0;
      bad_color.r = 1.0;
      bad_color.g = 0.0;
      bad_color.b = 0.0;

      visualization_msgs::MarkerArray arr;
      GroupCollection& gc = group_collection_map_[current_group_name_];
      if(gc.good_ik_solution_) { 
        cm_->getGroupAndUpdatedJointMarkersGivenState(*gc.group_state_,
                                                      arr,
                                                      current_group_name_,
                                                      group_color,
                                                      updated_color,
                                                      ros::Duration(.2));
      } else {
        std::vector<std::string> lnames = 
          cm_->getKinematicModel()->getChildLinkModelNames(cm_->getKinematicModel()->getLinkModel(gc.ik_link_name_));
        //first link is ik_link_name_
        lnames.erase(lnames.begin());
        cm_->getRobotMeshResourceMarkersGivenState(*gc.group_state_,
                                                   arr,
                                                   bad_color,
                                                   current_group_name_,
                                                   ros::Duration(.2),
                                                   &lnames);
        cm_->getAttachedCollisionObjectMarkers(*gc.group_state_,
                                               arr,
                                               current_group_name_,
                                               bad_color,
                                               ros::Duration(.2));

      }
      vis_marker_array_publisher_.publish(arr);
    }
    lock_.unlock();
  }

  void sendTransforms() {
    lock_.lock();
    ros::WallTime cur_time = ros::WallTime::now();
    rosgraph_msgs::Clock c;
    c.clock.nsec = cur_time.nsec;
    c.clock.sec = cur_time.sec;
    std::vector<geometry_msgs::TransformStamped> trans_vector;
    planning_environment::getAllKinematicStateStampedTransforms(*robot_state_, trans_vector, c.clock);
    transform_broadcaster_.sendTransform(trans_vector);
    lock_.unlock();
  };

protected:
  
  void deleteKinematicStates() {
    for(std::map<std::string, GroupCollection>::iterator it = group_collection_map_.begin();
        it != group_collection_map_.end();
        it++) {
      if(it->second.group_state_ != NULL) {
        delete it->second.group_state_;
        it->second.group_state_ = NULL;
      }
    }
  }

  ros::NodeHandle nh_;
  std::string current_group_name_;

  std::map<std::string, GroupCollection> group_collection_map_;

  planning_environment::CollisionModels* cm_;
  planning_models::KinematicState* robot_state_;

  tf::TransformBroadcaster transform_broadcaster_;
  ros::Publisher vis_marker_publisher_;
  ros::Publisher vis_marker_array_publisher_;

  ros::ServiceClient get_planning_scene_client_;
  
  boost::recursive_mutex lock_;
};

PlanningComponentsVisualizer* pcv;

bool inited = false;

void spin_function() {
  ros::WallRate r(100.0);
  unsigned int counter = 0;
  while(ros::ok()) {
    if(inited) {
      pcv->sendTransforms();
      if(counter%CONTROL_SPEED == 0) {
        counter = 1;
        pcv->sendMarkers();
      } else {
        counter++;
      }
    }
    r.sleep();
    ros::spinOnce();
  }
}

void quit(int sig)
{
  if(pcv != NULL) {
    delete pcv;
  }
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planning_components_visualizer", ros::init_options::NoSigintHandler);

  boost::thread spin_thread(boost::bind(&spin_function));

  pcv = new PlanningComponentsVisualizer();

  inited = true;

  initscr();
  use_default_colors();
  start_color();

  pcv->selectPlanningGroup();

  ros::waitForShutdown();

  return 0;
}
