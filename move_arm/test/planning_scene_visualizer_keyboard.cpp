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
#include <planning_environment/models/model_utils.h>
#include <planning_environment_msgs/SetPlanningSceneAction.h>
#include <rosgraph_msgs/Clock.h>
#include <tf/transform_broadcaster.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <actionlib/client/simple_action_client.h>
#include <motion_planning_msgs/GetMotionPlan.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <motion_planning_msgs/FilterJointTrajectoryWithConstraints.h>

int kfd = 0;
struct termios cooked, raw;

static const std::string VIS_TOPIC_NAME = "planning_scene_visualizer";

//in 100 hz ticks
static const unsigned int CONTROL_SPEED = 10;

static const double BASE_TRANS_SPEED=.3;
static const double BASE_ROT_SPEED = .15;

static const double HAND_TRANS_SPEED=.05;
static const double HAND_ROT_SPEED = .15;

static const std::string SET_PLANNING_SCENE_NAME_1="/pr2_right_arm_kinematics/set_planning_scene";
static const std::string SET_PLANNING_SCENE_NAME_2="/pr2_left_arm_kinematics/set_planning_scene";
static const std::string SET_PLANNING_SCENE_NAME_3="/ompl_planning/set_planning_scene";
static const std::string SET_PLANNING_SCENE_NAME_4="/trajectory_filter/set_planning_scene";

static const std::string LEFT_IK_NAME="/pr2_left_arm_kinematics/get_constraint_aware_ik";
static const std::string RIGHT_IK_NAME="/pr2_right_arm_kinematics/get_constraint_aware_ik";

static const std::string RIGHT_ARM_GROUP = "right_arm";
static const std::string LEFT_ARM_GROUP = "left_arm";

static const std::string RIGHT_ARM_REDUNDANCY = "r_upper_arm_roll_joint";
static const std::string LEFT_ARM_REDUNDANCY = "l_upper_arm_roll_joint";

static const std::string LEFT_IK_LINK = "l_wrist_roll_link";
static const std::string RIGHT_IK_LINK = "r_wrist_roll_link";

static const std::string PLANNER_SERVICE_NAME="/ompl_planning/plan_kinematic_path";
static const std::string TRAJECTORY_FILTER_SERVICE_NAME="/trajectory_filter/filter_trajectory_with_constraints";

static const ros::Duration PLANNING_DURATION = ros::Duration(5.0);

class PlanningSceneVisualizer {
public:  
  enum CollisionStateDisplay {
    NONE = 0,
    SCENE = 1,
    END_EFFECTOR = 2,
    PLANNER_TRAJECTORY = 3,
    FILTER_TRAJECTORY = 4
  };

  PlanningSceneVisualizer()
  {
    std::string robot_description_name = nh_.resolveName("robot_description", true);
    cm_ = new planning_environment::CollisionModels(robot_description_name);
    
    vis_marker_publisher_ = nh_.advertise<visualization_msgs::Marker>(VIS_TOPIC_NAME, 128);
    vis_marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(VIS_TOPIC_NAME+"_array", 128);

    ros::service::waitForService(LEFT_IK_NAME);
    ros::service::waitForService(RIGHT_IK_NAME);
    ros::service::waitForService(PLANNER_SERVICE_NAME);
    
    left_ik_service_client_ = nh_.serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>(LEFT_IK_NAME, true);
    right_ik_service_client_ = nh_.serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>(RIGHT_IK_NAME, true);
    planning_service_client_ = nh_.serviceClient<motion_planning_msgs::GetMotionPlan>(PLANNER_SERVICE_NAME, true);
    trajectory_filter_service_client_ = nh_.serviceClient<motion_planning_msgs::FilterJointTrajectoryWithConstraints>(TRAJECTORY_FILTER_SERVICE_NAME);

    set_planning_scene_action_1_ = new actionlib::SimpleActionClient<planning_environment_msgs::SetPlanningSceneAction>(SET_PLANNING_SCENE_NAME_1);
    while(!set_planning_scene_action_1_->waitForServer(ros::Duration(.1)));

    set_planning_scene_action_2_ = new actionlib::SimpleActionClient<planning_environment_msgs::SetPlanningSceneAction>(SET_PLANNING_SCENE_NAME_2);
    while(!set_planning_scene_action_2_->waitForServer(ros::Duration(.1)));

    set_planning_scene_action_3_ = new actionlib::SimpleActionClient<planning_environment_msgs::SetPlanningSceneAction>(SET_PLANNING_SCENE_NAME_3);
    while(!set_planning_scene_action_3_->waitForServer(ros::Duration(.1)));

    set_planning_scene_action_4_ = new actionlib::SimpleActionClient<planning_environment_msgs::SetPlanningSceneAction>(SET_PLANNING_SCENE_NAME_4);
    while(!set_planning_scene_action_4_->waitForServer(ros::Duration(.1)));

    pole_obj_.object.header.stamp = ros::Time::now();
    pole_obj_.object.header.frame_id = "r_gripper_r_finger_tip_link";
    pole_obj_.link_name = "r_gripper_palm_link";
    pole_obj_.touch_links.push_back("r_gripper_palm_link");
    pole_obj_.touch_links.push_back("r_gripper_r_finger_link");
    pole_obj_.touch_links.push_back("r_gripper_l_finger_link");
    pole_obj_.touch_links.push_back("r_gripper_r_finger_tip_link");
    pole_obj_.touch_links.push_back("r_gripper_l_finger_tip_link");
    pole_obj_.object.id = "pole";
    pole_obj_.object.shapes.resize(1);
    pole_obj_.object.shapes[0].type = geometric_shapes_msgs::Shape::CYLINDER;
    pole_obj_.object.shapes[0].dimensions.resize(2);
    pole_obj_.object.shapes[0].dimensions[0] = .025;
    pole_obj_.object.shapes[0].dimensions[1] = .4;
    pole_obj_.object.poses.resize(1);
    pole_obj_.object.poses[0].position.x = 0.0;
    pole_obj_.object.poses[0].position.y = 0.0;
    pole_obj_.object.poses[0].position.z = 0.0;
    pole_obj_.object.poses[0].orientation.w = 1.0;

    //clock_publisher_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 128);

    send_collision_markers_ = false;
    send_ik_solution_ = false;
    send_end_effector_goal_markers_ = false;

    point_color_.a = 1.0;
    point_color_.r = 1.0;
    point_color_.g = .8;
    point_color_.b = 0.04;

    stat_color_.a = 0.5;
    stat_color_.r = 0.1;
    stat_color_.g = 0.8;
    stat_color_.b = 0.3;

    attached_color_.a = 0.5;
    attached_color_.r = 0.6;
    attached_color_.g = 0.4;
    attached_color_.b = 0.3;

    end_effector_bad_color_.a = .9;
    end_effector_bad_color_.r = 1.0;
    end_effector_bad_color_.b = 0.0;
    end_effector_bad_color_.g = 0.0;

    end_effector_good_color_.a = .9;
    end_effector_good_color_.r = 1.0;
    end_effector_good_color_.b = 1.0;
    end_effector_good_color_.g = 0.0;

    trajectory_color_.a = .6;
    trajectory_color_.r = 0.0;
    trajectory_color_.b = 1.0;
    trajectory_color_.g = 0.0;

    filter_color_.a = .6;
    filter_color_.r = 0.0;
    filter_color_.b = 1.0;
    filter_color_.g = 1.0;

    planning_state_ = NULL;
    end_effector_state_ = NULL;
    trajectory_state_ = NULL;
    filter_state_ = NULL;
  }
  
  ~PlanningSceneVisualizer() {
    delete set_planning_scene_action_1_;
    delete set_planning_scene_action_2_;
    delete set_planning_scene_action_3_;
    delete set_planning_scene_action_4_;
    deleteKinematicStates();
    delete cm_;
  }

  void deleteKinematicStates() {
    if(planning_state_ != NULL) {
      delete planning_state_;
      planning_state_ = NULL;
    }
    if(end_effector_state_ != NULL) {
      delete end_effector_state_;
      end_effector_state_ = NULL;
    }
    if(trajectory_state_ != NULL) {
      delete trajectory_state_;
      trajectory_state_ = NULL;
    }
    if(filter_state_ != NULL) {
      delete filter_state_;
      filter_state_ = NULL;
    }
  }

  bool loadPlanningScene(const std::string& filename) {
    lock_scene_.lock();
    if(!cm_->readPlanningSceneBag(filename,
                                  planning_scene_)) {
      ROS_ERROR("Bag file doesn't exist or doesn't contain a planning scene");
      lock_scene_.unlock();
      return false;
    }
    bool ok = sendPlanningScene();
    lock_scene_.unlock();
    return ok;
  }

  bool sendPlanningScene() {
    lock_scene_.lock();
    if(planning_state_ != NULL) {
      cm_->revertPlanningScene(planning_state_);
      planning_state_ = NULL;
    }
    deleteKinematicStates();
    setEndEffectorGoal(false);
    planning_state_ = cm_->setPlanningScene(planning_scene_);
    if(planning_state_ == NULL) {
      ROS_ERROR("Something wrong with planning scene");
      lock_scene_.unlock();
      return false;
    }
    planning_state_joint_values_.clear();
    planning_state_->getKinematicStateValues(planning_state_joint_values_);
    setShowCollisions(false);
    ros::WallTime cur_time = ros::WallTime::now();
    rosgraph_msgs::Clock c;
    c.clock.nsec = cur_time.nsec;
    c.clock.sec = cur_time.sec;
    getAllRobotStampedTransforms(*planning_state_, planning_scene_robot_transforms_, c.clock);

    planning_environment_msgs::SetPlanningSceneGoal planning_scene_goal;
    planning_scene_goal.planning_scene = planning_scene_;
    
    set_planning_scene_action_1_->sendGoal(planning_scene_goal);
    set_planning_scene_action_2_->sendGoal(planning_scene_goal);
    set_planning_scene_action_3_->sendGoal(planning_scene_goal);
    set_planning_scene_action_4_->sendGoal(planning_scene_goal);
    while(!set_planning_scene_action_1_->waitForResult(ros::Duration(.1)));
    while(!set_planning_scene_action_2_->waitForResult(ros::Duration(.1)));
    while(!set_planning_scene_action_3_->waitForResult(ros::Duration(.1)));
    while(!set_planning_scene_action_4_->waitForResult(ros::Duration(.1)));

    send_start_state_ = true;
    lock_scene_.unlock();
    return true;
  }
  
  void sendTransformsAndClock() {
    lock_scene_.lock();
    ros::WallTime cur_time = ros::WallTime::now();
    rosgraph_msgs::Clock c;
    c.clock.nsec = cur_time.nsec;
    c.clock.sec = cur_time.sec;
    //clock_publisher_.publish(c);
    getAllRobotStampedTransforms(*planning_state_, planning_scene_robot_transforms_, c.clock);
    transform_broadcaster_.sendTransform(planning_scene_robot_transforms_);
    lock_scene_.unlock();
  };

  void resetPlanningStateToStartState() {
    planning_state_->setKinematicState(planning_state_joint_values_);
    if(send_collision_markers_) {
      updateCurrentCollisionSet(collision_marker_state_);
    }
  }

  void getAllRobotStampedTransforms(const planning_models::KinematicState& state,
                                    std::vector<geometry_msgs::TransformStamped>& trans_vector,
                                    const ros::Time& stamp) 
  {
    trans_vector.clear();
    const std::map<std::string, geometry_msgs::TransformStamped>& transforms = cm_->getSceneTransformMap();
    geometry_msgs::TransformStamped transvec;
    for(std::map<std::string, geometry_msgs::TransformStamped>::const_iterator it = transforms.begin();
        it != transforms.end();
        it++) {
      trans_vector.push_back(it->second);
    }
    for(unsigned int i = 0; i < state.getLinkStateVector().size(); i++) {
      const planning_models::KinematicState::LinkState* ls = state.getLinkStateVector()[i];
      geometry_msgs::TransformStamped ts;
      ts.header.stamp = stamp;
      ts.header.frame_id = cm_->getWorldFrameId();
      ts.child_frame_id = ls->getName();
      tf::transformTFToMsg(ls->getGlobalLinkTransform(),ts.transform);
      trans_vector.push_back(ts);
    }
  }

  bool togglePole() {
    lock_scene_.lock();
    bool ok;
    if(have_pole_) {
      planning_scene_.attached_collision_objects.pop_back();
      have_pole_ = false;      
      ok =  sendPlanningScene();
    } else {
      cm_->convertAttachedCollisionObjectToNewWorldFrame(*planning_state_,
                                                         pole_obj_);
      planning_scene_.attached_collision_objects.push_back(pole_obj_);
      have_pole_ = true;
      ok = sendPlanningScene();
    }
    lock_scene_.unlock();
    return ok;
  }

  void moveBase(double vx, double vy, double vt) 
  {
    lock_scene_.lock();
    btTransform cur = planning_state_->getRootTransform();
    double mult = CONTROL_SPEED/100.0;
    cur.setOrigin(btVector3(cur.getOrigin().x()+(vx*mult), cur.getOrigin().y()+(vy*mult), 0.0));
    cur.setRotation(btQuaternion(btVector3(0.0,0.0,1.0),cur.getRotation().getAngle()*cur.getRotation().getAxis().z()+(vt*mult)));
    planning_state_->getJointStateVector()[0]->setJointStateValues(cur);
    planning_state_->updateKinematicLinks();
    if(send_collision_markers_) {
      updateCurrentCollisionSet(collision_marker_state_);
    }
    lock_scene_.unlock();
  }

  void moveEndEffectorMarkers(double vx, double vy, double vz,
                              double vr, double vp, double vw)
  {
    lock_scene_.lock();
    btTransform cur = end_effector_state_->getLinkState(end_effector_link_)->getGlobalLinkTransform();
    double mult = CONTROL_SPEED/100.0;
    cur.setOrigin(btVector3(cur.getOrigin().x()+(vx*mult), cur.getOrigin().y()+(vy*mult), cur.getOrigin().z()+(vz*mult))); 
    btScalar roll, pitch, yaw;
    cur.getBasis().getRPY(roll,pitch,yaw);
    roll += vr*mult;
    pitch += vp*mult;
    yaw += vw*mult;
    cur.getBasis().setRPY(roll,pitch,yaw);
    if(!end_effector_state_->updateKinematicStateWithLinkAt(end_effector_link_, cur)) {
      ROS_INFO("Problem");
    }
    if(solveIKForEndEffectorPose(false)) {
      use_good_ik_color_ = true;
    } else {
      use_good_ik_color_ = false;
      send_ik_solution_ = false;
    }
    lock_scene_.unlock();
  }
  
  bool solveIKForEndEffectorPose(bool show, double change_redundancy = 0.0)
  {
    bool right = (end_effector_link_ == RIGHT_IK_LINK);
    kinematics_msgs::PositionIKRequest ik_request;
    ik_request.ik_link_name = end_effector_link_;
    ik_request.pose_stamped.header.frame_id = cm_->getWorldFrameId();
    ik_request.pose_stamped.header.stamp = ros::Time::now();
    tf::poseTFToMsg(end_effector_state_->getLinkState(end_effector_link_)->getGlobalLinkTransform(), ik_request.pose_stamped.pose);
    planning_environment::convertKinematicStateToRobotState(*end_effector_state_, ros::Time::now(), 
                                                            cm_->getWorldFrameId(), ik_request.robot_state);
    ik_request.ik_seed_state = ik_request.robot_state;
    if(change_redundancy != 0.0) {
      for(unsigned int i = 0; i < ik_request.ik_seed_state.joint_state.name.size(); i++) {
        if(ik_request.ik_seed_state.joint_state.name[i] == redundancy_joint_) {
          ik_request.ik_seed_state.joint_state.position[i] += change_redundancy;
        }
      }
    }

    ros::ServiceClient* s;
    if(right) {
      s = &right_ik_service_client_;
    } else {
      s = &left_ik_service_client_;
    }
    kinematics_msgs::GetConstraintAwarePositionIK::Request ik_req;
    kinematics_msgs::GetConstraintAwarePositionIK::Response ik_res;
    ik_req.ik_request = ik_request;
    ik_req.timeout = ros::Duration(1.0);
    if(!s->call(ik_req, ik_res)) {
      ROS_INFO("Problem with ik service call");
      return false;
    }
    if(ik_res.error_code.val != ik_res.error_code.SUCCESS) {
      ROS_DEBUG_STREAM("Call yields bad error code " << ik_res.error_code.val);
      return false;
    }
    if(show) {
      send_ik_solution_ = true;
    }
    std::map<std::string, double> joint_values;
    for(unsigned int i = 0; i < ik_res.solution.joint_state.name.size(); i++) {
      joint_values[ik_res.solution.joint_state.name[i]] = ik_res.solution.joint_state.position[i];
    }
    lock_scene_.lock();
    end_effector_state_->setKinematicState(joint_values);
    lock_scene_.unlock();
    return true;
  }

  void sendMarkers() 
  {
    lock_scene_.lock();
    visualization_msgs::MarkerArray arr;
    if(send_start_state_) {
      cm_->getAllCollisionSpaceObjectMarkers(*planning_state_,
                                             arr,
                                             "",
                                             stat_color_,
                                             attached_color_,
                                             ros::Duration(.2));
    }
    if(send_collision_markers_) {
      arr.markers.insert(arr.markers.end(), collision_markers_.markers.begin(), collision_markers_.markers.end());
    }
    if(send_ik_solution_) {
      std::vector<std::string> lnames = cm_->getKinematicModel()->getModelGroup(arm_group_name_)->getUpdatedLinkModelNames();
      cm_->getRobotMeshResourceMarkersGivenState(*end_effector_state_,
                                                 arr,
                                                 end_effector_good_color_,
                                                 "ik_solution_pose",
                                                 ros::Duration(.2),
                                                 &lnames);
      cm_->getAttachedCollisionObjectMarkers(*end_effector_state_,
                                             arr,
                                             "ik_solution_pose",
                                             end_effector_good_color_,
                                             ros::Duration(.2));
    } else if(send_end_effector_goal_markers_) {
      std_msgs::ColorRGBA col;
      if(use_good_ik_color_) {
        col = end_effector_good_color_;
      } else {
        col = end_effector_bad_color_;
      }
      std::vector<std::string> lnames = 
        cm_->getKinematicModel()->getChildLinkModelNames(cm_->getKinematicModel()->getLinkModel(end_effector_link_));
      cm_->getRobotMeshResourceMarkersGivenState(*end_effector_state_,
                                                 arr,
                                                 col,
                                                 "end_effector_pose",
                                                 ros::Duration(.2),
                                                 &lnames);
      cm_->getAttachedCollisionObjectMarkers(*end_effector_state_,
                                             arr,
                                             "end_effector_pose",
                                             col,
                                             ros::Duration(.2));

    }
    if(send_planner_trajectory_) {
      const std::vector<const planning_models::KinematicModel::LinkModel*>& updated_links = 
        cm_->getKinematicModel()->getModelGroup(arm_group_name_)->getUpdatedLinkModels();
      std::vector<std::string> lnames;
      lnames.resize(updated_links.size());
      for(unsigned int i = 0; i < updated_links.size(); i++) {
        lnames[i] = updated_links[i]->getName();
      }
      cm_->getRobotMeshResourceMarkersGivenState(*trajectory_state_,
                                                 arr,
                                                 trajectory_color_,
                                                 "planner_trajectory",
                                                 ros::Duration(.2),
                                                 &lnames);
      cm_->getAttachedCollisionObjectMarkers(*trajectory_state_,
                                             arr,
                                             "planner_trajectory",
                                             trajectory_color_,
                                             ros::Duration(.2));

    }
    if(send_filter_trajectory_) {
      const std::vector<const planning_models::KinematicModel::LinkModel*>& updated_links = 
        cm_->getKinematicModel()->getModelGroup(arm_group_name_)->getUpdatedLinkModels();
      std::vector<std::string> lnames;
      lnames.resize(updated_links.size());
      for(unsigned int i = 0; i < updated_links.size(); i++) {
        lnames[i] = updated_links[i]->getName();
      }
      cm_->getRobotMeshResourceMarkersGivenState(*filter_state_,
                                                 arr,
                                                 filter_color_,
                                                 "filter_trajectory",
                                                 ros::Duration(.2),
                                                 &lnames);
      cm_->getAttachedCollisionObjectMarkers(*filter_state_,
                                             arr,
                                             "filter_trajectory",
                                             filter_color_,
                                             ros::Duration(.2));

    }
    cm_->getCollisionMapAsMarkers(arr, stat_color_, ros::Duration(.2));
    vis_marker_array_publisher_.publish(arr);
    lock_scene_.unlock();
  }

  

  void setEndEffectorGoal(bool on, bool right = false) {
    lock_scene_.lock();
    send_end_effector_goal_markers_ = on;
    send_ik_solution_ = false;
    if(send_end_effector_goal_markers_) {
      if(end_effector_state_ == NULL) {
        end_effector_state_ = new planning_models::KinematicState(*planning_state_);
      }
      if(right) {
        end_effector_link_ = RIGHT_IK_LINK;
        redundancy_joint_ = RIGHT_ARM_REDUNDANCY;
        arm_group_name_ = RIGHT_ARM_GROUP;
      } else {
        end_effector_link_ = LEFT_IK_LINK;
        redundancy_joint_ = LEFT_ARM_REDUNDANCY;
        arm_group_name_ = LEFT_ARM_GROUP;
      }
      moveEndEffectorMarkers(0.0,0.0,0.0,0.0,0.0,0.0);
    } else if(end_effector_state_ != NULL) {
      delete end_effector_state_;
      end_effector_state_ = NULL;
    }
    lock_scene_.unlock();
  }

  bool planToEndEffectorState() {
    motion_planning_msgs::GetMotionPlan::Request plan_req;
    plan_req.motion_plan_request.group_name = arm_group_name_;
    plan_req.motion_plan_request.num_planning_attempts = 1;
    plan_req.motion_plan_request.allowed_planning_time = PLANNING_DURATION;
    const planning_models::KinematicState::JointStateGroup* jsg = end_effector_state_->getJointStateGroup(arm_group_name_);
    plan_req.motion_plan_request.goal_constraints.joint_constraints.resize(jsg->getJointNames().size());
    std::vector<double> joint_values;
    jsg->getKinematicStateValues(joint_values);
    for(unsigned int i = 0; i < jsg->getJointNames().size(); i++) {
      plan_req.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = jsg->getJointNames()[i];
      plan_req.motion_plan_request.goal_constraints.joint_constraints[i].position = joint_values[i];
      plan_req.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.001;
      plan_req.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.001;
    }      
    planning_environment::convertKinematicStateToRobotState(*planning_state_, ros::Time::now(),
                                                            cm_->getWorldFrameId(), plan_req.motion_plan_request.start_state);
    motion_planning_msgs::GetMotionPlan::Response plan_res;
    if(!planning_service_client_.call(plan_req, plan_res)) {
      ROS_INFO("Something wrong with planner client");
      return false;
    }
    if(plan_res.error_code.val != plan_res.error_code.SUCCESS) {
      ROS_INFO_STREAM("Bad planning error code " << plan_res.error_code.val);
      return false;
    }
    lock_scene_.lock();
    send_planner_trajectory_ = true;
    current_planner_trajectory_point_ = 0;
    trajectory_state_ = new planning_models::KinematicState(*planning_state_);
    planner_trajectory_ = plan_res.trajectory.joint_trajectory;
    ROS_INFO_STREAM("Trajectory has " << plan_res.trajectory.joint_trajectory.points.size());
    moveThroughPlannerTrajectory(0, false);
    last_goal_constraints_ = plan_req.motion_plan_request.goal_constraints;
    last_path_constraints_ = plan_req.motion_plan_request.path_constraints;
    lock_scene_.unlock();
    return true;
  }

  bool filterPlannerTrajectory() {
    motion_planning_msgs::FilterJointTrajectoryWithConstraints::Request filter_req;
    motion_planning_msgs::FilterJointTrajectoryWithConstraints::Response filter_res;
    
    planning_environment::convertKinematicStateToRobotState(*planning_state_, ros::Time::now(),
                                                            cm_->getWorldFrameId(), filter_req.start_state);
    filter_req.trajectory = planner_trajectory_;
    filter_req.group_name = arm_group_name_;

    filter_req.goal_constraints = last_goal_constraints_;
    filter_req.path_constraints = last_path_constraints_;
    filter_req.allowed_time = ros::Duration(2.0);

    if(!trajectory_filter_service_client_.call(filter_req, filter_res)) {
      ROS_INFO("Problem with trajectory filter");
      return false;
    }
    if(filter_res.error_code.val != filter_res.error_code.SUCCESS) {
      ROS_INFO_STREAM("Bad trajectory_filter error code " << filter_res.error_code.val);
      return false;
    }
    lock_scene_.lock();
    send_filter_trajectory_ = true;
    current_filter_trajectory_point_ = 0;
    filter_state_ = new planning_models::KinematicState(*planning_state_);
    filter_trajectory_ = filter_res.trajectory;
    moveThroughFilterTrajectory(0, false);   
    lock_scene_.unlock();
    return true;
  }

  void moveThroughPlannerTrajectory(int step, bool print) {
    lock_scene_.lock();
    unsigned int tsize = planner_trajectory_.points.size(); 
    if(tsize == 0 || trajectory_state_ == NULL) {
      lock_scene_.unlock();
      return;
    }
    if((int) current_planner_trajectory_point_ + step < 0) {
      current_planner_trajectory_point_ = 0;
    } else {
      current_planner_trajectory_point_ = ((int)current_planner_trajectory_point_)+step;
    }
    if(current_planner_trajectory_point_ >= tsize-1) {
      current_planner_trajectory_point_ = tsize-1;
    }
    std::map<std::string, double> joint_values;
    for(unsigned int i = 0; i < planner_trajectory_.joint_names.size(); i++) {
      joint_values[planner_trajectory_.joint_names[i]] = planner_trajectory_.points[current_planner_trajectory_point_].positions[i];
    }
    if(print) {
      printTrajectoryPoint(planner_trajectory_.joint_names,
                           planner_trajectory_.points[current_planner_trajectory_point_].positions);
    }
    trajectory_state_->setKinematicState(joint_values);
    if(send_collision_markers_ && collision_marker_state_ == PLANNER_TRAJECTORY) {
      updateCurrentCollisionSet(PLANNER_TRAJECTORY);
    }
    lock_scene_.unlock();
  }

  void printTrajectoryPoint(const std::vector<std::string>& joint_names, const std::vector<double>& joint_values) {
    for(unsigned int i = 0; i < joint_names.size(); i++) {
      ROS_INFO_STREAM("Joint name " << joint_names[i] << " value " << joint_values[i]);
    }
  }

  void moveThroughFilterTrajectory(int step, bool print) {
    lock_scene_.lock();
    unsigned int tsize = filter_trajectory_.points.size(); 
    if(tsize == 0 || filter_state_ == NULL) {
      lock_scene_.unlock();
      return;
    }
    if((int) current_filter_trajectory_point_ + step < 0) {
      current_filter_trajectory_point_ = 0;
    } else {
      current_filter_trajectory_point_ = ((int)current_filter_trajectory_point_)+step;
    }
    if(current_filter_trajectory_point_ >= tsize-1) {
      current_filter_trajectory_point_ = tsize-1;
    }
    std::map<std::string, double> joint_values;
    for(unsigned int i = 0; i < filter_trajectory_.joint_names.size(); i++) {
      joint_values[filter_trajectory_.joint_names[i]] = filter_trajectory_.points[current_filter_trajectory_point_].positions[i];
    }
    if(print) {
      printTrajectoryPoint(filter_trajectory_.joint_names,
                           filter_trajectory_.points[current_planner_trajectory_point_].positions);
    }
    filter_state_->setKinematicState(joint_values);
    if(send_collision_markers_ && collision_marker_state_ == FILTER_TRAJECTORY) {
      updateCurrentCollisionSet(FILTER_TRAJECTORY);
    }
    lock_scene_.unlock();
  }

  void stopShowingPlannerTrajectory() {
    lock_scene_.lock();
    send_planner_trajectory_ = false;
    delete trajectory_state_;
    trajectory_state_ = NULL;
    lock_scene_.unlock();
  }

  void stopShowingFilterTrajectory() {
    lock_scene_.lock();
    send_filter_trajectory_ = false;
    delete filter_state_;
    filter_state_ = NULL;
    lock_scene_.unlock();
  }
  
  void setShowCollisions(bool send_collision_markers, CollisionStateDisplay csd = NONE) {
    lock_scene_.lock();
    send_collision_markers_ = send_collision_markers;
    if(send_collision_markers_) {
      updateCurrentCollisionSet(csd);
    } else {
      collision_markers_.markers.clear();
    }
    lock_scene_.unlock();
  }

  void updateCurrentCollisionSet(CollisionStateDisplay csd)
  {
    lock_scene_.lock();
    const planning_models::KinematicState* state = NULL;
    if(csd == END_EFFECTOR) {
      state = end_effector_state_;
    } else if(csd == PLANNER_TRAJECTORY) {
      state = trajectory_state_;
    } else if(csd == FILTER_TRAJECTORY) {
      state = filter_state_;
    } else {
      state = planning_state_;
    }
    collision_marker_state_ = csd;
    collision_markers_.markers.clear();
    if(state == NULL) {
      lock_scene_.unlock();
      return;
    }
    cm_->getAllCollisionPointMarkers(*state,
                                     collision_markers_,
                                     point_color_,
                                     ros::Duration(.2)); 
    lock_scene_.unlock();
  }

  bool getSendCollisionMarkers() const {
    return send_collision_markers_;
  }
  
protected:
  ros::NodeHandle nh_;
  planning_environment::CollisionModels* cm_;

  planning_models::KinematicState* planning_state_;
  planning_models::KinematicState* end_effector_state_;
  planning_models::KinematicState* trajectory_state_;
  planning_models::KinematicState* filter_state_;

  planning_environment_msgs::PlanningScene planning_scene_;
  trajectory_msgs::JointTrajectory planner_trajectory_;
  trajectory_msgs::JointTrajectory filter_trajectory_;
  mapping_msgs::AttachedCollisionObject pole_obj_;

  motion_planning_msgs::Constraints last_goal_constraints_;
  motion_planning_msgs::Constraints last_path_constraints_;

  std::string end_effector_link_;
  std::string arm_group_name_;
  std::string redundancy_joint_;
  unsigned int current_planner_trajectory_point_;
  unsigned int current_filter_trajectory_point_;

  CollisionStateDisplay collision_marker_state_;
  bool have_pole_;
  bool send_start_state_;
  bool send_collision_markers_;
  bool send_end_effector_goal_markers_;
  bool send_ik_solution_;
  bool send_planner_trajectory_;
  bool send_filter_trajectory_;
  bool use_good_ik_color_;
  
  std_msgs::ColorRGBA point_color_;
  std_msgs::ColorRGBA stat_color_;
  std_msgs::ColorRGBA attached_color_;
  std_msgs::ColorRGBA end_effector_bad_color_;
  std_msgs::ColorRGBA end_effector_good_color_;
  std_msgs::ColorRGBA trajectory_color_;
  std_msgs::ColorRGBA filter_color_;

  ros::Publisher vis_marker_publisher_;
  ros::Publisher vis_marker_array_publisher_;
  ros::Publisher clock_publisher_;

  ros::ServiceClient left_ik_service_client_;
  ros::ServiceClient right_ik_service_client_;
  ros::ServiceClient planning_service_client_;
  ros::ServiceClient trajectory_filter_service_client_;

  actionlib::SimpleActionClient<planning_environment_msgs::SetPlanningSceneAction>* set_planning_scene_action_1_;  
  actionlib::SimpleActionClient<planning_environment_msgs::SetPlanningSceneAction>* set_planning_scene_action_2_;  
  actionlib::SimpleActionClient<planning_environment_msgs::SetPlanningSceneAction>* set_planning_scene_action_3_;  
  actionlib::SimpleActionClient<planning_environment_msgs::SetPlanningSceneAction>* set_planning_scene_action_4_;  

  tf::TransformBroadcaster transform_broadcaster_;
  std::vector<geometry_msgs::TransformStamped> planning_scene_robot_transforms_;  
  std::map<std::string, double> planning_state_joint_values_;  

  visualization_msgs::MarkerArray collision_markers_;
  boost::recursive_mutex lock_scene_;
};

PlanningSceneVisualizer* psv = NULL;
bool inited = false;

void spin_function() {
  ros::WallRate r(100.0);
  unsigned int counter = 0;
  while(ros::ok()) {
    if(inited) {
      psv->sendTransformsAndClock();
      if(counter%CONTROL_SPEED == 0) {
        counter = 1;
        psv->sendMarkers();
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
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  if(psv != NULL) {
    delete psv;
  }
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planning_scene_keyboard", ros::init_options::NoSigintHandler);
  if(argc <= 1) {
    ROS_ERROR("Must give address of bag file");
    exit(0);
  }

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  //ros::NodeHandle loc("~");
  //loc.setParam("tf_prefix", "planning_scene");

  signal(SIGINT,quit);
  boost::thread spin_thread(boost::bind(&spin_function));

  psv = new PlanningSceneVisualizer();

  if(!psv->loadPlanningScene(argv[1])) {
    exit(0);
  }

  inited = true;
    
  char c;
  
  bool stop = false;
  bool reprint = true;
  while(!stop)
  {
    if(reprint) {
      reprint = false;
      puts("------------------");
      puts("Use 'i/k' for forward/back");
      puts("Use 'j/l' for strafe left/right");
      puts("Use 'n/m' for turning left and right");
      puts("Use 'c' to toggle collision markers");
      puts("Use 'r' to reset to original planning state");
      puts("Use 'y' to mask collision map");
      puts("Use 'w/e' for left/right end effector pose submenu");
      puts("Use 'p' to toggle pole attached to right end effector");
      puts("Use 'q' to quit");
    }
    
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }
    switch(c)
    {
    case 'w': case 'e':
      {
        bool right = (c == 'e');
        psv->setEndEffectorGoal(true, right);
        bool end_effector_stop = false;
        bool reprint_end = true;
        while(!end_effector_stop) {
          if(reprint_end) {
            puts("------------------");
            puts("Use 'i/k' for end effector forward/back");
            puts("Use 'j/l' for end effector left/right");
            puts("Use 'h/n' for end effector up/down");
            puts("Use 'r/f' for pitch up/down");
            puts("Use 'd/g' for yaw left/right");
            puts("Use 'v/b' for roll up/down");
            puts("Use 's' to solve for collision free ik given the current end effector position");
            puts("Use 'a/z' to change redundancy");
            puts("Use 'p' to plan to current end effector pose");
            puts("Use 'q' to exit submenu");
            reprint_end = false;
          }
          if(read(kfd, &c, 1) < 0)
          {
            perror("read():");
            exit(-1);
          }
          switch(c) {
          case 'i':
            psv->moveEndEffectorMarkers(HAND_TRANS_SPEED, 0.0, 0.0,
                                        0.0,0.0,0.0);
            break;
          case 'k':
            psv->moveEndEffectorMarkers(-HAND_TRANS_SPEED, 0.0, 0.0,
                                        0.0,0.0,0.0);
            break;
          case 'j':
            psv->moveEndEffectorMarkers(0.0, HAND_TRANS_SPEED, 0.0,
                                        0.0,0.0,0.0);
            break;
          case 'l':
            psv->moveEndEffectorMarkers(0.0, -HAND_TRANS_SPEED, 0.0,
                                        0.0,0.0,0.0);
            break;
          case 'h':
            psv->moveEndEffectorMarkers(0.0, 0.0, HAND_TRANS_SPEED,
                                        0.0,0.0,0.0);
            break;
          case 'n':
            psv->moveEndEffectorMarkers(0.0, 0.0, -HAND_TRANS_SPEED,
                                        0.0,0.0,0.0);
            break;
          case 'r':
            psv->moveEndEffectorMarkers(0.0, 0.0, 0.0,
                                        0.0, HAND_ROT_SPEED, 0.0);
            break;
          case 'f':
            psv->moveEndEffectorMarkers(0.0, 0.0, 0.0,
                                        0.0, -HAND_ROT_SPEED, 0.0);
            break;
          case 'd':
            psv->moveEndEffectorMarkers(0.0, 0.0, 0.0,
                                        0.0, 0.0, HAND_ROT_SPEED);
            break;
          case 'g':
            psv->moveEndEffectorMarkers(0.0, 0.0, 0.0,
                                        0.0, 0.0, -HAND_ROT_SPEED);
            break;
          case 'v':
            psv->moveEndEffectorMarkers(0.0, 0.0, 0.0,
                                        HAND_ROT_SPEED, 0.0, 0.0);
            break;
          case 'b':
            psv->moveEndEffectorMarkers(0.0, 0.0, 0.0,
                                        -HAND_ROT_SPEED, 0.0, 0.0);
            break;
          case 's':
            psv->solveIKForEndEffectorPose(true);
            break;
          case 'a':
            psv->solveIKForEndEffectorPose(true, HAND_ROT_SPEED);
            break;
          case 'z':
            psv->solveIKForEndEffectorPose(true, -HAND_ROT_SPEED);
            break;
          case 'p':
            if(psv->solveIKForEndEffectorPose(true) && psv->planToEndEffectorState()) {
              puts("------------------");
              puts("Use 'i/k' to advance/rewind planner trajectory points");
              puts("Use 'o/l' to advance/rewind filter trajectory points");
              puts("Use 'f' to call trajectory filter on planner trajectory");
              puts("Use 'y' to toggle planner trajectory collisions");
              puts("Use 'h' to toggle filter trajectory collisions");
              puts("Use 'z' to toggle printing planning trajectory values");
              puts("use 'x' to toggle printing filter trajectory values");
              puts("Use 'q' to return to end effector menu");
              bool trajectory_stop = false;
              bool planner_trajectory_printing = false;
              bool filter_trajectory_printing = false;
              while(!trajectory_stop) {
                if(read(kfd, &c, 1) < 0)
                {
                  perror("read():");
                  exit(-1);
                }
                switch(c) {
                case 'i': 
                  psv->moveThroughPlannerTrajectory(1, planner_trajectory_printing);
                  break;
                case 'k':
                  psv->moveThroughPlannerTrajectory(-1, planner_trajectory_printing);
                  break;
                case 'o': 
                  psv->moveThroughFilterTrajectory(1, filter_trajectory_printing);
                  break;
                case 'l':
                  psv->moveThroughFilterTrajectory(-1,filter_trajectory_printing);
                  break;
                case 'f':
                  psv->filterPlannerTrajectory();
                  break;
                case 'y':
                  psv->setShowCollisions(!psv->getSendCollisionMarkers(), PlanningSceneVisualizer::PLANNER_TRAJECTORY);
                  break;
                case 'h':
                  psv->setShowCollisions(!psv->getSendCollisionMarkers(), PlanningSceneVisualizer::FILTER_TRAJECTORY);
                  break;
                case 'z':
                  planner_trajectory_printing = !planner_trajectory_printing;
                  break;
                case 'x':
                  filter_trajectory_printing = !filter_trajectory_printing;
                  break;
                case 'q':
                  trajectory_stop = true;
                  break;
                default: 
                  ROS_INFO_STREAM("Invalid key " << c);
                  break;
                }
              }
              psv->stopShowingPlannerTrajectory();
              psv->stopShowingFilterTrajectory();
              psv->setShowCollisions(false);
              reprint_end = true;
            }
            break;
          case 'q':
            psv->setEndEffectorGoal(false);
            end_effector_stop = true;
            reprint = true;
            break;
          }
        }
      }
      break;
    case 'i':
      psv->moveBase(BASE_TRANS_SPEED, 0.0, 0.0);
      break;
    case 'k':
      psv->moveBase(-BASE_TRANS_SPEED, 0.0, 0.0);
      break;
    case 'j':
      psv->moveBase(0.0, BASE_TRANS_SPEED, 0.0);
      break;
    case 'l':
      psv->moveBase(0.0, -BASE_TRANS_SPEED, 0.0);
      break;
    case 'n':
      psv->moveBase(0.0, 0.0, BASE_ROT_SPEED);
      break;
    case 'm':
      psv->moveBase(0.0, 0.0, -BASE_ROT_SPEED);
      break;
    case 'c':
      psv->setShowCollisions(!psv->getSendCollisionMarkers(), PlanningSceneVisualizer::SCENE);
      break;
    case 'r':
      psv->resetPlanningStateToStartState();
      break;
    case 'p':
      psv->togglePole();
      break;
    case 'q':
      stop = true;
      break;
    default:
      //ROS_INFO_STREAM("Keycode is " << c);
      break;
    }
  }
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  spin_thread.join();
  return(0);
}


