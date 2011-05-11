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
 *  \author Adam Harmat, E. Gil Jones
 *********************************************************************/

#include <ros/ros.h>

#include <planning_environment/models/collision_models_interface.h>
#include <planning_environment/monitors/kinematic_model_state_monitor.h>
#include <planning_environment/models/model_utils.h>
#include <planning_environment/monitors/monitor_utils.h>
#include <motion_planning_msgs/convert_messages.h>
#include <angles/angles.h>
#include "planning_environment/util/construct_object.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>

#include <pr2_controllers_msgs/PointHeadAction.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>

#include <move_arm_msgs/HeadMonitorStatus.h>
#include <move_arm_msgs/HeadMonitorAction.h>
#include <move_arm_msgs/PreplanHeadScanAction.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>

static const std::string RIGHT_ARM_GROUP = "right_arm";
static const std::string LEFT_ARM_GROUP = "left_arm";

static const double JOINT_BOUNDS_MARGIN = .02;

std::string convertFromGroupNameToArmName(const std::string& arm_name) {
  if(arm_name.find(RIGHT_ARM_GROUP) != std::string::npos) {
    return(RIGHT_ARM_GROUP);
  } else if(arm_name.find(LEFT_ARM_GROUP) != std::string::npos) {
    return(LEFT_ARM_GROUP);
  } else {
    ROS_WARN_STREAM("Neither left arm group or right arm group in group: " << arm_name);
  }
  return std::string("");
}

class HeadMonitor
{
protected:

  ros::NodeHandle private_handle_;
  ros::NodeHandle root_handle_;
  actionlib::SimpleActionServer<move_arm_msgs::HeadMonitorAction> head_monitor_action_server_;
  actionlib::SimpleActionServer<move_arm_msgs::PreplanHeadScanAction> head_preplan_scan_action_server_;

  ros::Publisher marker_pub_;

  actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> head_controller_action_client_;
  actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>* right_arm_controller_action_client_;
  actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>* left_arm_controller_action_client_;
  actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> point_head_action_client_;

  std::string current_group_name_;
  actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>* current_arm_controller_action_client_;

  planning_environment::CollisionModelsInterface* collision_models_interface_;
  planning_environment::KinematicModelStateMonitor* kmsm_;

  move_arm_msgs::HeadMonitorGoal monitor_goal_;
  move_arm_msgs::HeadMonitorFeedback monitor_feedback_;
  move_arm_msgs::HeadMonitorResult monitor_result_;

  sensor_msgs::JointState last_joint_state_;

  trajectory_msgs::JointTrajectory logged_trajectory_;
  ros::Time logged_trajectory_start_time_;

  tf::TransformListener tf_;

  ros::Timer paused_callback_timer_;
  ros::Timer start_trajectory_timer_;

  visualization_msgs::Marker marker_;

  tf::MessageFilter<sensor_msgs::PointCloud2> *mn_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> *sub_;

  //TODO - get rid of once we have Adam's stuff
  double start_head_pan_, start_head_tilt_, goal_head_pan_, goal_head_tilt_;

  double point_sphere_size_;
  double pause_time_;
  double max_point_distance_;

  move_arm_msgs::HeadMonitorStatus current_execution_status_;

public:

  HeadMonitor() :
    private_handle_("~"),
    head_monitor_action_server_(root_handle_, "head_monitor_action", false),
    head_preplan_scan_action_server_(root_handle_, "preplan_head_scan", boost::bind(&HeadMonitor::preplanHeadScanCallback, this, _1), false), 
    head_controller_action_client_("/head_traj_controller/joint_trajectory_action", true),
    point_head_action_client_("/head_traj_controller/point_head_action", true)
  {
    ROS_INFO_STREAM("In constructor");
    
    current_execution_status_.status = current_execution_status_.IDLE;

    private_handle_.param<double>("point_sphere_size", point_sphere_size_, .01);
    private_handle_.param<double>("pause_time", pause_time_, 5.0);
    private_handle_.param<double>("max_point_distance", max_point_distance_, 1.0);

    std::string robot_description_name = root_handle_.resolveName("robot_description", true);
    
    collision_models_interface_ = new planning_environment::CollisionModelsInterface(robot_description_name);
    kmsm_ = new planning_environment::KinematicModelStateMonitor(collision_models_interface_, &tf_);

    kmsm_->addOnStateUpdateCallback(boost::bind(&HeadMonitor::jointStateCallback, this, _1));

    sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2> (root_handle_, "cloud_in", 1);	
    mn_ = new tf::MessageFilter<sensor_msgs::PointCloud2> (*sub_, tf_, "", 1);
    mn_->setTargetFrame(collision_models_interface_->getWorldFrameId());
    mn_->registerCallback(boost::bind(&HeadMonitor::cloudCallback, this, _1));

    head_monitor_action_server_.registerGoalCallback(boost::bind(&HeadMonitor::monitorGoalCallback, this));
    head_monitor_action_server_.registerPreemptCallback(boost::bind(&HeadMonitor::monitorPreemptCallback, this));

    right_arm_controller_action_client_ = new actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>("/r_arm_controller/joint_trajectory_action", true);
    left_arm_controller_action_client_ = new actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>("/l_arm_controller/joint_trajectory_action", true);
    while(ros::ok() && !right_arm_controller_action_client_->waitForServer(ros::Duration(1.0))){
      ROS_INFO("Waiting for the right_joint_trajectory_action server to come up.");
    }
    while(ros::ok() && !left_arm_controller_action_client_->waitForServer(ros::Duration(1.0))){
      ROS_INFO("Waiting for the right_joint_trajectory_action server to come up.");
    }

    ROS_INFO("Connected to the controllers");
    
    head_monitor_action_server_.start();
    head_preplan_scan_action_server_.start();
    	
    ROS_INFO_STREAM("Starting head monitor action server for "<<ros::this_node::getName());
  }

  ~HeadMonitor(void)
  {
    delete kmsm_;
    delete collision_models_interface_;
    delete right_arm_controller_action_client_;
    delete left_arm_controller_action_client_;
  }

  ///
  ///  Message and action callbacks
  ///
  
  void cloudCallback (const sensor_msgs::PointCloud2ConstPtr &cloud2)
  {
    if(current_execution_status_.status == current_execution_status_.IDLE ||
       current_execution_status_.status == current_execution_status_.PREPLAN_SCAN) {
      return;
    }

    ros::Time callback_delay = ros::Time::now();

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud, trans_cloud, near_cloud;
    pcl::fromROSMsg(*cloud2, pcl_cloud);

    if(pcl_cloud.points.size() == 0) {
      ROS_WARN_STREAM("No points in cloud");
      return;
    } 

    collision_models_interface_->bodiesLock();

    sensor_msgs::JointState js = last_joint_state_;
    trajectory_msgs::JointTrajectory joint_trajectory_subset;
    if(current_execution_status_.status == current_execution_status_.EXECUTING) {
      removeCompletedTrajectory(monitor_goal_.joint_trajectory,
                                js,
                                joint_trajectory_subset,
                                false);
    } else {
      joint_trajectory_subset = monitor_goal_.joint_trajectory;
    }

    if(joint_trajectory_subset.points.size() == 0) {
      //no points left to check 
      if(current_execution_status_.status == current_execution_status_.PAUSED) {
        ROS_ERROR_STREAM("Paused, but closest trajectory point is goal");
      }
      collision_models_interface_->bodiesUnlock();
      return;
    }
    ros::WallTime n1 = ros::WallTime::now();

    planning_models::KinematicState state(collision_models_interface_->getKinematicModel());
    kmsm_->setStateValuesFromCurrentValues(state);

    planning_environment::updateAttachedObjectBodyPoses(collision_models_interface_,
                                                        state,
                                                        tf_);

    //now in the right frame

    if (collision_models_interface_->getWorldFrameId() != pcl_cloud.header.frame_id) {
      pcl_ros::transformPointCloud(collision_models_interface_->getWorldFrameId(), pcl_cloud, trans_cloud, tf_);
    } else {
      trans_cloud = pcl_cloud;
    }

    btTransform cur_link_pose = state.getLinkState(monitor_goal_.target_link)->getGlobalCollisionBodyTransform();
    near_cloud.header = trans_cloud.header;
    for(unsigned int i = 0; i < trans_cloud.points.size(); i++) {
      btVector3 pt = btVector3(trans_cloud.points[i].x, trans_cloud.points[i].y, trans_cloud.points[i].z);
      double dist = pt.distance(cur_link_pose.getOrigin());
      if(dist <= max_point_distance_) {
        near_cloud.push_back(trans_cloud.points[i]);
      }
    }
    
    std::vector<shapes::Shape*> spheres(near_cloud.points.size());;
    std::vector<btTransform> positions(near_cloud.points.size());
    
    ros::WallTime n3 = ros::WallTime::now();
      
    if(near_cloud.points.size() != 0) {

      btQuaternion ident(0.0, 0.0, 0.0, 1.0);
      //making spheres from the points
      for (unsigned int i = 0 ; i < near_cloud.points.size(); ++i) {
        positions[i] = btTransform(ident, 
                                   btVector3(near_cloud.points[i].x, near_cloud.points[i].y, near_cloud.points[i].z));
        spheres[i] = new shapes::Sphere(point_sphere_size_);
      }
      
      //collisions should have been turned off with everything but this
      collision_models_interface_->addStaticObject("point_spheres",
                                                   spheres,
                                                   positions,
                                                   0.0);

      //turning off collisions except between point spheres and downstream links
      collision_space::EnvironmentModel::AllowedCollisionMatrix acm = collision_models_interface_->getCollisionSpace()->getCurrentAllowedCollisionMatrix();
      const planning_models::KinematicModel::JointModelGroup* joint_model_group = collision_models_interface_->getKinematicModel()->getModelGroup(current_group_name_);
      
      acm.addEntry("point_spheres", true);
      acm.changeEntry(true);
      acm.changeEntry("point_spheres", joint_model_group->getUpdatedLinkModelNames(), false);

      collision_models_interface_->setAlteredAllowedCollisionMatrix(acm);
    }
    motion_planning_msgs::Constraints empty_constraints;
    motion_planning_msgs::ArmNavigationErrorCodes error_code;
    std::vector<motion_planning_msgs::ArmNavigationErrorCodes> trajectory_error_codes;
    
    ros::WallTime n2 = ros::WallTime::now();
    if(!collision_models_interface_->isJointTrajectoryValid(state, 
                                                            joint_trajectory_subset,
                                                            empty_constraints,
                                                            empty_constraints, 
                                                            error_code,
                                                            trajectory_error_codes, 
                                                            false)) {
      if(current_execution_status_.status == current_execution_status_.MONITOR_BEFORE_EXECUTION ||
         current_execution_status_.status == current_execution_status_.EXECUTING) {
        ROS_WARN_STREAM("Trajectory judged invalid " << error_code.val << ". Pausing");
        //TODO - stop more gracefully, do something else?
        if(current_execution_status_.status == current_execution_status_.MONITOR_BEFORE_EXECUTION) {
          start_trajectory_timer_.stop();
        } 
        kmsm_->setStateValuesFromCurrentValues(state);
        planning_environment::convertKinematicStateToRobotState(state,
                                                                ros::Time::now(),
                                                                collision_models_interface_->getWorldFrameId(),
                                                                monitor_feedback_.current_state);
        unsigned int trajectory_point = trajectory_error_codes.size();
        if(trajectory_error_codes.size() > 0) {
          trajectory_point--;
        }
        ROS_INFO_STREAM("Setting trajectory state to point " << trajectory_point << " of " << joint_trajectory_subset.points.size());
        
        std::map<std::string, double> vals;
        for(unsigned int i = 0; i < joint_trajectory_subset.joint_names.size(); i++) {
          vals[joint_trajectory_subset.joint_names[i]] = joint_trajectory_subset.points[trajectory_point].positions[i];
        }
        state.setKinematicState(vals);
        planning_environment::convertKinematicStateToRobotState(state,
                                                                ros::Time::now(),
                                                                collision_models_interface_->getWorldFrameId(),
                                                                monitor_feedback_.paused_trajectory_state);
        monitor_feedback_.paused_collision_map.header.frame_id = collision_models_interface_->getWorldFrameId();
        monitor_feedback_.paused_collision_map.header.stamp = cloud2->header.stamp;
        monitor_feedback_.paused_collision_map.id = "point_spheres";
        monitor_feedback_.paused_collision_map.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
        for (unsigned int j = 0 ; j < spheres.size(); ++j) {
          geometric_shapes_msgs::Shape obj;
          if (planning_environment::constructObjectMsg(spheres[j], obj)) {
            geometry_msgs::Pose pose;
            tf::poseTFToMsg(positions[j], pose);
            monitor_feedback_.paused_collision_map.shapes.push_back(obj);
            monitor_feedback_.paused_collision_map.poses.push_back(pose);
          }
        }
        std::vector<planning_environment_msgs::ContactInformation> contacts;
        collision_models_interface_->getAllCollisionsForState(state, contacts, 1);
        if(contacts.size() == 0) {
          ROS_INFO_STREAM("No contacts for last trajectory state");
        } else {
          for(unsigned int i = 0; i < contacts.size(); i++) {
            ROS_INFO_STREAM("Contact between " << contacts[i].contact_body_1 << " and " << contacts[i].contact_body_2
                            << " at point " << contacts[i].position.x << " " << contacts[i].position.y << " " << contacts[i].position.z);
          }
          
        }
        head_monitor_action_server_.publishFeedback(monitor_feedback_);
  
        if(current_execution_status_.status == current_execution_status_.EXECUTING) {
          current_arm_controller_action_client_->cancelGoal();
        }
        ROS_INFO_STREAM("Delay from data timestamp to cancel is " << (ros::Time::now() - cloud2->header.stamp).toSec()
                        << " delay to receipt is " << (ros::Time::now()-callback_delay).toSec());
        
        stopHead();
        current_execution_status_.status = current_execution_status_.PAUSED;
        paused_callback_timer_ = root_handle_.createTimer(ros::Duration(pause_time_), boost::bind(&HeadMonitor::pauseTimeoutCallback, this), true);
      } 
      //if paused no need to do anything else, except maybe provide additional feedback?
    } else {
      if(current_execution_status_.status == current_execution_status_.PAUSED) {
        paused_callback_timer_.stop();
       
        ROS_WARN_STREAM("Unpausing");

        current_execution_status_.status = current_execution_status_.MONITOR_BEFORE_EXECUTION;
        
        sensor_msgs::JointState js = last_joint_state_;
        trajectory_msgs::JointTrajectory joint_trajectory_subset;
        
        removeCompletedTrajectory(monitor_goal_.joint_trajectory,
                                  js,
                                  joint_trajectory_subset,
                                  false);

        pr2_controllers_msgs::JointTrajectoryGoal goal;
        goal.trajectory = generateHeadTrajectory(monitor_goal_.target_link, joint_trajectory_subset);
        head_controller_action_client_.sendGoal(goal);

        //Setting timer for actually sending trajectory
        start_trajectory_timer_ = root_handle_.createTimer(ros::Duration(monitor_goal_.time_offset), boost::bind(&HeadMonitor::trajectoryTimerCallback, this), true);
      }
    }
    if(near_cloud.points.size() != 0) {
      collision_models_interface_->deleteStaticObject("point_spheres");
      monitor_feedback_.paused_collision_map.shapes.clear();
      monitor_feedback_.paused_collision_map.poses.clear();
    }
    ROS_DEBUG_STREAM("Trajectory check took " << (ros::WallTime::now() - n1).toSec() 
                    << " shape " << (n2-n3).toSec()
                    << " traj part " << (ros::WallTime::now() - n2).toSec());
    collision_models_interface_->bodiesUnlock();
  }
  
  void moveInsideSafetyLimits(const move_arm_msgs::PreplanHeadScanGoalConstPtr &goal,
                              const planning_models::KinematicState& state) {
    const planning_models::KinematicModel::JointModelGroup* joint_model_group = collision_models_interface_->getKinematicModel()->getModelGroup(goal->group_name);
    const std::vector<std::string>& joint_names = joint_model_group->getJointModelNames();
    if(state.areJointsWithinBounds(joint_names)) {
      return;
    }
    trajectory_msgs::JointTrajectory traj;
    traj.joint_names = joint_names;
    traj.header.stamp = ros::Time::now();
    traj.header.frame_id = collision_models_interface_->getWorldFrameId();
    traj.points.resize(1);
    traj.points[0].positions.resize(joint_names.size());
    traj.points[0].velocities.resize(joint_names.size());
    traj.points[0].time_from_start = ros::Duration(.4);

    std::map<std::string, double> joint_values;
    state.getKinematicStateValues(joint_values);
    
    for(unsigned int j = 0; j < joint_names.size(); j++) {
      if(!state.isJointWithinBounds(joint_names[j])) {
        std::pair<double, double> bounds; 
        state.getJointState(joint_names[j])->getJointModel()->getVariableBounds(joint_names[j], bounds);
        ROS_INFO_STREAM("Joint " << joint_names[j] << " out of bounds. " <<
                        " value: " << state.getJointState(joint_names[j])->getJointStateValues()[0] << 
                        " low: " << bounds.first << " high: " << bounds.second);
        if(joint_values[joint_names[j]] < bounds.first) {
          traj.points[0].positions[j] = bounds.first+JOINT_BOUNDS_MARGIN;
          ROS_INFO_STREAM("Setting joint " << joint_names[j] << " inside lower bound " << traj.points[0].positions[j]);
        } else {
          traj.points[0].positions[j] = bounds.second-JOINT_BOUNDS_MARGIN;
          ROS_INFO_STREAM("Setting joint " << joint_names[j] << " inside upper bound " << traj.points[0].positions[j]);
        }
      } else {
        traj.points[0].positions[j] = joint_values[joint_names[j]];
      }
    }

    pr2_controllers_msgs::JointTrajectoryGoal traj_goal;  
    traj_goal.trajectory = traj;

    if(current_arm_controller_action_client_->sendGoalAndWait(traj_goal, ros::Duration(1.0), ros::Duration(.5)) != actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_WARN_STREAM("Joint bounds correction trajectory failed");
    }
  }

  void preplanHeadScanCallback(const move_arm_msgs::PreplanHeadScanGoalConstPtr &goal)
  { 
    move_arm_msgs::PreplanHeadScanResult res;

    collision_models_interface_->bodiesLock();

    if(current_execution_status_.status != current_execution_status_.IDLE) {
      ROS_WARN_STREAM("Got preplan in something other than IDLE mode");
    }

    current_execution_status_.status = current_execution_status_.PREPLAN_SCAN;

    //need a kinematic state
    planning_models::KinematicState state(collision_models_interface_->getKinematicModel());
    
    kmsm_->setStateValuesFromCurrentValues(state);

    moveInsideSafetyLimits(goal, state);

    kmsm_->setStateValuesFromCurrentValues(state);

    //supplementing with start state changes
    planning_environment::setRobotStateAndComputeTransforms(goal->motion_plan_request.start_state,
                                                            state);
    double x_start, y_start, z_start;

    if(!state.hasLinkState(goal->head_monitor_link)) {
      ROS_WARN_STREAM("No monitor link " << goal->head_monitor_link);
      head_preplan_scan_action_server_.setAborted(res);
      current_execution_status_.status = current_execution_status_.IDLE;
      collision_models_interface_->bodiesUnlock();
      return;
    }

    btTransform link_state = state.getLinkState(goal->head_monitor_link)->getGlobalCollisionBodyTransform();
    x_start = link_state.getOrigin().x();
    y_start = link_state.getOrigin().y();
    z_start = link_state.getOrigin().z();

    double x_goal, y_goal, z_goal;
    if(goal->motion_plan_request.goal_constraints.position_constraints.size() >= 1
       && goal->motion_plan_request.goal_constraints.orientation_constraints.size() >= 1) {
      geometry_msgs::PoseStamped goal_pose 
        = motion_planning_msgs::poseConstraintsToPoseStamped(goal->motion_plan_request.goal_constraints.position_constraints[0],
                                                             goal->motion_plan_request.goal_constraints.orientation_constraints[0]);

      std::string es;
      if (tf_.getLatestCommonTime(collision_models_interface_->getWorldFrameId(), goal_pose.header.frame_id, goal_pose.header.stamp, &es) != tf::NO_ERROR) {
      }
      geometry_msgs::PoseStamped psout;
      tf_.transformPose(collision_models_interface_->getWorldFrameId(), goal_pose, psout);

      x_goal = psout.pose.position.x;
      y_goal = psout.pose.position.y;
      z_goal = psout.pose.position.z;
    } else {
      if(goal->motion_plan_request.goal_constraints.joint_constraints.size() <= 1) {
        ROS_WARN("Not a pose goal and not enough joint constraints");
        head_preplan_scan_action_server_.setAborted(res);
        current_execution_status_.status = current_execution_status_.IDLE;
        collision_models_interface_->bodiesUnlock();
        return;
      }
      std::map<std::string, double> joint_values;
      for(unsigned int i = 0; i < goal->motion_plan_request.goal_constraints.joint_constraints.size(); i++) {
        joint_values[goal->motion_plan_request.goal_constraints.joint_constraints[i].joint_name] = goal->motion_plan_request.goal_constraints.joint_constraints[i].position;
      }
      state.setKinematicState(joint_values);
      btTransform link_state = state.getLinkState(goal->head_monitor_link)->getGlobalCollisionBodyTransform();
      x_goal = link_state.getOrigin().x();
      y_goal = link_state.getOrigin().y();
      z_goal = link_state.getOrigin().z();
    }
    std::map<std::string, double> state_values;
    ROS_INFO_STREAM("Looking at goal " << x_goal << " " << y_goal << " " << z_goal);
    lookAt(collision_models_interface_->getWorldFrameId(), x_goal, y_goal, z_goal, true);
    kmsm_->setStateValuesFromCurrentValues(state);
    state.getKinematicStateValues(state_values);
    goal_head_pan_ = state_values["head_pan_joint"];
    goal_head_tilt_ = state_values["head_tilt_joint"];
    ROS_INFO_STREAM("Looking at start " << x_start << " " << y_start << " " << z_start);
    lookAt(collision_models_interface_->getWorldFrameId(), x_start, y_start, z_start, true);
    kmsm_->setStateValuesFromCurrentValues(state);
    state.getKinematicStateValues(state_values);
    start_head_pan_ = state_values["head_pan_joint"];
    start_head_tilt_ = state_values["head_tilt_joint"];
    head_preplan_scan_action_server_.setSucceeded(res);
    current_execution_status_.status = current_execution_status_.IDLE;
    collision_models_interface_->bodiesUnlock();
  }

  // Called when a new monitoring goal is received
  void monitorGoalCallback()
  {
    if(head_monitor_action_server_.isActive())
    {
      stopHead();
    }

    if(head_monitor_action_server_.isPreemptRequested())
    {
      ROS_DEBUG_STREAM(ros::this_node::getName() << ": Preempted");
      head_monitor_action_server_.setPreempted(monitor_result_);
      return;
    }

    if(head_monitor_action_server_.isNewGoalAvailable())
    {
      monitor_goal_ = move_arm_msgs::HeadMonitorGoal(*head_monitor_action_server_.acceptNewGoal());
    } else {
      ROS_WARN_STREAM("Not preempted but no new goal available");
    }

    current_group_name_ = convertFromGroupNameToArmName(monitor_goal_.group_name);

    if(current_group_name_.empty()) {
      ROS_WARN_STREAM("Group name doesn't have left or right arm in it, gonna be bad");
    }

    current_arm_controller_action_client_ = ((current_group_name_ == RIGHT_ARM_GROUP) ? right_arm_controller_action_client_ : left_arm_controller_action_client_);
    current_execution_status_.status = current_execution_status_.MONITOR_BEFORE_EXECUTION;

    //Generating head trajectory and deploying it
    pr2_controllers_msgs::JointTrajectoryGoal goal;
    goal.trajectory = generateHeadTrajectory(monitor_goal_.target_link, monitor_goal_.joint_trajectory);
    head_controller_action_client_.sendGoal(goal);
    
    logged_trajectory_ = monitor_goal_.joint_trajectory;
    logged_trajectory_.points.clear();

    logged_trajectory_start_time_ = ros::Time::now();

    //Setting timer for actually sending trajectory
    start_trajectory_timer_ = root_handle_.createTimer(ros::Duration(monitor_goal_.time_offset), boost::bind(&HeadMonitor::trajectoryTimerCallback, this), true);
  }

  void stopHead()
  {
    head_controller_action_client_.cancelAllGoals();
  }
  
  void stopArm() 
  {
    if(current_execution_status_.status == current_execution_status_.EXECUTING) {
      current_arm_controller_action_client_->cancelGoal();
    }
  }

  void stopEverything() {
    start_trajectory_timer_.stop();
    paused_callback_timer_.stop();
    stopHead();
    stopArm();
  }

  void monitorPreemptCallback()
  {
    if(current_execution_status_.status != current_execution_status_.MONITOR_BEFORE_EXECUTION &&
       current_execution_status_.status != current_execution_status_.EXECUTING &&
       current_execution_status_.status != current_execution_status_.PAUSED) {
      ROS_WARN_STREAM("Got preempt not in a relevant mode");
      return;
    }
    //no reason not to cancel
    stopEverything();

    current_execution_status_.status = current_execution_status_.IDLE;

    ROS_DEBUG_STREAM(ros::this_node::getName() << ": Preempted");
    head_monitor_action_server_.setPreempted(monitor_result_);
  }

  void jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state) {
    if(current_execution_status_.status == current_execution_status_.IDLE
       || current_execution_status_.status == current_execution_status_.PREPLAN_SCAN) {
      return;
    }
    std::map<std::string, double> joint_state_map;
    std::map<std::string, double> joint_velocity_map;
    //message already been validated in kmsm
    for (unsigned int i = 0 ; i < joint_state->position.size(); ++i)
    {
      joint_state_map[joint_state->name[i]] = joint_state->position[i];
      joint_velocity_map[joint_state->name[i]] = joint_state->velocity[i];
    }
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(logged_trajectory_.joint_names.size());
    point.velocities.resize(logged_trajectory_.joint_names.size());
    for(unsigned int i = 0; i < logged_trajectory_.joint_names.size(); i++) {
      point.positions[i] = joint_state_map[logged_trajectory_.joint_names[i]];
      point.velocities[i] = joint_velocity_map[logged_trajectory_.joint_names[i]];
    }
    point.time_from_start = ros::Time::now()-logged_trajectory_start_time_;
    logged_trajectory_.points.push_back(point);

    last_joint_state_ = *joint_state;
  }

  void controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                              const pr2_controllers_msgs::JointTrajectoryResultConstPtr& result)
  {
    if(current_execution_status_.status != current_execution_status_.EXECUTING) {
      //because we cancelled
      return;
    }
    ROS_INFO_STREAM("Trajectory reported done with state " << state.toString());
    current_execution_status_.status = current_execution_status_.IDLE;
    monitor_result_.actual_trajectory = logged_trajectory_;
    monitor_result_.error_code.val = monitor_result_.error_code.SUCCESS;
    head_monitor_action_server_.setSucceeded(monitor_result_);
  }

  void pauseTimeoutCallback() {
    ROS_INFO_STREAM("Paused trajectory timed out");
    stopEverything();
    current_execution_status_.status = current_execution_status_.IDLE;
    monitor_result_.actual_trajectory = logged_trajectory_;
    monitor_result_.error_code.val = monitor_result_.error_code.TIMED_OUT;
    head_monitor_action_server_.setAborted(monitor_result_);
  }

  ///
  ///  Timer callbacks
  ///


  void trajectoryTimerCallback() {
    current_execution_status_.status = current_execution_status_.EXECUTING;

    pr2_controllers_msgs::JointTrajectoryGoal goal;  

    sensor_msgs::JointState js = last_joint_state_;
    removeCompletedTrajectory(monitor_goal_.joint_trajectory,
                              js,
                              goal.trajectory,
                              false);

    goal.trajectory.header.stamp = ros::Time::now()+ros::Duration(0.2);

    ROS_INFO("Sending trajectory with %d points and timestamp: %f",(int)goal.trajectory.points.size(),goal.trajectory.header.stamp.toSec());
    current_arm_controller_action_client_->sendGoal(goal,boost::bind(&HeadMonitor::controllerDoneCallback, this, _1, _2));
  }

  ///
  /// Convenience functions
  /// 

  trajectory_msgs::JointTrajectory generateHeadTrajectory(const std::string& link, const trajectory_msgs::JointTrajectory& joint_trajectory) {
    trajectory_msgs::JointTrajectory ret_traj;
    ret_traj.header = joint_trajectory.header;
    ret_traj.header.stamp = ros::Time::now();
    ret_traj.joint_names.resize(2); 
    ret_traj.joint_names[0] = "head_pan_joint";
    ret_traj.joint_names[1] = "head_tilt_joint";

    //TODO - incorporate Adam's stuff
    
    ret_traj.points.resize(2);

    ret_traj.points[0].positions.resize(2);
    ret_traj.points[1].positions.resize(2);
    
    ret_traj.points[0].positions[0] = start_head_pan_;
    ret_traj.points[0].positions[1] = start_head_tilt_;
    ret_traj.points[0].time_from_start = ros::Duration(.2);

    ret_traj.points[1].positions[0] = goal_head_pan_;
    ret_traj.points[1].positions[1] = goal_head_tilt_;
    ret_traj.points[1].time_from_start = joint_trajectory.points.back().time_from_start;

    return ret_traj;
  }

 // Points the head at a point in a given frame  
  void lookAt(std::string frame_id, double x, double y, double z, bool wait)
  {
    
    pr2_controllers_msgs::PointHeadGoal goal;

    //the target point, expressed in the requested frame
    geometry_msgs::PointStamped point;
    point.header.frame_id = frame_id;
    point.point.x = x; 
    point.point.y = y; 
    point.point.z = z;
  
    goal.target = point;
  
    //take at least 0.4 seconds to get there, lower value makes head jerky
    goal.min_duration = ros::Duration(.4);
    goal.max_velocity = 1.0;

    if(wait)
    {
      if(point_head_action_client_.sendGoalAndWait(goal, ros::Duration(5.0), ros::Duration(0.5)) != actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_WARN("Point head timed out, continuing");
    }
    else
    {
      point_head_action_client_.sendGoal(goal);
    }
  }

  int closestStateOnTrajectory(const trajectory_msgs::JointTrajectory &trajectory, const sensor_msgs::JointState &joint_state, unsigned int start, unsigned int end)
  {
    double dist = 0.0;
    int    pos  = -1;

    std::map<std::string, double> current_state_map;
    std::map<std::string, bool> continuous;
    for(unsigned int i = 0; i < joint_state.name.size(); i++) {
      current_state_map[joint_state.name[i]] = joint_state.position[i];
    }

    for(unsigned int j = 0; j < trajectory.joint_names.size(); j++) {
      std::string name = trajectory.joint_names[j];
      boost::shared_ptr<const urdf::Joint> joint = collision_models_interface_->getParsedDescription()->getJoint(name);
      if (joint.get() == NULL)
      {
        ROS_ERROR("Joint name %s not found in urdf model", name.c_str());
        return false;
      }
      if (joint->type == urdf::Joint::CONTINUOUS) {
        continuous[name] = true;
      } else {
        continuous[name] = false;
      }

    }

    for (unsigned int i = start ; i <= end ; ++i)
    {
      double d = 0.0;
      for(unsigned int j = 0; j < trajectory.joint_names.size(); j++) {
        double diff; 
        if(!continuous[trajectory.joint_names[j]]) {
          diff = fabs(trajectory.points[i].positions[j] - current_state_map[trajectory.joint_names[j]]);
        } else {
          diff = angles::shortest_angular_distance(trajectory.points[i].positions[j],current_state_map[trajectory.joint_names[j]]);
        }
        d += diff * diff;
      }
	
      if (pos < 0 || d < dist)
      {
        pos = i;
        dist = d;
      }
    }    

    // if(pos == 0) {
    //   for(unsigned int i = 0; i < joint_state.name.size(); i++) {
    //     ROS_INFO_STREAM("Current state for joint " << joint_state.name[i] << " is " << joint_state.position[i]);
    //   }
    //   for(unsigned int j = 0; j < trajectory.joint_names.size(); j++) {
    //     ROS_INFO_STREAM("Trajectory zero has " << trajectory.points[0].positions[j] << " for joint " << trajectory.joint_names[j]);
    //   }
    // }

    return pos;
  }

  bool removeCompletedTrajectory(const trajectory_msgs::JointTrajectory &trajectory_in, 
                                 const sensor_msgs::JointState& current_state, 
                                 trajectory_msgs::JointTrajectory &trajectory_out, 
                                 bool zero_vel_acc)
  {

    trajectory_out = trajectory_in;
    trajectory_out.points.clear();

    if(trajectory_in.points.empty())
    {
      ROS_WARN("No points in input trajectory");
      return true;
    }
        
    int current_position_index = 0;        
    //Get closest state in given trajectory
    current_position_index = closestStateOnTrajectory(trajectory_in, current_state, current_position_index, trajectory_in.points.size() - 1);
    if (current_position_index < 0)
    {
      ROS_ERROR("Unable to identify current state in trajectory");
      return false;
    } else {
      ROS_DEBUG_STREAM("Closest state is " << current_position_index << " of " << trajectory_in.points.size());
    }
    
    // Start one ahead of returned closest state index to make sure first trajectory point is not behind current state
    for(unsigned int i = current_position_index+1; i < trajectory_in.points.size(); ++i)
    {
      trajectory_out.points.push_back(trajectory_in.points[i]);
    }

    if(trajectory_out.points.empty())
    {
      ROS_DEBUG("No points in output trajectory");
      return false;
    }	

    ros::Duration first_time = trajectory_out.points[0].time_from_start;

    if(first_time < ros::Duration(.1)) {
      first_time = ros::Duration(0.0);
    } else {
      first_time -= ros::Duration(.1);
    }

    for(unsigned int i=0; i < trajectory_out.points.size(); ++i)
    {
      if(trajectory_out.points[i].time_from_start > first_time) {
        trajectory_out.points[i].time_from_start -= first_time;
      } else {
        ROS_INFO_STREAM("Not enough time in time from start for trajectory point " << i);
      }
    }

    if(zero_vel_acc) {
      for(unsigned int i=0; i < trajectory_out.joint_names.size(); ++i) {
        for(unsigned int j=0; j < trajectory_out.points.size(); ++j) {
          trajectory_out.points[j].velocities[i] = 0;
          trajectory_out.points[j].accelerations[i] = 0;
        }
      }
    }
    return true;
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_arm_head_monitor");

  ros::AsyncSpinner spinner(4); 
  spinner.start();

  HeadMonitor head_monitor;

  ros::waitForShutdown();
  return 0;
}

