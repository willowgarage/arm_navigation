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
 *  \author Sachin Chitta, Ioan Sucan
 *********************************************************************/

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <urdf/model.h>
#include <angles/angles.h>

#include <actionlib/client/action_client.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>

#include <actionlib/server/simple_action_server.h>
#include <move_arm_msgs/MoveArmStatistics.h>
#include <move_arm_msgs/MoveArmAction.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <kinematics_msgs/GetPositionFK.h>

#include <motion_planning_msgs/FilterJointTrajectoryWithConstraints.h>
#include <geometric_shapes_msgs/Shape.h>
#include <motion_planning_msgs/DisplayTrajectory.h>
#include <motion_planning_msgs/GetMotionPlan.h>
#include <motion_planning_msgs/ConvertToJointConstraint.h>
#include <motion_planning_msgs/convert_messages.h>
#include <motion_planning_msgs/ArmNavigationErrorCodes.h>

#include <visualization_msgs/Marker.h>

#include <planning_environment/util/construct_object.h>
#include <planning_environment_msgs/utils.h>
#include <geometric_shapes/bodies.h>

#include <visualization_msgs/MarkerArray.h>

#include <planning_environment/models/collision_models.h>
#include <planning_environment/models/model_utils.h>
#include <planning_environment_msgs/GetPlanningScene.h>
#include <planning_environment_msgs/LogPlanningScene.h>

#include <planning_environment_msgs/GetRobotState.h>

#include <move_arm_msgs/HeadMonitorAction.h>
#include <move_arm_msgs/PreplanHeadScanAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>

#include <move_arm/move_arm_warehouse_logger.h>

#include <std_msgs/Bool.h>

#include <valarray>
#include <algorithm>
#include <cstdlib>

namespace move_arm
{

enum MoveArmState {
  PLANNING,
  START_CONTROL,
  VISUALIZE_PLAN,
  MONITOR
};

enum EnvironmentServerChecks{
  COLLISION_TEST        = 1,
  PATH_CONSTRAINTS_TEST = 2,
  GOAL_CONSTRAINTS_TEST = 4,
  JOINT_LIMITS_TEST     = 8,
  CHECK_FULL_TRAJECTORY = 16
};	

typedef struct{
  bool accept_partial_plans;
  bool accept_invalid_goals;
  bool disable_ik;
  bool disable_collision_monitoring;
  bool is_pose_goal;
  double allowed_planning_time;
  std::string planner_service_name;
} MoveArmParameters;
  
static const std::string ARM_IK_NAME = "arm_ik";
static const std::string ARM_FK_NAME = "arm_fk";
static const std::string TRAJECTORY_FILTER = "filter_trajectory";
static const std::string DISPLAY_PATH_PUB_TOPIC  = "display_path";
static const std::string DISPLAY_JOINT_GOAL_PUB_TOPIC  = "display_joint_goal";

//bunch of statics for remapping purposes

static const std::string GET_PLANNING_SCENE_NAME = "get_planning_scene";
static const std::string LOG_PLANNING_SCENE_NAME = "/environment_server/log_planning_scene";
static const double MIN_TRAJECTORY_MONITORING_FREQUENCY = 1.0;
static const double MAX_TRAJECTORY_MONITORING_FREQUENCY = 100.0;
  
class MoveArm
{
public:	

  MoveArm(const std::string &group_name) :  
    group_(group_name), 
    private_handle_("~")
  {
    private_handle_.param<double>("move_arm_frequency",move_arm_frequency_, 50.0);
    private_handle_.param<double>("trajectory_filter_allowed_time",trajectory_filter_allowed_time_, 2.0);
    private_handle_.param<double>("ik_allowed_time",ik_allowed_time_, 2.0);

    private_handle_.param<std::string>("head_monitor_link",head_monitor_link_, std::string());
    private_handle_.param<double>("head_monitor_time_offset",head_monitor_time_offset_, 1.0);

    private_handle_.param<bool>("publish_stats",publish_stats_, true);
    private_handle_.param<bool>("log_to_warehouse", log_to_warehouse_, false);

    planning_scene_state_ = NULL;

    collision_models_ = new planning_environment::CollisionModels("robot_description");

    num_planning_attempts_ = 0;
    state_ = PLANNING;

    if(head_monitor_link_.empty())
    {
      ROS_WARN("No 'head_monitor_link' parameter specified, head monitoring will not be used.");
    } 

    if(log_to_warehouse_) {
      warehouse_logger_ = new MoveArmWarehouseLogger();
    } else {
      warehouse_logger_ = NULL;
    }

    ik_client_ = root_handle_.serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>(ARM_IK_NAME);
    allowed_contact_regions_publisher_ = root_handle_.advertise<visualization_msgs::MarkerArray>("allowed_contact_regions_array", 128);
    filter_trajectory_client_ = root_handle_.serviceClient<motion_planning_msgs::FilterJointTrajectoryWithConstraints>("filter_trajectory");      
    vis_marker_publisher_ = root_handle_.advertise<visualization_msgs::Marker>("move_" + group_name+"_markers", 128);
    vis_marker_array_publisher_ = root_handle_.advertise<visualization_msgs::MarkerArray>("move_" + group_name+"_markers_array", 128);
    get_state_client_ = root_handle_.serviceClient<planning_environment_msgs::GetRobotState>("get_robot_state");      

    get_planning_scene_client_ = root_handle_.serviceClient<planning_environment_msgs::GetPlanningScene>(GET_PLANNING_SCENE_NAME);
    log_planning_scene_client_ = root_handle_.serviceClient<planning_environment_msgs::LogPlanningScene>(LOG_PLANNING_SCENE_NAME);
    
    preplan_scan_action_client_.reset(new actionlib::SimpleActionClient<move_arm_msgs::PreplanHeadScanAction> ("preplan_head_scan", true));

    while(ros::ok() && !preplan_scan_action_client_->waitForServer(ros::Duration(10))) {
      ROS_WARN("No preplan scan service");
    }

    head_monitor_client_.reset(new actionlib::SimpleActionClient<move_arm_msgs::HeadMonitorAction> ("head_monitor_action", true));
    while(ros::ok() && !head_monitor_client_->waitForServer(ros::Duration(10))) {
      ROS_INFO("Waiting for head monitor server");
    }

    //    ros::service::waitForService(ARM_IK_NAME);
    arm_ik_initialized_ = false;
    ros::service::waitForService(GET_PLANNING_SCENE_NAME);
    ros::service::waitForService("filter_trajectory");

    action_server_.reset(new actionlib::SimpleActionServer<move_arm_msgs::MoveArmAction>(root_handle_, "move_" + group_name, boost::bind(&MoveArm::execute, this, _1), false));
    action_server_->start();

    display_path_publisher_ = root_handle_.advertise<motion_planning_msgs::DisplayTrajectory>(DISPLAY_PATH_PUB_TOPIC, 1, true);
    display_joint_goal_publisher_ = root_handle_.advertise<motion_planning_msgs::DisplayTrajectory>(DISPLAY_JOINT_GOAL_PUB_TOPIC, 1, true);
    stats_publisher_ = private_handle_.advertise<move_arm_msgs::MoveArmStatistics>("statistics",1,true);
    //        fk_client_ = root_handle_.serviceClient<kinematics_msgs::FKService>(ARM_FK_NAME);
  }	
  virtual ~MoveArm()
  {
    revertPlanningScene();
    delete collision_models_;
    if(warehouse_logger_ != NULL) {
      delete warehouse_logger_;
    }
  }

  bool configure()
  {
    if (group_.empty())
    {
      ROS_ERROR("No 'group' parameter specified. Without the name of the group of joints to plan for, action cannot start");
      return false;
    }
    const planning_models::KinematicModel::JointModelGroup* joint_model_group = collision_models_->getKinematicModel()->getModelGroup(group_);
    if(joint_model_group == NULL) {
      ROS_WARN_STREAM("No joint group " << group_);
      return false;
    }
    group_joint_names_ = joint_model_group->getJointModelNames();
    group_link_names_ = joint_model_group->getGroupLinkNames();
    return true;
  }
	
private:

  ///
  /// Kinematics
  /// 
  bool convertPoseGoalToJointGoal(motion_planning_msgs::GetMotionPlan::Request &req)
  {
    if(!arm_ik_initialized_)
    {
      if(!ros::service::waitForService(ARM_IK_NAME,ros::Duration(1.0)))
      {
        ROS_WARN("Inverse kinematics service is unavailable");
        return false;
      }
      else
      {
        arm_ik_initialized_ = true;
      }
    }


    ROS_DEBUG("Acting on goal to pose ...");// we do IK to find corresponding states
    ROS_DEBUG("Position constraint: %f %f %f",
              req.motion_plan_request.goal_constraints.position_constraints[0].position.x,
              req.motion_plan_request.goal_constraints.position_constraints[0].position.y,
              req.motion_plan_request.goal_constraints.position_constraints[0].position.z);
    ROS_DEBUG("Orientation constraint: %f %f %f %f",
              req.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.x,
              req.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.y,
              req.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.z,
              req.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.w);

    geometry_msgs::PoseStamped tpose = motion_planning_msgs::poseConstraintsToPoseStamped(req.motion_plan_request.goal_constraints.position_constraints[0],
                                                                                          req.motion_plan_request.goal_constraints.orientation_constraints[0]);
    std::string link_name = req.motion_plan_request.goal_constraints.position_constraints[0].link_name;
    sensor_msgs::JointState solution;		
    
    ROS_INFO("IK request");
    ROS_INFO("link_name   : %s",link_name.c_str());
    ROS_INFO("frame_id    : %s",tpose.header.frame_id.c_str());
    ROS_INFO("position    : (%f,%f,%f)",tpose.pose.position.x,tpose.pose.position.y,tpose.pose.position.z);
    ROS_INFO("orientation : (%f,%f,%f,%f)",tpose.pose.orientation.x,tpose.pose.orientation.y,tpose.pose.orientation.z,tpose.pose.orientation.w);
    ROS_INFO(" ");
    if (computeIK(tpose, 
                  link_name, 
                  solution))
    {
      /*if(!checkIK(tpose,link_name,solution))
        ROS_ERROR("IK solution does not get to desired pose");
      */
      std::map<std::string, double> joint_values;
      for (unsigned int i = 0 ; i < solution.name.size() ; ++i)
      {
        motion_planning_msgs::JointConstraint jc;
        jc.joint_name = solution.name[i];
        jc.position = solution.position[i];
        jc.tolerance_below = 0.01;
        jc.tolerance_above = 0.01;
        req.motion_plan_request.goal_constraints.joint_constraints.push_back(jc);
        joint_values[jc.joint_name] = jc.position;
      }
      motion_planning_msgs::ArmNavigationErrorCodes error_code;
      resetToStartState(planning_scene_state_);
      planning_scene_state_->setKinematicState(joint_values);
      if(!collision_models_->isKinematicStateValid(*planning_scene_state_,
                                                  group_joint_names_,
                                                  error_code,
                                                  original_request_.motion_plan_request.goal_constraints,
						   original_request_.motion_plan_request.path_constraints,
						   true))
      {
        ROS_INFO("IK returned joint state for goal that doesn't seem to be valid");
        if(error_code.val == error_code.GOAL_CONSTRAINTS_VIOLATED) {
          ROS_WARN("IK solution doesn't obey goal constraints");
        } else if(error_code.val == error_code.COLLISION_CONSTRAINTS_VIOLATED) {
          ROS_WARN("IK solution in collision");
        } else {
          ROS_WARN_STREAM("Some other problem with ik solution " << error_code.val);
        }
      }
      req.motion_plan_request.goal_constraints.position_constraints.clear();
      req.motion_plan_request.goal_constraints.orientation_constraints.clear();	    
      if(log_to_warehouse_) {
        warehouse_logger_->pushMotionPlanRequestToWarehouse(current_planning_scene_,
                                                            "after_ik",
                                                            req.motion_plan_request);
      }
      return true;
    }
    else
      return false;
  }

  bool computeIK(const geometry_msgs::PoseStamped &pose_stamped_msg,  
                 const std::string &link_name, 
                 sensor_msgs::JointState &solution)
  {
    kinematics_msgs::GetConstraintAwarePositionIK::Request request;
    kinematics_msgs::GetConstraintAwarePositionIK::Response response;
	    
    request.ik_request.pose_stamped = pose_stamped_msg;
    request.ik_request.robot_state = original_request_.motion_plan_request.start_state;
    request.ik_request.ik_seed_state = request.ik_request.robot_state;
    request.ik_request.ik_link_name = link_name;
    request.timeout = ros::Duration(ik_allowed_time_);
    request.constraints = original_request_.motion_plan_request.goal_constraints;
    if (ik_client_.call(request, response))
    {
      move_arm_action_result_.error_code = response.error_code;
      if(response.error_code.val != response.error_code.SUCCESS)
      {
        ROS_ERROR("IK Solution not found, IK returned with error_code: %d",response.error_code.val);
        return false;
      }         
      solution = response.solution.joint_state;
      if (solution.position.size() != group_joint_names_.size())
      {
        ROS_ERROR("Incorrect number of elements in IK output.");
        return false;
      }
      for(unsigned int i = 0; i < solution.position.size() ; ++i)
        ROS_DEBUG("IK[%d] = %f", (int)i, solution.position[i]);
    }
    else
    {
      ROS_ERROR("IK service failed");
      return false;
    }	    
    return true;
  }

  bool checkIK(const geometry_msgs::PoseStamped &pose_stamped_msg,  
               const std::string &link_name, 
               sensor_msgs::JointState &solution)
  {
    kinematics_msgs::GetPositionFK::Request request;
    kinematics_msgs::GetPositionFK::Response response;
	    
    request.robot_state.joint_state.name = group_joint_names_;	    
    request.fk_link_names.resize(1);
    request.fk_link_names[0] = link_name;
    request.robot_state.joint_state.position = solution.position;	    
    request.header = pose_stamped_msg.header;
    if (fk_client_.call(request, response))
    {
      if(response.error_code.val != response.error_code.SUCCESS)
        return false;
      ROS_DEBUG("Obtained FK solution");
      ROS_DEBUG("FK Pose:");
      ROS_DEBUG("Position : (%f,%f,%f)",
                response.pose_stamped[0].pose.position.x,
                response.pose_stamped[0].pose.position.y,
                response.pose_stamped[0].pose.position.z);
      ROS_DEBUG("Rotation : (%f,%f,%f,%f)",
                response.pose_stamped[0].pose.orientation.x,
                response.pose_stamped[0].pose.orientation.y,
                response.pose_stamped[0].pose.orientation.z,
                response.pose_stamped[0].pose.orientation.w);
      ROS_DEBUG(" ");
    }
    else
    {
      ROS_ERROR("FK service failed");
      return false;
    }	    
    return true;
  }
  ///
  /// End Kinematics
  /// 

  ///
  /// Trajectory Filtering
  ///
  bool filterTrajectory(const trajectory_msgs::JointTrajectory &trajectory_in, 
                        trajectory_msgs::JointTrajectory &trajectory_out)
  {
    motion_planning_msgs::FilterJointTrajectoryWithConstraints::Request  req;
    motion_planning_msgs::FilterJointTrajectoryWithConstraints::Response res;
    fillTrajectoryMsg(trajectory_in, req.trajectory);

    if(trajectory_filter_allowed_time_ == 0.0)
    {
      trajectory_out = req.trajectory;
      return true;
    }
    resetToStartState(planning_scene_state_);
    planning_environment::convertKinematicStateToRobotState(*planning_scene_state_,
                                                            ros::Time::now(),
                                                            collision_models_->getWorldFrameId(),
                                                            req.start_state);
    req.group_name = group_;
    req.path_constraints = original_request_.motion_plan_request.path_constraints;
    req.goal_constraints = original_request_.motion_plan_request.goal_constraints;
    req.allowed_time = ros::Duration(trajectory_filter_allowed_time_);
    ros::Time smoothing_time = ros::Time::now();
    if(filter_trajectory_client_.call(req,res))
    {
      move_arm_stats_.trajectory_duration = (res.trajectory.points.back().time_from_start-res.trajectory.points.front().time_from_start).toSec();
      move_arm_stats_.smoothing_time = (ros::Time::now()-smoothing_time).toSec();
      trajectory_out = res.trajectory;
      return true;
    }
    else
    {
      ROS_ERROR("Service call to filter trajectory failed.");
      return false;
    }
  }

  ///
  /// End Trajectory Filtering
  ///
 
  void discretizeTrajectory(const trajectory_msgs::JointTrajectory &trajectory, 
                            trajectory_msgs::JointTrajectory &trajectory_out,
                            const double &trajectory_discretization)
  {    
    trajectory_out.joint_names = trajectory.joint_names;
    for(unsigned int i=1; i < trajectory.points.size(); i++)
    {
      double diff = 0.0;      
      for(unsigned int j=0; j < trajectory.points[i].positions.size(); j++)
      {
        double start = trajectory.points[i-1].positions[j];
        double end   = trajectory.points[i].positions[j];
        if(fabs(end-start) > diff)
          diff = fabs(end-start);        
      }
      int num_intervals =(int) (diff/trajectory_discretization+1.0);
      
      for(unsigned int k=0; k < (unsigned int) num_intervals; k++)
      {
        trajectory_msgs::JointTrajectoryPoint point;
        for(unsigned int j=0; j < trajectory.points[i].positions.size(); j++)
        {
          double start = trajectory.points[i-1].positions[j];
          double end   = trajectory.points[i].positions[j];
          point.positions.push_back(start + (end-start)*k/num_intervals);
        }
        point.time_from_start = ros::Duration(trajectory.points[i].time_from_start.toSec() + k* (trajectory.points[i].time_from_start - trajectory.points[i-1].time_from_start).toSec()/num_intervals);
        trajectory_out.points.push_back(point);
      }
    }
    trajectory_out.points.push_back(trajectory.points.back());
  }
  ///
  /// State and trajectory validity checks
  ///

  ///
  /// Helper functions
  ///
  bool isPoseGoal(motion_planning_msgs::GetMotionPlan::Request &req)
  {
    if (req.motion_plan_request.goal_constraints.joint_constraints.empty() &&         // we have no joint constraints on the goal,
        req.motion_plan_request.goal_constraints.position_constraints.size() == 1 &&      // we have a single position constraint on the goal
        req.motion_plan_request.goal_constraints.orientation_constraints.size() ==  1)  // that is active on all 6 DOFs
      return true;
    else
      return false;
  }       
  bool hasPoseGoal(motion_planning_msgs::GetMotionPlan::Request &req)
  {
    if (req.motion_plan_request.goal_constraints.position_constraints.size() >= 1 &&      // we have a single position constraint on the goal
        req.motion_plan_request.goal_constraints.orientation_constraints.size() >=  1)  // that is active on all 6 DOFs
      return true;
    else
      return false;
  }       
  bool isJointGoal(motion_planning_msgs::GetMotionPlan::Request &req)
  {
    if (req.motion_plan_request.goal_constraints.position_constraints.empty() && 
        req.motion_plan_request.goal_constraints.orientation_constraints.empty() && 
        !req.motion_plan_request.goal_constraints.joint_constraints.empty())
      return true;
    else
      return false;
  }                   

  //stubbing out for now
  bool isExecutionSafe() {
    return true;
  }

  bool getRobotState(planning_models::KinematicState* state)
  {
    planning_environment_msgs::GetRobotState::Request req;
    planning_environment_msgs::GetRobotState::Response res;
    if(get_state_client_.call(req,res))
    {
      planning_environment::setRobotStateAndComputeTransforms(res.robot_state, *state);
    }
    else
    {
      ROS_ERROR("Service call to get robot state failed on %s",
                get_state_client_.getService().c_str());
      return false;
    }
    return true;
  }
  ///
  /// End Helper Functions
  ///

  ///
  /// Motion planning
  ///
  void moveArmGoalToPlannerRequest(const move_arm_msgs::MoveArmGoalConstPtr& goal, 
                                   motion_planning_msgs::GetMotionPlan::Request &req)
  {
    req.motion_plan_request.workspace_parameters.workspace_region_pose.header.stamp = ros::Time::now();
    req.motion_plan_request = goal->motion_plan_request;

    move_arm_parameters_.accept_partial_plans = goal->accept_partial_plans;
    move_arm_parameters_.accept_invalid_goals = goal->accept_invalid_goals;
    move_arm_parameters_.disable_ik           = goal->disable_ik;
    move_arm_parameters_.disable_collision_monitoring = goal->disable_collision_monitoring;
    move_arm_parameters_.allowed_planning_time = goal->motion_plan_request.allowed_planning_time.toSec();
    move_arm_parameters_.planner_service_name = goal->planner_service_name;
    // visualizeAllowedContactRegions(req.motion_plan_request.allowed_contacts);
    // ROS_INFO("Move arm: %d allowed contact regions",(int)req.motion_plan_request.allowed_contacts.size());
    // for(unsigned int i=0; i < req.motion_plan_request.allowed_contacts.size(); i++)
    // {
    //   ROS_INFO("Position                    : (%f,%f,%f)",req.motion_plan_request.allowed_contacts[i].pose_stamped.pose.position.x,req.motion_plan_request.allowed_contacts[i].pose_stamped.pose.position.y,req.motion_plan_request.allowed_contacts[i].pose_stamped.pose.position.z);
    //   ROS_INFO("Frame id                    : %s",req.motion_plan_request.allowed_contacts[i].pose_stamped.header.frame_id.c_str());
    //   ROS_INFO("Depth                       : %f",req.motion_plan_request.allowed_contacts[i].penetration_depth);
    //   ROS_INFO("Link                        : %s",req.motion_plan_request.allowed_contacts[i].link_names[0].c_str());
    //   ROS_INFO(" ");
    // }
  }
  bool doPrePlanningChecks(motion_planning_msgs::GetMotionPlan::Request &req,  
                           motion_planning_msgs::GetMotionPlan::Response &res)
  {
    motion_planning_msgs::Constraints empty_goal_constraints;
    if(planning_scene_state_ == NULL) {
      ROS_INFO("Can't do pre-planning checks without planning state");
    }
    resetToStartState(planning_scene_state_);
    motion_planning_msgs::ArmNavigationErrorCodes error_code;
    if(!collision_models_->isKinematicStateValid(*planning_scene_state_,
                                                 group_joint_names_,
                                                 error_code,
                                                 empty_goal_constraints,
                                                 original_request_.motion_plan_request.path_constraints,
						 true)) {
      if(error_code.val == error_code.COLLISION_CONSTRAINTS_VIOLATED) {
        move_arm_action_result_.error_code.val = error_code.START_STATE_IN_COLLISION;
        ROS_ERROR("Starting state is in collision, can't plan");
        visualization_msgs::MarkerArray arr;
        std_msgs::ColorRGBA point_color_;
        point_color_.a = 1.0;
        point_color_.r = 1.0;
        point_color_.g = .8;
        point_color_.b = 0.04;

        collision_models_->getAllCollisionPointMarkers(*planning_scene_state_,
                                                       arr,
                                                       point_color_,
                                                       ros::Duration(0.0)); 
        std_msgs::ColorRGBA col;
        col.a = .9;
        col.r = 1.0;
        col.b = 1.0;
        col.g = 0.0;
	/*
	  collision_models_->getRobotTrimeshMarkersGivenState(*planning_scene_state_,
	  arr,
	  true,
	  ros::Duration(0.0));
	*/
        vis_marker_array_publisher_.publish(arr);
      } else if (error_code.val == error_code.PATH_CONSTRAINTS_VIOLATED) {
        move_arm_action_result_.error_code.val = error_code.START_STATE_VIOLATES_PATH_CONSTRAINTS;
        ROS_ERROR("Starting state violated path constraints, can't plan");;
      } else if (error_code.val == error_code.JOINT_LIMITS_VIOLATED) {
        move_arm_action_result_.error_code.val = move_arm_action_result_.error_code.JOINT_LIMITS_VIOLATED;;
        ROS_ERROR("Start state violates joint limits, can't plan.");
      }
      if(log_to_warehouse_) {
        warehouse_logger_->pushOutcomeToWarehouse(current_planning_scene_,
                                                  "start_state", 
                                                  move_arm_action_result_.error_code);
      }
      ROS_INFO("Setting aborted because start state invalid");
      action_server_->setAborted(move_arm_action_result_);
      return false;
    }
    // processing and checking goal
    if (!move_arm_parameters_.disable_ik && isPoseGoal(req)) {
      ROS_INFO("Planning to a pose goal");
      if(!convertPoseGoalToJointGoal(req)) {
	ROS_INFO("Setting aborted because ik failed");
        if(log_to_warehouse_) {
          warehouse_logger_->pushOutcomeToWarehouse(current_planning_scene_,
                                                    "ik", 
                                                    move_arm_action_result_.error_code);
        }
	action_server_->setAborted(move_arm_action_result_);
	return false;
      }
    }
    //if we still have pose constraints at this point it's probably a constrained combo goal
    if(!hasPoseGoal(req)) {
      motion_planning_msgs::RobotState empty_state;
      empty_state.joint_state = motion_planning_msgs::jointConstraintsToJointState(req.motion_plan_request.goal_constraints.joint_constraints);
      planning_environment::setRobotStateAndComputeTransforms(empty_state, *planning_scene_state_);
      motion_planning_msgs::Constraints empty_constraints;
      if(!collision_models_->isKinematicStateValid(*planning_scene_state_,
						   group_joint_names_,
						   error_code,
						   original_request_.motion_plan_request.goal_constraints,
						   original_request_.motion_plan_request.path_constraints,
						   true)) {
	if(error_code.val == error_code.JOINT_LIMITS_VIOLATED) {
	  ROS_ERROR("Will not plan to requested joint goal since it violates joint limits constraints");
	  move_arm_action_result_.error_code.val = move_arm_action_result_.error_code.JOINT_LIMITS_VIOLATED;
	} else if(error_code.val == error_code.COLLISION_CONSTRAINTS_VIOLATED) {
	  ROS_ERROR("Will not plan to requested joint goal since it is in collision");
	  move_arm_action_result_.error_code.val = move_arm_action_result_.error_code.GOAL_IN_COLLISION;
	} else if(error_code.val == error_code.GOAL_CONSTRAINTS_VIOLATED) {
	  ROS_ERROR("Will not plan to requested joint goal since it violates goal constraints");
	  move_arm_action_result_.error_code.val = move_arm_action_result_.error_code.GOAL_VIOLATES_PATH_CONSTRAINTS;
	} else if(error_code.val == error_code.PATH_CONSTRAINTS_VIOLATED) {
	  ROS_ERROR("Will not plan to requested joint goal since it violates path constraints");
	  move_arm_action_result_.error_code.val = move_arm_action_result_.error_code.GOAL_VIOLATES_PATH_CONSTRAINTS;
	} else {
	  ROS_INFO_STREAM("Will not plan to request joint goal due to error code " << error_code.val);
	}
	ROS_INFO_STREAM("Setting aborted becuase joint goal is problematic");
        if(log_to_warehouse_) {
          warehouse_logger_->pushOutcomeToWarehouse(current_planning_scene_,
                                                    "goal_state", 
                                                    move_arm_action_result_.error_code);
        }        
	action_server_->setAborted(move_arm_action_result_);
	return false;
      }
    }
    return true;
  }

  bool createPlan(motion_planning_msgs::GetMotionPlan::Request &req,  
                  motion_planning_msgs::GetMotionPlan::Response &res)
  {
    ros::ServiceClient planning_client = root_handle_.serviceClient<motion_planning_msgs::GetMotionPlan>(move_arm_parameters_.planner_service_name);
    move_arm_stats_.planner_service_name = move_arm_parameters_.planner_service_name;
    ROS_DEBUG("Issuing request for motion plan");		    
    // call the planner and decide whether to use the path
    if (planning_client.call(req, res))
    {
      if (res.trajectory.joint_trajectory.points.empty())
      {
        ROS_WARN("Motion planner was unable to plan a path to goal");
        return false;
      }
      ROS_INFO("Motion planning succeeded");
      return true;
    }
    else
    {
      ROS_ERROR("Motion planning service failed on %s",planning_client.getService().c_str());
      return false;
    }
  }
  ///
  /// End Motion planning
  ///

  bool sendTrajectory(trajectory_msgs::JointTrajectory &current_trajectory)
  {
    head_monitor_done_ = false;
    head_monitor_error_code_.val = 0;
    current_trajectory.header.stamp = ros::Time::now()+ros::Duration(0.2);

    move_arm_msgs::HeadMonitorGoal goal;
    goal.group_name = group_;
    goal.joint_trajectory = current_trajectory;
    goal.target_link = head_monitor_link_;
    goal.time_offset = ros::Duration(head_monitor_time_offset_);

    ROS_INFO("Sending trajectory for monitoring with %d points and timestamp: %f",(int)goal.joint_trajectory.points.size(),goal.joint_trajectory.header.stamp.toSec());
    for(unsigned int i=0; i < goal.joint_trajectory.joint_names.size(); i++)
      ROS_INFO("Joint: %d name: %s",i,goal.joint_trajectory.joint_names[i].c_str());

    /*    for(unsigned int i = 0; i < goal.trajectory.points.size(); i++)
          {
          ROS_INFO("%f: %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
          goal.trajectory.points[i].time_from_start.toSec(),
          goal.trajectory.points[i].positions[0],
          goal.trajectory.points[i].positions[1],
          goal.trajectory.points[i].positions[2],
          goal.trajectory.points[i].positions[3],
          goal.trajectory.points[i].positions[4],
          goal.trajectory.points[i].positions[5],
          goal.trajectory.points[i].positions[6],
          goal.trajectory.points[i].velocities[0],
          goal.trajectory.points[i].velocities[1],
          goal.trajectory.points[i].velocities[2],
          goal.trajectory.points[i].velocities[3],
          goal.trajectory.points[i].velocities[4],
          goal.trajectory.points[i].velocities[5],
          goal.trajectory.points[i].velocities[6]);
          }*/
    head_monitor_client_->sendGoal(goal, 
                                   boost::bind(&MoveArm::monitorDoneCallback, this, _1, _2), 
                                   actionlib::SimpleActionClient<move_arm_msgs::HeadMonitorAction>::SimpleActiveCallback(),
                                   boost::bind(&MoveArm::monitorFeedbackCallback, this, _1));

    return true;
  }

  void fillTrajectoryMsg(const trajectory_msgs::JointTrajectory &trajectory_in, 
                         trajectory_msgs::JointTrajectory &trajectory_out)
  {
    trajectory_out = trajectory_in;
    if(trajectory_in.points.empty())
    {
      ROS_WARN("No points in trajectory");
      return;
    }
    // get the current state
    double d = 0.0;


    std::map<std::string, double> val_map;
    resetToStartState(planning_scene_state_);
    planning_scene_state_->getKinematicStateValues(val_map);
    sensor_msgs::JointState current;
    current.name = trajectory_out.joint_names;
    current.position.resize(trajectory_out.joint_names.size());
    for(unsigned int i = 0; i < trajectory_out.joint_names.size(); i++) {
      current.position[i] = val_map[trajectory_out.joint_names[i]];
    }
    std::map<std::string, bool> continuous;
    for(unsigned int j = 0; j < trajectory_in.joint_names.size(); j++) {
      std::string name = trajectory_in.joint_names[j];
      boost::shared_ptr<const urdf::Joint> joint = collision_models_->getParsedDescription()->getJoint(name);
      if (joint.get() == NULL)
      {
        ROS_ERROR("Joint name %s not found in urdf model", name.c_str());
        return;
      }
      if (joint->type == urdf::Joint::CONTINUOUS) {
        continuous[name] = true;
      } else {
        continuous[name] = false;
      }
    }
    for (unsigned int i = 0 ; i < current.position.size() ; ++i)
    {
      double diff; 
      if(!continuous[trajectory_in.joint_names[i]]) {
	diff = fabs(trajectory_in.points[0].positions[i] - val_map[trajectory_in.joint_names[i]]);
      } else {
	diff = angles::shortest_angular_distance(trajectory_in.points[0].positions[i],val_map[trajectory_in.joint_names[i]]);
      }
      d += diff * diff;
    }
    d = sqrt(d);	    
    // decide whether we place the current state in front of the trajectory_in
    int include_first = (d > 0.1) ? 1 : 0;
    double offset = 0.0;
    trajectory_out.points.resize(trajectory_in.points.size() + include_first);

    if (include_first)
    {
      ROS_INFO("Adding current state to front of trajectory");
      // ROS_INFO_STREAM("Old state " << trajectory_out.points[0].positions[0]
      // 		      << trajectory_out.points[0].positions[1]
      // 		      << trajectory_out.points[0].positions[2]
      // 		      << trajectory_out.points[0].positions[3]
      // 		      << trajectory_out.points[0].positions[4]
      // 		      << trajectory_out.points[0].positions[5]
      // 		      << trajectory_out.points[0].positions[6]);

      // ROS_INFO_STREAM("Current state " << current.position[0]
      // 		      << current.position[1]
      // 		      << current.position[2]
      // 		      << current.position[3]
      // 		      << current.position[4]
      // 		      << current.position[5]
      // 		      << current.position[6]);
      trajectory_out.points[0].positions = motion_planning_msgs::jointStateToJointTrajectoryPoint(current).positions;
      trajectory_out.points[0].time_from_start = ros::Duration(0.0);
      offset = 0.3 + d;
    } 
    for (unsigned int i = 0 ; i < trajectory_in.points.size() ; ++i)
    {
      trajectory_out.points[i+include_first].time_from_start = trajectory_in.points[i].time_from_start;
      trajectory_out.points[i+include_first].positions = trajectory_in.points[i].positions;
    }
    trajectory_out.header.stamp = ros::Time::now();
  }
  /// 
  /// End Control
  ///

  ///
  /// State machine
  ///
  void resetStateMachine()
  {
    num_planning_attempts_ = 0;
    current_trajectory_.points.clear();
    current_trajectory_.joint_names.clear();
    state_ = PLANNING;    
  }
  bool executeCycle(motion_planning_msgs::GetMotionPlan::Request &req)
  {
    motion_planning_msgs::GetMotionPlan::Response res;
    motion_planning_msgs::ArmNavigationErrorCodes error_code;
    
    switch(state_)
    {
    case PLANNING:
      {
        move_arm_action_feedback_.state = "planning";
        move_arm_action_feedback_.time_to_completion = ros::Duration(req.motion_plan_request.allowed_planning_time);
        action_server_->publishFeedback(move_arm_action_feedback_);

        if(!doPrePlanningChecks(req,res))
          return true;

        visualizeJointGoal(req);
        resetToStartState(planning_scene_state_);
        if(collision_models_->isKinematicStateValid(*planning_scene_state_,
                                                    group_joint_names_,
                                                    error_code,
                                                    original_request_.motion_plan_request.goal_constraints,
                                                    original_request_.motion_plan_request.path_constraints,
						    false)) {
          resetStateMachine();
	  move_arm_action_result_.error_code.val = move_arm_action_result_.error_code.SUCCESS;
          if(log_to_warehouse_) {
            warehouse_logger_->pushOutcomeToWarehouse(current_planning_scene_,
                                                      "start_state_at_goal",
                                                      move_arm_action_result_.error_code);
          }
	  action_server_->setSucceeded(move_arm_action_result_);
          ROS_INFO("Apparently start state satisfies goal");
          return true;
        }
        ros::Time planning_time = ros::Time::now();
        if(createPlan(req,res))
        {
          std::vector<motion_planning_msgs::ArmNavigationErrorCodes> traj_error_codes;
          move_arm_stats_.planning_time = (ros::Time::now()-planning_time).toSec();
          ROS_DEBUG("createPlan succeeded");
          resetToStartState(planning_scene_state_);
          if(log_to_warehouse_) {
            warehouse_logger_->pushJointTrajectoryToWarehouse(current_planning_scene_,
                                                              "planner", 
                                                              ros::Duration(move_arm_stats_.planning_time),
                                                              res.trajectory.joint_trajectory);
          }
          if(!collision_models_->isJointTrajectoryValid(*planning_scene_state_,
                                                        res.trajectory.joint_trajectory, 
                                                        original_request_.motion_plan_request.goal_constraints,
                                                        original_request_.motion_plan_request.path_constraints,
                                                        error_code,
                                                        traj_error_codes,
                                                        true))
          {
            if(error_code.val == error_code.COLLISION_CONSTRAINTS_VIOLATED) {
              ROS_WARN("Planner trajectory collides");
            } else if (error_code.val == error_code.PATH_CONSTRAINTS_VIOLATED) {
              ROS_WARN("Planner trajectory violates path constraints");
            } else if (error_code.val == error_code.JOINT_LIMITS_VIOLATED) {
              ROS_WARN("Planner trajectory violates joint limits");
            } else if (error_code.val == error_code.GOAL_CONSTRAINTS_VIOLATED) {
              ROS_WARN("Planner trajectory doesn't reach goal");
            }
            if(log_to_warehouse_) {
              warehouse_logger_->pushOutcomeToWarehouse(current_planning_scene_,
                                                        "planner",
                                                        error_code);
            }
	    num_planning_attempts_++;
	    if(num_planning_attempts_ > req.motion_plan_request.num_planning_attempts)
            {
              resetStateMachine();
              ROS_INFO_STREAM("Setting aborted because we're out of planning attempts");
              action_server_->setAborted(move_arm_action_result_);
              return true;
            }
          }
          else{
            ROS_DEBUG("Trajectory validity check was successful");
	    
	    current_trajectory_ = res.trajectory.joint_trajectory;
	    visualizePlan(current_trajectory_);
	    //          printTrajectory(current_trajectory_);
	    state_ = START_CONTROL;
	    ROS_INFO("Done planning. Transitioning to control");
	  }
        }
        else if(action_server_->isActive())
        {
          num_planning_attempts_++;
          motion_planning_msgs::ArmNavigationErrorCodes error_code;
          error_code.val = error_code.PLANNING_FAILED;
          if(log_to_warehouse_) {
	    warehouse_logger_->pushOutcomeToWarehouse(current_planning_scene_,
						      "planner",
						      error_code);
	  }
          if(num_planning_attempts_ > req.motion_plan_request.num_planning_attempts)
          {
            resetStateMachine();
            ROS_INFO_STREAM("Setting aborted because we're out of planning attempts");
            action_server_->setAborted(move_arm_action_result_);
            return true;
          }
        }
        else
        {
          ROS_ERROR("create plan failed");
        }
        break;
      }
    case START_CONTROL:
      {
        move_arm_action_feedback_.state = "start_control";
        move_arm_action_feedback_.time_to_completion = ros::Duration(1.0/move_arm_frequency_);
        action_server_->publishFeedback(move_arm_action_feedback_);
        ROS_DEBUG("Filtering Trajectory");
        trajectory_msgs::JointTrajectory filtered_trajectory;
        if(filterTrajectory(current_trajectory_, filtered_trajectory))
        {
          motion_planning_msgs::ArmNavigationErrorCodes error_code;
          std::vector<motion_planning_msgs::ArmNavigationErrorCodes> traj_error_codes;
          resetToStartState(planning_scene_state_);
          if(log_to_warehouse_) {
            warehouse_logger_->pushJointTrajectoryToWarehouse(current_planning_scene_,
                                                              "filter", 
                                                              ros::Duration(move_arm_stats_.smoothing_time),
                                                              filtered_trajectory);
          }
          if(!collision_models_->isJointTrajectoryValid(*planning_scene_state_,
                                                        filtered_trajectory,
                                                        original_request_.motion_plan_request.goal_constraints,
                                                        original_request_.motion_plan_request.path_constraints,
                                                        error_code,
                                                        traj_error_codes,
                                                        false))
          {
            if(error_code.val == error_code.COLLISION_CONSTRAINTS_VIOLATED) {
              ROS_WARN("Filtered trajectory collides");
            } else if (error_code.val == error_code.PATH_CONSTRAINTS_VIOLATED) {
              ROS_WARN("Filtered trajectory violates path constraints");
            } else if (error_code.val == error_code.JOINT_LIMITS_VIOLATED) {
              ROS_WARN("Filtered trajectory violates joint limits");
            } else if (error_code.val == error_code.GOAL_CONSTRAINTS_VIOLATED) {
              ROS_WARN("Filtered trajectory doesn't reach goal");
            }
            ROS_ERROR("Move arm will abort this goal.  Will replan");
            state_ = PLANNING;
	    num_planning_attempts_++;	    
            if(log_to_warehouse_) {
              warehouse_logger_->pushOutcomeToWarehouse(current_planning_scene_,
                                                        "filter",
                                                        error_code);
            }
	    if(num_planning_attempts_ > req.motion_plan_request.num_planning_attempts)
            {
              resetStateMachine();
              ROS_INFO_STREAM("Setting aborted because we're out of planning attempts");
              action_server_->setAborted(move_arm_action_result_);
              return true;
            }
            //resetStateMachine();
            //action_server_->setAborted(move_arm_action_result_);
	    break;
            //return true;
          }
          else{
            ROS_DEBUG("Trajectory validity check was successful");
          }
          current_trajectory_ = filtered_trajectory;
        } else {
          if(log_to_warehouse_) {
            motion_planning_msgs::ArmNavigationErrorCodes error_code;
            error_code.val = error_code.PLANNING_FAILED;
            warehouse_logger_->pushOutcomeToWarehouse(current_planning_scene_,
                                                      "filter_rejects_planner",
                                                      error_code);
          }
          resetStateMachine();
          ROS_INFO_STREAM("Setting aborted because trajectory filter call failed");
          action_server_->setAborted(move_arm_action_result_);
          return true;              
        }
        ROS_DEBUG("Sending trajectory");
        move_arm_stats_.time_to_execution = (ros::Time::now() - ros::Time(move_arm_stats_.time_to_execution)).toSec();
        if(sendTrajectory(current_trajectory_))
        {
          state_ = MONITOR;
        }
        else
        {
          resetStateMachine();
          ROS_INFO("Setting aborted because we couldn't send the trajectory");
          action_server_->setAborted(move_arm_action_result_);
          return true;              
        }
        break;
      }
    case MONITOR:
      {
        move_arm_action_feedback_.state = "monitor";
        move_arm_action_feedback_.time_to_completion = current_trajectory_.points.back().time_from_start;
        action_server_->publishFeedback(move_arm_action_feedback_);
        ROS_DEBUG("Start to monitor");
        if(head_monitor_done_)
        {
          move_arm_stats_.time_to_result = (ros::Time::now()-ros::Time(move_arm_stats_.time_to_result)).toSec();

          motion_planning_msgs::RobotState empty_state;
          motion_planning_msgs::ArmNavigationErrorCodes state_error_code;
          getRobotState(planning_scene_state_);
          if(collision_models_->isKinematicStateValid(*planning_scene_state_,
                                                      group_joint_names_,
                                                      state_error_code,
                                                      original_request_.motion_plan_request.goal_constraints,
                                                      original_request_.motion_plan_request.path_constraints,
						      true))
          {
            move_arm_action_result_.error_code.val = move_arm_action_result_.error_code.SUCCESS;
            resetStateMachine();
            action_server_->setSucceeded(move_arm_action_result_);
            if(head_monitor_error_code_.val == head_monitor_error_code_.TRAJECTORY_CONTROLLER_FAILED) {
              ROS_INFO("Monitor failed but we seem to be at goal");
              if(log_to_warehouse_) {
                warehouse_logger_->pushOutcomeToWarehouse(current_planning_scene_,
                                                          "trajectory_failed_at_goal",
                                                          move_arm_action_result_.error_code);
              }
            } else {
              if(log_to_warehouse_) {
                warehouse_logger_->pushOutcomeToWarehouse(current_planning_scene_,
                                                          "ok",
                                                          move_arm_action_result_.error_code);
              }
              ROS_INFO("Reached goal");
            }
            return true;
          }
          else
          {
            if(state_error_code.val == state_error_code.COLLISION_CONSTRAINTS_VIOLATED) {
              move_arm_action_result_.error_code.val = state_error_code.START_STATE_IN_COLLISION;
              ROS_WARN("Though trajectory is done current state is in collision");
            } else if (state_error_code.val == state_error_code.PATH_CONSTRAINTS_VIOLATED) {
              ROS_WARN("Though trajectory is done current state violates path constraints");
            } else if (state_error_code.val == state_error_code.JOINT_LIMITS_VIOLATED) {
              ROS_WARN("Though trajectory is done current state violates joint limits");
            } else if(state_error_code.val == state_error_code.GOAL_CONSTRAINTS_VIOLATED) {
              ROS_WARN("Though trajectory is done current state does not seem to be at goal");
            }
            if(log_to_warehouse_) {
              warehouse_logger_->pushOutcomeToWarehouse(current_planning_scene_,
                                                        "trajectory_failed_not_at_goal",
                                                        state_error_code);
            }
            resetStateMachine();
            action_server_->setAborted(move_arm_action_result_);
            ROS_INFO("Monitor done but not in good state");
	    return true;              
          }
        }
        break;
      }
    default:
      {
        ROS_INFO("Should not be here.");
        break;
      }
    }   
    if(!action_server_->isActive())
    {
      ROS_DEBUG("Move arm no longer has an active goal");
      return true;
    }
    return false;		
  }
  void execute(const move_arm_msgs::MoveArmGoalConstPtr& goal)
  {
    motion_planning_msgs::GetMotionPlan::Request req;	    
    moveArmGoalToPlannerRequest(goal,req);	    

    move_arm_msgs::PreplanHeadScanGoal preplan_scan_goal;
    preplan_scan_goal.group_name = group_;
    preplan_scan_goal.motion_plan_request = goal->motion_plan_request;
    preplan_scan_goal.head_monitor_link = head_monitor_link_;
    if(preplan_scan_action_client_->sendGoalAndWait(preplan_scan_goal, ros::Duration(30.0), ros::Duration(1.0)) != actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_WARN_STREAM("Preplan scan failed");
    }

    if(!getAndSetPlanningScene(goal->planning_scene_diff)) {
      ROS_INFO("Problem setting planning scene");
      move_arm_action_result_.error_code.val = move_arm_action_result_.error_code.INCOMPLETE_ROBOT_STATE;
      action_server_->setAborted(move_arm_action_result_);
      return;
    }

    collision_models_->convertConstraintsGivenNewWorldTransform(*planning_scene_state_,
                                                                req.motion_plan_request.goal_constraints);

    collision_models_->convertConstraintsGivenNewWorldTransform(*planning_scene_state_,
                                                                req.motion_plan_request.path_constraints);

    //overwriting start state - move arm only deals with current state state
    planning_environment::convertKinematicStateToRobotState(*planning_scene_state_,
                                                            ros::Time::now(),
                                                            collision_models_->getWorldFrameId(),
                                                            req.motion_plan_request.start_state);
    original_request_ = req;

    if(log_to_warehouse_) {
      warehouse_logger_->pushMotionPlanRequestToWarehouse(current_planning_scene_,
                                                          "original",
                                                          req.motion_plan_request);
    }
    
    ros::Rate move_arm_rate(move_arm_frequency_);
    move_arm_action_result_.contacts.clear();
    move_arm_action_result_.error_code.val = 0;
    move_arm_stats_.time_to_execution = ros::Time::now().toSec();
    move_arm_stats_.time_to_result = ros::Time::now().toSec();
    while(private_handle_.ok())
    {	    	    
      if (action_server_->isPreemptRequested())
      {
	if(log_to_warehouse_) {
	  move_arm_action_result_.error_code.val = move_arm_action_result_.error_code.TIMED_OUT;
	  warehouse_logger_->pushOutcomeToWarehouse(current_planning_scene_,
						    "preempted", 
						    move_arm_action_result_.error_code);
	}
        revertPlanningScene();
        move_arm_stats_.preempted = true;
        if(publish_stats_)
          publishStats();
        move_arm_stats_.time_to_execution = ros::Time::now().toSec();
        move_arm_stats_.time_to_result = ros::Time::now().toSec();
        if(action_server_->isNewGoalAvailable())
        {
          move_arm_action_result_.contacts.clear();
          move_arm_action_result_.error_code.val = 0;
          const move_arm_msgs::MoveArmGoalConstPtr& new_goal = action_server_->acceptNewGoal();
          moveArmGoalToPlannerRequest(new_goal,req);
          ROS_INFO("Received new goal, will preempt previous goal");
          if(!getAndSetPlanningScene(new_goal->planning_scene_diff)) {
            ROS_INFO("Problem setting planning scene");
            move_arm_action_result_.error_code.val = move_arm_action_result_.error_code.INCOMPLETE_ROBOT_STATE;
            action_server_->setAborted(move_arm_action_result_);
            return;
          }
          
          collision_models_->convertConstraintsGivenNewWorldTransform(*planning_scene_state_,
                                                                      req.motion_plan_request.goal_constraints);
          
          collision_models_->convertConstraintsGivenNewWorldTransform(*planning_scene_state_,
                                                                      req.motion_plan_request.path_constraints);
          planning_environment::convertKinematicStateToRobotState(*planning_scene_state_,
                                                                  ros::Time::now(),
                                                                  collision_models_->getWorldFrameId(),
                                                                  req.motion_plan_request.start_state);
          original_request_ = req;

          if(log_to_warehouse_) {
            warehouse_logger_->pushMotionPlanRequestToWarehouse(current_planning_scene_,
                                                                "original", 
                                                                req.motion_plan_request);
          }
          state_ = PLANNING;
        }
        else               //if we've been preempted explicitly we need to shut things down
        {
          ROS_INFO("The move arm action was preempted by the action client. Preempting this goal.");
          if(state_ == MONITOR) {
            head_monitor_client_->cancelGoal();
          }
          revertPlanningScene();
          resetStateMachine();
          action_server_->setPreempted();
          return;
        }
      }

      //for timing that gives real time even in simulation
      ros::WallTime start = ros::WallTime::now();

      //the real work on pursuing a goal is done here
      bool done = executeCycle(req);


      if(done)
      {
        if(publish_stats_)
          publishStats();
        return;
      }

      ros::WallDuration t_diff = ros::WallTime::now() - start;
      ROS_DEBUG("Full control cycle time: %.9f\n", t_diff.toSec());

      move_arm_rate.sleep();
    }	    
    //if the node is killed then we'll abort and return
    ROS_INFO("Node was killed, aborting");
    action_server_->setAborted(move_arm_action_result_);
  }

  void monitorDoneCallback(const actionlib::SimpleClientGoalState& state, 
                           const move_arm_msgs::HeadMonitorResultConstPtr& result) {
    //TODO - parse goal state for success or failure
    head_monitor_done_ = true;
    head_monitor_error_code_ = result->error_code;
    ROS_INFO_STREAM("Actual trajectory with " << result->actual_trajectory.points.size());
    if(log_to_warehouse_) {
      warehouse_logger_->pushJointTrajectoryToWarehouse(current_planning_scene_,
                                                        "monitor",
                                                        result->actual_trajectory.points.back().time_from_start,
                                                        result->actual_trajectory);
    }
  }

  void monitorFeedbackCallback(const move_arm_msgs::HeadMonitorFeedbackConstPtr& feedback) {
    ROS_INFO_STREAM("Got feedback from monitor");
    if(log_to_warehouse_) {
      warehouse_logger_->pushPausedStateToWarehouse(current_planning_scene_,
                                                    *feedback);
    }
  }


  bool logPlanningScene(std::string fn_suffix) {
    planning_environment_msgs::LogPlanningScene::Request log_req;
    planning_environment_msgs::LogPlanningScene::Response log_res;
    log_req.package_name = "move_arm";
    log_req.filename = "planning_scene_"+fn_suffix+".bag";
    if(!log_planning_scene_client_.call(log_req,log_res)) {
      ROS_WARN("Problem logging planning scene");
      return false;
    }
    return true;
  }

  bool getAndSetPlanningScene(const planning_environment_msgs::PlanningScene& planning_diff) {
    planning_environment_msgs::GetPlanningScene::Request planning_scene_req;
    planning_environment_msgs::GetPlanningScene::Response planning_scene_res;

    revertPlanningScene();

    planning_scene_req.planning_scene_diff = planning_diff;

    if(!get_planning_scene_client_.call(planning_scene_req, planning_scene_res)) {
      ROS_WARN("Can't get planning scene");
      return false;
    }

    current_planning_scene_ = planning_scene_res.planning_scene;

    if(log_to_warehouse_) {
      warehouse_logger_->pushPlanningSceneToWarehouse(current_planning_scene_);
    }

    planning_scene_state_ = collision_models_->setPlanningScene(current_planning_scene_);

    collision_models_->disableCollisionsForNonUpdatedLinks(group_);

    if(planning_scene_state_ == NULL) {
      ROS_WARN("Problems setting local state");
      return false;
    }    
    return true;
  }

  void resetToStartState(planning_models::KinematicState* state) {
    planning_environment::setRobotStateAndComputeTransforms(current_planning_scene_.robot_state, *state);
  }

  bool revertPlanningScene() {
    if(planning_scene_state_ != NULL) {
      collision_models_->revertPlanningScene(planning_scene_state_);
      planning_scene_state_ = NULL;
    }
    return true;
  }

  ///
  /// End State machine
  ///
	
  ///
  /// Visualization and I/O	
  ///
  void publishStats()
  {
    move_arm_stats_.error_code.val = move_arm_action_result_.error_code.val;
    move_arm_stats_.result = motion_planning_msgs::armNavigationErrorCodeToString(move_arm_action_result_.error_code);
    stats_publisher_.publish(move_arm_stats_);
    // Reset
    move_arm_stats_.error_code.val = 0;
    move_arm_stats_.result = " ";
    move_arm_stats_.request_id++;
    move_arm_stats_.planning_time = -1.0;
    move_arm_stats_.smoothing_time = -1.0;
    move_arm_stats_.ik_time = -1.0;
    move_arm_stats_.time_to_execution = -1.0;
    move_arm_stats_.time_to_result = -1.0;
    move_arm_stats_.preempted = false;
    move_arm_stats_.num_replans = 0.0;
    move_arm_stats_.trajectory_duration = -1.0;
  }

  void printTrajectory(const trajectory_msgs::JointTrajectory &trajectory)
  {
    for (unsigned int i = 0 ; i < trajectory.points.size() ; ++i)
    {
      std::stringstream ss;
      for (unsigned int j = 0 ; j < trajectory.points[i].positions.size() ; ++j)
        ss << trajectory.points[i].positions[j] << " ";
      ss << trajectory.points[i].time_from_start.toSec();
      ROS_DEBUG("%s", ss.str().c_str());
    }
  }	 
  void visualizeJointGoal(motion_planning_msgs::GetMotionPlan::Request &req)
  {
    //if(!isJointGoal(req))
    //{
    //  ROS_WARN("Only joint goals can be displayed");
    //  return;
    //}
    ROS_DEBUG("Displaying joint goal");
    motion_planning_msgs::DisplayTrajectory d_path;
    d_path.model_id = req.motion_plan_request.group_name;
    d_path.trajectory.joint_trajectory = motion_planning_msgs::jointConstraintsToJointTrajectory(req.motion_plan_request.goal_constraints.joint_constraints);
    resetToStartState(planning_scene_state_);
    planning_environment::convertKinematicStateToRobotState(*planning_scene_state_,
                                                            ros::Time::now(),
                                                            collision_models_->getWorldFrameId(),
                                                            d_path.robot_state);
    display_joint_goal_publisher_.publish(d_path);
    ROS_INFO("Displaying move arm joint goal.");
  }
  void visualizeJointGoal(const trajectory_msgs::JointTrajectory &trajectory)
  {
    ROS_DEBUG("Displaying joint goal");
    motion_planning_msgs::DisplayTrajectory d_path;
    d_path.trajectory.joint_trajectory = trajectory;
    resetToStartState(planning_scene_state_);
    planning_environment::convertKinematicStateToRobotState(*planning_scene_state_,
                                                            ros::Time::now(),
                                                            collision_models_->getWorldFrameId(),
                                                            d_path.robot_state);
    display_joint_goal_publisher_.publish(d_path);
  }
  void visualizePlan(const trajectory_msgs::JointTrajectory &trajectory)
  {
    move_arm_action_feedback_.state = "visualizing plan";
    if(action_server_->isActive())
      action_server_->publishFeedback(move_arm_action_feedback_);
    motion_planning_msgs::DisplayTrajectory d_path;
    d_path.model_id = original_request_.motion_plan_request.group_name;
    d_path.trajectory.joint_trajectory = trajectory;
    resetToStartState(planning_scene_state_);
    planning_environment::convertKinematicStateToRobotState(*planning_scene_state_,
                                                            ros::Time::now(),
                                                            collision_models_->getWorldFrameId(),
                                                            d_path.robot_state);
    display_path_publisher_.publish(d_path);
  }
  void visualizeAllowedContactRegions(const std::vector<motion_planning_msgs::AllowedContactSpecification> &allowed_contacts)
  {
    static int count = 0;
    visualization_msgs::MarkerArray mk;
    mk.markers.resize(allowed_contacts.size());
    for(unsigned int i=0; i < allowed_contacts.size(); i++) 
    { 
      bool valid_shape = true;
      mk.markers[i].header.stamp = ros::Time::now();
      mk.markers[i].header.frame_id = allowed_contacts[i].pose_stamped.header.frame_id;
      mk.markers[i].ns = "move_arm::"+allowed_contacts[i].name;
      mk.markers[i].id = count++;
      if(allowed_contacts[i].shape.type == geometric_shapes_msgs::Shape::SPHERE)
      {        
        mk.markers[i].type = visualization_msgs::Marker::SPHERE;
        if(allowed_contacts[i].shape.dimensions.size() >= 1)
          mk.markers[i].scale.x = mk.markers[i].scale.y = mk.markers[i].scale.z = allowed_contacts[i].shape.dimensions[0];
        else
          valid_shape = false;
      }      
      else if (allowed_contacts[i].shape.type == geometric_shapes_msgs::Shape::BOX)
      {
        mk.markers[i].type = visualization_msgs::Marker::CUBE;
        if(allowed_contacts[i].shape.dimensions.size() >= 3)
        {
          mk.markers[i].scale.x = allowed_contacts[i].shape.dimensions[0];
          mk.markers[i].scale.y = allowed_contacts[i].shape.dimensions[1];
          mk.markers[i].scale.z = allowed_contacts[i].shape.dimensions[2];
        }
        else
          valid_shape = false;
      }
      else if (allowed_contacts[i].shape.type == geometric_shapes_msgs::Shape::CYLINDER)
      {
        mk.markers[i].type = visualization_msgs::Marker::CYLINDER;
        if(allowed_contacts[i].shape.dimensions.size() >= 2)
        {
          mk.markers[i].scale.x = allowed_contacts[i].shape.dimensions[0];
          mk.markers[i].scale.y = allowed_contacts[i].shape.dimensions[0];
          mk.markers[i].scale.z = allowed_contacts[i].shape.dimensions[1];
        }
        else
          valid_shape = false;
      }
      else
      {
        mk.markers[i].scale.x = mk.markers[i].scale.y = mk.markers[i].scale.z = 0.01;
        valid_shape = false;
      }        

      mk.markers[i].action = visualization_msgs::Marker::ADD;
      mk.markers[i].pose = allowed_contacts[i].pose_stamped.pose;  
      if(!valid_shape)
      {
        mk.markers[i].scale.x = mk.markers[i].scale.y = mk.markers[i].scale.z = 0.01;
        mk.markers[i].color.a = 0.3;
        mk.markers[i].color.r = 1.0;
        mk.markers[i].color.g = 0.04;
        mk.markers[i].color.b = 0.04;
      }
      else
      {
        mk.markers[i].color.a = 1.0;
        mk.markers[i].color.r = 0.04;
        mk.markers[i].color.g = 1.0;
        mk.markers[i].color.b = 0.04;
      }  
      //mk.markers[i].lifetime = ros::Duration(30.0);  
    }
    allowed_contact_regions_publisher_.publish(mk);
  }
  ///
  /// End Visualization and I/O	
  ///

private:

  std::string group_;

  boost::shared_ptr<actionlib::SimpleActionClient<move_arm_msgs::HeadMonitorAction> >  head_monitor_client_;
  boost::shared_ptr<actionlib::SimpleActionClient<move_arm_msgs::PreplanHeadScanAction> >  preplan_scan_action_client_;

  ros::ServiceClient ik_client_;
  ros::ServiceClient trajectory_start_client_,trajectory_cancel_client_,trajectory_query_client_;	
  ros::NodeHandle private_handle_, root_handle_;
  boost::shared_ptr<actionlib::SimpleActionServer<move_arm_msgs::MoveArmAction> > action_server_;	

  planning_environment::CollisionModels* collision_models_;

  planning_environment_msgs::GetPlanningScene::Request get_planning_scene_req_;
  planning_environment_msgs::GetPlanningScene::Response get_planning_scene_res_;
  planning_environment_msgs::PlanningScene current_planning_scene_;
  planning_models::KinematicState* planning_scene_state_;

  tf::TransformListener *tf_;
  MoveArmState state_;
  double move_arm_frequency_;      	
  trajectory_msgs::JointTrajectory current_trajectory_;

  int num_planning_attempts_;

  std::vector<std::string> group_joint_names_;
  std::vector<std::string> group_link_names_;
  std::vector<std::string> all_link_names_;
  move_arm_msgs::MoveArmResult move_arm_action_result_;
  move_arm_msgs::MoveArmFeedback move_arm_action_feedback_;

  motion_planning_msgs::GetMotionPlan::Request original_request_;

  ros::Publisher display_path_publisher_;
  ros::Publisher display_joint_goal_publisher_;
  ros::Publisher allowed_contact_regions_publisher_;
  ros::Publisher vis_marker_publisher_;
  ros::Publisher vis_marker_array_publisher_;
  ros::ServiceClient filter_trajectory_client_;
  ros::ServiceClient fk_client_;
  ros::ServiceClient get_state_client_;
  ros::ServiceClient get_planning_scene_client_;
  ros::ServiceClient log_planning_scene_client_;
  MoveArmParameters move_arm_parameters_;

  double trajectory_filter_allowed_time_, ik_allowed_time_;
  double trajectory_discretization_;
  bool arm_ik_initialized_;
  
  std::string head_monitor_link_;
  double head_monitor_time_offset_;
  motion_planning_msgs::ArmNavigationErrorCodes head_monitor_error_code_;
  bool head_monitor_done_;

  bool publish_stats_;
  move_arm_msgs::MoveArmStatistics move_arm_stats_;
  ros::Publisher stats_publisher_;
  
  bool log_to_warehouse_;
  MoveArmWarehouseLogger* warehouse_logger_;

};
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_arm");
  
  ros::AsyncSpinner spinner(1); // Use 1 thread
  spinner.start();
  ros::NodeHandle nh("~");
  std::string group;
  nh.param<std::string>("group", group, std::string());
  ROS_INFO("Move arm operating on group %s",group.c_str());    
  move_arm::MoveArm move_arm(group);
  if(!move_arm.configure())
  {
    ROS_ERROR("Could not configure move arm, exiting");
    ros::shutdown();
    return 1;
  }
  ROS_INFO("Move arm action started");
  ros::waitForShutdown();
    
  return 0;
}
