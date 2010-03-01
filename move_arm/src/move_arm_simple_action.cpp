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

#include <arm_control_msgs/utils.h>
#include <arm_control_msgs/TrajectoryStart.h>
#include <arm_control_msgs/TrajectoryQuery.h>
#include <arm_control_msgs/TrajectoryCancel.h>


#include <actionlib/server/simple_action_server.h>
#include <move_arm_msgs/MoveArmAction.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <kinematics_msgs/GetCollisionFreePositionIK.h>
#include <kinematics_msgs/GetPositionFK.h>

#include <motion_planning_msgs/FilterJointTrajectory.h>
#include <geometric_shapes_msgs/Shape.h>
#include <motion_planning_msgs/DisplayTrajectory.h>
#include <motion_planning_msgs/GetMotionPlan.h>
#include <motion_planning_msgs/ConvertToJointConstraint.h>
#include <motion_planning_msgs/convert_messages.h>

#include <visualization_msgs/Marker.h>

#include <planning_environment/util/construct_object.h>
#include <planning_environment/monitors/joint_state_monitor.h>
#include <geometric_shapes/bodies.h>

#include <planning_environment_msgs/GetRobotState.h>
#include <planning_environment_msgs/GetJointTrajectoryValidity.h>
#include <planning_environment_msgs/GetStateValidity.h>
#include <planning_environment_msgs/GetJointsInGroup.h>
#include <planning_environment_msgs/GetEnvironmentSafety.h>
#include <planning_environment_msgs/SetConstraints.h>

#include <visualization_msgs/MarkerArray.h>

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
  WAIT_FOR_VISUALIZER,
  MONITOR
};

/// the string used internally to access control starting service; this should be remaped in the launch file
static const std::string CONTROL_START_NAME      = "controller_start";

/// the string used internally to access control querying service; this should be remaped in the launch file
static const std::string CONTROL_QUERY_NAME      = "controller_query";

/// the string used internally to access control canceling service; this should be remaped in the launch file
static const std::string CONTROL_CANCEL_NAME     = "controller_cancel";

/// the string used internally to access valid state searching service; this should be remaped in the launch file
static const std::string SEARCH_VALID_STATE_NAME = "get_valid_state";

/// the string used internally to access inverse kinematics service; this should be remaped in the launch file
static const std::string ARM_IK_NAME             = "arm_ik";
/// the string used internally to access inverse kinematics service; this should be remaped in the launch file
static const std::string ARM_FK_NAME             = "arm_fk";

static const std::string DISPLAY_PATH_PUB_TOPIC  = "display_path";
static const std::string DISPLAY_PATH_SUB_TOPIC  = DISPLAY_PATH_PUB_TOPIC + std::string("state");

static const std::string DISPLAY_JOINT_GOAL_PUB_TOPIC  = "display_joint_goal";

static const std::string TRAJECTORY_FILTER = "filter_trajectory";

class MoveArm
{
public:	

  MoveArm(const std::string &group_name) :  group_(group_name), node_handle_("~")
  {
    node_handle_.param<bool>("perform_ik", perform_ik_, true);
    node_handle_.param<bool>("monitor_execution", monitor_execution_, true);
    //        node_handle_.param<std::string>("planner_service_name", planner_service_name_, "plan_path");
    node_handle_.param<double>("move_arm_frequency",move_arm_frequency_, 50.0);
    node_handle_.param<bool>("visualize_plan",visualize_plan_, false);
    node_handle_.param<bool>("wait_for_visualizer",wait_for_visualizer_, false);
    node_handle_.param<double>("wait_for_visualizer_timeout",wait_for_visualizer_timeout_, 10.0);
    node_handle_.param<bool>("visualize_joint_goal",display_joint_goal_, false);

    ROS_INFO("IK will %sbe performed", perform_ik_ ? "" : "not ");
    ROS_INFO("Trajectory execution will %sbe monitored for safe behavior",monitor_execution_ ? "" : "not ");
    current_trajectory_id_ = -1;
    num_planning_attempts_ = 0;
    state_ = PLANNING;

    done_visualizer_ = false;
    ik_client_ = root_handle_.serviceClient<kinematics_msgs::GetCollisionFreePositionIK>(ARM_IK_NAME);
    trajectory_start_client_  = root_handle_.serviceClient<arm_control_msgs::TrajectoryStart>(CONTROL_START_NAME);
    trajectory_query_client_  = root_handle_.serviceClient<arm_control_msgs::TrajectoryQuery>(CONTROL_QUERY_NAME);
    trajectory_cancel_client_ = root_handle_.serviceClient<arm_control_msgs::TrajectoryCancel>(CONTROL_CANCEL_NAME);

    check_plan_validity_client_ = root_handle_.serviceClient<planning_environment_msgs::GetJointTrajectoryValidity>("get_trajectory_validity");
    check_env_safe_client_ = root_handle_.serviceClient<planning_environment_msgs::GetEnvironmentSafety>("get_environment_safety");
    check_state_validity_client_ = root_handle_.serviceClient<planning_environment_msgs::GetStateValidity>("get_state_validity");
    check_execution_safe_client_ = root_handle_.serviceClient<planning_environment_msgs::GetJointTrajectoryValidity>("get_execution_safety");
    update_planning_monitor_client_ = root_handle_.serviceClient<planning_environment_msgs::SetConstraints>("set_constraints");
    get_joints_in_group_client_ = root_handle_.serviceClient<planning_environment_msgs::GetJointsInGroup>("get_joints_in_group");      
    get_state_client_ = root_handle_.serviceClient<planning_environment_msgs::GetRobotState>("get_robot_state");      

    allowed_contact_regions_publisher_ = root_handle_.advertise<visualization_msgs::MarkerArray>("allowed_contact_regions_array", 128);
    filter_trajectory_client_ = root_handle_.serviceClient<motion_planning_msgs::FilterJointTrajectory>("filter_trajectory");      

    ros::service::waitForService(ARM_IK_NAME);
    ros::service::waitForService(CONTROL_START_NAME);      
    ros::service::waitForService(CONTROL_START_NAME);
    ros::service::waitForService(CONTROL_CANCEL_NAME);

    ros::service::waitForService("get_trajectory_validity");
    ros::service::waitForService("get_environment_safety");
    ros::service::waitForService("get_state_validity");
    ros::service::waitForService("get_execution_safety");
    ros::service::waitForService("set_constraints");
    ros::service::waitForService("get_joints_in_group");
    ros::service::waitForService("get_robot_state");
    ros::service::waitForService("filter_trajectory");

    last_valid_plan_time_ = ros::Time();

    action_server_.reset(new actionlib::SimpleActionServer<move_arm_msgs::MoveArmAction>(root_handle_, "move_" + group_name, boost::bind(&MoveArm::execute, this, _1)));
    // advertise the topic for displaying plans
    display_path_publisher_ = root_handle_.advertise<motion_planning_msgs::DisplayTrajectory>(DISPLAY_PATH_PUB_TOPIC, 1);

    display_joint_goal_publisher_ = root_handle_.advertise<motion_planning_msgs::DisplayTrajectory>(DISPLAY_JOINT_GOAL_PUB_TOPIC, 1);

    //        fk_client_ = root_handle_.serviceClient<kinematics_msgs::FKService>(ARM_FK_NAME);
    display_path_subscriber_ = root_handle_.subscribe(DISPLAY_PATH_SUB_TOPIC, 1, &MoveArm::displayPathCallback, this);

  }	
  virtual ~MoveArm()
  {
  }

  bool configure()
  {
    if(!state_monitor_.active_)
    {
      ROS_ERROR("State monitor is not active, aborting.");
      return false;
    }          

    if (group_.empty())
    {
      ROS_ERROR("No 'group' parameter specified. Without the name of the group of joints to plan for, action cannot start");
      return false;
    }
    else
    {
      planning_environment_msgs::GetJointsInGroup::Request req;
      planning_environment_msgs::GetJointsInGroup::Response res;
      req.group_name = group_;
      if(get_joints_in_group_client_.call(req,res))
      {
        if(!res.joint_names.empty())
        {
          group_joint_names_ = res.joint_names;
          return true;
        }
        else
        {
          ROS_ERROR("Could not get the list of joint names in the group: %s",group_.c_str());
          return false;
        }
      }
      else
      {
        ROS_ERROR("Service call to find list of joint names failed on %s",get_joints_in_group_client_.getService().c_str());
        return false;
      }
    }
  }
	
private:
		
  void updateRequest(motion_planning_msgs::GetMotionPlan::Request &req, 
                     const sensor_msgs::JointState &joint_state)
  {
    // update request
    for (unsigned int i = 0 ; i < joint_state.name.size() ; ++i)
    {
      motion_planning_msgs::JointConstraint jc;
      jc.joint_name = joint_state.name[i];
      //      jc.header.frame_id = joint_state.header.frame_id;
      //      jc.header.stamp = state_monitor_.last_update_;//planning_monitor_->lastJointStateUpdate();
      jc.position = joint_state.position[i];
      jc.tolerance_below = 0.1;
      jc.tolerance_above = 0.1;
      req.goal_constraints.joint_constraints.push_back(jc);
    }
    req.goal_constraints.position_constraints.clear();
    req.goal_constraints.orientation_constraints.clear();	    
  }
	

  bool convertPoseGoalToJointGoal(motion_planning_msgs::GetMotionPlan::Request &req)
  {
    ROS_INFO("Acting on goal to pose ...");// we do IK to find corresponding states
    ROS_INFO("Original pose constraint: %f %f %f %f",req.goal_constraints.orientation_constraints[0].orientation.x,req.goal_constraints.orientation_constraints[0].orientation.y,req.goal_constraints.orientation_constraints[0].orientation.z,req.goal_constraints.orientation_constraints[0].orientation.w);
    geometry_msgs::PoseStamped tpose = motion_planning_msgs::poseConstraintsToPoseStamped(req.goal_constraints.position_constraints[0],req.goal_constraints.orientation_constraints[0]);//there's only one constraint	    
    std::string link_name = req.goal_constraints.position_constraints[0].link_name;
    sensor_msgs::JointState solution;		

    ROS_INFO("Requested IK Pose: %s",tpose.header.frame_id.c_str());
    ROS_INFO("Position : (%f,%f,%f)",tpose.pose.position.x,tpose.pose.position.y,tpose.pose.position.z);
    ROS_INFO("Rotation : (%f,%f,%f,%f)",tpose.pose.orientation.x,tpose.pose.orientation.y,tpose.pose.orientation.z,tpose.pose.orientation.w);
    ROS_INFO(" ");
    motion_planning_msgs::ArmNavigationErrorCodes error_code;
    if (computeIK(tpose, link_name, solution, error_code, req.workspace_parameters.ordered_collision_operations))
    {
      /*            if(!checkIK(tpose,link_name,solution))
                    {
                    ROS_ERROR("IK solution does not get to desired pose");
                    }*/
      updateRequest(req, solution);
      return true;
    }
    else
    {
      feedback_.error_code = error_code;
      action_server_->publishFeedback(feedback_);
      ROS_ERROR("Could not find an IK solution for requested goal. Error code was: %d",error_code.val);
      action_server_->setAborted();
      return false;
    }
  }

  bool filterTrajectory(const trajectory_msgs::JointTrajectory &trajectory_in, 
                        trajectory_msgs::JointTrajectory &trajectory_out)
  {
    motion_planning_msgs::FilterJointTrajectory::Request  req;
    motion_planning_msgs::FilterJointTrajectory::Response res;
    fillTrajectoryMsg(trajectory_in, req.trajectory);
    //req.trajectory = trajectory_in;

    if(filter_trajectory_client_.call(req,res))
    {
      trajectory_out = res.trajectory;
      return true;
    }
    else
    {
      ROS_ERROR("Service call to filter trajectory failed.");
      return false;
    }
  }


  /** \brief Check the joint goal if it is valid */
  bool checkJointGoal(motion_planning_msgs::GetMotionPlan::Request &req, const int &move_arm_options)
  {
    ROS_DEBUG("Acting on goal to set of joints ...");
    ROS_DEBUG("Checking validity of goal");

    motion_planning_msgs::ArmNavigationErrorCodes error_code;
    // try to skip straight to planning
    if (isStateValid(motion_planning_msgs::jointConstraintsToJointState(req.goal_constraints.joint_constraints),planning_environment_msgs::GetJointTrajectoryValidity::Request::JOINT_LIMITS_TEST))
    {
      ROS_DEBUG("Inside plan to joint goal");
      return true;
    }
    else
    {
      ROS_ERROR("Could not plan to requested joint goal since it was invalid");
      if(move_arm_options & move_arm_msgs::MoveArmGoal::ACCEPT_INVALID_GOALS)
        return true;
      else
      {
        action_server_->setAborted();
        return false;
      }
    }
  }

  /** \brief Depending on the type of constraint, decide whether or not to use IK, decide which planners to use */
  bool isPoseGoal(motion_planning_msgs::GetMotionPlan::Request &req)
  {
    ROS_DEBUG("Acting on goal...");	    
    // change pose constraints to joint constraints, if possible and so desired
    if (req.goal_constraints.joint_constraints.empty() &&         // we have no joint constraints on the goal,
        req.goal_constraints.position_constraints.size() == 1 &&      // we have a single position constraint on the goal
        req.goal_constraints.orientation_constraints.size() ==  1)  // that is active on all 6 DOFs
      return true;
    else
      return false;
  }       

  /** \brief Depending on the type of constraint, decide whether or not to use IK, decide which planners to use */
  bool isJointGoal(motion_planning_msgs::GetMotionPlan::Request &req)
  {
    ROS_DEBUG("Acting on goal...");	    
    if (req.goal_constraints.position_constraints.empty() && req.goal_constraints.orientation_constraints.empty() && !req.goal_constraints.joint_constraints.empty())
      return true;
    else
      return false;
  }                   
	
  /** \brief Depending on the type of constraint, decide whether or not to use IK, decide which planners to use */
  bool processAndCheckGoal(motion_planning_msgs::GetMotionPlan::Request &req)
  {
    ROS_DEBUG("Acting on goal...");
	    
    // change pose constraints to joint constraints, if possible and so desired
    if (perform_ik_ && req.goal_constraints.joint_constraints.empty() &&         // we have no joint constraints on the goal,
        req.goal_constraints.position_constraints.size() == 1 &&      // we have a single position constraint on the goal
        req.goal_constraints.orientation_constraints.size() ==  1)  // that is active on all 6 DOFs
    {
      ROS_INFO("Planning to a pose goal");
      if(!convertPoseGoalToJointGoal(req))
      {
        return false;
      }
    }
    // if we have only joint constraints, we call the method that gets us to a goal state
    else if (req.goal_constraints.position_constraints.empty() && req.goal_constraints.orientation_constraints.empty())
    {
      ROS_INFO("Planning to a joint goal");
      if(!checkJointGoal(req,move_arm_options_))
        return false;
    }
    return true;
  }       

  bool isEnvironmentSafe()
  {
    planning_environment_msgs::GetEnvironmentSafety::Request req;
    planning_environment_msgs::GetEnvironmentSafety::Response res;
    if(check_env_safe_client_.call(req,res))
    {
      if(res.error_code.val == res.error_code.SUCCESS)
        return true;
      else
      {
        feedback_.error_code = res.error_code;
        action_server_->publishFeedback(feedback_);        
        return false;
      }
    }
    else
    {
      ROS_ERROR("Service call to planning monitor failed on %s",check_env_safe_client_.getService().c_str());
      return false;
    }
  }

  bool isTrajectoryValid(const trajectory_msgs::JointTrajectory &traj)
  {
    planning_environment_msgs::GetJointTrajectoryValidity::Request req;
    planning_environment_msgs::GetJointTrajectoryValidity::Response res;
        
    ROS_DEBUG("Received plan has %d points with %d joints",(int) traj.points.size(),(int)traj.joint_names.size());

    req.trajectory = traj;
    if(!getRobotState(req.robot_state))
    {
      ROS_ERROR("Could not get robot state");
      return false;
    }
    ROS_DEBUG("Got robot state");

    //        req.robot_state.joint_state = state_monitor_.getJointState();
    req.start_index = 0;
    req.end_index = (int)req.trajectory.points.size()-1;
    req.flag = planning_environment_msgs::GetJointTrajectoryValidity::Request::PATH_CONSTRAINTS_TEST | planning_environment_msgs::GetJointTrajectoryValidity::Request::COLLISION_TEST;

    if(check_plan_validity_client_.call(req,res))
    {
      ROS_DEBUG("Service call to check plan validity succeeded");
      if(res.error_code.val == res.error_code.SUCCESS)
        return true;
      else
      {
        feedback_.error_code = res.error_code;
        action_server_->publishFeedback(feedback_);        
        ROS_ERROR("Trajectory invalid");
        return false;
      }
    }
    else
    {
      ROS_ERROR("Service call to check trajectory validity failed on %s",check_plan_validity_client_.getService().c_str());
      return false;
    }
  }

  bool getRobotState(motion_planning_msgs::RobotState &robot_state)
  {
    planning_environment_msgs::GetRobotState::Request req;
    planning_environment_msgs::GetRobotState::Response res;
    if(get_state_client_.call(req,res))
    {
      robot_state = res.robot_state;
      return true;
    }
    else
    {
      ROS_ERROR("Service call to get robot state failed on %s",get_state_client_.getService().c_str());
      return false;
    }
  }

  bool isExecutionSafe()
  {
    planning_environment_msgs::GetJointTrajectoryValidity::Request req;
    planning_environment_msgs::GetJointTrajectoryValidity::Response res;
        
    req.trajectory = current_trajectory_;
    if(!getRobotState(req.robot_state))
      return false;
    req.start_index = 0;
    req.end_index = (int)req.trajectory.points.size()-1;
    req.flag = planning_environment_msgs::GetJointTrajectoryValidity::Request::PATH_CONSTRAINTS_TEST | planning_environment_msgs::GetJointTrajectoryValidity::Request::COLLISION_TEST;
    if(check_execution_safe_client_.call(req,res))
    {
      if(res.error_code.val == res.error_code.SUCCESS)
        return true;
      else
      {
        feedback_.error_code = res.error_code;
        action_server_->publishFeedback(feedback_);        
        return false;
      }
    }
    else
    {
      ROS_ERROR("Service call to check execution safety failed on %s",check_execution_safe_client_.getService().c_str());
      return false;
    }
  }


  bool isStateValidAtGoal(const sensor_msgs::JointState &joint_state)
  {
    planning_environment_msgs::GetStateValidity::Request req;
    planning_environment_msgs::GetStateValidity::Response res;
    req.robot_state.joint_state = joint_state;
    req.robot_state.joint_state.header.stamp = ros::Time::now();
    req.flag = planning_environment_msgs::GetJointTrajectoryValidity::Request::GOAL_CONSTRAINTS_TEST | planning_environment_msgs::GetJointTrajectoryValidity::Request::COLLISION_TEST;
    if(check_state_validity_client_.call(req,res))
    {
      if(res.error_code.val == res.error_code.SUCCESS)
        return true;
      else
      {
        feedback_.error_code = res.error_code;
        action_server_->publishFeedback(feedback_);        
        return false;
      }
    }
    else
    {
      ROS_ERROR("Service call to check goal validity failed %s",check_state_validity_client_.getService().c_str());
      return false;
    }
  }

  bool isStateValid(const sensor_msgs::JointState &joint_state, int flag=0)
  {
    planning_environment_msgs::GetStateValidity::Request req;
    planning_environment_msgs::GetStateValidity::Response res;
    req.robot_state.joint_state = joint_state;
    req.robot_state.joint_state.header.stamp = ros::Time::now();
    req.flag = planning_environment_msgs::GetJointTrajectoryValidity::Request::COLLISION_TEST | flag;
    if(check_state_validity_client_.call(req,res))
    {
      ROS_DEBUG("Service call to check state validity succeeded");
      if(res.error_code.val == res.error_code.SUCCESS)
        return true;
      else
      {
        feedback_.error_code = res.error_code;
        action_server_->publishFeedback(feedback_);        
        return false;
      }
    }
    else
    {
      ROS_ERROR("Service call to check state validity failed on %s",check_state_validity_client_.getService().c_str());
      return false;
    }
  }

  bool updateConstraintsPlanningMonitor(motion_planning_msgs::GetMotionPlan::Request &req_in)
  {
    planning_environment_msgs::SetConstraints::Request req;
    planning_environment_msgs::SetConstraints::Response res;
    req.goal_constraints = req_in.goal_constraints;
    req.path_constraints = req_in.path_constraints;
    if(!update_planning_monitor_client_.call(req,res))
    {
      ROS_ERROR("Service call to update goal and path constraints failed");
      return false;
    }
    req_in.goal_constraints = res.goal_constraints;
    req_in.path_constraints = res.path_constraints;

    if(isPoseGoal(original_request_))
    {
      req.goal_constraints = original_request_.goal_constraints;
      req.goal_constraints.joint_constraints.clear();
      if(!update_planning_monitor_client_.call(req,res))
      {
        ROS_ERROR("Service call to update goal and path constraints failed on %s",update_planning_monitor_client_.getService().c_str());
        return false;
      }        
    }
    if(res.error_code.val == res.error_code.SUCCESS)
      return true;
    else
    {
      feedback_.error_code = res.error_code;
      action_server_->publishFeedback(feedback_);              
      return false;
    }
  }



  bool doPrePlanningChecks(motion_planning_msgs::GetMotionPlan::Request &req,  motion_planning_msgs::GetMotionPlan::Response &res)
  {
    //checking current state for validity
    motion_planning_msgs::RobotState current_state;
    current_state.joint_state = state_monitor_.getJointState(group_joint_names_);
    //is state valid adds collision test by default
    if(!isStateValid(current_state.joint_state)){
      ROS_ERROR("Current state in collision.  Can't plan.");
      action_server_->setAborted();
      return false;
    }
    if(!processAndCheckGoal(req))
    {
      if(action_server_->isActive())
        action_server_->setAborted();
      return false;
    }
    // update the request and transform it to the correct frame using the planning monitor
    if(!updateConstraintsPlanningMonitor(req))
    {
      ROS_ERROR("Could not contact move arm monitor. Aborting");
      action_server_->setAborted();
      return false;
    }
    if(display_joint_goal_)
      displayJointGoal(req);
    return true;
  }

  bool createPlan(motion_planning_msgs::GetMotionPlan::Request &req,  
                  motion_planning_msgs::GetMotionPlan::Response &res,
                  const int &move_arm_options)
  {
    if (!isEnvironmentSafe())
    {
      ROS_WARN("Environment is not safe. Will not issue request for planning");
      return false;
    }        
    //        std::string plan_client_name = req.planner_id + "/" + planner_service_name_;
    std::string plan_client_name = planner_service_name_;
    ros::ServiceClient planning_client = root_handle_.serviceClient<motion_planning_msgs::GetMotionPlan>(plan_client_name);

    if(!getRobotState(req.start_state))
      return false;
    //        req.start_state.joint_state = state_monitor_.getJointState();		    
    ROS_DEBUG("Issued request for motion planning");		    
    // call the planner and decide whether to use the path
    if (planning_client.call(req, res))
    {
      if (res.trajectory.joint_trajectory.points.empty())
      {
        feedback_.error_code = res.error_code;
        action_server_->publishFeedback(feedback_);        
        ROS_WARN("Unable to plan path to desired goal");
        return false;
      }
      ROS_INFO("Motion planning succeeded");
        motion_planning_msgs::RobotState current_state;
        current_state.joint_state.header = res.trajectory.joint_trajectory.header;
	current_state.joint_state.position = res.trajectory.joint_trajectory.points.back().positions;
	current_state.joint_state.name = res.trajectory.joint_trajectory.joint_names;
        if(!isStateValidAtGoal(current_state.joint_state))
        {
          if(move_arm_options & move_arm_msgs::MoveArmGoal::ACCEPT_PARTIAL_PLANS)
          {
            ROS_WARN("Returned path from planner does not go all the way to goal");
            return true;
          }
          else
          {
            feedback_.error_code = res.error_code;
            action_server_->publishFeedback(feedback_);        
            ROS_ERROR("Returned path from planner does not go all the way to goal");
            return false;
          }
        }
      return true;
    }
    else
    {
      ROS_ERROR("Motion planning service failed on %s",planning_client.getService().c_str());
      return false;
    }
  }

  bool stopTrajectory()
  {
    bool result = true;
    // we are already executing the path; we need to stop it
    arm_control_msgs::TrajectoryCancel::Request  send_traj_cancel_req;
    arm_control_msgs::TrajectoryCancel::Response send_traj_cancel_res;
    send_traj_cancel_req.trajectory_id = current_trajectory_id_;
    if (trajectory_cancel_client_.call(send_traj_cancel_req, send_traj_cancel_res))
    {
      ROS_DEBUG("Stopped trajectory %d", current_trajectory_id_);
    }
    else
    {
      feedback_.error_code.val = feedback_.error_code.TRAJECTORY_CONTROLLER_FAILED;
      action_server_->publishFeedback(feedback_);        
      ROS_ERROR("Unable to cancel trajectory %d. Continuing...", current_trajectory_id_);
      result = false;
    }
    current_trajectory_id_ = -1;
    return result;
  }

  bool sendTrajectory(trajectory_msgs::JointTrajectory current_trajectory)
  {
    arm_control_msgs::TrajectoryStart::Request  send_traj_start_req;
    arm_control_msgs::TrajectoryStart::Response send_traj_start_res;
			
    send_traj_start_req.trajectory = current_trajectory;
    send_traj_start_req.request_timing = 0;			
    if (trajectory_start_client_.call(send_traj_start_req, send_traj_start_res))
    {
      current_trajectory_id_ = send_traj_start_res.trajectory_id;
      if (current_trajectory_id_ < 0)
      {
        feedback_.error_code.val = feedback_.error_code.TRAJECTORY_CONTROLLER_FAILED;
        action_server_->publishFeedback(feedback_);        
        ROS_ERROR("Invalid trajectory id: %d", current_trajectory_id_);
        return false;
      }
      ROS_INFO("Sent trajectory %d to controller", current_trajectory_id_);
      return true;
    }
    else
    {
      ROS_ERROR("Unable to connect to trajectory controller");
      return false;
    }       
  }
 
  bool trajectoryControllerDone()
  {
    // monitor controller execution by calling trajectory query		    
    arm_control_msgs::TrajectoryQuery::Request  send_traj_query_req;
    arm_control_msgs::TrajectoryQuery::Response send_traj_query_res;
    send_traj_query_req.trajectory_id = current_trajectory_id_;
    if (trajectory_query_client_.call(send_traj_query_req, send_traj_query_res))
    {
      // we are done; exit with success
      if (send_traj_query_res.status == arm_control_msgs::TrajectoryQuery::Response::STATE_DONE)
      {
        ROS_INFO("Completed trajectory %d", current_trajectory_id_);
        return true;
      }
      // something bad happened in the execution
      if (!(send_traj_query_res.status == arm_control_msgs::TrajectoryQuery::Response::STATE_ACTIVE ||
            send_traj_query_res.status == arm_control_msgs::TrajectoryQuery::Response::STATE_QUEUED))
      {
        feedback_.error_code.val = feedback_.error_code.TRAJECTORY_CONTROLLER_FAILED;
        action_server_->publishFeedback(feedback_);        
        ROS_ERROR("Unable to finish executing trajectory %d: Trajectory status is: %s", current_trajectory_id_, (arm_control_msgs::statusToString((int)send_traj_query_res.status)).c_str());
        resetStateMachine();
        action_server_->setAborted();
        return false;
      }
    }
    else
    {
      feedback_.error_code.val = feedback_.error_code.TRAJECTORY_CONTROLLER_FAILED;
      action_server_->publishFeedback(feedback_);        
      ROS_ERROR("Unable to query trajectory %d", current_trajectory_id_);
      resetStateMachine();
      action_server_->setAborted();
      return false;
    }
    return false;
  }

  bool executeCycle(motion_planning_msgs::GetMotionPlan::Request &req)
  {
    motion_planning_msgs::GetMotionPlan::Response res;

    switch(state_)
    {
    case PLANNING:
      {
        feedback_.state = "planning";
        feedback_.time_to_completion = ros::Duration(req.allowed_planning_time);
        action_server_->publishFeedback(feedback_);

        if(!doPrePlanningChecks(req,res))
          return true;

        motion_planning_msgs::RobotState current_state;
        current_state.joint_state = state_monitor_.getJointState(group_joint_names_);
        if(isStateValidAtGoal(current_state.joint_state))
        {
          resetStateMachine();
          action_server_->setSucceeded();
          ROS_INFO("Reached goal");
          return true;
        }

        if(createPlan(req,res,move_arm_options_))
        {
          ROS_DEBUG("createPlan succeeded");
          if(!isTrajectoryValid(res.trajectory.joint_trajectory))
          {
            ROS_ERROR("Trajectory returned by the planner is in collision with a part of the environment");
            ROS_ERROR("Move arm will abort this goal.");
            resetStateMachine();
            action_server_->setAborted();
            return true;
          }
          else{
            ROS_DEBUG("Trajectory validity check was successful");
          }
          last_valid_plan_time_ = ros::Time::now();
          current_trajectory_ = res.trajectory.joint_trajectory;
          //          printTrajectory(current_trajectory_);
          if(visualize_plan_)
          {
            state_ = VISUALIZE_PLAN;                
            ROS_INFO("Done planning. Transitioning to display path");
          }
          else
          {
            state_ = START_CONTROL;
            ROS_INFO("Done planning. Transitioning to control");
          }
        }
        else if(action_server_->isActive())
        {
          num_planning_attempts_++;
          if(num_planning_attempts_ > req.num_planning_attempts)
          {
            resetStateMachine();
            action_server_->setAborted();
            return true;
          }
        }
        else
        {
          ROS_ERROR("create plan failed");
        }
        break;
      }
    case VISUALIZE_PLAN:
      {            
        feedback_.state = "visualizing plan";
        feedback_.time_to_completion = ros::Duration(req.allowed_planning_time);
        action_server_->publishFeedback(feedback_);
        motion_planning_msgs::DisplayTrajectory d_path;
        d_path.model_id = req.group_name;
        d_path.trajectory.joint_trajectory = current_trajectory_;
        if(!getRobotState(d_path.robot_state))
        {
          ROS_ERROR("Could not get robot state");
          state_ = START_CONTROL;
        }
        else
        {
          display_path_publisher_.publish(d_path);
          done_visualizer_ = false;
          ROS_INFO("Publishing plan for display");
          if(wait_for_visualizer_)
          {
            if(display_path_subscriber_.getNumPublishers() <= 0)
            {
              ROS_INFO("Will not wait for visualizer callback since planning display on rviz is disabled.");
              state_ = START_CONTROL;
            }              
            else
            {
              ROS_INFO("Will wait for visualizer callback on topic %s before executing control.",display_path_subscriber_.getTopic().c_str());
              state_ = WAIT_FOR_VISUALIZER;
              visualizer_start_time_ = ros::Time::now();
            }
          }
          else
          {
            state_ = START_CONTROL;
          }
        }
        break;
      }
    case WAIT_FOR_VISUALIZER:
      {
        if(display_path_subscriber_.getNumPublishers() <= 0)
        {
          ROS_INFO("Will not wait for visualizer callback since planning display on rviz is disabled.");
          state_ = START_CONTROL;
        }              
        if(done_visualizer_ || (ros::Time::now() - visualizer_start_time_) > ros::Duration(wait_for_visualizer_timeout_))
        {
          state_ = START_CONTROL;
        }
        break;
      }
    case START_CONTROL:
      {
        feedback_.state = "start_control";
        feedback_.time_to_completion = ros::Duration(1.0/move_arm_frequency_);
        action_server_->publishFeedback(feedback_);
        ROS_DEBUG("Filtering Trajectory");
        trajectory_msgs::JointTrajectory filtered_trajectory;
        if(filterTrajectory(current_trajectory_, filtered_trajectory))
        {
          current_trajectory_ = filtered_trajectory;
        }


        ROS_DEBUG("Sending trajectory");
        if(sendTrajectory(current_trajectory_))
        {
          state_ = MONITOR;
        }
        else
        {
          resetStateMachine();
          action_server_->setAborted();
          return true;              
        }
        break;
      }	
    case MONITOR:
      {
        feedback_.state = "monitor";
        feedback_.time_to_completion = current_trajectory_.points.back().time_from_start;
        action_server_->publishFeedback(feedback_);
        ROS_DEBUG("Start to monitor");
        if(trajectoryControllerDone())
        {
          motion_planning_msgs::RobotState current_state;
          current_state.joint_state = state_monitor_.getJointState(group_joint_names_);
          if(isStateValidAtGoal(current_state.joint_state))
          {
            resetStateMachine();
            action_server_->setSucceeded();
            ROS_INFO("Reached goal");
            return true;
          }
          else
          {
            ROS_INFO("Not at goal yet, replanning");
            if(isPoseGoal(original_request_))
            {
              //              ROS_INFO("Current joint state");
              //              printJointState(current_state.joint_state);
              //              ROS_INFO("Desired joint state");
              //              printJointState(motion_planning_msgs::jointConstraintsToJointState(req.goal_constraints.joint_constraints));
            }
            state_ = PLANNING;
            break;
          }
        }            
        if(monitor_execution_ && action_server_->isActive())
        {
	  ROS_DEBUG("Monitoring trajectory");
          if(!isExecutionSafe())
          {
            ROS_INFO("Stopping trajectory since it is unsafe");
            stopTrajectory();
            state_ = PLANNING;
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

  void resetStateMachine()
  {
    pose_goal_ = false;
    current_trajectory_.points.clear();
    current_trajectory_.joint_names.clear();
    state_ = PLANNING;
  }

  void moveArmGoalToPlannerRequest(const move_arm_msgs::MoveArmGoalConstPtr& goal, motion_planning_msgs::GetMotionPlan::Request &req)
  {
    //    req.workspace_parameters.contacts = goal->contacts;
    //TODO -- figure out what these should be
    req.workspace_parameters.workspace_region_pose.header.stamp = ros::Time::now();
    req.workspace_parameters.ordered_collision_operations = goal->ordered_collision_operations;
    req.workspace_parameters.allowed_contacts = goal->allowed_contacts;
    req.group_name = goal->group_name;
    req.planner_id = goal->planner_id;
	    
    // forward the goal & path constraints
    req.goal_constraints = goal->goal_constraints;
    req.path_constraints = goal->path_constraints;
	    	    
    // do not spend more than this amount of time
    req.allowed_planning_time = ros::Duration(goal->allowed_planning_time);
    req.num_planning_attempts = goal->num_planning_attempts;
    move_arm_options_ = goal->options;
    visualizeAllowedContactRegions(req.workspace_parameters.allowed_contacts);
  }
	
  void execute(const move_arm_msgs::MoveArmGoalConstPtr& goal)
  {
    motion_planning_msgs::GetMotionPlan::Request req;	    
    moveArmGoalToPlannerRequest(goal,req);	    
    original_request_ = req;
    planner_service_name_ = goal->planner_service_name;
    ros::Rate move_arm_rate(move_arm_frequency_);

    while(node_handle_.ok())
    {	    	    
      if (action_server_->isPreemptRequested())
      {
        if(action_server_->isNewGoalAvailable())
        {
          moveArmGoalToPlannerRequest((action_server_->acceptNewGoal()),req);
          original_request_ = req;
          stopTrajectory();
          state_ = PLANNING;
        }
        else               //if we've been preempted explicitly we need to shut things down
        {
          ROS_INFO("The move arm action was preempted by the action client. Preempting this goal.");
          stopTrajectory();
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
        return;
      }

      ros::WallDuration t_diff = ros::WallTime::now() - start;
      ROS_DEBUG("Full control cycle time: %.9f\n", t_diff.toSec());

      move_arm_rate.sleep();
    }	    
    //if the node is killed then we'll abort and return
    ROS_INFO("Node was killed, aborting");
    action_server_->setAborted();
  }
	
  void fillTrajectoryMsg(const trajectory_msgs::JointTrajectory &path, trajectory_msgs::JointTrajectory &traj)
  {
    traj = path;
    if(path.points.empty())
    {
      ROS_WARN("No points in trajectory");
      return;
    }

    // get the current state
    double d = 0.0;
    sensor_msgs::JointState current = state_monitor_.getJointState(traj.joint_names);
    for (unsigned int i = 0 ; i < current.position.size() ; ++i)
    {
      double dif = current.position[i] - path.points[0].positions[i];
      d += dif * dif;
    }
    d = sqrt(d);
	    
    // decide whether we place the current state in front of the path
    int include_first = (d > 0.1) ? 1 : 0;
    double offset = 0.0;
    traj.points.resize(path.points.size() + include_first);

    if (include_first)
    {
      traj.points[0].positions = motion_planning_msgs::jointStateToJointTrajectoryPoint(current).positions;
      traj.points[0].time_from_start = ros::Duration(0.0);
      offset = 0.3 + d;
    }
	    
    for (unsigned int i = 0 ; i < path.points.size() ; ++i)
    {
      traj.points[i+include_first].time_from_start = path.points[i].time_from_start;
      traj.points[i+include_first].positions = path.points[i].positions;
    }
    traj.header.stamp = ros::Time::now();
  }
	
  void printTrajectory(const trajectory_msgs::JointTrajectory &trajectory)
  {
    for (unsigned int i = 0 ; i < trajectory.points.size() ; ++i)
    {
      std::stringstream ss;
      for (unsigned int j = 0 ; j < trajectory.points[i].positions.size() ; ++j)
        ss << trajectory.points[i].positions[j] << " ";
      ROS_DEBUG("%s", ss.str().c_str());
    }
  }
		
  bool computeIK(const geometry_msgs::PoseStamped &pose_stamped_msg,  
                 const std::string &link_name, 
                 sensor_msgs::JointState &solution, 
                 motion_planning_msgs::ArmNavigationErrorCodes &error_code, 
                 motion_planning_msgs::OrderedCollisionOperations &ordered_collision_operations)
  {
    // define the service messages
    kinematics_msgs::GetCollisionFreePositionIK::Request request;
    kinematics_msgs::GetCollisionFreePositionIK::Response response;
	    
    request.ik_request.pose_stamped = pose_stamped_msg;
    request.ik_request.ik_seed_state.joint_state.name = group_joint_names_;	    
    request.ik_request.ik_link_name = link_name;
    request.ik_request.ik_seed_state.joint_state.position = state_monitor_.getJointState(group_joint_names_).position;	    
    request.timeout = ros::Duration(2.0);
    request.ordered_collision_operations = ordered_collision_operations;
    if (ik_client_.call(request, response))
    {
      error_code = response.error_code;
      if(response.error_code.val != response.error_code.SUCCESS)
      {
        ROS_ERROR("IK Solution not found, IK returned with error_code: %d",response.error_code.val);
        error_code = response.error_code;
        return false;
      }         
      solution = response.solution.joint_state;
      if (solution.position.size() != request.ik_request.ik_seed_state.joint_state.name.size())
      {
        ROS_ERROR("Incorrect number of elements in IK output.");
        error_code = response.error_code;
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
    // define the service messages
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
      ROS_DEBUG("Position : (%f,%f,%f)",response.pose_stamped[0].pose.position.x,response.pose_stamped[0].pose.position.y,response.pose_stamped[0].pose.position.z);
      ROS_DEBUG("Rotation : (%f,%f,%f,%f)",response.pose_stamped[0].pose.orientation.x,response.pose_stamped[0].pose.orientation.y,response.pose_stamped[0].pose.orientation.z,response.pose_stamped[0].pose.orientation.w);
      ROS_DEBUG(" ");
    }
    else
    {
      ROS_ERROR("FK service failed");
      return false;
    }	    
    return true;
  }

	
  void displayPathCallback(const std_msgs::Bool::ConstPtr& msg)
  {
    done_visualizer_ = msg->data;
  }


  void displayJointGoal(motion_planning_msgs::GetMotionPlan::Request &req)
  {
    if(!isJointGoal(req))
    {
      ROS_WARN("Only joint goals can be displayed");
      return;
    }
    ROS_DEBUG("Displaying joint goal");
    motion_planning_msgs::DisplayTrajectory d_path;
    d_path.model_id = req.group_name;
    d_path.trajectory.joint_trajectory = motion_planning_msgs::jointConstraintsToJointTrajectory(req.goal_constraints.joint_constraints);
    if(!getRobotState(d_path.robot_state))
    {
      ROS_ERROR("Could not get robot state");
    }
    else
    {
      display_joint_goal_publisher_.publish(d_path);
      ROS_INFO("Displaying move arm joint goal.");
    }
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
      mk.markers[i].ns = allowed_contacts[i].name;
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
        mk.markers[i].color.a = 0.3;
        mk.markers[i].color.r = 0.04;
        mk.markers[i].color.g = 1.0;
        mk.markers[i].color.b = 0.04;
      }  
      //mk.markers[i].lifetime = ros::Duration(30.0);  
    }
    allowed_contact_regions_publisher_.publish(mk);
  }


private:

  std::string group_;

  ros::ServiceClient get_joints_in_group_client_, get_state_client_;
  ros::ServiceClient ik_client_, check_state_at_goal_client_, check_plan_validity_client_;
  ros::ServiceClient check_env_safe_client_, check_execution_safe_client_, update_planning_monitor_client_, check_state_validity_client_;
  ros::ServiceClient trajectory_start_client_,trajectory_cancel_client_,trajectory_query_client_;	
  ros::NodeHandle node_handle_, root_handle_;
  boost::shared_ptr<actionlib::SimpleActionServer<move_arm_msgs::MoveArmAction> > action_server_;	

  tf::TransformListener *tf_;
  bool perform_ik_, monitor_execution_;
  MoveArmState state_;
  double move_arm_frequency_;      	
  trajectory_msgs::JointTrajectory current_trajectory_;

  int current_trajectory_id_;
  int num_planning_attempts_;

  ros::Time last_valid_plan_time_;

  planning_environment::JointStateMonitor state_monitor_;
  std::vector<std::string>               group_joint_names_;

  std::string planner_service_name_;
  move_arm_msgs::MoveArmFeedback feedback_;

  int move_arm_options_;
  bool pose_goal_;
  motion_planning_msgs::GetMotionPlan::Request original_request_;

  bool visualize_plan_, done_visualizer_, wait_for_visualizer_;
  double wait_for_visualizer_timeout_;
  ros::Time visualizer_start_time_;
  ros::Subscriber display_path_subscriber_;
  ros::Publisher display_path_publisher_;

  bool display_joint_goal_;
  ros::Publisher display_joint_goal_publisher_;

  ros::ServiceClient fk_client_;
  ros::Publisher allowed_contact_regions_publisher_;
  ros::ServiceClient filter_trajectory_client_;
};
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_arm");
  ROS_INFO("Starting action...");
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
  ROS_INFO("Move arm simple action started");
  ros::spin();
    
  return 0;
}
