/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

#include <move_arm/move_arm_action.h>

namespace move_arm
{

MoveArm::MoveArm() :  private_handle_("~"),planning_visualizer_(DISPLAY_PATH_TOPIC,DISPLAY_MARKER_TOPIC)
{
  private_handle_.param<bool>("publish_stats",publish_stats_, true);
  private_handle_.param<double>("ik_allowed_time",ik_allowed_time_, 2.0);
  private_handle_.param<double>("move_arm_frequency",move_arm_frequency_, 50.0);
  private_handle_.param<double>("trajectory_filter_allowed_time",trajectory_filter_allowed_time_, 2.0);
  collision_models_interface_ = new planning_environment::CollisionModelsInterface("robot_description");

  planning_scene_state_ = NULL;
  num_planning_attempts_ = 0;
  state_ = PLANNING;

  ros::service::waitForService(TRAJECTORY_FILTER);
  ros::service::waitForService(GET_STATE_SERVICE_NAME);
  ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);

  filter_trajectory_client_ = root_handle_.serviceClient<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>(TRAJECTORY_FILTER);      
  get_state_client_ = root_handle_.serviceClient<arm_navigation_msgs::GetRobotState>("/environment_server/get_robot_state");      
  set_planning_scene_diff_client_ = root_handle_.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);
  action_server_.reset(new actionlib::SimpleActionServer<arm_navigation_msgs::MoveArmAction>(root_handle_, "move_arms", boost::bind(&MoveArm::execute, this, _1), false));
  action_server_->start();

  stats_publisher_ = private_handle_.advertise<arm_navigation_msgs::MoveArmStatistics>("statistics",1,true);
}	

MoveArm::~MoveArm()
{
  revertPlanningScene();
  delete collision_models_interface_;
}

bool MoveArm::initialize()
{
  return loadGroups();
}

bool MoveArm::loadGroups()
{
  XmlRpc::XmlRpcValue group_list;
  std::vector<std::string> group_names;

  if(!private_handle_.getParam("/groups", group_list))
  {
    ROS_ERROR("Could not find groups on param server");
    return false;
  }
  if(group_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Group list should be of XmlRpc Array type");
    return false;
  } 
  for (int32_t i = 0; i < group_list.size(); ++i) 
  {
    if(group_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_ERROR("Group names should be strings");
      return false;
    }
    group_names.push_back(static_cast<std::string>(group_list[i]));
    ROS_DEBUG("Adding group: %s",group_names.back().c_str());
    group_map_[group_names.back()].reset(new move_arm::MoveGroup(group_names.back(),collision_models_interface_));
    if(!group_map_[group_names.back()]->initialize())
    {
      ROS_ERROR("Could not initialize group %s",group_names.back().c_str());
      return false;
    }
  }
  return true;
}

bool MoveArm::findGroup(std::string &group_name, boost::shared_ptr<move_arm::MoveGroup> &group)
{
  if(group_map_.find(group_name) != group_map_.end()) 
  {
    group = group_map_[group_name];
    return true;
  }
  else
    return false;
}

bool MoveArm::filterTrajectory(const trajectory_msgs::JointTrajectory &trajectory_in, 
                               trajectory_msgs::JointTrajectory &trajectory_out,
                               arm_navigation_msgs::Constraints &goal_constraints,
                               arm_navigation_msgs::Constraints &path_constraints,
                               planning_models::KinematicState *kinematic_state,
                               arm_navigation_msgs::ArmNavigationErrorCodes &error_code)
{
  arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Request  request;
  arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Response response;
  fillTrajectoryMsg(trajectory_in, request.trajectory);

  if(trajectory_filter_allowed_time_ == 0.0)
  {
    trajectory_out = request.trajectory;
    return true;
  }
  resetToStartState(kinematic_state);
  planning_environment::convertKinematicStateToRobotState(*kinematic_state,
                                                          ros::Time::now(),
                                                          collision_models_interface_->getWorldFrameId(),
                                                          request.start_state);
  request.group_name = current_group_->getPhysicalGroupName();
  request.path_constraints = path_constraints;
  request.goal_constraints = goal_constraints;
  request.allowed_time = ros::Duration(trajectory_filter_allowed_time_);
  ros::Time smoothing_time = ros::Time::now();
  if(filter_trajectory_client_.call(request,response))
  {
    move_arm_stats_.trajectory_duration = (response.trajectory.points.back().time_from_start-response.trajectory.points.front().time_from_start).toSec();
    move_arm_stats_.smoothing_time = (ros::Time::now()-smoothing_time).toSec();
    trajectory_out = response.trajectory;
    error_code = response.error_code;
    return true;
  }
  else
  {
    error_code = response.error_code;
    ROS_ERROR("Service call to filter trajectory failed.");
    return false;
  }
}

bool MoveArm::getRobotState(planning_models::KinematicState* state)
{
  arm_navigation_msgs::GetRobotState::Request request;
  arm_navigation_msgs::GetRobotState::Response response;
  if(get_state_client_.call(request,response))
  {
    planning_environment::setRobotStateAndComputeTransforms(response.robot_state, *state);
  }
  else
  {
    ROS_ERROR("Service call to get robot state failed on %s",get_state_client_.getService().c_str());
    return false;
  }
  return true;
}

bool MoveArm::moveArmGoalToPlannerRequest(const arm_navigation_msgs::MoveArmGoalConstPtr& goal, 
                                          arm_navigation_msgs::GetMotionPlan::Request &request)
{
  request.motion_plan_request.workspace_parameters.workspace_region_pose.header.stamp = ros::Time::now();
  request.motion_plan_request = goal->motion_plan_request;

  disable_ik_           = goal->disable_ik;
  allowed_planning_time_ = goal->motion_plan_request.allowed_planning_time.toSec();
  planner_service_name_ = goal->planner_service_name;
  planning_visualizer_.visualize(goal->planning_scene_diff.allowed_contacts);

  if(!getAndSetPlanningScene(goal->planning_scene_diff, goal->operations)) 
  {
    ROS_ERROR("Problem setting planning scene");
    move_arm_action_result_.error_code.val = move_arm_action_result_.error_code.INCOMPLETE_ROBOT_STATE;
    action_server_->setAborted(move_arm_action_result_);
    return false;
  }

  collision_models_interface_->convertConstraintsGivenNewWorldTransform(*planning_scene_state_,
                                                              request.motion_plan_request.goal_constraints);
  collision_models_interface_->convertConstraintsGivenNewWorldTransform(*planning_scene_state_,
                                                              request.motion_plan_request.path_constraints);
  //overwriting start state - move arm only deals with current state
  planning_environment::convertKinematicStateToRobotState(*planning_scene_state_,
                                                          ros::Time::now(),
                                                          collision_models_interface_->getWorldFrameId(),
                                                          request.motion_plan_request.start_state);
  original_goal_constraints_ = request.motion_plan_request.goal_constraints;
  return true;
}

void MoveArm::setAborted(arm_navigation_msgs::GetMotionPlan::Response &response)
{
  setAborted(response.error_code);
}

bool MoveArm::setAborted(arm_navigation_msgs::ArmNavigationErrorCodes &error_code)
{
  resetStateMachine();
  resetToStartState(planning_scene_state_);
  move_arm_action_result_.error_code = error_code;
  action_server_->setAborted(move_arm_action_result_);
  return true;
}

bool MoveArm::visualizeCollisions(const planning_models::KinematicState *kinematic_state)
{
  visualization_msgs::MarkerArray array;
  std_msgs::ColorRGBA point_color;
  point_color.a = 1.0;
  point_color.r = 1.0;
  point_color.g = .8;
  point_color.b = 0.04;
  collision_models_interface_->getAllCollisionPointMarkers(*kinematic_state,array,point_color,ros::Duration(0.0)); 
  planning_visualizer_.visualize(array);
  return true;
}


bool MoveArm::createPlan(arm_navigation_msgs::GetMotionPlan::Request &request,  
                         arm_navigation_msgs::GetMotionPlan::Response &response)
{
  ros::Time planning_time = ros::Time::now();
  while(!ros::service::waitForService(planner_service_name_, ros::Duration(1.0))) 
  {
    ROS_INFO_STREAM("Waiting for requested service " << planner_service_name_);
  }
  ros::ServiceClient planning_client = root_handle_.serviceClient<arm_navigation_msgs::GetMotionPlan>(planner_service_name_);
  move_arm_stats_.planner_service_name = planner_service_name_;
  ROS_DEBUG("Issuing request for motion plan");		    
  // call the planner and decide whether to use the path
  if (planning_client.call(request, response))
  {
    move_arm_stats_.planning_time = (ros::Time::now()-planning_time).toSec();
    if (response.trajectory.joint_trajectory.points.empty())
    {
      ROS_WARN("Motion planner was unable to plan a path to goal");
      return false;
    }
    ROS_DEBUG("Motion planning succeeded");
    return true;
  }
  else
  {
    ROS_ERROR("Motion planning service failed on %s",planning_client.getService().c_str());
    return false;
  }
}

void MoveArm::fillTrajectoryMsg(const trajectory_msgs::JointTrajectory &trajectory_in, 
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
    boost::shared_ptr<const urdf::Joint> joint = collision_models_interface_->getParsedDescription()->getJoint(name);
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
    trajectory_out.points[0].positions = arm_navigation_msgs::jointStateToJointTrajectoryPoint(current).positions;
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

void MoveArm::resetStateMachine()
{
  num_planning_attempts_ = 0;
  current_trajectory_.points.clear();
  current_trajectory_.joint_names.clear();
  state_ = PLANNING;    
}

bool MoveArm::jointTrajectoryValid(trajectory_msgs::JointTrajectory &joint_trajectory,
                                   arm_navigation_msgs::Constraints &goal_constraints,
                                   arm_navigation_msgs::Constraints &path_constraints,
                                   planning_models::KinematicState *kinematic_state,
                                   arm_navigation_msgs::ArmNavigationErrorCodes &error_code)
{
  std::vector<arm_navigation_msgs::ArmNavigationErrorCodes> traj_error_codes;
  resetToStartState(planning_scene_state_);
  if(!collision_models_interface_->isJointTrajectoryValid(*planning_scene_state_,
                                                          joint_trajectory, 
                                                          goal_constraints,
                                                          path_constraints,
                                                          error_code,
                                                          traj_error_codes,
                                                          true))
  {
    if(error_code.val == error_code.COLLISION_CONSTRAINTS_VIOLATED) 
    {
      ROS_WARN("Trajectory collides");
    } 
    else if (error_code.val == error_code.PATH_CONSTRAINTS_VIOLATED) 
    {
      ROS_WARN("Trajectory violates path constraints");
    } 
    else if (error_code.val == error_code.JOINT_LIMITS_VIOLATED) 
    {
      ROS_WARN("Trajectory violates joint limits");
    } 
    else if (error_code.val == error_code.GOAL_CONSTRAINTS_VIOLATED) 
    {
      ROS_WARN("Trajectory doesn't reach goal");
    }
  }
  else
  {
    ROS_DEBUG("Trajectory validity check was successful");
    return true;
  }
  return false;
}

bool MoveArm::executeCycle(arm_navigation_msgs::GetMotionPlan::Request &request)
{
  arm_navigation_msgs::GetMotionPlan::Response response;
  arm_navigation_msgs::ArmNavigationErrorCodes error_code;
    
  switch(state_)
  {
  case PLANNING:
    {
      move_arm_action_feedback_.state = "planning";
      move_arm_action_feedback_.time_to_completion = ros::Duration(request.motion_plan_request.allowed_planning_time);
      action_server_->publishFeedback(move_arm_action_feedback_);

      if(!disable_ik_)
        if(!current_group_->runIKOnRequest(request,response,planning_scene_state_))
          return true;

      if(!current_group_->checkRequest(request,response,planning_scene_state_))
        return true;

      planning_visualizer_.visualize(request,planning_scene_state_);
        
      if(createPlan(request,response))
      {
        ROS_DEBUG("createPlan succeeded");
        if(!jointTrajectoryValid(response.trajectory.joint_trajectory,original_goal_constraints_,request.motion_plan_request.path_constraints,planning_scene_state_,response.error_code))
        {
          num_planning_attempts_++;
          if(num_planning_attempts_ > request.motion_plan_request.num_planning_attempts)
          {
            ROS_INFO_STREAM("Aborting because we're out of planning attempts");
            response.error_code.val = response.error_code.PLANNING_FAILED;
            setAborted(response.error_code);
            return true;
          }
          ROS_INFO("Attempt %d of %d: Joint trajectory invalid. Will try to plan again.",num_planning_attempts_-1,request.motion_plan_request.num_planning_attempts);
        }	    
        else
        {
          current_trajectory_ = response.trajectory.joint_trajectory;
          planning_visualizer_.visualize(current_trajectory_,planning_scene_state_);
          state_ = START_CONTROL;
          ROS_DEBUG("Done planning. Transitioning to filtering.");
        }
      }
      else if(action_server_->isActive())
      {
        num_planning_attempts_++;
        error_code.val = error_code.PLANNING_FAILED;
        if(num_planning_attempts_ > request.motion_plan_request.num_planning_attempts)
        {
          ROS_INFO_STREAM("Setting aborted because we're out of planning attempts");
          setAborted(error_code);
          return true;
        }
        ROS_INFO("Attempt %d of %d: Joint trajectory invalid. Will try to plan again.",num_planning_attempts_-1,request.motion_plan_request.num_planning_attempts);
      }
      else
      {
        ROS_ERROR("Motion planner could not find plan");
        return false;
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
      if(filterTrajectory(current_trajectory_,filtered_trajectory,original_goal_constraints_,request.motion_plan_request.path_constraints,planning_scene_state_,error_code))
      {
        ROS_DEBUG("Checking filtered trajectory");
        if(!jointTrajectoryValid(filtered_trajectory,original_goal_constraints_,request.motion_plan_request.path_constraints,planning_scene_state_,error_code))
        {
          ROS_ERROR("Move arm will abort this goal.  Will replan");
          state_ = PLANNING;
          num_planning_attempts_++;	    
          if(num_planning_attempts_ > request.motion_plan_request.num_planning_attempts)
          {
            ROS_INFO_STREAM("Setting aborted because we're out of planning attempts");
            error_code.val = error_code.PLANNING_FAILED;
            setAborted(response.error_code);
            return true;
          }
          break;
        }
        else
        {
          ROS_DEBUG("Trajectory validity check was successful");
        }
        current_trajectory_ = filtered_trajectory;
      } 
      else 
      {
        ROS_INFO_STREAM("Setting aborted because trajectory filter call failed");
        setAborted(error_code);
        return true;              
      }
      ROS_DEBUG("Sending trajectory");
      move_arm_stats_.time_to_execution = (ros::Time::now() - ros::Time(move_arm_stats_.time_to_execution)).toSec();

      if(current_group_->sendTrajectory(current_trajectory_))
        state_ = MONITOR;
      else
      {
        ROS_INFO("Setting aborted because we couldn't send the trajectory");
        error_code.val = error_code.TRAJECTORY_CONTROLLER_FAILED;
        setAborted(error_code);
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
      arm_navigation_msgs::ArmNavigationErrorCodes controller_error_code;
      if(current_group_->isControllerDone(controller_error_code))
      {
        move_arm_stats_.time_to_result = (ros::Time::now()-ros::Time(move_arm_stats_.time_to_result)).toSec();
        arm_navigation_msgs::RobotState empty_state;
        arm_navigation_msgs::ArmNavigationErrorCodes state_error_code;
        getRobotState(planning_scene_state_);
      
        if(current_group_->checkState(original_goal_constraints_,
                                      request.motion_plan_request.path_constraints,
                                      planning_scene_state_,
                                      state_error_code))
        {
          move_arm_action_result_.error_code.val = move_arm_action_result_.error_code.SUCCESS;
          resetStateMachine();
          action_server_->setSucceeded(move_arm_action_result_);
          if(controller_error_code.val == controller_error_code.TRAJECTORY_CONTROLLER_FAILED) 
          {
            ROS_INFO("Trajectory controller failed but we seem to be at goal");
          } 
          else 
          {
            ROS_DEBUG("Reached goal");
          }
          return true;
        }
        else
        {
          if(state_error_code.val == state_error_code.COLLISION_CONSTRAINTS_VIOLATED) 
          {
            move_arm_action_result_.error_code.val = state_error_code.START_STATE_IN_COLLISION;
            ROS_WARN("Though trajectory is done current state is in collision");
          } 
          else if (state_error_code.val == state_error_code.PATH_CONSTRAINTS_VIOLATED) 
          {
            ROS_WARN("Though trajectory is done current state violates path constraints");
          } 
          else if (state_error_code.val == state_error_code.JOINT_LIMITS_VIOLATED) 
          {
            ROS_WARN("Though trajectory is done current state violates joint limits");
          } 
          else if(state_error_code.val == state_error_code.GOAL_CONSTRAINTS_VIOLATED) 
          {
            ROS_WARN("Though trajectory is done current state does not seem to be at goal");
          }
          setAborted(state_error_code);
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

void MoveArm::resetStats()
{
  move_arm_stats_.time_to_execution = ros::Time::now().toSec();
  move_arm_stats_.time_to_result = ros::Time::now().toSec();
}

void MoveArm::resetActionResult()
{
  move_arm_action_result_.contacts.clear();
  move_arm_action_result_.error_code.val = 0;
}

void MoveArm::execute(const arm_navigation_msgs::MoveArmGoalConstPtr& goal)
{
  arm_navigation_msgs::GetMotionPlan::Request request;	    
  if(!moveArmGoalToPlannerRequest(goal,request))
    return;
  ros::Rate move_arm_rate(move_arm_frequency_);
  resetActionResult();
  resetStats();
  if(!findGroup(request.motion_plan_request.group_name,current_group_))
  {
    ROS_ERROR("Could not find group %s for move arm to act on",request.motion_plan_request.group_name.c_str());
    return;
  }

  while(private_handle_.ok())
  {	    	    
    if (action_server_->isPreemptRequested())
    {
      revertPlanningScene();
      move_arm_stats_.preempted = true;
      if(publish_stats_)
        publishStats();
      resetStats();
      if(action_server_->isNewGoalAvailable())
      {
        resetActionResult();
        const arm_navigation_msgs::MoveArmGoalConstPtr& new_goal = action_server_->acceptNewGoal();
        ROS_DEBUG("Received new goal, will preempt previous goal");
        if(!moveArmGoalToPlannerRequest(new_goal,request))
          return;
        state_ = PLANNING;
      }
      else               //if we've been preempted explicitly we need to shut things down
      {
        ROS_DEBUG("The move arm action was preempted by the action client. Preempting this goal.");
        revertPlanningScene();
        resetStateMachine();
        action_server_->setPreempted();
        return;
      }
    }

    //for timing that gives real time even in simulation
    ros::WallTime start = ros::WallTime::now();
    //the real work on pursuing a goal is done here
    bool done = executeCycle(request);
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
  ROS_INFO("Node was killed, aborting");  //if the node is killed then we'll abort and return
  action_server_->setAborted(move_arm_action_result_);
}

bool MoveArm::getAndSetPlanningScene(const arm_navigation_msgs::PlanningScene& planning_diff,
                                     const arm_navigation_msgs::OrderedCollisionOperations& operations) 
{
  arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_request;
  arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_response;

  revertPlanningScene();

  planning_scene_request.planning_scene_diff = planning_diff;
  planning_scene_request.operations = operations;

  if(!set_planning_scene_diff_client_.call(planning_scene_request, planning_scene_response)) {
    ROS_WARN("Can't get planning scene");
    return false;
  }

  current_planning_scene_ = planning_scene_response.planning_scene;

  planning_scene_state_ = collision_models_interface_->setPlanningScene(current_planning_scene_);

  collision_models_interface_->disableCollisionsForNonUpdatedLinks(current_group_->getPhysicalGroupName());

  if(planning_scene_state_ == NULL) {
    ROS_WARN("Problems setting local state");
    return false;
  }    
  return true;
}

void MoveArm::resetToStartState(planning_models::KinematicState* state) 
{
  planning_environment::setRobotStateAndComputeTransforms(current_planning_scene_.robot_state, *state);
}

bool MoveArm::revertPlanningScene() 
{
  if(planning_scene_state_ != NULL) 
  {
    collision_models_interface_->revertPlanningScene(planning_scene_state_);
    planning_scene_state_ = NULL;
  }
  return true;
}

void MoveArm::publishStats()
{
  move_arm_stats_.error_code.val = move_arm_action_result_.error_code.val;
  move_arm_stats_.result = arm_navigation_msgs::armNavigationErrorCodeToString(move_arm_action_result_.error_code);
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

}
/*
int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_arm");  
  ros::AsyncSpinner spinner(1); // Use 1 thread
  spinner.start();
  move_arm::MoveArm move_arm();
  if(!move_arm.initialize())
  {
    ROS_ERROR("Could not configure move arm, exiting");
    ros::shutdown();
    return 1;
  }
  ROS_INFO("Move arm action started");
  ros::waitForShutdown();
    
  return 0;
}*/
