/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

/** \author Sachin Chitta */

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <planning_environment_msgs/GetRobotState.h>
#include <motion_planning_msgs/FilterJointTrajectory.h>
#include <planning_environment_msgs/GetJointTrajectoryValidity.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/action_client.h>

#include <planning_environment/monitors/joint_state_monitor.h>
#include <motion_planning_msgs/convert_messages.h>

#include <boost/thread/condition.hpp>
#include <boost/scoped_ptr.hpp>
#include <algorithm>
#include <string>
#include <limits>

namespace collision_free_arm_trajectory_controller
{
static const std::string TRAJECTORY_FILTER = "filter_trajectory";
static const double MIN_DELTA = 0.01;
typedef actionlib::ActionClient<pr2_controllers_msgs::JointTrajectoryAction> JointExecutorActionClient;

enum ControllerState{
  START_CONTROL,
  MONITOR
};


class CollisionFreeArmTrajectoryController
{

public:
  CollisionFreeArmTrajectoryController(): private_handle_("~"), active_goal_(false)
  {
    traj_action_client_ = NULL;
    ros::service::waitForService("get_execution_safety");
    ros::service::waitForService("filter_trajectory");
    ros::service::waitForService("get_robot_state");

    check_trajectory_validity_client_ = node_handle_.serviceClient<planning_environment_msgs::GetJointTrajectoryValidity>("get_execution_safety");
    filter_trajectory_client_ = node_handle_.serviceClient<motion_planning_msgs::FilterJointTrajectory>("filter_trajectory");      
    get_state_client_ = node_handle_.serviceClient<planning_environment_msgs::GetRobotState>("get_robot_state");

    std::string traj_action_name, group_name;
    private_handle_.param<std::string>("traj_action_name", traj_action_name, "action");
    private_handle_.param<std::string>("group_name", group_name, "");
    traj_action_client_ = new JointExecutorActionClient(traj_action_name);
    action_server_.reset(new actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction>(node_handle_, "collision_free_arm_trajectory_action_" + group_name, boost::bind(&CollisionFreeArmTrajectoryController::executeTrajectory, this, _1)));
    state_ = START_CONTROL;
  }

  ~CollisionFreeArmTrajectoryController()
  {
    if(traj_action_client_) 
      delete traj_action_client_;
  }


  bool execute(pr2_controllers_msgs::JointTrajectoryGoal &goal)
  {
    switch(state_)
    {
    case START_CONTROL:
      {
        ROS_DEBUG("Starting control");
        addCurrentState(goal);
        if(isTrajectoryValid(goal.trajectory))
        {
          goal.trajectory.header.stamp = ros::Time::now();
          sendGoalToController(goal);
          state_ = MONITOR;
        }
        else
        {
          traj_action_client_->cancelAllGoals();
          action_server_->setAborted();
          ROS_INFO("Aborting since trajectory is unsafe");
        }
        break;
      }
    case MONITOR:
      {
        if(!isTrajectoryValid(goal.trajectory))
        {
          traj_action_client_->cancelAllGoals();
          action_server_->setAborted();
          ROS_INFO("Aborting since trajectory is unsafe");
        }
        break;
      }
    default:
      {
        ROS_ERROR("Should not be here");
        break;
      }
    }
    if(!action_server_->isActive())
    {
      ROS_DEBUG("Controller no longer has an active goal");
      return true;
    }
    return false;
  }

  void executeTrajectory(const pr2_controllers_msgs::JointTrajectoryGoalConstPtr &goal_input)
  {
    ROS_INFO("Got trajectory with %d points and %d joints",(int)goal_input->trajectory.points.size(),(int)goal_input->trajectory.joint_names.size());
 
    //now we need to shove it into the action message
    pr2_controllers_msgs::JointTrajectoryGoal goal = *goal_input;  
    state_ = START_CONTROL;
    while(node_handle_.ok())
    {
      if(action_server_->isPreemptRequested())
      {
        if(action_server_->isNewGoalAvailable())
        {
          action_server_->setAborted();
          goal = *(action_server_->acceptNewGoal());
          state_ = START_CONTROL;
        }
      }
      bool done = execute(goal);
      if(done)
        return;
      ros::Duration(0.01).sleep();
    }
    ROS_INFO("Node was aborted");
    action_server_->setAborted();
    return;
  }


  bool isTrajectoryValid(const trajectory_msgs::JointTrajectory &traj)
  {
    planning_environment_msgs::GetJointTrajectoryValidity::Request req;
    planning_environment_msgs::GetJointTrajectoryValidity::Response res;
        
    ROS_DEBUG("Received trajectory has %d points with %d joints",(int) traj.points.size(),(int)traj.joint_names.size());
    trajectory_msgs::JointTrajectory traj_discretized;
    discretizeTrajectory(traj,traj_discretized);
    req.trajectory = traj_discretized;
    ROS_DEBUG("Got robot state");

    getRobotState(req.robot_state);
    req.check_path_constraints = true;
    req.check_collisions =  true;

    if(check_trajectory_validity_client_.call(req,res))
    {
      ROS_DEBUG("Service call to check plan validity succeeded");
      if(res.error_code.val == res.error_code.SUCCESS)
        return true;
      else
      {
        ROS_ERROR("Trajectory invalid. Error code: %d",res.error_code.val);
        return false;
      }
    }
    else
    {
      ROS_ERROR("Service call to check trajectory validity failed on %s",check_trajectory_validity_client_.getService().c_str());
      return false;
    }
  }

  void sendGoalToController(const pr2_controllers_msgs::JointTrajectoryGoal &goal)
  {
    goal_handle_ = traj_action_client_->sendGoal(goal,boost::bind(&CollisionFreeArmTrajectoryController::transitionCallback, this, _1));
  }

  void transitionCallback(JointExecutorActionClient::GoalHandle gh) 
  {
    actionlib::CommState comm_state = gh.getCommState();
    if(comm_state.state_ == actionlib::CommState::DONE)
    {
      switch(gh.getTerminalState().state_)
      {
      case actionlib::TerminalState::RECALLED:
      case actionlib::TerminalState::REJECTED:
      case actionlib::TerminalState::PREEMPTED:
      case actionlib::TerminalState::ABORTED:
      case actionlib::TerminalState::LOST:
        ROS_DEBUG("Trajectory action did not succeed");
        action_server_->setAborted();
        return;
      case actionlib::TerminalState::SUCCEEDED:
        ROS_DEBUG("Reached goal");
        action_server_->setSucceeded();
        return;
      default:
        ROS_DEBUG("Unknown terminal state [%u]",gh.getTerminalState().state_);
      }
    }
    else
      ROS_DEBUG("Unknown comm state");

  }

  void discretizeTrajectory(const trajectory_msgs::JointTrajectory &trajectory, trajectory_msgs::JointTrajectory &trajectory_out)
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
      int num_intervals =(int) (diff/MIN_DELTA+0.5);
      
      for(unsigned int k=0; k < (unsigned int) num_intervals; k++)
      {
        trajectory_msgs::JointTrajectoryPoint point;
        for(unsigned int j=0; j < trajectory.points[i].positions.size(); j++)
        {
          double start = trajectory.points[i-1].positions[j];
          double end   = trajectory.points[i].positions[j];
          point.positions.push_back(start + (end-start)*k/num_intervals);
        }
        point.time_from_start = ros::Duration(trajectory.points[i].time_from_start.toSec() + k* (trajectory.points[i].time_from_start - trajectory.points[i].time_from_start).toSec()/num_intervals);
        trajectory_out.points.push_back(point);
      }
    }
    trajectory_out.points.push_back(trajectory.points.back());
  }

  bool filterTrajectory(trajectory_msgs::JointTrajectory &trajectory)
  {
    motion_planning_msgs::FilterJointTrajectory::Request  req;
    motion_planning_msgs::FilterJointTrajectory::Response res;
    req.filter_request.trajectory = trajectory;
    if(filter_trajectory_client_.call(req,res))
    {
      if(res.error_code.val == res.error_code.SUCCESS)
      {
        trajectory = res.trajectory;
        return true;
      }
      else
      {
        ROS_ERROR("Trajectory filtering failed");
        return false;
      }
    }
    else
    {
      ROS_ERROR("Service call to filter trajectory failed.");
      return false;
    }

  }

  bool addCurrentState(pr2_controllers_msgs::JointTrajectoryGoal &goal)
  {
    // get the current state
    double d = 0.0;
    sensor_msgs::JointState current = state_monitor_.getJointState(goal.trajectory.joint_names);
    for (unsigned int i = 0 ; i < current.position.size() ; ++i)
    {
      double dif = current.position[i] - goal.trajectory.points[0].positions[i];
      d = std::max<double>(d,fabs(dif));
    }	    
    // decide whether we place the current state in front of the path
    int include_first = (d > 0.1) ? 1 : 0;
    if (include_first)
    {
      trajectory_msgs::JointTrajectory start_segment;
      start_segment.points.resize(2);
      start_segment.points[0].positions = motion_planning_msgs::jointStateToJointTrajectoryPoint(current).positions;
      start_segment.points[0].time_from_start = ros::Duration(0.0);
      start_segment.points[1] = goal.trajectory.points[0];
      start_segment.joint_names = goal.trajectory.joint_names;
      if(!filterTrajectory(start_segment))
        ROS_WARN("Could not filter trajectory");

      trajectory_msgs::JointTrajectory traj;
      traj.points = start_segment.points;
      traj.joint_names = start_segment.joint_names;
      for (unsigned int i = 1; i < goal.trajectory.points.size() ; ++i)
      {
        traj.points.push_back(goal.trajectory.points[i]);
        traj.points.back().time_from_start = goal.trajectory.points[i].time_from_start + start_segment.points.back().time_from_start;
      }
      goal.trajectory = traj;
    }
    return true;
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

private:
  ros::NodeHandle node_handle_, private_handle_;
  JointExecutorActionClient *traj_action_client_;
  JointExecutorActionClient::GoalHandle goal_handle_;
  boost::shared_ptr<actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction> > action_server_;
  ros::ServiceClient get_state_client_, check_trajectory_validity_client_, filter_trajectory_client_;
  planning_environment::JointStateMonitor state_monitor_;
  bool active_goal_;
  ControllerState state_;
};
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "collision_free_arm_trajectory_controller");
  collision_free_arm_trajectory_controller::CollisionFreeArmTrajectoryController cf;
  ROS_INFO("Collision free arm trajectory controller started");
  ros::spin();    
  return 0;
}

