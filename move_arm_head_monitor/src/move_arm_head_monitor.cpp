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
 *  \author Adam Harmat
 *********************************************************************/

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>

#include <move_arm_head_monitor/HeadMonitorAction.h>
#include <move_arm_head_monitor/HeadLookAction.h>

#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionFK.h>

#include <pr2_controllers_msgs/PointHeadAction.h>
#include <pr2_controllers_msgs/QueryTrajectoryState.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_datatypes.h>


// The head point action doesn't look exactly at target point, so add correction first
static const double HEAD_CORRECTION_DISTANCE = 0.08;

class HeadMonitor
{
protected:

  ros::NodeHandle nh_;
  ros::NodeHandle root_handle_;
  actionlib::SimpleActionServer<move_arm_head_monitor::HeadMonitorAction> head_monitor_actionserver_;
  actionlib::SimpleActionServer<move_arm_head_monitor::HeadLookAction> head_look_actionserver_;

  ros::Publisher marker_pub_;

  actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> point_head_actionclient_;

  ros::ServiceClient trajectory_state_serviceclient_;
  ros::ServiceClient forward_kinematics_serviceclient_;

  move_arm_head_monitor::HeadMonitorGoal monitor_goal_;
  move_arm_head_monitor::HeadMonitorFeedback monitor_feedback_;
  move_arm_head_monitor::HeadMonitorResult monitor_result_;

  ros::Timer monitor_timer_;

  visualization_msgs::Marker marker_;


public:

  HeadMonitor(const ros::NodeHandle &n) :
    nh_("~"),
    root_handle_(n),
    head_monitor_actionserver_(nh_, "monitor_action"),
    head_look_actionserver_(nh_, "look_action", boost::bind(&HeadMonitor::lookExecuteCallback, this, _1)),
    point_head_actionclient_("head_controller_actionserver", true)  // ie "/head_traj_controller/point_head_action"
  {

    head_monitor_actionserver_.registerGoalCallback(boost::bind(&HeadMonitor::monitorGoalCallback, this));
  	head_monitor_actionserver_.registerPreemptCallback(boost::bind(&HeadMonitor::monitorPreemptCallback, this));
  
  	monitor_timer_ = nh_.createTimer(ros::Duration(1), boost::bind(&HeadMonitor::monitorTimerCallback, this, _1));
    monitor_timer_.stop();
  
  	marker_pub_ = nh_.advertise<visualization_msgs::Marker>("point_head_target_marker", 1,true);
  
  	trajectory_state_serviceclient_ = root_handle_.serviceClient<pr2_controllers_msgs::QueryTrajectoryState>("trajectory_query_service");
  	forward_kinematics_serviceclient_ = root_handle_.serviceClient<kinematics_msgs::GetPositionFK>("forward_kinematics_service");
  
  	while(ros::ok() && !point_head_actionclient_.waitForServer(ros::Duration(5.0)))
  	{
  	  ROS_INFO("Waiting for point head action server");
  	}
  
  	while(ros::ok() && !ros::service::waitForService("trajectory_query_service", ros::Duration(5.0))) // ie "/r_arm_controller/query_state"
  	{	
  	  ROS_INFO_STREAM("Waiting for trajectory query service: "<<trajectory_state_serviceclient_.getService());
  	}
  	
  	while(ros::ok() && !ros::service::waitForService("forward_kinematics_service", ros::Duration(5.0)))  // ie "/pr2_right_arm_kinematics/get_fk"
  	{
  	  ROS_INFO_STREAM("Waiting for forward kinematics service: "<<forward_kinematics_serviceclient_.getService());
  	}
  	
  	ROS_INFO_STREAM("Starting head monitor action server for "<<ros::this_node::getName());
  }

  ~HeadMonitor(void)
  {
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
  	goal.min_duration = ros::Duration(0.4);

  	if(wait)
  	{
  	  if(point_head_actionclient_.sendGoalAndWait(goal, ros::Duration(3.0), ros::Duration(0.5)) != actionlib::SimpleClientGoalState::SUCCEEDED)
  	    ROS_WARN("Point head timed out, continuing");
  	}
  	else
  	{
      point_head_actionclient_.sendGoal(goal);
  	}
  }

  // Looks at where a target link will be at a given time by looking up future trajectory and calling forward kinematics service
  bool lookAtTarget(std::string target_link, double target_x, double target_y, double target_z, ros::Time check_time, bool wait_for_head, visualization_msgs::Marker* marker = NULL)
  {
  	pr2_controllers_msgs::QueryTrajectoryState::Request  traj_request;
  	pr2_controllers_msgs::QueryTrajectoryState::Response traj_response;
  
  	kinematics_msgs::GetPositionFK::Request  fk_request;
  	kinematics_msgs::GetPositionFK::Response fk_response;
  
  	fk_request.header.frame_id = "base_link";
  	fk_request.fk_link_names.push_back(target_link);
  
  	traj_request.time = check_time;
  
  	if(trajectory_state_serviceclient_.call(traj_request, traj_response))
  	{
  	  fk_request.robot_state.joint_state.name = traj_response.name;
  	  fk_request.robot_state.joint_state.position = traj_response.position;
  	  fk_request.robot_state.joint_state.velocity = traj_response.velocity;
  
      if(forward_kinematics_serviceclient_.call(fk_request, fk_response))
      {
        if(fk_response.error_code.val == fk_response.error_code.SUCCESS)
        {
          tf::Vector3 link_position(fk_response.pose_stamped[0].pose.position.x, fk_response.pose_stamped[0].pose.position.y, fk_response.pose_stamped[0].pose.position.z);
          tf::Quaternion link_pose(fk_response.pose_stamped[0].pose.orientation.x, fk_response.pose_stamped[0].pose.orientation.y,
          		fk_response.pose_stamped[0].pose.orientation.z, fk_response.pose_stamped[0].pose.orientation.w);
          
          tf::Transform link_transform(link_pose, link_position);
          tf::Vector3 relative_target_position(target_x, target_y, target_z);
          tf::Vector3 absolute_target_position = link_transform * relative_target_position;
    
          double approx_angle = atan2(absolute_target_position.y(), absolute_target_position.x());
          double x_corr = -HEAD_CORRECTION_DISTANCE*sin(approx_angle);
          double y_corr = HEAD_CORRECTION_DISTANCE*cos(approx_angle);
          
          ROS_DEBUG_STREAM("Approx angle of target point: " << approx_angle << "  x correction: " <<x_corr<< "  y correction: " <<y_corr);
          
          lookAt("base_link", absolute_target_position.x() + x_corr, absolute_target_position.y() + y_corr, absolute_target_position.z(), wait_for_head);
    
          if(marker)
          {
            geometry_msgs::Point tempPoint;
            tempPoint.x = absolute_target_position.x() + x_corr;
            tempPoint.y = absolute_target_position.y() + y_corr;
            tempPoint.z = absolute_target_position.z();
            marker->points.push_back(tempPoint);
            
            marker->header.stamp = ros::Time::now();
          }
        }
        else
        {
          ROS_ERROR("Forward kinematics failed");
          return false;
        }
      }
  		else
  		{
  			ROS_ERROR("Forward kinematics service call failed");
  			return false;
  		}	
  	}
  	else
  	{
  		ROS_ERROR("Trajectory service call failed");
  		return false;
  	}
  
  	return true;
  }

  // Looks at a given target at the current time and returns	
  void lookExecuteCallback(const move_arm_head_monitor::HeadLookGoalConstPtr &goalPtr)
  {
	  move_arm_head_monitor::HeadLookGoal goal(*goalPtr);
    move_arm_head_monitor::HeadLookResult result;
	
    bool success = lookAtTarget(goal.target_link, goal.target_x, goal.target_y, goal.target_z, goal.target_time, true);

  	if(success)
    {
      result.resultStatus.status = result.resultStatus.SUCCEEDED;
      ROS_DEBUG_STREAM(ros::this_node::getName()<<": Succeeded at looking");
      head_look_actionserver_.setSucceeded(result);
    }
    else
    {
      stopHead();
    
      if(head_look_actionserver_.isPreemptRequested())
      {
        result.resultStatus.status = result.resultStatus.PREEMPTED;
        ROS_DEBUG_STREAM(ros::this_node::getName()<<": Preempted while looking");
        head_look_actionserver_.setPreempted();
      }
      else
      {
        result.resultStatus.status = result.resultStatus.ABORTED;
        ROS_ERROR_STREAM(ros::this_node::getName()<<": Aborted while looking");
        head_look_actionserver_.setAborted(result);
      }
    }
    
  }

  void stopHead()
  {
  	if(point_head_actionclient_.getState().state_ == actionlib::SimpleClientGoalState::ACTIVE)
  	{
  		point_head_actionclient_.cancelGoal();
  		point_head_actionclient_.waitForResult(ros::Duration(0.2));
  	}
  }

  void monitorPreemptCallback()
  {
  	stopHead();
  	monitor_timer_.stop();
  
  	monitor_result_.resultStatus.status = monitor_result_.resultStatus.PREEMPTED;
  	ROS_DEBUG_STREAM(ros::this_node::getName()<<": Preempted");
  	head_monitor_actionserver_.setPreempted(monitor_result_);
  }

  // Periodic timer callback computes next head position and sends it to head controller
  void monitorTimerCallback(const ros::TimerEvent& event)
  {
  	if(!head_monitor_actionserver_.isActive())
  		return;
  
  	monitor_feedback_.feedbackStatus.status = monitor_feedback_.feedbackStatus.ACTIVE;
  	head_monitor_actionserver_.publishFeedback(monitor_feedback_);
  
  	ros::Time now = ros::Time::now();
  	ros::Time check_time = now + monitor_goal_.time_offset;
  
    // Stop if past goal stop time
  	if(monitor_goal_.stop_time > ros::Time(0) && now > monitor_goal_.stop_time)
  	{
  		monitor_result_.resultStatus.status = monitor_result_.resultStatus.SUCCEEDED;
   		ROS_DEBUG_STREAM(ros::this_node::getName()<<": Succeeded");
  		head_monitor_actionserver_.setSucceeded(monitor_result_);
  		monitor_timer_.stop();
  		return;
  	}
  
  	bool success = lookAtTarget(monitor_goal_.target_link, monitor_goal_.target_x, monitor_goal_.target_y, monitor_goal_.target_z, check_time, false, &marker_);
  
  	if(!success)
  	{
  		monitor_result_.resultStatus.status = monitor_result_.resultStatus.ABORTED;
  		ROS_ERROR_STREAM(ros::this_node::getName()<<": Aborted");
  		head_monitor_actionserver_.setAborted(monitor_result_);
  		monitor_timer_.stop();
  	}
  	else
  	{
  		marker_pub_.publish(marker_);
  	}	

  }

  void monitorGoalCallback()
  {
  	if(head_monitor_actionserver_.isActive())
  	{
  		stopHead();
  		monitor_timer_.stop();
  	}
  
  	monitor_goal_ = move_arm_head_monitor::HeadMonitorGoal(*head_monitor_actionserver_.acceptNewGoal());
  
  	if(head_monitor_actionserver_.isPreemptRequested())
  	{
  		monitor_result_.resultStatus.status = monitor_result_.resultStatus.PREEMPTED;
  		ROS_DEBUG_STREAM(ros::this_node::getName() << ": Preempted");
  		head_monitor_actionserver_.setPreempted(monitor_result_);
  		return;
  	}
  
    ROS_DEBUG_STREAM("Received goal");
   	ROS_DEBUG_STREAM("  Stop time: " << monitor_goal_.stop_time);
  	ROS_DEBUG_STREAM("  Max Frequency: " << monitor_goal_.max_frequency);
  	ROS_DEBUG_STREAM("  Time offset: " << monitor_goal_.time_offset);
  
  	// ---------- markers for visualization --- 
  	marker_ = visualization_msgs::Marker();
  	marker_.header.frame_id = "/base_link";
  	marker_.header.stamp = ros::Time::now();
  	marker_.ns = "basic_shapes";
  	marker_.id = 0;
  	marker_.type = visualization_msgs::Marker::POINTS;
  	marker_.action = visualization_msgs::Marker::ADD;
  	marker_.scale.x = 0.02;
  	marker_.scale.y = 0.02;
  	marker_.scale.z = 0.02;
  	marker_.color.r = 1;
  	marker_.color.g = 1;
  	marker_.color.b = 0;
  	marker_.color.a = 1;
  	marker_.lifetime = ros::Duration();
    // -------------------------------------------
  
    monitor_timer_.setPeriod(ros::Duration(1/monitor_goal_.max_frequency));
  	monitor_timer_.start();

  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_arm_head_monitor");

  ros::NodeHandle node;
  HeadMonitor head_monitor(node);
  
  ros::spin();

  return 0;
}

