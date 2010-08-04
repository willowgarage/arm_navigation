/*********************************************************************
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
*********************************************************************/

/** \author Sachin Chitta */

#include <ros/ros.h>
#include <tf/tf.h>
#include <planning_environment/monitors/planning_monitor.h>

#include <motion_planning_msgs/convert_messages.h>
#include <motion_planning_msgs/ArmNavigationErrorCodes.h>
#include <ik_constrained_planner/ik_constrained_planner.h>
using namespace ik_constrained_planner;

class IKConstrainedPlannerROS
{
public:
    
  IKConstrainedPlannerROS(void) : node_handle_("~")
  {	
    collision_models_ = new planning_environment::CollisionModels("robot_description");
    node_handle_.param("distance_metric", distance_metric_, std::string("L2Square"));
    if (node_handle_.hasParam("planning_frame_id"))
    {
      std::string frame; node_handle_.param("planning_frame_id", frame, std::string(""));
      planning_monitor_ = new planning_environment::PlanningMonitor(collision_models_, &tf_, frame);
    }
    else
	    planning_monitor_ = new planning_environment::PlanningMonitor(collision_models_, &tf_);
    //    motion_planner_ = new IKConstrainedPlanner();
	
    plan_path_service_ = node_handle_.advertiseService("plan_path", &IKConstrainedPlannerROS::planToGoal, this);
  }
    
  /** Free the memory */
  ~IKConstrainedPlannerROS(void)
  {
    delete planning_monitor_;
    delete collision_models_;
  }
    
  void run(void)
  {
    //    bool execute = false;
    
    motion_planner_.initialize(planning_monitor_,node_handle_.getNamespace());
    if (collision_models_->loadedModels())
    {
      bool verbose_collisions;	
      node_handle_.param("verbose_collisions", verbose_collisions, false);
      if (verbose_collisions)
      {
        planning_monitor_->getEnvironmentModel()->setVerbose(true);
        ROS_WARN("Verbose collisions is enabled");
      }
      else
        planning_monitor_->getEnvironmentModel()->setVerbose(false);
      
      ros::spin();
    }
    else
      ROS_ERROR("Collision models not loaded.");
  }

private:

  void contactFound(collision_space::EnvironmentModel::Contact &contact)
  {
  }  

  bool planToGoal(motion_planning_msgs::GetMotionPlan::Request &req, 
                  motion_planning_msgs::GetMotionPlan::Response &res)
  {
    motion_planning_msgs::ArmNavigationErrorCodes error_code;
    std::vector<motion_planning_msgs::ArmNavigationErrorCodes> trajectory_error_codes;
    ROS_INFO("Received request for planning");
    bool st = false;
	
    res.trajectory.joint_trajectory.points.clear();
    res.trajectory.joint_trajectory.joint_names.clear();
    res.trajectory.joint_trajectory.header.frame_id = planning_monitor_->getFrameId();
    res.trajectory.joint_trajectory.header.stamp = planning_monitor_->lastMapUpdate();
	
    st = motion_planner_.computePlan(req,res);
    if (st && !res.trajectory.joint_trajectory.points.empty()) 
    {
      planning_monitor_->setOnCollisionContactCallback(boost::bind(&IKConstrainedPlannerROS::contactFound, this, _1));
      if (!planning_monitor_->isTrajectoryValid(res.trajectory.joint_trajectory, req.motion_plan_request.start_state,planning_environment::PlanningMonitor::COLLISION_TEST, true, error_code, trajectory_error_codes)) 
        ROS_ERROR("Reported solution appears to have already become invalidated");
    }
    else
	    ROS_ERROR("Could not plan path.");    
    return st;	
  }
  
  // ROS interface 
  ros::NodeHandle                        node_handle_;
  planning_environment::CollisionModels *collision_models_;
  planning_environment::PlanningMonitor *planning_monitor_;
  tf::TransformListener                  tf_;
  ros::ServiceServer                     plan_path_service_;
    
  // planning data
  std::string distance_metric_;
  IKConstrainedPlanner motion_planner_;
};


int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "ik_constrained_planner");

  IKConstrainedPlannerROS planner;
  planner.run();
    
  return 0;
}
