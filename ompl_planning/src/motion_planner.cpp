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

/** \author Ioan Sucan */

#include "ompl_planning/Model.h"
#include "request_handler/RequestHandler.h"
#include <motion_planning_msgs/convert_messages.h>
#include <motion_planning_msgs/ArmNavigationErrorCodes.h>
#include <tf/transform_listener.h>

using namespace ompl_planning;

class OMPLPlanning 
{
public:
    
  OMPLPlanning(void) : nodeHandle_("~")
  {	
    // register with ROS
    collision_models_interface_ = new planning_environment::CollisionModelsInterface("robot_description");
    nodeHandle_.param("distance_metric", distance_metric_, std::string("L2Square"));
	
    nodeHandle_.param<double>("state_delay", stateDelay_, 0.01);	

    while(nodeHandle_.ok()) {
      bool got_tf = tf_.waitForTransform(collision_models_interface_->getWorldFrameId(), collision_models_interface_->getRobotFrameId(),
                                         ros::Time::now(), ros::Duration(5.0));
      if(got_tf) {
        break;
      } else {
        ROS_INFO_STREAM("Waiting for tf");
      }
    }
    planKinematicPathService_ = nodeHandle_.advertiseService("plan_kinematic_path", &OMPLPlanning::planToGoal, this);
  }
    
  /** Free the memory */
  ~OMPLPlanning(void)
  {
    destroyPlanningModels(models_);
    delete collision_models_interface_;
  }
    
  void run(void)
  {
    bool execute = false;
    std::vector<std::string> mlist;    
	
    if (collision_models_interface_->loadedModels())
      {
        setupPlanningModels(collision_models_interface_, models_);
	    
        mlist = knownModels(models_);
        ROS_INFO("Known models:");    
        for (unsigned int i = 0 ; i < mlist.size() ; ++i)
          ROS_INFO("  * %s", mlist[i].c_str());    

        execute = !mlist.empty();
	    
        if (execute)
          ROS_INFO("Motion planning running in frame '%s'", collision_models_interface_->getWorldFrameId().c_str());
      }
	
    if (execute)
      {
        bool verbose_collisions;	
        ros::spin();
      }
    else
      if (mlist.empty())
        ROS_ERROR("No robot model loaded. OMPL planning node cannot start.");
  }

private:
  bool planToGoal(motion_planning_msgs::GetMotionPlan::Request &req, motion_planning_msgs::GetMotionPlan::Response &res)
  {
    motion_planning_msgs::ArmNavigationErrorCodes error_code;
    std::vector<motion_planning_msgs::ArmNavigationErrorCodes> trajectory_error_codes;
    ROS_INFO("Received request for planning");
    bool st = false;
	
    res.trajectory.joint_trajectory.points.clear();
    res.trajectory.joint_trajectory.joint_names.clear();
    res.trajectory.joint_trajectory.header.frame_id = collision_models_interface_->getWorldFrameId();
    res.trajectory.joint_trajectory.header.stamp = ros::Time::now();

    st = requestHandler_.computePlan(models_, stateDelay_, req, res, distance_metric_);
    // if (st && !res.trajectory.joint_trajectory.points.empty())
    //   planningMonitor_->setOnCollisionContactCallback(boost::bind(&ompl_planning::RequestHandler::contactFound, (&requestHandler_), _1));
    //   if (!planningMonitor_->isTrajectoryValid(res.trajectory.joint_trajectory,res.robot_state,planning_environment::PlanningMonitor::COLLISION_TEST, true, error_code, trajectory_error_codes))
    //     ROS_ERROR("Reported solution appears to have already become invalidated");
    //   planningMonitor_->setOnCollisionContactCallback(NULL);
    return st;	
  }
    
  // ROS interface 
  ros::NodeHandle                        nodeHandle_;
  planning_environment::CollisionModelsInterface *collision_models_interface_;
  tf::TransformListener                  tf_;
  ros::ServiceServer                     planKinematicPathService_;
  double                                 stateDelay_;
    
  // planning data
  ModelMap                               models_;
  RequestHandler                         requestHandler_;
  std::string distance_metric_;
};


int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "ompl_planning");

  OMPLPlanning planner;
  planner.run();
    
  return 0;
}
