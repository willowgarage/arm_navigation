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

using namespace ompl_planning;

class OMPLPlanning 
{
public:
    
  OMPLPlanning(void) : nodeHandle_("~")
  {	
    // register with ROS
    collisionModels_ = new planning_environment::CollisionModels("robot_description");
    nodeHandle_.param("distance_metric", distance_metric_, std::string("L2Square"));
    planningMonitor_ = new planning_environment::PlanningMonitor(collisionModels_, &tf_);
	
    planningMonitor_->waitForState();

    planningMonitor_->startEnvironmentMonitor();

    nodeHandle_.param<double>("state_delay", stateDelay_, 0.01);	
    planKinematicPathService_ = nodeHandle_.advertiseService("plan_kinematic_path", &OMPLPlanning::planToGoal, this);
  }
    
  /** Free the memory */
  ~OMPLPlanning(void)
  {
    destroyPlanningModels(models_);
    delete planningMonitor_;
    delete collisionModels_;
  }
    
  void run(void)
  {
    bool execute = false;
    std::vector<std::string> mlist;    
	
    if (collisionModels_->loadedModels())
      {
        setupPlanningModels(planningMonitor_, models_);
	    
        mlist = knownModels(models_);
        ROS_INFO("Known models:");    
        for (unsigned int i = 0 ; i < mlist.size() ; ++i)
          ROS_INFO("  * %s", mlist[i].c_str());    

        execute = !mlist.empty();
	    
        if (execute)
          ROS_INFO("Motion planning running in frame '%s'", planningMonitor_->getWorldFrameId().c_str());
      }
	
    if (execute)
      {
        bool verbose_collisions;	
        nodeHandle_.param("verbose_collisions", verbose_collisions, false);
        if (verbose_collisions)
          {
            planningMonitor_->getEnvironmentModel()->setVerbose(true);
            ROS_WARN("Verbose collisions is enabled");
          }
        else
          planningMonitor_->getEnvironmentModel()->setVerbose(false);
    
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
    res.trajectory.joint_trajectory.header.frame_id = planningMonitor_->getWorldFrameId();
    res.trajectory.joint_trajectory.header.stamp = planningMonitor_->lastMapUpdate();

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
  planning_environment::CollisionModels *collisionModels_;
  planning_environment::PlanningMonitor *planningMonitor_;
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
