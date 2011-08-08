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

/** \author Ioan Sucan, E. Gil Jones */

#ifndef PLANNING_ENVIRONMENT_MONITORS_PLANNING_MONITOR_
#define PLANNING_ENVIRONMENT_MONITORS_PLANNING_MONITOR_

#include "planning_environment/monitors/collision_space_monitor.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <arm_navigation_msgs/Constraints.h>
#include <arm_navigation_msgs/GetMotionPlan.h>
#include <arm_navigation_msgs/ArmNavigationErrorCodes.h>
#include <iostream>

namespace planning_environment
{
/** \brief @b PlanningMonitor is a class which in addition to being aware
    of a robot model, and the collision model is also aware of
    constraints and can check the validity of states and paths.
*/    
class PlanningMonitor : public CollisionSpaceMonitor
{
public:
		
  PlanningMonitor(CollisionModels *cm, tf::TransformListener *tf) : CollisionSpaceMonitor(cm, tf)
  {
    loadParams();
    use_collision_map_ = true;
  }
	
  virtual ~PlanningMonitor(void)
  {
  }
	
  bool getCompletePlanningScene(const arm_navigation_msgs::PlanningScene& planning_diff,
                                const arm_navigation_msgs::OrderedCollisionOperations& ordered_collision_operations,
                                arm_navigation_msgs::PlanningScene& planning_scene) const;

  void getAllFixedFrameTransforms(std::vector<geometry_msgs::TransformStamped>& transform_vec) const;

protected:

  /** \brief Load ROS parameters */
  void loadParams(void);
	
  double intervalCollisionMap_;
  double intervalState_;
  double intervalPose_;
};	
}

#endif
