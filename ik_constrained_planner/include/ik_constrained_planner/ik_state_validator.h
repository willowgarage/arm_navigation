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

#ifndef IK_STATE_VALIDATOR_
#define IK_STATE_VALIDATOR_

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/State.h>

#include <motion_planning_msgs/Constraints.h>
#include <motion_planning_msgs/ArmNavigationErrorCodes.h>

#include <kinematics_base/kinematics_base.h>
#include <planning_environment/monitors/planning_monitor.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <iostream>

#include <motion_planning_msgs/DisplayTrajectory.h>
#include <planning_environment_msgs/GetRobotState.h>
#include <sensor_msgs/JointState.h>

namespace ik_constrained_planner
{    

class IKStateValidator : public ompl::base::StateValidityChecker
{
public:
  IKStateValidator(ompl::base::SpaceInformation *si, 
                   planning_environment::PlanningMonitor *planning_monitor) : 
    ompl::base::StateValidityChecker(si), 
    planning_monitor_(planning_monitor),
    space_information_(si)
  {
  }
	
  virtual ~IKStateValidator(void)
  {
  }
	
  virtual bool operator()(const ompl::base::State *s) const;
	
  /** \brief Used by the planner to set the group name */
  void configure(const std::string &group_name, 
                 const std::string &redundant_joint_name,
                 const geometry_msgs::Pose &kinematics_planner_frame,
                 kinematics::KinematicsBase *kinematics_solver);

  /** \brief Used by the ROS space information to print information */
  void printSettings(std::ostream &out) const;
	
protected:	
  btTransform kinematics_planner_tf_;
  std::string group_name_;
  unsigned int redundant_joint_index_;
  planning_environment::PlanningMonitor *planning_monitor_;
  kinematics::KinematicsBase* kinematics_solver_;
  ompl::base::SpaceInformation *space_information_;
  ros::Publisher display_path_publisher_;
};
}

#endif
    
