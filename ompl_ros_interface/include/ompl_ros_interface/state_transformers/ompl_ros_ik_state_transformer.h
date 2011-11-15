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
*********************************************************************/

/** \author Sachin Chitta, Ioan Sucan */

#ifndef OMPL_ROS_IK_STATE_TRANSFORMER_H_
#define OMPL_ROS_IK_STATE_TRANSFORMER_H_

// OMPL ROS Interface
#include <ompl_ros_interface/ompl_ros_state_transformer.h>
#include <ompl_ros_interface/helpers/ompl_ros_exception.h>

// Kinematics
#include <pluginlib/class_loader.h>
#include <kinematics_base/kinematics_base.h>

namespace ompl_ros_interface
{
/**
 * @class OmplRosIKStateTransformer
 * @brief A state trasformer that uses forward and inverse kinematics to convert to and from ompl and physical robot states
 */
class OmplRosIKStateTransformer : public OmplRosStateTransformer
{
public:
  /**
   * @brief Default constructor
   * @param state_space - The state space that the planner is operating on
   * @param physical_joint_model_group - The "physical" joint model group that the planner is operating on
   */
  OmplRosIKStateTransformer(const ompl::base::StateSpacePtr &state_space,
                            const planning_models::KinematicModel::JointModelGroup* physical_joint_model_group);
  ~OmplRosIKStateTransformer(){}
   
  /** 
   * @brief Custom initialization can be performed here
   */ 
  virtual bool initialize() = 0;
 
  /**
   * @brief Configure the transformer when a request is received. This is typically a one time configuration 
   * for each planning request.
   */ 
  virtual bool configureOnRequest(const arm_navigation_msgs::GetMotionPlan::Request &request,
                                  arm_navigation_msgs::GetMotionPlan::Response &response) = 0;

  /* @brief Compute the inverse transform (from planning state to physical state)
   */ 
  virtual bool inverseTransform(const ompl::base::State &ompl_state,
                                arm_navigation_msgs::RobotState &robot_state) = 0;

  /**
   * @brief Compute the forward transform (from physical state to planning state)
   */ 
  virtual bool forwardTransform(const arm_navigation_msgs::RobotState &robot_state,
                                ompl::base::State &ompl_state) = 0;

  /** 
   * @brief Get the frame in which the kinematics solver is operating
   */
  virtual std::string getFrame() 
  {
    if(kinematics_solver_)
      return kinematics_solver_->getBaseName();
    else
      return std::string(" ");
  };

  /** 
   * @brief Get a default physical state
  */ 
  virtual  arm_navigation_msgs::RobotState getDefaultState() = 0;

protected:
  kinematics::KinematicsBase* kinematics_solver_;

private:
  std::string group_name_;
  std::string physical_group_name_;
  pluginlib::ClassLoader<kinematics::KinematicsBase> kinematics_loader_;

};
}

#endif //OMPL_ROS_IK_STATE_TRANSFORMER_H_
