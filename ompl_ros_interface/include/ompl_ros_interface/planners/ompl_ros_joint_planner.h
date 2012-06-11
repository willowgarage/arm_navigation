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

#ifndef OMPL_ROS_JOINT_PLANNER_H_
#define OMPL_ROS_JOINT_PLANNER_H_

// OMPL ROS Interface
#include <ompl_ros_interface/state_validity_checkers/ompl_ros_joint_state_validity_checker.h>
#include <ompl_ros_interface/ompl_ros_planning_group.h>
#include <ompl_ros_interface/ik/ompl_ros_ik_sampler.h>

// OMPL
#include <ompl/base/goals/GoalLazySamples.h>

namespace ompl_ros_interface
{
/**
 * @class OmplRosJointPlanner
 * @brief A joint planner - this is the planner that most applications will use
*/
  class OmplRosJointPlanner: public OmplRosPlanningGroup
  {
  public:
    
    OmplRosJointPlanner():ik_sampler_available_(false){}
    ~OmplRosJointPlanner(){}

  protected:

    /**
     * @brief Returns whether the motion planning request is valid
     */
    virtual bool isRequestValid(arm_navigation_msgs::GetMotionPlan::Request &request,
                                arm_navigation_msgs::GetMotionPlan::Response &response);

    /**
     * @brief Set the start state(s)
     */
    virtual bool setStart(arm_navigation_msgs::GetMotionPlan::Request &request,
                          arm_navigation_msgs::GetMotionPlan::Response &response);

    /**
     * @brief Set the goal state(s)
     */
    virtual bool setGoal(arm_navigation_msgs::GetMotionPlan::Request &request,
                         arm_navigation_msgs::GetMotionPlan::Response &response);

    /**
     * @brief Initialize the state validity checker
     */
    virtual bool initializeStateValidityChecker(ompl_ros_interface::OmplRosStateValidityCheckerPtr &state_validity_checker);

    /**
     * @brief Initialize the planning state space
     */
    virtual bool initializePlanningStateSpace(ompl::base::StateSpacePtr &state_space);

    /**
      @brief Returns the solution path
     */
    virtual arm_navigation_msgs::RobotTrajectory getSolutionPath();

  private:

    //Mappings in between ompl state and robot state, these are used for efficiency
    arm_navigation_msgs::RobotState robot_state_; //message representation of the state that this class is planning for
    ompl_ros_interface::OmplStateToRobotStateMapping ompl_state_to_robot_state_mapping_;
    ompl_ros_interface::RobotStateToOmplStateMapping robot_state_to_ompl_state_mapping_;

    //Mappings between ompl state and kinematic state, these are used for efficiency
    ompl_ros_interface::OmplStateToKinematicStateMapping ompl_state_to_kinematic_state_mapping_;
    ompl_ros_interface::KinematicStateToOmplStateMapping kinematic_state_to_ompl_state_mapping_;

    bool setPoseGoal(arm_navigation_msgs::GetMotionPlan::Request &request,
                     arm_navigation_msgs::GetMotionPlan::Response &response);
    
    bool setJointGoal(arm_navigation_msgs::GetMotionPlan::Request &request,
                      arm_navigation_msgs::GetMotionPlan::Response &response);
    
    std::string kinematics_solver_name_;

    std::string end_effector_name_;

    ompl_ros_interface::OmplRosIKSampler ik_sampler_;

    bool ik_sampler_available_;

  };
}
#endif //OMPL_ROS_JOINT_PLANNER_H_
