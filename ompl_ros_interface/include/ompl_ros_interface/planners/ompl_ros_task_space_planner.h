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

#ifndef OMPL_ROS_TASK_SPACE_PLANNER_H_
#define OMPL_ROS_TASK_SPACE_PLANNER_H_

// OMPL ROS Interface
#include <ompl_ros_interface/ompl_ros_planning_group.h>
#include <ompl_ros_interface/ik/ompl_ros_ik_sampler.h>
#include <ompl_ros_interface/state_validity_checkers/ompl_ros_task_space_validity_checker.h>

#include <ompl/base/goals/GoalStates.h>

namespace ompl_ros_interface
{
/**
 * @class OmplRosTaskSpacePlanner
 * @brief A generic task space planner
 */
  class OmplRosTaskSpacePlanner: public OmplRosPlanningGroup
  {
  public:
    
    OmplRosTaskSpacePlanner(){}
    ~OmplRosTaskSpacePlanner(){}

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
     * @brief Load a state space from the parameter server
     * @param node_handle - The node handle to load the state space information from
     * @param space_name - The name of the state space to initialize
     * @param state_space - The state space to use
     * @param real_vector_index -  The index of the real vector state space in the state space (treated as a compound state space)
     */
    bool getSpaceFromParamServer(const ros::NodeHandle &node_handle,
                                    const std::string &space_name,
                                    ompl::base::StateSpacePtr& state_space,
                                    int& real_vector_index);

  protected:

    std::string planning_frame_id_;

    std::string end_effector_name_;
    /**
      @brief Returns the solution path
     */
    virtual arm_navigation_msgs::RobotTrajectory getSolutionPath() = 0;

    boost::shared_ptr<ompl_ros_interface::OmplRosStateTransformer> state_transformer_;

    virtual bool constraintsToOmplState(const arm_navigation_msgs::Constraints &constraints, 
                                        ompl::base::ScopedState<ompl::base::CompoundStateSpace> &goal);

  };
}
#endif //OMPL_ROS_TASK_SPACE_PLANNER_H_
