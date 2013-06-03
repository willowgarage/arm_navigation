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

#ifndef OMPL_ROS_PLANNING_GROUP_H_
#define OMPL_ROS_PLANNING_GROUP_H_

// ROS
#include <ros/ros.h>
#include <ros/console.h>

// ROS msgs
#include <arm_navigation_msgs/ArmNavigationErrorCodes.h>
#include <arm_navigation_msgs/RobotTrajectory.h>

// Planning environment and models
#include <planning_environment/models/collision_models_interface.h>
#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>

// OMPL ROS Interface
#include <ompl_ros_interface/ompl_ros_state_validity_checker.h>
#include <ompl_ros_interface/ompl_ros_projection_evaluator.h>
#include <ompl_ros_interface/ompl_ros_planner_config.h>
#include <ompl_ros_interface/helpers/ompl_ros_conversions.h>

// OMPL
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

namespace ompl_ros_interface
{
/**
 * @class OmplRosPlanningGroup
 * @brief An instantiation of a particular planner for a specific group
 */
  class OmplRosPlanningGroup
  {
  public:
    
    OmplRosPlanningGroup(){}
    
    /**
       @brief Initialize the planning group from the param server
       @param param_server_prefix The param server namespace from which all data can be accessed
       @param group_name The name of the group that this class will plan for
       @param planner_config_name The planner configuration that this class will use
       @param planning_monitor A copy of the planning monitor for this planner
    */
    bool initialize(const ros::NodeHandle &node_handle,
                    const std::string &group_name,
                    const std::string &planner_config_name,
                    planning_environment::CollisionModelsInterface *cmi);
        
    /*
      @brief Return the name of the group this planner is operating on
     */
    std::string getName()
    {
      return group_name_;
    };

    /*
      @brief Return the planning frame id
     */
    std::string getFrameId()
    {
      return collision_models_interface_->getWorldFrameId();
    }

    /*
      @brief Compute the plan
      @param The motion planning request
      @param The motion planner response
     */
    bool computePlan(arm_navigation_msgs::GetMotionPlan::Request &request, 
                     arm_navigation_msgs::GetMotionPlan::Response &response);

    /**
       @brief The underlying planner to be used for planning
     */
    boost::shared_ptr<ompl::geometric::SimpleSetup> planner_;

  protected:

    /*
      @brief Check whether the request is valid. This function must be implemented by every derived class.
      @param The motion planning request
      @param The motion planner response
     */
    virtual bool isRequestValid(arm_navigation_msgs::GetMotionPlan::Request &request,
                                arm_navigation_msgs::GetMotionPlan::Response &response) = 0;

    /*
      @brief Set the start. This function must be implemented by every derived class.
      @param The motion planning request
      @param The motion planner response
     */
    virtual bool setStart(arm_navigation_msgs::GetMotionPlan::Request &request,
                          arm_navigation_msgs::GetMotionPlan::Response &response) = 0;

    /*
      @brief Set the start. This function must be implemented by every derived class.
      @param The motion planning request
      @param The motion planner response
     */
    virtual bool setGoal(arm_navigation_msgs::GetMotionPlan::Request &request,
                         arm_navigation_msgs::GetMotionPlan::Response &response) = 0;

    /**
     * @brief Initialize the state validity checker. This function must allocate and instantiate a state validity checker
     */
     virtual bool initializeStateValidityChecker(ompl_ros_interface::OmplRosStateValidityCheckerPtr &state_validity_checker) =0;

    /**
     * @brief Initialize the planning state space. This function must allocate and instantiate a state space on which planning will take place.
     */
    virtual bool initializePlanningStateSpace(ompl::base::StateSpacePtr &state_space) = 0;

    std::string group_name_;///the name of the group

    planning_environment::CollisionModelsInterface* collision_models_interface_;///A pointer to an instance of the planning monitor

    ompl::base::StateSpacePtr state_space_;///possibly abstract state

    /** 
        @brief The actual (physical) joint group that this state maps onto.
        In most cases, this will be an identity mapping but in some cases it will be through IK 
    */
    const planning_models::KinematicModel::JointModelGroup* physical_joint_group_;

    /** 
        @brief A pointer to the state in the kinematic state corresponding to this joint group 
    */
    planning_models::KinematicState::JointStateGroup* physical_joint_state_group_;
     
    /**
       @brief A state validity checker
    */
    ompl_ros_interface::OmplRosStateValidityCheckerPtr state_validity_checker_;

    /**
      @brief Returns the solution path
     */
    virtual arm_navigation_msgs::RobotTrajectory getSolutionPath() = 0;

  protected:
    ros::NodeHandle node_handle_;
    bool omplPathGeometricToRobotTrajectory(const ompl::geometric::PathGeometric &path, 
                                            arm_navigation_msgs::RobotTrajectory &robot_trajectory);
    bool finish(const bool &result);

  private:

    std::string planner_config_name_;
    boost::shared_ptr<ompl_ros_interface::PlannerConfig> planner_config_;    
        
    // The kinematic state
    boost::scoped_ptr<planning_models::KinematicState> kinematic_state_;

    ompl::base::PlannerPtr ompl_planner_;

    bool initializeProjectionEvaluator();

    bool initializePhysicalGroup();

    bool initializePlanner();
    bool initializeESTPlanner();
    bool initializeSBLPlanner();
    bool initializeRRTPlanner();
    bool initializepRRTPlanner();
    bool initializepSBLPlanner();
    bool initializeKPIECEPlanner();
    bool initializeLazyRRTPlanner();
    bool initializeLBKPIECEPlanner();
    bool initializeRRTConnectPlanner();
    bool initializeRRTStarPlanner();
    bool initializeBKPIECEPlanner();


    bool configureStateValidityChecker(arm_navigation_msgs::GetMotionPlan::Request &request,
                                       arm_navigation_msgs::GetMotionPlan::Response &response,
                                       planning_models::KinematicState *kinematic_state);

    bool transformConstraints(arm_navigation_msgs::GetMotionPlan::Request &request,
                              arm_navigation_msgs::GetMotionPlan::Response &response);

    bool setStartAndGoalStates(arm_navigation_msgs::GetMotionPlan::Request &request, 
                               arm_navigation_msgs::GetMotionPlan::Response &response);

    
  };
}
#endif //OMPL_ROS_PLANNING_GROUP_H_
