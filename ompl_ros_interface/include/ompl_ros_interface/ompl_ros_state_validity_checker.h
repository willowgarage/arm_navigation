/*********************************************************************
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

/** \author Sachin Chitta */

#ifndef OMPL_ROS_STATE_VALIDITY_CHECKER_
#define OMPL_ROS_STATE_VALIDITY_CHECKER_

#include <planning_environment/models/collision_models_interface.h>
#include <planning_environment/util/kinematic_state_constraint_evaluator.h>
#include <arm_navigation_msgs/GetMotionPlan.h>

#include <ompl_ros_interface/helpers/ompl_ros_conversions.h>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/State.h>


namespace ompl_ros_interface
{
  /**
   * @class OmplRosStateValidityChecker
   * @brief A ROS wrapper around a ompl::base::StateValidityChecker object
   */
class OmplRosStateValidityChecker : public ompl::base::StateValidityChecker
{
public:
  /**
   * @brief A default constructor
   * @param si - A pointer to the space information used by this checker
   * @param planning_monitor - A pointer to the planning monitor instance used by this checker
   */
  OmplRosStateValidityChecker(ompl::base::SpaceInformation *si, 
                              planning_environment::CollisionModelsInterface *cmi) :
    ompl::base::StateValidityChecker(si), 
    collision_models_interface_(cmi)
  {
  }
  
  /** @brief Callback function used to determine whether a state is valid or not */
  virtual bool 	isValid  (const ompl::base::State *ompl_state) const = 0;	

  /** \brief Used by the ROS space information to print information */
  void printSettings(std::ostream &out) const;

  /*
    @brief Configure the state validity checker on request - this sets the current kinematic state, a pointer to the
    physical joint state group that this planner is planning for and passes in the request
    @param kinematic_state A pointer to the kinematic state containing joint values for the entire robot
    @param physical_joint_state_group A pointer to the joint state for the group that this planner is planning for
    @param request The motion planning request
   */
  virtual void configureOnRequest(planning_models::KinematicState *kinematic_state,
                                  planning_models::KinematicState::JointStateGroup *physical_joint_state_group,
                                  const arm_navigation_msgs::GetMotionPlan::Request &request);

  /*
    @brief Return the error code for the last state checked. 
    @return The error code for the last state that was checked.
   */
  arm_navigation_msgs::ArmNavigationErrorCodes getLastErrorCode()
  {
    return error_code_;
  }

  /*
    @brief A non-const version of isValid designed to fill in the last error code 
    @param ompl_state The state that needs to be checked
   */
  virtual bool isStateValid(const ompl::base::State *ompl_state) = 0;

protected:	
  planning_models::KinematicState::JointStateGroup *joint_state_group_;
  planning_environment::CollisionModelsInterface* collision_models_interface_;
  planning_models::KinematicState *kinematic_state_;
    
  planning_environment::KinematicConstraintEvaluatorSet path_constraint_evaluator_set_;
  planning_environment::KinematicConstraintEvaluatorSet goal_constraint_evaluator_set_;
  arm_navigation_msgs::ArmNavigationErrorCodes error_code_;
  sensor_msgs::JointState joint_state_;
  arm_navigation_msgs::Constraints getPhysicalConstraints(const arm_navigation_msgs::Constraints &constraints);
};

typedef boost::shared_ptr<ompl_ros_interface::OmplRosStateValidityChecker> OmplRosStateValidityCheckerPtr;

}

#endif
    
