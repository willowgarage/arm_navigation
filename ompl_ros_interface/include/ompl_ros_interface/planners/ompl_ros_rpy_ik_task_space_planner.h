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

#ifndef OMPL_ROS_IK_RPY_TASK_SPACE_PLANNER_H_
#define OMPL_ROS_IK_RPY_TASK_SPACE_PLANNER_H_

// OMPL ROS Interface
#include <ompl_ros_interface/planners/ompl_ros_task_space_planner.h>
#include <ompl_ros_interface/state_validity_checkers/ompl_ros_task_space_validity_checker.h>
#include <ompl_ros_interface/state_transformers/ompl_ros_rpy_ik_state_transformer.h>

#include <arm_navigation_msgs/ArmNavigationErrorCodes.h>

namespace ompl_ros_interface
{
  class OmplRosRPYIKTaskSpacePlanner: public OmplRosTaskSpacePlanner
  {
  protected:
    virtual bool initializeStateValidityChecker(ompl_ros_interface::OmplRosStateValidityCheckerPtr &state_validity_checker);

    virtual arm_navigation_msgs::RobotTrajectory getSolutionPath();

    virtual bool constraintsToOmplState(const arm_navigation_msgs::Constraints &constraints, 
                                        ompl::base::ScopedState<ompl::base::CompoundStateSpace> &goal);

  private:

    bool poseStampedToOmplState(const geometry_msgs::PoseStamped &pose_stamped, 
                                ompl::base::ScopedState<ompl::base::CompoundStateSpace> &goal,
                                const bool &return_if_outside_constraints = true);

    geometry_msgs::PoseStamped getEndEffectorPose(const arm_navigation_msgs::RobotState &robot_state);

    virtual bool setStart(arm_navigation_msgs::GetMotionPlan::Request &request,
                          arm_navigation_msgs::GetMotionPlan::Response &response);

    bool checkAndCorrectForWrapAround(double &value, 
                                      const double &min_v, 
                                      const double &max_v);

    boost::shared_ptr<ompl::base::RealVectorBounds> original_real_vector_bounds_;

    bool getEndEffectorConstraints(const arm_navigation_msgs::Constraints &constraints,
                                   arm_navigation_msgs::PositionConstraint &position_constraint,
                                   arm_navigation_msgs::OrientationConstraint &orientation_constraint,
                                   const bool &need_both_constraints);

    bool positionConstraintToOmplStateBounds(const arm_navigation_msgs::PositionConstraint &position_constraint,
                                             ompl::base::StateSpacePtr &goal);

    bool orientationConstraintToOmplStateBounds(const arm_navigation_msgs::OrientationConstraint &orientation_constraint,
                                                ompl::base::StateSpacePtr &goal);

    std::string end_effector_name_;
      
  };
}
#endif //OMPL_ROS_IK_RPY_TASK_SPACE_PLANNER_H_
