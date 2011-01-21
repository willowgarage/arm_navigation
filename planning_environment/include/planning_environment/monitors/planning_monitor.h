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

#ifndef PLANNING_ENVIRONMENT_MONITORS_PLANNING_MONITOR_
#define PLANNING_ENVIRONMENT_MONITORS_PLANNING_MONITOR_

#include "planning_environment/monitors/collision_space_monitor.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <motion_planning_msgs/Constraints.h>
#include <motion_planning_msgs/GetMotionPlan.h>
#include <motion_planning_msgs/ArmNavigationErrorCodes.h>
#include <iostream>

namespace planning_environment
{
/** \breif @b PlanningMonitor is a class which in addition to being aware
    of a robot model, and the collision model is also aware of
    constraints and can check the validity of states and paths.
*/    
class PlanningMonitor : public CollisionSpaceMonitor
{
public:
		
  PlanningMonitor(CollisionModels *cm, tf::TransformListener *tf) : CollisionSpaceMonitor(static_cast<CollisionModels*>(cm), tf)
  {
    onCollisionContact_ = NULL;	    
    loadParams();
    use_collision_map_ = true;
  }
	
  virtual ~PlanningMonitor(void)
  {
  }
	
  /** \brief Mask for validity testing functions */
  enum 
    {
      COLLISION_TEST        = 1,
      PATH_CONSTRAINTS_TEST = 2,
      GOAL_CONSTRAINTS_TEST = 4,
      JOINT_LIMITS_TEST     = 8,
      CHECK_FULL_TRAJECTORY = 16

    };	

  void getCompletePlanningScene(const std::string& group_name,
                                const motion_planning_msgs::RobotState& state_diff,
                                const motion_planning_msgs::Constraints& goal_constraints,
                                const motion_planning_msgs::Constraints& path_constraints,
                                const std::vector<motion_planning_msgs::AllowedContactSpecification>& allowed_contacts_diffs,
                                const motion_planning_msgs::OrderedCollisionOperations& ordered_collision_operations_diff,
                                const std::vector<motion_planning_msgs::LinkPadding>& link_padding_diff,
                                const std::vector<mapping_msgs::CollisionObject>& collision_object_diffs,
                                const std::vector<mapping_msgs::AttachedCollisionObject>& attached_collision_object_diffs,
                                motion_planning_msgs::RobotState& complete_robot_state,
                                motion_planning_msgs::Constraints& transformed_goal_constraints,
                                motion_planning_msgs::Constraints& transformed_path_constraints,
                                planning_environment_msgs::AllowedCollisionMatrix& allowed_collision_matrix,
                                std::vector<motion_planning_msgs::AllowedContactSpecification>& transformed_allowed_contacts,
                                std::vector<motion_planning_msgs::LinkPadding>& all_link_padding,
                                std::vector<mapping_msgs::CollisionObject>& all_collision_objects,
                                std::vector<mapping_msgs::AttachedCollisionObject>& all_attached_collision_objects,
                                mapping_msgs::CollisionMap& unmasked_collision_map);

  // bool prepareForValidityChecks(const std::vector<std::string>& joint_names,
  //                               const motion_planning_msgs::OrderedCollisionOperations& ordered_collision_operations,
  //                               const std::vector<motion_planning_msgs::AllowedContactSpecification>& allowed_contacts,
  //                               const motion_planning_msgs::Constraints& path_constraints,
  //                               const motion_planning_msgs::Constraints& goal_constraints,
  //                               const std::vector<motion_planning_msgs::LinkPadding>& link_padding,
  //                               motion_planning_msgs::ArmNavigationErrorCodes &error_code);

  // void revertToDefaultState();
	    
  // /** \brief Return true if recent enough data is available so that planning is considered safe */
  // bool isEnvironmentSafe(motion_planning_msgs::ArmNavigationErrorCodes &error_code) const;
       
  // /**
  //    @brief This function takes a path and checks for collisions and violations of path constraints, goal constraints and joint limits.   
  //    @param The input path.
  //    @param The robot state, joint values specified here are over-written by corresponding joint values in the JointPath message. 
  //    @param The test flag used to specify which set of tests should be run. The choices are collision test, path constraints test, goal constraints test and joint limits test. E.g. to specify that tests should be run for collision, path and goal constraints, set test = planning_environment_msgs::IsPlanValid::Request::COLLISION_TEST | planning_environment_msgs::IsPlanValid::Request::PATH_CONSTRAINTS_TEST | planning_environment_msgs::IsPlanValid::Request::GOAL_CONSTRAINTS_TEST;
  //    @param Set verbosity level
  //    @return True if state satisfies all the constraints, false otherwise.
  // */
  // bool isTrajectoryValid(const trajectory_msgs::JointTrajectory &trajectory, 
  //                        motion_planning_msgs::RobotState &robot_state, 
  //                        const int test, 
  //                        bool verbose, 
  //                        motion_planning_msgs::ArmNavigationErrorCodes &error_code,
  //                        std::vector<motion_planning_msgs::ArmNavigationErrorCodes> &trajectory_error_codes);

  // /**
  //    @brief This function takes a path and checks a segment of the path for collisions and violations of path constraints, goal constraints and joint limits.   
  //    @param The input path.
  //    @param The robot state, joint values specified here are over-written by corresponding joint values in the JointPath message. 
  //    @param The test flag used to specify which set of tests should be run. The choices are collision test, path constraints test, goal constraints test and joint limits test. E.g. to specify that tests should be run for collision, path and goal constraints, set test = planning_environment_msgs::IsPlanValid::Request::COLLISION_TEST | planning_environment_msgs::IsPlanValid::Request::PATH_CONSTRAINTS_TEST | planning_environment_msgs::IsPlanValid::Request::GOAL_CONSTRAINTS_TEST;
  //    @param Set verbosity level
  //    @return True if state satisfies all the constraints, false otherwise.
  // */
  // bool isTrajectoryValid(const trajectory_msgs::JointTrajectory &trajectory, 
  //                        motion_planning_msgs::RobotState &robot_state, 
  //                        unsigned int start, 
  //                        unsigned int end, 
  //                        const int test, 
  //                        bool verbose,
  //                        motion_planning_msgs::ArmNavigationErrorCodes &error_code,
  //                        std::vector<motion_planning_msgs::ArmNavigationErrorCodes> &trajectory_error_codes);

  // bool isStateValid(const motion_planning_msgs::RobotState &robot_state, 
  //                   const int test, 
  //                   bool verbose, 
  //                   motion_planning_msgs::ArmNavigationErrorCodes &error_code);
	
  // /** \brief Find the index of the state on the path that is closest to a given state */
  // int  closestStateOnTrajectory(const trajectory_msgs::JointTrajectory &trajectory, 
  //                               motion_planning_msgs::RobotState &robot_state, 
  //                               motion_planning_msgs::ArmNavigationErrorCodes &error_code) const;

  // /** \brief Find the index of the state on the path segment that is closest to a given state */
  // int  closestStateOnTrajectory(const trajectory_msgs::JointTrajectory &trajectory, 
  //                               motion_planning_msgs::RobotState &robot_state, 
  //                               unsigned int start, 
  //                               unsigned int end, 
  //                               motion_planning_msgs::ArmNavigationErrorCodes &error_code) const;
	
  // /** \brief Set the kinematic constraints the monitor should use when checking a path */
  // bool setPathConstraints(const motion_planning_msgs::Constraints &kc, 
  //                         motion_planning_msgs::ArmNavigationErrorCodes &error_code);

  // /** \brief Get the kinematic constraints the monitor uses when checking a path */
  // const motion_planning_msgs::Constraints& getPathConstraints(void) const
  // {
  //   return path_constraints_;
  // }
	
  // /** \brief Set the kinematic constraints the monitor should use when checking a path's last state (the goal) */
  // bool setGoalConstraints(const motion_planning_msgs::Constraints &kc, 
  //                         motion_planning_msgs::ArmNavigationErrorCodes &error_code);
	
  // /** \brief Get the kinematic constraints the monitor uses when checking a path's last state (the goal) */
  // const motion_planning_msgs::Constraints& getGoalConstraints(void) const
  // {
  //   return goal_constraints_;
  // }

  // /** \brief Print active constraints */
  // void printConstraints(std::ostream &out = std::cout);

  // /** \brief Clear previously set constraints */
  // void clearConstraints(void);
	
  // /** \brief Transform the frames in which constraints are specified to the one requested */
  // bool transformConstraintsToFrame(motion_planning_msgs::Constraints &kc, 
  //                                  const std::string &target,
  //                                  motion_planning_msgs::ArmNavigationErrorCodes &error_code) const;
	
  // /** \brief Transform the path to the frame requested */
  // bool transformTrajectoryToFrame(trajectory_msgs::JointTrajectory &kp, 
  //                                 motion_planning_msgs::RobotState &robot_state, 
  //                                 const std::string &target,
  //                                 motion_planning_msgs::ArmNavigationErrorCodes &error_code) const;

  // /** \brief Transform the kinematic joint to the frame requested */
  // bool transformJointToFrame(double &value, 
  //                            const std::string &joint_name, 
  //                            std::string &frame_id, 
  //                            const std::string &target,
  //                            motion_planning_msgs::ArmNavigationErrorCodes &error_code) const;

  // /** \brief Set the set of contacts allowed when collision checking */
  // void setAllowedContacts(const std::vector<motion_planning_msgs::AllowedContactSpecification> &allowedContacts);
	
  // /** \brief Set the set of contacts allowed when collision checking */
  // void setAllowedContacts(const std::vector<collision_space::EnvironmentModel::AllowedContact> &allowedContacts);
	
  // /** \brief Get the set of contacts allowed when collision checking */
  // const std::vector<collision_space::EnvironmentModel::AllowedContact>& getAllowedContacts(void) const;

  // /** \brief Print allowed contacts */
  // void printAllowedContacts(std::ostream &out = std::cout);
	
  // /** \brief Clear the set of allowed contacts */
  // void clearAllowedContacts(void);

  // bool broadcastCollisions();
	
  // /** \brief Set a callback to be called when a collision is found */
  // void setOnCollisionContactCallback(const boost::function<void(collision_space::EnvironmentModel::Contact&)> &callback)
  // {
  //   onCollisionContact_ = callback;
  // }

  // /** \brief Return the maximum amount of time that is allowed to pass between updates to the map. */
  // double getExpectedMapUpdateInterval(void) const
  // {
  //   return intervalCollisionMap_;
  // }

  // /** \brief Return the maximum amount of time that is allowed to pass between updates to the state. */
  // double getExpectedJointStateUpdateInterval(void) const
  // {
  //   return intervalState_;
  // }

  // /** \brief Return the maximum amount of time that is allowed to pass between updates to the pose. */
  // double getExpectedPoseUpdateInterval(void) const
  // {
  //   return intervalPose_;
  // }

  // void setCollisionCheck(const std::string link_name, bool state);

  // void setCollisionCheckAll(bool state);

  // void setCollisionCheckLinks(const std::vector<std::string> &link_names, bool state);

  // void setCollisionCheckOnlyLinks(const std::vector<std::string> &link_names, bool state);

  // void getChildLinks(const std::vector<std::string> &joints,std::vector<std::string> &link_names);

  // void getOrderedCollisionOperationsForOnlyCollideLinks(const std::vector<std::string> &collision_check_links, 
  //       						const motion_planning_msgs::OrderedCollisionOperations &requested_collision_operations,
  //       						motion_planning_msgs::OrderedCollisionOperations &result_collision_operations);

  // bool checkPathConstraints(const planning_models::KinematicState* state, bool verbose) const;

  // bool checkGoalConstraints(const planning_models::KinematicState* state, bool verbose) const;

protected:

  /** \brief Load ROS parameters */
  void loadParams(void);
 
  // /** \brief Transform the joint parameters (if needed) to a target frame */
  // bool transformJoint(const std::string &name, 
  //                     unsigned int index, 
  //                     double &param, 
  //                     std::string& frame_id, 
  //                     const std::string& target,
  //                     motion_planning_msgs::ArmNavigationErrorCodes &error_code) const;
	
  // /** \brief Check the path assuming it is in the frame of the model */
  // bool isTrajectoryValidAux(const trajectory_msgs::JointTrajectory &trajectory, 
  //                           motion_planning_msgs::RobotState &robot_state, 
  //                           unsigned int start, 
  //                           unsigned int end, 
  //                           const int test, 
  //                           bool verbose,
  //                           motion_planning_msgs::ArmNavigationErrorCodes &error_code,
  //                           std::vector<motion_planning_msgs::ArmNavigationErrorCodes> &trajectory_error_codes);

  // /** \brief Find the index of the state on the path that is closest to a given state assuming the path is in the frame of the model */
  // int closestStateOnTrajectoryAux(const trajectory_msgs::JointTrajectory &trajectory, 
  //                                 unsigned int start, 
  //                                 unsigned int end, 
  //                                 motion_planning_msgs::ArmNavigationErrorCodes &error_code) const;
	
  // /** \brief User callback when a collision is found */
  boost::function<void(collision_space::EnvironmentModel::Contact&)> onCollisionContact_;
	
  // std::vector<collision_space::EnvironmentModel::AllowedContact>     allowedContacts_;

  // motion_planning_msgs::Constraints path_constraints_;
  // motion_planning_msgs::Constraints goal_constraints_;

  int num_contacts_allowable_contacts_test_;
  int num_contacts_for_display_;
	
  double intervalCollisionMap_;
  double intervalState_;
  double intervalPose_;	

  ros::Publisher display_collision_pose_publisher_;
  ros::Publisher display_state_validity_publisher_;

};	
}

#endif
