/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

#ifndef COLLISION_PROXIMITY_PLANNER_H_
#define COLLISION_PROXIMITY_PLANNER_H_

#include <ros/ros.h>
#include <collision_proximity_planner/chomp_robot_model.h>
#include <collision_proximity/collision_proximity_space.h>

// System
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <algorithm>
#include <vector>
#include <string>
#include <map>

// KDL
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

// ROS msgs
#include <motion_planning_msgs/FilterJointTrajectory.h>
#include <motion_planning_msgs/RobotTrajectory.h>
#include <motion_planning_msgs/RobotState.h>
#include <visualization_msgs/MarkerArray.h>
#include <collision_proximity_planner/GetFreePath.h>
#include <motion_planning_msgs/DisplayTrajectory.h>

// Arm Navigation
#include <planning_environment/monitors/collision_space_monitor.h>
#include <planning_environment/monitors/planning_monitor.h>
#include <spline_smoother/cubic_trajectory.h>

// MISC
#include <angles/angles.h>

namespace collision_proximity_planner
{
  class CollisionProximityPlanner
  {
  public:


    CollisionProximityPlanner();

    virtual ~CollisionProximityPlanner();    

    bool initialize();

    bool initialize(const std::string &group_name);

    void fillInGroupState(motion_planning_msgs::RobotState &robot_state,
                          const motion_planning_msgs::RobotState &group_state);

    bool findPathToFreeState(const motion_planning_msgs::RobotState &robot_state, 
                             motion_planning_msgs::RobotTrajectory &robot_trajectory);

    /**
     * @brief Set the robot state that you want to check. Note that this locks the collision space 
     * and you will have to call clear() to unlock the space.
     * @param robot_state The full robot state.
     * @return True if setting robot state was successful
     */
    bool setRobotState(const motion_planning_msgs::RobotState &robot_state);

    /**
     * @brief Set a group state. This must be called before you can make multiple queries to refineState below.
     * This function can be used to define a mapping between the group state specified in the argument and 
     * the internal group state for more efficiency.
     * @param group_state 
     */
    bool setGroupState(const motion_planning_msgs::RobotState &group_state);

    /**
     * @brief Given a robot state, get the gradient direction to be moved in.
     * @param joint_state_group The group state that needs to be refined
     * @return True if a valid refinement was found, false otherwise
     */
    bool getStateGradient(const motion_planning_msgs::RobotState &group_state,
                          motion_planning_msgs::RobotState &gradient);

    bool refineState(const motion_planning_msgs::RobotState &group_state, 
                     motion_planning_msgs::RobotTrajectory &robot_trajectory);

    void clear();

  private:
    ros::Publisher display_trajectory_publisher_;

    bool mapGroupState(const motion_planning_msgs::RobotState &group_state,const std::vector<int>& mapping);
    bool calculateCollisionIncrements(Eigen::MatrixXd &collision_increments);
    void isParentJoint(const int& link_index, const int& joint_index);
    void setPlanningMonitorToCurrentState();
    void performForwardKinematics(KDL::JntArray &jnt_array, const bool& full);
    void updateJointState(KDL::JntArray &jnt_array, Eigen::MatrixXd &collision_increments);
    void getGroupArray(const KDL::JntArray &jnt_array,
                       const std::vector<int> &group_joint_to_kdl_joint_index,
                       KDL::JntArray &jnt_array_group);
    void fillInGroupArray(const KDL::JntArray &jnt_array_group,
                          const std::vector<int> &group_joint_to_kdl_joint_index,
                          KDL::JntArray &jnt_array);
    void updateGroupRobotState(const KDL::JntArray &jnt_array);
    void updateCollisionProximitySpace(const motion_planning_msgs::RobotState &group_state);
    void kdlJointTrajectoryToRobotTrajectory(std::vector<KDL::JntArray> &jnt_trajectory,
                                             motion_planning_msgs::RobotTrajectory &robot_trajectory);
    void visualizeRobotTrajectory(const motion_planning_msgs::RobotTrajectory &robot_trajectory);

    motion_planning_msgs::RobotState robot_state_group_;

    ros::NodeHandle private_handle_, root_handle_;
    planning_environment::CollisionModels* collision_models_;
    planning_environment::CollisionSpaceMonitor* monitor_;
    planning_environment::PlanningMonitor* planning_monitor_;
    collision_proximity::CollisionProximitySpace* collision_proximity_space_;
    std::string reference_frame_, group_name_cps_;
    int num_joints_;
    chomp::ChompRobotModel chomp_robot_model_;
    const chomp::ChompRobotModel::ChompPlanningGroup *planning_group_;
    int max_iterations_;
    double max_joint_update_;

    bool use_pseudo_inverse_;
    Eigen::MatrixXd jacobian_;
    Eigen::MatrixXd jacobian_pseudo_inverse_;
    Eigen::MatrixXd jacobian_jacobian_tranpose_;
    Eigen::MatrixXd collision_increments_;
    
    std::vector<KDL::Vector> joint_axis_;
    std::vector<KDL::Vector> joint_pos_;
    std::vector<KDL::Frame> segment_frames_;
    std::vector<KDL::Vector> collision_point_pos_;
    
    std::vector<Eigen::Map<Eigen::Vector3d> > joint_axis_eigen_;
    std::vector<Eigen::Map<Eigen::Vector3d> > joint_pos_eigen_;
    std::vector<Eigen::Map<Eigen::Vector3d> > collision_point_pos_eigen_;
    std::vector<double> collision_point_potential_;
    std::vector<double> collision_point_vel_mag_;
    std::vector<Eigen::Vector3d> collision_point_potential_gradient_;

    std::vector<int> group_joint_to_kdl_joint_index_;
    
    tf::TransformListener tf_;

    std::vector<std::vector<int> > active_joints_;
    
    ros::Publisher vis_marker_array_publisher_, vis_marker_publisher_;
    template<typename Derived>
    void getJacobian(const int& link_index,
                     std::vector<Eigen::Map<Eigen::Vector3d> >& joint_pos, 
                     std::vector<Eigen::Map<Eigen::Vector3d> >& joint_axis,
                     Eigen::Vector3d& collision_point_pos, 
                     Eigen::MatrixBase<Derived>& jacobian, 
                     const std::vector<int>& group_joint_to_kdl_joint_index) const;
    inline bool isParentJoint(const int& link_index, const int& joint_index) const;

    bool getFreePath(collision_proximity_planner::GetFreePath::Request &req,
                     collision_proximity_planner::GetFreePath::Response &res);

    ros::ServiceServer planning_service_;

    KDL::JntArray jnt_array_, jnt_array_group_;
    std::vector<int> group_state_joint_array_group_mapping_, joint_array_group_group_state_mapping_;
  };

template<typename Derived>
void CollisionProximityPlanner::getJacobian(const int& link_index,
                                            std::vector<Eigen::Map<Eigen::Vector3d> >& joint_pos, 
                                            std::vector<Eigen::Map<Eigen::Vector3d> >& joint_axis,
                                            Eigen::Vector3d& collision_point_pos, 
                                            Eigen::MatrixBase<Derived>& jacobian, 
                                            const std::vector<int>& group_joint_to_kdl_joint_index) const
{
  for(unsigned int joint = 0; joint < group_joint_to_kdl_joint_index.size(); joint++) 
  {
    if(!isParentJoint(link_index, group_joint_to_kdl_joint_index[joint]))
    {
      // since the joint is not active, fill the jacobian column with zeros
      jacobian.col(joint).setZero();
    }
    else
    {
      int kj = group_joint_to_kdl_joint_index[joint];
      jacobian.col(joint) = joint_axis[kj].cross(collision_point_pos - joint_pos[kj]);
    }
  }
}

inline bool CollisionProximityPlanner::isParentJoint(const int& link_index, const int& joint_index) const
{
  return(find(active_joints_[link_index].begin(), active_joints_[link_index].end(), joint_index) != active_joints_[link_index].end());
}

} // namespace collision_proximity_planner

#endif /**COLLISION_PROXIMITY_PLANNER_H_**/
