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

#ifndef OMPL_ROS_CONVERSIONS_
#define OMPL_ROS_CONVERSIONS_

#include <ros/ros.h>
#include <ros/console.h>
#include <angles/angles.h>

#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/State.h>

#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
#include <arm_navigation_msgs/convert_messages.h>
#include <arm_navigation_msgs/RobotState.h>
#include <arm_navigation_msgs/RobotTrajectory.h>
#include <arm_navigation_msgs/JointConstraint.h>
#include <sensor_msgs/JointState.h>

namespace ompl_ros_interface
{

/**
   @brief enumeration of different mapping types for ompl
*/
typedef enum{REAL_VECTOR, SO2, SO3, SE2, SE3, COMPOUND, UNKNOWN}MAPPING_TYPE;

/**
 * @class This class contains a mapping from an ompl::base::CompoundState object 
 * to a arm_navigation_msgs::RobotState object
 */
class OmplStateToRobotStateMapping
{
public:
  OmplStateToRobotStateMapping():real_vector_index(-1){}
  std::vector<int> ompl_state_mapping;
  int real_vector_index;
  std::vector<int> real_vector_mapping;
  std::vector<ompl_ros_interface::MAPPING_TYPE> mapping_type;
  unsigned int num_joints;
};

/**
 * @class This class contains a mapping from an arm_navigation_msgs::RobotState object 
 * to a ompl::base::CompoundState object
 */
class RobotStateToOmplStateMapping
{
public:
  RobotStateToOmplStateMapping():real_vector_index(-1){}
  std::vector<int> multi_dof_mapping;
  int real_vector_index;
  std::vector<int> joint_state_mapping;
  std::vector<ompl_ros_interface::MAPPING_TYPE> joint_mapping_type;
  std::vector<ompl_ros_interface::MAPPING_TYPE> multi_dof_joint_mapping_type;
};

/**
 * @class This class contains a mapping from a kinematic_state object
 * to an ompl::base::CompoundState object
 */
class KinematicStateToOmplStateMapping
{
public:
  KinematicStateToOmplStateMapping():real_vector_index(-1){}
  int real_vector_index;
  std::vector<unsigned int> joint_state_mapping;
  std::vector<ompl_ros_interface::MAPPING_TYPE> joint_mapping_type;
};

/**
 * @class This class contains a mapping from a kinematic_state object
 * to an ompl::base::CompoundState object
 */
class OmplStateToKinematicStateMapping
{
public:
  OmplStateToKinematicStateMapping():real_vector_index(-1){}
  int real_vector_index;
  std::vector<unsigned int> ompl_state_mapping;
  std::vector<unsigned int> real_vector_mapping;
  std::vector<ompl_ros_interface::MAPPING_TYPE> mapping_type;
};

/**
 * @class This class contains a mapping from a kinematic_state object
 * to an ompl::base::CompoundState object
 */
class RobotStateToKinematicStateMapping
{
public:
  std::vector<int> multi_dof_mapping;
  std::vector<int> joint_state_mapping;
};


ompl_ros_interface::MAPPING_TYPE getMappingType(const planning_models::KinematicModel::JointModel *joint_model);

ompl_ros_interface::MAPPING_TYPE getMappingType(const ompl::base::StateSpace *state_space);

ompl_ros_interface::MAPPING_TYPE getMappingType(const ompl::base::StateSpacePtr &state_space);

/**
 * @brief create a object of type ompl::base::CompoundStateSpace from an object of 
 * type planning_models::KinematicModel::JointGroup
 * @param joint_group The joint group to construct this from
 * @param state_space The output state space
 * @param ompl_kinematic_mapping Mapping from the ompl state to the corresponding kinematic state
 * @param kinematic_ompl_mapping Mapping from the kinematic state to the corresponding ompl state
 */
ompl::base::StateSpacePtr jointGroupToOmplStateSpacePtr(const planning_models::KinematicModel::JointModelGroup *joint_group, 
                                                              ompl_ros_interface::OmplStateToKinematicStateMapping &ompl_kinematic_mapping,
                                                              ompl_ros_interface::KinematicStateToOmplStateMapping &kinematic_ompl_mapping);

/**
 * @brief Create a mapping from the ROS message robot_state to an ompl state. This function is 
 * intended to help in efficient conversion between ROS messages and an ompl state object by 
 * providing a mapping that can be cached and used frequently.
 * @param robot_state The robot state to create the mapping from
 * @param ompl_state The ompl state to create the mapping to
 * @param mapping The resultant mapping
 */
bool getRobotStateToOmplStateMapping(const arm_navigation_msgs::RobotState &robot_state, 
                                     const ompl::base::ScopedState<ompl::base::CompoundStateSpace> &ompl_state,
                                     ompl_ros_interface::RobotStateToOmplStateMapping &mapping,
                                     const bool &fail_if_match_not_found = true);

/**
 * @brief Create a mapping from the ROS message joint_state to an ompl state. This function is 
 * intended to help in efficient conversion between ROS messages and OMPL state objects by 
 * providing a mapping that can be cached and used frequently.
 * @param robot_state The joint state message to create the mapping from
 * @param ompl_state The ompl state to create the mapping to
 * @param mapping The resultant mapping
 */
bool getJointStateToOmplStateMapping(const sensor_msgs::JointState &joint_state, 
                                     const ompl::base::ScopedState<ompl::base::CompoundStateSpace> &ompl_state,
                                     ompl_ros_interface::RobotStateToOmplStateMapping &mapping,
                                     const bool &fail_if_match_not_found = true);

/**
 * @brief Create a mapping from an ompl state to the ROS message robot_state.  This function is 
 * intended to help in efficient conversion between ROS messages and OMPL state objects by 
 * providing a mapping that can be cached and used frequently.
 * @param ompl_state The ompl state to create the mapping from
 * @param joint_state The joint state message to create the mapping to
 * @param mapping The resultant mapping
 */
bool getOmplStateToJointStateMapping(const ompl::base::ScopedState<ompl::base::CompoundStateSpace> &ompl_state,
                                     const sensor_msgs::JointState &joint_state,
                                     ompl_ros_interface::OmplStateToRobotStateMapping &mapping,
                                     const bool &fail_if_match_not_found = true);

/**
 * @brief Create a mapping from an ompl state to the ROS message robot_state.  This function is 
 * intended to help in efficient conversion between ROS messages and OMPL state objects by 
 * providing a mapping that can be cached and used frequently.
 * @param ompl_state The ompl state to create the mapping from
 * @param robot_state The joint state message to create the mapping to
 * @param mapping The resultant mapping
 */
bool getOmplStateToRobotStateMapping(const ompl::base::ScopedState<ompl::base::CompoundStateSpace> &ompl_state,
                                     const arm_navigation_msgs::RobotState &robot_state, 
                                     ompl_ros_interface::OmplStateToRobotStateMapping &mapping,
                                     const bool &fail_if_match_not_found = true);

/**
 * @brief Convert an ompl state to the ROS message robot_state.  This function is 
 * intended to help in efficient conversion between ROS messages and OMPL state objects by 
 *  providing a mapping that can be cached and used frequently.
 *  @param ompl_state The ompl state to create the mapping from
 *  @param mapping The mapping to use for this conversion
 *  @param robot_state The robot state message to create the mapping to
 */
bool omplStateToRobotState(const ompl::base::ScopedState<ompl::base::CompoundStateSpace> &ompl_state,
                           const ompl_ros_interface::OmplStateToRobotStateMapping &mapping,
                           arm_navigation_msgs::RobotState &robot_state);

/**
 * @brief Convert an ompl state to the ROS message robot_state.  This function is 
 * intended to help in efficient conversion between ROS messages and OMPL state objects by 
 * providing a mapping that can be cached and used frequently.
 * @param ompl_state The ompl state to create the mapping from
 * @param mapping The mapping to use for this conversion
 * @param robot_state The robot state message to create the mapping to
 */
bool omplStateToRobotState(const ompl::base::State &ompl_state,
                           const ompl_ros_interface::OmplStateToRobotStateMapping &mapping,
                           arm_navigation_msgs::RobotState &robot_state);

/**
 * @brief Convert an ompl state to the ROS message joint_state.  This function is 
 * intended to help in efficient conversion between ROS messages and OMPL state objects by 
 * providing a mapping that can be cached and used frequently.
 * @param ompl_state The ompl state to create the mapping from
 * @param mapping The mapping to use for this conversion
 * @param joint_state The joint state message to create the mapping to
*/
bool omplRealVectorStateToJointState(const ompl::base::RealVectorStateSpace::StateType &ompl_state,
                                     const ompl_ros_interface::OmplStateToRobotStateMapping &mapping,
                                     sensor_msgs::JointState &joint_state);

/**
 * @brief Convert a ROS message robot state to an ompl state.  This function is 
 * intended to help in efficient conversion between ROS messages and OMPL state objects by 
 * providing a mapping that can be cached and used frequently.
 * @param robot_state The robot state message to create the mapping to
 * @param mapping The mapping to use for this conversion
 * @param ompl_state The ompl state to create the mapping from
*/
bool robotStateToOmplState(const arm_navigation_msgs::RobotState &robot_state,
                           const ompl_ros_interface::RobotStateToOmplStateMapping &mapping,
                           ompl::base::ScopedState<ompl::base::CompoundStateSpace> &ompl_state,
                           const bool &fail_if_match_not_found = true);

/**
 * @brief Convert a ROS message robot state to an ompl state.  This function is 
 * intended to help in efficient conversion between ROS messages and OMPL state objects by 
 * providing a mapping that can be cached and used frequently.
 * @param robot_state The robot state message to create the mapping to
 * @param mapping The mapping to use for this conversion
 * @param ompl_state The ompl state to create the mapping from
*/
bool robotStateToOmplState(const arm_navigation_msgs::RobotState &robot_state,
                           const ompl_ros_interface::RobotStateToOmplStateMapping &mapping,
                           ompl::base::State *ompl_state,
                           const bool &fail_if_match_not_found = true);

/**
 * @brief Convert a ROS message robot state to an ompl state.  This function is 
 * intended to help in efficient conversion between ROS messages and OMPL state objects by 
 * providing a mapping that can be cached and used frequently.
 * @param robot_state The robot state message to create the mapping to
 * @param ompl_state The ompl state to create the mapping from
*/
bool robotStateToOmplState(const arm_navigation_msgs::RobotState &robot_state,
                           ompl::base::ScopedState<ompl::base::CompoundStateSpace> &ompl_scoped_state,
                           const bool &fail_if_match_not_found = true);


/**
 * @brief Convert a ROS message joint state to an ompl real vector state.  This function is 
 * intended to help in efficient conversion between ROS messages and OMPL state objects by 
 * providing a mapping that can be cached and used frequently.
 * @param joint_state The joint state message to create the mapping to
 * @param real_vector_state The ompl state to create the mapping from
 * @param mapping The mapping to use for this conversion
*/
bool jointStateToRealVectorState(const sensor_msgs::JointState &joint_state,
                                 const ompl_ros_interface::RobotStateToOmplStateMapping &mapping,
                                 ompl::base::RealVectorStateSpace::StateType &real_vector_state,
                                 const bool &fail_if_match_not_found = true);

/**
 * @brief Convert a SE3StateSpace::StateType to a pose message
 * @param pose - an object of type SE3StateSpace::StateType
 * @param pose_msg - the resultant pose message
 */
void SE3StateSpaceToPoseMsg(const ompl::base::SE3StateSpace::StateType &pose,
                          geometry_msgs::Pose &pose_msg);

/**
 * @brief Convert a SO3StateSpace::StateType to a quaternion message
 * @param pose - an object of type SO3StateSpace::StateType
 * @param quaternion_msg - the resultant quaternion message
 */
void SO3StateSpaceToQuaternionMsg(const ompl::base::SO3StateSpace::StateType &quaternion,
                                geometry_msgs::Quaternion &quaternion_msg);

/**
 * @brief Convert a SO3StateSpace::StateType to a pose message
 * @param pose - an object of type SO3StateSpace::StateType
 * @param pose_msg - the resultant pose message
 */
void SO3StateSpaceToPoseMsg(const ompl::base::SO3StateSpace::StateType &quaternion,
                          geometry_msgs::Pose &pose_msg);

/**
 * @brief Convert a SE2StateSpace::StateType to a pose message
 * @param pose - an object of type SE2StateSpace::StateType
 * @param pose_msg - the resultant pose message
 */
void SE2StateSpaceToPoseMsg(const ompl::base::SE2StateSpace::StateType &pose,
                          geometry_msgs::Pose &pose_msg);

/**
 * @brief Convert a pose message to a SE3StateSpace::StateType
 * @param pose_msg - the input pose message
 * @param pose - the resultant object of type SE3StateSpace::StateType
 */
void poseMsgToSE3StateSpace(const geometry_msgs::Pose &pose_msg,
                          ompl::base::SE3StateSpace::StateType &pose);

/**
 * @brief Convert a quaternion message to a SO3StateSpace::StateType
 * @param pose_msg - the resultant quaternion message
 * @param pose - an object of type SO3StateSpace::StateType
 */
void quaternionMsgToSO3StateSpace(const geometry_msgs::Quaternion &quaternion_msg,
                                ompl::base::SO3StateSpace::StateType &quaternion);


/**
 * @brief Create a mapping from the kinematic state to an ompl state.
 * @param kinematic_state The kinematic state to create the mapping from
 * @param ompl_state The ompl state to create the mapping to
 * @param mapping The resultant mapping
 */
bool getJointStateGroupToOmplStateMapping(planning_models::KinematicState::JointStateGroup* joint_state_group, 
                                          const ompl::base::ScopedState<ompl::base::CompoundStateSpace> &ompl_state,
                                          ompl_ros_interface::KinematicStateToOmplStateMapping &mapping);


/**
 * @brief Create a mapping from the ompl state to a kinematic state.
 * @param ompl_state The ompl state to create the mapping from
 * @param joint_state_group The kinematic state to create the mapping to
 * @param mapping The resultant mapping
 */
bool getOmplStateToJointStateGroupMapping(const ompl::base::ScopedState<ompl::base::CompoundStateSpace> &ompl_state,
                                          planning_models::KinematicState::JointStateGroup* joint_state_group,
                                          ompl_ros_interface::OmplStateToKinematicStateMapping &mapping);

/**
 * @brief Convert a kinematic state to an ompl scoped state given the appropriate mapping
 * @param joint_state_group The kinematic state to convert from
 * @param mapping The given mapping
 * @param ompl_state The ompl scoped state to convert to
 */
bool kinematicStateGroupToOmplState(const planning_models::KinematicState::JointStateGroup* joint_state_group, 
                                    const ompl_ros_interface::KinematicStateToOmplStateMapping &mapping,
                                    ompl::base::ScopedState<ompl::base::CompoundStateSpace> &ompl_state);

/**
 * @brief Convert an ompl scoped state to a kinematic state given the appropriate mapping
 * @param ompl_state The ompl scoped state to convert from
 * @param mapping The given mapping
 * @param joint_state_group The kinematic state to convert to
 */
bool omplStateToKinematicStateGroup(const ompl::base::ScopedState<ompl::base::CompoundStateSpace> &ompl_state,
                                    const ompl_ros_interface::OmplStateToKinematicStateMapping &mapping,
                                    planning_models::KinematicState::JointStateGroup* joint_state_group);

/**
 * @brief Convert an ompl state to a kinematic state given the appropriate mapping
 * @param ompl_state The ompl state to convert to
 * @param mapping The given mapping
 * @param joint_state_group The kinematic state to convert from
 */
bool omplStateToKinematicStateGroup(const ompl::base::State *ompl_state,
                                    const ompl_ros_interface::OmplStateToKinematicStateMapping &mapping,
                                    planning_models::KinematicState::JointStateGroup* joint_state_group);

/**
 * @brief Create a RobotTrajectory message from a joint state group
 * @param joint_state_group The kinematic state to convert from
 * @param robot_trajectory The robot trajectory that was created
 * @return false if any error occured
 */
bool jointStateGroupToRobotTrajectory(planning_models::KinematicState::JointStateGroup* joint_state_group, 
                                      arm_navigation_msgs::RobotTrajectory &robot_trajectory);

/**
 * @brief Convert an ompl path to a RobotTrajectory message
 * @param path The ompl path 
 * @param state_space The state space corresponding to the path
 * @param robot_trajectory The RobotTrajectory message to convert the path to
 * @return false if any error occured
 */
bool omplPathGeometricToRobotTrajectory(const ompl::geometric::PathGeometric &path,
                                        const ompl::base::StateSpacePtr &state_space,
                                        arm_navigation_msgs::RobotTrajectory &robot_trajectory);

/**
 * @brief Convert an ompl path to a RobotTrajectory message
 * @param path The ompl path 
 * @param mapping The mapping from the path to the robot trajectory
 * @param robot_trajectory The RobotTrajectory message to convert the path to
 * @return false if any error occured
 */
bool omplPathGeometricToRobotTrajectory(const ompl::geometric::PathGeometric &path,
                                        const ompl_ros_interface::OmplStateToRobotStateMapping &mapping,
                                        arm_navigation_msgs::RobotTrajectory &robot_trajectory);

/**
 * @brief Get the mapping from an ompl state space to a RobotTrajectory message
 * @param state_space The state space 
 * @param robot_trajectory The RobotTrajectory message to find the mapping for
 * @param mapping The mapping from the state_space to the robot trajectory
 * @return false if any error occured
 */
bool getOmplStateToRobotTrajectoryMapping(const ompl::base::StateSpacePtr &state_space,
                                          const arm_navigation_msgs::RobotTrajectory &robot_trajectory, 
                                          ompl_ros_interface::OmplStateToRobotStateMapping &mapping);

/**
 * @brief Get the mapping from an ompl state space to a JointTrajectory message
 * @param state_space The state space 
 * @param joint_trajectory The JointTrajectory message to find the mapping for
 * @param mapping The mapping from the state_space to the robot trajectory
 * @return false if any error occured
 */
bool getOmplStateToJointTrajectoryMapping(const ompl::base::StateSpacePtr &state_space,
                                          const trajectory_msgs::JointTrajectory &joint_trajectory,
                                          ompl_ros_interface::OmplStateToRobotStateMapping &mapping);

/**
 * @brief Convert a set of joint constraints to an ompl scoped state
 * @param joint_constraints The input joint constraints
 * @param ompl_scoped_state The output scoped state
 * @return false if any error occured
 */
bool jointConstraintsToOmplState(const std::vector<arm_navigation_msgs::JointConstraint> &joint_constraints,
                                 ompl::base::ScopedState<ompl::base::CompoundStateSpace> &ompl_scoped_state);

/**
 * @brief Convert a set of joint constraints to an ompl scoped state
 * @param joint_constraints The input joint constraints
 * @param ompl_scoped_state The output scoped state
 * @return false if any error occured
 */
bool jointConstraintsToOmplState(const std::vector<arm_navigation_msgs::JointConstraint> &joint_constraints,
                                 ompl::base::ScopedState<ompl::base::CompoundStateSpace> &ompl_scoped_state);

/**
 * @brief Convert a set of constraints to an ompl scoped state
 * @param constraints The input joint constraints
 * @param ompl_scoped_state The output scoped state
 * @param fail_if_match_not_found - if true, fail if no matching joint was found for any of the joints specified in the constraints
 * @return false if any error occured
 */
bool constraintsToOmplState(const arm_navigation_msgs::Constraints &constraints,
                            ompl::base::ScopedState<ompl::base::CompoundStateSpace> &ompl_scoped_state,
                            const bool &fail_if_match_not_found = true);

/**
 * @brief Convert from a RobotState to a JointStateGroup
 * @param robot_state - The input RobotState
 * @param robot_state_to_joint_state_group_mapping - The mapping from the robot state to the joint state group
 * @param joint_state_group - The resultant joint state group
 * @return false if any error occured
 */
bool robotStateToJointStateGroup(const arm_navigation_msgs::RobotState &robot_state,
                                 const ompl_ros_interface::RobotStateToKinematicStateMapping &robot_state_to_joint_state_group_mapping,
                                 planning_models::KinematicState::JointStateGroup *joint_state_group);

/**
 * @brief Get the mapping from a RobotState message to a JointModelGroup
 * @param robot_state - The input RobotState
 * @param joint_model_group - The resultant joint model group
 * @param mapping - The mapping from the robot state to the joint model
 * @return false if any error occured
 */
bool getRobotStateToJointModelGroupMapping(const arm_navigation_msgs::RobotState &robot_state,
                                           const planning_models::KinematicModel::JointModelGroup *joint_model_group,
                                           ompl_ros_interface::RobotStateToKinematicStateMapping &mapping);

/**
 * @brief Add a state to the ompl state space
 * @param kinematic_model - the kinematic model to use for getting properties of the particular joint
 * @param joint_name - The joint name to add
 * @param ompl_state_space - The state space to add joints to
 * @return false if any error occured
 */
bool addToOmplStateSpace(const planning_models::KinematicModel* kinematic_model, 
                         const std::string &joint_name,
                         ompl::base::StateSpacePtr &ompl_state_space);

}
#endif
