//Software License Agreement (BSD License)

//Copyright (c) 2011, Willow Garage, Inc.
//All rights reserved.

//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:

// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of Willow Garage, Inc. nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.

#include <arm_kinematics_constraint_aware/arm_kinematics_solver_constraint_aware.h>
#include <arm_kinematics_constraint_aware/arm_kinematics_constraint_aware_utils.h>
#include <planning_environment/models/model_utils.h>

namespace arm_kinematics_constraint_aware 
{

ArmKinematicsSolverConstraintAware::ArmKinematicsSolverConstraintAware(kinematics::KinematicsBase* solver,
                                                                       planning_environment::CollisionModels* cm,
                                                                       const std::string& group_name) :
  cm_(cm),
  active_(false),
  kinematics_solver_(solver),
  group_name_(group_name)
{
  if(group_name.empty()) {
    ROS_INFO_STREAM("Must have non-empty group");
    return;
  }
  
  const planning_models::KinematicModel::JointModelGroup* joint_model_group = cm_->getKinematicModel()->getModelGroup(group_name_);
  if(joint_model_group == NULL) {
    ROS_WARN_STREAM("No joint group " << group_name_);
    return;
  }
  const planning_models::KinematicModel::GroupConfig& config =
    cm->getKinematicModel()->getJointModelGroupConfigMap().at(group_name_);
  if(config.base_link_.empty() || config.tip_link_.empty()) {
    ROS_WARN_STREAM("Group does not have base/tip config definition, can't solve ik");
    return;
  }

  base_name_ = config.base_link_;
  tip_name_ = config.tip_link_;

  const planning_models::KinematicModel::LinkModel* end_effector_link = cm->getKinematicModel()->getLinkModel(tip_name_);
  end_effector_collision_links_ = cm->getKinematicModel()->getChildLinkModelNames(end_effector_link);

  if(kinematics_solver_->initialize(group_name_,
                                    base_name_,
                                    tip_name_,
                                    .025)) {
  } else {
    ROS_INFO_STREAM("Initialize is failing for " << group_name);
    return;
  }

  active_ = true;
}

bool ArmKinematicsSolverConstraintAware::getPositionFK(const planning_models::KinematicState* robot_state,
                                                       const std::vector<std::string>& link_names,
                                                       std::vector<geometry_msgs::Pose> &poses)
{
  poses.resize(link_names.size());

  for(unsigned int i = 0; i < link_names.size(); i++) {
    const planning_models::KinematicState::LinkState* ls = robot_state->getLinkState(link_names[i]);
    if(ls == NULL) return false;
    tf::poseTFToMsg(ls->getGlobalLinkTransform(), poses[i]);
  }
  return true;
}
bool ArmKinematicsSolverConstraintAware::getPositionIK(const geometry_msgs::Pose &pose,
                                                       const planning_models::KinematicState* robot_state,
                                                       sensor_msgs::JointState& solution,
                                                       arm_navigation_msgs::ArmNavigationErrorCodes& error_code)
{
  std::map<std::string, double> seed_state_map;
  robot_state->getKinematicStateValues(seed_state_map);
  std::vector<double> seed_state_vector(kinematics_solver_->getJointNames().size());
  for(unsigned int i = 0; i < kinematics_solver_->getJointNames().size(); i++) {
    seed_state_vector[i] = seed_state_map[kinematics_solver_->getJointNames()[i]];
  } 

  std::vector<double> sol;
  int kinematics_error_code;
  bool ik_valid = kinematics_solver_->getPositionIK(pose,
                                                    seed_state_vector,
                                                    sol,
                                                    kinematics_error_code);
  if(ik_valid) {
    solution.name = kinematics_solver_->getJointNames();
    solution.position = sol;
    error_code.val = error_code.SUCCESS;
  } else {
    solution.name.clear();
    solution.position.clear();
    error_code = kinematicsErrorCodeToMotionPlanningErrorCode(kinematics_error_code);
  }
  return ik_valid;
}

bool ArmKinematicsSolverConstraintAware::findConstraintAwareSolution(const geometry_msgs::Pose& pose,
                                                                     const arm_navigation_msgs::Constraints& constraints,
                                                                     planning_models::KinematicState* robot_state,
                                                                     sensor_msgs::JointState& solution,
                                                                     arm_navigation_msgs::ArmNavigationErrorCodes& error_code, 
                                                                     const bool& do_initial_pose_check)
{
  do_initial_pose_check_ = do_initial_pose_check;
  constraints_ = constraints;
  state_ = robot_state;
  
  std::map<std::string, double> seed_state_map;
  robot_state->getKinematicStateValues(seed_state_map);
  
  std::vector<double> seed_state_vector(kinematics_solver_->getJointNames().size());
  for(unsigned int i = 0; i < kinematics_solver_->getJointNames().size(); i++) {
    seed_state_vector[i] = seed_state_map[kinematics_solver_->getJointNames()[i]];
  } 
  
  std::vector<double> sol;
  int kinematics_error_code;
  bool ik_valid = kinematics_solver_->searchPositionIK(pose,
                                                      seed_state_vector,
                                                      1.0,
                                                      sol,
                                                      boost::bind(&ArmKinematicsSolverConstraintAware::initialPoseCheck, this, _1, _2, _3),
                                                      boost::bind(&ArmKinematicsSolverConstraintAware::collisionCheck, this, _1, _2, _3),
                                                      kinematics_error_code);
  if(ik_valid) {
    solution.name = kinematics_solver_->getJointNames();
    solution.position = sol;
    error_code.val = error_code.SUCCESS;
  } else {
    solution.name.clear();
    solution.position.clear();
    error_code = kinematicsErrorCodeToMotionPlanningErrorCode(kinematics_error_code);
  }
  return ik_valid;
}

bool ArmKinematicsSolverConstraintAware::findConsistentConstraintAwareSolution(const geometry_msgs::Pose& pose,
                                                                               const arm_navigation_msgs::Constraints& constraints,
                                                                               planning_models::KinematicState* robot_state,
                                                                               sensor_msgs::JointState& solution,
                                                                               arm_navigation_msgs::ArmNavigationErrorCodes& error_code, 
                                                                               const unsigned int& redundancy,
                                                                               const double& max_consistency,
                                                                               const bool& do_initial_pose_check)
{
  do_initial_pose_check_ = do_initial_pose_check;
  constraints_ = constraints;
  state_ = robot_state;
  
  std::map<std::string, double> seed_state_map;
  robot_state->getKinematicStateValues(seed_state_map);
  
  std::vector<double> seed_state_vector(kinematics_solver_->getJointNames().size());
  for(unsigned int i = 0; i < kinematics_solver_->getJointNames().size(); i++) {
    seed_state_vector[i] = seed_state_map[kinematics_solver_->getJointNames()[i]];
  } 
  
  std::vector<double> sol;
  int kinematics_error_code;
  bool ik_valid = kinematics_solver_->searchPositionIK(pose,
                                                       seed_state_vector,
                                                       1.0,
                                                       redundancy,
                                                       max_consistency,
                                                       sol,
                                                       boost::bind(&ArmKinematicsSolverConstraintAware::initialPoseCheck, this, _1, _2, _3),
                                                       boost::bind(&ArmKinematicsSolverConstraintAware::collisionCheck, this, _1, _2, _3),
                                                       kinematics_error_code);
  if(ik_valid) {
    solution.name = kinematics_solver_->getJointNames();
    solution.position = sol;
    error_code.val = error_code.SUCCESS;
  } else {
    solution.name.clear();
    solution.position.clear();
    error_code = kinematicsErrorCodeToMotionPlanningErrorCode(kinematics_error_code);
  }
  return ik_valid;
}


void ArmKinematicsSolverConstraintAware::collisionCheck(const geometry_msgs::Pose &ik_pose,
                                                        const std::vector<double> &ik_solution,
                                                        int &error_code)
{
  std::map<std::string, double> joint_values;
  for(unsigned int i=0; i < kinematics_solver_->getJointNames().size(); i++) {
    joint_values[kinematics_solver_->getJointNames()[i]] = ik_solution[i];
  }
  
  state_->setKinematicState(joint_values);
  error_code = kinematics::SUCCESS;
  if(cm_->isKinematicStateInCollision(*state_)) {
    error_code = kinematics::STATE_IN_COLLISION;
  } else if(!planning_environment::doesKinematicStateObeyConstraints(*state_, 
                                                                     constraints_)) {
    error_code = kinematics::GOAL_CONSTRAINTS_VIOLATED;
  }
}

void ArmKinematicsSolverConstraintAware::initialPoseCheck(const geometry_msgs::Pose &ik_pose,
                                                          const std::vector<double> &ik_solution,
                                                          int &error_code)
{
  if(!do_initial_pose_check_) {
    error_code = kinematics::SUCCESS;
    return;
  }
  std::string kinematic_frame_id = kinematics_solver_->getBaseName();
  std::string planning_frame_id = cm_->getWorldFrameId();
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.pose = ik_pose;
  pose_stamped.header.stamp = ros::Time::now();
  pose_stamped.header.frame_id = kinematic_frame_id;
  if(!cm_->convertPoseGivenWorldTransform(*state_,
                                          planning_frame_id,
                                          pose_stamped.header,
                                          pose_stamped.pose,
                                          pose_stamped)) {
    ROS_ERROR_STREAM("Cannot transform from " << pose_stamped.header.frame_id << " to " << planning_frame_id);
  }
  //disabling all collision for arm links
  collision_space::EnvironmentModel::AllowedCollisionMatrix save_acm = cm_->getCurrentAllowedCollisionMatrix();
  collision_space::EnvironmentModel::AllowedCollisionMatrix acm = save_acm;
  for(unsigned int i = 0; i < kinematics_solver_->getLinkNames().size(); i++) {
    acm.changeEntry(kinematics_solver_->getLinkNames()[i], true);
  }
  cm_->setAlteredAllowedCollisionMatrix(acm);
  
  tf::Transform transform;
  tf::poseMsgToTF(pose_stamped.pose,transform);
  if(!state_->hasLinkState(tip_name_)) {
    error_code = kinematics::INVALID_LINK_NAME;
    return;
  }
  state_->updateKinematicStateWithLinkAt(tip_name_, transform);
  if(cm_->isKinematicStateInCollision(*state_)) {
    error_code = kinematics::IK_LINK_IN_COLLISION;
    ROS_DEBUG_STREAM("Initial pose check failing");
  } else {
    error_code = kinematics::SUCCESS;
  }
  cm_->setAlteredAllowedCollisionMatrix(save_acm);
}

bool ArmKinematicsSolverConstraintAware::interpolateIKDirectional(const geometry_msgs::Pose& start_pose,
                                                                  const tf::Vector3& direction,
                                                                  const double& distance,
                                                                  const arm_navigation_msgs::Constraints& constraints,
                                                                  planning_models::KinematicState* robot_state,
                                                                  arm_navigation_msgs::ArmNavigationErrorCodes& error_code, 
                                                                  trajectory_msgs::JointTrajectory& traj,
                                                                  const unsigned int& redundancy,
                                                                  const double& max_consistency,
                                                                  const bool& reverse, 
                                                                  const bool& premultiply,
                                                                  const unsigned int& num_points,
                                                                  const ros::Duration& total_dur,
                                                                  const bool& do_initial_pose_check)
{
  trajectory_msgs::JointTrajectory ret_traj;
  ret_traj.joint_names = kinematics_solver_->getJointNames();
  ret_traj.points.resize(num_points+1);
    
  tf::Transform first_pose;
  tf::poseMsgToTF(start_pose, first_pose);

  unsigned int index;
  unsigned int val;
  if(reverse) {
    val = 0;
    index = num_points;
  } else {
    val = 0;
    index = 0;
  }

  while(1) {
    //assumes that the axis is aligned
    tf::Transform trans(tf::Quaternion(0,0,0,1.0), direction*(int)val*fabs(distance/(num_points*1.0)));
    tf::Transform mult_trans;
    if(premultiply) {
      mult_trans = trans*first_pose;
    } else {
      mult_trans = first_pose*trans;
    }
    geometry_msgs::Pose trans_pose;
    tf::poseTFToMsg(mult_trans, trans_pose);

    ROS_DEBUG_STREAM("Pose is " << trans_pose.position.x << " "
                     << trans_pose.position.y << " "
                     << trans_pose.position.z << " "
                     << trans_pose.orientation.x << " "
                     << trans_pose.orientation.y << " "
                     << trans_pose.orientation.z << " "
                     << trans_pose.orientation.w);

    sensor_msgs::JointState solution;
    arm_navigation_msgs::ArmNavigationErrorCodes temp_error_code;
    if(findConsistentConstraintAwareSolution(trans_pose,
                                             constraints,
                                             robot_state,
                                             solution,
                                             temp_error_code,
                                             redundancy,
                                             max_consistency,
                                             do_initial_pose_check)) {
      ret_traj.points[index].positions = solution.position;
      ROS_DEBUG_STREAM("Setting point " << index << " to " <<
                       solution.position[0] << " " <<
                       solution.position[1] << " " <<
                       solution.position[2] << " " <<
                       solution.position[3] << " " <<
                       solution.position[4] << " " <<
                       solution.position[5] << " " <<
                       solution.position[6]); 
      ret_traj.points[index].time_from_start = ros::Duration((index*1.0)*total_dur.toSec()/(num_points*1.0));
    } else {
      return false;
    }
    if(reverse) {
      val++;
      if(index == 0) {
        break;
      }
      index--;
    } else {
      val++;
      index++;
      if(index > num_points) {
        break;
      }
    }
  }
  checkForWraparound(ret_traj);
  traj = ret_traj;
  return true;
}

void ArmKinematicsSolverConstraintAware::checkForWraparound(const trajectory_msgs::JointTrajectory& joint_trajectory) {

  std::vector<unsigned int> checks;
  for(unsigned int i = 0; i < joint_trajectory.joint_names.size(); i++) {
    const planning_models::KinematicModel::RevoluteJointModel* rev 
      = dynamic_cast<const planning_models::KinematicModel::RevoluteJointModel*>(cm_->getKinematicModel()->getJointModel(joint_trajectory.joint_names[i]));
    if(rev != NULL && rev->continuous_) {
      checks.push_back(i);
    }
  }
  for(unsigned int i = 1; i < joint_trajectory.points.size(); i++) {
    for(unsigned int j = 0; j < checks.size(); j++) {
      double last_val = joint_trajectory.points[i-1].positions[checks[j]];
      double cur_val = joint_trajectory.points[i].positions[checks[j]];
      if((last_val < (-2.0*M_PI+.04) && cur_val > (2*M_PI-.04)) ||
         (last_val > (2.0*M_PI-.04) && cur_val < (-2*M_PI+.04))) {
        ROS_ERROR_STREAM("Wrap around problem point " << i << " last val " << last_val << " cur val " << cur_val);
      }
    }
  }
}

}


