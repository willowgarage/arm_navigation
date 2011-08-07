/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Sachin Chitta
 */

#include <arm_kinematics_constraint_aware/multi_arm_kinematics_constraint_aware.h>

#include <planning_environment/models/model_utils.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace arm_kinematics_constraint_aware {

MultiArmKinematicsConstraintAware::MultiArmKinematicsConstraintAware(const std::vector<std::string> &group_names, 
                                                                     const std::vector<std::string> &kinematics_solver_names,
                                                                     const std::vector<std::string> &end_effector_link_names):kinematics_loader_("kinematics_base","kinematics::KinematicsBase"),group_names_(group_names),kinematics_solver_names_(kinematics_solver_names),end_effector_link_names_(end_effector_link_names)
{
  planning_environment::CollisionModelsInterface* collision_models_interface = new planning_environment::CollisionModelsInterface("robot_description");
  if(!initialize(group_names,kinematics_solver_names,end_effector_link_names,collision_models_interface))
        throw new MultiArmKinematicsException();
  collision_models_interface_generated_ = true;
}

MultiArmKinematicsConstraintAware::MultiArmKinematicsConstraintAware(const std::vector<std::string> &group_names, 
                                                                     const std::vector<std::string> &kinematics_solver_names,
                                                                     const std::vector<std::string> &end_effector_link_names,
                                                                     planning_environment::CollisionModelsInterface *collision_models_interface):kinematics_loader_("kinematics_base","kinematics::KinematicsBase"),collision_models_interface_generated_(false)
{
  if(!initialize(group_names,kinematics_solver_names,end_effector_link_names,collision_models_interface))
        throw new MultiArmKinematicsException();
}

bool MultiArmKinematicsConstraintAware::initialize(const std::vector<std::string> &group_names, 
                                                   const std::vector<std::string> &kinematics_solver_names,
                                                   const std::vector<std::string> &end_effector_link_names,
                                                   planning_environment::CollisionModelsInterface *collision_models_interface)
{
  group_names_ = group_names;
  kinematics_solver_names_ = kinematics_solver_names;
  end_effector_link_names_ = end_effector_link_names;
  collision_models_interface_ = collision_models_interface;

  srand ( time(NULL) ); // initialize random seed
  if(group_names_.empty())
    return false;
  if(kinematics_solver_names_.size() != group_names_.size())
    return false;
  if(end_effector_link_names_.size() != group_names_.size())
    return false;
  num_groups_ = group_names_.size();
  end_effector_collision_links_.resize(num_groups_);
  kinematics_solvers_.resize(num_groups_);
  bounds_.resize(num_groups_);

  for(unsigned int i=0; i < num_groups_; i++)
  {
    try
	  {
	    kinematics_solvers_[i] = kinematics_loader_.createClassInstance(kinematics_solver_names_[i]);
	  }
    catch(pluginlib::PluginlibException& ex)
	  {
	    ROS_ERROR("The plugin failed to load. Error1: %s", ex.what());    //handle the class failing to load
      return false;
	  }
      
    if(!kinematics_solvers_[i]->initialize(group_names_[i]))
      return false;
    if(i==0)
      base_frame_ = kinematics_solvers_[i]->getBaseFrame();
    else if(kinematics_solvers_[i]->getBaseFrame() != base_frame_)
      return false;

    const planning_models::KinematicModel::JointModelGroup* joint_model_group = collision_models_interface_->getKinematicModel()->getModelGroup(group_names_[i]);
    if(joint_model_group == NULL) 
	  {
	    ROS_WARN_STREAM("No joint group " << group_names_[i]);
      return false;
	  }
    std::vector<std::string> arm_links = joint_model_group->getGroupLinkNames();
    for(unsigned int j=0; j < arm_links.size(); j++)
      arm_links_.push_back(arm_links[j]);
    //    arm_links_.insert(arm_links_.end(),joint_model_group->getGroupLinkNames());
    const planning_models::KinematicModel::LinkModel* end_effector_link = collision_models_interface_->getKinematicModel()->getLinkModel(end_effector_link_names[i]);
    end_effector_collision_links_[i] = collision_models_interface_->getKinematicModel()->getChildLinkModelNames(end_effector_link);

    std::vector<string> joint_names = kinematics_solvers_[i]->getJointNames();
    bounds_[i].resize(joint_names.size());
    for(unsigned int j=0; j < joint_names.size(); j++)
    {
      if(!joint_model_group->hasJointModel(joint_names[i]))
      {
        ROS_ERROR("Could not get joint model for joint %s",joint_names[j].c_str());
        return false;
      }
      const planning_models::KinematicModel::JointModel* joint_model = ((planning_models::KinematicModel::JointModelGroup*) (joint_model_group))->getJointModel(joint_names[j]);
      joint_model->getVariableBounds(joint_names[j],bounds_[i][j]);
    }
  }
  return true;
}

bool MultiArmKinematicsConstraintAware::setup(const arm_navigation_msgs::PlanningScene& planning_scene,
                                              const arm_navigation_msgs::OrderedCollisionOperations &collision_operations)
{
  original_state_ = collision_models_interface_->setPlanningScene(planning_scene);
  for(unsigned int i=0; i < num_groups_; i++)
    collision_models_interface_->disableCollisionsForNonUpdatedLinks(group_names_[i]);    
  return true;
}

bool MultiArmKinematicsConstraintAware::clear()
{
  if(original_state_ != NULL)
  {
    collision_models_interface_->revertPlanningScene(original_state_);
    original_state_ = NULL;
  }
  return true;
}

std::vector<std::string> MultiArmKinematicsConstraintAware::getArmNames()
{
  return group_names_;  
}

std::vector<std::string> MultiArmKinematicsConstraintAware::getEndEffectorNames()
{
  return end_effector_link_names_;  
}

std::vector<std::string> MultiArmKinematicsConstraintAware::getJointNames()
{
  std::vector<std::string> joint_names;
  for(unsigned int i=0; i < num_groups_; i++)
  {
    std::vector<std::string> jn = kinematics_solvers_[i]->getJointNames();
    for(unsigned int j=0; j < jn.size(); j++)
      joint_names.push_back(jn[j]);
  }
  return joint_names;  
}

bool MultiArmKinematicsConstraintAware::checkRequest(const std::vector<geometry_msgs::Pose> &poses,
                                                     const std::vector<std::vector<double> > &seed_states,
                                                     std::vector<std::vector<double> > &solutions,
                                                     std::vector<int> &error_codes)
{
  ros::Time start_time = ros::Time::now();
  if(poses.size() != num_groups_)
  {
    ROS_ERROR("Num poses: %d does not match number of groups: %d",(int)poses.size(),(int)num_groups_);
    return false;
  }
  if(seed_states.size() != num_groups_)
  {
    ROS_ERROR("Num seed state vectors: %d does not match number of groups: %d",(int)seed_states.size(),(int)num_groups_);
    return false;
  }
  if(solutions.size() != num_groups_)
  {
    ROS_ERROR("Num solutions vectors: %d does not match number of groups: %d",(int)solutions.size(),(int)num_groups_);
    return false;
  }
  if(error_codes.size() != num_groups_)
  {
    ROS_ERROR("Num error codes: %d does not match number of groups: %d",(int)error_codes.size(),(int)num_groups_);
    return false;
  }
  return true;
}

bool MultiArmKinematicsConstraintAware::getPositionIK(const std::vector<geometry_msgs::Pose> &poses,
                                                      const std::vector<std::vector<double> > &seed_states,
                                                      double &timeout,
                                                      std::vector<std::vector<double> > &solutions,
                                                      std::vector<int> &error_codes)
{
  ros::Time start_time = ros::Time::now();

  if(!checkRequest(poses,seed_states,solutions,error_codes))
  {
    ROS_ERROR("Request is malformed");
    timeout -= (ros::Time::now()-start_time).toSec();
    return false;
  }

  for(unsigned int i=0; i < num_groups_; i++)
  {
    bool ik_valid = kinematics_solvers_[i]->getPositionIK(poses[i],
                                                          seed_states[i],
                                                          solutions[i],
                                                          error_codes[i]);
    if(!ik_valid)
	  {
	    ROS_ERROR("Could not find IK solution for Arm %s",group_names_[i].c_str());
	    timeout -= (ros::Time::now()-start_time).toSec();
	    return false;
	  }
  }
  timeout -= (ros::Time::now()-start_time).toSec();  
  return true;
}

bool MultiArmKinematicsConstraintAware::checkJointStates(const std::vector<std::vector<double> > &solutions,
                                                         const arm_navigation_msgs::Constraints &constraints,
                                                         double &timeout,
                                                         int &error_code)
{
  ros::Time start_time = ros::Time::now();
  if(solutions.size() != num_groups_)
  {
    ROS_ERROR("Number of solutions %d does not match number of groups: %d",(int)solutions.size(),(int)num_groups_);
    timeout -= (ros::Time::now()-start_time).toSec();  
    return false;
  }
  std::map<std::string, double> joint_values;

  for(unsigned int i=0; i < num_groups_; i++)
    for(unsigned int j=0; j < kinematics_solvers_[i]->getJointNames().size(); j++)
      joint_values[(kinematics_solvers_[i]->getJointNames())[j]] = solutions[i][j];

  collision_models_interface_->getPlanningSceneState()->setKinematicState(joint_values);
  if(collision_models_interface_->getPlanningSceneState() == NULL) {
    ROS_INFO_STREAM("Messed up");
  }
  if(collision_models_interface_->isKinematicStateInCollision(*(collision_models_interface_->getPlanningSceneState()))) 
  {
    error_code = kinematics::STATE_IN_COLLISION;
    timeout -= (ros::Time::now()-start_time).toSec();  
    return false;
  } 
  else 
  {
    error_code = kinematics::SUCCESS;
  }
  if(!planning_environment::doesKinematicStateObeyConstraints(*(collision_models_interface_->getPlanningSceneState()), 
                                                              constraints)) 
  {
    error_code = kinematics::GOAL_CONSTRAINTS_VIOLATED;
    timeout -= (ros::Time::now()-start_time).toSec();  
    return false;
  }
  timeout -= (ros::Time::now()-start_time).toSec();  
  return true;
}

bool MultiArmKinematicsConstraintAware::checkEndEffectorStates(const std::vector<geometry_msgs::Pose> &poses,
                                                               double &timeout,
                                                               std::vector<int> &error_codes)
{
  ros::Time start_time = ros::Time::now();
  if(poses.size() != num_groups_)
  {
    ROS_ERROR("Number of solutions %d does not match number of groups: %d",(int)poses.size(),(int)num_groups_);
    timeout -= (ros::Time::now()-start_time).toSec();  
    return false;
  }
  if(error_codes.size() != num_groups_)
  {
    ROS_ERROR("Number of error codes %d does not match number of groups: %d",(int)error_codes.size(),(int)num_groups_);
    timeout -= (ros::Time::now()-start_time).toSec();  
    return false;
  }
  //disabling all collision for arm links
  collision_space::EnvironmentModel::AllowedCollisionMatrix save_acm = collision_models_interface_->getCurrentAllowedCollisionMatrix();
  collision_space::EnvironmentModel::AllowedCollisionMatrix acm = save_acm;
  for(unsigned int i = 0; i < arm_links_.size(); i++) 
  {
    acm.changeEntry(arm_links_[i], true);
  }
  collision_models_interface_->setAlteredAllowedCollisionMatrix(acm);

  for(unsigned int i=0; i < num_groups_; i++)
  {
    btTransform transform;
    tf::poseMsgToTF(poses[i],transform);
    ROS_INFO("Checking link %s",end_effector_link_names_[i].c_str());
    if(!collision_models_interface_->getPlanningSceneState()->updateKinematicStateWithLinkAt(end_effector_link_names_[i], transform))
    {
      ROS_ERROR("Could not transform link state for %s",end_effector_link_names_[i].c_str());
      return false;
    }
    if(collision_models_interface_->isKinematicStateInCollision(*(collision_models_interface_->getPlanningSceneState()))) 
    {
      error_codes[i] = kinematics::IK_LINK_IN_COLLISION;
      ROS_DEBUG_STREAM("Initial pose check failing");
      sendEndEffectorPoseToVisualizer(collision_models_interface_->getPlanningSceneState(), false);
      collision_models_interface_->setAlteredAllowedCollisionMatrix(save_acm);
      timeout -= (ros::Time::now()-start_time).toSec();  
      return false;
    }
    else
      error_codes[i] = kinematics::SUCCESS;
  }
  collision_models_interface_->setAlteredAllowedCollisionMatrix(save_acm);
  timeout -= (ros::Time::now()-start_time).toSec();  
  return true;
}

bool MultiArmKinematicsConstraintAware::searchPositionIK(const std::vector<geometry_msgs::Pose> &poses,
                                                         const std::vector<std::vector<double> > &seed_states,
                                                         double &timeout,
                                                         std::vector<std::vector<double> > &solutions,
                                                         std::vector<int> &error_codes)
{
  ros::Time start_time = ros::Time::now();
  if(!checkRequest(poses,seed_states,solutions,error_codes))
  {
    ROS_ERROR("Request is malformed");
    timeout -= (ros::Time::now()-start_time).toSec();
    return false;
  }
  for(unsigned int i=0; i < num_groups_; i++)
  {
    bool ik_valid = kinematics_solvers_[i]->searchPositionIK(poses[i],
                                                             seed_states[i],
                                                             timeout,
                                                             solutions[i],
                                                             error_codes[i]);
    timeout -= (ros::Time::now()-start_time).toSec();
    if(!ik_valid)
	  {
	    ROS_ERROR("Could not find IK solution for Arm %s",group_names_[i].c_str());
	    return false;
	  }
  }
  return true;
}

void MultiArmKinematicsConstraintAware::generateRandomState(std::vector<std::vector<double> > &state)
{
  state.resize(num_groups_);
  for(unsigned int i = 0; i < num_groups_; i++)
  {
    state[i].resize(kinematics_solvers_[i]->getJointNames().size());
    for(unsigned int j=0; j < kinematics_solvers_[i]->getJointNames().size(); j++)
      state[i][j] = generateRandomNumber(bounds_[i][j].first,bounds_[i][j].second);
  }
}

double MultiArmKinematicsConstraintAware::generateRandomNumber(const double &min, const double &max)
{
  int rand_num = rand()%100+1;
  double result = min + (double)((max-min)*rand_num)/101.0;
  return result;
}

bool MultiArmKinematicsConstraintAware::searchConstraintAwarePositionIK(const std::vector<geometry_msgs::Pose> &poses,
                                                                        const std::vector<std::vector<double> > &seed_states,
                                                                        double &timeout,
                                                                        std::vector<std::vector<double> > &solutions,
                                                                        std::vector<int> &error_codes,
                                                                        const double &max_distance)
{
  arm_navigation_msgs::Constraints constraints;
  return searchConstraintAwarePositionIK(poses,seed_states,constraints,timeout,solutions,error_codes);
}

bool MultiArmKinematicsConstraintAware::searchConstraintAwarePositionIK(const std::vector<geometry_msgs::Pose> &poses,
                                                                        double &timeout,
                                                                        std::vector<std::vector<double> > &solutions,
                                                                        std::vector<int> &error_codes,
                                                                        const double &max_distance)
{
  std::vector<std::vector<double> > seed_states;
  generateRandomState(seed_states);
  return searchConstraintAwarePositionIK(poses,seed_states,timeout,solutions,error_codes);
}

bool MultiArmKinematicsConstraintAware::searchConstraintAwarePositionIK(const std::vector<geometry_msgs::Pose> &poses,
                                                                        const arm_navigation_msgs::Constraints &constraints,
                                                                        double &timeout,
                                                                        std::vector<std::vector<double> > &solutions,
                                                                        std::vector<int> &error_codes,
                                                                        const double &max_distance)
{
  std::vector<std::vector<double> > seed_states;
  generateRandomState(seed_states);
  return searchConstraintAwarePositionIK(poses,seed_states,constraints,timeout,solutions,error_codes);
}

bool MultiArmKinematicsConstraintAware::searchConstraintAwarePositionIK(const std::vector<geometry_msgs::Pose> &poses,
                                                                        const std::vector<std::vector<double> > &seed_states,
                                                                        const arm_navigation_msgs::Constraints &constraints,
                                                                        double &timeout,
                                                                        std::vector<std::vector<double> > &solutions,
                                                                        std::vector<int> &error_codes,
                                                                        const double &max_distance)
{
  arm_navigation_msgs::Constraints empty_constraints;
  ros::Time start_time = ros::Time::now();
  error_codes.resize(num_groups_);
  if(!checkRequest(poses,seed_states,solutions,error_codes))
  {
    ROS_ERROR("Request is malformed");
    timeout -= (ros::Time::now()-start_time).toSec();
    return false;
  }
  if(!checkEndEffectorStates(poses,timeout,error_codes))
    return false;

  std::vector<std::vector<double> > seed_states_random = seed_states;
  while(timeout >= 0.0)
  {
    bool ik_valid = true;
    for(unsigned int i=0; i < num_groups_; i++)
    {
      ik_valid = kinematics_solvers_[i]->searchPositionIK(poses[i],
                                                          seed_states[i],
                                                          timeout,
                                                          max_distance,
                                                          solutions[i],
                                                          error_codes[i]);
      timeout -= (ros::Time::now()-start_time).toSec();
      if(!ik_valid)
      {
        ROS_DEBUG("Could not find IK solution for arm %s",group_names_[i].c_str());
        break;
      }
    }
    int error_code;
    if(ik_valid)
      if(checkJointStates(solutions,empty_constraints,timeout,error_code))
        return true;
    generateRandomState(seed_states_random);
  }
  return false;
}

void MultiArmKinematicsConstraintAware::sendEndEffectorPoseToVisualizer(const planning_models::KinematicState* state, bool valid) 
{
  boost::shared_ptr<urdf::Model> robot_model = collision_models_interface_->getParsedDescription();
  visualization_msgs::MarkerArray hand_array;
  unsigned int id = 0;
  for(unsigned int i = 0; i < end_effector_collision_links_.size(); i++) 
    for(unsigned int j=0; j < end_effector_collision_links_[i].size(); j++)
    {
      boost::shared_ptr<const urdf::Link> urdf_link = robot_model->getLink(end_effector_collision_links_[i][j]);
      if(urdf_link == NULL) {
        ROS_DEBUG_STREAM("No entry in urdf for link " << end_effector_collision_links_[i][j]);
        continue;
      }
      if(!urdf_link->collision) {
        continue;
      }
      const urdf::Geometry *geom = urdf_link->collision->geometry.get();
      if(!geom) {
        ROS_DEBUG_STREAM("No collision geometry for link " << end_effector_collision_links_[i][j]);
        continue;
      }
      const urdf::Mesh *mesh = dynamic_cast<const urdf::Mesh*>(geom);
      if(mesh) {
        if (!mesh->filename.empty()) {
          planning_models::KinematicState::LinkState* ls = state->getLinkState(end_effector_collision_links_[i][j]);
          visualization_msgs::Marker mark;
          mark.header.frame_id = collision_models_interface_->getWorldFrameId();
          mark.header.stamp = ros::Time::now();
          mark.id = id++;
          if(!valid) {
            mark.ns = "initial_pose_collision";
          } else {
            mark.ns = "initial_pose_ok";
          }
          mark.type = mark.MESH_RESOURCE;
          mark.scale.x = 1.0;
          mark.scale.y = 1.0;
          mark.scale.z = 1.0;
          if(!valid) {
            mark.color.r = 1.0;
          } else {
            mark.color.g = 1.0;
          }
          mark.color.a = .8;
          mark.pose.position.x = ls->getGlobalCollisionBodyTransform().getOrigin().x();
          mark.pose.position.y = ls->getGlobalCollisionBodyTransform().getOrigin().y();
          mark.pose.position.z = ls->getGlobalCollisionBodyTransform().getOrigin().z();
          mark.pose.orientation.x = ls->getGlobalCollisionBodyTransform().getRotation().x();
          mark.pose.orientation.y = ls->getGlobalCollisionBodyTransform().getRotation().y();
          mark.pose.orientation.z = ls->getGlobalCollisionBodyTransform().getRotation().z();
          mark.pose.orientation.w = ls->getGlobalCollisionBodyTransform().getRotation().w();
          mark.mesh_resource = mesh->filename;
          hand_array.markers.push_back(mark);
        }
      }
    }
  //vis_marker_array_publisher_.publish(hand_array);
}

} // namespace

