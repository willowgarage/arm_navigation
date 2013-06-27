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
 * Author: Sachin Chitta, E. Gil Jones
 */

#include <arm_kinematics_constraint_aware/arm_kinematics_constraint_aware.h>

#include <planning_environment/models/model_utils.h>
#include <sensor_msgs/JointState.h>
#include <kinematics_msgs/utils.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace arm_kinematics_constraint_aware {

static const std::string IK_WITH_COLLISION_SERVICE = "get_constraint_aware_ik";
static const std::string IK_INFO_SERVICE = "get_ik_solver_info";
static const std::string FK_INFO_SERVICE = "get_fk_solver_info";
static const std::string IK_SERVICE = "get_ik";
static const std::string FK_SERVICE = "get_fk";
static const double IK_DEFAULT_TIMEOUT = 10.0;

ArmKinematicsConstraintAware::ArmKinematicsConstraintAware(): kinematics_loader_("kinematics_base","kinematics::KinematicsBase"),node_handle_("~")
{
  std::string group_name, kinematics_solver_name;
  node_handle_.param<bool>("visualize_solution",visualize_solution_,true);
  node_handle_.param<std::string>("group", group_, std::string());
  node_handle_.param<std::string>("kinematics_solver",kinematics_solver_name," ");
  ROS_INFO("Using kinematics solver name: %s",kinematics_solver_name.c_str());
  if (group_.empty())
  {
    ROS_ERROR("No 'group' parameter specified. Without the name of the group of joints to monitor, node cannot compute collision aware inverse kinematics");
    active_ = false;
    return;
  }

  collision_models_interface_ = new planning_environment::CollisionModelsInterface("robot_description");

  kinematics::KinematicsBase* kinematics_solver = NULL;
  try
  {
    kinematics_solver = kinematics_loader_.createClassInstance(kinematics_solver_name);
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load. Error1: %s", ex.what());    //handle the class failing to load
    return;
  }

  solver_ = new ArmKinematicsSolverConstraintAware(kinematics_solver,
                                                   collision_models_interface_, 
                                                   group_);

  active_ = solver_->isActive();

  if(active_) {
    getChainInfoFromRobotModel(*collision_models_interface_->getParsedDescription(),
                               solver_->getBaseName(),
                               solver_->getTipName(),
                               chain_info_);
  }

  advertiseBaseKinematicsServices();
  advertiseConstraintIKService();
}

void ArmKinematicsConstraintAware::advertiseBaseKinematicsServices()
{
  ik_service_ = node_handle_.advertiseService(IK_SERVICE,&ArmKinematicsConstraintAware::getPositionIK,this);
  fk_service_ = node_handle_.advertiseService(FK_SERVICE,&ArmKinematicsConstraintAware::getPositionFK,this);
  ik_solver_info_service_ = node_handle_.advertiseService(IK_INFO_SERVICE,&ArmKinematicsConstraintAware::getIKSolverInfo,this);
  fk_solver_info_service_ = node_handle_.advertiseService(FK_INFO_SERVICE,&ArmKinematicsConstraintAware::getFKSolverInfo,this);
}

void ArmKinematicsConstraintAware::advertiseConstraintIKService()
{
  ik_collision_service_ = node_handle_.advertiseService(IK_WITH_COLLISION_SERVICE,&ArmKinematicsConstraintAware::getConstraintAwarePositionIK,this);
  display_trajectory_publisher_ = root_handle_.advertise<arm_navigation_msgs::DisplayTrajectory>("ik_solution_display", 1);
}

bool ArmKinematicsConstraintAware::isReady(arm_navigation_msgs::ArmNavigationErrorCodes &error_code)
{
  if(!active_)
  {
    ROS_ERROR("IK service is not ready");
    return false;
  }
  if(!collision_models_interface_->isPlanningSceneSet()) {
    ROS_WARN("Planning scene not set");
    error_code.val = error_code.COLLISION_CHECKING_UNAVAILABLE;
    return false;
  } 
  error_code.val = error_code.SUCCESS;
  return true;
}

bool ArmKinematicsConstraintAware::getConstraintAwarePositionIK(kinematics_msgs::GetConstraintAwarePositionIK::Request &request_in,
                                                                kinematics_msgs::GetConstraintAwarePositionIK::Response &response)
{
  if(!isReady(response.error_code))
    return true;

  if(!checkConstraintAwareIKService(request_in,response,chain_info_))
  {
    ROS_ERROR("IK service request is malformed");
    return true;
  }

  collision_models_interface_->disableCollisionsForNonUpdatedLinks(group_);


  ros::Time start_time = ros::Time::now();
  ROS_DEBUG("Received IK request is in the frame: %s",request_in.ik_request.pose_stamped.header.frame_id.c_str());

  geometry_msgs::PoseStamped pose_msg_in = request_in.ik_request.pose_stamped;
  ROS_DEBUG_STREAM("Before Pose is " << pose_msg_in.pose.position.x << " " << pose_msg_in.pose.position.y << " " << pose_msg_in.pose.position.z);
  geometry_msgs::PoseStamped pose_msg_out;
  planning_environment::setRobotStateAndComputeTransforms(request_in.ik_request.robot_state, *collision_models_interface_->getPlanningSceneState());
  
  if(!collision_models_interface_->convertPoseGivenWorldTransform(*collision_models_interface_->getPlanningSceneState(),
                                                                  solver_->getBaseName(),
                                                                  pose_msg_in.header,
                                                                  pose_msg_in.pose,
                                                                  pose_msg_out)) {
    response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
    return true;
  }  
  ROS_DEBUG_STREAM("Pose is " << pose_msg_out.pose.position.x << " " << pose_msg_out.pose.position.y << " " << pose_msg_out.pose.position.z);

  ros::Time ik_solver_time = ros::Time::now();
  bool ik_valid = solver_->findConstraintAwareSolution(pose_msg_out.pose,
                                                       request_in.constraints,
                                                       collision_models_interface_->getPlanningSceneState(),
                                                       response.solution.joint_state,
                                                       response.error_code,
                                                       true);
  if(ik_valid)
  {
    response.error_code.val = response.error_code.SUCCESS;
    return true;
  }
  else
  {
    ROS_DEBUG("A collision aware ik solution could not be found");
    if(response.error_code.val != response.error_code.IK_LINK_IN_COLLISION) 
    {
      //sendEndEffectorPose(collision_models_interface_->getPlanningSceneState(),true);
    }
    return true;
  }
}

void ArmKinematicsConstraintAware::sendEndEffectorPose(const planning_models::KinematicState* state, bool valid) {
  boost::shared_ptr<urdf::Model> robot_model = collision_models_interface_->getParsedDescription();
  visualization_msgs::MarkerArray hand_array;
  unsigned int id = 0;
  for(unsigned int i = 0; i < solver_->getLinkNames().size(); i++) {
    boost::shared_ptr<const urdf::Link> urdf_link = robot_model->getLink(solver_->getLinkNames()[i]);
    if(urdf_link == NULL) {
      ROS_DEBUG_STREAM("No entry in urdf for link " << solver_->getLinkNames()[i]);
      continue;
    }
    if(!urdf_link->collision) {
      continue;
    }
    const urdf::Geometry *geom = urdf_link->collision->geometry.get();
    if(!geom) {
      ROS_DEBUG_STREAM("No collision geometry for link " << solver_->getLinkNames()[i]);
      continue;
    }
    const urdf::Mesh *mesh = dynamic_cast<const urdf::Mesh*>(geom);
    if(mesh) {
      if (!mesh->filename.empty()) {
        planning_models::KinematicState::LinkState* ls = state->getLinkState(solver_->getLinkNames()[i]);
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
  vis_marker_array_publisher_.publish(hand_array);
}

bool ArmKinematicsConstraintAware::getPositionIK(kinematics_msgs::GetPositionIK::Request &request, 
                                                 kinematics_msgs::GetPositionIK::Response &response)
{
  if(!isReady(response.error_code)) {
    if(request.ik_request.pose_stamped.header.frame_id != solver_->getBaseName()) {
      response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
      return true;
    }
  }

  if(!checkIKService(request,response,chain_info_))
    return true;

  planning_models::KinematicState* state;
  if(collision_models_interface_->isPlanningSceneSet()) {
    state = new planning_models::KinematicState(*collision_models_interface_->getPlanningSceneState()); 
  } else {
    state = new planning_models::KinematicState(collision_models_interface_->getKinematicModel());
    state->setKinematicStateToDefault();
  }

  planning_environment::setRobotStateAndComputeTransforms(request.ik_request.robot_state,
                                                          *state);

  geometry_msgs::PoseStamped pose_msg_in = request.ik_request.pose_stamped;
  geometry_msgs::PoseStamped pose_msg_out;
  
  if(!collision_models_interface_->convertPoseGivenWorldTransform(*state,
                                                                  solver_->getBaseName(),
                                                                  pose_msg_in.header,
                                                                  pose_msg_in.pose,
                                                                  pose_msg_out)) {
    response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
    delete state;
    return true;
  }  
  ROS_DEBUG_STREAM("Pose is " << pose_msg_out.pose.position.x << " " << pose_msg_out.pose.position.y << " " << pose_msg_out.pose.position.z);

  
  bool ik_valid = solver_->getPositionIK(pose_msg_out.pose,
                                         state,
                                         response.solution.joint_state,
                                         response.error_code);

  if(ik_valid)
  {
    response.error_code.val = response.error_code.SUCCESS;
  }
  delete state;
  return ik_valid;
}

bool ArmKinematicsConstraintAware::getIKSolverInfo(kinematics_msgs::GetKinematicSolverInfo::Request &request, 
                                                     kinematics_msgs::GetKinematicSolverInfo::Response &response)
{
  if(!active_)
  {
    ROS_ERROR("IK node not active");
    return true;
  }
  response.kinematic_solver_info = chain_info_;
  return true;
}

bool ArmKinematicsConstraintAware::getFKSolverInfo(kinematics_msgs::GetKinematicSolverInfo::Request &request, 
                                                     kinematics_msgs::GetKinematicSolverInfo::Response &response)
{
  if(!active_)
  {
    ROS_ERROR("IK node not active");
    return true;
  }
  response.kinematic_solver_info = chain_info_;
  return true;
}

bool ArmKinematicsConstraintAware::getPositionFK(kinematics_msgs::GetPositionFK::Request &request, 
                                                 kinematics_msgs::GetPositionFK::Response &response)
{
  if(!active_)
  {
    ROS_ERROR("FK service not active");
    return true;
  }

  if(!checkFKService(request,response,chain_info_))
    return true;

  planning_models::KinematicState* state;
  if(collision_models_interface_->isPlanningSceneSet()) {
    state = new planning_models::KinematicState(*collision_models_interface_->getPlanningSceneState()); 
  } else {
    state = new planning_models::KinematicState(collision_models_interface_->getKinematicModel());
    state->setKinematicStateToDefault();
  }

  planning_environment::setRobotStateAndComputeTransforms(request.robot_state,
                                                          *state);

  std::vector<geometry_msgs::Pose> poses;
  bool valid = solver_->getPositionFK(state,
                                      request.fk_link_names,
                                      poses);
  if(valid) {
    response.pose_stamped.resize(poses.size());
    response.fk_link_names.resize(poses.size()); 
    for(unsigned int i=0; i < poses.size(); i++) {      
      std_msgs::Header world_header;
      world_header.frame_id = collision_models_interface_->getWorldFrameId();
      
      if(!collision_models_interface_->convertPoseGivenWorldTransform(*state,
                                                                      request.header.frame_id,
                                                                      world_header,
                                                                      poses[i],
                                                                      response.pose_stamped[i])) {
        response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
        delete state;
        return true;
      }
      response.fk_link_names[i] = request.fk_link_names[i];
      response.error_code.val = response.error_code.SUCCESS;
    }
  }
  else
  {
    ROS_ERROR("Could not compute FK");
    response.error_code.val = response.error_code.NO_FK_SOLUTION;
    valid = false;
  }
  delete state;
  return valid;
}

} // namespace

