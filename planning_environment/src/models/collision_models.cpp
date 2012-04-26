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

/** \author Ioan Sucan, E. Gil Jones */

#include "planning_environment/models/collision_models.h"
#include "planning_environment/models/model_utils.h"
#include "planning_environment/util/construct_object.h"
#include <collision_space/environmentODE.h>
#include <sstream>
#include <vector>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/body_operations.h>
#include <boost/foreach.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>

inline static std::string stripTFPrefix(const std::string& s) {
  
  if(s.find_last_of('/') == std::string::npos) {
    return s;
  }
  return s.substr(s.find_last_of('/')+1);
}

planning_environment::CollisionModels::CollisionModels(const std::string &description) : RobotModels(description)
{
  planning_scene_set_ = false;
  loadCollisionFromParamServer();
}

planning_environment::CollisionModels::CollisionModels(boost::shared_ptr<urdf::Model> urdf,
                                                       planning_models::KinematicModel* kmodel,
                                                       collision_space::EnvironmentModel* ode_collision_model) : RobotModels(urdf, kmodel)
{
  ode_collision_model_ = ode_collision_model;
}

planning_environment::CollisionModels::~CollisionModels(void)
{
  deleteAllStaticObjects();
  deleteAllAttachedObjects();
  shapes::deleteShapeVector(collision_map_shapes_);
  delete ode_collision_model_;
}

void planning_environment::CollisionModels::setupModelFromParamServer(collision_space::EnvironmentModel* model)
{
  XmlRpc::XmlRpcValue coll_ops;

  //first we do default collision operations
  if(!nh_.hasParam(description_ + "_planning/default_collision_operations")) {
    ROS_WARN("No default collision operations specified");
  } else {
  
    nh_.getParam(description_ + "_planning/default_collision_operations", coll_ops);
    
    if(coll_ops.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_WARN("default_collision_operations is not an array");
      return;
    }
    
    if(coll_ops.size() == 0) {
      ROS_WARN("No collision operations in default collision operations");
      return;
    }
    
    for(int i = 0; i < coll_ops.size(); i++) {
      if(!coll_ops[i].hasMember("object1") || !coll_ops[i].hasMember("object2") || !coll_ops[i].hasMember("operation")) {
        ROS_WARN("All collision operations must have two objects and an operation");
        continue;
      }
      arm_navigation_msgs::CollisionOperation collision_operation;
      collision_operation.object1 = std::string(coll_ops[i]["object1"]);
      collision_operation.object2 = std::string(coll_ops[i]["object2"]);
      std::string operation = std::string(coll_ops[i]["operation"]);
      if(operation == "enable") {
        collision_operation.operation =  arm_navigation_msgs::CollisionOperation::ENABLE;
      } else if(operation == "disable") {
        collision_operation.operation =  arm_navigation_msgs::CollisionOperation::DISABLE;
      } else {
        ROS_WARN_STREAM("Unrecognized collision operation " << operation << ". Must be enable or disable");
        continue;
      }
      default_collision_operations_.push_back(collision_operation);
    }
  }

  nh_.param(description_ + "_planning/default_robot_padding", default_padd_, 0.01);
  nh_.param(description_ + "_planning/default_robot_scale", default_scale_, 1.0);
  nh_.param(description_ + "_planning/default_object_padding", object_padd_, 0.02);
  nh_.param(description_ + "_planning/default_attached_padding", attached_padd_, 0.05);

  const std::vector<planning_models::KinematicModel::LinkModel*>& coll_links = kmodel_->getLinkModelsWithCollisionGeometry();
  std::map<std::string, double> default_link_padding_map;  
  std::vector<std::string> coll_names;
  for(unsigned int i = 0; i < coll_links.size(); i++) {
    default_link_padding_map[coll_links[i]->getName()] = default_padd_;
    coll_names.push_back(coll_links[i]->getName());
  }

  ros::NodeHandle priv("~");

  if(priv.hasParam("link_padding")) {
    XmlRpc::XmlRpcValue link_padding_xml;
    priv.getParam("link_padding", link_padding_xml);
    if(link_padding_xml.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_WARN("link_padding is not an array");
    } else if(link_padding_xml.size() == 0) {
      ROS_WARN("No links specified in link_padding");
    } else {
      for(int i = 0; i < link_padding_xml.size(); i++) {
        if(!link_padding_xml[i].hasMember("link") || !link_padding_xml[i].hasMember("padding")) {
          ROS_WARN("Each entry in link padding must specify a link and a padding");
          continue;
        }
        std::string link = std::string(link_padding_xml[i]["link"]);
        double padding = link_padding_xml[i]["padding"];
        //we don't care if this is a group or what, we're shoving it in
        default_link_padding_map[link] = padding;
      }
    }
  }
  
  //no allowed collisions by default
  collision_space::EnvironmentModel::AllowedCollisionMatrix default_collision_matrix(coll_names,false);

  for(std::vector<arm_navigation_msgs::CollisionOperation>::iterator it = default_collision_operations_.begin();
      it != default_collision_operations_.end();
      it++) {
    std::vector<std::string> svec1;
    std::vector<std::string> svec2;
    if(kmodel_->getModelGroup((*it).object1)) {
      svec1 = kmodel_->getModelGroup((*it).object1)->getGroupLinkNames();
    } else {
      svec1.push_back((*it).object1);
    }
    if(kmodel_->getModelGroup((*it).object2)) {
      svec2 = kmodel_->getModelGroup((*it).object2)->getGroupLinkNames();
    } else {
      svec2.push_back((*it).object2);
    }
    default_collision_matrix.changeEntry(svec1, svec2, (*it).operation != arm_navigation_msgs::CollisionOperation::ENABLE);
  }

  model->lock();
  model->setRobotModel(kmodel_, default_collision_matrix, default_link_padding_map, default_padd_, default_scale_);

  for (unsigned int i = 0 ; i < bounding_planes_.size() / 4 ; ++i)
  {
    shapes::Plane *plane = new shapes::Plane(bounding_planes_[i * 4], bounding_planes_[i * 4 + 1], bounding_planes_[i * 4 + 2], bounding_planes_[i * 4 + 3]);
    model->addObject("bounds", plane);
    ROS_INFO("Added static plane %fx + %fy + %fz + %f = 0 for model", bounding_planes_[i * 4], bounding_planes_[i * 4 + 1], bounding_planes_[i * 4 + 2], bounding_planes_[i * 4 + 3]);
  }
  
  model->unlock();    
}

void planning_environment::CollisionModels::loadCollisionFromParamServer()
{
  // a list of static planes bounding the environment
  bounding_planes_.clear();
    
  std::string planes;
  nh_.param<std::string>("bounding_planes", planes, std::string());
    
  std::stringstream ss(planes);
  if (!planes.empty())
    while (ss.good() && !ss.eof())
    {
      double value;
      ss >> value;
      bounding_planes_.push_back(value);
    }
  if (bounding_planes_.size() % 4 != 0)
  {
    ROS_WARN("~bounding_planes must be a list of 4-tuples (a b c d) that define planes ax+by+cz+d=0");
    bounding_planes_.resize(bounding_planes_.size() - (bounding_planes_.size() % 4));
  }

  if (loadedModels())
  {
    ode_collision_model_ = new collision_space::EnvironmentModelODE();
    setupModelFromParamServer(ode_collision_model_);
	
    //	bullet_collision_model_ = boost::shared_ptr<collision_space::EnvironmentModel>(new collision_space::EnvironmentModelBullet());
    //	setupModel(bullet_collision_model_, links);
  } else {
    ROS_WARN("Models not loaded");
  }
}

///
/// Functions for updating state
///

planning_models::KinematicState* 
planning_environment::CollisionModels::setPlanningScene(const arm_navigation_msgs::PlanningScene& planning_scene) {

  if(planning_scene_set_) {
    ROS_WARN("Must revert before setting planning scene again");
    return NULL;
  }
  //anything we've already got should go back to default
  deleteAllStaticObjects();
  deleteAllAttachedObjects();
  revertAllowedCollisionToDefault();
  revertCollisionSpacePaddingToDefault();
  clearAllowedContacts();

  scene_transform_map_.clear();

  for(unsigned int i = 0; i < planning_scene.fixed_frame_transforms.size(); i++) {
    if(planning_scene.fixed_frame_transforms[i].header.frame_id != getWorldFrameId()) {
      ROS_WARN_STREAM("Fixed transform for " << planning_scene.fixed_frame_transforms[i].child_frame_id 
                      << " has non-fixed header frame "  << planning_scene.fixed_frame_transforms[i].header.frame_id);
      return NULL;
    }
    scene_transform_map_[planning_scene.fixed_frame_transforms[i].child_frame_id] = planning_scene.fixed_frame_transforms[i];
  }

  planning_models::KinematicState* state = new planning_models::KinematicState(kmodel_);
  bool complete = setRobotStateAndComputeTransforms(planning_scene.robot_state, *state);
  if(!complete) {
    ROS_WARN_STREAM("Incomplete robot state in setPlanningScene");
    delete state;
    return NULL;
  }
  std::vector<arm_navigation_msgs::CollisionObject> conv_objects;
  std::vector<arm_navigation_msgs::AttachedCollisionObject> conv_att_objects;

  //need to do conversions first so we can delet the planning state
  for(unsigned int i = 0; i < planning_scene.collision_objects.size(); i++) {
    if(planning_scene.collision_objects[i].operation.operation != arm_navigation_msgs::CollisionObjectOperation::ADD) {
      ROS_WARN_STREAM("Planning scene shouldn't have collision operations other than add");
      delete state;
      return NULL;
    }
    conv_objects.push_back(planning_scene.collision_objects[i]);
    convertCollisionObjectToNewWorldFrame(*state, conv_objects.back());
  }
  for(unsigned int i = 0; i < planning_scene.attached_collision_objects.size(); i++) {
    if(planning_scene.attached_collision_objects[i].object.operation.operation != arm_navigation_msgs::CollisionObjectOperation::ADD) {
      ROS_WARN_STREAM("Planning scene shouldn't have collision operations other than add");
      delete state;
      return NULL;
    }
    conv_att_objects.push_back(planning_scene.attached_collision_objects[i]);
    convertAttachedCollisionObjectToNewWorldFrame(*state, conv_att_objects.back());
  }

  //now we delete temp_state to release the lock
  delete state;
  
  for(unsigned int i = 0; i < conv_objects.size(); i++) {
    addStaticObject(conv_objects[i]);
  }
  for(unsigned int i = 0; i < conv_att_objects.size(); i++) {
    addAttachedObject(conv_att_objects[i]);
  }

  //now we create again after adding the attached objects
  state = new planning_models::KinematicState(kmodel_);
  setRobotStateAndComputeTransforms(planning_scene.robot_state, *state);  

  //this updates the attached bodies before we mask the collision map
  updateAttachedBodyPoses(*state);

  //TODO - allowed contacts
  //have to call this first, because it reverts the allowed collision matrix
  setCollisionMap(planning_scene.collision_map, true);

  if(planning_scene.link_padding.size() > 0) {
    applyLinkPaddingToCollisionSpace(planning_scene.link_padding);
  }

  std::vector<arm_navigation_msgs::AllowedContactSpecification> acmv = planning_scene.allowed_contacts;
  for(unsigned int i = 0; i < planning_scene.allowed_contacts.size(); i++) {
    
    if(!convertPoseGivenWorldTransform(*state, 
                                       getWorldFrameId(),
                                       planning_scene.allowed_contacts[i].pose_stamped.header,
                                       planning_scene.allowed_contacts[i].pose_stamped.pose,
                                       acmv[i].pose_stamped)) {
      ROS_WARN_STREAM("Can't transform allowed contact from frame " << planning_scene.allowed_contacts[i].pose_stamped.header.frame_id
                      << " into " << getWorldFrameId());
    }
  }

  if(!planning_scene.allowed_contacts.empty()) {
    std::vector<collision_space::EnvironmentModel::AllowedContact> acv;
    convertAllowedContactSpecificationMsgToAllowedContactVector(acmv, 
                                                              acv);
    ode_collision_model_->lock();    
    ode_collision_model_->setAllowedContacts(acv);
    ode_collision_model_->unlock();    
  }

  if(!planning_scene.allowed_collision_matrix.link_names.empty()) {
    ode_collision_model_->lock();
    ode_collision_model_->setAlteredCollisionMatrix(convertFromACMMsgToACM(planning_scene.allowed_collision_matrix));
    ode_collision_model_->unlock();
  } 
  planning_scene_set_ = true;
  return state;
}

void planning_environment::CollisionModels::revertPlanningScene(planning_models::KinematicState* ks) {
  bodiesLock();
  planning_scene_set_ = false;
  delete ks;
  deleteAllStaticObjects();
  deleteAllAttachedObjects();
  revertAllowedCollisionToDefault();
  revertCollisionSpacePaddingToDefault();
  clearAllowedContacts();
  bodiesUnlock();
}

///
///  Conversion functions
///

bool planning_environment::CollisionModels::convertPoseGivenWorldTransform(const planning_models::KinematicState& state,
                                                                           const std::string& des_frame_id,
                                                                           const std_msgs::Header& header,
                                                                           const geometry_msgs::Pose& pose,
                                                                           geometry_msgs::PoseStamped& ret_pose) const
{
  ret_pose.header = header;
  ret_pose.pose = pose;
  
  std::string r_header_frame_id = stripTFPrefix(header.frame_id);
  std::string r_des_frame_id = stripTFPrefix(des_frame_id);

  bool header_is_fixed_frame = (r_header_frame_id == getWorldFrameId());
  bool des_is_fixed_frame = (r_des_frame_id == getWorldFrameId());

  //Scenario 1(fixed->fixed): if pose is in the world frame and
  //desired is in the world frame, just return
  if(header_is_fixed_frame && des_is_fixed_frame) {
    return true;
  }
  const planning_models::KinematicState::LinkState* header_link_state = state.getLinkState(r_header_frame_id);
  const planning_models::KinematicState::LinkState* des_link_state = state.getLinkState(r_des_frame_id);
  
  bool header_is_robot_frame = (header_link_state != NULL);
  bool des_is_robot_frame = (des_link_state != NULL);

  bool header_is_other_frame = !header_is_fixed_frame && !header_is_robot_frame;
  bool des_is_other_frame = !des_is_fixed_frame && !des_is_robot_frame;

  //Scenario 2(*-> other): We can't deal with desired being in a
  //non-fixed frame or relative to the robot.  TODO - decide if this is useful
  if(des_is_other_frame) {
    ROS_WARN_STREAM("Shouldn't be transforming into non-fixed non-robot frame " << r_des_frame_id);
    return false;
  }

  //Scenario 3 (other->fixed) && 4 (other->robot): we first need to
  //transform into the fixed frame
  if(header_is_other_frame) {
    if(scene_transform_map_.find(r_header_frame_id) == scene_transform_map_.end()) {
      ROS_WARN_STREAM("Trying to transform from other frame " << r_header_frame_id << " to " << r_des_frame_id << " but planning scene doesn't have other frame");
      return false;
    }
    geometry_msgs::TransformStamped trans = scene_transform_map_.find(r_header_frame_id)->second;
    
    tf::Transform tf_trans;
    tf::transformMsgToTF(trans.transform, tf_trans);
    tf::Transform tf_pose;
    tf::poseMsgToTF(ret_pose.pose, tf_pose);
    
    tf::Transform fpose = tf_trans*tf_pose;

    //assumes that we've already checked that this matched the world frame
    ret_pose.header.frame_id = getWorldFrameId();
    tf::poseTFToMsg(fpose, ret_pose.pose);

    if(des_is_fixed_frame) {
      return true;
    }
  }
  
  //getting tf version of pose
  tf::Transform bt_pose;
  tf::poseMsgToTF(ret_pose.pose,bt_pose);

  //Scenarios 4(other->robot)/5(fixed->robot): We either started out
  //with a header frame in the fixed frame or converted from a
  //non-robot frame into the fixed frame, and now we need to transform
  //into the desired robot frame given the new world transform
  if(ret_pose.header.frame_id == getWorldFrameId() && des_is_robot_frame) {
    tf::Transform trans_bt_pose = des_link_state->getGlobalLinkTransform().inverse()*bt_pose;
    tf::poseTFToMsg(trans_bt_pose,ret_pose.pose);
    ret_pose.header.frame_id = des_link_state->getName();
  } else if(header_is_robot_frame && des_is_fixed_frame) {
    //Scenario 6(robot->fixed): Just need to look up robot transform and pre-multiply
    tf::Transform trans_bt_pose = header_link_state->getGlobalLinkTransform()*bt_pose;
    tf::poseTFToMsg(trans_bt_pose,ret_pose.pose);
    ret_pose.header.frame_id = getWorldFrameId();
  } else if(header_is_robot_frame && des_is_robot_frame) {
    //Scenario 7(robot->robot): Completely tf independent
    tf::Transform trans_bt_pose = des_link_state->getGlobalLinkTransform().inverse()*(header_link_state->getGlobalLinkTransform()*bt_pose);
    tf::poseTFToMsg(trans_bt_pose,ret_pose.pose);
    ret_pose.header.frame_id = des_link_state->getName();
  } else {
    ROS_WARN("Really shouldn't have gotten here");
    return false;
  }
  return true;
}

bool planning_environment::CollisionModels::convertAttachedCollisionObjectToNewWorldFrame(const planning_models::KinematicState& state,
                                                                                          arm_navigation_msgs::AttachedCollisionObject& att_obj) const
{
  for(unsigned int i = 0; i < att_obj.object.poses.size(); i++) {
    geometry_msgs::PoseStamped ret_pose;    
    if(!convertPoseGivenWorldTransform(state,
                                       att_obj.link_name,
                                       att_obj.object.header,
                                       att_obj.object.poses[i],
                                       ret_pose)) {
      return false;
    }
    if(i == 0) {
      att_obj.object.header = ret_pose.header;
    }
    att_obj.object.poses[i] = ret_pose.pose;
  }
  return true;
}

bool planning_environment::CollisionModels::convertCollisionObjectToNewWorldFrame(const planning_models::KinematicState& state,
                                                                                  arm_navigation_msgs::CollisionObject& obj) const
{
  for(unsigned int i = 0; i < obj.poses.size(); i++) {
    geometry_msgs::PoseStamped ret_pose;
    if(!convertPoseGivenWorldTransform(state,
                                       getWorldFrameId(),
                                       obj.header,
                                       obj.poses[i],
                                       ret_pose)) {
      return false;
    }
    if(i == 0) {
      obj.header = ret_pose.header;
    }
    obj.poses[i] = ret_pose.pose;
  }
  return true;
}

bool planning_environment::CollisionModels::convertConstraintsGivenNewWorldTransform(const planning_models::KinematicState& state,
                                                                                     arm_navigation_msgs::Constraints& constraints,
                                                                                     const std::string& opt_frame) const {
  std::string trans_frame;
  if(!opt_frame.empty()) {
    trans_frame = opt_frame;
  } else {
    trans_frame = getWorldFrameId();
  }
  for(unsigned int i = 0; i < constraints.position_constraints.size(); i++) {
    geometry_msgs::PointStamped ps;
    if(!convertPointGivenWorldTransform(state,
                                        trans_frame,
                                        constraints.position_constraints[i].header,
                                        constraints.position_constraints[i].position,
                                        ps)) {
      return false;
    }
    constraints.position_constraints[i].header = ps.header;
    constraints.position_constraints[i].position = ps.point;
    
    geometry_msgs::QuaternionStamped qs;
    if(!convertQuaternionGivenWorldTransform(state,
                                             trans_frame,                                     
                                             constraints.position_constraints[i].header,
                                             constraints.position_constraints[i].constraint_region_orientation,
                                             qs)) {
      return false;
    }
    constraints.position_constraints[i].constraint_region_orientation = qs.quaternion; 
  }
  
  for(unsigned int i = 0; i < constraints.orientation_constraints.size(); i++) {
    geometry_msgs::QuaternionStamped qs;
    if(!convertQuaternionGivenWorldTransform(state,
                                             trans_frame,
                                             constraints.orientation_constraints[i].header,
                                             constraints.orientation_constraints[i].orientation,
                                             qs)) {
      return false;
    }
    constraints.orientation_constraints[i].header = qs.header;
    constraints.orientation_constraints[i].orientation = qs.quaternion;		    
  }
  
  for(unsigned int i = 0; i < constraints.visibility_constraints.size(); i++) {
    if(!convertPointGivenWorldTransform(state,
                                        trans_frame,
                                        constraints.visibility_constraints[i].target.header,
                                        constraints.visibility_constraints[i].target.point,
                                        constraints.visibility_constraints[i].target)) {
      return false;
    }
  }  
  return true;
}

bool planning_environment::CollisionModels::convertPointGivenWorldTransform(const planning_models::KinematicState& state,
                                                                            const std::string& des_frame_id,
                                                                            const std_msgs::Header& header,
                                                                            const geometry_msgs::Point& point,
                                                                            geometry_msgs::PointStamped& ret_point) const
{
  geometry_msgs::Pose arg_pose;
  arg_pose.position = point;
  arg_pose.orientation.w = 1.0;
  geometry_msgs::PoseStamped ret_pose;
  if(!convertPoseGivenWorldTransform(state, 
                                     des_frame_id,
                                     header,
                                     arg_pose,
                                     ret_pose)) {
    return false;
  }
  ret_point.header = ret_pose.header;
  ret_point.point = ret_pose.pose.position;
  return true;
}

bool planning_environment::CollisionModels::convertQuaternionGivenWorldTransform(const planning_models::KinematicState& state,
                                                                                 const std::string& des_frame_id,
                                                                                 const std_msgs::Header& header,
                                                                                 const geometry_msgs::Quaternion& quat,
                                                                                 geometry_msgs::QuaternionStamped& ret_quat) const
{
  geometry_msgs::Pose arg_pose;
  arg_pose.orientation = quat;
  geometry_msgs::PoseStamped ret_pose;
  if(!convertPoseGivenWorldTransform(state, 
                                     des_frame_id,
                                     header,
                                     arg_pose,
                                     ret_pose)) {
    return false;
  }
  ret_quat.header = ret_pose.header;
  ret_quat.quaternion = ret_pose.pose.orientation;
  return true;
}

bool planning_environment::CollisionModels::updateAttachedBodyPosesForLink(const planning_models::KinematicState& state,
                                                                           const std::string& link_name)
{
  bodiesLock();
  if(link_attached_objects_.find(link_name) == link_attached_objects_.end()) {
    bodiesUnlock();
    return false;
  }
  const planning_models::KinematicState::LinkState* ls = state.getLinkState(link_name);
  for(unsigned int j = 0; j < ls->getAttachedBodyStateVector().size(); j++) {
    const planning_models::KinematicState::AttachedBodyState* att_state = ls->getAttachedBodyStateVector()[j];
    std::map<std::string, bodies::BodyVector*>::iterator bvit = link_attached_objects_[link_name].find(att_state->getName());
    if(bvit == link_attached_objects_[link_name].end()) {
      ROS_WARN_STREAM("State out of sync with attached body vector for attached body " << att_state->getName());
      bodiesUnlock();
      return false;
    }
    if(bvit->second->getSize() != att_state->getGlobalCollisionBodyTransforms().size()) {
      ROS_WARN_STREAM("State out of sync with attached body vector for attached body " << att_state->getName());
      bodiesUnlock();
      return false;
    }
    for(unsigned int k = 0; k < att_state->getGlobalCollisionBodyTransforms().size(); k++) {
      bvit->second->setPose(k, att_state->getGlobalCollisionBodyTransforms()[k]);
    }
  }
  bodiesUnlock();
  return true;
}

bool planning_environment::CollisionModels::updateAttachedBodyPoses(const planning_models::KinematicState& state)
{
  for(std::map<std::string, std::map<std::string, bodies::BodyVector*> >::iterator it = link_attached_objects_.begin();
      it != link_attached_objects_.end();
      it++) {
    if(!updateAttachedBodyPosesForLink(state, it->first)) {
      return false;
    }
  }
  return true;
}

bool planning_environment::CollisionModels::addStaticObject(const arm_navigation_msgs::CollisionObject& obj)
{
  std::vector<shapes::Shape*> shapes;
  std::vector<tf::Transform> poses;
  for(unsigned int i = 0; i < obj.shapes.size(); i++) {
    shapes::Shape *shape = constructObject(obj.shapes[i]);
    if(!shape) {
      ROS_WARN_STREAM("Something wrong with shape");
      return false;
    }
    shapes.push_back(shape);
    tf::Transform pose;
    tf::poseMsgToTF(obj.poses[i], pose);
    poses.push_back(pose);
  }
  double padding = object_padd_;
  if(obj.padding < 0.0) {
    padding = 0.0;
  } else if(obj.padding > 0.0){
    padding = obj.padding;
  }
  addStaticObject(obj.id,
                  shapes, 
                  poses,
                  padding);
  return true;
}

//note - ownership of shape passes in
void planning_environment::CollisionModels::addStaticObject(const std::string& name,
                                                            std::vector<shapes::Shape*>& shapes,
                                                            const std::vector<tf::Transform>& poses,
                                                            double padding)
{
  if(ode_collision_model_->hasObject(name)) {
    deleteStaticObject(name);
  }
  bodiesLock();
  static_object_map_[name] = new bodies::BodyVector(shapes, poses, padding);
  ode_collision_model_->lock();
  ode_collision_model_->addObjects(name, shapes, poses);
  ode_collision_model_->unlock();
  bodiesUnlock();
}

void planning_environment::CollisionModels::deleteStaticObject(const std::string& name)
{
  bodiesLock();
  if(!ode_collision_model_->hasObject(name)) {
    return;
  }
  delete static_object_map_.find(name)->second;
  static_object_map_.erase(name);
  ode_collision_model_->lock();
  ode_collision_model_->clearObjects(name);
  ode_collision_model_->unlock();
  bodiesUnlock();
}

void planning_environment::CollisionModels::deleteAllStaticObjects() {
  bodiesLock();
  for(std::map<std::string, bodies::BodyVector*>::iterator it = static_object_map_.begin();
      it != static_object_map_.end();
      it++) {
    delete it->second;
  }
  static_object_map_.clear();
  ode_collision_model_->lock();
  ode_collision_model_->clearObjects();
  ode_collision_model_->unlock();
  bodiesUnlock();
}

void planning_environment::CollisionModels::setCollisionMap(const arm_navigation_msgs::CollisionMap& map, 
                                                            bool mask_before_insertion) {
  std::vector<shapes::Shape*> shapes(map.boxes.size());
  std::vector<tf::Transform> poses;
  for(unsigned int i = 0; i < map.boxes.size(); i++) {
    shapes[i] = new shapes::Box(map.boxes[i].extents.x, map.boxes[i].extents.y, map.boxes[i].extents.z);
    tf::Transform pose;
    pose.setOrigin(tf::Vector3(map.boxes[i].center.x, map.boxes[i].center.y, map.boxes[i].center.z));
    pose.setRotation(tf::Quaternion(tf::Vector3(map.boxes[i].axis.x, map.boxes[i].axis.y, map.boxes[i].axis.z), map.boxes[i].angle));
    poses.push_back(pose);
  }
  setCollisionMap(shapes, poses, mask_before_insertion);
}

void planning_environment::CollisionModels::setCollisionMap(std::vector<shapes::Shape*>& shapes,
                                                            const std::vector<tf::Transform>& poses,
                                                            bool mask_before_insertion)
{
  bodiesLock();
  shapes::deleteShapeVector(collision_map_shapes_);
  collision_map_shapes_ = shapes::cloneShapeVector(shapes);
  collision_map_poses_ = poses;
  std::vector<tf::Transform> masked_poses = poses;
  if(mask_before_insertion) {
    maskAndDeleteShapeVector(shapes,masked_poses); 
  } 
  ode_collision_model_->lock();
  ode_collision_model_->clearObjects(COLLISION_MAP_NAME);
  if(shapes.size() > 0) {
    ode_collision_model_->addObjects(COLLISION_MAP_NAME, shapes, masked_poses);
  } else {
    ROS_DEBUG_STREAM("Not setting any collision map objects");
  }
  ode_collision_model_->unlock();
  bodiesUnlock();
}

void planning_environment::CollisionModels::remaskCollisionMap() {
  std::vector<shapes::Shape*> shapes = shapes::cloneShapeVector(collision_map_shapes_);
  std::vector<tf::Transform> masked_poses = collision_map_poses_;
  maskAndDeleteShapeVector(shapes,masked_poses); 
  ode_collision_model_->lock();
  ode_collision_model_->clearObjects(COLLISION_MAP_NAME);
  ode_collision_model_->addObjects(COLLISION_MAP_NAME, shapes, masked_poses);
  ode_collision_model_->unlock();
}

void planning_environment::CollisionModels::maskAndDeleteShapeVector(std::vector<shapes::Shape*>& shapes,
                                                                     std::vector<tf::Transform>& poses)
{
  bodiesLock();
  std::vector<bool> mask;
  std::vector<bodies::BodyVector*> object_vector;
  //masking out static objects
  for(std::map<std::string, bodies::BodyVector*>::iterator it = static_object_map_.begin();
      it != static_object_map_.end();
      it++) {
    object_vector.push_back(it->second);
  }
  //also masking out attached objects
  for(std::map<std::string, std::map<std::string, bodies::BodyVector*> >::iterator it = link_attached_objects_.begin();
      it != link_attached_objects_.end();
      it++) {
    for(std::map<std::string, bodies::BodyVector*>::iterator it2 = it->second.begin();
	it2 != it->second.end();
	it2++) {
      object_vector.push_back(it2->second);
    }    
  }
  bodies::maskPosesInsideBodyVectors(poses, object_vector, mask, true);
  std::vector<tf::Transform> ret_poses;
  std::vector<shapes::Shape*> ret_shapes;
  unsigned int num_masked = 0;
  for(unsigned int i = 0; i < mask.size(); i++) {
    if(mask[i]) {
      ret_shapes.push_back(shapes[i]);
      ret_poses.push_back(poses[i]);
    } else {
      num_masked++;
      delete shapes[i];
    }
  }
  shapes = ret_shapes;
  poses = ret_poses;
  bodiesUnlock();
}

bool planning_environment::CollisionModels::addAttachedObject(const arm_navigation_msgs::AttachedCollisionObject& att)
{
  const arm_navigation_msgs::CollisionObject& obj = att.object;
  std::vector<shapes::Shape*> shapes;
  std::vector<tf::Transform> poses;
  for(unsigned int i = 0; i < obj.shapes.size(); i++) {
    shapes::Shape *shape = constructObject(obj.shapes[i]);
    if(!shape) {
      ROS_WARN_STREAM("Something wrong with shape");
      return false;
    }
    shapes.push_back(shape);
    tf::Transform pose;
    tf::poseMsgToTF(obj.poses[i], pose);
    poses.push_back(pose);
  }
  double padding = attached_padd_;
  if(obj.padding < 0.0) {
    padding = 0.0;
  } else if(obj.padding > 0.0){
    padding = obj.padding;
  }
  if(!addAttachedObject(obj.id,
                        att.link_name,
                        shapes,
                        poses,
                        att.touch_links,
                        padding)) {
    ROS_INFO_STREAM("Problem attaching " << obj.id);
    shapes::deleteShapeVector(shapes);
  }
  return true;
}


bool planning_environment::CollisionModels::addAttachedObject(const std::string& object_name,
                                                              const std::string& link_name,
                                                              std::vector<shapes::Shape*>& shapes,
                                                              const std::vector<tf::Transform>& poses,
                                                              const std::vector<std::string>& touch_links,
                                                              double padding)
{
  const planning_models::KinematicModel::LinkModel *link = kmodel_->getLinkModel(link_name);
  if(link == NULL) {
    ROS_WARN_STREAM("No link " << link_name << " for attaching " << object_name);
    return false;
  }
  bodiesLock();
  if(link_attached_objects_.find(link_name) != link_attached_objects_.end()) {
    if(link_attached_objects_[link_name].find(object_name) !=
       link_attached_objects_[link_name].end()) {
      delete link_attached_objects_[link_name][object_name];
      link_attached_objects_[link_name].erase(object_name);
      kmodel_->clearLinkAttachedBodyModel(link_name, object_name);
    }
  }

  //the poses will be totally incorrect until they are updated with a state
  link_attached_objects_[link_name][object_name] = new bodies::BodyVector(shapes, poses, padding);  

  std::vector<std::string> modded_touch_links;
  
  //first doing group expansion of touch links
  for(unsigned int i = 0; i < touch_links.size(); i++) {
    if(kmodel_->getModelGroup(touch_links[i])) {
      std::vector<std::string> links = kmodel_->getModelGroup(touch_links[i])->getGroupLinkNames();
      modded_touch_links.insert(modded_touch_links.end(), links.begin(), links.end());
    } else {
    modded_touch_links.push_back(touch_links[i]);
    }
  }

  if(find(modded_touch_links.begin(), modded_touch_links.end(), link_name) == modded_touch_links.end()) {
    modded_touch_links.push_back(link_name);
  }
  planning_models::KinematicModel::AttachedBodyModel* ab = 
    new planning_models::KinematicModel::AttachedBodyModel(link, object_name,
                                                           poses,
                                                           modded_touch_links,
                                                           shapes);
  kmodel_->addAttachedBodyModel(link->getName(),ab);
  ode_collision_model_->lock();
  ode_collision_model_->updateAttachedBodies();
  ode_collision_model_->unlock();

  bodiesUnlock();
  return true;
}
   
bool planning_environment::CollisionModels::deleteAttachedObject(const std::string& object_id,
                                                                 const std::string& link_name)

{
  bodiesLock();
  if(link_attached_objects_.find(link_name) != link_attached_objects_.end()) {
    if(link_attached_objects_[link_name].find(object_id) !=
       link_attached_objects_[link_name].end()) {
      //similarly, doing kmodel work first
      kmodel_->clearLinkAttachedBodyModel(link_name, object_id);
      delete link_attached_objects_[link_name][object_id];
      link_attached_objects_[link_name].erase(object_id);
    } else {
      ROS_WARN_STREAM("Link " << link_name << " has no object " << object_id << " to delete");
      bodiesUnlock();
      return false;
    }
  } else {
    ROS_WARN_STREAM("No link " << link_name << " for attached object delete");
    bodiesUnlock();
    return false;
  }
  ode_collision_model_->lock();
  ode_collision_model_->updateAttachedBodies();
  ode_collision_model_->unlock();
  bodiesUnlock();
  return true;
}

void planning_environment::CollisionModels::deleteAllAttachedObjects(const std::string& link_name)
{
  bodiesLock();
  for(std::map<std::string, std::map<std::string, bodies::BodyVector*> >::iterator it = link_attached_objects_.begin();
      it != link_attached_objects_.end();
      it++) {
    if(link_name.empty() || it->first == "link_name") {
      for(std::map<std::string, bodies::BodyVector*>::iterator it2 = it->second.begin();
          it2 != it->second.end();
          it2++) {
        delete it2->second;
      }
    }
  }
  if(link_name.empty()) {
    link_attached_objects_.clear();
  } else {
    link_attached_objects_.erase(link_name);
  }
  
  if(link_name.empty()) {
    ROS_DEBUG_STREAM("Clearing all attached body models");
    kmodel_->clearAllAttachedBodyModels();
  } else {
    ROS_DEBUG_STREAM("Clearing all attached body models for link " << link_name);
    kmodel_->clearLinkAttachedBodyModels(link_name);
  }
  ode_collision_model_->lock();
  ode_collision_model_->updateAttachedBodies();
  ode_collision_model_->unlock();
  bodiesUnlock();
}

//Link pose must be in world frame
bool planning_environment::CollisionModels::convertStaticObjectToAttachedObject(const std::string& object_name,
                                                                                const std::string& link_name,
                                                                                const tf::Transform& link_pose,
                                                                                const std::vector<std::string>& touch_links)
{
  const planning_models::KinematicModel::LinkModel *link = kmodel_->getLinkModel(link_name);
  if(link == NULL) {
    ROS_WARN_STREAM("No link " << link_name << " for attaching " << object_name);
    return false;
  }
  bodiesLock();
  if(static_object_map_.find(object_name) == static_object_map_.end()) {
    ROS_WARN_STREAM("No static object named " << object_name << " to convert");
    bodiesUnlock();
    return false;
  }
  link_attached_objects_[link_name][object_name] = static_object_map_[object_name];
  static_object_map_.erase(object_name);

  std::vector<std::string> modded_touch_links = touch_links;
  if(find(touch_links.begin(), touch_links.end(), link_name) == touch_links.end()) {
    modded_touch_links.push_back(link_name);
  }

  ode_collision_model_->lock();
  std::vector<tf::Transform> poses;
  std::vector<shapes::Shape*> shapes;
  const collision_space::EnvironmentObjects *eo = ode_collision_model_->getObjects();
  std::vector<std::string> ns = eo->getNamespaces();
  for (unsigned int i = 0 ; i < ns.size() ; ++i) {
    if(ns[i] == object_name) {
      const collision_space::EnvironmentObjects::NamespaceObjects &no = eo->getObjects(ns[i]);
      for(unsigned int j = 0; j < no.shape.size(); j++) {
        shapes.push_back(cloneShape(no.shape[j]));
        poses.push_back(no.shape_pose[j]);
      }
    }
  } 

  tf::Transform link_pose_inv = link_pose.inverse();
  //now we need to convert all these poses from the world frame into the link frame
  for(unsigned int i = 0; i < poses.size(); i++) {
    poses[i] = link_pose_inv*poses[i];
  }

  planning_models::KinematicModel::AttachedBodyModel* ab = 
    new planning_models::KinematicModel::AttachedBodyModel(link, object_name,
                                                           poses,
                                                           modded_touch_links,
                                                           shapes);
  kmodel_->addAttachedBodyModel(link->getName(),ab);

  //doing these in this order because clearObjects will take the entry
  //out of the allowed collision matrix and the update puts it back in
  ode_collision_model_->clearObjects(object_name);
  ode_collision_model_->updateAttachedBodies();
  ode_collision_model_->unlock();
  bodiesUnlock();
  return true;
}

bool planning_environment::CollisionModels::convertAttachedObjectToStaticObject(const std::string& object_name,
                                                                                const std::string& link_name,
                                                                                const tf::Transform& link_pose)
{
  const planning_models::KinematicModel::LinkModel *link = kmodel_->getLinkModel(link_name);
  if(link == NULL) {
    ROS_WARN_STREAM("No link " << link_name << " with attached object " << object_name);
    return false;
  }
  bodiesLock();

  if(link_attached_objects_.find(link_name) == link_attached_objects_.end() ||
     link_attached_objects_[link_name].find(object_name) == link_attached_objects_[link_name].end()) 
  {
    ROS_WARN_STREAM("No attached body " << object_name << " attached to link " << link_name);
    bodiesUnlock();
    return false;
  }

  static_object_map_[object_name] = link_attached_objects_[link_name][object_name];
  link_attached_objects_[link_name].erase(object_name);

  const planning_models::KinematicModel::AttachedBodyModel* att = NULL;
  for (unsigned int i = 0 ; i < link->getAttachedBodyModels().size() ; ++i) {
    if(link->getAttachedBodyModels()[i]->getName() == object_name) {
      att = link->getAttachedBodyModels()[i];
      break;
    }
  }

  if(att == NULL) {
    ROS_WARN_STREAM("Something seriously out of sync");
    bodiesUnlock();
    return false;
  }
  std::vector<shapes::Shape*> shapes = shapes::cloneShapeVector(att->getShapes());
  std::vector<tf::Transform> poses;
  for(unsigned int i = 0; i < att->getAttachedBodyFixedTransforms().size(); i++) {
    poses.push_back(link_pose*att->getAttachedBodyFixedTransforms()[i]);
  }
  kmodel_->clearLinkAttachedBodyModel(link_name, object_name);
  ode_collision_model_->lock();
  //updating attached objects first because it clears the entry from the allowed collision matrix
  ode_collision_model_->updateAttachedBodies();
  //and then this adds it back in
  ode_collision_model_->addObjects(object_name, shapes, poses);  
  ode_collision_model_->unlock();
  bodiesUnlock();
  return true;
}

void planning_environment::CollisionModels::applyLinkPaddingToCollisionSpace(const std::vector<arm_navigation_msgs::LinkPadding>& link_padding) {
  if(link_padding.empty()) return;
  
  std::map<std::string, double> link_padding_map;

  for(std::vector<arm_navigation_msgs::LinkPadding>::const_iterator it = link_padding.begin();
      it != link_padding.end();
      it++) {
    std::vector<std::string> svec1;
    if(kmodel_->getModelGroup((*it).link_name)) {
      svec1 = kmodel_->getModelGroup((*it).link_name)->getGroupLinkNames();
    } else {
      svec1.push_back((*it).link_name);
    }
    for(std::vector<std::string>::iterator stit1 = svec1.begin();
        stit1 != svec1.end();
        stit1++) {
      link_padding_map[(*stit1)] = (*it).padding;
    }
  }
  
  ode_collision_model_->lock();
  ode_collision_model_->setAlteredLinkPadding(link_padding_map);  
  ode_collision_model_->unlock();
}

void planning_environment::CollisionModels::getCurrentLinkPadding(std::vector<arm_navigation_msgs::LinkPadding>& link_padding)
{
  convertFromLinkPaddingMapToLinkPaddingVector(ode_collision_model_->getCurrentLinkPaddingMap(), link_padding);
}

bool planning_environment::CollisionModels::applyOrderedCollisionOperationsToCollisionSpace(const arm_navigation_msgs::OrderedCollisionOperations &ord, bool print) {

  ode_collision_model_->lock();
  collision_space::EnvironmentModel::AllowedCollisionMatrix acm = ode_collision_model_->getDefaultAllowedCollisionMatrix();;
  ode_collision_model_->unlock();

  std::vector<std::string> o_strings;
  getCollisionObjectNames(o_strings);
  std::vector<std::string> a_strings;
  getAttachedCollisionObjectNames(a_strings);

  bool ok = applyOrderedCollisionOperationsListToACM(ord, o_strings, a_strings, kmodel_, acm);

  if(!ok) {
    ROS_WARN_STREAM("Bad collision operations");
  }

  if(print) {
    //printAllowedCollisionMatrix(acm);
  }

  ode_collision_model_->lock();
  ode_collision_model_->setAlteredCollisionMatrix(acm);
  ode_collision_model_->unlock();
  return true;
}

bool planning_environment::CollisionModels::disableCollisionsForNonUpdatedLinks(const std::string& group_name,
                                                                                bool use_default)
{
  kmodel_->sharedLock();  
  const planning_models::KinematicModel::JointModelGroup* joint_model_group = kmodel_->getModelGroup(group_name);
  collision_space::EnvironmentModel::AllowedCollisionMatrix acm;
  if(use_default) { 
    acm = ode_collision_model_->getDefaultAllowedCollisionMatrix();
  } else {
    acm = ode_collision_model_->getCurrentAllowedCollisionMatrix();
  }
  if(joint_model_group == NULL) {
    ROS_WARN_STREAM("No joint group " << group_name);
    kmodel_->sharedUnlock();  
    return false;
  }

  std::vector<std::string> updated_link_names = joint_model_group->getUpdatedLinkModelNames();
  std::map<std::string, bool> updated_link_map;
  for(unsigned int i = 0; i < updated_link_names.size(); i++) {
    updated_link_map[updated_link_names[i]] = true;
  }
  std::vector<std::string> all_link_names;
  kmodel_->getLinkModelNames(all_link_names);

  std::vector<std::string> non_group_names;
  for(unsigned int i = 0; i < all_link_names.size(); i++) {
    //some links may not be in the matrix if they don't have collision geometry
    if(acm.hasEntry(all_link_names[i]) && updated_link_map.find(all_link_names[i]) == updated_link_map.end()) {
      non_group_names.push_back(all_link_names[i]);
    }
  }
  std::vector<const planning_models::KinematicModel::AttachedBodyModel*> att_bodies = kmodel_->getAttachedBodyModels();
  for(unsigned int i = 0; i < att_bodies.size(); i++) {
    if(updated_link_map.find(att_bodies[i]->getAttachedLinkModel()->getName()) == updated_link_map.end()) {
      non_group_names.push_back(att_bodies[i]->getName());
    }
  }

  //now adding in static object names too
  for(std::map<std::string, bodies::BodyVector*>::const_iterator it = static_object_map_.begin();
      it != static_object_map_.end();
      it++) {
    non_group_names.push_back(it->first);
  }

  //finally, adding collision map
  if(acm.hasEntry(COLLISION_MAP_NAME)) {
    non_group_names.push_back(COLLISION_MAP_NAME);
  }

  //this allows all collisions for these links with each other
  if(!acm.changeEntry(non_group_names, non_group_names, true)) {
    ROS_INFO_STREAM("Some problem changing entries");
    kmodel_->sharedUnlock();  
    return false;
  }
  setAlteredAllowedCollisionMatrix(acm);  

  kmodel_->sharedUnlock();  
  return true;
}

bool planning_environment::CollisionModels::setAlteredAllowedCollisionMatrix(const collision_space::EnvironmentModel::AllowedCollisionMatrix& acm)
{
  ode_collision_model_->lock();
  ode_collision_model_->setAlteredCollisionMatrix(acm);
  ode_collision_model_->unlock();
  return true;
}

const collision_space::EnvironmentModel::AllowedCollisionMatrix& planning_environment::CollisionModels::getCurrentAllowedCollisionMatrix() const
{
  return ode_collision_model_->getCurrentAllowedCollisionMatrix();
}

const collision_space::EnvironmentModel::AllowedCollisionMatrix& planning_environment::CollisionModels::getDefaultAllowedCollisionMatrix() const
{
  return ode_collision_model_->getDefaultAllowedCollisionMatrix();
}

void planning_environment::CollisionModels::getLastCollisionMap(arm_navigation_msgs::CollisionMap& cmap) const
{
  bodiesLock();
  cmap.header.frame_id = getWorldFrameId();
  cmap.header.stamp = ros::Time::now();
  cmap.boxes.clear();
  for(unsigned int i = 0; i < collision_map_shapes_.size(); i++) {
  if (collision_map_shapes_[i]->type == shapes::BOX) {
    const shapes::Box* box = static_cast<const shapes::Box*>(collision_map_shapes_[i]);
    arm_navigation_msgs::OrientedBoundingBox obb;
    obb.extents.x = box->size[0];
    obb.extents.y = box->size[1];
    obb.extents.z = box->size[2];
    const tf::Vector3 &c = collision_map_poses_[i].getOrigin();
    obb.center.x = c.x();
    obb.center.y = c.y();
    obb.center.z = c.z();
    const tf::Quaternion q = collision_map_poses_[i].getRotation();
    obb.angle = q.getAngle();
      const tf::Vector3 axis = q.getAxis();
      obb.axis.x = axis.x();
      obb.axis.y = axis.y();
      obb.axis.z = axis.z();
      cmap.boxes.push_back(obb);
    }
  }
  bodiesUnlock();
}

void planning_environment::CollisionModels::getCollisionSpaceCollisionMap(arm_navigation_msgs::CollisionMap& cmap) const
{
  bodiesLock();
  ode_collision_model_->lock();
  cmap.header.frame_id = getWorldFrameId();
  cmap.header.stamp = ros::Time::now();
  cmap.boxes.clear();
  
  const collision_space::EnvironmentObjects::NamespaceObjects &no = ode_collision_model_->getObjects()->getObjects(COLLISION_MAP_NAME);
  const unsigned int n = no.shape.size();
  for (unsigned int i = 0 ; i < n ; ++i) {
    if (no.shape[i]->type == shapes::BOX) {
      const shapes::Box* box = static_cast<const shapes::Box*>(no.shape[i]);
      arm_navigation_msgs::OrientedBoundingBox obb;
      obb.extents.x = box->size[0];
      obb.extents.y = box->size[1];
      obb.extents.z = box->size[2];
      const tf::Vector3 &c = no.shape_pose[i].getOrigin();
      obb.center.x = c.x();
      obb.center.y = c.y();
      obb.center.z = c.z();
      const tf::Quaternion q = no.shape_pose[i].getRotation();
      obb.angle = q.getAngle();
      const tf::Vector3 axis = q.getAxis();
      obb.axis.x = axis.x();
      obb.axis.y = axis.y();
      obb.axis.z = axis.z();
      cmap.boxes.push_back(obb);
    }
  }
  ode_collision_model_->unlock();
  bodiesUnlock();
}

void planning_environment::CollisionModels::revertAllowedCollisionToDefault() {
  ode_collision_model_->lock();
  ode_collision_model_->revertAlteredCollisionMatrix();
  ode_collision_model_->unlock();
}

void planning_environment::CollisionModels::revertCollisionSpacePaddingToDefault() {
  ode_collision_model_->lock();
  ode_collision_model_->revertAlteredLinkPadding();
  ode_collision_model_->unlock();
}

void planning_environment::CollisionModels::getCollisionSpaceAllowedCollisions(arm_navigation_msgs::AllowedCollisionMatrix& ret_matrix) const {

  convertFromACMToACMMsg(ode_collision_model_->getCurrentAllowedCollisionMatrix(),
                         ret_matrix);
}

void planning_environment::CollisionModels::getCollisionSpaceCollisionObjects(std::vector<arm_navigation_msgs::CollisionObject> &omap) const
{
  bodiesLock();
  ode_collision_model_->lock();
  omap.clear();
  const collision_space::EnvironmentObjects *eo = ode_collision_model_->getObjects();
  std::vector<std::string> ns = eo->getNamespaces();
  for (unsigned int i = 0 ; i < ns.size() ; ++i)
  {
    if (ns[i] == COLLISION_MAP_NAME)
      continue;
    const collision_space::EnvironmentObjects::NamespaceObjects &no = eo->getObjects(ns[i]);
    const unsigned int n = no.shape.size();

    arm_navigation_msgs::CollisionObject o;
    o.header.frame_id = getWorldFrameId();
    o.header.stamp = ros::Time::now();
    o.id = ns[i];
    o.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
    for (unsigned int j = 0 ; j < n ; ++j) {
      arm_navigation_msgs::Shape obj;
      if (constructObjectMsg(no.shape[j], obj)) {
        geometry_msgs::Pose pose;
        tf::poseTFToMsg(no.shape_pose[j], pose);
        o.shapes.push_back(obj);
        o.poses.push_back(pose);
      }
    }
    if(static_object_map_.find(ns[i]) == static_object_map_.end()) {
      ROS_WARN_STREAM("No matching internal object named " << ns[i]);
    } else {
      o.padding = static_object_map_.find(ns[i])->second->getPadding();
    }
    omap.push_back(o);
  }
  ode_collision_model_->unlock();
  bodiesUnlock();
}

void planning_environment::CollisionModels::getCollisionSpaceAttachedCollisionObjects(std::vector<arm_navigation_msgs::AttachedCollisionObject> &avec) const
{
  avec.clear();

  bodiesLock();
  ode_collision_model_->lock();

  std::vector<const planning_models::KinematicModel::AttachedBodyModel*> att_vec = kmodel_->getAttachedBodyModels();
  for(unsigned int i = 0; i < att_vec.size(); i++) 
  {
    arm_navigation_msgs::AttachedCollisionObject ao;
    ao.object.header.frame_id = att_vec[i]->getAttachedLinkModel()->getName();
    ao.object.header.stamp = ros::Time::now();
    ao.link_name = att_vec[i]->getAttachedLinkModel()->getName();
    double attached_padd = ode_collision_model_->getCurrentLinkPadding("attached");
    for(unsigned int j = 0; j < att_vec[i]->getShapes().size(); j++) {
      arm_navigation_msgs::Shape shape;
      constructObjectMsg(att_vec[i]->getShapes()[j], shape, attached_padd);
      geometry_msgs::Pose pose;
      tf::poseTFToMsg(att_vec[i]->getAttachedBodyFixedTransforms()[j], pose);
      ao.object.shapes.push_back(shape);
      ao.object.poses.push_back(pose);
    }
    ao.touch_links = att_vec[i]->getTouchLinks();
    ao.object.id = att_vec[i]->getName();
    if(link_attached_objects_.find(ao.link_name) == link_attached_objects_.end()) {
      ROS_WARN_STREAM("No matching attached objects for link " << ao.link_name);
    } else if(link_attached_objects_.find(ao.link_name)->second.find(ao.object.id) == 
              link_attached_objects_.find(ao.link_name)->second.end()) {
      ROS_WARN_STREAM("No matching attached objects for link " << ao.link_name << " object " << ao.object.id);
    } else {
      ao.object.padding = link_attached_objects_.find(ao.link_name)->second.find(ao.object.id)->second->getPadding();
    }
    avec.push_back(ao);
  }
  ode_collision_model_->unlock();
  bodiesUnlock();
}

bool planning_environment::CollisionModels::isKinematicStateInCollision(const planning_models::KinematicState& state)                                                                     
{
  ode_collision_model_->lock();
  ode_collision_model_->updateRobotModel(&state);
  bool in_coll = ode_collision_model_->isCollision();
  ode_collision_model_->unlock();
  return in_coll;
}

bool planning_environment::CollisionModels::isKinematicStateInSelfCollision(const planning_models::KinematicState& state)
{
  ode_collision_model_->lock();
  ode_collision_model_->updateRobotModel(&state);
  bool in_coll = ode_collision_model_->isSelfCollision();
  ode_collision_model_->unlock();
  return in_coll;
}

bool planning_environment::CollisionModels::isKinematicStateInEnvironmentCollision(const planning_models::KinematicState& state)
{
  ode_collision_model_->lock();
  ode_collision_model_->updateRobotModel(&state);
  bool in_coll = ode_collision_model_->isEnvironmentCollision();
  ode_collision_model_->unlock();
  return in_coll;
}

bool planning_environment::CollisionModels::isKinematicStateInObjectCollision(const planning_models::KinematicState &state, 
                                                                              const std::string& object_name) {
  ode_collision_model_->lock();
  ode_collision_model_->updateRobotModel(&state);
  bool in_coll = ode_collision_model_->isObjectRobotCollision(object_name);
  ode_collision_model_->unlock();
  return in_coll;
}

bool planning_environment::CollisionModels::isObjectInCollision(const std::string& object_name) {
  ode_collision_model_->lock();
  bool in_coll = ode_collision_model_->isObjectInEnvironmentCollision(object_name);
  ode_collision_model_->unlock();
  return in_coll;
}

void planning_environment::CollisionModels::getAllCollisionsForState(const planning_models::KinematicState& state,
                                                                     std::vector<arm_navigation_msgs::ContactInformation>& contacts,
                                                                     unsigned int num_per_pair) 
{
  ode_collision_model_->lock();
  ode_collision_model_->updateRobotModel(&state);
  std::vector<collision_space::EnvironmentModel::Contact> coll_space_contacts;
  ros::WallTime n1 = ros::WallTime::now();
  ode_collision_model_->getAllCollisionContacts(coll_space_contacts,
                                                num_per_pair);
  ros::WallTime n2 = ros::WallTime::now();
  ROS_DEBUG_STREAM("Got " << coll_space_contacts.size() << " collisions in " << (n2-n1).toSec());
  for(unsigned int i = 0; i < coll_space_contacts.size(); i++) {
    arm_navigation_msgs::ContactInformation contact_info;
    contact_info.header.frame_id = getWorldFrameId();
    collision_space::EnvironmentModel::Contact& contact = coll_space_contacts[i];
    contact_info.contact_body_1 = contact.body_name_1;
    contact_info.contact_body_2 = contact.body_name_2;
    if(contact.body_type_1 == collision_space::EnvironmentModel::LINK) {
      contact_info.body_type_1 = arm_navigation_msgs::ContactInformation::ROBOT_LINK;
    } else if(contact.body_type_1 == collision_space::EnvironmentModel::ATTACHED) {
      contact_info.body_type_1 = arm_navigation_msgs::ContactInformation::ATTACHED_BODY;
    } else {
      contact_info.body_type_1 = arm_navigation_msgs::ContactInformation::OBJECT;      
    }
    if(contact.body_type_2 == collision_space::EnvironmentModel::LINK) {
      contact_info.body_type_2 = arm_navigation_msgs::ContactInformation::ROBOT_LINK;
    } else if(contact.body_type_2 == collision_space::EnvironmentModel::ATTACHED) {
      contact_info.body_type_2 = arm_navigation_msgs::ContactInformation::ATTACHED_BODY;
    } else {
      contact_info.body_type_2 = arm_navigation_msgs::ContactInformation::OBJECT;      
    }
    contact_info.position.x = contact.pos.x();
    contact_info.position.y = contact.pos.y();
    contact_info.position.z = contact.pos.z();
    contacts.push_back(contact_info);
  }
  ode_collision_model_->unlock();
}

void planning_environment::CollisionModels::getAllEnvironmentCollisionsForObject(const std::string& object_name,  
                                                                                 std::vector<arm_navigation_msgs::ContactInformation>& contacts, 
                                                                                 unsigned int num_per_pair) {
  ode_collision_model_->lock();
  std::vector<collision_space::EnvironmentModel::Contact> coll_space_contacts;
  ode_collision_model_->getAllObjectEnvironmentCollisionContacts(object_name, coll_space_contacts, num_per_pair);
  for(unsigned int i = 0; i < coll_space_contacts.size(); i++) {
    arm_navigation_msgs::ContactInformation contact_info;
    contact_info.header.frame_id = getWorldFrameId();
    collision_space::EnvironmentModel::Contact& contact = coll_space_contacts[i];
    contact_info.contact_body_1 = contact.body_name_1;
    contact_info.contact_body_2 = contact.body_name_2;
    if(contact.body_type_1 == collision_space::EnvironmentModel::LINK) {
      contact_info.body_type_1 = arm_navigation_msgs::ContactInformation::ROBOT_LINK;
    } else if(contact.body_type_1 == collision_space::EnvironmentModel::ATTACHED) {
      contact_info.body_type_1 = arm_navigation_msgs::ContactInformation::ATTACHED_BODY;
    } else {
      contact_info.body_type_1 = arm_navigation_msgs::ContactInformation::OBJECT;      
    }
    if(contact.body_type_2 == collision_space::EnvironmentModel::LINK) {
      contact_info.body_type_2 = arm_navigation_msgs::ContactInformation::ROBOT_LINK;
    } else if(contact.body_type_2 == collision_space::EnvironmentModel::ATTACHED) {
      contact_info.body_type_2 = arm_navigation_msgs::ContactInformation::ATTACHED_BODY;
    } else {
      contact_info.body_type_2 = arm_navigation_msgs::ContactInformation::OBJECT;      
    }
    contact_info.position.x = contact.pos.x();
    contact_info.position.y = contact.pos.y();
    contact_info.position.z = contact.pos.z();
    contacts.push_back(contact_info);
  }
  ode_collision_model_->unlock();

}

bool planning_environment::CollisionModels::isKinematicStateValid(const planning_models::KinematicState& state,
                                                                  const std::vector<std::string>& joint_names,
                                                                  arm_navigation_msgs::ArmNavigationErrorCodes& error_code,
                                                                  const arm_navigation_msgs::Constraints goal_constraints,
                                                                  const arm_navigation_msgs::Constraints path_constraints,
								  bool verbose)
{
  if(!state.areJointsWithinBounds(joint_names)) {
    if(verbose) {
      for(unsigned int j = 0; j < joint_names.size(); j++) {
	if(!state.isJointWithinBounds(joint_names[j])) {
	  std::pair<double, double> bounds; 
	  state.getJointState(joint_names[j])->getJointModel()->getVariableBounds(joint_names[j], bounds);
          double val = state.getJointState(joint_names[j])->getJointStateValues()[0];
	  ROS_DEBUG_STREAM("Joint " << joint_names[j] << " out of bounds. " <<
                           " value: " << val << 
                           " low: " << bounds.first << " diff low: " << val-bounds.first << " high: " << bounds.second 
                           << " diff high: " << val-bounds.second);
	}
      }
    }
    error_code.val = error_code.JOINT_LIMITS_VIOLATED;
    return false;
  }
  if(!doesKinematicStateObeyConstraints(state, path_constraints, false)) {
    if(verbose) {
      doesKinematicStateObeyConstraints(state, path_constraints, true);
    }
    error_code.val = error_code.PATH_CONSTRAINTS_VIOLATED;
    return false;
  }
  if(!doesKinematicStateObeyConstraints(state, goal_constraints, false)) {
    if(verbose) {
      doesKinematicStateObeyConstraints(state, goal_constraints, true);
    }
    error_code.val = error_code.GOAL_CONSTRAINTS_VIOLATED;
    return false;
  }
  if(isKinematicStateInCollision(state)) {
    error_code.val = error_code.COLLISION_CONSTRAINTS_VIOLATED;    
    if(verbose) {
      std::vector<arm_navigation_msgs::ContactInformation> contacts;
      getAllCollisionsForState(state, contacts,1);
      if(contacts.size() == 0) {
        ROS_WARN_STREAM("Collision reported but no contacts");
      }
      for(unsigned int i = 0; i < contacts.size(); i++) {
	ROS_DEBUG_STREAM("Collision between " << contacts[i].contact_body_1 
                         << " and " << contacts[i].contact_body_2);
      }
    }
    return false;
  }
  error_code.val = error_code.SUCCESS;
  return true;
}

bool planning_environment::CollisionModels::isJointTrajectoryValid(const arm_navigation_msgs::PlanningScene& planning_scene,
                                                                   const trajectory_msgs::JointTrajectory &trajectory,
                                                                   const arm_navigation_msgs::Constraints& goal_constraints,
                                                                   const arm_navigation_msgs::Constraints& path_constraints,
                                                                   arm_navigation_msgs::ArmNavigationErrorCodes& error_code,
                                                                   std::vector<arm_navigation_msgs::ArmNavigationErrorCodes>& trajectory_error_codes,
                                                                   const bool evaluate_entire_trajectory)
{
  if(planning_scene_set_) {
    ROS_WARN("Must revert planning scene before checking trajectory with planning scene");
    return false;
  }

  planning_models::KinematicState* state = setPlanningScene(planning_scene);

  if(state == NULL) {
    ROS_WARN("Planning scene invalid in isTrajectoryValid");
    return false;
  }

  bool ok =  isJointTrajectoryValid(*state, trajectory, goal_constraints, path_constraints, error_code, trajectory_error_codes, evaluate_entire_trajectory);
  revertPlanningScene(state);
  return ok;
}


bool planning_environment::CollisionModels::isJointTrajectoryValid(planning_models::KinematicState& state,
                                                                   const trajectory_msgs::JointTrajectory &trajectory,
                                                                   const arm_navigation_msgs::Constraints& goal_constraints,
                                                                   const arm_navigation_msgs::Constraints& path_constraints,
                                                                   arm_navigation_msgs::ArmNavigationErrorCodes& error_code,
                                                                   std::vector<arm_navigation_msgs::ArmNavigationErrorCodes>& trajectory_error_codes,
                                                                   const bool evaluate_entire_trajectory)  
{
  error_code.val = error_code.SUCCESS;
  trajectory_error_codes.clear();
  
  //now we need to start evaluating the trajectory
  std::map<std::string, double> joint_value_map;

  // get the joints this trajectory is for
  std::vector<planning_models::KinematicState::JointState*> joints(trajectory.joint_names.size());
  for (unsigned int j = 0 ; j < joints.size() ; ++j)
  {
    joints[j] = state.getJointState(trajectory.joint_names[j]);
    if (joints[j] == NULL)
    {
      ROS_ERROR("Unknown joint '%s' found on path", trajectory.joint_names[j].c_str());
      error_code.val = error_code.INVALID_TRAJECTORY;
      return false;
    } else {
      joint_value_map[joints[j]->getName()] = 0.0;
    }
  }

  //now we specifically evaluate the first point of the trajectory
  for (unsigned int j = 0 ; j < trajectory.points.front().positions.size(); j++)
  {
    joint_value_map[trajectory.joint_names[j]] = trajectory.points.front().positions[j];
  }
  state.setKinematicState(joint_value_map);

  //then start state obeying path constraints
  if(!doesKinematicStateObeyConstraints(state, path_constraints)) {
    error_code.val = error_code.START_STATE_VIOLATES_PATH_CONSTRAINTS;
    if(!evaluate_entire_trajectory) {
      return false;
    }
  }

  //next check collision
  ode_collision_model_->updateRobotModel(&state);
  if(ode_collision_model_->isCollision()) {
    error_code.val = error_code.START_STATE_IN_COLLISION;
    if(!evaluate_entire_trajectory) {
      return false;
    }
  }

  //now we make sure that the goal constraints are reasonable, at least in terms of joint_constraints
  //note that this only checks the position itself, not there are values within the tolerance
  //that don't violate the joint limits
  joint_value_map.clear();
  std::vector<std::string> goal_joint_names;
  for(unsigned int i = 0; i < goal_constraints.joint_constraints.size(); i++) {
    goal_joint_names.push_back(goal_constraints.joint_constraints[i].joint_name);
    joint_value_map[goal_joint_names.back()] = goal_constraints.joint_constraints[i].position;
  }
  state.setKinematicState(joint_value_map);
  if(!state.areJointsWithinBounds(goal_joint_names)) {
    error_code.val = error_code.INVALID_GOAL_JOINT_CONSTRAINTS;
    return false;
  }

  joint_value_map.clear();
  //next we check that the final point in the trajectory satisfies the goal constraints
  for (unsigned int j = 0 ; j < trajectory.points.back().positions.size(); j++)
  {
    joint_value_map[trajectory.joint_names[j]] = trajectory.points.back().positions[j];
  }
  state.setKinematicState(joint_value_map);
  if(!doesKinematicStateObeyConstraints(state, goal_constraints)) {
    error_code.val = error_code.GOAL_CONSTRAINTS_VIOLATED;
    return false;
  }

  //now we can start checking the actual 
  arm_navigation_msgs::Constraints emp_goal_constraints;
  for(unsigned int i = 0; i < trajectory.points.size(); i++) {
    arm_navigation_msgs::ArmNavigationErrorCodes suc;
    suc.val = error_code.SUCCESS;
    trajectory_error_codes.push_back(suc);
    for (unsigned int j = 0 ; j < trajectory.points[i].positions.size(); j++)
    {
      joint_value_map[trajectory.joint_names[j]] = trajectory.points[i].positions[j];
    }
    state.setKinematicState(joint_value_map);

    if(!isKinematicStateValid(state, trajectory.joint_names, suc, 
                              emp_goal_constraints, path_constraints)) {
      //this means we return the last error code if we are evaluating the whole trajectory
      error_code = suc;
      trajectory_error_codes.back() = suc;
      if(!evaluate_entire_trajectory) {
        return false;
      }
    }
    trajectory_error_codes.back() = suc;
  }
  return(error_code.val == error_code.SUCCESS);
}

// bool planning_environment::CollisionModels::isRobotTrajectoryValid(const arm_navigation_msgs::PlanningScene& planning_scene,
//                                                                    const arm_navigation_msgs::RobotTrajectory& trajectory,
//                                                                    const arm_navigation_msgs::Constraints& goal_constraints,
//                                                                    const arm_navigation_msgs::Constraints& path_constraints,
//                                                                    arm_navigation_msgs::ArmNavigationErrorCodes& error_code,
//                                                                    std::vector<arm_navigation_msgs::ArmNavigationErrorCodes>& trajectory_error_codes,
//                                                                    const bool evaluate_entire_trajectory)
// {
//   if(planning_scene_set_) {
//     ROS_WARN("Must revert planning scene before checking trajectory with planning scene");
//     return false;
//   }

//   planning_models::KinematicState* state = setPlanningScene(planning_scene);

//   if(state == NULL) {
//     ROS_WARN("Planning scene invalid in isTrajectoryValid");
//     return false;
//   }

//   bool ok =  isRobotTrajectoryValid(*state, trajectory, goal_constraints, path_constraints, error_code, trajectory_error_codes, evaluate_entire_trajectory);
//   revertPlanningScene(state);
//   return ok;
// }

// bool planning_environment::CollisionModels::isRobotTrajectoryValid(planning_models::KinematicState& state,
//                                                                    const arm_navigation_msgs::RobotTrajectory& trajectory,
//                                                                    const arm_navigation_msgs::Constraints& goal_constraints,
//                                                                    const arm_navigation_msgs::Constraints& path_constraints,
//                                                                    arm_navigation_msgs::ArmNavigationErrorCodes& error_code,
//                                                                    std::vector<arm_navigation_msgs::ArmNavigationErrorCodes>& trajectory_error_codes,
//                                                                    const bool evaluate_entire_trajectory)
// {
//   //first thing to check - we can either deal with no joint_trajectory points
//   //meaning that the robot state controls the positions of those links
//   //or the same number of points as is in the multi_dof_trajectory
//   if(trajectory.joint_trajectory.points.size() != 0 &&
//      trajectory.joint_trajectory.points.size() != trajectory.multi_dof_trajectory.points.size()) {
//     ROS_WARN("Invalid robot trajectory due to point number disparities");
//     error_code.val = error_code.INVALID_TRAJECTORY;
//     return false;
//   }

//   std::vector<std::string> joint_names = trajectory.joint_trajectory.joint_names;

// }

double planning_environment::CollisionModels::getTotalTrajectoryJointLength(planning_models::KinematicState& state,
                                                                          const trajectory_msgs::JointTrajectory& jtraj) const 
{
  std::map<std::string, double> all_joint_state_values;
  state.getKinematicStateValues(all_joint_state_values);

  std::map<std::string, double> joint_positions;
  for(unsigned int i = 0; i < jtraj.joint_names.size(); i++) {
    joint_positions[jtraj.joint_names[i]] = all_joint_state_values[jtraj.joint_names[i]];
  }
  double total_path_length = 0.0;
  for(unsigned int i = 0; i < jtraj.points.size(); i++) {
    for(unsigned int j = 0; j < jtraj.joint_names.size(); j++) {
      joint_positions[jtraj.joint_names[j]] = jtraj.points[i].positions[j];
      if(i != jtraj.points.size()-1) {
        total_path_length += fabs(jtraj.points[i+1].positions[j]-joint_positions[jtraj.joint_names[j]]);
      }
    }
  }
  return total_path_length;
}

//visualization functions

void planning_environment::CollisionModels::getCollisionMapAsMarkers(visualization_msgs::MarkerArray& arr,
                                                                     const std_msgs::ColorRGBA& color,
                                                                     const ros::Duration& lifetime) 
{
  visualization_msgs::Marker mark;
  mark.type = visualization_msgs::Marker::CUBE_LIST;
  mark.color.a = 1.0;
  mark.lifetime = lifetime;
  mark.ns = "collision_map_markers";
  mark.id = 0;
  mark.header.frame_id = getWorldFrameId();
  mark.header.stamp = ros::Time::now();
  const collision_space::EnvironmentObjects::NamespaceObjects &no = ode_collision_model_->getObjects()->getObjects(COLLISION_MAP_NAME);
  const unsigned int n = no.shape.size();
  for (unsigned int i = 0 ; i < n ; ++i) {
    if (no.shape[i]->type == shapes::BOX) {
      const shapes::Box* box = static_cast<const shapes::Box*>(no.shape[i]);
      //first dimension of first box assumed to hold for all boxes
      if(i == 0) {
        mark.scale.x = box->size[0];
        mark.scale.y = box->size[0];
        mark.scale.z = box->size[0];
      }
      const tf::Vector3 &c = no.shape_pose[i].getOrigin();
      geometry_msgs::Point point;
      point.x = c.x();
      point.y = c.y();
      point.z = c.z();
      std_msgs::ColorRGBA color;      
      color.r = fmin(fmax(fabs(point.z)/0.5, 0.10), 1.0);
      color.g = fmin(fmax(fabs(point.z)/1.0, 0.20), 1.0);
      color.b = fmin(fmax(fabs(point.z)/1.5, 0.50), 1.0);
      mark.colors.push_back(color);
      mark.points.push_back(point);
    }
  }
  arr.markers.push_back(mark);
}

void planning_environment::CollisionModels::getAllCollisionPointMarkers(const planning_models::KinematicState& state,
                                                                        visualization_msgs::MarkerArray& arr,
                                                                        const std_msgs::ColorRGBA& color,
                                                                        const ros::Duration& lifetime)
{
  std::vector<arm_navigation_msgs::ContactInformation> coll_info_vec;
  getAllCollisionsForState(state,coll_info_vec,1);
  getCollisionMarkersFromContactInformation(coll_info_vec,
                                            getWorldFrameId(),
                                            arr,
                                            color,
                                            lifetime);
}

void planning_environment::CollisionModels::getStaticCollisionObjectMarkers(visualization_msgs::MarkerArray& arr,
                                                                            const std::string& name,
                                                                            const std_msgs::ColorRGBA& color,
                                                                            const ros::Duration& lifetime) const
{
  std::vector<arm_navigation_msgs::CollisionObject> static_objects;
  getCollisionSpaceCollisionObjects(static_objects);
  
  for(unsigned int i = 0; i < static_objects.size(); i++) {
    for(unsigned int j = 0; j < static_objects[i].shapes.size(); j++) {
      visualization_msgs::Marker mk;
      mk.header.frame_id = getWorldFrameId();
      mk.header.stamp = ros::Time::now();
      if(name.empty()) {
        mk.ns = static_objects[i].id;
      } else {
        mk.ns = name;
      }
      mk.id = j;
      mk.action = visualization_msgs::Marker::ADD;
      mk.pose = static_objects[i].poses[j];
      mk.color.a = color.a;
      if(mk.color.a == 0.0) {
        mk.color.a = 1.0;
      }
      mk.color.r = color.r;
      mk.color.g = color.g;
      mk.color.b = color.b;
      setMarkerShapeFromShape(static_objects[i].shapes[j], mk);
      mk.lifetime = lifetime;
      arr.markers.push_back(mk);
    }
  }
}
void planning_environment::CollisionModels::getAttachedCollisionObjectMarkers(const planning_models::KinematicState& state,
                                                                              visualization_msgs::MarkerArray& arr,
                                                                              const std::string& name,
                                                                              const std_msgs::ColorRGBA& color,
                                                                              const ros::Duration& lifetime,
                                                                              const bool show_padded,
                                                                              const std::vector<std::string>* link_names) const

{
  ode_collision_model_->lock();
  std::vector<arm_navigation_msgs::AttachedCollisionObject> attached_objects;
  getCollisionSpaceAttachedCollisionObjects(attached_objects);

  ode_collision_model_->updateRobotModel(&state);

  std::map<std::string, std::vector<tf::Transform> > att_pose_map;
  ode_collision_model_->getAttachedBodyPoses(att_pose_map);

  for(unsigned int i = 0; i < attached_objects.size(); i++) {
    if(link_names != NULL) {
      bool found = false;
      for(unsigned int j = 0; j < link_names->size(); j++) {
        if(attached_objects[i].link_name == (*link_names)[j]) {
          found = true;
          break;
        }
      }
      if(!found) {
        continue;
      }
    }

    if(att_pose_map.find(attached_objects[i].object.id) == att_pose_map.end()) {
      ROS_WARN_STREAM("No collision space poses for " << attached_objects[i].object.id); 
      continue;
    }
    if(att_pose_map[attached_objects[i].object.id].size() !=
       attached_objects[i].object.shapes.size()) {
      ROS_WARN_STREAM("Size mismatch between poses size " << att_pose_map[attached_objects[i].object.id].size()
                      << " and shapes size " << attached_objects[i].object.shapes.size());
      continue;
    }
    for(unsigned int j = 0; j < attached_objects[i].object.shapes.size(); j++) {
      visualization_msgs::Marker mk;
      mk.header.frame_id = getWorldFrameId();
      mk.header.stamp = ros::Time::now();
      if(name.empty()) {
        mk.ns = attached_objects[i].object.id;
      } else {
        mk.ns = name;
      }
      mk.id = j;
      mk.action = visualization_msgs::Marker::ADD;
      tf::poseTFToMsg(att_pose_map[attached_objects[i].object.id][j], mk.pose);
      mk.color.a = color.a;
      if(mk.color.a == 0.0) {
        mk.color.a = 1.0;
      }
      mk.color.r = color.r;
      mk.color.g = color.g;
      mk.color.b = color.b;
      mk.lifetime = lifetime;
      setMarkerShapeFromShape(attached_objects[i].object.shapes[j], mk);
      arr.markers.push_back(mk);
    }
  }
  ode_collision_model_->unlock();
}

void planning_environment::CollisionModels::getPlanningSceneGivenState(const planning_models::KinematicState& state,
                                                                       arm_navigation_msgs::PlanningScene& planning_scene)
{
  convertKinematicStateToRobotState(state, ros::Time::now(), getWorldFrameId(), planning_scene.robot_state);
  convertFromACMToACMMsg(getCurrentAllowedCollisionMatrix(), planning_scene.allowed_collision_matrix);
  //todo - any such thing as current allowed contacts?
  getCurrentLinkPadding(planning_scene.link_padding);
  getCollisionSpaceCollisionObjects(planning_scene.collision_objects);
  getCollisionSpaceAttachedCollisionObjects(planning_scene.attached_collision_objects);
  getCollisionSpaceCollisionMap(planning_scene.collision_map);
}

void planning_environment::CollisionModels::getAllCollisionSpaceObjectMarkers(const planning_models::KinematicState& state,
                                                                              visualization_msgs::MarkerArray& arr,
                                                                              const std::string& name,
                                                                              const std_msgs::ColorRGBA& static_color,
                                                                              const std_msgs::ColorRGBA& attached_color,
                                                                              const ros::Duration& lifetime)
{
  getStaticCollisionObjectMarkers(arr, name, static_color, lifetime);
  getAttachedCollisionObjectMarkers(state, arr, name, attached_color, lifetime);
}

void planning_environment::CollisionModels::getRobotMarkersGivenState(const planning_models::KinematicState& state,
                                                                      visualization_msgs::MarkerArray& arr,
                                                                      const std_msgs::ColorRGBA& color,
                                                                      const std::string& name,
                                                                      const ros::Duration& lifetime,
                                                                      const std::vector<std::string>* names,
                                                                      const double scale,
                                                                      const bool show_collision_models) const
{
  boost::shared_ptr<urdf::Model> robot_model = getParsedDescription();

  std::vector<std::string> link_names;
  if(names == NULL)
  {
    kmodel_->getLinkModelNames(link_names);
  }
  else
  {
    link_names = *names;
  }

  for(unsigned int i = 0; i < link_names.size(); i++)
  {
    boost::shared_ptr<const urdf::Link> urdf_link = robot_model->getLink(link_names[i]);

    if(!urdf_link)
    {
      ROS_INFO_STREAM("Invalid urdf name " << link_names[i]);
      continue;
    }

    if(show_collision_models && !urdf_link->collision && !urdf_link->visual)
    {
      continue;
    }
    if(!show_collision_models && !urdf_link->visual)
    {
      continue;
    }
    const urdf::Geometry *geom = NULL;
    if(show_collision_models && urdf_link->collision) {
      geom = urdf_link->collision->geometry.get();
    } else if(urdf_link->visual) {
      geom = urdf_link->visual->geometry.get();
    }

    if(!geom)
    {
      continue;
    }

    const urdf::Mesh *mesh = dynamic_cast<const urdf::Mesh*> (geom);
    const urdf::Box *box = dynamic_cast<const urdf::Box*> (geom);
    const urdf::Sphere *sphere = dynamic_cast<const urdf::Sphere*> (geom);
    const urdf::Cylinder *cylinder = dynamic_cast<const urdf::Cylinder*> (geom);

    const planning_models::KinematicState::LinkState* ls = state.getLinkState(link_names[i]);

    if(ls == NULL)
    {
      ROS_WARN_STREAM("No link state for name " << names << " though there's a mesh");
      continue;
    }

    visualization_msgs::Marker mark;
    mark.header.frame_id = getWorldFrameId();
    mark.header.stamp = ros::Time::now();
    mark.ns = name;
    mark.id = i;
    mark.lifetime = lifetime;
    tf::poseTFToMsg(ls->getGlobalCollisionBodyTransform(), mark.pose);
    mark.color = color;

    if(mesh)
    {
      if(mesh->filename.empty())
      {
        continue;
      }
      mark.type = mark.MESH_RESOURCE;
      mark.scale.x = mesh->scale.x*scale;
      mark.scale.y = mesh->scale.y*scale;
      mark.scale.z = mesh->scale.z*scale;
      mark.mesh_resource = mesh->filename;
    }
    else if(box)
    {
      mark.type = mark.CUBE;
      mark.scale.x = box->dim.x;
      mark.scale.y = box->dim.y;
      mark.scale.z = box->dim.z;
    }
    else if(cylinder)
    {
      mark.type = mark.CYLINDER;
      mark.scale.x = cylinder->radius;
      mark.scale.y = cylinder->radius;
      mark.scale.z = cylinder->length;
    } else if(sphere) {
      mark.type = mark.SPHERE;
      mark.scale.x = mark.scale.y = mark.scale.z = sphere->radius;
    } else {
      ROS_WARN_STREAM("Unknown object type for link " << link_names[i]);
      continue;
    }
    arr.markers.push_back(mark);
  }
}

void planning_environment::CollisionModels::getRobotPaddedMarkersGivenState(const planning_models::KinematicState& state,
                                                                            visualization_msgs::MarkerArray& arr,
                                                                            const std_msgs::ColorRGBA& color,
                                                                            const std::string& name,
                                                                            const ros::Duration& lifetime,
                                                                            const std::vector<std::string>* names) const
{
  std::vector<std::string> link_names;
  if(names == NULL)
  {
    kmodel_->getLinkModelNames(link_names);
  }
  else
  {
    link_names = *names;
  }
  for(unsigned int i = 0; i < link_names.size(); i++) {
    const planning_models::KinematicState::LinkState* ls = state.getLinkState(link_names[i]);
    if(ls->getLinkModel()->getLinkShape() == NULL) continue;

    const tf::Transform& trans = ls->getGlobalCollisionBodyTransform();

    visualization_msgs::Marker mark;
    mark.header.frame_id = getWorldFrameId();
    mark.header.stamp = ros::Time::now();
    mark.ns = name;
    mark.id = i;
    mark.color = color; 
    mark.lifetime = lifetime;
    tf::poseTFToMsg(trans, mark.pose);

    double padding = 0.0;
    if(getCurrentLinkPaddingMap().find(ls->getName()) !=
       getCurrentLinkPaddingMap().end()) {
      padding = getCurrentLinkPaddingMap().find(ls->getName())->second;
    }
    setMarkerShapeFromShape(ls->getLinkModel()->getLinkShape(), mark, padding);
    arr.markers.push_back(mark);
  }
}

void planning_environment::CollisionModels::getGroupAndUpdatedJointMarkersGivenState(const planning_models::KinematicState& state,
                                                                                     visualization_msgs::MarkerArray& arr,
                                                                                     const std::string& group_name, 
                                                                                     const std_msgs::ColorRGBA& group_color,
                                                                                     const std_msgs::ColorRGBA& updated_color,
                                                                                     const ros::Duration& lifetime) const {

  const planning_models::KinematicModel::JointModelGroup* jmg = kmodel_->getModelGroup(group_name);
  
  if(jmg == NULL) {
    ROS_WARN_STREAM("No group " << group_name << " for visualization");
    return;
  }

  std::vector<std::string> group_link_names = jmg->getGroupLinkNames();      
  getRobotMarkersGivenState(state,
                            arr,
                            group_color,
                            group_name,
                            lifetime,
                            &group_link_names);
  
  std::vector<std::string> updated_link_model_names = jmg->getUpdatedLinkModelNames();
  std::map<std::string, bool> dont_include;
  for(unsigned int i = 0; i < group_link_names.size(); i++) {
    dont_include[group_link_names[i]] = true;
  }

  std::vector<std::string> ex_list;
  for(unsigned int i = 0; i < updated_link_model_names.size(); i++) {
    if(dont_include.find(updated_link_model_names[i]) == dont_include.end()) {
      ex_list.push_back(updated_link_model_names[i]);
    }
  }
  getRobotMarkersGivenState(state,
                            arr,
                            updated_color,
                            group_name+"_updated_links",
                            lifetime,
                            &ex_list);

}


void planning_environment::CollisionModels::writePlanningSceneBag(const std::string& filename,
                                                                  const arm_navigation_msgs::PlanningScene& planning_scene) const
{
  rosbag::Bag bag;
  bag.open(filename, rosbag::bagmode::Write);
  
  ros::Time t = planning_scene.robot_state.joint_state.header.stamp;
  bag.write("planning_scene", t, planning_scene);
  bag.close();
}

bool planning_environment::CollisionModels::readPlanningSceneBag(const std::string& filename,
                                                                 arm_navigation_msgs::PlanningScene& planning_scene) const
{
  rosbag::Bag bag;
  try {
    bag.open(filename, rosbag::bagmode::Read);
  } catch(rosbag::BagException) {
    ROS_WARN_STREAM("Could not open bag file " << filename);
    return false;
  }

  std::vector<std::string> topics;
  topics.push_back("planning_scene");

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  bool has_one = false;
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    arm_navigation_msgs::PlanningScene::ConstPtr ps = m.instantiate<arm_navigation_msgs::PlanningScene>();
    if(ps != NULL) {
      planning_scene = *ps;
      has_one = true;
    }
  }    
  if(!has_one) {
    ROS_WARN_STREAM("Filename " << filename << " does not seem to contain a planning scene");
    return false;
  }
  return true;
}

bool planning_environment::CollisionModels::appendMotionPlanRequestToPlanningSceneBag(const std::string& filename,
                                                                                      const std::string& topic_name,
                                                                                      const arm_navigation_msgs::MotionPlanRequest& req)
{
  rosbag::Bag bag;
  try {
    bag.open(filename, rosbag::bagmode::Append);
  } catch(rosbag::BagException) {
    ROS_WARN_STREAM("Could not append to bag file " << filename);
    return false;
  }
  ros::Time t = req.start_state.joint_state.header.stamp;
  bag.write(topic_name, ros::Time::now(), req);
  bag.close();
  return true;
}

bool planning_environment::CollisionModels::loadMotionPlanRequestsInPlanningSceneBag(const std::string& filename,
                                                                                     const std::string& topic_name,
                                                                                     std::vector<arm_navigation_msgs::MotionPlanRequest>& motion_plan_reqs){
  rosbag::Bag bag;
  try {
    bag.open(filename, rosbag::bagmode::Read);
  } catch(rosbag::BagException) {
    ROS_WARN_STREAM("Could not open bag file " << filename);
    return false;
  }

  std::vector<std::string> topics;
  topics.push_back(topic_name);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    arm_navigation_msgs::MotionPlanRequest::ConstPtr mpr = m.instantiate<arm_navigation_msgs::MotionPlanRequest>();
    if(mpr != NULL) {
      motion_plan_reqs.push_back(*mpr);
    }
  }    
  if(motion_plan_reqs.size() == 0) {
    ROS_WARN_STREAM("No MotionPlanRequest messages with topic name " << topic_name << " in " << filename);
    return false;
  }
  return true;  
}

bool planning_environment::CollisionModels::loadJointTrajectoriesInPlanningSceneBag(const std::string& filename,
                                                                                    const std::string& topic_name,
                                                                                    std::vector<trajectory_msgs::JointTrajectory>& traj_vec){
  rosbag::Bag bag;
  try {
    bag.open(filename, rosbag::bagmode::Read);
  } catch(rosbag::BagException) {
    ROS_WARN_STREAM("Could not open bag file " << filename);
    return false;
  }

  std::vector<std::string> topics;
  topics.push_back(topic_name);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    trajectory_msgs::JointTrajectory::ConstPtr jt = m.instantiate<trajectory_msgs::JointTrajectory>();
    if(jt != NULL) {
      traj_vec.push_back(*jt);
    }
  }    
  if(traj_vec.size() == 0) {
    ROS_WARN_STREAM("No JointTrajectory messages with topic name " << topic_name << " in " << filename);
    return false;
  }
  return true;  
}

bool planning_environment::CollisionModels::appendJointTrajectoryToPlanningSceneBag(const std::string& filename,
                                                                                    const std::string& topic_name,
                                                                                    const trajectory_msgs::JointTrajectory& traj)
{
  rosbag::Bag bag;
  try {
    bag.open(filename, rosbag::bagmode::Append);
  } catch(rosbag::BagException) {
    ROS_WARN_STREAM("Could not append to bag file " << filename);
    return false;
  }
  bag.write(topic_name, ros::Time::now(), traj);
  bag.close();
  return true;
}

