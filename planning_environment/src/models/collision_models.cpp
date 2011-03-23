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

planning_environment::CollisionModels::CollisionModels(const std::string &description) : RobotModels(description)
{
  planning_scene_set_ = false;
  loadCollision();
}

planning_environment::CollisionModels::~CollisionModels(void)
{
  deleteAllStaticObjects();
  deleteAllAttachedObjects();
  shapes::deleteShapeVector(collision_map_shapes_);
  delete ode_collision_model_;
}

void planning_environment::CollisionModels::setupModel(collision_space::EnvironmentModel* model)
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
      motion_planning_msgs::CollisionOperation collision_operation;
      collision_operation.object1 = std::string(coll_ops[i]["object1"]);
      collision_operation.object2 = std::string(coll_ops[i]["object2"]);
      std::string operation = std::string(coll_ops[i]["operation"]);
      if(operation == "enable") {
        collision_operation.operation =  motion_planning_msgs::CollisionOperation::ENABLE;
      } else if(operation == "disable") {
        collision_operation.operation =  motion_planning_msgs::CollisionOperation::DISABLE;
      } else {
        ROS_WARN_STREAM("Unrecognized collision operation " << operation << ". Must be enable or disable");
        continue;
      }
      default_collision_operations_.push_back(collision_operation);
    }
  }

  nh_.param(description_ + "_planning/default_robot_padding", default_padd_, 0.01);
  nh_.param(description_ + "_planning/default_robot_scale", default_scale_, 1.0);
  nh_.param(description_ + "_planning/default_object_padding", object_padd_, 0.03);

  const std::vector<planning_models::KinematicModel::LinkModel*>& coll_links = kmodel_->getLinkModelsWithCollisionGeometry();
  
  std::vector<std::string> coll_names;
  for(unsigned int i = 0; i < coll_links.size(); i++) {
    default_link_padding_map_[coll_links[i]->getName()] = default_padd_;
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
        default_link_padding_map_[link] = padding;
      }
    }
  }
  
  //no allowed collisions by default
  collision_space::EnvironmentModel::AllowedCollisionMatrix default_collision_matrix(coll_names,false);

  for(std::vector<motion_planning_msgs::CollisionOperation>::iterator it = default_collision_operations_.begin();
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
    default_collision_matrix.changeEntry(svec1, svec2, (*it).operation != motion_planning_msgs::CollisionOperation::ENABLE);
  }

  model->lock();
  model->setRobotModel(kmodel_, default_collision_matrix, default_link_padding_map_, default_padd_, default_scale_);

  for (unsigned int i = 0 ; i < bounding_planes_.size() / 4 ; ++i)
  {
    shapes::Plane *plane = new shapes::Plane(bounding_planes_[i * 4], bounding_planes_[i * 4 + 1], bounding_planes_[i * 4 + 2], bounding_planes_[i * 4 + 3]);
    model->addObject("bounds", plane);
    ROS_INFO("Added static plane %fx + %fy + %fz + %f = 0 for model", bounding_planes_[i * 4], bounding_planes_[i * 4 + 1], bounding_planes_[i * 4 + 2], bounding_planes_[i * 4 + 3]);
  }
  
  model->unlock();    
}

void planning_environment::CollisionModels::loadCollision()
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
    setupModel(ode_collision_model_);
	
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
planning_environment::CollisionModels::setPlanningScene(const planning_environment_msgs::PlanningScene& planning_scene) {

  if(planning_scene_set_) {
    ROS_WARN("Must revert before setting planning scene again");
    return NULL;
  }
  //anything we've already got should go back to default
  deleteAllStaticObjects();
  deleteAllAttachedObjects();
  revertAllowedCollisionToDefault();
  revertCollisionSpacePaddingToDefault();

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
  planning_scene_set_ = true;

  std::vector<mapping_msgs::CollisionObject> conv_objects;
  std::vector<mapping_msgs::AttachedCollisionObject> conv_att_objects;

  //need to do conversions first so we can delet the planning state
  for(unsigned int i = 0; i < planning_scene.collision_objects.size(); i++) {
    if(planning_scene.collision_objects[i].operation.operation != mapping_msgs::CollisionObjectOperation::ADD) {
      ROS_WARN_STREAM("Planning scene shouldn't have collision operations other than add");
      delete state;
      return NULL;
    }
    conv_objects.push_back(planning_scene.collision_objects[i]);
    convertCollisionObjectToNewWorldFrame(*state, conv_objects.back());
  }
  for(unsigned int i = 0; i < planning_scene.attached_collision_objects.size(); i++) {
    if(planning_scene.attached_collision_objects[i].object.operation.operation != mapping_msgs::CollisionObjectOperation::ADD) {
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

  //TODO - allowed contacts
  //have to call this first, because it reverts the allowed collision matrix
  setCollisionMap(planning_scene.collision_map, true);

  if(planning_scene.link_padding.size() > 0) {
    applyLinkPaddingToCollisionSpace(planning_scene.link_padding);
  }
  if(!planning_scene.allowed_collision_matrix.link_names.empty()) {
    ode_collision_model_->setAlteredCollisionMatrix(convertFromACMMsgToACM(planning_scene.allowed_collision_matrix));
  }

  //now we create again
  state = new planning_models::KinematicState(kmodel_);
  setRobotStateAndComputeTransforms(planning_scene.robot_state, *state);  

  return state;
}

void planning_environment::CollisionModels::revertPlanningScene(planning_models::KinematicState* ks) {
  delete ks;
  planning_scene_set_ = false;
  deleteAllStaticObjects();
  deleteAllAttachedObjects();
  revertAllowedCollisionToDefault();
  revertCollisionSpacePaddingToDefault();
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

  bool header_is_fixed_frame = (header.frame_id == getWorldFrameId());
  bool des_is_fixed_frame = (des_frame_id == getWorldFrameId());

  //Scenario 1(fixed->fixed): if pose is in the world frame and
  //desired is in the world frame, just return
  if(header_is_fixed_frame && des_is_fixed_frame) {
    return true;
  }
  const planning_models::KinematicState::LinkState* header_link_state = state.getLinkState(header.frame_id);
  const planning_models::KinematicState::LinkState* des_link_state = state.getLinkState(des_frame_id);
  
  bool header_is_robot_frame = (header_link_state != NULL);
  bool des_is_robot_frame = (des_link_state != NULL);

  bool header_is_other_frame = !header_is_fixed_frame && !header_is_robot_frame;
  bool des_is_other_frame = !des_is_fixed_frame && !des_is_robot_frame;

  //Scenario 2(*-> other): We can't deal with desired being in a
  //non-fixed frame or relative to the robot.  TODO - decide if this is useful
  if(des_is_other_frame) {
    ROS_WARN_STREAM("Shouldn't be transforming into non-fixed non-robot frame " << des_frame_id);
    return false;
  }

  //Scenario 3 (other->fixed) && 4 (other->robot): we first need to
  //transform into the fixed frame
  if(header_is_other_frame) {
    if(scene_transform_map_.find(header.frame_id) == scene_transform_map_.end()) {
      ROS_WARN_STREAM("Planning scene didn't contain transform " << header.frame_id << " so can't transform");
      return false;
    }
    geometry_msgs::TransformStamped trans = scene_transform_map_.find(header.frame_id)->second;
    
    tf::Transform tf_trans;
    tf::transformMsgToTF(trans.transform, tf_trans);
    btTransform tf_pose;
    tf::poseMsgToTF(ret_pose.pose, tf_pose);
    
    btTransform fpose = tf_trans*tf_pose;

    //assumes that we've already checked that this matched the world frame
    ret_pose.header.frame_id = getWorldFrameId();
    tf::poseTFToMsg(fpose, ret_pose.pose);

    if(des_is_fixed_frame) {
      return true;
    }
  }
  
  //getting tf version of pose
  btTransform bt_pose;
  tf::poseMsgToTF(ret_pose.pose,bt_pose);

  //Scenarios 4(other->robot)/5(fixed->robot): We either started out
  //with a header frame in the fixed frame or converted from a
  //non-robot frame into the fixed frame, and now we need to transform
  //into the desired robot frame given the new world transform
  if(ret_pose.header.frame_id == getWorldFrameId() && des_is_robot_frame) {
    btTransform trans_bt_pose = des_link_state->getGlobalLinkTransform().inverse()*bt_pose;
    tf::poseTFToMsg(trans_bt_pose,ret_pose.pose);
    ret_pose.header.frame_id = des_link_state->getName();
  } else if(header_is_robot_frame && des_is_fixed_frame) {
    //Scenario 6(robot->fixed): Just need to look up robot transform and pre-multiply
    btTransform trans_bt_pose = header_link_state->getGlobalLinkTransform()*bt_pose;
    tf::poseTFToMsg(trans_bt_pose,ret_pose.pose);
    ret_pose.header.frame_id = getWorldFrameId();
  } else if(header_is_robot_frame && des_is_robot_frame) {
    //Scenario 7(robot->robot): Completely tf independent
    btTransform trans_bt_pose = des_link_state->getGlobalLinkTransform().inverse()*(header_link_state->getGlobalLinkTransform()*bt_pose);
    tf::poseTFToMsg(trans_bt_pose,ret_pose.pose);
    ret_pose.header.frame_id = des_link_state->getName();
  } else {
    ROS_WARN("Really shouldn't have gotten here");
    return false;
  }
  return true;
}

bool planning_environment::CollisionModels::convertAttachedCollisionObjectToNewWorldFrame(const planning_models::KinematicState& state,
                                                                                          mapping_msgs::AttachedCollisionObject& att_obj) const
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
                                                                                  mapping_msgs::CollisionObject& obj) const
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
                                                                                     motion_planning_msgs::Constraints& constraints,
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

bool planning_environment::CollisionModels::addStaticObject(const mapping_msgs::CollisionObject& obj)
{
  std::vector<shapes::Shape*> shapes;
  std::vector<btTransform> poses;
  for(unsigned int i = 0; i < obj.shapes.size(); i++) {
    shapes::Shape *shape = constructObject(obj.shapes[i]);
    if(!shape) {
      ROS_WARN_STREAM("Something wrong with shape");
      return false;
    }
    shapes.push_back(shape);
    btTransform pose;
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
                                                            const std::vector<btTransform>& poses,
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
  delete static_object_map_.find(name)->second;
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

void planning_environment::CollisionModels::setCollisionMap(const mapping_msgs::CollisionMap& map, 
                                                            bool mask_before_insertion) {
  std::vector<shapes::Shape*> shapes(map.boxes.size());
  std::vector<btTransform> poses;
  for(unsigned int i = 0; i < map.boxes.size(); i++) {
    shapes[i] = new shapes::Box(map.boxes[i].extents.x, map.boxes[i].extents.y, map.boxes[i].extents.z);
    btTransform pose;
    pose.setOrigin(btVector3(map.boxes[i].center.x, map.boxes[i].center.y, map.boxes[i].center.z));
    pose.setRotation(btQuaternion(btVector3(map.boxes[i].axis.x, map.boxes[i].axis.y, map.boxes[i].axis.z), map.boxes[i].angle));
    poses.push_back(pose);
  }
  setCollisionMap(shapes, poses, mask_before_insertion);
}

void planning_environment::CollisionModels::setCollisionMap(std::vector<shapes::Shape*>& shapes,
                                                            const std::vector<btTransform>& poses,
                                                            bool mask_before_insertion)
{
  shapes::deleteShapeVector(collision_map_shapes_);
  collision_map_shapes_ = shapes::cloneShapeVector(shapes);
  collision_map_poses_ = poses;
  std::vector<btTransform> masked_poses = poses;
  if(mask_before_insertion) {
    maskAndDeleteShapeVector(shapes,masked_poses); 
  } 
  ode_collision_model_->lock();
  ode_collision_model_->clearObjects(COLLISION_MAP_NAME);
  if(shapes.size() > 0) {
    ode_collision_model_->addObjects(COLLISION_MAP_NAME, shapes, masked_poses);
  }
  ode_collision_model_->unlock();
}

void planning_environment::CollisionModels::remaskCollisionMap() {
  std::vector<shapes::Shape*> shapes = shapes::cloneShapeVector(collision_map_shapes_);
  std::vector<btTransform> masked_poses = collision_map_poses_;
  maskAndDeleteShapeVector(shapes,masked_poses); 
  ode_collision_model_->lock();
  ode_collision_model_->clearObjects(COLLISION_MAP_NAME);
  ode_collision_model_->addObjects(COLLISION_MAP_NAME, shapes, masked_poses);
  ode_collision_model_->unlock();
}

void planning_environment::CollisionModels::maskAndDeleteShapeVector(std::vector<shapes::Shape*>& shapes,
                                                                     std::vector<btTransform>& poses)
{
  bodiesLock();
  std::vector<bool> mask;
  std::vector<bodies::BodyVector*> static_object_vector;
  for(std::map<std::string, bodies::BodyVector*>::iterator it = static_object_map_.begin();
      it != static_object_map_.end();
      it++) {
    static_object_vector.push_back(it->second);
  }
  bodies::maskPosesInsideBodyVectors(poses, static_object_vector, mask, true);
  std::vector<btTransform> ret_poses;
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

bool planning_environment::CollisionModels::addAttachedObject(const mapping_msgs::AttachedCollisionObject& att)
{
  const mapping_msgs::CollisionObject& obj = att.object;
  std::vector<shapes::Shape*> shapes;
  std::vector<btTransform> poses;
  for(unsigned int i = 0; i < obj.shapes.size(); i++) {
    shapes::Shape *shape = constructObject(obj.shapes[i]);
    if(!shape) {
      ROS_WARN_STREAM("Something wrong with shape");
      return false;
    }
    shapes.push_back(shape);
    btTransform pose;
    tf::poseMsgToTF(obj.poses[i], pose);
    poses.push_back(pose);
  }
  double padding = object_padd_;
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
                                                              const std::vector<btTransform>& poses,
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

  link_attached_objects_[link_name][object_name] = new bodies::BodyVector(shapes, poses, padding);  

  std::vector<std::string> modded_touch_links = touch_links;
  if(find(touch_links.begin(), touch_links.end(), link_name) == touch_links.end()) {
    modded_touch_links.push_back(link_name);
  }
  planning_models::KinematicModel::AttachedBodyModel* ab = 
    new planning_models::KinematicModel::AttachedBodyModel(link, object_name,
                                                           poses,
                                                           modded_touch_links,
                                                           shapes);
  kmodel_->addAttachedBodyModel(link->getName(),ab);
  ode_collision_model_->updateAttachedBodies();
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
  ode_collision_model_->updateAttachedBodies();
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
    kmodel_->clearAllAttachedBodyModels();
  } else {
    kmodel_->clearLinkAttachedBodyModels(link_name);
  }
  ode_collision_model_->lock();
  ode_collision_model_->updateAttachedBodies();
  ode_collision_model_->unlock();
  bodiesUnlock();
}

bool planning_environment::CollisionModels::convertStaticObjectToAttachedObject(const std::string& object_name,
                                                                                const std::string& link_name,
                                                                                const std::vector<std::string>& touch_links)
{
  const planning_models::KinematicModel::LinkModel *link = kmodel_->getLinkModel(link_name);
  if(link == NULL) {
    ROS_WARN_STREAM("No link " << link_name << " for attaching " << object_name);
    return false;
  }
  if(static_object_map_.find(object_name) == static_object_map_.end()) {
    ROS_WARN_STREAM("No static object named " << object_name << " to convert");
    return false;
  }
  bodiesLock();
  link_attached_objects_[link_name][object_name] = static_object_map_[object_name];
  static_object_map_.erase(object_name);

  std::vector<std::string> modded_touch_links = touch_links;
  if(find(touch_links.begin(), touch_links.end(), link_name) == touch_links.end()) {
    modded_touch_links.push_back(link_name);
  }

  std::vector<btTransform> poses;
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
  planning_models::KinematicModel::AttachedBodyModel* ab = 
    new planning_models::KinematicModel::AttachedBodyModel(link, object_name,
                                                           poses,
                                                           modded_touch_links,
                                                           shapes);
  kmodel_->addAttachedBodyModel(link->getName(),ab);

  ode_collision_model_->clearObjects(object_name);
  ode_collision_model_->updateAttachedBodies();
  bodiesUnlock();
  return true;
}

bool planning_environment::CollisionModels::convertAttachedObjectToStaticObject(const std::string& object_name,
                                                                                const std::string& link_name)
{
  const planning_models::KinematicModel::LinkModel *link = kmodel_->getLinkModel(link_name);
  if(link == NULL) {
    ROS_WARN_STREAM("No link " << link_name << " with attached object " << object_name);
    return false;
  }
  if(link_attached_objects_.find(link_name) == link_attached_objects_.end() ||
     link_attached_objects_[link_name].find(object_name) == link_attached_objects_[link_name].end()) 
  {
    ROS_WARN_STREAM("No attached body " << object_name << " attached to link " << link_name);
    return false;
  }
  bodiesLock();

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
  std::map<std::string, std::vector<btTransform> > att_pose_map;
  ode_collision_model_->getAttachedBodyPoses(att_pose_map);
  if(att_pose_map.find(att->getName()) == att_pose_map.end()) {
    ROS_WARN_STREAM("No poses for " << att->getName());
    shapes::deleteShapeVector(shapes);
    return false;
  }
  ode_collision_model_->addObjects(object_name, shapes, att_pose_map[att->getName()]);  
  kmodel_->clearLinkAttachedBodyModel(link_name, object_name);

  ode_collision_model_->updateAttachedBodies();
  bodiesUnlock();
  return true;
}

void planning_environment::CollisionModels::applyLinkPaddingToCollisionSpace(const std::vector<motion_planning_msgs::LinkPadding>& link_padding) {
  if(link_padding.empty()) return;
  
  std::map<std::string, double> link_padding_map;

  for(std::vector<motion_planning_msgs::LinkPadding>::const_iterator it = link_padding.begin();
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
  
  ode_collision_model_->setAlteredLinkPadding(link_padding_map);  
}

void planning_environment::CollisionModels::getCurrentLinkPadding(std::vector<motion_planning_msgs::LinkPadding>& link_padding)
{
  convertFromLinkPaddingMapToLinkPaddingVector(ode_collision_model_->getCurrentLinkPaddingMap(), link_padding);
}

bool planning_environment::CollisionModels::applyOrderedCollisionOperationsToCollisionSpace(const motion_planning_msgs::OrderedCollisionOperations &ord, bool print) {

  ode_collision_model_->lock();
  collision_space::EnvironmentModel::AllowedCollisionMatrix acm = ode_collision_model_->getDefaultAllowedCollisionMatrix();;
  ode_collision_model_->unlock();

  std::vector<std::string> o_strings;
  for(std::map<std::string, bodies::BodyVector*>::const_iterator it = static_object_map_.begin();
      it != static_object_map_.end();
      it++) {
    o_strings.push_back(it->first);
  }
  o_strings.push_back(COLLISION_MAP_NAME);
  
  kmodel_->sharedLock();  
  std::vector<std::string> a_strings;
  const std::vector<const planning_models::KinematicModel::AttachedBodyModel*>& att_vec = kmodel_->getAttachedBodyModels();
  for(unsigned int i = 0; i < att_vec.size(); i++) 
  {
    a_strings.push_back(att_vec[i]->getName());
  }
  kmodel_->sharedUnlock();

  bool ok = applyOrderedCollisionOperationsListToACM(ord, o_strings, a_strings, kmodel_, acm);

  if(!ok) {
    ROS_WARN_STREAM("Bad collision operations");
  }

  if(print) {
    //printAllowedCollisionMatrix(acm);
  }

  ode_collision_model_->setAlteredCollisionMatrix(acm);
  return true;
}

bool planning_environment::CollisionModels::disableCollisionsForNonUpdatedLinks(const std::string& group_name)
{
  kmodel_->sharedLock();  
  const planning_models::KinematicModel::JointModelGroup* joint_model_group = kmodel_->getModelGroup(group_name);
  collision_space::EnvironmentModel::AllowedCollisionMatrix acm = ode_collision_model_->getCurrentAllowedCollisionMatrix();
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
  ode_collision_model_->setAlteredCollisionMatrix(acm);
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

bool planning_environment::CollisionModels::computeAllowedContact(const motion_planning_msgs::AllowedContactSpecification& allowed_contact,
                                                                  collision_space::EnvironmentModel::AllowedContact& allowedContact) const
{
  shapes::Shape *shape = constructObject(allowed_contact.shape);
  if (shape)
  {
    boost::shared_ptr<bodies::Body> body(bodies::createBodyFromShape(shape));
    geometry_msgs::PoseStamped pose;
    //assumes the allowed contact is in the correct frame
    //tf_->transformPose(getWorldFrameId(), allowed_contact.pose_stamped, pose);
    btTransform tr;
    tf::poseMsgToTF(allowed_contact.pose_stamped.pose, tr);
    body->setPose(tr);
    allowedContact.bound = body;
    allowedContact.links = allowed_contact.link_names;
    allowedContact.depth = allowed_contact.penetration_depth;
    delete shape;
    return true;
  }
  else
    return false;
}

void planning_environment::CollisionModels::getCollisionSpaceCollisionMap(mapping_msgs::CollisionMap& cmap) const
{
  ode_collision_model_->lock();
  cmap.header.frame_id = getWorldFrameId();
  cmap.header.stamp = ros::Time::now();
  cmap.boxes.clear();
  
  const collision_space::EnvironmentObjects::NamespaceObjects &no = ode_collision_model_->getObjects()->getObjects(COLLISION_MAP_NAME);
  const unsigned int n = no.shape.size();
  for (unsigned int i = 0 ; i < n ; ++i) {
    if (no.shape[i]->type == shapes::BOX) {
      const shapes::Box* box = static_cast<const shapes::Box*>(no.shape[i]);
      mapping_msgs::OrientedBoundingBox obb;
      obb.extents.x = box->size[0];
      obb.extents.y = box->size[1];
      obb.extents.z = box->size[2];
      const btVector3 &c = no.shape_pose[i].getOrigin();
      obb.center.x = c.x();
      obb.center.y = c.y();
      obb.center.z = c.z();
      const btQuaternion q = no.shape_pose[i].getRotation();
      obb.angle = q.getAngle();
      const btVector3 axis = q.getAxis();
      obb.axis.x = axis.x();
      obb.axis.y = axis.y();
      obb.axis.z = axis.z();
      cmap.boxes.push_back(obb);
    }
  }
  ode_collision_model_->unlock();
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

void planning_environment::CollisionModels::getCollisionSpaceAllowedCollisions(planning_environment_msgs::AllowedCollisionMatrix& ret_matrix) const {

  convertFromACMToACMMsg(ode_collision_model_->getCurrentAllowedCollisionMatrix(),
                         ret_matrix);
}

void planning_environment::CollisionModels::getCollisionSpaceCollisionObjects(std::vector<mapping_msgs::CollisionObject> &omap) const
{
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

    mapping_msgs::CollisionObject o;
    o.header.frame_id = getWorldFrameId();
    o.header.stamp = ros::Time::now();
    o.id = ns[i];
    o.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
    for (unsigned int j = 0 ; j < n ; ++j) {
      geometric_shapes_msgs::Shape obj;
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
}

void planning_environment::CollisionModels::getCollisionSpaceAttachedCollisionObjects(std::vector<mapping_msgs::AttachedCollisionObject> &avec) const
{
  avec.clear();

  kmodel_->sharedLock();
  ode_collision_model_->lock();

  std::vector<const planning_models::KinematicModel::AttachedBodyModel*> att_vec = kmodel_->getAttachedBodyModels();
  for(unsigned int i = 0; i < att_vec.size(); i++) 
  {
    mapping_msgs::AttachedCollisionObject ao;
    ao.object.header.frame_id = att_vec[i]->getAttachedLinkModel()->getName();
    ao.object.header.stamp = ros::Time::now();
    ao.link_name = att_vec[i]->getAttachedLinkModel()->getName();
    double attached_padd = ode_collision_model_->getCurrentLinkPadding("attached");
    for(unsigned int j = 0; j < att_vec[i]->getShapes().size(); j++) {
      geometric_shapes_msgs::Shape shape;
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
  kmodel_->sharedUnlock();
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

void planning_environment::CollisionModels::getAllCollisionsForState(const planning_models::KinematicState& state,
                                                                     std::vector<planning_environment_msgs::ContactInformation>& contacts,
                                                                     unsigned int num_per_pair) 
{
  ode_collision_model_->lock();
  ode_collision_model_->updateRobotModel(&state);
  std::vector<collision_space::EnvironmentModel::AllowedContact> allowed_contacts;
  std::vector<collision_space::EnvironmentModel::Contact> coll_space_contacts;
  ros::WallTime n1 = ros::WallTime::now();
  ode_collision_model_->getAllCollisionContacts(allowed_contacts,
                                                coll_space_contacts,
                                                num_per_pair);
  ros::WallTime n2 = ros::WallTime::now();
  ROS_DEBUG_STREAM("Got " << coll_space_contacts.size() << " collisions in " << (n2-n1).toSec());
  for(unsigned int i = 0; i < coll_space_contacts.size(); i++) {
    planning_environment_msgs::ContactInformation contact_info;
    contact_info.header.frame_id = getWorldFrameId();
    collision_space::EnvironmentModel::Contact& contact = coll_space_contacts[i];
    contact_info.contact_body_1 = contact.body_name_1;
    contact_info.contact_body_2 = contact.body_name_2;
    if(contact.body_type_1 == collision_space::EnvironmentModel::LINK) {
      contact_info.body_type_1 = planning_environment_msgs::ContactInformation::ROBOT_LINK;
    } else if(contact.body_type_1 == collision_space::EnvironmentModel::ATTACHED) {
      contact_info.body_type_1 = planning_environment_msgs::ContactInformation::ATTACHED_BODY;
    } else {
      contact_info.body_type_1 = planning_environment_msgs::ContactInformation::OBJECT;      
    }
    if(contact.body_type_2 == collision_space::EnvironmentModel::LINK) {
      contact_info.body_type_2 = planning_environment_msgs::ContactInformation::ROBOT_LINK;
    } else if(contact.body_type_2 == collision_space::EnvironmentModel::ATTACHED) {
      contact_info.body_type_2 = planning_environment_msgs::ContactInformation::ATTACHED_BODY;
    } else {
      contact_info.body_type_2 = planning_environment_msgs::ContactInformation::OBJECT;      
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
                                                                  motion_planning_msgs::ArmNavigationErrorCodes& error_code,
                                                                  const motion_planning_msgs::Constraints goal_constraints,
                                                                  const motion_planning_msgs::Constraints path_constraints)
{
  if(!state.areJointsWithinBounds(joint_names)) {
    for(unsigned int j = 0; j < joint_names.size(); j++) {
      if(!state.isJointWithinBounds(joint_names[j])) {
        std::pair<double, double> bounds; 
        state.getJointState(joint_names[j])->getJointModel()->getVariableBounds(joint_names[j], bounds);
        ROS_INFO_STREAM("Joint " << joint_names[j] << " out of bounds. " <<
                        " value: " << state.getJointState(joint_names[j])->getJointStateValues()[0] << 
                        " low: " << bounds.first << " high: " << bounds.second);
      }
    }
    error_code.val = error_code.JOINT_LIMITS_VIOLATED;
    return false;
  }
  if(!doesKinematicStateObeyConstraints(state, path_constraints)) {
    error_code.val = error_code.PATH_CONSTRAINTS_VIOLATED;
    return false;
  }
  if(!doesKinematicStateObeyConstraints(state, goal_constraints)) {
    error_code.val = error_code.GOAL_CONSTRAINTS_VIOLATED;
    return false;
  }
  if(isKinematicStateInCollision(state)) {
    error_code.val = error_code.COLLISION_CONSTRAINTS_VIOLATED;    
    return false;
  }
  return true;
}

bool planning_environment::CollisionModels::isJointTrajectoryValid(const planning_environment_msgs::PlanningScene& planning_scene,
                                                                   const trajectory_msgs::JointTrajectory &trajectory,
                                                                   const motion_planning_msgs::Constraints& goal_constraints,
                                                                   const motion_planning_msgs::Constraints& path_constraints,
                                                                   motion_planning_msgs::ArmNavigationErrorCodes& error_code,
                                                                   std::vector<motion_planning_msgs::ArmNavigationErrorCodes>& trajectory_error_codes,
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
                                                                   const motion_planning_msgs::Constraints& goal_constraints,
                                                                   const motion_planning_msgs::Constraints& path_constraints,
                                                                   motion_planning_msgs::ArmNavigationErrorCodes& error_code,
                                                                   std::vector<motion_planning_msgs::ArmNavigationErrorCodes>& trajectory_error_codes,
                                                                   const bool evaluate_entire_trajectory)  
{
  error_code.val = error_code.SUCCESS;
  
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
  motion_planning_msgs::Constraints emp_goal_constraints;
  for(unsigned int i = 0; i < trajectory.points.size(); i++) {
    motion_planning_msgs::ArmNavigationErrorCodes suc;
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

// bool planning_environment::CollisionModels::isRobotTrajectoryValid(const planning_environment_msgs::PlanningScene& planning_scene,
//                                                                    const motion_planning_msgs::RobotTrajectory& trajectory,
//                                                                    const motion_planning_msgs::Constraints& goal_constraints,
//                                                                    const motion_planning_msgs::Constraints& path_constraints,
//                                                                    motion_planning_msgs::ArmNavigationErrorCodes& error_code,
//                                                                    std::vector<motion_planning_msgs::ArmNavigationErrorCodes>& trajectory_error_codes,
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
//                                                                    const motion_planning_msgs::RobotTrajectory& trajectory,
//                                                                    const motion_planning_msgs::Constraints& goal_constraints,
//                                                                    const motion_planning_msgs::Constraints& path_constraints,
//                                                                    motion_planning_msgs::ArmNavigationErrorCodes& error_code,
//                                                                    std::vector<motion_planning_msgs::ArmNavigationErrorCodes>& trajectory_error_codes,
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
  mark.color = color;
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
      const btVector3 &c = no.shape_pose[i].getOrigin();
      geometry_msgs::Point point;
      point.x = c.x();
      point.y = c.y();
      point.z = c.z();
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
  std::vector<planning_environment_msgs::ContactInformation> coll_info_vec;
  getAllCollisionsForState(state,coll_info_vec,1);

  std::map<std::string, unsigned> ns_counts;
  for(unsigned int i = 0; i < coll_info_vec.size(); i++) {
    std::string ns_name;
    if(coll_info_vec[i].body_type_1 == planning_environment_msgs::ContactInformation::ROBOT_LINK ||
       coll_info_vec[i].body_type_1 == planning_environment_msgs::ContactInformation::OBJECT) {
      ns_name = coll_info_vec[i].contact_body_1;
    } else {
      ns_name = coll_info_vec[i].attached_body_1;
    }
    ns_name +="+";
    if(coll_info_vec[i].body_type_2 == planning_environment_msgs::ContactInformation::ROBOT_LINK ||
       coll_info_vec[i].body_type_2 == planning_environment_msgs::ContactInformation::OBJECT) {
      ns_name += coll_info_vec[i].contact_body_2;
    } else {
      ns_name += coll_info_vec[i].attached_body_2;
    }
    if(ns_counts.find(ns_name) == ns_counts.end()) {
      ns_counts[ns_name] = 0;
    } else {
      ns_counts[ns_name]++;
    }
    visualization_msgs::Marker mk;
    mk.header.stamp = ros::Time::now();
    mk.header.frame_id = getWorldFrameId();
    mk.ns = ns_name;
    mk.id = ns_counts[ns_name];
    mk.type = visualization_msgs::Marker::SPHERE;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.position.x = coll_info_vec[i].position.x;
    mk.pose.position.y = coll_info_vec[i].position.y;
    mk.pose.position.z = coll_info_vec[i].position.z;
    mk.pose.orientation.w = 1.0;
    
    mk.scale.x = mk.scale.y = mk.scale.z = 0.035;

    mk.color.a = color.a;
    if(mk.color.a == 0.0) {
      mk.color.a = 1.0;
    }
    mk.color.r = color.r;
    mk.color.g = color.g;
    mk.color.b = color.b;
    
    mk.lifetime = lifetime;
    arr.markers.push_back(mk);
  }
}

void planning_environment::CollisionModels::getStaticCollisionObjectMarkers(visualization_msgs::MarkerArray& arr,
                                                                            const std::string& name,
                                                                            const std_msgs::ColorRGBA& color,
                                                                            const ros::Duration& lifetime) const
{
  std::vector<mapping_msgs::CollisionObject> static_objects;
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
                                                                              const ros::Duration& lifetime)
{
  ode_collision_model_->lock();
  std::vector<mapping_msgs::AttachedCollisionObject> attached_objects;
  getCollisionSpaceAttachedCollisionObjects(attached_objects);

  ode_collision_model_->updateRobotModel(&state);

  std::map<std::string, std::vector<btTransform> > att_pose_map;
  ode_collision_model_->getAttachedBodyPoses(att_pose_map);

  for(unsigned int i = 0; i < attached_objects.size(); i++) {
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
                                                                       planning_environment_msgs::PlanningScene& planning_scene)
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

void planning_environment::CollisionModels::getRobotMeshResourceMarkersGivenState(const planning_models::KinematicState& state,
                                                                                  visualization_msgs::MarkerArray& arr,
                                                                                  const std_msgs::ColorRGBA& color,
                                                                                  const std::string& name, 
                                                                                  const ros::Duration& lifetime,
                                                                                  const std::vector<std::string>* names) const
{  
  boost::shared_ptr<urdf::Model> robot_model = getParsedDescription();

  std::vector<std::string> link_names;
  if(names == NULL) {
    kmodel_->getLinkModelNames(link_names);
  } else {
    link_names = *names;
  }

  for(unsigned int i = 0; i < link_names.size(); i++) {
    boost::shared_ptr<const urdf::Link> urdf_link = robot_model->getLink(link_names[i]);
    if(!urdf_link) {
      ROS_INFO_STREAM("Invalid urdf name " << link_names[i]);
      continue;
    }
    if(!urdf_link->collision) {
      continue;
    }
    const urdf::Geometry *geom = urdf_link->collision->geometry.get();
    if(!geom) {
      continue;
    }
    const urdf::Mesh *mesh = dynamic_cast<const urdf::Mesh*>(geom);
    if(!mesh) {
      continue;
    }
    if(mesh->filename.empty()) {
      continue;
    }
    const planning_models::KinematicState::LinkState* ls = state.getLinkState(link_names[i]);
    if(ls == NULL) {
      ROS_WARN_STREAM("No link state for name " << names << " though there's a mesh");
      continue;
    }
    visualization_msgs::Marker mark;
    mark.header.frame_id = getWorldFrameId();
    mark.header.stamp = ros::Time::now();
    mark.ns = name;
    mark.id = i;
    mark.type = mark.MESH_RESOURCE;
    mark.scale.x = 1.0;
    mark.scale.y = 1.0;
    mark.scale.z = 1.0;
    mark.color = color;
    mark.mesh_resource = mesh->filename;
    mark.lifetime = lifetime;
    tf::poseTFToMsg(ls->getGlobalCollisionBodyTransform(),mark.pose); 
    arr.markers.push_back(mark);
  }
}

void planning_environment::CollisionModels::getRobotTrimeshMarkersGivenState(const planning_models::KinematicState& state,
                                                                             visualization_msgs::MarkerArray& arr,
                                                                             bool use_default_padding,
                                                                             const ros::Duration& lifetime) const
{
  unsigned int count = 0;
  for(unsigned int i = 0; i < state.getLinkStateVector().size(); i++) {
    const planning_models::KinematicState::LinkState* ls = state.getLinkStateVector()[i];
    if(ls->getLinkModel()->getLinkShape() == NULL) continue;
    const shapes::Mesh *mesh = dynamic_cast<const shapes::Mesh*>(ls->getLinkModel()->getLinkShape());
    if(mesh == NULL) continue;
    if (mesh->vertexCount > 0 && mesh->triangleCount > 0)
    {	
      const btTransform& trans = ls->getGlobalCollisionBodyTransform();

      visualization_msgs::Marker mesh_mark;
      mesh_mark.header.frame_id = getWorldFrameId();
      mesh_mark.header.stamp = ros::Time::now();
      mesh_mark.ns = ls->getName()+"_trimesh";
      mesh_mark.id = count++;
      mesh_mark.type = mesh_mark.LINE_LIST;
      mesh_mark.scale.x = mesh_mark.scale.y = mesh_mark.scale.z = .001;	     
      mesh_mark.color.r = 0.0;
      mesh_mark.color.g = 0.0;
      mesh_mark.color.b = 1.0;
      mesh_mark.color.a = 1.0;
      double padding = 0.0;
      if(use_default_padding) {
        if(getDefaultLinkPaddingMap().find(ls->getName()) !=
           getDefaultLinkPaddingMap().end()) {
          padding = getDefaultLinkPaddingMap().find(ls->getName())->second;
        }
      }
      double *vertices = new double[mesh->vertexCount* 3];
      double sx = 0.0, sy = 0.0, sz = 0.0;
      for (unsigned int i = 0 ; i < mesh->vertexCount ; ++i)
      {
        unsigned int i3 = i * 3;
        vertices[i3] = mesh->vertices[i3];
        vertices[i3 + 1] = mesh->vertices[i3 + 1];
        vertices[i3 + 2] = mesh->vertices[i3 + 2];
        sx += vertices[i3];
        sy += vertices[i3 + 1];
        sz += vertices[i3 + 2];
      }
      // the center of the mesh
      sx /= (double)mesh->vertexCount;
      sy /= (double)mesh->vertexCount;
      sz /= (double)mesh->vertexCount;

      // scale the mesh
      for (unsigned int i = 0 ; i < mesh->vertexCount ; ++i) {
        unsigned int i3 = i * 3;
          
        // vector from center to the vertex
        double dx = vertices[i3] - sx;
        double dy = vertices[i3 + 1] - sy;
        double dz = vertices[i3 + 2] - sz;
	
        // length of vector
        //double norm = sqrt(dx * dx + dy * dy + dz * dz);
	
        double ndx = ((dx > 0) ? dx+padding : dx-padding);
        double ndy = ((dy > 0) ? dy+padding : dy-padding);
        double ndz = ((dz > 0) ? dz+padding : dz-padding);
        
        // the new distance of the vertex from the center
        //double fact = scale + padding/norm;
        vertices[i3] = sx + ndx; //dx * fact;
        vertices[i3 + 1] = sy + ndy; //dy * fact;
        vertices[i3 + 2] = sz + ndz; //dz * fact;		    
      }
      for (unsigned int j = 0 ; j < mesh->triangleCount; ++j) {
        unsigned int t1ind = mesh->triangles[3*j];
        unsigned int t2ind = mesh->triangles[3*j + 1];
        unsigned int t3ind = mesh->triangles[3*j + 2];

        btVector3 vec1(vertices[t1ind*3],
                       vertices[t1ind*3+1],
                       vertices[t1ind*3+2]);

        btVector3 vec2(vertices[t2ind*3],
                       vertices[t2ind*3+1],
                       vertices[t2ind*3+2]);

        btVector3 vec3(vertices[t3ind*3],
                       vertices[t3ind*3+1],
                       vertices[t3ind*3+2]);

        vec1 = trans*vec1;
        vec2 = trans*vec2;
        vec3 = trans*vec3;

        geometry_msgs::Point pt1;
        pt1.x = vec1.x();
        pt1.y = vec1.y();
        pt1.z = vec1.z();

        geometry_msgs::Point pt2;
        pt2.x = vec2.x();
        pt2.y = vec2.y();
        pt2.z = vec2.z();

        geometry_msgs::Point pt3;
        pt3.x = vec3.x();
        pt3.y = vec3.y();
        pt3.z = vec3.z();
        
        mesh_mark.points.push_back(pt1);
        mesh_mark.points.push_back(pt2);
        
        mesh_mark.points.push_back(pt1);
        mesh_mark.points.push_back(pt3);
        
        mesh_mark.points.push_back(pt2);
        mesh_mark.points.push_back(pt3);
      }
      arr.markers.push_back(mesh_mark);
      delete[] vertices;
    }
  }
}

void planning_environment::CollisionModels::writePlanningSceneBag(const std::string& filename,
                                                                  const planning_environment_msgs::PlanningScene& planning_scene) const
{
  rosbag::Bag bag;
  bag.open(filename, rosbag::bagmode::Write);
  
  ros::Time t = planning_scene.robot_state.joint_state.header.stamp;
  bag.write("planning_scene", t, planning_scene);
  bag.close();
}

bool planning_environment::CollisionModels::readPlanningSceneBag(const std::string& filename,
                                                                 planning_environment_msgs::PlanningScene& planning_scene) const
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
    planning_environment_msgs::PlanningScene::ConstPtr ps = m.instantiate<planning_environment_msgs::PlanningScene>();
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
