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
#include <collision_space/environmentBullet.h>
#include <sstream>
#include <vector>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/body_operations.h>

planning_environment::CollisionModels::CollisionModels(const std::string &description) : RobotModels(description)
{
  loadCollision(group_link_union_);
}

planning_environment::CollisionModels::CollisionModels(const std::string &description, const std::vector<std::string> &links) : RobotModels(description)
{
  loadCollision(links);
}

planning_environment::CollisionModels::~CollisionModels(void)
{
  //TODOTODOTODO
}

/** \brief Reload the robot description and recreate the model */	
void planning_environment::CollisionModels::reload(void)
{
  RobotModels::reload();
  ode_collision_model_.reset();
  loadCollision(group_link_union_);
}


void planning_environment::CollisionModels::setupModel(boost::shared_ptr<collision_space::EnvironmentModel> &model, const std::vector<std::string>& links)
{
  XmlRpc::XmlRpcValue coll_ops;

  //first we do default collision operations
  if(!nh_.hasParam(description_ + "_collision/default_collision_operations")) {
    ROS_WARN("No default collision operations specified");
  } else {
  
    nh_.getParam(description_ + "_collision/default_collision_operations", coll_ops);
    
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

  //ROS_INFO_STREAM("Padd is " << padd_);

  //now we do paddings in the private namespace
  ros::NodeHandle priv("~");

  priv.param("default_robot_padding", default_padd_, 0.01);
  priv.param("robot_scale", default_scale_, 1.0);

  //ROS_INFO_STREAM("Padd is " << padd_);

  for(std::vector<std::string>::const_iterator it = links.begin();
      it != links.end();
      it++) {
    default_link_padding_map_[*it] = default_padd_;
  }

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
        std::vector<std::string> svec1;
        if(planning_group_links_.find(link) != planning_group_links_.end()) {
          svec1 = planning_group_links_[link];
        } else {
          svec1.push_back(link);
        }
        for(std::vector<std::string>::iterator it = svec1.begin();
            it != svec1.end();
            it++) {
          default_link_padding_map_[*it] = padding;
        }
      }
    }
  }
  
  model->lock();
  std::vector<std::string> links_with_collision = links;
  std::vector<std::string>::iterator lit = links_with_collision.begin();
  while(lit != links_with_collision.end()) {
    if(kmodel_->getLinkModel(*lit)->getLinkShape() == NULL) {
      lit = links_with_collision.erase(lit);
    } else {
      lit++;
    }
  }
  model->setRobotModel(kmodel_, links_with_collision, default_link_padding_map_, default_padd_,default_scale_);

  for(std::vector<motion_planning_msgs::CollisionOperation>::iterator it = default_collision_operations_.begin();
      it != default_collision_operations_.end();
      it++) {
    std::vector<std::string> svec1;
    std::vector<std::string> svec2;
    if(planning_group_links_.find((*it).object1) != planning_group_links_.end()) {
      svec1 = planning_group_links_[(*it).object1];
    } else {
      svec1.push_back((*it).object1);
    }
    if(planning_group_links_.find((*it).object2) != planning_group_links_.end()) {
      svec2 = planning_group_links_[(*it).object2];
    } else {
      svec2.push_back((*it).object2);
    }
    if((*it).operation == motion_planning_msgs::CollisionOperation::ENABLE) {
      model->addSelfCollisionGroup(svec1,svec2);
    } else {
      model->removeSelfCollisionGroup(svec1,svec2);
    }
  }
  for (unsigned int i = 0 ; i < bounding_planes_.size() / 4 ; ++i)
  {
    shapes::Plane *plane = new shapes::Plane(bounding_planes_[i * 4], bounding_planes_[i * 4 + 1], bounding_planes_[i * 4 + 2], bounding_planes_[i * 4 + 3]);
    model->addObject("bounds", plane);
    ROS_INFO("Added static plane %fx + %fy + %fz + %f = 0 for model %p", bounding_planes_[i * 4], bounding_planes_[i * 4 + 1], bounding_planes_[i * 4 + 2], bounding_planes_[i * 4 + 3], model.get());
  }
  
  model->unlock();    
}

void planning_environment::CollisionModels::loadCollision(const std::vector<std::string> &links)
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
    ode_collision_model_ = boost::shared_ptr<collision_space::EnvironmentModel>(new collision_space::EnvironmentModelODE());
    setupModel(ode_collision_model_,links);
	
    //	bullet_collision_model_ = boost::shared_ptr<collision_space::EnvironmentModel>(new collision_space::EnvironmentModelBullet());
    //	setupModel(bullet_collision_model_, links);
  } else {
    ROS_WARN("Models not loaded");
  }
}

///
/// Functions for updating state
///

bool planning_environment::CollisionModels::setPlanningScene(const motion_planning_msgs::RobotState& complete_robot_state,
                                                             const planning_environment_msgs::AllowedCollisionMatrix& allowed_collision_matrix,
                                                             const std::vector<motion_planning_msgs::AllowedContactSpecification>& transformed_allowed_contacts,
                                                             const std::vector<motion_planning_msgs::LinkPadding>& all_link_paddings,
                                                             const std::vector<mapping_msgs::CollisionObject>& all_collision_objects,
                                                             const std::vector<mapping_msgs::AttachedCollisionObject>& all_attached_collision_objects,
                                                             const mapping_msgs::CollisionMap& unmasked_collision_map,
                                                             planning_models::KinematicState& state){
  bool complete = setRobotStateAndComputeTransforms(complete_robot_state, state);
  if(!complete) {
    ROS_WARN_STREAM("Incomplete robot state in setPlanningScene");
    return false;
  }
  for(unsigned int i = 0; i < all_collision_objects.size(); i++) {
    if(all_collision_objects[i].header.frame_id != getWorldFrameId()) {
      ROS_WARN_STREAM("Can't cope with objects not in " << getWorldFrameId());
      return false;
    }
    if(all_collision_objects[i].operation.operation != mapping_msgs::CollisionObjectOperation::ADD) {
      ROS_WARN_STREAM("Planning scene shouldn't have collision operations other than add");
      return false;
    }
    addStaticObject(all_collision_objects[i]);
  }
  for(unsigned int i = 0; i < all_attached_collision_objects.size(); i++) {
    if(all_attached_collision_objects[i].object.header.frame_id != getWorldFrameId()) {
      ROS_WARN_STREAM("Can't cope with objects not in " << getWorldFrameId());
      return false;
    }
    if(all_attached_collision_objects[i].object.operation.operation != mapping_msgs::CollisionObjectOperation::ADD) {
      ROS_WARN_STREAM("Planning scene shouldn't have collision operations other than add");
      return false;
    }
    addAttachedObject(all_attached_collision_objects[i]);
  }
  applyLinkPaddingToCollisionSpace(all_link_paddings);
  //TODO - allowed contacts, allowed collision matrix
  setCollisionMap(unmasked_collision_map, true);
  return true;
}

void planning_environment::CollisionModels::updateRobotModelPose(const planning_models::KinematicState& state)
{
  ode_collision_model_->updateRobotModel(&state);
  for(std::map<std::string, std::map<std::string, bodies::BodyVector*> >::iterator it = link_attached_objects_.begin();
      it != link_attached_objects_.end();
      it++) {
    const planning_models::KinematicState::LinkState* ls = state.getLinkState(it->first);
    for(unsigned int j = 0; j < ls->getAttachedBodyStateVector().size(); j++) {
      const planning_models::KinematicState::AttachedBodyState* att_state = ls->getAttachedBodyStateVector()[j];
      std::map<std::string, bodies::BodyVector*>::iterator bvit = it->second.find(att_state->getName());
      if(bvit == it->second.end()) {
        ROS_WARN_STREAM("State out of sync with attached body vector for attached body " << att_state->getName());
        continue;
      }
      if(bvit->second->getSize() != att_state->getGlobalCollisionBodyTransforms().size()) {
        ROS_WARN_STREAM("State out of sync with attached body vector for attached body " << att_state->getName());
        continue;
      }
      for(unsigned int k = 0; k < att_state->getGlobalCollisionBodyTransforms().size(); k++) {
        bvit->second->setPose(k, att_state->getGlobalCollisionBodyTransforms()[k]);
      }
    }
  }
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
  addStaticObject(obj.id,
                  shapes, 
                  poses);
  return true;
}

//note - ownership of shape passes in
void planning_environment::CollisionModels::addStaticObject(const std::string& name,
                                                            std::vector<shapes::Shape*>& shapes,
                                                            const std::vector<btTransform>& poses)
{
  if(static_object_map_.find(name) != static_object_map_.end()) {
    deleteStaticObject(name);
  }
  static_object_map_[name] = new bodies::BodyVector(shapes,poses);
  //std::vector<shapes::Shape*> nv = cloneShapeVector(shapes);
  ode_collision_model_->lock();
  ode_collision_model_->addObjects(name, shapes, poses);
  ode_collision_model_->unlock();
}

void planning_environment::CollisionModels::deleteStaticObject(const std::string& name)
{
  ode_collision_model_->lock();
  delete static_object_map_.find(name)->second;
  ode_collision_model_->clearObjects(name);
  ode_collision_model_->unlock();
}

void planning_environment::CollisionModels::deleteAllStaticObjects() {
  ode_collision_model_->lock();
  for(std::map<std::string, bodies::BodyVector*>::iterator it = static_object_map_.begin();
      it != static_object_map_.end();
      it++) {
    delete it->second;
  }
  static_object_map_.clear();
  ode_collision_model_->clearObjects();
  ode_collision_model_->unlock();
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
  collision_map_shapes_ = shapes::cloneShapeVector(shapes);
  collision_map_poses_ = poses;
  std::vector<btTransform> masked_poses = poses;
  if(mask_before_insertion) {
    maskAndDeleteShapeVector(shapes,masked_poses); 
  } 
  ode_collision_model_->lock();
  ode_collision_model_->clearObjects(COLLISION_MAP_NAME);
  ode_collision_model_->addObjects(COLLISION_MAP_NAME, shapes, masked_poses);
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
  std::vector<bool> mask;
  std::vector<bodies::BodyVector*> static_object_vector;
  for(std::map<std::string, bodies::BodyVector*>::iterator it = static_object_map_.begin();
      it != static_object_map_.end();
      it++) {
    static_object_vector.push_back(it->second);
  }

  bodies::maskPosesInsideBodyVectors(poses, static_object_vector, mask);
  std::vector<btTransform> ret_poses;
  std::vector<shapes::Shape*> ret_shapes;
  for(unsigned int i = 0; i < mask.size(); i++) {
    if(mask[i]) {
      ret_shapes.push_back(shapes[i]);
      ret_poses.push_back(poses[i]);
    } else {
      delete shapes[i];
    }
  }
  shapes = ret_shapes;
  poses = ret_poses;
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
  addAttachedObject(obj.id,
                    att.link_name,
                    shapes,
                    poses,
                    att.touch_links);
  return true;
}


void planning_environment::CollisionModels::addAttachedObject(const std::string& object_name,
                                                              const std::string& link_name,
                                                              std::vector<shapes::Shape*>& shapes,
                                                              const std::vector<btTransform>& poses,
                                                              const std::vector<std::string>& touch_links)
{
  const planning_models::KinematicModel::LinkModel *link = kmodel_->getLinkModel(link_name);
  if(link == NULL) {
    ROS_WARN_STREAM("No link " << link_name << " for attaching " << object_name);
    return;
  }
  if(link_attached_objects_.find(link_name) != link_attached_objects_.end()) {
    if(link_attached_objects_[link_name].find(object_name) !=
       link_attached_objects_[link_name].end()) {
      delete link_attached_objects_[link_name][object_name];
      link_attached_objects_[link_name].erase(object_name);
      kmodel_->clearLinkAttachedBodyModel(link_name, object_name);
    }
  }

  link_attached_objects_[link_name][object_name] = new bodies::BodyVector(shapes,poses);  

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
}
   
void planning_environment::CollisionModels::deleteAttachedObject(const std::string& object_id,
                                                                 const std::string& link_name)

{
  if(link_attached_objects_.find(link_name) != link_attached_objects_.end()) {
    if(link_attached_objects_[link_name].find(object_id) !=
       link_attached_objects_[link_name].end()) {
      delete link_attached_objects_[link_name][object_id];
      link_attached_objects_[link_name].erase(object_id);
      kmodel_->clearLinkAttachedBodyModel(link_name, object_id);
    } else {
      ROS_WARN_STREAM("Link " << link_name << " has no object " << object_id << " to delete");
      return;
    }
  } else {
    ROS_WARN_STREAM("No link " << link_name << " for attached object delete");
    return;
  }
  ode_collision_model_->updateAttachedBodies();
}

void planning_environment::CollisionModels::deleteAllAttachedObjects(const std::string& link_name)
{
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
  ode_collision_model_->updateAttachedBodies();
}

void planning_environment::CollisionModels::convertStaticObjectToAttachedObject(const std::string& object_name,
                                                                                const std::string& link_name,
                                                                                const std::vector<std::string>& touch_links)
{
  const planning_models::KinematicModel::LinkModel *link = kmodel_->getLinkModel(link_name);
  if(link == NULL) {
    ROS_WARN_STREAM("No link " << link_name << " for attaching " << object_name);
    return;
  }
  if(static_object_map_.find(object_name) == static_object_map_.end()) {
    ROS_WARN_STREAM("No static object named " << object_name << " to convert");
    return;
  }
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
        poses.push_back(no.shapePose[j]);
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
}

void planning_environment::CollisionModels::convertAttachedObjectToStaticObject(const std::string& object_name,
                                                                                const std::string& link_name)
{
  const planning_models::KinematicModel::LinkModel *link = kmodel_->getLinkModel(link_name);
  if(link == NULL) {
    ROS_WARN_STREAM("No link " << link_name << " with attached object " << object_name);
    return;
  }
  if(link_attached_objects_.find(link_name) == link_attached_objects_.end() ||
     link_attached_objects_[link_name].find(object_name) == link_attached_objects_[link_name].end()) 
  {
    ROS_WARN_STREAM("No attached body " << object_name << " attached to link " << link_name);
    return;
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
    return;
  }
  {
    //need parentheses so that state goes out of scope
    planning_models::KinematicState state(kmodel_);
    std::vector<shapes::Shape*> shapes = shapes::cloneShapeVector(att->getShapes());
    const planning_models::KinematicState::AttachedBodyState* att_state = state.getAttachedBodyState(att->getName());
    ode_collision_model_->addObjects(object_name, shapes, att_state->getGlobalCollisionBodyTransforms());
  }
  
  kmodel_->clearLinkAttachedBodyModel(link_name, object_name);

  ode_collision_model_->updateAttachedBodies();
}

void planning_environment::CollisionModels::applyLinkPaddingToCollisionSpace(const std::vector<motion_planning_msgs::LinkPadding>& link_padding) {
  if(link_padding.empty()) return;
  
  const std::map<std::string, std::vector<std::string > >& group_link_map = getPlanningGroupLinks();
  std::map<std::string, double> link_padding_map;

  for(std::vector<motion_planning_msgs::LinkPadding>::const_iterator it = link_padding.begin();
      it != link_padding.end();
      it++) {
    std::vector<std::string> svec1;
    if(group_link_map.find((*it).link_name) != group_link_map.end()) {
      svec1 = group_link_map.find((*it).link_name)->second;
    } else {
      svec1.push_back((*it).link_name);
    }
    for(std::vector<std::string>::iterator stit1 = svec1.begin();
        stit1 != svec1.end();
        stit1++) {
      link_padding_map[(*stit1)] = (*it).padding;
    }
  }

  ode_collision_model_->setRobotLinkPadding(link_padding_map);  

}

bool planning_environment::CollisionModels::expandOrderedCollisionOperations(const motion_planning_msgs::OrderedCollisionOperations &ord,
                                                                             motion_planning_msgs::OrderedCollisionOperations &ex) const {

  std::vector<std::string> o_strings;
  for(std::map<std::string, bodies::BodyVector*>::const_iterator it = static_object_map_.begin();
      it != static_object_map_.end();
      it++) {
    o_strings.push_back(it->first);
  }
  o_strings.push_back(COLLISION_MAP_NAME);
  
  kmodel_->sharedLock();  
  std::vector<std::string> a_strings;
  const std::vector<const planning_models::KinematicModel::AttachedBodyModel*>& att_vec = ode_collision_model_->getAttachedBodies();
  for(unsigned int i = 0; i < att_vec.size(); i++) 
  {
    a_strings.push_back(att_vec[i]->getName());
  }
  kmodel_->sharedUnlock();
  
  const std::map<std::string, std::vector<std::string > >& group_link_map = getPlanningGroupLinks();
  
  ex = ord;
  std::vector<motion_planning_msgs::CollisionOperation>::iterator it = ex.collision_operations.begin();
  while(it != ex.collision_operations.end()) {
    std::vector<std::string> svec1;
    std::vector<std::string> svec2;
    bool special1 = false;
    bool special2 = false;
    if((*it).object1 == (*it).COLLISION_SET_OBJECTS) {
      svec1 = o_strings;
      special1 = true;
    }
    if((*it).object2 == (*it).COLLISION_SET_OBJECTS) {
      svec2 = o_strings;
      special2 = true;
    }
    if((*it).object1 == (*it).COLLISION_SET_ATTACHED_OBJECTS) {
      svec1 = a_strings;
      if(a_strings.empty()) {
        ROS_DEBUG_STREAM("No attached bodies");
      } else {
        ROS_DEBUG_STREAM("Setting first vec to all attached bodies, first entry " << a_strings[0]);
      }
      special1 = true;
    }
    if((*it).object2 == (*it).COLLISION_SET_ATTACHED_OBJECTS) {
      svec2 = a_strings;
      if(a_strings.empty()) {
        ROS_DEBUG_STREAM("No attached bodies");
      } else {
        ROS_DEBUG_STREAM("Setting second vec to all attached bodies, first entry " << a_strings[0]);
      }
      special2 = true;
    }
    if(group_link_map.find((*it).object1) != group_link_map.end()) {
      svec1 = group_link_map.find((*it).object1)->second;
      special1 = true;
    }
    if(group_link_map.find((*it).object2) != group_link_map.end()) {
      svec2 = group_link_map.find((*it).object2)->second;
      special2 = true;
    }
    if(!special1) {
      svec1.push_back((*it).object1);
    }
    if(!special2) {
      svec2.push_back((*it).object2);
    }

    int32_t op = (*it).operation;
      
    //we erase the original operation
    it = ex.collision_operations.erase(it);

    //EGJ 05/24/2010 Actually adding in attached bodies by default
    //addAttachedCollisionObjects(svec1);
    //addAttachedCollisionObjects(svec2);

    //now we need to add all the new pairwise constraints
    for(std::vector<std::string>::iterator stit1 = svec1.begin();
        stit1 != svec1.end();
        stit1++) {
      for(std::vector<std::string>::iterator stit2 = svec2.begin();
          stit2 != svec2.end();
          stit2++) {
        motion_planning_msgs::CollisionOperation coll;
        coll.object1 = (*stit1);
        coll.object2 = (*stit2);
        coll.operation = op;
        it = ex.collision_operations.insert(it,coll);
        it++;
      }
    }
  }
  for(it = ex.collision_operations.begin(); it != ex.collision_operations.end();it++) {
    ROS_DEBUG_STREAM("Expanded coll " << (*it).object1 << " " << (*it).object2 << " op " << (*it).operation);
  }
  ode_collision_model_->unlock();
  return true;
}

bool planning_environment::CollisionModels::applyOrderedCollisionOperationsToCollisionSpace(const motion_planning_msgs::OrderedCollisionOperations &ord, bool print) {

  //getting the default set of enabled collisions
  std::vector<std::vector<bool> > curAllowed;
  std::map<std::string, unsigned int> vecIndices;
  ode_collision_model_->lock();
  ode_collision_model_->getDefaultAllowedCollisionMatrix(curAllowed, vecIndices);
  ode_collision_model_->unlock();

  motion_planning_msgs::OrderedCollisionOperations expanded;
  expandOrderedCollisionOperations(ord, expanded);

  //std::cout << "Default:\n";
  //printAllowedCollisionMatrix(curAllowed, vecIndices);

  applyOrderedCollisionOperationsToMatrix(expanded, curAllowed, vecIndices);

  if(print) {
    printAllowedCollisionMatrix(curAllowed, vecIndices);
  }

  ode_collision_model_->setAllowedCollisionMatrix(curAllowed, vecIndices);
  return true;
}

void planning_environment::CollisionModels::addAttachedCollisionObjects(std::vector<std::string>& svec) const {

  kmodel_->sharedLock();
  
  std::vector<std::string>::iterator stit = svec.begin();
  while(stit != svec.end()) {
    const std::vector<const planning_models::KinematicModel::AttachedBodyModel*> att_vec = ode_collision_model_->getAttachedBodies(*stit);
    //these get inserted after the link
    stit++;
    for(std::vector<const planning_models::KinematicModel::AttachedBodyModel*>::const_iterator ait = att_vec.begin();
        ait != att_vec.end();
        ait++) {
      ROS_DEBUG_STREAM("Adding attached collision object " << (*ait)->getName() << " to list");
      stit = svec.insert(stit,(*ait)->getName());
    }
  }

  kmodel_->sharedUnlock();
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
      const btVector3 &c = no.shapePose[i].getOrigin();
      obb.center.x = c.x();
      obb.center.y = c.y();
      obb.center.z = c.z();
      const btQuaternion q = no.shapePose[i].getRotation();
      obb.angle = q.getAngle();
      const btVector3 axis = q.getAxis();
      obb.axis.x = axis.x();
      obb.axis.y = axis.y();
      obb.axis.z = axis.z();
      cmap.boxes.push_back(obb);
    }
  }
}

void planning_environment::CollisionModels::revertAllowedCollisionToDefault() {
  ode_collision_model_->revertAllowedCollisionMatrix();
}

void planning_environment::CollisionModels::revertCollisionSpacePaddingToDefault() {
  ode_collision_model_->revertRobotLinkPadding();
}

void planning_environment::CollisionModels::getCollisionSpaceAllowedCollisions(planning_environment_msgs::AllowedCollisionMatrix& ret_matrix) const {

  std::vector<std::vector<bool> > matrix;
  std::map<std::string, unsigned int> ind;
  
  ode_collision_model_->getCurrentAllowedCollisionMatrix(matrix, ind);

  //printAllowedCollisionMatrix(matrix, ind);

  unsigned int i = 0;
  ret_matrix.link_names.resize(ind.size());
  ret_matrix.entries.resize(ind.size());
  for(std::map<std::string, unsigned int>::iterator it = ind.begin();
      it != ind.end();
      it++,i++) {
    ret_matrix.link_names[i] = it->first;
    ret_matrix.entries[i].enabled.resize(matrix[it->second].size());
    unsigned int j = 0;
    for(std::map<std::string, unsigned int>::iterator it2 = ind.begin();
        it2 != ind.end();
        it2++,j++) {
      ret_matrix.entries[i].enabled[j] = matrix[it->second][it2->second];
    }
  }
}

void planning_environment::CollisionModels::getCollisionSpaceCollisionObjects(std::vector<mapping_msgs::CollisionObject> &omap) const
{
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
        tf::poseTFToMsg(no.shapePose[j], pose);
        o.shapes.push_back(obj);
        o.poses.push_back(pose);
      }
    }
    omap.push_back(o);
  }
}

void planning_environment::CollisionModels::getCollisionSpaceAttachedCollisionObjects(std::vector<mapping_msgs::AttachedCollisionObject> &avec) const
{
  avec.clear();

  const std::vector<const planning_models::KinematicModel::AttachedBodyModel*>& att_vec = ode_collision_model_->getAttachedBodies();
  for(unsigned int i = 0; i < att_vec.size(); i++) 
  {
    mapping_msgs::AttachedCollisionObject ao;
    ao.object.header.frame_id = getWorldFrameId();
    ao.object.header.stamp = ros::Time::now();
    ao.link_name = att_vec[i]->getAttachedLinkModel()->getName();
    double attached_padd = ode_collision_model_->getCurrentLinkPadding("attached");
    for(unsigned int j = 0; j < att_vec[i]->getShapes().size(); j++) {
      geometric_shapes_msgs::Shape shape;
      constructObjectMsg(att_vec[i]->getShapes()[j], shape, attached_padd);
      geometry_msgs::Pose pose;
      //TODO - fix this!!!
      //planning_models::KinematicState::AttachedBodyState* att_state = state.getAttachedBodyState(att_vec[i]->getName());
      //if(att_state == NULL) {
      //  ROS_WARN_STREAM("No attached body state for attached body model " << att_vec[i]->getName());
      //}
      //tf::poseTFToMsg(att_state->getGlobalCollisionBodyTransforms()[j], pose);
      ao.object.shapes.push_back(shape);
      ao.object.poses.push_back(pose);
    }
    ao.touch_links = att_vec[i]->getTouchLinks();
    ao.object.id = att_vec[i]->getName();
    avec.push_back(ao);
  }
}

bool planning_environment::CollisionModels::isKinematicStateInCollision(const planning_models::KinematicState& state)                                                                     
{
  ode_collision_model_->updateRobotModel(&state);
  return(ode_collision_model_->isCollision());
}

bool planning_environment::CollisionModels::isTrajectoryValid(const trajectory_msgs::JointTrajectory &trajectory,
                                                              const motion_planning_msgs::RobotState& robot_state,
                                                              const motion_planning_msgs::Constraints& path_constraints,
                                                              const motion_planning_msgs::Constraints& goal_constraints, 
                                                              motion_planning_msgs::ArmNavigationErrorCodes& error_code,
                                                              std::vector<motion_planning_msgs::ArmNavigationErrorCodes>& trajectory_error_codes,
                                                              const bool evaluate_entire_trajectory)
{
  error_code.val = error_code.SUCCESS;
  
  planning_models::KinematicState state(kmodel_);

  //first making sure that the robot state is complete
  if(!setRobotStateAndComputeTransforms(robot_state, state)) {
    error_code.val = error_code.INCOMPLETE_ROBOT_STATE;
    return false;
  }

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

  for(unsigned int i = 0; i < trajectory.points.size(); i++) {
    motion_planning_msgs::ArmNavigationErrorCodes suc;
    suc.val = error_code.SUCCESS;
    trajectory_error_codes.push_back(suc);
    for (unsigned int j = 0 ; j < trajectory.points[i].positions.size(); j++)
    {
      joint_value_map[trajectory.joint_names[j]] = trajectory.points[i].positions[j];
    }
    state.setKinematicState(joint_value_map);

    //first check joint limits
    if(!state.areJointsWithinBounds(trajectory.joint_names)) {
      error_code.val = error_code.JOINT_LIMITS_VIOLATED;
      if(!evaluate_entire_trajectory) {
        return false;
      }
      trajectory_error_codes.back() = error_code;
      continue;
    }
    
    //next check path constraints
    if(!doesKinematicStateObeyConstraints(state, path_constraints)) {
      error_code.val = error_code.START_STATE_VIOLATES_PATH_CONSTRAINTS;
      if(!evaluate_entire_trajectory) {
        return false;
      }
      trajectory_error_codes.back() = error_code;
      continue;
    }
    
    ode_collision_model_->updateRobotModel(&state);
    if(ode_collision_model_->isCollision()) {
      error_code.val = error_code.COLLISION_CONSTRAINTS_VIOLATED;
      if(!evaluate_entire_trajectory) {
        return false;
      }
      trajectory_error_codes.back() = error_code;
      continue;
    }
  }
  return(error_code.val == error_code.SUCCESS);
}

