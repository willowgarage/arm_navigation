/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

/** \author E. Gil Jones */

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <collision_proximity/collision_proximity_space.h>

using collision_proximity::CollisionProximitySpace;

static double gen_rand(double min, double max)
{
  int rand_num = rand()%100+1;
  double result = min + (double)((max-min)*rand_num)/101.0;
  return result;
}

static std::string makeStringFromUnsignedInt(unsigned int j)
{
  std::stringstream ss;
  ss << j;
  return ss.str();
}

static std::string makeAttachedObjectId(std::string link, std::string object) 
{
  return link+"_"+object;
}

CollisionProximitySpace::CollisionProximitySpace(planning_environment::CollisionModels* model) :
  priv_handle_("~")
{
  model_ = model;

  priv_handle_.param("size_x", size_x_, 3.0);
  priv_handle_.param("size_y", size_y_, 3.0);
  priv_handle_.param("size_z", size_z_, 4.0);
  priv_handle_.param("origin_x", origin_x_, -1.0);
  priv_handle_.param("origin_y", origin_y_, -1.5);
  priv_handle_.param("origin_z", origin_z_, -2.0);
  priv_handle_.param("resolution", resolution_, 0.02);
  priv_handle_.param("collision_tolerance", tolerance_, 0.00);
  priv_handle_.param("max_environment_distance", max_environment_distance_, 0.25);

  vis_marker_publisher_ = root_handle_.advertise<visualization_msgs::Marker>("collision_proximity_body_spheres", 128);
  vis_marker_array_publisher_ = root_handle_.advertise<visualization_msgs::MarkerArray>("collision_proximity_body_spheres_array", 128);

  distance_field_ = new distance_field::PropagationDistanceField(size_x_, size_y_, size_z_, resolution_, origin_x_, origin_y_, origin_z_, max_environment_distance_);

  loadRobotBodyDecompositions();

  loadDefaultCollisionOperations();

  /*
    shapes::Box base_box(1.0,1.0,.3);
    BodyDecomposition* bd = new BodyDecomposition("base_box", &base_box, resolution_);
    btTransform trans;
    trans.setIdentity();
    trans.setOrigin(btVector3(0.0,0.0,.15));
    bd->updatePose(trans);
    BodyDecompositionVector* bdv = new BodyDecompositionVector();
    bdv->addToVector(bd);
    static_object_map_["base_box"] = bdv;
  */

  monitor_->setOnAfterCollisionObjectCallback(boost::bind(&CollisionProximitySpace::staticObjectUpdateEvent, this, _1));
  monitor_->setOnAfterAttachCollisionObjectCallback(boost::bind(&CollisionProximitySpace::attachedObjectUpdateEvent, this, _1));
  monitor_->startEnvironmentMonitor();
}

CollisionProximitySpace::~CollisionProximitySpace()
{
  delete distance_field_;
  for(std::map<std::string, BodyDecomposition*>::iterator it = body_decomposition_map_.begin();
      it != body_decomposition_map_.end();
      it++) {
    delete it->second;
  }
  for(std::map<std::string, BodyDecompositionVector*>::iterator it = static_object_map_.begin();
      it != static_object_map_.end();
      it++) {
    delete it->second;
  }
  for(std::map<std::string, BodyDecompositionVector*>::iterator it = attached_object_map_.begin();
      it != attached_object_map_.end();
      it++) {
    delete it->second;
  }
}

void CollisionProximitySpace::loadDefaultCollisionOperations()
{
  std::map<std::string, bool> all_true_map;
  boost::shared_ptr<planning_models::KinematicModel> kmodel = model_->getKinematicModel();
  
  for(unsigned int i = 0; i < kmodel->getLinkModels().size(); i++) {
    all_true_map[kmodel->getLinkModels()[i]->getName()] = true;
    for(unsigned int j = 0; j < kmodel->getLinkModels().size(); j++) {
      intra_group_collision_links_[kmodel->getLinkModels()[i]->getName()][kmodel->getLinkModels()[j]->getName()] = false;
    }
  }

  //creating lists for each group in planning groups
  const std::map<std::string, std::vector<std::string> >& pgl = model_->getPlanningGroupLinks();
  for(std::map<std::string, std::vector<std::string> >::const_iterator it = pgl.begin();
      it != pgl.end();
      it++) {
    enabled_self_collision_links_[it->first] = all_true_map;
    for(unsigned int i = 0; i < it->second.size(); i++) {
      enabled_self_collision_links_[it->first][it->second[i]] = false;
    }
  }

  XmlRpc::XmlRpcValue coll_ops;
  if(!root_handle_.hasParam("/robot_description_collision/default_collision_operations")) {
    ROS_WARN("No default collision operations specified");
    return;
  }
  root_handle_.getParam("/robot_description_collision/default_collision_operations", coll_ops);
  
  if(coll_ops.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_WARN("proximity_collision_operations is not an array");
    return;
  } 
  
  if(coll_ops.size() == 0) {
    ROS_WARN("No collision operations in proximity collision operations");
    return;
  }
  
  for(int i = 0; i < coll_ops.size(); i++) {
    if(!coll_ops[i].hasMember("object1") || !coll_ops[i].hasMember("object2") || !coll_ops[i].hasMember("operation")) {
      ROS_WARN("All collision operations must have two objects and an operation");
      continue;
    }
    std::string object1 = std::string(coll_ops[i]["object1"]);
    std::string object2 = std::string(coll_ops[i]["object2"]);
    std::string operation = std::string(coll_ops[i]["operation"]);
      
    if(pgl.find(object1) == pgl.end()) {
      if(object1 == object2) {
        //using for disabling environment checks
        environment_excludes_[object1] = true;
      } else {
        //must be intra_collision
        if(pgl.find(object2) == pgl.end()) {
          intra_group_collision_links_[object1][object2] = (operation == "enable");
          intra_group_collision_links_[object2][object1] = (operation == "enable");
        } else {
          const std::vector<std::string>& group_links = pgl.find(object2)->second;
          for(unsigned int j = 0; j < group_links.size(); j++) {
            intra_group_collision_links_[object1][group_links[j]] = (operation == "enable");
            intra_group_collision_links_[group_links[j]][object1] = (operation == "enable");
          }
        }
      }
    } else if(pgl.find(object2) == pgl.end()) {
      if(enabled_self_collision_links_[object1].find(object2) == enabled_self_collision_links_[object1].end()) {
        ROS_INFO_STREAM("No object found for non-group object " << object2);
        continue;
      }
      enabled_self_collision_links_[object1][object2] = (operation == "enable");
    } else {
      const std::vector<std::string>& group_links = pgl.find(object2)->second;
      for(unsigned int j = 0; j < group_links.size(); j++) {
        if(enabled_self_collision_links_[object1].find(group_links[j]) == enabled_self_collision_links_[object1].end()) {
          ROS_INFO_STREAM("No object found for group object " << group_links[j]);
          continue;
        } 
        enabled_self_collision_links_[object1][group_links[j]] = (operation == "enable");
      }
    }
  }
}

void CollisionProximitySpace::loadRobotBodyDecompositions()
{
  boost::shared_ptr<planning_models::KinematicModel> kmodel = model_->getKinematicModel();
  
  for(unsigned int i = 0; i < kmodel->getLinkModels().size(); i++) {
    //if(kmodel->getLinkModels()[i]->getName() == "r_upper_arm_link") {
    if(kmodel->getLinkModels()[i]->getLinkShape() != NULL) {
      body_decomposition_map_[kmodel->getLinkModels()[i]->getName()] = new BodyDecomposition(kmodel->getLinkModels()[i]->getName(),
                                                                                             kmodel->getLinkModels()[i]->getLinkShape(),
                                                                                             resolution_/2.0);
    }
      //}
  }
}

void CollisionProximitySpace::setupForGroupQueries(const std::string& group_name,
                                                   const motion_planning_msgs::RobotState& rob_state) {
  group_queries_lock_.lock();
  //setting up current info
  current_group_name_ = group_name;
  getGroupLinkAndAttachedBodyNames(current_group_name_, 
                                   current_link_names_, 
                                   current_link_indices_,
                                   current_attached_body_names_,
                                   current_attached_body_indices_);
  setupGradientStructures(current_link_names_,
                          current_attached_body_names_,
                          current_link_distances_,
                          current_closest_distances_,
                          current_closest_gradients_);

  current_link_body_decompositions_.clear();
  current_attached_body_decompositions_.clear();
  current_environment_excludes_.clear();
  for(unsigned int i = 0; i < current_link_names_.size(); i++) {
    current_link_body_decompositions_.push_back(body_decomposition_map_[current_link_names_[i]]);
    if(environment_excludes_.find(current_link_names_[i]) != environment_excludes_.end()) {
      current_environment_excludes_.push_back(false);
    } else {
      current_environment_excludes_.push_back(true);
    }
  }
  for(unsigned int i = 0; i < current_attached_body_names_.size(); i++) {
    current_attached_body_decompositions_.push_back(attached_object_map_[current_attached_body_names_[i]]);
  }
  current_intra_group_collision_links_.clear();
  unsigned int num_links = current_link_names_.size();
  unsigned int num_attached = current_attached_body_names_.size();
  unsigned int tot = num_links+num_attached;
  current_intra_group_collision_links_.resize(tot);
  for(unsigned int i = 0; i < tot; i++) {
    current_intra_group_collision_links_[i].resize(tot);
    for(unsigned int j = 0; j < tot; j++) {
      if(i < num_links && j < num_links) {
        current_intra_group_collision_links_[i][j] = intra_group_collision_links_.find(current_link_names_[i])->second.find(current_link_names_[j])->second;
        //ROS_INFO_STREAM("Setting intra_group_collision_links for " << current_link_names_[i] << " and " << current_link_names_[j] << " to " << current_intra_group_collision_links_[i][j]);
      } else {
        std::string name1;
        std::string name2;
        bool attached1 = false;
        bool attached2 = false;
        if(i < num_links) {
          name1 = current_link_names_[i];
        } else {
          attached1 = true;          
          name1 = current_attached_body_names_[i-num_links];
        }
        if(j < num_links) {
          name2 = current_link_names_[j];
        } else {
          attached2 = true;
          name2 = current_attached_body_names_[j-num_links];
        }
        current_intra_group_collision_links_[i][j] = true;
        if(attached1) {
          if(attached_object_collision_links_.find(name1)->second.find(name2) != attached_object_collision_links_.find(name1)->second.end() && !attached_object_collision_links_.find(name1)->second.find(name2)->second) {
            current_intra_group_collision_links_[i][j] = false;
          }
        }
        if(attached2) {
          if(attached_object_collision_links_.find(name2)->second.find(name1) != attached_object_collision_links_.find(name2)->second.end() && !attached_object_collision_links_.find(name2)->second.find(name1)->second) {
            current_intra_group_collision_links_[i][j] = false;
          }
        }
      }
    }
  }

  planning_models::KinematicState kin_state(model_->getKinematicModel());
  monitor_->setRobotStateAndComputeTransforms(rob_state, kin_state);
  monitor_->setCollisionSpace();
  setBodyPosesGivenKinematicState(kin_state);
  setDistanceFieldForGroupQueries(current_group_name_, kin_state);
}

void CollisionProximitySpace::revertAfterGroupQueries()
{
  current_group_name_ = "";
  group_queries_lock_.unlock();
}

void CollisionProximitySpace::setBodyPosesToCurrent()
{
  planning_models::KinematicState state(monitor_->getKinematicModel());
  monitor_->setStateValuesFromCurrentValues(state);
  setBodyPosesGivenKinematicState(state);
}

void CollisionProximitySpace::setCurrentGroupState(const planning_models::KinematicState& state)
{
  btTransform inv = getInverseWorldTransform(state);
  for(unsigned int i = 0; i < current_link_indices_.size(); i++) {
    const planning_models::KinematicState::LinkState* ls = state.getLinkStateVector()[current_link_indices_[i]];
    current_link_body_decompositions_[i]->updateSpheresPose(inv*ls->getGlobalCollisionBodyTransform());
  }
  for(unsigned int i = 0; i < current_attached_body_indices_.size(); i++) {
    const planning_models::KinematicState::LinkState* ls = state.getLinkStateVector()[current_attached_body_indices_[i]];
    for(unsigned int j = 0; j < ls->getAttachedBodyStateVector().size(); j++) {
      const planning_models::KinematicState::AttachedBodyState* att_state = ls->getAttachedBodyStateVector()[j];
      for(unsigned int k = 0; k < att_state->getGlobalCollisionBodyTransforms().size(); k++) {
        current_attached_body_decompositions_[i]->updateBodyPose(k, inv*att_state->getGlobalCollisionBodyTransforms()[k]);
      }
    }
  }
}

void CollisionProximitySpace::setBodyPosesGivenKinematicState(const planning_models::KinematicState& state)
{
  btTransform inv = getInverseWorldTransform(state);
  for(unsigned int i = 0; i < state.getLinkStateVector().size(); i++) {
    const planning_models::KinematicState::LinkState* ls = state.getLinkStateVector()[i];
    if(body_decomposition_map_.find(ls->getName()) == body_decomposition_map_.end()) {
      //ROS_WARN_STREAM("Can't find body decomposition for link state " << ls->getName());
      continue;
    }
    body_decomposition_map_[ls->getName()]->updatePose(inv*ls->getGlobalCollisionBodyTransform());
    for(unsigned int j = 0; j < ls->getAttachedBodyStateVector().size(); j++) {
      const planning_models::KinematicState::AttachedBodyState* att_state = ls->getAttachedBodyStateVector()[j];
      std::string id = makeAttachedObjectId(ls->getName(), att_state->getName());
      if(attached_object_map_.find(id) == attached_object_map_.end()) {
        ROS_WARN_STREAM("Updating pose but no attached object for id " << id);
        continue;
      }
      if(attached_object_map_[id]->getSize() != att_state->getGlobalCollisionBodyTransforms().size()) {
        ROS_WARN_STREAM("Bodies number not equal to state number. Not updating");
        continue;
      }
      for(unsigned int k = 0; k < att_state->getGlobalCollisionBodyTransforms().size(); k++) {
        attached_object_map_[id]->updateBodyPose(k, inv*att_state->getGlobalCollisionBodyTransforms()[k]);
      }
    }
  }
}

btTransform CollisionProximitySpace::getInverseWorldTransform(const planning_models::KinematicState& state) const {
  const planning_models::KinematicState::JointState* world_state = state.getJointStateVector()[0];//(monitor_->getKinematicModel()->getRoot()->getName());
  if(world_state == NULL) {
    ROS_WARN_STREAM("World state " << monitor_->getKinematicModel()->getRoot()->getName() << " not found");
  }
  const btTransform& world_trans = world_state->getVariableTransform();
  btTransform ret(world_trans);
  return ret.inverse();
}

void CollisionProximitySpace::staticObjectUpdateEvent(const mapping_msgs::CollisionObjectConstPtr &coll)
{
  group_queries_lock_.lock();
  monitor_->getEnvironmentModel()->lock();
  planning_models::KinematicState state(monitor_->getKinematicModel());
  monitor_->setStateValuesFromCurrentValues(state);
  btTransform inv = getInverseWorldTransform(state);
  if(coll->operation.operation == coll->operation.ADD) {
    if(static_object_map_.find(coll->id) != static_object_map_.end()) {
      delete static_object_map_[coll->id];
      static_object_map_.erase(coll->id);
    }
    BodyDecompositionVector* bdv = new BodyDecompositionVector();
    const collision_space::EnvironmentObjects *eo = monitor_->getEnvironmentModel()->getObjects();
    std::vector<std::string> ns = eo->getNamespaces();
    bool found = false;
    for(unsigned int i = 0; i < ns.size(); i++) {
      if(ns[i] == coll->id) {
        found = true;
        const collision_space::EnvironmentObjects::NamespaceObjects &no = eo->getObjects(ns[i]);
        for(unsigned int j = 0; j < no.shape.size(); j++) {
          BodyDecomposition* bd = new BodyDecomposition(coll->id+"_"+makeStringFromUnsignedInt(j), no.shape[j], resolution_);
          bd->updatePose(no.shapePose[j]);
          ROS_INFO_STREAM("Pose is " << no.shapePose[j].getOrigin().x() << " " 
                          << no.shapePose[j].getOrigin().y() << " " 
                          << no.shapePose[j].getOrigin().z());
          bdv->addToVector(bd); 
        }
        break;
      }
    }
    if(!found) {
      ROS_WARN_STREAM("Added object named " << coll->id << " not found in collision space");
    } else {
      static_object_map_[coll->id] = bdv;
    }
  } else if(coll->operation.operation == coll->operation.REMOVE){
    if(coll->id == "all") {
      for(std::map<std::string, BodyDecompositionVector*>::iterator it = static_object_map_.begin();
          it != static_object_map_.end();
          it++) {
        delete it->second;
      }
      static_object_map_.clear();
    } else if(static_object_map_.find(coll->id) != static_object_map_.end()) {
      delete static_object_map_[coll->id];
      static_object_map_.erase(coll->id);
    }
  }
  monitor_->getEnvironmentModel()->unlock();
  group_queries_lock_.unlock();
  ROS_INFO_STREAM("Done adding static object");
}

void CollisionProximitySpace::attachedObjectUpdateEvent(const mapping_msgs::AttachedCollisionObjectConstPtr &coll)
{
  ROS_INFO("In attached object update");
  group_queries_lock_.lock();
  monitor_->getEnvironmentModel()->lock();
  //getting state to invoke lock
  planning_models::KinematicState state(monitor_->getKinematicModel());
  monitor_->setStateValuesFromCurrentValues(state);
  std::string id = makeAttachedObjectId(coll->link_name,coll->object.id);
  if(coll->object.operation.operation == mapping_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT) {
    if(attached_object_map_.find(id) == attached_object_map_.end()) {
      ROS_WARN_STREAM("No object named " << id << " to detach");
      monitor_->getEnvironmentModel()->unlock();
      group_queries_lock_.unlock();
      return;
    }
    BodyDecompositionVector* vec = attached_object_map_.find(id)->second;
    link_attached_objects_[coll->link_name].erase(id);
    attached_object_map_.erase(id);
    static_object_map_[id] = vec;
  } else if (coll->object.operation.operation == mapping_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT) {
    if(static_object_map_.find(id) == static_object_map_.end()) {
      ROS_WARN_STREAM("No object named " << id << " to attach");
      monitor_->getEnvironmentModel()->unlock();
      group_queries_lock_.unlock();
      return;
    }
    BodyDecompositionVector* vec = static_object_map_.find(id)->second;
    static_object_map_.erase(id);
    attached_object_map_[id] = vec;
    link_attached_objects_[coll->link_name][id] = true;
    for(unsigned int i = 0; i < coll->touch_links.size(); i++) {
      attached_object_collision_links_[id][coll->touch_links[i]] = false;
    }
  } else if (coll->object.operation.operation == mapping_msgs::CollisionObjectOperation::REMOVE) {
    if(coll->object.id == "all") {
      for(std::map<std::string, BodyDecompositionVector*>::iterator it = attached_object_map_.begin();
          it != attached_object_map_.end();
          it++) {
        delete it->second;
      }
      link_attached_objects_.clear();
      attached_object_map_.clear();
      attached_object_collision_links_.clear();
    } else {
      if(attached_object_map_.find(id) != attached_object_map_.end()) {
        delete attached_object_map_[id];
        attached_object_map_.erase(id);
        attached_object_collision_links_.erase(id);
        link_attached_objects_[coll->link_name].erase(id);
      }
    }
  } else {
    //must be add
    const planning_models::KinematicModel::LinkModel* lm = monitor_->getKinematicModel()->getLinkModel(coll->link_name);
    const planning_models::KinematicModel::AttachedBodyModel* abm = NULL;
    for(unsigned int j = 0; j < lm->getAttachedBodyModels().size(); j++) {
      if(lm->getAttachedBodyModels()[j]->getName() == coll->object.id) {
        abm = lm->getAttachedBodyModels()[j];
        break;
      }
    }
    if(abm == NULL) {
      ROS_WARN_STREAM("Can't find attached object named " << coll->object.id << " for link " << coll->link_name);
      monitor_->getEnvironmentModel()->unlock();
      group_queries_lock_.unlock();
      return;
    }
    BodyDecompositionVector* bdv = new BodyDecompositionVector();
    ROS_INFO_STREAM("Adding attached body with id " << id);
    for(unsigned int j = 0; j < abm->getShapes().size(); j++) {
      BodyDecomposition* bd = new BodyDecomposition(id+makeStringFromUnsignedInt(j), abm->getShapes()[j], resolution_);
      bdv->addToVector(bd);
    }
    attached_object_map_[id] = bdv;
    link_attached_objects_[coll->link_name][id] = true;
    for(unsigned int i = 0; i < coll->touch_links.size(); i++) {
      attached_object_collision_links_[id][coll->touch_links[i]] = false;
    }
  }
  setBodyPosesToCurrent();
  monitor_->getEnvironmentModel()->unlock();
  //TODO - retain robot state some way
  if(current_group_name_ != "") {
    std::string group_name = current_group_name_;
    group_queries_lock_.unlock();
    revertAfterGroupQueries();
    motion_planning_msgs::RobotState rob_state;
    setupForGroupQueries(group_name, rob_state);
  }
}

void CollisionProximitySpace::setDistanceFieldForGroupQueries(const std::string& group_name,
                                                              const planning_models::KinematicState& state)
{
  if(enabled_self_collision_links_.find(group_name) == enabled_self_collision_links_.end()) {
    ROS_WARN_STREAM("No group named " << group_name << " in planning groups");
    return;
  }
  std::vector<std::string> df_links;
  for(std::map<std::string,bool>::iterator it = enabled_self_collision_links_[group_name].begin();
      it != enabled_self_collision_links_[group_name].end();
      it++) {
    if(it->second) {
      df_links.push_back(it->first);
    }
  }
  prepareDistanceField(df_links, state);
}

void CollisionProximitySpace::prepareDistanceField(const std::vector<std::string>& link_names, 
                                                   const planning_models::KinematicState& state)
{
  ros::WallTime n1 = ros::WallTime::now();
  distance_field_->reset();
  ros::WallTime n2 = ros::WallTime::now();
  //ROS_INFO_STREAM("Reset took " << (n2-n1).toSec());
  n1 = ros::WallTime::now();
  btTransform inv = getInverseWorldTransform(state);
  std::vector<btVector3> all_points;
  for(std::map<std::string, BodyDecompositionVector*>::iterator it = static_object_map_.begin();
      it != static_object_map_.end();
      it++) {
    for(unsigned int i = 0; i < it->second->getSize(); i++) {
      std::vector<btVector3> obj_points = it->second->getBodyDecomposition(i)->getCollisionPoints();
      for(unsigned int j = 0; j < obj_points.size(); j++) {
        btVector3 temp = it->second->getBodyDecomposition(i)->getBody()->getPose()*obj_points[j];
        obj_points[j] = inv*obj_points[j];
      }
      all_points.insert(all_points.end(),obj_points.begin(), obj_points.end());
    }
  }
  for(unsigned int i = 0; i < link_names.size(); i++) {
    if(body_decomposition_map_.find(link_names[i]) == body_decomposition_map_.end()) {
      //there is no collision geometry as far as we can tell
      continue;
    }
    //ROS_INFO_STREAM("Adding link " << link_names[i]);
    const BodyDecomposition* bd = body_decomposition_map_.find(link_names[i])->second;
    all_points.insert(all_points.end(), bd->getCollisionPoints().begin(), bd->getCollisionPoints().end());
    const planning_models::KinematicState::LinkState* ls = state.getLinkState(link_names[i]);
    for(unsigned int j = 0; j < ls->getAttachedBodyStateVector().size(); j++) {
      std::string id = makeAttachedObjectId(ls->getName(),ls->getAttachedBodyStateVector()[j]->getName());
      if(attached_object_map_.find(id) == attached_object_map_.end()) {
        ROS_WARN_STREAM("Have no attached object body for attached object " << id);
        continue;
      }
      const BodyDecompositionVector* att = attached_object_map_.find(id)->second;
      const std::vector<btVector3>& att_points = att->getCollisionPoints();
      all_points.insert(all_points.end(), att_points.begin(), att_points.end());
    }
  }
  //now we need to add the points 
  const collision_space::EnvironmentObjects *eo = monitor_->getEnvironmentModel()->getObjects();
  std::vector<std::string> ns = eo->getNamespaces();
  for (unsigned int i = 0 ; i < ns.size() ; ++i)
  {
    const collision_space::EnvironmentObjects::NamespaceObjects &no = eo->getObjects(ns[i]);
    const unsigned int n = no.shape.size();
    
    //special case for collision map points
    if(ns[i] == "points") {
      //points.reserve(points.size()+n);
      for(unsigned int j = 0; j < n;  ++j) 
      {
        all_points.push_back(inv*no.shapePose[j].getOrigin());
      }
    }
  }
  //ROS_INFO_STREAM("Points addition took " << (n2-n1).toSec());  
  //ROS_INFO_STREAM("Adding " << all_points.size() << " to field");
  n1 = ros::WallTime::now();
  distance_field_->addPointsToField(all_points);
  n2 = ros::WallTime::now();
  //ROS_INFO_STREAM("Adding points took " << (n2-n1).toSec());
}

/*
bool CollisionProximitySpace::getEnvironmentProximity(ProximityInfo& prox) const
{
  updating_objects_lock_.lock();
  prox.proximity = DBL_MAX;
  for(unsigned int i = 0; i < link_names.size(); i++) {
    if(body_decomposition_map_.find(link_names[i]) == body_decomposition_map_.end()) {
      ROS_WARN_STREAM("No link " << link_names[i] << " for getEnvironmentProximity");
      continue;
    }
    const BodyDecomposition* bd = body_decomposition_map_.find(link_names[i])->second;
    unsigned int lc;
    btVector3 grad;
    double dist = getCollisionSphereProximity(bd->getCollisionSpheres(), lc, grad);
    if(dist == 0.0) {
      //ROS_WARN_STREAM("Zero distance on link " << link_names[i]);
    } else {
      //ROS_INFO_STREAM("Link " << link_names[i] << " proximity " << dist);
      if(dist < prox.proximity) {
        prox.proximity = dist;
        prox.link_name = link_names[i];
        prox.sphere_index = lc;
        prox.closest_point = bd->getCollisionSpheres()[lc].center_;
        prox.closest_gradient = grad;
      }
    }
    const planning_models::KinematicModel::LinkModel* lm = monitor_->getKinematicModel()->getLinkModel(link_names[i]);
    for(unsigned int j = 0; j < lm->getAttachedBodyModels().size(); j++) {
      std::string id = makeAttachedObjectId(lm->getName(),lm->getAttachedBodyModels()[j]->getName());
      if(attached_object_map_.find(id) == attached_object_map_.end()) {
        ROS_WARN_STREAM("Have no attached object body for attached object " << id);
        continue;
      }
      const BodyDecompositionVector* att = attached_object_map_.find(id)->second;
      for(unsigned int k = 0; k < att->getSize(); k++) {
        double dist = getCollisionSphereProximity(att->getBodyDecomposition(k)->getCollisionSpheres(), lc, grad);
        if(dist < prox.proximity) {
          prox.proximity = dist;
          prox.link_name = lm->getName();
          prox.attached_object_name = id;
          prox.att_index = j;
          prox.sphere_index = lc;
          prox.closest_point = att->getBodyDecomposition(k)->getCollisionSpheres()[lc].center_;
          prox.closest_gradient = grad; 
        }
      }
    }
  }
  updating_objects_lock_.unlock();
  return true;
}

double CollisionProximitySpace::getCollisionSphereProximity(const std::vector<CollisionSphere>& sphere_list, unsigned int& closest, btVector3& gradient) const {
  double distance = DBL_MAX;
  for(unsigned int i = 0; i < sphere_list.size(); i++) {
    btVector3 p = sphere_list[i].center_;
    double gx, gy, gz;
    double dist = distance_field_->getDistanceGradient(p.x(), p.y(), p.z(), gx, gy, gz);//-sphere_list[i].radius_;
    //ROS_INFO_STREAM("Point " << p.x() << " " << p.y() << " " << p.z() << " dist " << dist);
    
    if(dist < distance) {
      distance = dist;
      closest = i;
      gradient = btVector3(gx,gy,gz);
    }
  }
  return distance;
}

bool CollisionProximitySpace::getEnvironmentCollision(const std::vector<std::string>& link_names)
{
  updating_objects_lock_.lock();
  bool any_link_in_coll = false;
  for(unsigned int i = 0; i < link_names.size(); i++) {
    if(body_decomposition_map_.find(link_names[i]) == body_decomposition_map_.end()) {
      ROS_WARN_STREAM("No link " << link_names[i] << " for getEnvironmentCollision");
      continue;
    }
    const BodyDecomposition* bd = body_decomposition_map_.find(link_names[i])->second;
    bool coll = getCollisionSphereCollision(bd->getCollisionSpheres());
    if(coll) {
      ROS_INFO_STREAM("Link " << link_names[i] << " in collision");
      any_link_in_coll = true;
    }
    const planning_models::KinematicModel::LinkModel* lm = monitor_->getKinematicModel()->getLinkModel(link_names[i]);
    for(unsigned int j = 0; j < lm->getAttachedBodyModels().size(); j++) {
      std::string id = makeAttachedObjectId(lm->getName(),lm->getAttachedBodyModels()[j]->getName());
      if(attached_object_map_.find(id) == attached_object_map_.end()) {
        ROS_WARN_STREAM("Have no attached object body for attached object " << id);
        continue;
      }
      const BodyDecompositionVector* att = attached_object_map_[id];
      for(unsigned int k = 0; k < att->getSize(); k++) {
        coll = getCollisionSphereCollision(att->getBodyDecomposition(k)->getCollisionSpheres());
        if(coll) {
          ROS_INFO_STREAM("Attached body " << id << " in collision");
          any_link_in_coll = true;
        }
      }
    }
  }
  updating_objects_lock_.unlock();
  return any_link_in_coll;
}
*/
bool CollisionProximitySpace::getGroupLinkAndAttachedBodyNames(const std::string& group_name,
                                                               std::vector<std::string>& link_names,
                                                               std::vector<unsigned int>& link_indices,
                                                               std::vector<std::string>& attached_body_names,
                                                               std::vector<unsigned int>& attached_body_link_indices) const
{
  planning_models::KinematicState state(monitor_->getKinematicModel());
  ros::WallTime n1 = ros::WallTime::now();
  const std::map<std::string, std::vector<std::string> >& pgl = monitor_->getRobotModels()->getPlanningGroupLinks();

  link_names = pgl.find(group_name)->second;
  if(link_names.size() == 0) {
    return false;
  }

  //making sure that we have bodies for the links - otherwise they don't have collision geometries
  std::vector<std::string>::iterator it = link_names.begin();
  while(it != link_names.end()) {
    if(body_decomposition_map_.find(*it) == body_decomposition_map_.end()) {
      it = link_names.erase(it);
    } else {
      it++;
    }
  }

  attached_body_names.clear();
  link_indices.clear();
  attached_body_link_indices.clear();
  const std::vector<planning_models::KinematicState::LinkState*>& lsv = state.getLinkStateVector();
  for(unsigned int i = 0; i < link_names.size(); i++) {
    bool found = false;
    for(unsigned int j = 0; j < lsv.size(); j++) {
      if(lsv[j]->getName() == link_names[i]) {
        link_indices.push_back(j);
        found = true;
        break;
      }
    }
    if(!found) {
      ROS_INFO_STREAM("No link state found for link " << link_names[i]);
    }
    if(link_attached_objects_.find(link_names[i]) != link_attached_objects_.end()) {
      unsigned int j = 0;
      for(std::map<std::string, bool>::const_iterator it = link_attached_objects_.find(link_names[i])->second.begin();
          it != link_attached_objects_.find(link_names[i])->second.end();
          it++, j++) {
        attached_body_names.push_back(it->first);
        attached_body_link_indices.push_back(link_indices[i]);
      }
    }
  }
  return true;
}

bool CollisionProximitySpace::setupGradientStructures(const std::vector<std::string>& link_names,
                                                      const std::vector<std::string>& attached_body_names, 
                                                      std::vector<double>& link_closest_distances, 
                                                      std::vector<std::vector<double> >& closest_distances, 
                                                      std::vector<std::vector<btVector3> >& closest_gradients) const
{
  if(link_names.size() == 0) {
    ROS_WARN_STREAM("No link names in setup gradient structure");
    return false;
  }

  unsigned int att_count = attached_body_names.size();
  closest_distances.clear();
  closest_gradients.clear();
  closest_distances.resize(link_names.size()+att_count);
  closest_gradients.resize(link_names.size()+att_count);
  link_closest_distances.clear();
  link_closest_distances.resize(link_names.size()+att_count, DBL_MAX);

  for(unsigned int i = 0; i < link_names.size(); i++) {
    const std::vector<CollisionSphere>& lcs1 = body_decomposition_map_.find(link_names[i])->second->getCollisionSpheres();
    closest_distances[i].resize(lcs1.size(), DBL_MAX);
    closest_gradients[i].resize(lcs1.size());
  }
  unsigned int att_index = link_names.size();
  for(unsigned int i = 0; i < att_count; i++) {
    if(attached_object_map_.find(attached_body_names[i]) == attached_object_map_.end()) {
      ROS_WARN_STREAM("No attached object " << attached_body_names[i]);
      return false;
    }
    const std::vector<CollisionSphere>& att_vec = attached_object_map_.find(attached_body_names[i])->second->getCollisionSpheres();
    closest_distances[att_index].resize(att_vec.size(), DBL_MAX);
    closest_gradients[att_index].resize(att_vec.size());
    att_index++;
  }
  return true;
}

bool CollisionProximitySpace::isStateInCollision() const
{
  if(isEnvironmentCollision()) return true;
  return isIntraGroupCollision();
}

bool CollisionProximitySpace::getStateCollisions(std::vector<std::string>& link_names, 
                                                 std::vector<std::string>& attached_body_names,
                                                 bool& in_collision, 
                                                 std::vector<CollisionType>& collisions) const
{
  link_names = current_link_names_;
  attached_body_names = current_attached_body_names_;
  collisions.clear();
  collisions.resize(current_link_names_.size()+current_attached_body_names_.size(), NONE);
  std::vector<bool> env_collisions, intra_collisions;
  env_collisions.resize(collisions.size(), false);
  intra_collisions = env_collisions;
  bool env_collision = getEnvironmentCollisions(env_collisions, false);
  bool intra_group_collision = getIntraGroupCollisions(intra_collisions, false);
  if(env_collision || intra_group_collision) {
    for(unsigned int i = 0; i < current_link_names_.size()+current_attached_body_names_.size(); i++) {
      if(intra_collisions[i]) {
        if(!env_collisions[i]) {
          collisions[i] = INTRA_GROUP;
        } else {
          collisions[i] = INTRA_GROUP_AND_ENVIRONMENT;
        }
      } else if(env_collisions[i]) {
        collisions[i] = ENVIRONMENT;
      }
    } 
    in_collision = true;
  } else {
    in_collision = false;
  }
  return in_collision;
}

bool CollisionProximitySpace::getStateGradients(std::vector<std::string>& link_names,
                                                std::vector<std::string>& attached_body_names,
                                                std::vector<double>& link_closest_distances, 
                                                std::vector<std::vector<double> >& closest_distances, 
                                                std::vector<std::vector<btVector3> >& closest_gradients,
                                                bool subtract_radii) const
{
  link_names = current_link_names_;
  attached_body_names = current_attached_body_names_;
  std::vector<double> link_closest_distances_intra, link_closest_distances_env;
  std::vector<std::vector<double> > closest_distances_env, closest_distances_intra; 
  std::vector<std::vector<btVector3> > closest_gradients_env, closest_gradients_intra; 
  link_closest_distances_intra = link_closest_distances_env = link_closest_distances = current_link_distances_;
  closest_distances_env = closest_distances_intra = closest_distances = current_closest_distances_;
  closest_gradients_env = closest_gradients_intra = closest_gradients = current_closest_gradients_;

  bool env_coll = getEnvironmentProximityGradients(link_closest_distances_env, closest_distances_env, closest_gradients_env, subtract_radii);
  bool intra_coll = getIntraGroupProximityGradients(link_closest_distances_intra, closest_distances_intra, closest_gradients_intra, subtract_radii);

  for(unsigned int i = 0; i < closest_distances.size(); i++) {
    if(link_closest_distances_intra[i] > link_closest_distances_env[i]) {
      link_closest_distances[i] = link_closest_distances_env[i];
    } else {
      link_closest_distances[i] = link_closest_distances_intra[i];
    }
    for(unsigned int j = 0; j < closest_distances[i].size(); j++) {
      if(closest_distances_env[i][j] > closest_distances_intra[i][j] ||
         closest_distances_env[i][j] >= max_environment_distance_) {
        closest_distances[i][j] = closest_distances_intra[i][j];
        closest_gradients[i][j] = closest_gradients_intra[i][j];
      } else {
        closest_distances[i][j] = closest_distances_env[i][j];
        closest_gradients[i][j] = closest_gradients_env[i][j];
      }
    }
  }
  return (env_coll || intra_coll);
}

bool CollisionProximitySpace::isIntraGroupCollision() const
{
  std::vector<bool> collisions;
  return(getIntraGroupCollisions(collisions, true));
}

bool CollisionProximitySpace::getIntraGroupCollisions(std::vector<bool>& collisions, bool stop_at_first_collision) const {
  bool in_collision = false;
  unsigned int num_links = current_link_names_.size();
  unsigned int num_attached = current_attached_body_names_.size();
  unsigned int tot = num_links+num_attached;
  for(unsigned int i = 0; i < tot; i++) {
    for(unsigned int j = i; j < tot; j++) {
      if(i == j) continue;
      if(!current_intra_group_collision_links_[i][j]) continue;
      const std::vector<CollisionSphere>* lcs1;
      const std::vector<CollisionSphere>* lcs2; 
      if(i < num_links) {
        lcs1 = &(current_link_body_decompositions_[i]->getCollisionSpheres());
      } else {
        lcs1 = &(current_attached_body_decompositions_[i-num_links]->getCollisionSpheres());
      }
      if(j < num_links) {
        lcs2 = &(current_link_body_decompositions_[j]->getCollisionSpheres());
      } else {
        lcs2 = &(current_attached_body_decompositions_[j-num_links]->getCollisionSpheres());
      }
      for(unsigned int k = 0; k < lcs1->size(); k++) {
        for(unsigned int l = 0; l < lcs2->size(); l++) {
          double dist = (*lcs1)[k].center_.distance((*lcs2)[l].center_);
          dist += -(*lcs1)[k].radius_-(*lcs2)[l].radius_;
          if(dist <= tolerance_) {
            if(stop_at_first_collision) {
              return true;
            }
            collisions[i] = true;
            collisions[j] = true;
            in_collision = true;
          }
        }
      }
    }
  }
  return in_collision;
}



bool CollisionProximitySpace::getIntraGroupProximityGradients(std::vector<double>& link_closest_distances,  
                                                              std::vector<std::vector<double> >& closest_distances, 
                                                              std::vector<std::vector<btVector3> >& closest_gradients,
                                                              bool subtract_radii) const {
  bool in_collision = false;
  unsigned int count = 0;
  unsigned int num_links = current_link_names_.size();
  unsigned int num_attached = current_attached_body_names_.size();
  unsigned int tot = num_links+num_attached;
  for(unsigned int i = 0; i < tot; i++) {
    for(unsigned int j = i; j < tot; j++) {
      if(i == j) continue;
      if(!current_intra_group_collision_links_[i][j]) continue;
      const std::vector<CollisionSphere>* lcs1;
      const std::vector<CollisionSphere>* lcs2; 
      if(i < num_links) {
        lcs1 = &(current_link_body_decompositions_[i]->getCollisionSpheres());
      } else {
        lcs1 = &(current_attached_body_decompositions_[i-num_links]->getCollisionSpheres());
      }
      if(j < num_links) {
        lcs2 = &(current_link_body_decompositions_[j]->getCollisionSpheres());
      } else {
        lcs2 = &(current_attached_body_decompositions_[j-num_links]->getCollisionSpheres());
      }
      for(unsigned int k = 0; k < lcs1->size(); k++) {
        for(unsigned int l = 0; l < lcs2->size(); l++) {
          double dist = (*lcs1)[k].center_.distance((*lcs2)[l].center_);
          if(subtract_radii) {
            dist += -(*lcs1)[k].radius_-(*lcs2)[l].radius_;
            if(dist <= tolerance_) {
              in_collision = true;
            }
          }
          count++;
          if(dist < closest_distances[i][k]) {
            closest_distances[i][k] = dist;
            //ROS_INFO_STREAM("Setting dist for " << name1 << " sphere " << k << " to " << dist << " closest " << name2 << " sphere " << l);
            closest_gradients[i][k] = (*lcs1)[k].center_-(*lcs2)[l].center_;
          }
          if(dist < link_closest_distances[i]) {
            link_closest_distances[i] = dist;
          }
          if(dist < closest_distances[j][l]) {
            closest_distances[j][l] = dist;
            closest_gradients[j][l] = (*lcs2)[l].center_-(*lcs1)[k].center_;
          }
          if(dist < link_closest_distances[j]) {
            link_closest_distances[j] = dist;
          }
        }
      }
    }
  }
  return in_collision;
}

bool CollisionProximitySpace::isEnvironmentCollision() const
{
  std::vector<bool> collisions;
  return(getEnvironmentCollisions(collisions, true));
}

bool CollisionProximitySpace::getEnvironmentCollisions(std::vector<bool>& collisions,
                                                       bool stop_at_first_collision) const
{
  bool in_collision = false;
  for(unsigned int i = 0; i < current_link_names_.size(); i++) {
    if(!current_environment_excludes_[i]) continue;
    const std::vector<CollisionSphere>& body_spheres = current_link_body_decompositions_[i]->getCollisionSpheres();
    bool coll = getCollisionSphereCollision(distance_field_, body_spheres, tolerance_);
    if(coll) {
      if(stop_at_first_collision) {
        return true;
      }
      in_collision = true;
      collisions[i] = true;
    }
  }
  for(unsigned int i = 0; i < current_attached_body_names_.size(); i++) {
    const std::vector<CollisionSphere>& body_spheres = current_attached_body_decompositions_[i]->getCollisionSpheres();
    bool coll = getCollisionSphereCollision(distance_field_, body_spheres, tolerance_);
    if(coll) {
      if(stop_at_first_collision) {
        return true;
      }
      in_collision = true;
      collisions[i+current_link_names_.size()] = true;
    }
  }
  return in_collision;
}



bool CollisionProximitySpace::getEnvironmentProximityGradients(std::vector<double>& link_closest_distances, 
                                                               std::vector<std::vector<double> >& closest_distances, 
                                                               std::vector<std::vector<btVector3> >& closest_gradients,
                                                               bool subtract_radii) const {
  unsigned int tot = current_link_names_.size()+current_attached_body_names_.size();
  if(link_closest_distances.size() != tot) {
    ROS_WARN_STREAM("Wrong sized link closest " << tot << " " << link_closest_distances.size());
    return false;
  }
  bool in_collision = false;
  for(unsigned int i = 0; i < current_link_names_.size(); i++) {
    if(!current_environment_excludes_[i]) continue;
    const std::vector<CollisionSphere>& body_spheres = current_link_body_decompositions_[i]->getCollisionSpheres();
    if(closest_distances[i].size() != body_spheres.size()) {
      ROS_INFO_STREAM("Wrong size for closest distances for link " << current_link_names_[i]);
    }
    bool coll = getCollisionSphereGradients(distance_field_, body_spheres, link_closest_distances[i], closest_distances[i], closest_gradients[i], tolerance_, subtract_radii, false);
    if(coll) {
      in_collision = true;
    }
  }
  for(unsigned int i = 0; i < current_attached_body_names_.size(); i++) {
    const std::vector<CollisionSphere>& body_spheres = current_attached_body_decompositions_[i]->getCollisionSpheres();
    bool coll = getCollisionSphereGradients(distance_field_, body_spheres, link_closest_distances[i+current_link_names_.size()], closest_distances[i+current_link_names_.size()], closest_gradients[i+current_link_names_.size()], tolerance_, subtract_radii, false);
    if(coll) {
      in_collision = true;
    }
  }
  return in_collision;
}

void CollisionProximitySpace::visualizeProximityGradients(const std::vector<std::string>& link_names, 
                                                          const std::vector<std::string>& attached_body_names, 
                                                          const std::vector<double>& link_closest_distance, 
                                                          const std::vector<std::vector<double> >& closest_distances, 
                                                          const std::vector<std::vector<btVector3> >& closest_gradients) const {

  visualization_msgs::MarkerArray arr;
  for(unsigned int i = 0; i < closest_gradients.size(); i++) {
    
    const std::vector<CollisionSphere>* lcs;
    std::string name;
    if(i < link_names.size()) {
      name = link_names[i];
      lcs = &(body_decomposition_map_.find(name)->second->getCollisionSpheres());
    } else {
      name = attached_body_names[i-link_names.size()];
      lcs = &(attached_object_map_.find(name)->second->getCollisionSpheres());
    }
    for(unsigned int j = 0; j < closest_gradients[i].size(); j++) {
      visualization_msgs::Marker arrow_mark;
      arrow_mark.header.frame_id = monitor_->getRobotFrameId();
      arrow_mark.header.stamp = ros::Time::now();
      arrow_mark.ns = "self_coll_gradients";
      arrow_mark.id = i*1000+j;
      double xscale = 0.0;
      double yscale = 0.0;
      double zscale = 0.0;
      if(closest_distances[i][j] > 0.0) {
        if(closest_gradients[i][j].length() > 0.0) {
          xscale = closest_gradients[i][j].x()/closest_gradients[i][j].length();
          yscale = closest_gradients[i][j].y()/closest_gradients[i][j].length();
          zscale = closest_gradients[i][j].z()/closest_gradients[i][j].length();
        } else {
          ROS_DEBUG_STREAM("Negative length for " << name << " " << arrow_mark.id << " " << closest_gradients[i][j].length());
        }
      } else {
        ROS_DEBUG_STREAM("Negative dist for " << name << " " << arrow_mark.id);
      }
      arrow_mark.points.resize(2);
      arrow_mark.points[1].x = (*lcs)[j].center_.x();
      arrow_mark.points[1].y = (*lcs)[j].center_.y();
      arrow_mark.points[1].z = (*lcs)[j].center_.z();
      arrow_mark.points[0] = arrow_mark.points[1];
      arrow_mark.points[0].x -= xscale*closest_distances[i][j];
      arrow_mark.points[0].y -= yscale*closest_distances[i][j];
      arrow_mark.points[0].z -= zscale*closest_distances[i][j];
      arrow_mark.scale.x = 0.01;
      arrow_mark.scale.y = 0.03;
      arrow_mark.color.r = 1.0;
      arrow_mark.color.g = 0.2;
      arrow_mark.color.b = .5;
      arrow_mark.color.a = 1.0;
      arr.markers.push_back(arrow_mark);
    }
  }
  vis_marker_array_publisher_.publish(arr);
}

////////////
// Visualization functions
///////////
  
void CollisionProximitySpace::visualizeDistanceField() const
{
  btTransform ident;
  ident.setIdentity();
  distance_field_->visualize(0.0, 0.0, monitor_->getRobotFrameId(), ident, ros::Time::now());
}

/*
void CollisionProximitySpace::visualizeClosestCollisionSpheres(const std::vector<std::string>& link_names) const 
{
  planning_models::KinematicState state(monitor_->getKinematicModel());
  ProximityInfo prox;
  getEnvironmentProximity(current_link_names, prox);
  const BodyDecomposition* bsd = body_decomposition_map_.find(prox.link_name)->second;
  if(!prox.attached_object_name.empty()) {
    bsd = attached_object_map_.find(prox.attached_object_name)->second->getBodyDecomposition(prox.att_index);
  }
  if(prox.sphere_index < bsd->getCollisionSpheres().size()) {
    visualization_msgs::Marker mark;
    mark.header.frame_id = monitor_->getRobotFrameId();
    mark.header.stamp = ros::Time::now();
    mark.type = mark.SPHERE;
    mark.ns = "closest_sphere";
    mark.id = 0;
    mark.scale.x = bsd->getCollisionSpheres()[prox.sphere_index].radius_*2.0;
    mark.scale.y = mark.scale.x;
    mark.scale.z = mark.scale.x;
    mark.color.b = 1.0;
    mark.color.a = .5;
    mark.pose.position.x = bsd->getCollisionSpheres()[prox.sphere_index].center_.x();
    mark.pose.position.y = bsd->getCollisionSpheres()[prox.sphere_index].center_.y();
    mark.pose.position.z = bsd->getCollisionSpheres()[prox.sphere_index].center_.z();
    mark.pose.orientation.w = 1.0;
    vis_marker_publisher_.publish(mark);
  }

  std::vector<CollisionSphere> all_collision_spheres;
  for(unsigned int i = 0; i < link_names.size(); i++) {
    const planning_models::KinematicState::LinkState* ls = state.getLinkState(link_names[i]);
    if(body_decomposition_map_.find(ls->getName()) == body_decomposition_map_.end()) {
      //ROS_WARN_STREAM("Can't find body decomposition for link state " << ls->getName());
      continue;
    }
    const std::vector<CollisionSphere>& body_spheres = body_decomposition_map_.find(ls->getName())->second->getCollisionSpheres();
    all_collision_spheres.insert(all_collision_spheres.end(), body_spheres.begin(), body_spheres.end());
    for(unsigned int j = 0; j < ls->getAttachedBodyStateVector().size(); j++) {
      const planning_models::KinematicState::AttachedBodyState* att_state = ls->getAttachedBodyStateVector()[j];
      std::string id = makeAttachedObjectId(ls->getName(), att_state->getName());
      if(attached_object_map_.find(id) == attached_object_map_.end()) {
        continue;
      }
      const std::vector<CollisionSphere>& body_spheres2 = attached_object_map_.find(id)->second->getCollisionSpheres();
      all_collision_spheres.insert(all_collision_spheres.end(), body_spheres2.begin(), body_spheres2.end());
    }
  }
  std::vector<btVector3> all_collision_gradients;
  std::vector<double> distances;
  double closest_distance;
  getCollisionSphereGradients(all_collision_spheres, closest_distance, distances, all_collision_gradients);
  visualization_msgs::MarkerArray arr;
  for(unsigned int i = 0; i < all_collision_gradients.size(); i++) {
    visualization_msgs::Marker arrow_mark;
    arrow_mark.header.frame_id = monitor_->getRobotFrameId();
    arrow_mark.header.stamp = ros::Time::now();
    arrow_mark.ns = "closest_sphere";
    arrow_mark.id = i+1;
    double xscale = 0.0;
    double yscale = 0.0;
    double zscale = 0.0;
    if(all_collision_gradients[i].length() > 0.0) {
      xscale = all_collision_gradients[i].x()/all_collision_gradients[i].length();
      yscale = all_collision_gradients[i].y()/all_collision_gradients[i].length();
      zscale = all_collision_gradients[i].z()/all_collision_gradients[i].length();
    }
    arrow_mark.points.resize(2);
    arrow_mark.points[0].x = all_collision_spheres[i].center_.x();
    arrow_mark.points[0].y = all_collision_spheres[i].center_.y();
    arrow_mark.points[0].z = all_collision_spheres[i].center_.z();
    arrow_mark.points[1] = arrow_mark.points[0];
    arrow_mark.points[1].x -= xscale*distances[i];
    arrow_mark.points[1].y -= yscale*distances[i];
    arrow_mark.points[1].z -= zscale*distances[i];
    arrow_mark.scale.x = 0.01;
    arrow_mark.scale.y = 0.03;
    arrow_mark.color.r = 1.0;
    arrow_mark.color.g = 0.2;
    arrow_mark.color.b = .5;
    arrow_mark.color.a = 1.0;
    arr.markers.push_back(arrow_mark);
  }

  vis_marker_array_publisher_.publish(arr);
}
*/
void CollisionProximitySpace::visualizeCollisions(const std::vector<std::string>& link_names, 
                                                  const std::vector<std::string>& attached_body_names, 
                                                  const std::vector<CollisionType> collisions) const
{
  visualization_msgs::MarkerArray arr;

  for(unsigned int i = 0; i < link_names.size(); i++) {
    std::string name = link_names[i];
    //if(collisions[i] != NONE) {
      boost::shared_ptr<const urdf::Link> urdf_link = monitor_->getRobotModels()->getParsedDescription()->getLink(name);
      if(urdf_link == NULL) {
        ROS_INFO_STREAM("No entry in urdf for link " << name);
        continue;
      }
      if(!urdf_link->collision) {
        continue;
      }
      const urdf::Geometry *geom = urdf_link->collision->geometry.get();
      if(!geom) {
        ROS_DEBUG_STREAM("No collision geometry for link " << name);
        continue;
      }
      const urdf::Mesh *mesh = dynamic_cast<const urdf::Mesh*>(geom);
      if(mesh) {
        if (!mesh->filename.empty()) {
          visualization_msgs::Marker mark;
          mark.header.frame_id = monitor_->getRobotFrameId();
          mark.header.stamp = ros::Time::now();
          mark.ns = "proximity_collisions";
          mark.id = i;
          mark.type = mark.MESH_RESOURCE;
          mark.scale.x = 1.05;
          mark.scale.y = 1.05;
          mark.scale.z = 1.05;
          mark.color.a = .5;
          if(collisions[i] == INTRA_GROUP) {
            ROS_DEBUG_STREAM(name << " in intra_group");
            mark.color.r = 1.0;
          } else if(collisions[i] == ENVIRONMENT) {
            ROS_DEBUG_STREAM(name << " in environment");
            mark.color.r = 1.0;
            mark.color.g = 1.0;
          } else if(collisions[i] == INTRA_GROUP_AND_ENVIRONMENT) {
            ROS_DEBUG_STREAM(name << " in intra_group and environment");
            mark.color.b = 1.0;
          } else {
            mark.color.g = 1.0;
          }
          //mark.lifetime = ros::Duration(1.0);
          mark.frame_locked = true;
          const btTransform& trans = body_decomposition_map_.find(name)->second->getBody()->getPose();
          tf::poseTFToMsg(trans, mark.pose);
          mark.mesh_resource = mesh->filename;
          arr.markers.push_back(mark);
        }
      }
      //}
  }
  for(unsigned int i = 0; i < attached_body_names.size(); i++) {
    std::string name = attached_body_names[i];
    const BodyDecompositionVector* bdv = attached_object_map_.find(attached_body_names[i])->second;
    for(unsigned int j = 0; j < bdv->getSize(); j++) {
      const BodyDecomposition* bd = bdv->getBodyDecomposition(j);
      visualization_msgs::Marker mark;
      mark.header.frame_id = monitor_->getRobotFrameId();
      mark.header.stamp = ros::Time::now();
      mark.ns = "proximity_collisions";
      mark.id = i+link_names.size();      
      mark.type = mark.CYLINDER;
      bodies::BoundingCylinder cyl;
      bd->getBody()->computeBoundingCylinder(cyl);
      mark.scale.x = cyl.radius*2.0;
      mark.scale.y = cyl.radius*2.0;
      mark.scale.z = cyl.length;
      mark.color.a = .5;
      if(collisions[i+link_names.size()] == INTRA_GROUP) {
        ROS_DEBUG_STREAM(name << " in intra_group");
        mark.color.r = 1.0;
      } else if(collisions[i+link_names.size()] == ENVIRONMENT) {
        ROS_DEBUG_STREAM(name << " in environment");
        mark.color.r = 1.0;
        mark.color.g = 1.0;
      } else if(collisions[i+link_names.size()] == INTRA_GROUP_AND_ENVIRONMENT) {
        ROS_DEBUG_STREAM(name << " in intra_group and environment");
        mark.color.b = 1.0;
      } else {
        mark.color.g = 1.0;
      }
      //mark.lifetime = ros::Duration(1.0);
      const btTransform& trans = bd->getBody()->getPose();
      tf::poseTFToMsg(trans, mark.pose);
      arr.markers.push_back(mark);
    }
  }
  vis_marker_array_publisher_.publish(arr);
}

void CollisionProximitySpace::visualizeObjectVoxels(const std::vector<std::string>& object_names) const
{
  visualization_msgs::Marker cube_list;
  cube_list.header.frame_id = monitor_->getRobotFrameId();
  cube_list.header.stamp = ros::Time::now();
  cube_list.type = cube_list.CUBE_LIST;
  cube_list.ns = "body_voxels";
  cube_list.id = 1000;
  cube_list.scale.x = resolution_;
  cube_list.scale.y = resolution_;
  cube_list.scale.z = resolution_;
  cube_list.color.b = 1.0;
  cube_list.color.a = .5;  
  for(unsigned int i = 0; i < object_names.size(); i++) {
    const std::vector<btVector3>* coll_points;
    if(body_decomposition_map_.find(object_names[i]) != body_decomposition_map_.end()) {
      coll_points = &(body_decomposition_map_.find(object_names[i])->second)->getCollisionPoints();
    } else if(static_object_map_.find(object_names[i]) != static_object_map_.end()) {
      coll_points = &(static_object_map_.find(object_names[i])->second)->getCollisionPoints();
    } else if(attached_object_map_.find(object_names[i]) != attached_object_map_.end()) {
      coll_points = &(attached_object_map_.find(object_names[i])->second)->getCollisionPoints();
    } else {
      ROS_WARN_STREAM("Don't have object named " << object_names[i]);
      continue;
    }
    for(unsigned int i = 0; i < coll_points->size(); i++) {
      geometry_msgs::Point p;
      p.x = (*coll_points)[i].x();
      p.y = (*coll_points)[i].y();
      p.z = (*coll_points)[i].z();
      cube_list.points.push_back(p);
    }
  }
  vis_marker_publisher_.publish(cube_list);
}

void CollisionProximitySpace::visualizeObjectSpheres(const std::vector<std::string>& object_names) const
{
  visualization_msgs::MarkerArray arr;
  unsigned int count = 0;
  for(unsigned int i = 0; i < object_names.size(); i++) {
    // visualization_msgs::Marker sphere_list;
    // sphere_list.header.frame_id = monitor_->getRobotFrameId();
    // sphere_list.header.stamp = ros::Time::now();
    // sphere_list.type = sphere_list.SPHERE_LIST;
    // sphere_list.ns = "body_spheres";
    // sphere_list.id = i;
    // sphere_list.color.g = 1.0;
    // sphere_list.color.a = .5;  
    const std::vector<CollisionSphere>* coll_spheres;
    if(body_decomposition_map_.find(object_names[i]) != body_decomposition_map_.end()) {
      coll_spheres = &(body_decomposition_map_.find(object_names[i])->second)->getCollisionSpheres();
    } else if(static_object_map_.find(object_names[i]) != static_object_map_.end()) {
      coll_spheres = &(static_object_map_.find(object_names[i])->second)->getCollisionSpheres();
    } else if(attached_object_map_.find(object_names[i]) != attached_object_map_.end()) {
      coll_spheres = &(attached_object_map_.find(object_names[i])->second)->getCollisionSpheres();
    } else {
      ROS_WARN_STREAM("Don't have object named " << object_names[i]);
      continue;
    }
    // sphere_list.scale.x = (*coll_spheres)[0].radius_*2.0;
    // sphere_list.scale.y = sphere_list.scale.x;
    // sphere_list.scale.z = sphere_list.scale.x;
    for(unsigned int i = 0; i < coll_spheres->size(); i++) {
      visualization_msgs::Marker sphere;
      sphere.header.frame_id = monitor_->getRobotFrameId();
      sphere.header.stamp = ros::Time::now();
      sphere.type = sphere.SPHERE;
      sphere.ns = "body_spheres";
      sphere.id = count++;
      sphere.color.g = 1.0;
      sphere.color.a = .5;  
      sphere.scale.x = (*coll_spheres)[i].radius_*2.0;
      sphere.scale.y = sphere.scale.x;
      sphere.scale.z = sphere.scale.x;      
      geometry_msgs::Point p;
      sphere.pose.position.x = (*coll_spheres)[i].center_.x();
      sphere.pose.position.y = (*coll_spheres)[i].center_.y();
      sphere.pose.position.z = (*coll_spheres)[i].center_.z();
      sphere.points.push_back(p);
      arr.markers.push_back(sphere);
    }
  }
  vis_marker_array_publisher_.publish(arr);
}

void CollisionProximitySpace::visualizePaddedTrimeshes(const planning_models::KinematicState& state, const std::vector<std::string>& link_names) const {
  btTransform inv = getInverseWorldTransform(state);
  visualization_msgs::MarkerArray arr;
  unsigned int count = 0;
  for(unsigned int i = 0; i < link_names.size(); i++) {
    if(state.getLinkState(link_names[i]) == NULL) continue;
    const planning_models::KinematicModel::LinkModel* lm = state.getLinkState(link_names[i])->getLinkModel();
    const btTransform& trans = inv*state.getLinkState(link_names[i])->getGlobalCollisionBodyTransform();
    if(lm == NULL) continue;
    if(lm->getLinkShape() == NULL) continue;
    const shapes::Mesh *mesh = dynamic_cast<const shapes::Mesh*>(lm->getLinkShape());
    if(mesh == NULL) continue;
    if (mesh->vertexCount > 0 && mesh->triangleCount > 0)
    {	
      visualization_msgs::Marker mesh_mark;
      mesh_mark.header.frame_id = monitor_->getRobotFrameId();
      mesh_mark.header.stamp = ros::Time::now();
      mesh_mark.ns = link_names[i]+"_trimesh";
      mesh_mark.id = count++;
      mesh_mark.type = mesh_mark.LINE_LIST;
      mesh_mark.scale.x = mesh_mark.scale.y = mesh_mark.scale.z = .001;	     
      mesh_mark.color.r = 0.0;
      mesh_mark.color.g = 0.0;
      mesh_mark.color.b = 1.0;
      mesh_mark.color.a = 1.0;
      double padding = 0.0;
      if(monitor_->getCollisionModels()->getDefaultLinkPaddingMap().find(link_names[i]) !=
         monitor_->getCollisionModels()->getDefaultLinkPaddingMap().end()) {
        padding = monitor_->getCollisionModels()->getDefaultLinkPaddingMap().find(link_names[i])->second;
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
        btVector3 vec1_trans = trans*vec1;

        btVector3 vec2(vertices[t2ind*3],
                       vertices[t2ind*3+1],
                       vertices[t2ind*3+2]);
        btVector3 vec2_trans = trans*vec2;

        btVector3 vec3(vertices[t3ind*3],
                       vertices[t3ind*3+1],
                       vertices[t3ind*3+2]);
        btVector3 vec3_trans = trans*vec3;

        geometry_msgs::Point pt1;
        pt1.x = vec1_trans.x();
        pt1.y = vec1_trans.y();
        pt1.z = vec1_trans.z();

        geometry_msgs::Point pt2;
        pt2.x = vec2_trans.x();
        pt2.y = vec2_trans.y();
        pt2.z = vec2_trans.z();

        geometry_msgs::Point pt3;
        pt3.x = vec3_trans.x();
        pt3.y = vec3_trans.y();
        pt3.z = vec3_trans.z();
        
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
  vis_marker_array_publisher_.publish(arr);
}

void CollisionProximitySpace::visualizeConvexMeshes(const std::vector<std::string>& link_names) const {
  visualization_msgs::MarkerArray arr;
  unsigned int count = 0;
  for(unsigned int i = 0; i < link_names.size(); i++) {
    if(body_decomposition_map_.find(link_names[i]) == body_decomposition_map_.end())
      continue;
    const BodyDecomposition* bsd = body_decomposition_map_.find(link_names[i])->second;
    const bodies::ConvexMesh* mesh = dynamic_cast<const bodies::ConvexMesh*>(bsd->getBody());
    if(mesh) {     
      // for(unsigned int j = 0; j < bsd->getBodies().size(); j++) {
      //   const bodies::ConvexMesh* mesh = dynamic_cast<const bodies::ConvexMesh*>(bsd->getBodies()[j]);
      //   if(!mesh) continue;
      visualization_msgs::Marker mesh_mark;
      mesh_mark.header.frame_id = monitor_->getRobotFrameId();
      mesh_mark.header.stamp = ros::Time::now();
      mesh_mark.ns = link_names[i]+"_convex";
      mesh_mark.id = count++;
      mesh_mark.type = mesh_mark.LINE_LIST;
      mesh_mark.scale.x = mesh_mark.scale.y = mesh_mark.scale.z = .001;
      if(count-1 >= colors_.size()) {
        std::vector<double> color(3);
        color[0] = gen_rand(0.0,1.0);
        color[1] = gen_rand(0.0,1.0);
        color[2] = gen_rand(0.0,1.0);
        colors_.push_back(color);
      }
      mesh_mark.color.r = colors_[count-1][0];
      mesh_mark.color.g = colors_[count-1][1];
      mesh_mark.color.b = colors_[count-1][2];
      mesh_mark.color.a = 1.0;
      for(unsigned int k = 0; k < mesh->getTriangles().size()/3; k++) {
        unsigned int v = mesh->getTriangles()[3*k];
        btVector3 vec1(mesh->getScaledVertices()[v].x(),
                       mesh->getScaledVertices()[v].y(),
                       mesh->getScaledVertices()[v].z());
        btVector3 vec1_trans = bsd->getBody()->getPose()*vec1;
        geometry_msgs::Point pt1;
        pt1.x = vec1_trans.x();
        pt1.y = vec1_trans.y();
        pt1.z = vec1_trans.z();

        v = mesh->getTriangles()[3*k+1];
        btVector3 vec2(mesh->getScaledVertices()[v].x(),
                       mesh->getScaledVertices()[v].y(),
                       mesh->getScaledVertices()[v].z());
        btVector3 vec2_trans = bsd->getBody()->getPose()*vec2;
        geometry_msgs::Point pt2;
        pt2.x = vec2_trans.x();
        pt2.y = vec2_trans.y();
        pt2.z = vec2_trans.z();

        v = mesh->getTriangles()[3*k+2];
        btVector3 vec3(mesh->getScaledVertices()[v].x(),
                       mesh->getScaledVertices()[v].y(),
                       mesh->getScaledVertices()[v].z());
        btVector3 vec3_trans = bsd->getBody()->getPose()*vec3;
        geometry_msgs::Point pt3;
        pt3.x = vec3_trans.x();
        pt3.y = vec3_trans.y();
        pt3.z = vec3_trans.z();

        mesh_mark.points.push_back(pt1);
        mesh_mark.points.push_back(pt2);        

        mesh_mark.points.push_back(pt1);
        mesh_mark.points.push_back(pt3);
        
        mesh_mark.points.push_back(pt2);
        mesh_mark.points.push_back(pt3);
      }
      arr.markers.push_back(mesh_mark);
    }
  }
  vis_marker_array_publisher_.publish(arr);
}

void CollisionProximitySpace::visualizeBoundingCylinders(const std::vector<std::string>& object_names) const
{
  visualization_msgs::MarkerArray arr;
  for(unsigned int i = 0; i < object_names.size(); i++) {
    visualization_msgs::Marker mark;
    mark.header.frame_id = monitor_->getRobotFrameId();
    mark.header.stamp = ros::Time::now();
    mark.type = mark.CYLINDER;
    mark.ns = "body_cylinders";
    mark.id = i;
    mark.color.r = 1.0;
    mark.color.a = .5;  
    const BodyDecomposition* bsd = NULL;
    if(body_decomposition_map_.find(object_names[i]) != body_decomposition_map_.end()) {
      bsd = body_decomposition_map_.find(object_names[i])->second;
    } else if(static_object_map_.find(object_names[i]) != static_object_map_.end()) {
      bsd = static_object_map_.find(object_names[i])->second->getBodyDecomposition(0);
    } else if(attached_object_map_.find(object_names[i]) != attached_object_map_.end()) {
      bsd = attached_object_map_.find(object_names[i])->second->getBodyDecomposition(0);
    } else {
      ROS_WARN_STREAM("Don't have object named " << object_names[i]);
      continue;
    }
    bodies::BoundingCylinder cyl;
    bsd->getBody()->computeBoundingCylinder(cyl);
    mark.scale.x = cyl.radius*2.0;
    mark.scale.y = cyl.radius*2.0;
    mark.scale.z = cyl.length;
    arr.markers.push_back(mark);
  }
  vis_marker_array_publisher_.publish(arr);
}
