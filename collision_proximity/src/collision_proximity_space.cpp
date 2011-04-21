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
#include <tf/tf.h>

using collision_proximity::CollisionProximitySpace;

// static double gen_rand(double min, double max)
// {
//   int rand_num = rand()%100+1;
//   double result = min + (double)((max-min)*rand_num)/101.0;
//   return result;
// }

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

CollisionProximitySpace::CollisionProximitySpace(const std::string& robot_description_name) :
  priv_handle_("~")
{
  collision_models_interface_ = new planning_environment::CollisionModelsInterface(robot_description_name);

  priv_handle_.param("size_x", size_x_, 3.0);
  priv_handle_.param("size_y", size_y_, 3.0);
  priv_handle_.param("size_z", size_z_, 4.0);
  priv_handle_.param("origin_x", origin_x_, -1.0);
  priv_handle_.param("origin_y", origin_y_, -1.5);
  priv_handle_.param("origin_z", origin_z_, -2.0);
  priv_handle_.param("resolution", resolution_, 0.02);
  priv_handle_.param("collision_tolerance", tolerance_, 0.00);
  priv_handle_.param("max_environment_distance", max_environment_distance_, 0.25);
  priv_handle_.param("max_self_distance", max_self_distance_, 0.10);
  priv_handle_.param("undefined_distance", undefined_distance_, 1.0);

  vis_marker_publisher_ = root_handle_.advertise<visualization_msgs::Marker>("collision_proximity_body_spheres", 128);
  vis_marker_array_publisher_ = root_handle_.advertise<visualization_msgs::MarkerArray>("collision_proximity_body_spheres_array", 128);

  self_distance_field_ = new distance_field::PropagationDistanceField(size_x_, size_y_, size_z_, resolution_, origin_x_, origin_y_, origin_z_, max_self_distance_);
  environment_distance_field_ = new distance_field::PropagationDistanceField(size_x_, size_y_, size_z_, resolution_, origin_x_, origin_y_, origin_z_, max_environment_distance_);

  collision_models_interface_->addSetPlanningSceneCallback(boost::bind(&CollisionProximitySpace::setPlanningSceneCallback, this, _1));
  collision_models_interface_->addRevertPlanningSceneCallback(boost::bind(&CollisionProximitySpace::revertPlanningSceneCallback, this));

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
}

CollisionProximitySpace::~CollisionProximitySpace()
{
  delete collision_models_interface_;
  delete self_distance_field_;
  delete environment_distance_field_;
  for(std::map<std::string, BodyDecomposition*>::iterator it = body_decomposition_map_.begin();
      it != body_decomposition_map_.end();
      it++) {
    delete it->second;
  }
  deleteAllStaticObjectDecompositions();
  deleteAllAttachedObjectDecompositions();
}

void CollisionProximitySpace::deleteAllStaticObjectDecompositions()
{
  for(std::map<std::string, BodyDecompositionVector*>::iterator it = static_object_map_.begin();
      it != static_object_map_.end();
      it++) {
    delete it->second;
  }
  static_object_map_.clear();
}

void CollisionProximitySpace::deleteAllAttachedObjectDecompositions()
{
  for(std::map<std::string, BodyDecompositionVector*>::iterator it = attached_object_map_.begin();
      it != attached_object_map_.end();
      it++) {
    delete it->second;
  }
  attached_object_map_.clear();
}

void CollisionProximitySpace::loadDefaultCollisionOperations()
{
  std::map<std::string, bool> all_true_map;
  const planning_models::KinematicModel* kmodel = collision_models_interface_->getKinematicModel();

  for(unsigned int i = 0; i < kmodel->getLinkModels().size(); i++) {
    all_true_map[kmodel->getLinkModels()[i]->getName()] = true;
  }
  
  for(unsigned int i = 0; i < kmodel->getLinkModels().size(); i++) {
    intra_group_collision_links_[kmodel->getLinkModels()[i]->getName()] = all_true_map;
  }

  //creating lists for each group in planning groups
  const std::map<std::string, planning_models::KinematicModel::JointModelGroup*>& jmgm = 
    collision_models_interface_->getKinematicModel()->getJointModelGroupMap();
  for(std::map<std::string, planning_models::KinematicModel::JointModelGroup*>::const_iterator it = jmgm.begin();
      it != jmgm.end();
      it++) {
    enabled_self_collision_links_[it->first] = all_true_map;
    std::vector<std::string> link_names = it->second->getUpdatedLinkModelNames();
    for(unsigned int i = 0; i < link_names.size(); i++) {
      enabled_self_collision_links_[it->first][link_names[i]] = false;
    }
  }

  XmlRpc::XmlRpcValue coll_ops;
  if(!root_handle_.hasParam("/robot_description_planning/distance_collision_operations")) {
    ROS_WARN("No distance collision operations specified");
    return;
  }
  root_handle_.getParam("/robot_description_planning/distance_collision_operations", coll_ops);
  
  if(coll_ops.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_WARN("distance_collision_operations is not an array");
    return;
  } 
  
  if(coll_ops.size() == 0) {
    ROS_WARN("No collision operations in distance collision operations");
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
      
    if(jmgm.find(object1) == jmgm.end()) {
      if(object1 == object2) {
        //using for disabling self checks
        self_excludes_[object1] = true;
      } else {
        //must be intra_collision
        if(jmgm.find(object2) == jmgm.end()) {
          intra_group_collision_links_[object1][object2] = (operation == "enable");
          intra_group_collision_links_[object2][object1] = (operation == "enable");
        } else {
          std::vector<std::string> group_links = jmgm.find(object2)->second->getUpdatedLinkModelNames();
          for(unsigned int j = 0; j < group_links.size(); j++) {
            intra_group_collision_links_[object1][group_links[j]] = (operation == "enable");
            intra_group_collision_links_[group_links[j]][object1] = (operation == "enable");
          }
        }
      }
    } else if(jmgm.find(object2) == jmgm.end()) {
      std::vector<std::string> group_links = jmgm.find(object1)->second->getUpdatedLinkModelNames();
      for(unsigned int j = 0; j < group_links.size(); j++) {
        intra_group_collision_links_[object2][group_links[j]] = (operation == "enable");
        intra_group_collision_links_[group_links[j]][object2] = (operation == "enable");
      }
    } else {
      std::vector<std::string> group_links_1 = jmgm.find(object1)->second->getUpdatedLinkModelNames();
      std::vector<std::string> group_links_2 = jmgm.find(object2)->second->getUpdatedLinkModelNames();
      for(unsigned int j = 0; j < group_links_1.size(); j++) {
        for(unsigned int k = 0; k < group_links_2.size(); k++) {
          intra_group_collision_links_[group_links_1[j]][group_links_2[k]] = (operation == "enable");
          intra_group_collision_links_[group_links_2[k]][group_links_1[j]] = (operation == "enable");
        }
      }
    }
  }
}

void CollisionProximitySpace::loadRobotBodyDecompositions()
{
  const planning_models::KinematicModel* kmodel = collision_models_interface_->getKinematicModel();
  
  for(unsigned int i = 0; i < kmodel->getLinkModels().size(); i++) {
    if(kmodel->getLinkModels()[i]->getLinkShape() != NULL) {
      body_decomposition_map_[kmodel->getLinkModels()[i]->getName()] = new BodyDecomposition(kmodel->getLinkModels()[i]->getName(),
                                                                                             kmodel->getLinkModels()[i]->getLinkShape(),
                                                                                             resolution_/2.0);
    }
  }
}

bool CollisionProximitySpace::setPlanningScene(const planning_environment_msgs::PlanningScene& scene) {
  return(collision_models_interface_->setPlanningSceneWithCallbacks(scene));
}

void CollisionProximitySpace::setPlanningSceneCallback(const planning_environment_msgs::PlanningScene& scene) 
{
  ros::WallTime n1 = ros::WallTime::now();
  deleteAllStaticObjectDecompositions();
  deleteAllAttachedObjectDecompositions();

  syncObjectsWithCollisionSpace(*collision_models_interface_->getPlanningSceneState());
  
  prepareEnvironmentDistanceField(*collision_models_interface_->getPlanningSceneState());
  ros::WallTime n2 = ros::WallTime::now();
  ROS_DEBUG_STREAM("Setting environment took " << (n2-n1).toSec());
  visualizeDistanceField(environment_distance_field_);
}

void CollisionProximitySpace::setupForGroupQueries(const std::string& group_name,
                                                   const motion_planning_msgs::RobotState& rob_state)  
{
  ros::WallTime n1 = ros::WallTime::now();
  //setting up current info
  current_group_name_ = group_name;
  getGroupLinkAndAttachedBodyNames(current_group_name_, 
                                   *collision_models_interface_->getPlanningSceneState(),
                                   current_link_names_, 
                                   current_link_indices_,
                                   current_attached_body_names_,
                                   current_attached_body_indices_);
  setupGradientStructures(current_link_names_,
                          current_attached_body_names_,
                          current_gradients_);

  current_link_body_decompositions_.clear();
  current_attached_body_decompositions_.clear();
  current_self_excludes_.clear();
  for(unsigned int i = 0; i < current_link_names_.size(); i++) {
    current_link_body_decompositions_.push_back(body_decomposition_map_[current_link_names_[i]]);
    if(self_excludes_.find(current_link_names_[i]) != self_excludes_.end()) {
      current_self_excludes_.push_back(false);
    } else {
      current_self_excludes_.push_back(true);
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
  setBodyPosesGivenKinematicState(*collision_models_interface_->getPlanningSceneState());
  setDistanceFieldForGroupQueries(current_group_name_, *collision_models_interface_->getPlanningSceneState());
  ros::WallTime n2 = ros::WallTime::now();
  ROS_DEBUG_STREAM("Setting self took " << (n2-n1).toSec());
  //visualizeDistanceField(self_distance_field_);
}

void CollisionProximitySpace::revertPlanningSceneCallback() {

  collision_models_interface_->bodiesLock();
  current_group_name_ = "";

  deleteAllStaticObjectDecompositions();
  deleteAllAttachedObjectDecompositions();
  collision_models_interface_->bodiesUnlock();
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
        btTransform test = inv*att_state->getGlobalCollisionBodyTransforms()[k];
        current_attached_body_decompositions_[i]->updateSpheresPose(k, inv*att_state->getGlobalCollisionBodyTransforms()[k]);
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
        ROS_WARN_STREAM("For attached object bodies number " << attached_object_map_[id]->getSize() 
                        << " not equal to state number " << att_state->getGlobalCollisionBodyTransforms().size() << ". Not updating");
        continue;
      }
      for(unsigned int k = 0; k < att_state->getGlobalCollisionBodyTransforms().size(); k++) {
        btTransform test = inv*att_state->getGlobalCollisionBodyTransforms()[k];
        attached_object_map_[id]->updateBodyPose(k, inv*att_state->getGlobalCollisionBodyTransforms()[k]);
      }
    }
  }
}

btTransform CollisionProximitySpace::getInverseWorldTransform(const planning_models::KinematicState& state) const {
  const planning_models::KinematicState::JointState* world_state = state.getJointStateVector()[0];//(monitor_->getKinematicModel()->getRoot()->getName());
  if(world_state == NULL) {
    ROS_WARN_STREAM("World state " << collision_models_interface_->getKinematicModel()->getRoot()->getName() << " not found");
  }
  const btTransform& world_trans = world_state->getVariableTransform();
  btTransform ret(world_trans);
  return ret.inverse();
}

void CollisionProximitySpace::syncObjectsWithCollisionSpace(const planning_models::KinematicState& state)
{
  btTransform inv = getInverseWorldTransform(state);
  ROS_INFO_STREAM("Inv x is " << inv.getOrigin().x());
  const collision_space::EnvironmentObjects *eo = collision_models_interface_->getCollisionSpace()->getObjects();
  std::vector<std::string> ns = eo->getNamespaces();
  for(unsigned int i = 0; i < ns.size(); i++) {
    if(ns[i] == COLLISION_MAP_NAME) continue;
    const collision_space::EnvironmentObjects::NamespaceObjects &no = eo->getObjects(ns[i]);
    BodyDecompositionVector* bdv = new BodyDecompositionVector();
    for(unsigned int j = 0; j < no.shape.size(); j++) {
      BodyDecomposition* bd = new BodyDecomposition(ns[i]+"_"+makeStringFromUnsignedInt(j), no.shape[j], resolution_);
      bd->updatePose(inv*no.shape_pose[j]);
      ROS_INFO_STREAM("Updated pose is " << bd->getBody()->getPose().getOrigin().x());
      bdv->addToVector(bd); 
    }
    static_object_map_[ns[i]] = bdv;
  }
  
  const std::vector<planning_models::KinematicState::LinkState*> link_states = state.getLinkStateVector();
  for(unsigned int i = 0; i < link_states.size(); i++) {
    const planning_models::KinematicState::LinkState* ls = link_states[i];
    for(unsigned int j = 0; j < ls->getAttachedBodyStateVector().size(); j++) {
      BodyDecompositionVector* bdv = new BodyDecompositionVector();
      const planning_models::KinematicState::AttachedBodyState* abs = ls->getAttachedBodyStateVector()[j];
      std::string id = makeAttachedObjectId(ls->getName(),abs->getName());
      for(unsigned int k = 0; k < abs->getAttachedBodyModel()->getShapes().size(); k++) {
        BodyDecomposition* bd = new BodyDecomposition(id+makeStringFromUnsignedInt(j), abs->getAttachedBodyModel()->getShapes()[k], resolution_);
        bd->updatePose(inv*abs->getGlobalCollisionBodyTransforms()[k]);
        bdv->addToVector(bd);
      }
      attached_object_map_[id] = bdv;
      for(unsigned int k = 0; k < abs->getAttachedBodyModel()->getTouchLinks().size(); k++) {
        attached_object_collision_links_[id][abs->getAttachedBodyModel()->getTouchLinks()[k]] = false;
      }
    }
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
  prepareSelfDistanceField(df_links, state);
}

void CollisionProximitySpace::prepareEnvironmentDistanceField(const planning_models::KinematicState& state)
{
  environment_distance_field_->reset();
  btTransform inv = getInverseWorldTransform(state);
  std::vector<btVector3> all_points;
  for(std::map<std::string, BodyDecompositionVector*>::iterator it = static_object_map_.begin();
      it != static_object_map_.end();
      it++) {
    for(unsigned int i = 0; i < it->second->getSize(); i++) {
      std::vector<btVector3> obj_points = it->second->getBodyDecomposition(i)->getCollisionPoints();
      for(unsigned int j = 0; j < obj_points.size(); j++) {
        //body pose already accounts for inverse transform
        obj_points[j] = it->second->getBodyDecomposition(i)->getBody()->getPose()*obj_points[j];
      }
      all_points.insert(all_points.end(),obj_points.begin(), obj_points.end());
    }
  }
  for(unsigned int i = 0; i < collision_models_interface_->getCollisionMapPoses().size(); i++) {
    all_points.push_back(inv*collision_models_interface_->getCollisionMapPoses()[i].getOrigin());
  }
  environment_distance_field_->addPointsToField(all_points);
  //ROS_INFO_STREAM("Adding points took " << (n2-n1).toSec());
}

void CollisionProximitySpace::prepareSelfDistanceField(const std::vector<std::string>& link_names, 
                                                       const planning_models::KinematicState& state)
{
  self_distance_field_->reset();
  btTransform inv = getInverseWorldTransform(state);
  std::vector<btVector3> all_points;
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
  self_distance_field_->addPointsToField(all_points);
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
                                                               const planning_models::KinematicState& state,
                                                               std::vector<std::string>& link_names,
                                                               std::vector<unsigned int>& link_indices,
                                                               std::vector<std::string>& attached_body_names,
                                                               std::vector<unsigned int>& attached_body_link_indices) const
{
  ros::WallTime n1 = ros::WallTime::now();
  link_names = collision_models_interface_->getKinematicModel()->getModelGroup(group_name)->getUpdatedLinkModelNames();
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
    if(collision_models_interface_->getLinkAttachedObjects().find(link_names[i]) != collision_models_interface_->getLinkAttachedObjects().end()) {
      unsigned int j = 0;
      for(std::map<std::string, bodies::BodyVector*>::const_iterator it = collision_models_interface_->getLinkAttachedObjects().find(link_names[i])->second.begin();
          it != collision_models_interface_->getLinkAttachedObjects().find(link_names[i])->second.end();
          it++, j++) {
        std::string id = makeAttachedObjectId(link_names[i],it->first);
        attached_body_names.push_back(id);
        attached_body_link_indices.push_back(link_indices[i]);
      }
    }
  }
  return true;
}

bool CollisionProximitySpace::setupGradientStructures(const std::vector<std::string>& link_names,
                                                      const std::vector<std::string>& attached_body_names, 
                                                      std::vector<GradientInfo>& gradients) const
{
  if(link_names.size() == 0) {
    ROS_WARN_STREAM("No link names in setup gradient structure");
    return false;
  }
  gradients.clear();
  unsigned int att_count = attached_body_names.size();

  gradients.resize(link_names.size()+att_count);

  for(unsigned int i = 0; i < link_names.size(); i++) {
    const std::vector<CollisionSphere>& lcs1 = body_decomposition_map_.find(link_names[i])->second->getCollisionSpheres();
    gradients[i].sphere_locations.resize(lcs1.size());
    for(unsigned int j = 0; j < lcs1.size(); j++) {
      gradients[i].sphere_locations[j] = lcs1[j].center_;
    }
    gradients[i].distances.resize(lcs1.size(), DBL_MAX);
    gradients[i].gradients.resize(lcs1.size());
  }
  unsigned int att_index = link_names.size();
  for(unsigned int i = 0; i < att_count; i++) {
    if(attached_object_map_.find(attached_body_names[i]) == attached_object_map_.end()) {
      ROS_WARN_STREAM("No attached object " << attached_body_names[i]);
      return false;
    }
    const std::vector<CollisionSphere>& att_vec = attached_object_map_.find(attached_body_names[i])->second->getCollisionSpheres();
    gradients[att_index].sphere_locations.resize(att_vec.size());
    for(unsigned int j = 0; j < att_vec.size(); j++) {
      gradients[att_index].sphere_locations[j] = att_vec[j].center_;
    }
    gradients[att_index].distances.resize(att_vec.size(), DBL_MAX);
    gradients[att_index].gradients.resize(att_vec.size());
    att_index++;
  }
  return true;
}

bool CollisionProximitySpace::isStateInCollision() const
{
  if(isEnvironmentCollision()) return true;
  if(isSelfCollision()) return true;
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
  collisions.resize(current_link_names_.size()+current_attached_body_names_.size());
  std::vector<bool> env_collisions, intra_collisions, self_collisions;
  env_collisions.resize(collisions.size(), false);
  intra_collisions = self_collisions = env_collisions;
  bool env_collision = getEnvironmentCollisions(env_collisions, false);
  bool intra_group_collision = getIntraGroupCollisions(intra_collisions, false);
  bool self_collision = getSelfCollisions(intra_collisions, false);
  for(unsigned int i = 0; i < current_link_names_.size()+current_attached_body_names_.size(); i++) {
    collisions[i].environment = env_collisions[i];
    collisions[i].self = self_collisions[i];
    collisions[i].intra = intra_collisions[i];
  }
  if(env_collision || intra_group_collision || self_collision) {
    in_collision = true;
  } else {
    in_collision = false;
  }
  return in_collision;
}

bool CollisionProximitySpace::getStateGradients(std::vector<std::string>& link_names,
                                                std::vector<std::string>& attached_body_names,
                                                std::vector<GradientInfo>& gradients,
                                                bool subtract_radii) const
{
  link_names = current_link_names_;
  attached_body_names = current_attached_body_names_;
  gradients = current_gradients_;

  std::vector<GradientInfo> intra_gradients = current_gradients_;
  std::vector<GradientInfo> self_gradients = current_gradients_;
  std::vector<GradientInfo> env_gradients = current_gradients_;

  bool env_coll = getEnvironmentProximityGradients(env_gradients, subtract_radii);
  bool self_coll = getSelfProximityGradients(self_gradients, subtract_radii);
  bool intra_coll = getIntraGroupProximityGradients(intra_gradients, subtract_radii);

  for(unsigned int i = 0; i < gradients.size(); i++) {
    if(i < current_link_names_.size()) {
      ROS_DEBUG_STREAM("Link " << link_names[i] 
                       << " env " << env_gradients[i].closest_distance
                       << " self " << self_gradients[i].closest_distance
                       << " intra " << intra_gradients[i].closest_distance);
    }
    {
      bool env_at_max = env_gradients[i].closest_distance >= max_environment_distance_;
      bool self_at_max = self_gradients[i].closest_distance >= max_self_distance_;
      if(env_at_max) {
        if(self_at_max || intra_gradients[i].closest_distance < self_gradients[i].closest_distance) {
          if(intra_gradients[i].closest_distance == DBL_MAX) {
            gradients[i].closest_distance = undefined_distance_;
          } else {
            gradients[i].closest_distance = intra_gradients[i].closest_distance;          
          }
          ROS_DEBUG_STREAM("Intra gradient is closest");
        } else {
          ROS_DEBUG_STREAM("Self gradient is closest");
          gradients[i].closest_distance = self_gradients[i].closest_distance;          
        }
      } else if(self_at_max) {
        //don't need to check env_at_max, as the previous condition should take care of it
        if(intra_gradients[i].closest_distance < env_gradients[i].closest_distance) {
          gradients[i].closest_distance = intra_gradients[i].closest_distance;
          ROS_DEBUG_STREAM("Intra gradient is closest");
        } else {
          gradients[i].closest_distance = env_gradients[i].closest_distance;
          ROS_DEBUG_STREAM("Env gradient is closest");
        }
      } else if(self_gradients[i].closest_distance < env_gradients[i].closest_distance) {
        gradients[i].closest_distance = self_gradients[i].closest_distance;
          ROS_DEBUG_STREAM("Self gradient is closest");
      } else {
        gradients[i].closest_distance = env_gradients[i].closest_distance;
        ROS_DEBUG_STREAM("Env gradient is closest");
      }
    }

    for(unsigned int j = 0; j < gradients[i].distances.size(); j++) {
      bool env_at_max = env_gradients[i].distances[j] >= max_environment_distance_;
      bool self_at_max = self_gradients[i].distances[j] >= max_self_distance_;
      if(env_at_max) {
        if(self_at_max || intra_gradients[i].distances[j] < self_gradients[i].distances[j]) {
          if(intra_gradients[i].distances[j] == DBL_MAX) {
            gradients[i].distances[j] = undefined_distance_;
          } else {
            gradients[i].distances[j] = intra_gradients[i].distances[j];
          }
          gradients[i].gradients[j] = intra_gradients[i].gradients[j]; 
       } else {
          gradients[i].distances[j] = self_gradients[i].distances[j];
          gradients[i].gradients[j] = self_gradients[i].gradients[j];
        }
      } else if(self_at_max) {
        if(intra_gradients[i].distances[j] < env_gradients[i].distances[j]) {
          gradients[i].distances[j] = intra_gradients[i].distances[j];
          gradients[i].gradients[j] = intra_gradients[i].gradients[j];
        } else {
          gradients[i].distances[j] = env_gradients[i].distances[j];
          gradients[i].gradients[j] = env_gradients[i].gradients[j];
        }
      } else if(self_gradients[i].distances[j] < env_gradients[i].distances[j]) {
        gradients[i].distances[j] = self_gradients[i].distances[j];
        gradients[i].gradients[j] = self_gradients[i].gradients[j]; 
      } else {
        gradients[i].distances[j] = env_gradients[i].distances[j];
        gradients[i].gradients[j] = env_gradients[i].gradients[j];
      }
    }
  }
  return (env_coll || intra_coll || self_coll);
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

bool CollisionProximitySpace::getIntraGroupProximityGradients(std::vector<GradientInfo>& gradients,
                                                              bool subtract_radii) const {
  bool in_collision = false;
  unsigned int count = 0;
  unsigned int num_links = current_link_names_.size();
  unsigned int num_attached = current_attached_body_names_.size();
  unsigned int tot = num_links+num_attached;
  std::vector<std::string> all_names = current_link_names_;
  all_names.insert(all_names.end(), current_attached_body_names_.begin(), current_attached_body_names_.end());
  for(unsigned int i = 0; i < tot; i++) {
    for(unsigned int j = 0; j < tot; j++) {
      if(i == j) continue;
      if(!current_intra_group_collision_links_[i][j]) {
        continue;
      }
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
          if(dist < gradients[i].distances[k]) {
            gradients[i].distances[k] = dist;
            gradients[i].gradients[k] = (*lcs1)[k].center_-(*lcs2)[l].center_;
          }
          if(dist < gradients[i].closest_distance) {
            gradients[i].closest_distance = dist;
          }
          if(dist < gradients[j].distances[l]) {
            gradients[j].distances[l] = dist;
            gradients[j].gradients[l] = (*lcs2)[l].center_-(*lcs1)[k].center_;
          }
          if(dist < gradients[j].closest_distance) {
            gradients[j].closest_distance = dist;
          }
        }
      }
    }
  }
  return in_collision;
}

bool CollisionProximitySpace::isSelfCollision() const
{
  std::vector<bool> collisions;
  return(getSelfCollisions(collisions, true));
}

bool CollisionProximitySpace::getSelfCollisions(std::vector<bool>& collisions,
                                                bool stop_at_first_collision) const
{
  bool in_collision = false;
  for(unsigned int i = 0; i < current_link_names_.size(); i++) {
    const std::vector<CollisionSphere>& body_spheres = current_link_body_decompositions_[i]->getCollisionSpheres();
    bool coll = getCollisionSphereCollision(self_distance_field_, body_spheres, tolerance_);
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
    bool coll = getCollisionSphereCollision(self_distance_field_, body_spheres, tolerance_);
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

bool CollisionProximitySpace::getSelfProximityGradients(std::vector<GradientInfo>& gradients,
                                                        bool subtract_radii) const {
  unsigned int tot = current_link_names_.size()+current_attached_body_names_.size();
  if(gradients.size() != tot) {
    ROS_WARN_STREAM("Wrong sized link closest " << tot << " " << gradients.size());
    return false;
  }
  bool in_collision = false;
  for(unsigned int i = 0; i < current_link_names_.size(); i++) {
    if(!current_self_excludes_[i]) continue;
    const std::vector<CollisionSphere>& body_spheres = current_link_body_decompositions_[i]->getCollisionSpheres();
    if(gradients[i].distances.size() != body_spheres.size()) {
      ROS_INFO_STREAM("Wrong size for closest distances for link " << current_link_names_[i]);
    }
    bool coll = getCollisionSphereGradients(self_distance_field_, body_spheres, gradients[i], tolerance_, subtract_radii, max_self_distance_, false);
    if(coll) {
      in_collision = true;
    }
  }
  for(unsigned int i = 0; i < current_attached_body_names_.size(); i++) {
    const std::vector<CollisionSphere>& body_spheres = current_attached_body_decompositions_[i]->getCollisionSpheres();
    bool coll = getCollisionSphereGradients(self_distance_field_, body_spheres, gradients[i+current_link_names_.size()],
                                            tolerance_, subtract_radii, max_self_distance_, false);
    if(coll) {
      in_collision = true;
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
    const std::vector<CollisionSphere>& body_spheres = current_link_body_decompositions_[i]->getCollisionSpheres();
    bool coll = getCollisionSphereCollision(environment_distance_field_, body_spheres, tolerance_);
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
    bool coll = getCollisionSphereCollision(environment_distance_field_, body_spheres, tolerance_);
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

bool CollisionProximitySpace::getEnvironmentProximityGradients(std::vector<GradientInfo>& gradients,
                                                               bool subtract_radii) const {
  unsigned int tot = current_link_names_.size()+current_attached_body_names_.size();
  if(gradients.size() != tot) {
    ROS_WARN_STREAM("Wrong sized link closest " << tot << " " << gradients.size());
    return false;
  }
  bool in_collision = false;
  for(unsigned int i = 0; i < current_link_names_.size(); i++) {
    const std::vector<CollisionSphere>& body_spheres = current_link_body_decompositions_[i]->getCollisionSpheres();
    if(gradients[i].distances.size() != body_spheres.size()) {
      ROS_INFO_STREAM("Wrong size for closest distances for link " << current_link_names_[i]);
    }
    bool coll = getCollisionSphereGradients(environment_distance_field_, body_spheres, gradients[i], tolerance_, subtract_radii, max_environment_distance_, false);
    if(coll) {
      in_collision = true;
    }
  }
  for(unsigned int i = 0; i < current_attached_body_names_.size(); i++) {
    const std::vector<CollisionSphere>& body_spheres = current_attached_body_decompositions_[i]->getCollisionSpheres();
    bool coll = getCollisionSphereGradients(environment_distance_field_, body_spheres, gradients[i+current_link_names_.size()], tolerance_, subtract_radii, max_environment_distance_, false);
    if(coll) {
      in_collision = true;
    }
  }
  return in_collision;
}

void CollisionProximitySpace::getProximityGradientMarkers(const std::vector<std::string>& link_names, 
                                                          const std::vector<std::string>& attached_body_names, 
                                                          const std::vector<GradientInfo>& gradients,
                                                          const std::string& ns, 
                                                          visualization_msgs::MarkerArray& arr) const 
{
  for(unsigned int i = 0; i < gradients.size(); i++) {
    
    const std::vector<CollisionSphere>* lcs;
    std::string name;
    if(i < link_names.size()) {
      name = link_names[i];
      lcs = &(body_decomposition_map_.find(name)->second->getCollisionSpheres());
    } else {
      name = attached_body_names[i-link_names.size()];
      lcs = &(attached_object_map_.find(name)->second->getCollisionSpheres());
    }
    for(unsigned int j = 0; j < gradients[i].distances.size(); j++) {
      visualization_msgs::Marker arrow_mark;
      arrow_mark.header.frame_id = collision_models_interface_->getRobotFrameId();
      arrow_mark.header.stamp = ros::Time::now();
      if(ns.empty()) {
        arrow_mark.ns = "self_coll_gradients";
      } else {
        arrow_mark.ns = ns;
      }
      arrow_mark.id = i*1000+j;
      double xscale = 0.0;
      double yscale = 0.0;
      double zscale = 0.0;
      if(gradients[i].distances[j] > 0.0) {
        if(gradients[i].gradients[j].length() > 0.0) {
          xscale = gradients[i].gradients[j].x()/gradients[i].gradients[j].length();
          yscale = gradients[i].gradients[j].y()/gradients[i].gradients[j].length();
          zscale = gradients[i].gradients[j].z()/gradients[i].gradients[j].length();
        } else {
          ROS_DEBUG_STREAM("Negative length for " << name << " " << arrow_mark.id << " " << gradients[i].gradients[j].length());
        }
      } else {
        ROS_DEBUG_STREAM("Negative dist for " << name << " " << arrow_mark.id);
      }
      arrow_mark.points.resize(2);
      arrow_mark.points[1].x = (*lcs)[j].center_.x();
      arrow_mark.points[1].y = (*lcs)[j].center_.y();
      arrow_mark.points[1].z = (*lcs)[j].center_.z();
      arrow_mark.points[0] = arrow_mark.points[1];
      arrow_mark.points[0].x -= xscale*gradients[i].distances[j];
      arrow_mark.points[0].y -= yscale*gradients[i].distances[j];
      arrow_mark.points[0].z -= zscale*gradients[i].distances[j];
      arrow_mark.scale.x = 0.01;
      arrow_mark.scale.y = 0.03;
      arrow_mark.color.r = 1.0;
      arrow_mark.color.g = 0.2;
      arrow_mark.color.b = .5;
      arrow_mark.color.a = 1.0;
      arr.markers.push_back(arrow_mark);
    }
  }
}

////////////
// Visualization functions
///////////
  
void CollisionProximitySpace::visualizeDistanceField(distance_field::PropagationDistanceField* distance_field) const
{
  btTransform ident;
  ident.setIdentity();
  distance_field->visualize(0.0, 0.0, collision_models_interface_->getRobotFrameId(), ident, ros::Time::now());
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
      boost::shared_ptr<const urdf::Link> urdf_link = collision_models_interface_->getParsedDescription()->getLink(name);
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
          mark.header.frame_id = collision_models_interface_->getRobotFrameId();
          mark.header.stamp = ros::Time::now();
          mark.ns = "proximity_collisions";
          mark.id = i;
          mark.type = mark.MESH_RESOURCE;
          mark.scale.x = 1.05;
          mark.scale.y = 1.05;
          mark.scale.z = 1.05;
          mark.color.a = .5;
          if(collisions[i].intra) {
            ROS_DEBUG_STREAM(name << " in intra_group");
            mark.color.r = 1.0;
          } else if(collisions[i].environment) {
            ROS_DEBUG_STREAM(name << " in environment");
            mark.color.r = 1.0;
            mark.color.g = 1.0;
          } else if(collisions[i].self) {
            ROS_DEBUG_STREAM(name << " in self");
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
      mark.header.frame_id = collision_models_interface_->getRobotFrameId();
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
      if(collisions[i+link_names.size()].intra) {
        ROS_DEBUG_STREAM(name << " in intra_group");
        mark.color.r = 1.0;
      } else if(collisions[i+link_names.size()].environment) {
        ROS_DEBUG_STREAM(name << " in environment");
        mark.color.r = 1.0;
        mark.color.g = 1.0;
      } else if(collisions[i+link_names.size()].self) {
        ROS_DEBUG_STREAM(name << " in self");
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
  cube_list.header.frame_id = collision_models_interface_->getRobotFrameId();
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
      sphere.header.frame_id = collision_models_interface_->getRobotFrameId();
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

void CollisionProximitySpace::visualizeBoundingCylinders(const std::vector<std::string>& object_names) const
{
  visualization_msgs::MarkerArray arr;
  for(unsigned int i = 0; i < object_names.size(); i++) {
    visualization_msgs::Marker mark;
    mark.header.frame_id = collision_models_interface_->getRobotFrameId();
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
