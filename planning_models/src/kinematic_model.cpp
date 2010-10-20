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

/** \author Ioan Sucan */

#include <planning_models/kinematic_model.h>
#include <geometric_shapes/shape_operations.h>
#include <resource_retriever/retriever.h>
#include <queue>
#include <ros/console.h>
#include <cmath>
#include <angles/angles.h>
#include <assimp/assimp.hpp>     
#include <assimp/aiScene.h>      
#include <assimp/aiPostProcess.h>


/* ------------------------ KinematicModel ------------------------ */
planning_models::KinematicModel::KinematicModel(const urdf::Model &model, 
                                                const std::map< std::string, std::vector<std::string> > &groups,
                                                const std::vector<MultiDofConfig>& multi_dof_configs)
{    
  model_name_ = model.getName();
  if (model.getRoot())
  {
    const urdf::Link *root = model.getRoot().get();
    root_ = buildRecursive(NULL, root, multi_dof_configs);
    buildGroups(groups);
  }
  else
  {
    root_ = NULL;
    ROS_WARN("No root link found");
  }
}

planning_models::KinematicModel::KinematicModel(const KinematicModel &source)
{
  copyFrom(source);
}

planning_models::KinematicModel::~KinematicModel(void)
{
  for (std::map<std::string, JointModelGroup*>::iterator it = joint_model_group_map_.begin() ; it != joint_model_group_map_.end() ; ++it)
  {
    delete it->second;
  }
  //this destroys the whole tree
  if(root_) {
    delete root_;
  }
}

void planning_models::KinematicModel::exclusiveLock(void) const
{
  lock_.lock();
}

void planning_models::KinematicModel::exclusiveUnlock(void) const
{
  lock_.unlock();
}

void planning_models::KinematicModel::sharedLock(void) const
{
  lock_.lock_shared();
}

void planning_models::KinematicModel::sharedUnlock(void) const
{
  lock_.unlock_shared();
}

const std::string& planning_models::KinematicModel::getName() const {
  return model_name_;
}

void planning_models::KinematicModel::copyFrom(const KinematicModel &source)
{
  model_name_ = source.model_name_;

  if (source.root_)
  {
    root_ = copyRecursive(NULL, source.root_->child_link_model_);
    

    const std::map<std::string, JointModelGroup*>& source_group_map = source.getJointModelGroupMap();
    std::map< std::string, std::vector<std::string> > groupContent;
    for(std::map<std::string, JointModelGroup*>::const_iterator it = source_group_map.begin();
        it != source_group_map.end();
        it++) {
      groupContent[it->second->getName()] = it->second->getJointModelNames();
    }
    buildGroups(groupContent);
  } else 
  {
    root_ = NULL;
  }
}

void planning_models::KinematicModel::buildGroups(const std::map< std::string, std::vector<std::string> > &groups)
{
  for (std::map< std::string, std::vector<std::string> >::const_iterator it = groups.begin() ; it != groups.end() ; ++it)
  {
    std::vector<const JointModel*> jointv;
    for (unsigned int i = 0 ; i < it->second.size() ; ++i)
    {
      std::map<std::string, JointModel*>::iterator p = joint_model_map_.find(it->second[i]);
      if (p == joint_model_map_.end())
      {
        ROS_ERROR("Unknown joint '%s'. Not adding to group '%s'", it->second[i].c_str(), it->first.c_str());
        jointv.clear();
        break;
      }
      else
        jointv.push_back(p->second);
    }
    
    if (jointv.empty())
      ROS_DEBUG("Skipping group '%s'", it->first.c_str());
    else
    {
      ROS_DEBUG("Adding group '%s'", it->first.c_str());
      joint_model_group_map_[it->first] = new JointModelGroup(it->first, jointv);
    }
  }
}

planning_models::KinematicModel::JointModel* planning_models::KinematicModel::buildRecursive(LinkModel *parent, const urdf::Link *link, 
                                                                                             const std::vector<MultiDofConfig>& multi_dof_configs)
{  
  JointModel *joint = constructJointModel(link->parent_joint.get(), link, multi_dof_configs);
  joint_model_map_[joint->name_] = joint;
  for(JointModel::js_type::const_iterator it = joint->getJointStateEquivalents().begin();
      it != joint->getJointStateEquivalents().end();
      it++) {
    joint_model_map_[it->right] = joint;
  }
  joint_model_vector_.push_back(joint);
  joint->parent_link_model_ = parent;
  joint->child_link_model_ = constructLinkModel(link);
  if (parent == NULL)
    joint->child_link_model_->joint_origin_transform_.setIdentity();
  link_model_map_[joint->child_link_model_->name_] = joint->child_link_model_;
  link_model_vector_.push_back(joint->child_link_model_);
  joint->child_link_model_->parent_joint_model_ = joint;
  
  for (unsigned int i = 0 ; i < link->child_links.size() ; ++i)
    joint->child_link_model_->child_joint_models_.push_back(buildRecursive(joint->child_link_model_, link->child_links[i].get(), multi_dof_configs));
  
  return joint;
}

planning_models::KinematicModel::JointModel* planning_models::KinematicModel::constructJointModel(const urdf::Joint *urdf_joint,
                                                                                                  const urdf::Link *child_link,
                                                                                                  const std::vector<MultiDofConfig>& multi_dof_configs)
{
  const MultiDofConfig* joint_config = NULL;
  bool found = false;
  for(std::vector<MultiDofConfig>::const_iterator it = multi_dof_configs.begin();
      it != multi_dof_configs.end();
      it++) {
    if(it->child_frame_id == child_link->name) {
      if(found == true) {
        ROS_WARN_STREAM("KinematicModel - two multi-dof joints with same " << it->child_frame_id << " child frame");
      } else {
        found = true;
        joint_config = &(*it);
      }
    }
  }

  planning_models::KinematicModel::JointModel* result = NULL;

  //must be the root link transform
  if(urdf_joint == NULL) {
    if(!found) {
      ROS_ERROR("Root transform has no multi dof joint config");
      return NULL;
    }
    if(joint_config->type == "Planar") {
      result = new PlanarJointModel(joint_config->name, joint_config);
    } else if(joint_config->type == "Floating") {
      result = new FloatingJointModel(joint_config->name, joint_config);
    } else {
      ROS_ERROR_STREAM("Unrecognized type of multi dof joint " << joint_config->type);
      return NULL;
    }
  } else {
    switch (urdf_joint->type)
    {
    case urdf::Joint::REVOLUTE:
      {
        RevoluteJointModel *j = new RevoluteJointModel(urdf_joint->name, joint_config);
        if(urdf_joint->safety)
        {
          j->setVariableBounds(j->name_, urdf_joint->safety->soft_lower_limit, urdf_joint->safety->soft_upper_limit);
        }
        else
        {
          j->setVariableBounds(j->name_, urdf_joint->limits->lower, urdf_joint->limits->upper);
        }
        j->continuous_ = false;
        j->axis_.setValue(urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z);
        result = j;
      }
      break;
    case urdf::Joint::CONTINUOUS:
      {
        RevoluteJointModel *j = new RevoluteJointModel(urdf_joint->name, joint_config);
        j->continuous_ = true;
        j->setVariableBounds(j->name_, -M_PI, M_PI);
        j->axis_.setValue(urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z);
        result = j;
      }
      break;
    case urdf::Joint::PRISMATIC:
      {
        PrismaticJointModel *j = new PrismaticJointModel(urdf_joint->name, joint_config);
        if(urdf_joint->safety)
        {
          j->setVariableBounds(j->name_, urdf_joint->safety->soft_lower_limit, urdf_joint->safety->soft_upper_limit);
        }
        else
        {
          j->setVariableBounds(j->name_, urdf_joint->limits->upper, urdf_joint->limits->lower);
        }
        j->axis_.setValue(urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z);
        result = j;
      }
      break;
    case urdf::Joint::FLOATING:
      result = new FloatingJointModel(urdf_joint->name, joint_config);
      break;
    case urdf::Joint::PLANAR:
      result = new PlanarJointModel(urdf_joint->name, joint_config);
      break;
    case urdf::Joint::FIXED:
      result = new FixedJointModel(urdf_joint->name, joint_config);
      break;
    default:
      ROS_ERROR("Unknown joint type: %d", (int)urdf_joint->type);
      break;
    }
  }
  return result;
}

namespace planning_models
{
static inline btTransform urdfPose2btTransform(const urdf::Pose &pose)
{
  return btTransform(btQuaternion(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w),
                     btVector3(pose.position.x, pose.position.y, pose.position.z));
}
}

planning_models::KinematicModel::LinkModel* planning_models::KinematicModel::constructLinkModel(const urdf::Link *urdf_link)
{
  ROS_ASSERT(urdf_link);
  
  LinkModel *result = new LinkModel(this);
  result->name_ = urdf_link->name;

  if(urdf_link->collision)
    result->collision_origin_transform_ = urdfPose2btTransform(urdf_link->collision->origin);
  else
    result->collision_origin_transform_.setIdentity();

  if (urdf_link->parent_joint.get())
    result->joint_origin_transform_ = urdfPose2btTransform(urdf_link->parent_joint->parent_to_joint_origin_transform);
  else
    result->joint_origin_transform_.setIdentity();
    
  if(urdf_link->collision) {
    result->shape_ = constructShape(urdf_link->collision->geometry.get());
  } else {
    shapes::Shape *tmp_shape = NULL;
    tmp_shape = new shapes::Sphere(0.0001);
    result->shape_ = tmp_shape;   
  } 
  return result;
}

shapes::Shape* planning_models::KinematicModel::constructShape(const urdf::Geometry *geom)
{
  ROS_ASSERT(geom);
 
  shapes::Shape *result = NULL;
  switch (geom->type)
  {
  case urdf::Geometry::SPHERE:
    result = new shapes::Sphere(dynamic_cast<const urdf::Sphere*>(geom)->radius);
    break;	
  case urdf::Geometry::BOX:
    {
      urdf::Vector3 dim = dynamic_cast<const urdf::Box*>(geom)->dim;
      result = new shapes::Box(dim.x, dim.y, dim.z);
    }
    break;
  case urdf::Geometry::CYLINDER:
    result = new shapes::Cylinder(dynamic_cast<const urdf::Cylinder*>(geom)->radius,
                                  dynamic_cast<const urdf::Cylinder*>(geom)->length);
    break;
  case urdf::Geometry::MESH:
    {
      const urdf::Mesh *mesh = dynamic_cast<const urdf::Mesh*>(geom);
      if (!mesh->filename.empty())
      {
        resource_retriever::Retriever retriever;
        resource_retriever::MemoryResource res;
        bool ok = true;
	
        try
        {
          res = retriever.get(mesh->filename);
        }
        catch (resource_retriever::Exception& e)
        {
          ROS_ERROR("%s", e.what());
          ok = false;
        }
	
        if (ok)
        {
          if (res.size == 0)
            ROS_WARN("Retrieved empty mesh for resource '%s'", mesh->filename.c_str());
          else
          {
            // Create an instance of the Importer class
            Assimp::Importer importer;
            
            // try to get a file extension
            std::string hint;
            std::size_t pos = mesh->filename.find_last_of(".");
            if (pos != std::string::npos)
            {
              hint = mesh->filename.substr(pos + 1);
              
              // temp hack until everything is stl not stlb
              if (hint.find("stl") != std::string::npos)
                hint = "stl";
            }
            
            // And have it read the given file with some postprocessing
            const aiScene* scene = importer.ReadFileFromMemory(reinterpret_cast<void*>(res.data.get()), res.size,
                                                               aiProcess_Triangulate            |
                                                               aiProcess_JoinIdenticalVertices  |
                                                               aiProcess_SortByPType, hint.c_str());
            btVector3 scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
            if (scene)
            {
              if (scene->HasMeshes())
              {
                if (scene->mNumMeshes > 1)
                  ROS_WARN("More than one mesh specified in resource. Using first one");
                result = shapes::createMeshFromAsset(scene->mMeshes[0], scale);
              }
              else
                ROS_ERROR("There is no mesh specified in the indicated resource");
            }
            else
            {
              std::string ext;
              importer.GetExtensionList(ext);
              ROS_ERROR("Failed to import scene containing mesh: %s. Supported extensions are: %s", importer.GetErrorString(), ext.c_str());
            }
            
            if (result == NULL)
              ROS_ERROR("Failed to load mesh '%s'", mesh->filename.c_str());
            else
              ROS_DEBUG("Loaded mesh '%s' consisting of %d triangles", mesh->filename.c_str(), static_cast<shapes::Mesh*>(result)->triangleCount);			
          }
        }
      }
      else
        ROS_WARN("Empty mesh filename");
    }
    
    break;
  default:
    ROS_ERROR("Unknown geometry type: %d", (int)geom->type);
    break;
  }
    
  return result;
}

const planning_models::KinematicModel::JointModel* planning_models::KinematicModel::getRoot(void) const
{
  return root_;
}

bool planning_models::KinematicModel::hasJointModel(const std::string &name) const
{
  return joint_model_map_.find(name) != joint_model_map_.end();
}

bool planning_models::KinematicModel::hasLinkModel(const std::string &name) const
{
  return link_model_map_.find(name) != link_model_map_.end();
}

bool planning_models::KinematicModel::hasModelGroup(const std::string &name) const
{
  return joint_model_group_map_.find(name) != joint_model_group_map_.end();
}

const planning_models::KinematicModel::JointModel* planning_models::KinematicModel::getJointModel(const std::string &name) const
{
  std::map<std::string, JointModel*>::const_iterator it = joint_model_map_.find(name);
  if (it == joint_model_map_.end())
  {
    ROS_ERROR("Joint '%s' not found", name.c_str());
    return NULL;
  }
  else
    return it->second;
}

const planning_models::KinematicModel::LinkModel* planning_models::KinematicModel::getLinkModel(const std::string &name) const
{
  std::map<std::string, LinkModel*>::const_iterator it = link_model_map_.find(name);
  if (it == link_model_map_.end())
  {
    ROS_ERROR("Link '%s' not found", name.c_str());
    return NULL;
  }
  else
    return it->second;
}

void planning_models::KinematicModel::getModelGroupNames(std::vector<std::string> &groups) const
{
  groups.clear();
  groups.reserve(joint_model_group_map_.size());
  for (std::map<std::string, JointModelGroup*>::const_iterator it = joint_model_group_map_.begin() ; it != joint_model_group_map_.end() ; ++it)
    groups.push_back(it->second->name_);
}

void planning_models::KinematicModel::getLinkModelNames(std::vector<std::string> &links) const
{
  links.clear();
  links.reserve(link_model_vector_.size());
  for(unsigned int i = 0; i < link_model_vector_.size(); i++) {
    links.push_back(link_model_vector_[i]->getName());
  }
}

void planning_models::KinematicModel::getJointModelNames(std::vector<std::string> &joints) const
{
  joints.clear();
  joints.reserve(joint_model_vector_.size());
  for (unsigned int i = 0 ; i < joint_model_vector_.size() ; ++i)
    joints.push_back(joint_model_vector_[i]->getName());
}

planning_models::KinematicModel::JointModel* planning_models::KinematicModel::copyRecursive(LinkModel *parent, const LinkModel *link)
{
  JointModel *joint = copyJointModel(link->parent_joint_model_);
  joint_model_map_[joint->name_] = joint;
  for(JointModel::js_type::iterator it = joint->joint_state_equivalents_.begin();
      it != joint->joint_state_equivalents_.end();
      it++) {
    joint_model_map_[it->right] = joint;
  }
  joint_model_vector_.push_back(joint);
  joint->parent_link_model_ = parent;
  joint->child_link_model_ = new LinkModel(link);
  link_model_map_[joint->child_link_model_->name_] = joint->child_link_model_;
  joint->child_link_model_->parent_joint_model_ = joint;
    
  for (unsigned int i = 0 ; i < link->child_joint_models_.size() ; ++i)
    joint->child_link_model_->child_joint_models_.push_back(copyRecursive(joint->child_link_model_, link->child_joint_models_[i]->child_link_model_));
    
  return joint;
}

planning_models::KinematicModel::JointModel* planning_models::KinematicModel::copyJointModel(const JointModel *joint)
{
  JointModel *newJoint = NULL;

  if (dynamic_cast<const FixedJointModel*>(joint))
  {
    newJoint = new FixedJointModel(dynamic_cast<const FixedJointModel*>(joint));
  }
  else if (dynamic_cast<const FloatingJointModel*>(joint))
  {
    newJoint = new FloatingJointModel(dynamic_cast<const FloatingJointModel*>(joint));
  }
  else if (dynamic_cast<const PlanarJointModel*>(joint))
  {
    newJoint = new PlanarJointModel(dynamic_cast<const PlanarJointModel*>(joint));
  }
  else if (dynamic_cast<const PrismaticJointModel*>(joint))
  {
    newJoint = new PrismaticJointModel(dynamic_cast<const PrismaticJointModel*>(joint));
  }
  else if (dynamic_cast<const RevoluteJointModel*>(joint))
  {
    newJoint = new RevoluteJointModel(dynamic_cast<const RevoluteJointModel*>(joint));
  }
  else
    ROS_FATAL("Unimplemented type of joint");
    
  return newJoint;
}

void planning_models::KinematicModel::getChildLinkModels(const KinematicModel::LinkModel *parent, 
                                                         std::vector<const KinematicModel::LinkModel*> &links) const
{
  std::queue<const KinematicModel::LinkModel*> q;
  q.push(parent);
  while (!q.empty())
  {
    const KinematicModel::LinkModel* t = q.front();
    q.pop();
    
    for (unsigned int i = 0 ; i < t->child_joint_models_.size() ; ++i) {
      if (t->child_joint_models_[i]->child_link_model_)
      {
        links.push_back(t->child_joint_models_[i]->child_link_model_);
        q.push(t->child_joint_models_[i]->child_link_model_);
      }
    }
  }
}

void planning_models::KinematicModel::clearAllAttachedBodyModels()
{
  exclusiveLock();
  for(unsigned int i =0; i < link_model_vector_.size(); i++) {
    link_model_vector_[i]->clearAttachedBodyModels();
  }
  exclusiveUnlock();
}

void planning_models::KinematicModel::clearLinkAttachedBodyModels(const std::string& link_name)
{
  exclusiveLock();
  if(link_model_map_.find(link_name) == link_model_map_.end()) return;
  link_model_map_[link_name]->clearAttachedBodyModels();
  exclusiveUnlock();
}

void planning_models::KinematicModel::replaceAttachedBodyModels(const std::string& link_name, 
                                                               std::vector<AttachedBodyModel*>& attached_body_vector)
{
  exclusiveLock();
  if(link_model_map_.find(link_name) == link_model_map_.end())
  {
    ROS_WARN_STREAM("Model has no link named " << link_name << ".  This is probably going to introduce a memory leak");
    return;
  }
  link_model_map_[link_name]->replaceAttachedBodyModels(attached_body_vector);
  exclusiveUnlock();
}

void planning_models::KinematicModel::clearLinkAttachedBodyModel(const std::string& link_name, 
                                                                 const std::string& att_name)
{
  exclusiveLock();
  if(link_model_map_.find(link_name) == link_model_map_.end()) return;
  link_model_map_[link_name]->clearLinkAttachedBodyModel(att_name);
  exclusiveUnlock();
}

void planning_models::KinematicModel::addAttachedBodyModel(const std::string& link_name, 
                                                           planning_models::KinematicModel::AttachedBodyModel* ab)
{
  exclusiveLock();
  if(link_model_map_.find(link_name) == link_model_map_.end()) {
    ROS_WARN_STREAM("Model has no link named " << link_name << " to attach body to.  This is probably going to introduce a memory leak");
    return;
  }
  link_model_map_[link_name]->addAttachedBodyModel(ab);
  exclusiveUnlock();
}


/* ------------------------ JointModel ------------------------ */

planning_models::KinematicModel::JointModel::JointModel(const std::string& n) :
  name_(n), parent_link_model_(NULL), child_link_model_(NULL)
{
}

void planning_models::KinematicModel::JointModel::initialize(const std::vector<std::string>& local_joint_names,
                                                             const MultiDofConfig* config)
{
  for(std::vector<std::string>::const_iterator it = local_joint_names.begin();
      it != local_joint_names.end();
      it++) {
    joint_state_equivalents_.insert(js_type::value_type(*it,*it));
  } 
  if(config != NULL) {
    for(std::map<std::string, std::string>::const_iterator it = config->name_equivalents.begin();
        it != config->name_equivalents.end();
        it++) {
      js_type::left_iterator lit = joint_state_equivalents_.left.find(it->first);
      if(lit != joint_state_equivalents_.left.end()) {
        joint_state_equivalents_.left.replace_data(lit, it->second);
      }
    }
    parent_frame_id_ = config->parent_frame_id;
    child_frame_id_ = config->child_frame_id;
  } 
  unsigned int i = 0;
  for(std::vector<std::string>::const_iterator it = local_joint_names.begin();
      it != local_joint_names.end();
      it++, i++) {
    computation_order_map_index_[i] = joint_state_equivalents_.left.at(*it);
  }

  for(js_type::iterator it = joint_state_equivalents_.begin();
      it != joint_state_equivalents_.end();
      it++) {
    setVariableBounds(it->right,-DBL_MAX,DBL_MAX);
  }
}

planning_models::KinematicModel::JointModel::JointModel(const JointModel* joint) {
  name_ = joint->name_;
  parent_frame_id_  = joint->parent_frame_id_;
  child_frame_id_  = joint->child_frame_id_;
  joint_state_equivalents_ = joint->joint_state_equivalents_;
  joint_state_bounds_ = joint->joint_state_bounds_;
}

planning_models::KinematicModel::JointModel::~JointModel(void)
{
  if (child_link_model_)
    delete child_link_model_;
}

std::string planning_models::KinematicModel::JointModel::getEquiv(const std::string name) const {
  js_type::left_const_iterator lit = joint_state_equivalents_.left.find(name);
  if(lit != joint_state_equivalents_.left.end()) {
    return lit->second;
  } else {
    return "";
  }
}

void planning_models::KinematicModel::JointModel::setVariableBounds(std::string variable, double low, double high) {
  if(joint_state_equivalents_.right.find(variable) == joint_state_equivalents_.right.end()) {
    ROS_WARN_STREAM("Can't find variable " << variable << " to set bounds");
    return;
  }
  joint_state_bounds_[joint_state_equivalents_.right.at(variable)] = std::pair<double,double>(low, high);
}

std::pair<double, double> planning_models::KinematicModel::JointModel::getVariableBounds(std::string variable) const{
  if(joint_state_equivalents_.right.find(variable) == joint_state_equivalents_.right.end()) {
    ROS_WARN_STREAM("Can't find variable " << variable << " to get bounds");
    return std::pair<double,double>(0.0,0.0);
  }
  std::string config_name = joint_state_equivalents_.right.find(variable)->second;
  if(joint_state_bounds_.find(config_name) == joint_state_bounds_.end()) {
    ROS_WARN_STREAM("No joint bounds for " << config_name);
    return std::pair<double,double>(0.0,0.0);
  }
  return joint_state_bounds_.find(config_name)->second;
}

planning_models::KinematicModel::PlanarJointModel::PlanarJointModel(const std::string& name, const MultiDofConfig* multi_dof_config) 
  : JointModel(name)
{
  if(multi_dof_config == NULL) {
    ROS_WARN("Planar joint needs a config");
    return;
  }
  std::vector<std::string> local_names;
  local_names.push_back("planar_x");
  local_names.push_back("planar_y");
  local_names.push_back("planar_th");
  initialize(local_names, multi_dof_config);
  setVariableBounds(getEquiv("planar_x"),-DBL_MAX,DBL_MAX);
  setVariableBounds(getEquiv("planar_y"),-DBL_MAX,DBL_MAX);
  setVariableBounds(getEquiv("planar_th"),-M_PI,M_PI);
}

btTransform planning_models::KinematicModel::PlanarJointModel::computeTransform(const std::vector<double>& joint_values) const 
{
  btTransform variable_transform;
  variable_transform.setIdentity();
  if(joint_values.size() != 3) {
    ROS_ERROR("Planar joint given too few values");
    return variable_transform;
  }
  variable_transform.setOrigin(btVector3(joint_values[0],
                                         joint_values[1],
                                         0.0));
  variable_transform.setRotation(btQuaternion(btVector3(0.0, 0.0, 1.0),
                                              joint_values[2]));
  return variable_transform;
}

std::vector<double> planning_models::KinematicModel::PlanarJointModel::computeJointStateValues(const btTransform& transform) const 
{
  std::vector<double> ret;
  ret.push_back(transform.getOrigin().x());
  ret.push_back(transform.getOrigin().y());
  ret.push_back(transform.getRotation().getAngle()*transform.getRotation().getAxis().z());
  return ret;
}

planning_models::KinematicModel::FloatingJointModel::FloatingJointModel(const std::string& name, 
                                                                        const MultiDofConfig* multi_dof_config) 
  : JointModel(name)
{
  if(multi_dof_config == NULL) {
    ROS_WARN("Planar joint needs a config");
    return;
  }
  std::vector<std::string> local_names;
  local_names.push_back("floating_trans_x");
  local_names.push_back("floating_trans_y");
  local_names.push_back("floating_trans_z");
  local_names.push_back("floating_rot_x");
  local_names.push_back("floating_rot_y");
  local_names.push_back("floating_rot_z");
  local_names.push_back("floating_rot_w");
  initialize(local_names, multi_dof_config);

  setVariableBounds(getEquiv("floating_trans_x"),-DBL_MAX,DBL_MAX);
  setVariableBounds(getEquiv("floating_trans_y"),-DBL_MAX,DBL_MAX);
  setVariableBounds(getEquiv("floating_trans_z"),-DBL_MAX,DBL_MAX);  
  setVariableBounds(getEquiv("floating_rot_x"),-1.0,1.0);
  setVariableBounds(getEquiv("floating_rot_y"),-1.0,1.0);
  setVariableBounds(getEquiv("floating_rot_z"),-1.0,1.0);
  setVariableBounds(getEquiv("floating_rot_w"),-1.0,1.0);
}

btTransform planning_models::KinematicModel::FloatingJointModel::computeTransform(const std::vector<double>& joint_values) const 
{
  btTransform variable_transform;
  variable_transform.setIdentity();
  if(joint_values.size() != 7) {
    ROS_ERROR("Floating joint given too few values");
    return variable_transform;
  }
  variable_transform.setOrigin(btVector3(joint_values[0], joint_values[1], joint_values[2]));
  variable_transform.setRotation(btQuaternion(joint_values[3], joint_values[4], joint_values[5], joint_values[6]));
  return variable_transform;
}

std::vector<double> planning_models::KinematicModel::FloatingJointModel::computeJointStateValues(const btTransform& transform) const 
{
  std::vector<double> ret;
  ret.push_back(transform.getOrigin().x());
  ret.push_back(transform.getOrigin().y());
  ret.push_back(transform.getOrigin().z());
  ret.push_back(transform.getRotation().x());
  ret.push_back(transform.getRotation().y());
  ret.push_back(transform.getRotation().z());
  ret.push_back(transform.getRotation().w());
  return ret;
}

planning_models::KinematicModel::PrismaticJointModel::PrismaticJointModel(const std::string& name,
                                                                          const MultiDofConfig* multi_dof_config) 
  : JointModel(name), 
    axis_(0.0, 0.0, 0.0) 
{
  std::vector<std::string> local_names(1,name);
  initialize(local_names, multi_dof_config);
}

btTransform planning_models::KinematicModel::PrismaticJointModel::computeTransform(const std::vector<double>& joint_values) const 
{
  btTransform variable_transform;
  variable_transform.setIdentity();
  if(joint_values.size() != 1) {
    ROS_ERROR("Prismatic joint given wrong number of values");
    return variable_transform;
  }
  variable_transform.setOrigin(axis_*joint_values[0]);
  return variable_transform;
}

std::vector<double> planning_models::KinematicModel::PrismaticJointModel::computeJointStateValues(const btTransform& transform) const
{
  std::vector<double> ret;
  ret.push_back(transform.getOrigin().dot(axis_));
  return ret;
}

planning_models::KinematicModel::RevoluteJointModel::RevoluteJointModel(const std::string& name,
                                                              const MultiDofConfig* multi_dof_config) 
  : JointModel(name),
    axis_(0.0, 0.0, 0.0), 
    continuous_(false)
{
  std::vector<std::string> local_names(1,name);
  initialize(local_names, multi_dof_config);
}

btTransform planning_models::KinematicModel::RevoluteJointModel::computeTransform(const std::vector<double>& joint_values) const 
{
  btTransform variable_transform;
  variable_transform.setIdentity();
  if(joint_values.size() != 1) {
    ROS_ERROR("Revolute joint given wrong number of values");
    return variable_transform;
  }
  double val = joint_values.front();
  if(continuous_) {
    val = angles::normalize_angle(val);
  }
  variable_transform.setRotation(btQuaternion(axis_,val));
  return variable_transform;
}

std::vector<double> planning_models::KinematicModel::RevoluteJointModel::computeJointStateValues(const btTransform& transform) const
{
  std::vector<double> ret;
  ret.push_back(transform.getRotation().getAngle()*transform.getRotation().getAxis().dot(axis_));
  return ret;
}

/* ------------------------ LinkModel ------------------------ */

planning_models::KinematicModel::LinkModel::LinkModel(const KinematicModel* kinematic_model) : 
  kinematic_model_(kinematic_model),
  parent_joint_model_(NULL), 
  shape_(NULL)
{
  joint_origin_transform_.setIdentity();
  collision_origin_transform_.setIdentity();
}

planning_models::KinematicModel::LinkModel::LinkModel(const LinkModel* link_model) :
  name_(link_model->name_), 
  kinematic_model_(link_model->kinematic_model_),
  joint_origin_transform_(link_model->joint_origin_transform_),
  collision_origin_transform_(link_model->collision_origin_transform_)
{
  shape_ = shapes::cloneShape(link_model->shape_);
  for (unsigned int i = 0 ; i < link_model->attached_body_models_.size() ; ++i)
  {
    std::vector<shapes::Shape*> shapes;
    for(unsigned int j = 0; j < link_model->attached_body_models_[i]->getShapes().size(); j++) {
      shapes.push_back(shapes::cloneShape(link_model->attached_body_models_[i]->getShapes()[j]));
    }
    AttachedBodyModel *ab = new AttachedBodyModel(this, 
                                                  link_model->attached_body_models_[i]->getName(),
                                                  link_model->attached_body_models_[i]->getAttachedBodyFixedTransforms(),
                                                  link_model->attached_body_models_[i]->getTouchLinks(),
                                                  shapes);
    attached_body_models_.push_back(ab);
  }
}

planning_models::KinematicModel::LinkModel::~LinkModel(void)
{
  if (shape_)
    delete shape_;
  for (unsigned int i = 0 ; i < child_joint_models_.size() ; ++i)
    delete child_joint_models_[i];
  for (unsigned int i = 0 ; i < attached_body_models_.size() ; ++i)
    delete attached_body_models_[i];
}

void planning_models::KinematicModel::LinkModel::clearAttachedBodyModels() 
{
  //assumes exclusive lock has been granted
  for (unsigned int i = 0 ; i < attached_body_models_.size() ; ++i)
    delete attached_body_models_[i];
  attached_body_models_.clear();
}

void planning_models::KinematicModel::LinkModel::replaceAttachedBodyModels(std::vector<AttachedBodyModel*>& attached_body_vector) 
{
  //assumes exclusive lock has been granted
  for (unsigned int i = 0 ; i < attached_body_models_.size() ; ++i)
    delete attached_body_models_[i];
  attached_body_models_.clear();

  attached_body_models_ = attached_body_vector;
}

void planning_models::KinematicModel::LinkModel::clearLinkAttachedBodyModel(const std::string& att_name) 
{
  for(std::vector<AttachedBodyModel*>::iterator it = attached_body_models_.begin();
      it != attached_body_models_.end();
      it++) {
    if((*it)->getName() == att_name) {
      delete (*it);
      attached_body_models_.erase(it);
      return;
    }
  }
}

void planning_models::KinematicModel::LinkModel::addAttachedBodyModel(planning_models::KinematicModel::AttachedBodyModel* ab)
{
  attached_body_models_.push_back(ab);
}

/* ------------------------ AttachedBodyModel ------------------------ */

planning_models::KinematicModel::AttachedBodyModel::AttachedBodyModel(const LinkModel *link, 
                                                                      const std::string& nid,
                                                                      const std::vector<btTransform>& attach_trans,
                                                                      const std::vector<std::string>& touch_links,
                                                                      std::vector<shapes::Shape*>& shapes)

  : attached_link_model_(link), 
    id_(nid)
{
  attach_trans_ = attach_trans;
  touch_links_ = touch_links;
  shapes_ = shapes;
}

planning_models::KinematicModel::AttachedBodyModel::~AttachedBodyModel(void)
{
  for(unsigned int i = 0; i < shapes_.size(); i++) {
    delete shapes_[i];
  }
}

/* ------------------------ JointModelGroup ------------------------ */
planning_models::KinematicModel::JointModelGroup::JointModelGroup(const std::string& group_name,
                                                                  const std::vector<const JointModel*> &group_joints) :

  name_(group_name)
{
  joint_model_vector_ = group_joints;
  joint_model_name_vector_.resize(group_joints.size());
    
  for (unsigned int i = 0 ; i < group_joints.size() ; ++i)
  {
    joint_model_name_vector_[i] = group_joints[i]->getName();
    joint_model_map_[group_joints[i]->getName()] = group_joints[i];
  }
    
  for (unsigned int i = 0 ; i < group_joints.size() ; ++i)
  {
    bool found = false;
    const JointModel *joint = joint_model_vector_[i];
    while (joint->parent_link_model_)
    {
      joint = joint->getParentLinkModel()->getParentJointModel();
      if (hasJointModel(joint->name_))
      {
        found = true;
        break;
      }
    }
	
    if (!found)
      joint_roots_.push_back(joint_model_vector_[i]);
  }

  for (unsigned int i = 0 ; i < joint_roots_.size() ; ++i)
  {
    std::queue<const LinkModel*> links;
    links.push(joint_roots_[i]->getChildLinkModel());
	
    while (!links.empty())
    {
      const LinkModel *link = links.front();
      if(link == NULL) {
        ROS_WARN("Null link in group creation");
      }
      links.pop();
      updated_link_model_vector_.push_back(link);
      for (unsigned int i = 0 ; i < link->getChildJointModels().size() ; ++i)
        links.push(link->getChildJointModels()[i]->getChildLinkModel());
    }
  }
}

planning_models::KinematicModel::JointModelGroup::~JointModelGroup(void)
{
}

bool planning_models::KinematicModel::JointModelGroup::hasJointModel(const std::string &joint) const
{
  return joint_model_map_.find(joint) != joint_model_map_.end();
}

const planning_models::KinematicModel::JointModel* planning_models::KinematicModel::JointModelGroup::getJointModel(const std::string &name)
{
  if(!hasJointModel(name)) return NULL;
  return joint_model_map_.find(name)->second;
}

