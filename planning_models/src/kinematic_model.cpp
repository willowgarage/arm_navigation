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
                                                const std::vector<MultiDofConfig>& multi_dof_configs,
                                                bool load_meshes)
{    
  load_meshes_ = load_meshes;
  model_name_ = model.getName();
  if (model.getRoot())
  {
    const urdf::Link *root = model.getRoot().get();
    root_ = buildRecursive(NULL, root, multi_dof_configs);
    buildGroups(groups);
    buildConvenientDatastructures();    
  }
  else
  {
    root_ = NULL;
    ROS_WARN("No root link found");
  }
}

planning_models::KinematicModel::KinematicModel(const KinematicModel &source, bool load_meshes)
{
  load_meshes_ = load_meshes;
  copyFrom(source);
}

planning_models::KinematicModel::~KinematicModel(void)
{
  for (std::map<std::string, JointGroup*>::iterator it = group_map_.begin() ; it != group_map_.end() ; ++it)
    delete it->second;
  if (root_)
    delete root_;
}

const std::string& planning_models::KinematicModel::getName() const {
  return model_name_;
}

void planning_models::KinematicModel::copyFrom(const KinematicModel &source)
{
  model_name_ = source.model_name_;

  if (source.root_)
  {
    root_ = copyRecursive(NULL, source.root_->child_link);
    
    std::vector<const JointGroup*> groups;
    source.getGroups(groups);
    std::map< std::string, std::vector<std::string> > groupContent;
    for (unsigned int i = 0 ; i < groups.size() ; ++i)
      groupContent[groups[i]->name] = groups[i]->joint_names;
    buildGroups(groupContent);
    buildConvenientDatastructures();    
  } else 
  {
    root_ = NULL;
  }
}

const btTransform& planning_models::KinematicModel::getRootTransform(void) const
{
  return root_->variable_transform;
}

void planning_models::KinematicModel::setRootTransform(const btTransform &transform)
{
  root_->variable_transform = transform;
}

void planning_models::KinematicModel::lock(void)
{
  lock_.lock();
}

void planning_models::KinematicModel::unlock(void)
{
  lock_.unlock();
}

void planning_models::KinematicModel::defaultState(void)
{
  std::map<std::string, double> default_joint_states;

  const unsigned int js = joint_list_.size();
  for (unsigned int i = 0  ; i < js ; ++i)
  {
    for(std::map<std::string, std::pair<double,double> >::iterator it = joint_list_[i]->joint_state_bounds.begin();
        it != joint_list_[i]->joint_state_bounds.end();
        it++) {
      std::pair<double,double> bounds = it->second;
      if(bounds.first <= 0.0 && bounds.second >= 0.0) {
        default_joint_states[it->first] = 0.0;
      } else {
        default_joint_states[it->first] = (bounds.first+bounds.second)/2.0;
      }
    }
  }
  computeTransforms(default_joint_states);
}

void planning_models::KinematicModel::buildConvenientDatastructures(void)
{
  if (root_)
  {
    updated_links_.push_back(root_->child_link);
    getChildLinks(root_->child_link, updated_links_);      
  }
}

void planning_models::KinematicModel::buildGroups(const std::map< std::string, std::vector<std::string> > &groups)
{
  for (std::map< std::string, std::vector<std::string> >::const_iterator it = groups.begin() ; it != groups.end() ; ++it)
  {
    std::vector<Joint*> jointv;
    for (unsigned int i = 0 ; i < it->second.size() ; ++i)
    {
      std::map<std::string, Joint*>::iterator p = joint_map_.find(it->second[i]);
      if (p == joint_map_.end())
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
      group_map_[it->first] = new JointGroup(this, it->first, jointv);
    }
  }
}

planning_models::KinematicModel::Joint* planning_models::KinematicModel::buildRecursive(Link *parent, const urdf::Link *link, 
                                                                                        const std::vector<MultiDofConfig>& multi_dof_configs)
{  
  Joint *joint = constructJoint(link->parent_joint.get(), link, multi_dof_configs);
  joint_map_[joint->name] = joint;
  for(Joint::js_type::iterator it = joint->joint_state_equivalents.begin();
      it != joint->joint_state_equivalents.end();
      it++) {
    joint_map_[it->right] = joint;
  }
  joint_list_.push_back(joint);
  joint->parent_link = parent;
  joint->child_link = constructLink(link);
  if (parent == NULL)
    joint->child_link->joint_origin_transform.setIdentity();
  link_map_[joint->child_link->name] = joint->child_link;
  joint->child_link->parent_joint = joint;
  
  for (unsigned int i = 0 ; i < link->child_links.size() ; ++i)
    joint->child_link->child_joint.push_back(buildRecursive(joint->child_link, link->child_links[i].get(), multi_dof_configs));
  
  return joint;
}

planning_models::KinematicModel::Joint* planning_models::KinematicModel::constructJoint(const urdf::Joint *urdf_joint,
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

  planning_models::KinematicModel::Joint *result = NULL;

  //must be the root link transform
  if(urdf_joint == NULL) {
    if(!found) {
      ROS_ERROR("Root transform has no multi dof joint config");
      return NULL;
    }
    if(joint_config->type == "Planar") {
      result = new PlanarJoint(this, joint_config->name, joint_config);
    } else if(joint_config->type == "Floating") {
      result = new FloatingJoint(this, joint_config->name, joint_config);
    } else {
      ROS_ERROR_STREAM("Unrecognized type of multi dof joint " << joint_config->type);
      return NULL;
    }
  } else {
    switch (urdf_joint->type)
    {
    case urdf::Joint::REVOLUTE:
      {
        RevoluteJoint *j = new RevoluteJoint(this, urdf_joint->name, joint_config);
        if(urdf_joint->safety)
        {
          j->setVariableBounds(j->name, urdf_joint->safety->soft_lower_limit, urdf_joint->safety->soft_upper_limit);
        }
        else
        {
          j->setVariableBounds(j->name, urdf_joint->limits->upper, urdf_joint->limits->lower);
        }
        j->continuous = false;
        j->axis.setValue(urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z);
        result = j;
      }
      break;
    case urdf::Joint::CONTINUOUS:
      {
        RevoluteJoint *j = new RevoluteJoint(this, urdf_joint->name, joint_config);
        j->continuous = true;
        j->setVariableBounds(j->name, -M_PI, M_PI);
        j->axis.setValue(urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z);
        result = j;
      }
      break;
    case urdf::Joint::PRISMATIC:
      {
        PrismaticJoint *j = new PrismaticJoint(this, urdf_joint->name, joint_config);
        if(urdf_joint->safety)
        {
          j->setVariableBounds(j->name, urdf_joint->safety->soft_lower_limit, urdf_joint->safety->soft_upper_limit);
        }
        else
        {
          j->setVariableBounds(j->name, urdf_joint->limits->upper, urdf_joint->limits->lower);
        }
        j->axis.setValue(urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z);
        result = j;
      }
      break;
    case urdf::Joint::FLOATING:
      result = new FloatingJoint(this, urdf_joint->name, joint_config);
      break;
    case urdf::Joint::PLANAR:
      result = new PlanarJoint(this, urdf_joint->name, joint_config);
      break;
    case urdf::Joint::FIXED:
      result = new FixedJoint(this, urdf_joint->name, joint_config);
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

planning_models::KinematicModel::Link* planning_models::KinematicModel::constructLink(const urdf::Link *urdf_link)
{
  ROS_ASSERT(urdf_link);

  Link *result = new Link(this);
  result->name = urdf_link->name;

  if(urdf_link->collision)
    result->collision_origin_transform = urdfPose2btTransform(urdf_link->collision->origin);
  else
    result->collision_origin_transform.setIdentity();

  if (urdf_link->parent_joint.get())
    result->joint_origin_transform = urdfPose2btTransform(urdf_link->parent_joint->parent_to_joint_origin_transform);
  else
    result->joint_origin_transform.setIdentity();
    
  if(urdf_link->collision && load_meshes_)
    result->shape = constructShape(urdf_link->collision->geometry.get());
  else if(load_meshes_)
  {
    shapes::Shape *tmp_shape = NULL;
    tmp_shape = new shapes::Sphere(0.0001);
    result->shape = tmp_shape;   
  } else {
    result->shape = NULL;
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

void planning_models::KinematicModel::computeTransforms(std::map<std::string, double> joint_value_map)
{
  unsigned int js = joint_list_.size();
  for (unsigned int i = 0  ; i < js ; ++i)
    joint_list_[i]->updateVariableTransform(joint_value_map);
    
  unsigned int ls = updated_links_.size();
  for (unsigned int i = 0 ; i < ls ; ++i)
    updated_links_[i]->computeTransform();
}

void planning_models::KinematicModel::computeTransforms(){
  //assume that joint transforms have been computed
  const unsigned int ls = updated_links_.size();
  for (unsigned int i = 0 ; i < ls ; ++i)
    updated_links_[i]->computeTransform();
}

std::map<std::string, double> planning_models::KinematicModel::getAllJointsValues() const {
  std::map<std::string, double> ret;
  unsigned int js = joint_list_.size();
  for (unsigned int i = 0  ; i < js ; ++i) {
    std::map<std::string, double> j_vals = joint_list_[i]->getVariableTransformValues();
    ret.insert(j_vals.begin(), j_vals.end());
  }
  return ret;
}

std::vector<double> planning_models::KinematicModel::getAllJointsValuesVector() const {
  std::vector<double> ret;
  for(std::map<std::string,Joint*>::const_iterator it = joint_map_.begin();
      it != joint_map_.end();
      it++) {
    for(std::map<std::string, double>::const_iterator it2 = it->second->stored_joint_values.begin();
        it2 != it->second->stored_joint_values.end();
        it2++) {
      ret.push_back(it2->second);
    }
  }
  return ret;
}

std::map<std::string, unsigned int> planning_models::KinematicModel::getMapOrderIndex() const {
  std::map<std::string, unsigned int> ret;
  unsigned int i = 0;
  for(std::map<std::string,Joint*>::const_iterator it = joint_map_.begin();
      it != joint_map_.end();
      it++) {
    for(std::map<std::string, double>::const_iterator it2 = it->second->stored_joint_values.begin();
        it2 != it->second->stored_joint_values.end();
        it2++) {
      ret[it2->first] = i;
      i++;
    }
  }
  return ret;
}

void planning_models::KinematicModel::setAllJointsValues(const std::vector<double>& joint_values) {
  std::vector<double>::const_iterator vit = joint_values.begin();
  for(std::map<std::string,Joint*>::iterator it = joint_map_.begin();
      it != joint_map_.end();
      it++) {
    for(std::map<std::string, double>::iterator it2 = it->second->stored_joint_values.begin();
        it2 != it->second->stored_joint_values.end();
        it2++, vit++) {
      if(vit == joint_values.end()) {
        ROS_WARN_STREAM("Not enough joint values to set " << it2->first);
      } else {
        it2->second = *vit;
      } 
    }
    it->second->updateVariableTransformFromStoredJointValues();
  }
  if(vit != joint_values.end()) {
    ROS_WARN("Too many values in joint values");
  }
}

std::map<std::string, double> planning_models::KinematicModel::getJointValues(const std::string joint) const {
  std::map<std::string, double> ret;
  if(hasJoint(joint)) {
   ret = joint_map_.find(joint)->second->getVariableTransformValues();
  }
  return ret;
}


bool planning_models::KinematicModel::checkJointBounds(std::string joint_name) const {

  if(joint_map_.find(joint_name) == joint_map_.end()) {
    ROS_WARN_STREAM("Can't check bounds for joint I don't have " << joint_name);
    return false;
  }
  const Joint* joint = joint_map_.find(joint_name)->second;

  //returns with config names
  std::map<std::string, double> cur_vals = joint->getVariableTransformValues();
  for(std::map<std::string, double>::iterator it = cur_vals.begin();
      it != cur_vals.end();
      it++) {
    std::pair<double,double> bounds = joint->getVariableBounds(it->first);
    if(bounds.first > it->second ||
       bounds.second < it->second) {
      return false;
    }
  }
  return true;
}

bool planning_models::KinematicModel::checkJointsBounds(std::vector<std::string> joints) const {
  for(std::vector<std::string>::iterator it = joints.begin();
      it != joints.end();
      it++) {
    if(!checkJointBounds(*it)) {
      return false;
    }
  }
  return true;
}

const planning_models::KinematicModel::Joint* planning_models::KinematicModel::getRoot(void) const
{
  return root_;
}

planning_models::KinematicModel::Joint* planning_models::KinematicModel::getRoot(void)
{						
  return root_;
}

bool planning_models::KinematicModel::hasJoint(const std::string &name) const
{
  return joint_map_.find(name) != joint_map_.end();
}

bool planning_models::KinematicModel::hasLink(const std::string &name) const
{
  return link_map_.find(name) != link_map_.end();
}

bool planning_models::KinematicModel::hasGroup(const std::string &name) const
{
  return group_map_.find(name) != group_map_.end();
}

const planning_models::KinematicModel::Joint* planning_models::KinematicModel::getJoint(const std::string &name) const
{
  std::map<std::string, Joint*>::const_iterator it = joint_map_.find(name);
  if (it == joint_map_.end())
  {
    ROS_ERROR("Joint '%s' not found", name.c_str());
    return NULL;
  }
  else
    return it->second;
}

planning_models::KinematicModel::Joint* planning_models::KinematicModel::getJoint(const std::string &name)
{
  std::map<std::string, Joint*>::iterator it = joint_map_.find(name);
  if (it == joint_map_.end())
  {
    ROS_ERROR("Joint '%s' not found", name.c_str());
    return NULL;
  }
  else
    return it->second;
}

const planning_models::KinematicModel::Link* planning_models::KinematicModel::getLink(const std::string &name) const
{
  std::map<std::string, Link*>::const_iterator it = link_map_.find(name);
  if (it == link_map_.end())
  {
    ROS_ERROR("Link '%s' not found", name.c_str());
    return NULL;
  }
  else
    return it->second;
}

planning_models::KinematicModel::Link* planning_models::KinematicModel::getLink(const std::string &name)
{
  std::map<std::string, Link*>::iterator it = link_map_.find(name);
  if (it == link_map_.end())
  {
    ROS_ERROR("Link '%s' not found", name.c_str());
    return NULL;
  }
  else
    return it->second;
}

const planning_models::KinematicModel::JointGroup* planning_models::KinematicModel::getGroup(const std::string &name) const
{
  std::map<std::string, JointGroup*>::const_iterator it = group_map_.find(name);
  if (it == group_map_.end())
  {
    ROS_ERROR("Joint group '%s' not found", name.c_str());
    return NULL;
  }
  else
    return it->second;
}

planning_models::KinematicModel::JointGroup* planning_models::KinematicModel::getGroup(const std::string &name)
{
  std::map<std::string, JointGroup*>::iterator it = group_map_.find(name);
  if (it == group_map_.end())
  {
    ROS_ERROR("Joint group '%s' not found", name.c_str());
    return NULL;
  }
  else
    return it->second;
}

void planning_models::KinematicModel::getGroups(std::vector<const JointGroup*> &groups) const
{
  groups.clear();
  groups.reserve(group_map_.size());
  for (std::map<std::string, JointGroup*>::const_iterator it = group_map_.begin() ; it != group_map_.end() ; ++it)
    groups.push_back(it->second);
}

void planning_models::KinematicModel::getGroupNames(std::vector<std::string> &groups) const
{
  groups.clear();
  groups.reserve(group_map_.size());
  for (std::map<std::string, JointGroup*>::const_iterator it = group_map_.begin() ; it != group_map_.end() ; ++it)
    groups.push_back(it->second->name);
}

void planning_models::KinematicModel::getLinks(std::vector<const Link*> &links) const
{
  links.clear();
  links.reserve(link_map_.size());
  for (std::map<std::string, Link*>::const_iterator it = link_map_.begin() ; it != link_map_.end() ; ++it)
    links.push_back(it->second);
}

void planning_models::KinematicModel::getLinkNames(std::vector<std::string> &links) const
{
  links.clear();
  links.reserve(link_map_.size());
  for (std::map<std::string, Link*>::const_iterator it = link_map_.begin() ; it != link_map_.end() ; ++it)
    links.push_back(it->second->name);
}

void planning_models::KinematicModel::getJoints(std::vector<const Joint*> &joints) const
{
  joints.clear();
  joints.reserve(joint_list_.size());
  for (unsigned int i = 0 ; i < joint_list_.size() ; ++i)
    joints.push_back(joint_list_[i]);
}

void planning_models::KinematicModel::getJoints(std::vector<Joint*> &joints) const
{
  joints.clear();
  joints.reserve(joint_list_.size());
  for (unsigned int i = 0 ; i < joint_list_.size() ; ++i)
    joints.push_back(joint_list_[i]);
}

void planning_models::KinematicModel::getJointNames(std::vector<std::string> &joints) const
{
  joints.clear();
  joints.reserve(joint_list_.size());
  for (unsigned int i = 0 ; i < joint_list_.size() ; ++i)
    joints.push_back(joint_list_[i]->name);
}

planning_models::KinematicModel::Joint* planning_models::KinematicModel::copyRecursive(Link *parent, const Link *link)
{
  Joint *joint = copyJoint(link->parent_joint);
  joint_map_[joint->name] = joint;
  for(Joint::js_type::iterator it = joint->joint_state_equivalents.begin();
      it != joint->joint_state_equivalents.end();
      it++) {
    joint_map_[it->right] = joint;
  }
  joint_list_.push_back(joint);
  joint->parent_link = parent;
  joint->child_link = copyLink(link);
  link_map_[joint->child_link->name] = joint->child_link;
  joint->child_link->parent_joint = joint;
    
  for (unsigned int i = 0 ; i < link->child_joint.size() ; ++i)
    joint->child_link->child_joint.push_back(copyRecursive(joint->child_link, link->child_joint[i]->child_link));
    
  return joint;
}

planning_models::KinematicModel::Link* planning_models::KinematicModel::copyLink(const Link *link)
{
  Link *newLink = new Link(this);
    
  newLink->name = link->name;
  newLink->joint_origin_transform = link->joint_origin_transform;
  newLink->collision_origin_transform = link->collision_origin_transform;
  newLink->global_link_transform = link->global_link_transform;
  newLink->global_collision_body_transform = link->global_collision_body_transform;
  newLink->shape = shapes::cloneShape(link->shape);
    
  for (unsigned int i = 0 ; i < link->attached_bodies.size() ; ++i)
  {
    AttachedBody *ab = new AttachedBody(newLink, link->attached_bodies[i]->id);
    for(unsigned int j = 0; j < ab->shapes.size(); j++) {
      ab->shapes.push_back(shapes::cloneShape(link->attached_bodies[i]->shapes[j]));
    }
    ab->attach_trans = link->attached_bodies[i]->attach_trans;
    ab->global_collision_body_transform = link->attached_bodies[i]->global_collision_body_transform;
    ab->touch_links = link->attached_bodies[i]->touch_links;
    newLink->attached_bodies.push_back(ab);
  }
    
  return newLink;
}

planning_models::KinematicModel::Joint* planning_models::KinematicModel::copyJoint(const Joint *joint)
{
  Joint *newJoint = NULL;

  if (dynamic_cast<const FixedJoint*>(joint))
  {
    newJoint = new FixedJoint(dynamic_cast<const FixedJoint*>(joint));
  }
  else if (dynamic_cast<const FloatingJoint*>(joint))
  {
    newJoint = new FloatingJoint(dynamic_cast<const FloatingJoint*>(joint));
  }
  else if (dynamic_cast<const PlanarJoint*>(joint))
  {
    newJoint = new PlanarJoint(dynamic_cast<const PlanarJoint*>(joint));
  }
  else if (dynamic_cast<const PrismaticJoint*>(joint))
  {
    newJoint = new PrismaticJoint(dynamic_cast<const PrismaticJoint*>(joint));
  }
  else if (dynamic_cast<const RevoluteJoint*>(joint))
  {
    newJoint = new RevoluteJoint(dynamic_cast<const RevoluteJoint*>(joint));
  }
  else
    ROS_FATAL("Unimplemented type of joint");
    
  return newJoint;
}

void planning_models::KinematicModel::printModelInfo(std::ostream &out) const
{
  out << "Complete model state dimension = " << getAllJointsValues().size() << std::endl;
    
  std::ios_base::fmtflags old_flags = out.flags();    
  out.setf(std::ios::fixed, std::ios::floatfield);
  std::streamsize old_prec = out.precision();
  out.precision(5);
  out << "State bounds: ";
  for(unsigned int i = 0; i < joint_list_.size(); i++) {
    for(std::map<std::string, std::pair<double, double> >::const_iterator it = joint_list_[i]->joint_state_bounds.begin();
        it != joint_list_[i]->joint_state_bounds.end();
        it++) {
      if(it->second.first == -DBL_MAX) {
        out << "[-DBL_MAX, ";
      } else {
        out << "[" << it->second.first << ", ";
      }
      if(it->second.second == DBL_MAX) {
        out << "DBL_MAX] ";
      } else {
        out << it->second.second << "] ";  
      }
    }
  }
    
  out << std::endl;
  out.precision(old_prec);    
  out.flags(old_flags);
        
  out << "Root joint : ";
  out << getRoot()->name << " ";
  out << std::endl;
    
  out << "Available groups: ";
  std::vector<std::string> l;
  getGroupNames(l);
  for (unsigned int i = 0 ; i < l.size() ; ++i)
    out << l[i] << " ";
  out << std::endl;
    
  for (unsigned int i = 0 ; i < l.size() ; ++i)
  {
    const JointGroup *g = getGroup(l[i]);
    out << "Group " << l[i] << " has " << g->joint_roots.size() << " roots: ";
    for (unsigned int j = 0 ; j < g->joint_roots.size() ; ++j)
      out << g->joint_roots[j]->name << " ";
    out << std::endl;
  }
}

void planning_models::KinematicModel::printTransform(const std::string &st, const btTransform &t, std::ostream &out) const
{
  out << st << std::endl;
  const btVector3 &v = t.getOrigin();
  out << "  origin: " << v.x() << ", " << v.y() << ", " << v.z() << std::endl;
  const btQuaternion &q = t.getRotation();
  out << "  quaternion: " << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << std::endl;
}

void planning_models::KinematicModel::printTransforms(std::ostream &out) const
{
  out << "Joint transforms:" << std::endl;
  std::vector<const Joint*> joints;
  getJoints(joints);
  for (unsigned int i = 0 ; i < joints.size() ; ++i)
  {
    printTransform(joints[i]->name, joints[i]->variable_transform, out);
    out << std::endl;	
  }
  out << "Link poses:" << std::endl;
  std::vector<const Link*> links;
  getLinks(links);
  for (unsigned int i = 0 ; i < links.size() ; ++i)
  {
    printTransform(links[i]->name, links[i]->global_collision_body_transform, out);
    out << std::endl;	
  }    
}

/* ------------------------ Joint ------------------------ */

planning_models::KinematicModel::Joint::Joint(KinematicModel *model, const std::string n) :
  name(n), owner(model), parent_link(NULL), child_link(NULL)
{
  variable_transform.setIdentity();
}

void planning_models::KinematicModel::Joint::initialize(const std::vector<std::string>& local_joint_names,
                                                        const MultiDofConfig* config)
{
  for(std::vector<std::string>::const_iterator it = local_joint_names.begin();
      it != local_joint_names.end();
      it++) {
    joint_state_equivalents.insert(js_type::value_type(*it,*it));
  } 
  if(config != NULL) {
    for(std::map<std::string, std::string>::const_iterator it = config->name_equivalents.begin();
        it != config->name_equivalents.end();
        it++) {
      js_type::left_iterator lit = joint_state_equivalents.left.find(it->first);
      if(lit != joint_state_equivalents.left.end()) {
        joint_state_equivalents.left.replace_data(lit, it->second);
      }
    }
    parent_frame_id = config->parent_frame_id;
    child_frame_id = config->child_frame_id;
  }

  for(js_type::iterator it = joint_state_equivalents.begin();
      it != joint_state_equivalents.end();
      it++) {
    setVariableBounds(it->right, 0.0,0.0);
    stored_joint_values[it->right] = 0.0;
  }
}

planning_models::KinematicModel::Joint::Joint(const Joint* joint) {
  name = joint->name;
  owner = joint->owner;
  parent_frame_id  = joint->parent_frame_id;
  child_frame_id  = joint->child_frame_id;
  joint_state_equivalents = joint->joint_state_equivalents;
  joint_state_bounds = joint->joint_state_bounds;
  variable_transform = joint->variable_transform;
  stored_joint_values = joint->stored_joint_values;
}

planning_models::KinematicModel::Joint::~Joint(void)
{
  if (child_link)
    delete child_link;
}

std::string planning_models::KinematicModel::Joint::getEquiv(const std::string name) {
  js_type::left_iterator lit = joint_state_equivalents.left.find(name);
  if(lit != joint_state_equivalents.left.end()) {
    return lit->second;
  } else {
    return "";
  }
}

void planning_models::KinematicModel::Joint::setVariableBounds(std::string variable, double low, double high) {
  if(joint_state_equivalents.right.find(variable) == joint_state_equivalents.right.end()) {
    ROS_WARN_STREAM("Can't find variable " << variable << " to set bounds");
    return;
  }
  joint_state_bounds[joint_state_equivalents.right.at(variable)] = std::pair<double,double>(low, high);
}

std::pair<double, double> planning_models::KinematicModel::Joint::getVariableBounds(std::string variable) const{
  if(joint_state_equivalents.right.find(variable) == joint_state_equivalents.right.end()) {
    ROS_WARN_STREAM("Can't find variable " << variable << " to get bounds");
    return std::pair<double,double>(0.0,0.0);
  }
  std::string config_name = joint_state_equivalents.right.find(variable)->second;
  if(joint_state_bounds.find(config_name) == joint_state_bounds.end()) {
    ROS_WARN_STREAM("No joint bounds for " << config_name);
    return std::pair<double,double>(0.0,0.0);
  }
  return joint_state_bounds.find(config_name)->second;
}

bool planning_models::KinematicModel::Joint::allJointStateEquivalentsAreDefined(const std::map<std::string, double>& joint_value_map) const 
{
  for(js_type::const_iterator it = joint_state_equivalents.begin();
      it != joint_state_equivalents.end();
      it++) {
    if(joint_value_map.find(it->right) == joint_value_map.end()) {
      return false;
    }
  }
  return true;
}

bool planning_models::KinematicModel::Joint::setStoredJointValues(const std::map<std::string, double>& joint_value_map) {
  bool changed = false;
  for(js_type::const_iterator it = joint_state_equivalents.begin();
      it != joint_state_equivalents.end();
      it++) {
    if(joint_value_map.find(it->right) != joint_value_map.end()) {
      if(stored_joint_values[it->right] != joint_value_map.at(it->right)) {
        stored_joint_values[it->right] = joint_value_map.at(it->right);
        changed = true;
      }
    }
  }
  return changed;
}

bool planning_models::KinematicModel::Joint::updateVariableTransform(const std::map<std::string, double>& joint_value_map){
  bool changed = setStoredJointValues(joint_value_map);
  if(changed) {
    updateVariableTransformFromStoredJointValues();
  }  
  return(allJointStateEquivalentsAreDefined(joint_value_map));
}

const std::map<std::string, double>& planning_models::KinematicModel::Joint::getVariableTransformValues() const{
  return stored_joint_values;
}

planning_models::KinematicModel::PlanarJoint::PlanarJoint(KinematicModel *owner, const std::string name, const MultiDofConfig* multi_dof_config) 
  : Joint(owner, name)
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

void planning_models::KinematicModel::PlanarJoint::updateVariableTransformFromStoredJointValues() {
  variable_transform.setOrigin(btVector3(stored_joint_values[getEquiv("planar_x")],
                                         stored_joint_values[getEquiv("planar_y")],
                                         0.0));
  variable_transform.setRotation(btQuaternion(btVector3(0.0, 0.0, 1.0),
                                              stored_joint_values[getEquiv("planar_th")]));
}

bool planning_models::KinematicModel::PlanarJoint::updateVariableTransform(const btTransform& trans){
  stored_joint_values[getEquiv("planar_x")] = trans.getOrigin().x();
  stored_joint_values[getEquiv("planar_y")] = trans.getOrigin().y();
  stored_joint_values[getEquiv("planar_th")] = trans.getRotation().getAngle()*trans.getRotation().getAxis().z();
  updateVariableTransformFromStoredJointValues();
  return true;
}

planning_models::KinematicModel::FloatingJoint::FloatingJoint(KinematicModel *owner, const std::string name, const MultiDofConfig* multi_dof_config) 
  : Joint(owner, name)
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

void planning_models::KinematicModel::FloatingJoint::updateVariableTransformFromStoredJointValues()
{
  variable_transform.setOrigin(btVector3(stored_joint_values[getEquiv("floating_trans_x")],
                                         stored_joint_values[getEquiv("floating_trans_y")],
                                         stored_joint_values[getEquiv("floating_trans_z")]));
  variable_transform.setRotation(btQuaternion(stored_joint_values[getEquiv("floating_rot_x")],
                                              stored_joint_values[getEquiv("floating_rot_w")],
                                              stored_joint_values[getEquiv("floating_rot_z")],
                                              stored_joint_values[getEquiv("floating_rot_w")]));
}
 
bool planning_models::KinematicModel::FloatingJoint::updateVariableTransform(const btTransform& trans){
  stored_joint_values[getEquiv("floating_trans_x")] = trans.getOrigin().x();
  stored_joint_values[getEquiv("floating_trans_y")] = trans.getOrigin().y();
  stored_joint_values[getEquiv("floating_trans_z")] = trans.getOrigin().z();
  stored_joint_values[getEquiv("floating_rot_x")] = trans.getRotation().x();
  stored_joint_values[getEquiv("floating_rot_y")] = trans.getRotation().y();
  stored_joint_values[getEquiv("floating_rot_z")] = trans.getRotation().z();
  stored_joint_values[getEquiv("floating_rot_w")] = trans.getRotation().w();
  updateVariableTransformFromStoredJointValues();
  return true;
}

planning_models::KinematicModel::PrismaticJoint::PrismaticJoint(KinematicModel *owner, 
                                                                const std::string name,
                                                                const MultiDofConfig* multi_dof_config) 
  : Joint(owner,name), 
    axis(0.0, 0.0, 0.0) 
{
  std::vector<std::string> local_names(1,name);
  initialize(local_names, multi_dof_config);
}
 
void planning_models::KinematicModel::PrismaticJoint::updateVariableTransformFromStoredJointValues() {
  variable_transform.setOrigin(axis*stored_joint_values[name]);
}

bool planning_models::KinematicModel::PrismaticJoint::updateVariableTransform(const btTransform& trans){
  stored_joint_values[name] = trans.getOrigin().dot(axis);
  updateVariableTransformFromStoredJointValues();
  return true;
}

planning_models::KinematicModel::RevoluteJoint::RevoluteJoint(KinematicModel *owner, 
                                                              std::string name,
                                                              const MultiDofConfig* multi_dof_config) 
  : Joint(owner,name),
    axis(0.0, 0.0, 0.0), 
    continuous(false)
{
  std::vector<std::string> local_names(1,name);
  initialize(local_names, multi_dof_config);
}

bool planning_models::KinematicModel::RevoluteJoint::updateVariableTransform(const std::map<std::string, double>& joint_value_map){
  
  std::map<std::string,double>::const_iterator it = joint_value_map.find(getEquiv(name));

  if(it == joint_value_map.end()) {
    return false;
  }
  double val = it->second;
  if(continuous) {
    val = angles::normalize_angle(val);
  }
  std::map<std::string,double> my_map;
  my_map[it->first] = val;
  bool changed = setStoredJointValues(my_map);
  if(changed) {
    updateVariableTransformFromStoredJointValues();
  }  
  return true;
}

void planning_models::KinematicModel::RevoluteJoint::updateVariableTransformFromStoredJointValues() {
  variable_transform.setRotation(btQuaternion(axis,stored_joint_values[name]));
}

bool planning_models::KinematicModel::RevoluteJoint::updateVariableTransform(const btTransform& trans){
  stored_joint_values[name] = trans.getRotation().getAngle()*trans.getRotation().getAxis().dot(axis);;
  updateVariableTransformFromStoredJointValues();
  return true;
}

/* ------------------------ Link ------------------------ */

planning_models::KinematicModel::Link::Link(KinematicModel *model) : owner(model), parent_joint(NULL), shape(NULL)
{
  joint_origin_transform.setIdentity();
  collision_origin_transform.setIdentity();
  global_link_transform.setIdentity();
  global_collision_body_transform.setIdentity();		
}

planning_models::KinematicModel::Link::~Link(void)
{
  if (shape)
    delete shape;
  for (unsigned int i = 0 ; i < child_joint.size() ; ++i)
    delete child_joint[i];
  for (unsigned int i = 0 ; i < attached_bodies.size() ; ++i)
    delete attached_bodies[i];
}

void planning_models::KinematicModel::Link::computeTransform(void)
{
  btTransform ident;
  ident.setIdentity();
  global_link_transform.mult(parent_joint->parent_link ? parent_joint->parent_link->global_link_transform : ident, joint_origin_transform);
  global_link_transform *= parent_joint->variable_transform;    
  global_collision_body_transform.mult(global_link_transform, collision_origin_transform);

  for (unsigned int i = 0 ; i < attached_bodies.size() ; ++i)
    attached_bodies[i]->computeTransform();
}

void planning_models::KinematicModel::updateTransformsWithLinkAt(Link *link, const btTransform &transform)
{
    // update the link at the new position
    link->global_link_transform = transform;
    link->global_collision_body_transform.mult(link->global_link_transform, link->collision_origin_transform);
    for (unsigned int i = 0 ; i < link->attached_bodies.size() ; ++i)
	link->attached_bodies[i]->computeTransform();
    
    std::vector<Link*> l;
    getChildLinks(link, l);
    for (unsigned int i = 0 ; i < l.size() ; ++i)
	l[i]->computeTransform();
}

void planning_models::KinematicModel::getChildLinks(const KinematicModel::Link *parent, std::vector<KinematicModel::Link*> &links)
{
  std::queue<const KinematicModel::Link*> q;
  q.push(parent);
  while (!q.empty())
  {
    const KinematicModel::Link* t = q.front();
    q.pop();
    
    for (unsigned int i = 0 ; i < t->child_joint.size() ; ++i) {
      if (t->child_joint[i]->child_link)
      {
        links.push_back(t->child_joint[i]->child_link);
        q.push(t->child_joint[i]->child_link);
      }
    }
  }
}

/* ------------------------ AttachedBody ------------------------ */

planning_models::KinematicModel::AttachedBody::AttachedBody(Link *link, const std::string& nid) : owner(link), id(nid)
{
}

planning_models::KinematicModel::AttachedBody::~AttachedBody(void)
{
  for(unsigned int i = 0; i < shapes.size(); i++) {
    delete shapes[i];
  }
}

void planning_models::KinematicModel::AttachedBody::computeTransform(void)
{
  for(unsigned int i = 0; i < global_collision_body_transform.size(); i++) {
    global_collision_body_transform[i] = owner->global_link_transform * attach_trans[i];
  }
}

/* ------------------------ JointGroup ------------------------ */
bool planning_models::KinematicModel::JointGroup::containsGroup(const JointGroup *group) const
{
  for (unsigned int i = 0 ; i < group->joint_names.size() ; ++i)
    if (!hasJoint(group->joint_names[i]))
      return false;
  return true;
}

planning_models::KinematicModel::JointGroup* planning_models::KinematicModel::JointGroup::addGroup(const JointGroup *group) const
{
  std::vector<Joint*> gjoints = joints;
  for (unsigned int j = 0 ; j < group->joints.size() ; ++j)
    if (!hasJoint(group->joints[j]->name))
      gjoints.push_back(group->joints[j]);
  return new JointGroup(owner, name + "+" + group->name, gjoints);
}

planning_models::KinematicModel::JointGroup* planning_models::KinematicModel::JointGroup::removeGroup(const JointGroup *group) const
{ 
  std::vector<Joint*> gjoints;
  for (unsigned int j = 0 ; j < joints.size() ; ++j)
    if (!group->hasJoint(joints[j]->name))
      gjoints.push_back(joints[j]);
  return new JointGroup(owner, name + "-" + group->name, gjoints);
}

planning_models::KinematicModel::JointGroup::JointGroup(KinematicModel *model, const std::string& group_name,
                                                        const std::vector<Joint*> &group_joints) : owner(model)
{
  name = group_name;
  joints = group_joints;
  joint_names.resize(joints.size());
    
  for (unsigned int i = 0 ; i < joints.size() ; ++i)
  {
    joint_names[i] = joints[i]->name;
    joint_map[joint_names[i]] = joints[i];
  }
    
  for (unsigned int i = 0 ; i < joints.size() ; ++i)
  {
    bool found = false;
    Joint *joint = joints[i];
    while (joint->parent_link)
    {
      joint = joint->parent_link->parent_joint;
      if (hasJoint(joint->name))
      {
        found = true;
        break;
      }
    }
	
    if (!found)
      joint_roots.push_back(joints[i]);
  }

  for (unsigned int i = 0 ; i < joint_roots.size() ; ++i)
  {
    std::queue<Link*> links;
    links.push(joint_roots[i]->child_link);
	
    while (!links.empty())
    {
      Link *link = links.front();
      links.pop();
      updated_links.push_back(link);
      for (unsigned int i = 0 ; i < link->child_joint.size() ; ++i)
        links.push(link->child_joint[i]->child_link);
    }
  }
}

planning_models::KinematicModel::JointGroup::~JointGroup(void)
{
}

bool planning_models::KinematicModel::JointGroup::hasJoint(const std::string &joint) const
{
  return joint_map.find(joint) != joint_map.end();
}

void planning_models::KinematicModel::JointGroup::computeTransforms(const std::map<std::string, double>& joint_value_map)
{
  const unsigned int js = joint_names.size();
  for (unsigned int i = 0  ; i < js ; ++i) {
    Joint* joint = owner->getJoint(joint_names[i]);
    if(joint == NULL) {
      ROS_WARN_STREAM("Joint group " << name << " has invalid joint " << joint_names[i]); 
    } else {
      joint->updateVariableTransform(joint_value_map);
    }
  }
    
  const unsigned int ls = updated_links.size();
  for (unsigned int i = 0 ; i < ls ; ++i)
    updated_links[i]->computeTransform();
}

void planning_models::KinematicModel::JointGroup::computeTransforms(){
  //assume that joint transforms have been computed
  const unsigned int ls = updated_links.size();
  for (unsigned int i = 0 ; i < ls ; ++i)
    updated_links[i]->computeTransform();
}

std::map<std::string, unsigned int> planning_models::KinematicModel::JointGroup::getMapOrderIndex() const {
  std::map<std::string, unsigned int> ret;
  unsigned int i = 0;
  for(std::map<std::string,Joint*>::const_iterator it = joint_map.begin();
      it != joint_map.end();
      it++) {
    for(std::map<std::string, double>::const_iterator it2 = it->second->stored_joint_values.begin();
        it2 != it->second->stored_joint_values.end();
        it2++) {
      ret[it2->first] = i;
      i++;
    }
  }
  return ret;
}

std::map<std::string, double> planning_models::KinematicModel::JointGroup::getAllJointsValues() const {
  std::map<std::string, double> ret;
  const unsigned int js = joint_names.size();
  for (unsigned int i = 0  ; i < js ; ++i) {
    std::map<std::string, double> j_vals = owner->getJointValues(joint_names[i]);
    ret.insert(j_vals.begin(), j_vals.end());
  }
  return ret;
}

planning_models::KinematicModel::Joint* planning_models::KinematicModel::JointGroup::getJoint(const std::string &name)
{
  std::map<std::string, Joint*>::iterator it = joint_map.find(name);
  if (it == joint_map.end())
  {
    ROS_ERROR("Joint '%s' not found", name.c_str());
    return NULL;
  }
  else
    return it->second;
}

std::map<std::string, double> planning_models::KinematicModel::JointGroup::getJointValues(const std::string joint) const {
  return owner->getJointValues(joint);
}

std::vector<double> planning_models::KinematicModel::JointGroup::getAllJointsValuesVector() const {
  std::vector<double> ret;
  for(std::map<std::string,Joint*>::const_iterator it = joint_map.begin();
      it != joint_map.end();
      it++) {
    for(std::map<std::string, double>::const_iterator it2 = it->second->stored_joint_values.begin();
        it2 != it->second->stored_joint_values.end();
        it2++) {
      ret.push_back(it2->second);
    }
  }
  return ret;
}

void planning_models::KinematicModel::JointGroup::setAllJointsValues(const std::vector<double>& joint_values) {
  std::vector<double>::const_iterator vit = joint_values.begin();
  for(std::map<std::string,Joint*>::iterator it = joint_map.begin();
      it != joint_map.end();
      it++) {
    for(std::map<std::string, double>::iterator it2 = it->second->stored_joint_values.begin();
        it2 != it->second->stored_joint_values.end();
        it2++, vit++) {
      if(vit == joint_values.end()) {
        ROS_WARN_STREAM("Not enough joint values to set " << it2->first);
      } else {
        it2->second = *vit;
      } 
    }
    it->second->updateVariableTransformFromStoredJointValues();
  }
  if(vit != joint_values.end()) {
    ROS_WARN("Too many values in joint values");
  }
}

void planning_models::KinematicModel::JointGroup::defaultState(void)
{
  std::map<std::string, double> default_joint_states;
  
  const unsigned int js = joint_names.size();

  for (unsigned int i = 0  ; i < js ; ++i)
  {
    Joint* joint = owner->getJoint(joint_names[i]);
    if(joint == NULL) {
      ROS_WARN_STREAM("Joint group " << name << " has invalid joint " << joint_names[i]); 
    } else {
      for(std::map<std::string, std::pair<double,double> >::iterator it = joint->joint_state_bounds.begin();
          it != joint->joint_state_bounds.end();
          it++) {
        std::pair<double,double> bounds = it->second;
        if(bounds.first <= 0.0 && bounds.second >= 0.0) {
          default_joint_states[it->first] = 0.0;
        } else {
          default_joint_states[it->first] = (bounds.first+bounds.second)/2.0;
        }
      }
    }
  }
  computeTransforms(default_joint_states);
}
