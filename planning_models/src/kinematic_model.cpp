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
#include <resource_retriever/retriever.h>
#include <queue>
#include <ros/console.h>
#include <cmath>

/* ------------------------ KinematicModel ------------------------ */
planning_models::KinematicModel::KinematicModel(const KinematicModel &source)
{
  dimension_ = 0;
  modelName_ = source.modelName_;
  rootTransform_ = source.rootTransform_;
    
  if (source.root_)
  {
    root_ = copyRecursive(NULL, source.root_->after);
    stateBounds_ = source.stateBounds_;
	
    std::vector<const JointGroup*> groups;
    source.getGroups(groups);
    std::map< std::string, std::vector<std::string> > groupContent;
    for (unsigned int i = 0 ; i < groups.size() ; ++i)
	    groupContent[groups[i]->name] = groups[i]->jointNames;
    buildGroups(groupContent);
    buildConvenientDatastructures();
  }
  else
    root_ = NULL;
}

planning_models::KinematicModel::KinematicModel(const urdf::Model &model, const std::map< std::string, std::vector<std::string> > &groups)
{    
  dimension_ = 0;
  modelName_ = model.getName();
  rootTransform_.setIdentity();
    
  if (model.getRoot())
  {
    const urdf::Link *root = model.getRoot().get();
	
    if (root->name == "world")
    {
	    if (root->child_links.size() > 1)
	    {
        std::stringstream ss;
        for (unsigned int i = 0 ; i < root->child_links.size() ; ++i)
          ss << " " << root->child_links[i]->name;		
        ROS_WARN("More than one link connected to the world:%s. Only considering the first one", ss.str().c_str());
	    }
	    if (root->child_links.empty())
	    {
        root = NULL;
        ROS_ERROR("No links connected to the world");
	    }
	    else
        root = root->child_links[0].get();
    }
	
    if (root)
    {
	    root_ = buildRecursive(NULL, root);
	    buildGroups(groups);
	    buildConvenientDatastructures();
    }
  }
  else
  {
    root_ = NULL;
    ROS_WARN("No root link found");
  }
}

planning_models::KinematicModel::~KinematicModel(void)
{
  for (std::map<std::string, JointGroup*>::iterator it = groupMap_.begin() ; it != groupMap_.end() ; ++it)
    delete it->second;
  if (root_)
    delete root_;
}

const std::string& planning_models::KinematicModel::getName(void) const
{
  return modelName_;
}

unsigned int planning_models::KinematicModel::getDimension(void) const
{
  return dimension_;
}

const std::vector<double> &planning_models::KinematicModel::getStateBounds(void) const
{
  return stateBounds_;
}

const btTransform& planning_models::KinematicModel::getRootTransform(void) const
{
  return rootTransform_;
}

void planning_models::KinematicModel::setRootTransform(const btTransform &transform)
{
  rootTransform_ = transform;
}

const std::vector<std::string> &planning_models::KinematicModel::getFloatingJoints(void) const
{
  return floatingJoints_;
}

const std::vector<std::string> &planning_models::KinematicModel::getPlanarJoints(void) const
{
  return planarJoints_;
}

const std::vector<std::string> &planning_models::KinematicModel::getFixedJoints(void) const
{
  return fixedJoints_;
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
  if (dimension_ <= 0)
    return;
    
  double params[dimension_];
  for (unsigned int i = 0 ; i < dimension_ ; ++i)
  {
    if (stateBounds_[2 * i] <= 0.0 && stateBounds_[2 * i + 1] >= 0.0)
	    params[i] = 0.0;
    else
	    params[i] = (stateBounds_[2 * i] + stateBounds_[2 * i + 1]) / 2.0;
  }
  computeTransforms(params);
}

void planning_models::KinematicModel::buildConvenientDatastructures(void)
{
  if (root_)
  {
    std::queue<Link*> links;
    links.push(root_->after);
    while (!links.empty())
    {
	    Link *link = links.front();
	    links.pop();
	    updatedLinks_.push_back(link);
	    for (unsigned int i = 0 ; i < link->after.size() ; ++i)
        links.push(link->after[i]->after);
    }
  }
}

void planning_models::KinematicModel::buildGroups(const std::map< std::string, std::vector<std::string> > &groups)
{
  for (std::map< std::string, std::vector<std::string> >::const_iterator it = groups.begin() ; it != groups.end() ; ++it)
  {
    std::vector<Joint*> jointv;
    for (unsigned int i = 0 ; i < it->second.size() ; ++i)
    {
	    std::map<std::string, Joint*>::iterator p = jointMap_.find(it->second[i]);
	    if (p == jointMap_.end())
	    {
        ROS_ERROR("Unknown joint '%s'. Not adding to group '%s'", it->second[i].c_str(), it->first.c_str());
        jointv.clear();
        break;
	    }
	    else
        jointv.push_back(p->second);
    }
    // hack
    
    if (jointv.empty() && it->first == "base")
    {
	jointv.push_back(jointMap_["base_joint"]);
    }
    
	
    if (jointv.empty())
	    ROS_ERROR("Skipping group '%s'", it->first.c_str());
    else
    {
	    ROS_ERROR("Adding group '%s'", it->first.c_str());
	    groupMap_[it->first] = new JointGroup(this, it->first, jointv);
    }
  }
}

planning_models::KinematicModel::Joint* planning_models::KinematicModel::buildRecursive(Link *parent, const urdf::Link *link)
{
  urdf::Joint *temp = link->parent_joint.get() ? NULL : new urdf::Joint();
  Joint *joint = NULL;
  if (temp)
  {
    temp->type = urdf::Joint::PLANAR;
    temp->name = "base_joint";
    joint = constructJoint(temp, stateBounds_);
    delete temp;
  }
  else
    joint = constructJoint(link->parent_joint.get(), stateBounds_);
    
  joint->stateIndex = dimension_;
  jointMap_[joint->name] = joint;
  jointList_.push_back(joint);
  jointIndex_.push_back(dimension_);
  dimension_ += joint->usedParams;
  joint->before = parent;
  joint->after = constructLink(link);
  if (parent == NULL)
    joint->after->constTrans.setIdentity();
  linkMap_[joint->after->name] = joint->after;
  joint->after->before = joint;
    
  if (dynamic_cast<FloatingJoint*>(joint))
    floatingJoints_.push_back(joint->name);
  if (dynamic_cast<PlanarJoint*>(joint))
    planarJoints_.push_back(joint->name);
  if (dynamic_cast<FixedJoint*>(joint))
    fixedJoints_.push_back(joint->name);
    
  for (unsigned int i = 0 ; i < link->child_links.size() ; ++i)
    joint->after->after.push_back(buildRecursive(joint->after, link->child_links[i].get()));
    
  return joint;
}

planning_models::KinematicModel::Joint* planning_models::KinematicModel::constructJoint(const urdf::Joint *urdfJoint,
                                                                                        std::vector<double> &bounds)
{
  planning_models::KinematicModel::Joint *result = NULL;
    
  ROS_ASSERT(urdfJoint);
    
  switch (urdfJoint->type)
  {
  case urdf::Joint::REVOLUTE:
    {
	    RevoluteJoint *j = new RevoluteJoint(this);
      if(urdfJoint->safety)
      {
        j->hiLimit = urdfJoint->safety->soft_upper_limit;
        j->lowLimit = urdfJoint->safety->soft_lower_limit;
      }
      else
      {
        j->hiLimit = urdfJoint->limits->upper;
        j->lowLimit = urdfJoint->limits->lower;
      }
	    j->continuous = false;
	    j->axis.setValue(urdfJoint->axis.x, urdfJoint->axis.y, urdfJoint->axis.z);
	    bounds.push_back(j->lowLimit);
	    bounds.push_back(j->hiLimit);
	    result = j;
    }
    break;
  case urdf::Joint::CONTINUOUS:
    {
	    RevoluteJoint *j = new RevoluteJoint(this);
	    j->hiLimit = M_PI;
	    j->lowLimit = -M_PI;
	    j->continuous = true;
	    j->axis.setValue(urdfJoint->axis.x, urdfJoint->axis.y, urdfJoint->axis.z);
	    bounds.push_back(j->lowLimit);
	    bounds.push_back(j->hiLimit);
	    result = j;
    }
    break;
  case urdf::Joint::PRISMATIC:
    {
	    PrismaticJoint *j = new PrismaticJoint(this);
      if(urdfJoint->safety)
      {
        j->hiLimit = urdfJoint->safety->soft_upper_limit;
        j->lowLimit = urdfJoint->safety->soft_lower_limit;
      }
      else
      {
        j->hiLimit = urdfJoint->limits->upper;
        j->lowLimit = urdfJoint->limits->lower;
      }
	    j->axis.setValue(urdfJoint->axis.x, urdfJoint->axis.y, urdfJoint->axis.z);
	    bounds.push_back(j->lowLimit);
	    bounds.push_back(j->hiLimit);
	    result = j;
    }
    break;
  case urdf::Joint::FLOATING:
    result = new FloatingJoint(this);
    bounds.insert(bounds.end(), 14, 0.0);
    break;
  case urdf::Joint::PLANAR:
    result = new PlanarJoint(this);
    bounds.insert(bounds.end(), 4, 0.0);
    bounds.push_back(-M_PI);
    bounds.push_back(M_PI);
    break;
  case urdf::Joint::FIXED:
    result = new FixedJoint(this);
    break;
  default:
    ROS_ERROR("Unknown joint type: %d", (int)urdfJoint->type);
    break;
  }
    
  if (result)
    result->name = urdfJoint->name;
    
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

planning_models::KinematicModel::Link* planning_models::KinematicModel::constructLink(const urdf::Link *urdfLink)
{
  ROS_ASSERT(urdfLink);

  Link *result = new Link(this);
  result->name = urdfLink->name;

  if(urdfLink->collision)
    result->constGeomTrans = urdfPose2btTransform(urdfLink->collision->origin);
  else
    result->constGeomTrans.setIdentity();

  if (urdfLink->parent_joint.get())
    result->constTrans = urdfPose2btTransform(urdfLink->parent_joint->parent_to_joint_origin_transform);
  else
    result->constTrans.setIdentity();
    
  if(urdfLink->collision)
    result->shape = constructShape(urdfLink->collision->geometry.get());
  else
  {
    shapes::Shape *tmp_shape = NULL;
    tmp_shape = new shapes::Sphere(0.0001);
    result->shape = tmp_shape;   
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
            result = shapes::createMeshFromBinaryStlData(reinterpret_cast<char*>(res.data.get()), res.size);
            if (result == NULL)
              ROS_ERROR("Failed to load mesh '%s'", mesh->filename.c_str());
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

void planning_models::KinematicModel::computeTransforms(const double *params)
{
  const unsigned int js = jointList_.size();
  for (unsigned int i = 0  ; i < js ; ++i)
    jointList_[i]->updateVariableTransform(params + jointIndex_[i]);
    
  const unsigned int ls = updatedLinks_.size();
  for (unsigned int i = 0 ; i < ls ; ++i)
    updatedLinks_[i]->computeTransform();
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
  return jointMap_.find(name) != jointMap_.end();
}

bool planning_models::KinematicModel::hasLink(const std::string &name) const
{
  return linkMap_.find(name) != linkMap_.end();
}

bool planning_models::KinematicModel::hasGroup(const std::string &name) const
{
  return groupMap_.find(name) != groupMap_.end();
}

const planning_models::KinematicModel::Joint* planning_models::KinematicModel::getJoint(const std::string &name) const
{
  std::map<std::string, Joint*>::const_iterator it = jointMap_.find(name);
  if (it == jointMap_.end())
  {
    ROS_ERROR("Joint '%s' not found", name.c_str());
    return NULL;
  }
  else
    return it->second;
}

planning_models::KinematicModel::Joint* planning_models::KinematicModel::getJoint(const std::string &name)
{
  std::map<std::string, Joint*>::iterator it = jointMap_.find(name);
  if (it == jointMap_.end())
  {
    ROS_ERROR("Joint '%s' not found", name.c_str());
    return NULL;
  }
  else
    return it->second;
}

const planning_models::KinematicModel::Link* planning_models::KinematicModel::getLink(const std::string &name) const
{
  std::map<std::string, Link*>::const_iterator it = linkMap_.find(name);
  if (it == linkMap_.end())
  {
    ROS_ERROR("Link '%s' not found", name.c_str());
    return NULL;
  }
  else
    return it->second;
}

planning_models::KinematicModel::Link* planning_models::KinematicModel::getLink(const std::string &name)
{
  std::map<std::string, Link*>::iterator it = linkMap_.find(name);
  if (it == linkMap_.end())
  {
    ROS_ERROR("Link '%s' not found", name.c_str());
    return NULL;
  }
  else
    return it->second;
}

const planning_models::KinematicModel::JointGroup* planning_models::KinematicModel::getGroup(const std::string &name) const
{
  std::map<std::string, JointGroup*>::const_iterator it = groupMap_.find(name);
  if (it == groupMap_.end())
  {
    ROS_ERROR("Joint group '%s' not found", name.c_str());
    return NULL;
  }
  else
    return it->second;
}

planning_models::KinematicModel::JointGroup* planning_models::KinematicModel::getGroup(const std::string &name)
{
  std::map<std::string, JointGroup*>::iterator it = groupMap_.find(name);
  if (it == groupMap_.end())
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
  groups.reserve(groupMap_.size());
  for (std::map<std::string, JointGroup*>::const_iterator it = groupMap_.begin() ; it != groupMap_.end() ; ++it)
    groups.push_back(it->second);
}

void planning_models::KinematicModel::getGroupNames(std::vector<std::string> &groups) const
{
  groups.clear();
  groups.reserve(groupMap_.size());
  for (std::map<std::string, JointGroup*>::const_iterator it = groupMap_.begin() ; it != groupMap_.end() ; ++it)
    groups.push_back(it->second->name);
}

void planning_models::KinematicModel::getLinks(std::vector<const Link*> &links) const
{
  links.clear();
  links.reserve(linkMap_.size());
  for (std::map<std::string, Link*>::const_iterator it = linkMap_.begin() ; it != linkMap_.end() ; ++it)
    links.push_back(it->second);
}

void planning_models::KinematicModel::getLinkNames(std::vector<std::string> &links) const
{
  links.clear();
  links.reserve(linkMap_.size());
  for (std::map<std::string, Link*>::const_iterator it = linkMap_.begin() ; it != linkMap_.end() ; ++it)
    links.push_back(it->second->name);
}

void planning_models::KinematicModel::getJoints(std::vector<const Joint*> &joints) const
{
  joints.clear();
  joints.reserve(jointList_.size());
  for (unsigned int i = 0 ; i < jointList_.size() ; ++i)
    joints.push_back(jointList_[i]);
}

void planning_models::KinematicModel::getJointNames(std::vector<std::string> &joints) const
{
  joints.clear();
  joints.reserve(jointList_.size());
  for (unsigned int i = 0 ; i < jointList_.size() ; ++i)
    joints.push_back(jointList_[i]->name);
}

planning_models::KinematicModel::Joint* planning_models::KinematicModel::copyRecursive(Link *parent, const Link *link)
{
  Joint *joint = copyJoint(link->before);
  joint->stateIndex = dimension_;
  jointMap_[joint->name] = joint;
  jointList_.push_back(joint);
  jointIndex_.push_back(dimension_);
  dimension_ += joint->usedParams;
  joint->before = parent;
  joint->after = copyLink(link);
  linkMap_[joint->after->name] = joint->after;
  joint->after->before = joint;
    
  if (dynamic_cast<FloatingJoint*>(joint))
    floatingJoints_.push_back(joint->name);
  if (dynamic_cast<PlanarJoint*>(joint))
    planarJoints_.push_back(joint->name);
    
  for (unsigned int i = 0 ; i < link->after.size() ; ++i)
    joint->after->after.push_back(copyRecursive(joint->after, link->after[i]->after));
    
  return joint;
}

planning_models::KinematicModel::Link* planning_models::KinematicModel::copyLink(const Link *link)
{
  Link *newLink = new Link(this);
    
  newLink->name = link->name;
  newLink->constTrans = link->constTrans;
  newLink->constGeomTrans = link->constGeomTrans;
  newLink->globalTransFwd = link->globalTransFwd;
  newLink->globalTrans = link->globalTrans;
  newLink->shape = shapes::cloneShape(link->shape);
    
  for (unsigned int i = 0 ; i < link->attachedBodies.size() ; ++i)
  {
    AttachedBody *ab = new AttachedBody(newLink, link->attachedBodies[i]->id);
    for(unsigned int j = 0; j < ab->shapes.size(); j++) {
      ab->shapes.push_back(shapes::cloneShape(link->attachedBodies[i]->shapes[j]));
    }
    ab->attachTrans = link->attachedBodies[i]->attachTrans;
    ab->globalTrans = link->attachedBodies[i]->globalTrans;
    ab->touchLinks = link->attachedBodies[i]->touchLinks;
    newLink->attachedBodies.push_back(ab);
  }
    
  return newLink;
}

planning_models::KinematicModel::Joint* planning_models::KinematicModel::copyJoint(const Joint *joint)
{
  Joint *newJoint = NULL;

  if (dynamic_cast<const FixedJoint*>(joint))
  {
    newJoint = new FixedJoint(this);
  }
  else
    if (dynamic_cast<const FloatingJoint*>(joint))
    {
      newJoint = new FloatingJoint(this);
    }
    else
      if (dynamic_cast<const PlanarJoint*>(joint))
      {
        newJoint = new PlanarJoint(this);
      }
      else
        if (dynamic_cast<const PrismaticJoint*>(joint))
        {
          PrismaticJoint *pj = new PrismaticJoint(this);
          const PrismaticJoint *src = static_cast<const PrismaticJoint*>(joint);
          pj->axis = src->axis;
          pj->hiLimit = src->hiLimit;
          pj->lowLimit = src->lowLimit;
          newJoint = pj;
        }
        else
          if (dynamic_cast<const RevoluteJoint*>(joint))
          {
            RevoluteJoint *pj = new RevoluteJoint(this);
            const RevoluteJoint *src = static_cast<const RevoluteJoint*>(joint);
            pj->axis = src->axis;
            pj->continuous = src->continuous;
            pj->hiLimit = src->hiLimit;
            pj->lowLimit = src->lowLimit;
            newJoint = pj;
          }
          else
            ROS_FATAL("Unimplemented type of joint");
    
  if (newJoint)
  {
    newJoint->name = joint->name;
    newJoint->varTrans = joint->varTrans;
  }
    
  return newJoint;
}

void planning_models::KinematicModel::printModelInfo(std::ostream &out) const
{
  out << "Complete model state dimension = " << dimension_ << std::endl;
    
  std::ios_base::fmtflags old_flags = out.flags();    
  out.setf(std::ios::fixed, std::ios::floatfield);
  std::streamsize old_prec = out.precision();
  out.precision(5);
  out << "State bounds: ";
  for (unsigned int i = 0 ; i < dimension_ ; ++i)
    out << "[" << stateBounds_[2 * i] << ", " << stateBounds_[2 * i + 1] << "] ";
  out << std::endl;
  out.precision(old_prec);    
  out.flags(old_flags);
        
  out << "Floating joints : ";
  for (unsigned int i = 0 ; i < floatingJoints_.size() ; ++i)
    out << floatingJoints_[i] << " ";
  out << std::endl;
    
  out << "Planar joints : ";
  for (unsigned int i = 0 ; i < planarJoints_.size() ; ++i)
    out << planarJoints_[i] << " ";
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
    out << "Group " << l[i] << " has " << g->jointRoots.size() << " roots: ";
    for (unsigned int j = 0 ; j < g->jointRoots.size() ; ++j)
	    out << g->jointRoots[j]->name << " ";
    out << std::endl;
    out << "The state components for this group are: ";
    for (unsigned int j = 0 ; j < g->stateIndex.size() ; ++j)
	    out << g->stateIndex[j] << " ";
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
    printTransform(joints[i]->name, joints[i]->varTrans, out);
    out << std::endl;	
  }
  out << "Link poses:" << std::endl;
  std::vector<const Link*> links;
  getLinks(links);
  for (unsigned int i = 0 ; i < links.size() ; ++i)
  {
    printTransform(links[i]->name, links[i]->globalTrans, out);
    out << std::endl;	
  }    
}

/* ------------------------ Joint ------------------------ */

planning_models::KinematicModel::Joint::Joint(KinematicModel *model) : owner(model), usedParams(0), stateIndex(0), before(NULL), after(NULL)
{
  varTrans.setIdentity();
}

planning_models::KinematicModel::Joint::~Joint(void)
{
  if (after)
    delete after;
}

void planning_models::KinematicModel::FixedJoint::updateVariableTransform(const double *params)
{
  // the joint remains identity
}


void planning_models::KinematicModel::PlanarJoint::updateVariableTransform(const double *params)
{
  varTrans.setOrigin(btVector3(params[0], params[1], 0.0));
  varTrans.setRotation(btQuaternion(btVector3(0.0, 0.0, 1.0), params[2]));
}

void planning_models::KinematicModel::FloatingJoint::updateVariableTransform(const double *params)
{
  varTrans.setOrigin(btVector3(params[0], params[1], params[2]));
  varTrans.setRotation(btQuaternion(params[3], params[4], params[5], params[6]));
}

void planning_models::KinematicModel::PrismaticJoint::updateVariableTransform(const double *params)
{
  varTrans.setOrigin(axis * params[0]);
}

void planning_models::KinematicModel::RevoluteJoint::updateVariableTransform(const double *params)
{
  varTrans.setRotation(btQuaternion(axis, params[0]));
}

/* ------------------------ Link ------------------------ */

planning_models::KinematicModel::Link::Link(KinematicModel *model) : owner(model), before(NULL), shape(NULL)
{
  constTrans.setIdentity();
  constGeomTrans.setIdentity();
  globalTransFwd.setIdentity();
  globalTrans.setIdentity();		
}

planning_models::KinematicModel::Link::~Link(void)
{
  if (shape)
    delete shape;
  for (unsigned int i = 0 ; i < after.size() ; ++i)
    delete after[i];
  for (unsigned int i = 0 ; i < attachedBodies.size() ; ++i)
    delete attachedBodies[i];
}

void planning_models::KinematicModel::Link::setTransform(const btTransform &transform)
{
  globalTransFwd = transform;
  globalTrans.mult(globalTransFwd, constGeomTrans);
}

void planning_models::KinematicModel::Link::computeTransform(void)
{
  globalTransFwd.mult(before->before ? before->before->globalTransFwd : owner->getRootTransform(), constTrans);
  globalTransFwd *= before->varTrans;    
  globalTrans.mult(globalTransFwd, constGeomTrans);

  for (unsigned int i = 0 ; i < attachedBodies.size() ; ++i)
    attachedBodies[i]->computeTransform();
}

void planning_models::KinematicModel::Link::updateTransformsRecursive(void)
{
  computeTransform();
  ROS_DEBUG("Update transforms for %s",name.c_str());
  for(unsigned int i=0; i < after.size(); i++)
  {
    if(after[i]->after)
      after[i]->after->updateTransformsRecursive();
  }
}

void planning_models::KinematicModel::Link::getAllChildLinksRecursive(std::vector<std::string> &link_names)
{
  link_names.push_back(name);
  for(unsigned int i=0; i < after.size(); i++)
  {
    if(after[i]->after)
      after[i]->after->getAllChildLinksRecursive(link_names);
  }
}


/* ------------------------ AttachedBody ------------------------ */

planning_models::KinematicModel::AttachedBody::AttachedBody(Link *link, std::string nid) : owner(link), id(nid)
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
  for(unsigned int i = 0; i < globalTrans.size(); i++) {
    globalTrans[i] = owner->globalTransFwd * attachTrans[i];
  }
}

/* ------------------------ JointGroup ------------------------ */

planning_models::KinematicModel::JointGroup::JointGroup(KinematicModel *model, const std::string& groupName,
                                                        const std::vector<Joint*> &groupJoints) : owner(model)
{
  name = groupName;
  joints = groupJoints;
  jointNames.resize(joints.size());
  jointIndex.resize(joints.size());
  dimension = 0;
    
  const std::vector<double> &allBounds = owner->getStateBounds();
    
  for (unsigned int i = 0 ; i < joints.size() ; ++i)
  {
    jointNames[i] = joints[i]->name;
    jointIndex[i] = dimension;
    dimension += joints[i]->usedParams;
    jointMap[jointNames[i]] = i;

    for (unsigned int k = 0 ; k < joints[i]->usedParams ; ++k)
    {
	    const unsigned int si = joints[i]->stateIndex + k;
	    stateIndex.push_back(si);
	    stateBounds.push_back(allBounds[2 * si]);
	    stateBounds.push_back(allBounds[2 * si + 1]);
    }
  }
    
  for (unsigned int i = 0 ; i < joints.size() ; ++i)
  {
    bool found = false;
    Joint *joint = joints[i];
    while (joint->before)
    {
	    joint = joint->before->before;
	    if (hasJoint(joint->name))
	    {
        found = true;
        break;
	    }
    }
	
    if (!found)
	    jointRoots.push_back(joints[i]);
  }

  for (unsigned int i = 0 ; i < jointRoots.size() ; ++i)
  {
    std::queue<Link*> links;
    links.push(jointRoots[i]->after);
	
    while (!links.empty())
    {
	    Link *link = links.front();
	    links.pop();
	    updatedLinks.push_back(link);
	    for (unsigned int i = 0 ; i < link->after.size() ; ++i)
        links.push(link->after[i]->after);
    }
  }
}

planning_models::KinematicModel::JointGroup::~JointGroup(void)
{
}

bool planning_models::KinematicModel::JointGroup::hasJoint(const std::string &joint) const
{
  return jointMap.find(joint) != jointMap.end();
}

int planning_models::KinematicModel::JointGroup::getJointPosition(const std::string &joint) const
{
  std::map<std::string, unsigned int>::const_iterator it = jointMap.find(joint);
  if (it != jointMap.end())
    return it->second;
  else
  {
    ROS_ERROR("Joint '%s' is not part of group '%s'", joint.c_str(), name.c_str());
    return -1;
  }
}

void planning_models::KinematicModel::JointGroup::computeTransforms(const double *params)
{
  const unsigned int js = joints.size();
  for (unsigned int i = 0  ; i < js ; ++i)
    joints[i]->updateVariableTransform(params + jointIndex[i]);

  const unsigned int ls = updatedLinks.size();
  for (unsigned int i = 0 ; i < ls ; ++i)
    updatedLinks[i]->computeTransform();
}

void planning_models::KinematicModel::JointGroup::defaultState(void)
{
    double params[dimension];
    for (unsigned int i = 0 ; i < dimension ; ++i)
    {
	if (stateBounds[2 * i] <= 0.0 && stateBounds[2 * i + 1] >= 0.0)
	    params[i] = 0.0;
	else
	    params[i] = (stateBounds[2 * i] + stateBounds[2 * i + 1]) / 2.0;
    }
    computeTransforms(params);
}
