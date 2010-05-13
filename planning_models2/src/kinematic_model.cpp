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

#include "planning_models/kinematic_model.h"

#include <resource_retriever/retriever.h>
#include <assimp/assimp.hpp>     
#include <assimp/aiScene.h>      
#include <assimp/aiPostProcess.h>

#include <queue>
#include <ros/console.h>
#include <cmath>

/* ------------------------ KinematicModel ------------------------ */

planning_models::KinematicModel::KinematicModel(const KinematicModel &source)
{
    dimension_ = 0;
    model_name_ = source.model_name_;
    root_transform_ = source.root_transform_;
    connect_type_ = source.connect_type_;
    
    if (source.root_)
    {
	root_ = copyRecursive(NULL, source.root_->child_link);
	state_bounds_ = source.state_bounds_;
	
	std::vector<const JointGroup*> groups;
	source.getGroups(groups);
	std::map< std::string, std::vector<std::string> > groupContent;
	for (unsigned int i = 0 ; i < groups.size() ; ++i)
	    groupContent[groups[i]->name] = groups[i]->joint_names;
	buildGroups(groupContent);
	buildConvenientDatastructures();
    }
    else
	root_ = NULL;
}

planning_models::KinematicModel::KinematicModel(const urdf::Model &model, const std::map< std::string, std::vector<std::string> > &groups, WorldJointType wjt)
{    
    dimension_ = 0;
    model_name_ = model.getName();
    root_transform_.setIdentity();
    connect_type_ = wjt;
    
    if (model.getRoot())
    {
	const urdf::Link *root = model.getRoot().get();
	root_ = buildRecursive(NULL, root);
	buildGroups(groups);
	buildConvenientDatastructures();    
    }
    else
    {
	root_ = NULL;
	ROS_WARN("No root link found");
    }
}

planning_models::KinematicModel::~KinematicModel(void)
{
    for (std::map<std::string, JointGroup*>::iterator it = group_map_.begin() ; it != group_map_.end() ; ++it)
	delete it->second;
    if (root_)
	delete root_;
}

const std::string& planning_models::KinematicModel::getName(void) const
{
    return model_name_;
}

planning_models::KinematicModel::WorldJointType planning_models::KinematicModel::getWorldJointType(void) const
{
    return connect_type_;
}

unsigned int planning_models::KinematicModel::getDimension(void) const
{
    return dimension_;
}

const std::vector<double> &planning_models::KinematicModel::getStateBounds(void) const
{
    return state_bounds_;
}

const btTransform& planning_models::KinematicModel::getRootTransform(void) const
{
    return root_transform_;
}

void planning_models::KinematicModel::setRootTransform(const btTransform &transform)
{
    root_transform_ = transform;
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
	if (state_bounds_[2 * i] <= 0.0 && state_bounds_[2 * i + 1] >= 0.0)
	    params[i] = 0.0;
	else
	    params[i] = (state_bounds_[2 * i] + state_bounds_[2 * i + 1]) / 2.0;
    }
    computeTransforms(params);
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
	    ROS_ERROR("Skipping group '%s'", it->first.c_str());
	else
	{
	    ROS_DEBUG("Adding group '%s'", it->first.c_str());
	    group_map_[it->first] = new JointGroup(this, it->first, jointv);
	}
    }
}

planning_models::KinematicModel::Joint* planning_models::KinematicModel::buildRecursive(Link *parent, const urdf::Link *link)
{
    Joint *joint = constructJoint(link->parent_joint.get(), state_bounds_);
    joint->state_index = dimension_;
    joint_map_[joint->name] = joint;
    joint_list_.push_back(joint);
    joint_index_.push_back(dimension_);
    dimension_ += joint->used_params;
    joint->parent_link = parent;
    joint->child_link = constructLink(link);
    if (parent == NULL)
	joint->child_link->joint_origin_transform.setIdentity();
    link_map_[joint->child_link->name] = joint->child_link;
    joint->child_link->parent_joint = joint;
    
    for (unsigned int i = 0 ; i < link->child_links.size() ; ++i)
	joint->child_link->child_joint.push_back(buildRecursive(joint->child_link, link->child_links[i].get()));
    
    return joint;
}

planning_models::KinematicModel::Joint* planning_models::KinematicModel::constructJoint(const urdf::Joint *urdfJoint,
                                                                                        std::vector<double> &bounds)
{
    planning_models::KinematicModel::Joint *result = NULL;
    
    if (urdfJoint)    
    {
	switch (urdfJoint->type)
	{
	case urdf::Joint::REVOLUTE:
	    {
		RevoluteJoint *j = new RevoluteJoint(this);
		if(urdfJoint->safety)
		{
		    j->hi_limit = urdfJoint->safety->soft_upper_limit;
		    j->low_limit = urdfJoint->safety->soft_lower_limit;
		}
		else
		{
		    j->hi_limit = urdfJoint->limits->upper;
		    j->low_limit = urdfJoint->limits->lower;
		}
		j->continuous = false;
		j->axis.setValue(urdfJoint->axis.x, urdfJoint->axis.y, urdfJoint->axis.z);
		bounds.push_back(j->low_limit);
		bounds.push_back(j->hi_limit);
		result = j;
	    }
	    break;
	case urdf::Joint::CONTINUOUS:
	    {
		RevoluteJoint *j = new RevoluteJoint(this);
		j->hi_limit = M_PI;
		j->low_limit = -M_PI;
		j->continuous = true;
		j->axis.setValue(urdfJoint->axis.x, urdfJoint->axis.y, urdfJoint->axis.z);
		bounds.push_back(j->low_limit);
		bounds.push_back(j->hi_limit);
		result = j;
	    }
	    break;
	case urdf::Joint::PRISMATIC:
	    {
		PrismaticJoint *j = new PrismaticJoint(this);
		if(urdfJoint->safety)
		{
		    j->hi_limit = urdfJoint->safety->soft_upper_limit;
		    j->low_limit = urdfJoint->safety->soft_lower_limit;
		}
		else
		{
		    j->hi_limit = urdfJoint->limits->upper;
		    j->low_limit = urdfJoint->limits->lower;
		}
		j->axis.setValue(urdfJoint->axis.x, urdfJoint->axis.y, urdfJoint->axis.z);
		bounds.push_back(j->low_limit);
		bounds.push_back(j->hi_limit);
		result = j;
	    }
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
    }
    else
    {
	// we had no joint, so this must be connecting to the world;
	result = new WorldJoint(this);
	switch (connect_type_)
	{
	case CONNECT_XYZ_QUAT:
	    bounds.insert(bounds.end(), 14, 0.0);
	    break;	    
	case CONNECT_XY_YAW:
	    bounds.insert(bounds.end(), 4, 0.0);	    
	    bounds.push_back(-M_PI);	    
	    bounds.push_back(M_PI);
	    break;
	default:
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

planning_models::KinematicModel::Link* planning_models::KinematicModel::constructLink(const urdf::Link *urdfLink)
{
    ROS_ASSERT(urdfLink);
    
    Link *result = new Link(this);
    result->name = urdfLink->name;
    
    if(urdfLink->collision)
	result->collision_origin_transform = urdfPose2btTransform(urdfLink->collision->origin);
    else
	result->collision_origin_transform.setIdentity();
    
    if (urdfLink->parent_joint.get())
	result->joint_origin_transform = urdfPose2btTransform(urdfLink->parent_joint->parent_to_joint_origin_transform);
    else
	result->joint_origin_transform.setIdentity();
    
    if (urdfLink->collision)
	result->shape = constructShape(urdfLink->collision->geometry.get());
    
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
			
			// And have it read the given file with some postprocessing
			const aiScene* scene = importer.ReadFileFromMemory(reinterpret_cast<void*>(res.data.get()), res.size,
									   aiProcess_Triangulate            |
									   aiProcess_JoinIdenticalVertices  |
									   aiProcess_SortByPType);
			if (scene)
			{
			    if (scene->HasMeshes())
			    {
				if (scene->mNumMeshes > 1)
				    ROS_WARN("More than one mesh specified in resource. Using first one");
				result = shapes::createMeshFromAsset(scene->mMeshes[0]);
			    }
			    else
				ROS_ERROR("There is no mesh specified in the indicated resource");
			}
			
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
    const unsigned int js = joint_list_.size();
    for (unsigned int i = 0  ; i < js ; ++i)
	joint_list_[i]->updateVariableTransform(params + joint_index_[i]);
    
    const unsigned int ls = updated_links_.size();
    for (unsigned int i = 0 ; i < ls ; ++i)
	updated_links_[i]->computeTransform();
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

namespace planning_models
{
    namespace detail
    {
	
	template<typename T>
	T getJoint(const std::map<std::string, KinematicModel::Joint*> &joint_map, const std::string &name)
	{
	    std::map<std::string, KinematicModel::Joint*>::const_iterator it = joint_map.find(name);
	    if (it == joint_map.end())
	    {
		ROS_ERROR("Joint '%s' not found", name.c_str());
		return NULL;
	    }
	    else
		return it->second;	    
	}	
	
	template<typename T>
	T getLink(const std::map<std::string, KinematicModel::Link*> &link_map, const std::string &name)
	{
	    std::map<std::string, KinematicModel::Link*>::const_iterator it = link_map.find(name);
	    if (it == link_map.end())
	    {
		ROS_ERROR("Link '%s' not found", name.c_str());
		return NULL;
	    }
	    else
		return it->second;	    
	}
	
	template<typename T>
	T getGroup(const std::map<std::string, KinematicModel::JointGroup*> &group_map, const std::string &name)
	{
	    std::map<std::string, KinematicModel::JointGroup*>::const_iterator it = group_map.find(name);
	    if (it == group_map.end())
	    {
		ROS_ERROR("Joint group '%s' not found", name.c_str());
		return NULL;
	    }
	    else
		return it->second;	    
	}
	
	template<typename T>
	void getJoints(const std::vector<KinematicModel::Joint*> &joint_list, std::vector<T> &joints)
	{
	    joints.reserve(joint_list.size());
	    for (unsigned int i = 0 ; i < joint_list.size() ; ++i)
		joints.push_back(joint_list[i]);
	}
	
	template<typename T>
	void getLinks(const std::map<std::string, KinematicModel::Link*> &link_map, std::vector<T> &links)
	{
	    links.reserve(link_map.size());
	    for (std::map<std::string, KinematicModel::Link*>::const_iterator it = link_map.begin() ; it != link_map.end() ; ++it)
		links.push_back(it->second);
	}
	
	template<typename T>
	void getGroups(const std::map<std::string, KinematicModel::JointGroup*> &group_map, std::vector<T> &groups)
	{
	    groups.reserve(group_map.size());
	    for (std::map<std::string, KinematicModel::JointGroup*>::const_iterator it = group_map.begin() ; it != group_map.end() ; ++it)
		groups.push_back(it->second);
	}
	
	template<typename T>
	void getChildLinks(const KinematicModel::Link *parent, std::vector<T> &links)
	{
	    std::queue<const KinematicModel::Link*> q;
	    q.push(parent);
	    while (!q.empty())
	    {
		const KinematicModel::Link* t = q.front();
		q.pop();
		
		for (unsigned int i = 0 ; i < t->child_joint.size() ; ++i)
		    if (t->child_joint[i]->child_link)
		    {
			links.push_back(t->child_joint[i]->child_link);
			q.push(t->child_joint[i]->child_link);
		    }
	    }
	}
	
	template<typename T>
	void getChildJoints(const KinematicModel::Joint* parent, std::vector<T> &joints)
	{
	    std::queue<const KinematicModel::Joint*> q;
	    q.push(parent);
	    while (!q.empty())
	    {
		const KinematicModel::Joint* t = q.front();
		q.pop();
		
		if (t->child_link)
		    for (unsigned int i = 0 ; i < t->child_link->child_joint.size() ; ++i)
			if (t->child_link->child_joint[i])
			{
			    joints.push_back(t->child_link->child_joint[i]);
			    q.push(t->child_link->child_joint[i]);
			}
	    }
	}
	
	template<typename T>
	void getAttachedBodies(const std::map<std::string, KinematicModel::Link*> &link_map, std::vector<T> &attached_bodies)
	{
	    for (std::map<std::string, KinematicModel::Link*>::const_iterator it = link_map.begin() ; it != link_map.end() ; ++it)
		attached_bodies.insert(attached_bodies.end(), it->second->attached_bodies.begin(), it->second->attached_bodies.end());
	}
	
    }
}

planning_models::KinematicModel::Joint* planning_models::KinematicModel::getJoint(const std::string &name)
{
    return detail::getJoint<Joint*>(joint_map_, name);
}

const planning_models::KinematicModel::Joint* planning_models::KinematicModel::getJoint(const std::string &name) const
{
    return detail::getJoint<const Joint*>(joint_map_, name);
}

planning_models::KinematicModel::Link* planning_models::KinematicModel::getLink(const std::string &name)
{
    return detail::getLink<Link*>(link_map_, name);
}

const planning_models::KinematicModel::Link* planning_models::KinematicModel::getLink(const std::string &name) const
{
    return detail::getLink<const Link*>(link_map_, name);
}

planning_models::KinematicModel::JointGroup* planning_models::KinematicModel::getGroup(const std::string &name)
{
    return detail::getGroup<JointGroup*>(group_map_, name);
}

const planning_models::KinematicModel::JointGroup* planning_models::KinematicModel::getGroup(const std::string &name) const
{
    return detail::getGroup<const JointGroup*>(group_map_, name);
}

void planning_models::KinematicModel::getGroups(std::vector<JointGroup*> &groups)
{
    detail::getGroups<JointGroup*>(group_map_, groups);
}

void planning_models::KinematicModel::getGroups(std::vector<const JointGroup*> &groups) const
{
    detail::getGroups<const JointGroup*>(group_map_, groups);
}

void planning_models::KinematicModel::getGroupNames(std::vector<std::string> &groups) const
{
    groups.reserve(group_map_.size());
    for (std::map<std::string, JointGroup*>::const_iterator it = group_map_.begin() ; it != group_map_.end() ; ++it)
	groups.push_back(it->second->name);
}

void planning_models::KinematicModel::getLinks(std::vector<Link*> &links)
{
    detail::getLinks<Link*>(link_map_, links);
}

void planning_models::KinematicModel::getLinks(std::vector<const Link*> &links) const
{
    detail::getLinks<const Link*>(link_map_, links);
}

void planning_models::KinematicModel::getLinkNames(std::vector<std::string> &links) const
{
    links.reserve(link_map_.size());
    for (std::map<std::string, Link*>::const_iterator it = link_map_.begin() ; it != link_map_.end() ; ++it)
	links.push_back(it->second->name);
}

void planning_models::KinematicModel::getChildLinks(const Link* parent, std::vector<Link*> &links)
{
    detail::getChildLinks<Link*>(parent, links);
}

void planning_models::KinematicModel::getChildLinks(const Link* parent, std::vector<const Link*> &links) const
{
    detail::getChildLinks<const Link*>(parent, links);
}

void planning_models::KinematicModel::getJoints(std::vector<Joint*> &joints)
{
    detail::getJoints<Joint*>(joint_list_, joints);
}

void planning_models::KinematicModel::getJoints(std::vector<const Joint*> &joints) const
{
    detail::getJoints<const Joint*>(joint_list_, joints);
}

void planning_models::KinematicModel::getJointNames(std::vector<std::string> &joints) const
{
    joints.reserve(joint_list_.size());
    for (unsigned int i = 0 ; i < joint_list_.size() ; ++i)
	joints.push_back(joint_list_[i]->name);
}

void planning_models::KinematicModel::getChildJoints(const Joint* parent, std::vector<Joint*> &joints)
{
    detail::getChildJoints<Joint*>(parent, joints);
}

void planning_models::KinematicModel::getChildJoints(const Joint* parent, std::vector<const Joint*> &joints) const
{
    detail::getChildJoints<const Joint*>(parent, joints);
}

void planning_models::KinematicModel::getAttachedBodies(std::vector<AttachedBody*> &attached_bodies)
{
    detail::getAttachedBodies<AttachedBody*>(link_map_, attached_bodies);
}

void planning_models::KinematicModel::getAttachedBodies(std::vector<const AttachedBody*> &attached_bodies) const
{
    detail::getAttachedBodies<const AttachedBody*>(link_map_, attached_bodies);
}

planning_models::KinematicModel::Joint* planning_models::KinematicModel::copyRecursive(Link *parent, const Link *link)
{
    Joint *joint = copyJoint(link->parent_joint);
    joint->state_index = dimension_;
    joint_map_[joint->name] = joint;
    joint_list_.push_back(joint);
    joint_index_.push_back(dimension_);
    dimension_ += joint->used_params;
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
	newJoint = new FixedJoint(this);
    }
    else
	if (dynamic_cast<const PrismaticJoint*>(joint))
	{
	    PrismaticJoint *pj = new PrismaticJoint(this);
	    const PrismaticJoint *src = static_cast<const PrismaticJoint*>(joint);
	    pj->axis = src->axis;
	    pj->hi_limit = src->hi_limit;
	    pj->low_limit = src->low_limit;
	    newJoint = pj;
	}
        else
	    if (dynamic_cast<const RevoluteJoint*>(joint))
	    {
		RevoluteJoint *pj = new RevoluteJoint(this);
		const RevoluteJoint *src = static_cast<const RevoluteJoint*>(joint);
		pj->axis = src->axis;
		pj->continuous = src->continuous;
		pj->hi_limit = src->hi_limit;
		pj->low_limit = src->low_limit;
		newJoint = pj;
	    }
	    else
		if (dynamic_cast<const WorldJoint*>(joint))
		    newJoint = new WorldJoint(this);
		else
		    ROS_FATAL("Unimplemented type of joint");
    
    if (newJoint)
    {
	newJoint->name = joint->name;
	newJoint->variable_transform = joint->variable_transform;
    }
    
    return newJoint;
}

void planning_models::KinematicModel::printModelInfo(std::ostream &out) const
{
    out << "Model " << model_name_ << " connecting to the world using connection type " << connect_type_ << std::endl;
    
    out << "Complete model state dimension = " << dimension_ << std::endl;
    
    std::ios_base::fmtflags old_flags = out.flags();    
    out.setf(std::ios::fixed, std::ios::floatfield);
    std::streamsize old_prec = out.precision();
    out.precision(5);
    out << "State bounds: ";
    for (unsigned int i = 0 ; i < dimension_ ; ++i)
	out << "[" << state_bounds_[2 * i] << ", " << state_bounds_[2 * i + 1] << "] ";
    out << std::endl;
    out.precision(old_prec);    
    out.flags(old_flags);
    
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
	for (unsigned int j = 0 ; j < g->state_index.size() ; ++j)
	    out << g->state_index[j] << " ";
	out << std::endl;
    }
}

namespace planning_models
{
    
    namespace detail 
    {
	void printTransform(const std::string &st, const btTransform &t, std::ostream &out)
	{
	    out << st << std::endl;
	    const btVector3 &v = t.getOrigin();
	    out << "  origin: " << v.x() << ", " << v.y() << ", " << v.z() << std::endl;
	    const btQuaternion &q = t.getRotation();
	    out << "  quaternion: " << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << std::endl;
	}
    }
}

void planning_models::KinematicModel::printTransforms(std::ostream &out) const
{
    out << "Joint transforms:" << std::endl;
    std::vector<const Joint*> joints;
    getJoints(joints);
    for (unsigned int i = 0 ; i < joints.size() ; ++i)
    {
	detail::printTransform(joints[i]->name, joints[i]->variable_transform, out);
	out << std::endl;	
    }
    out << "Link poses:" << std::endl;
    std::vector<const Link*> links;
    getLinks(links);
    for (unsigned int i = 0 ; i < links.size() ; ++i)
    {
	detail::printTransform(links[i]->name, links[i]->global_collision_body_transform, out);
	out << std::endl;	
    }    
}

/* ------------------------ Joint ------------------------ */

planning_models::KinematicModel::Joint::Joint(KinematicModel *model) : owner(model), used_params(0), state_index(0), parent_link(NULL), child_link(NULL)
{
    variable_transform.setIdentity();
}

planning_models::KinematicModel::Joint::~Joint(void)
{
    if (child_link)
	delete child_link;
}

void planning_models::KinematicModel::FixedJoint::updateVariableTransform(const double *params)
{
    // the joint remains identity
}

void planning_models::KinematicModel::PrismaticJoint::updateVariableTransform(const double *params)
{
    variable_transform.setOrigin(axis * params[0]);
}

void planning_models::KinematicModel::RevoluteJoint::updateVariableTransform(const double *params)
{
    variable_transform.setRotation(btQuaternion(axis, params[0]));
}

const std::string planning_models::KinematicModel::WorldJoint::NAME = "world";

void planning_models::KinematicModel::WorldJoint::updateVariableTransform(const double *params)
{
    switch (type)
    {
    case CONNECT_XY_YAW:
	variable_transform.setOrigin(btVector3(params[0], params[1], 0.0));
	variable_transform.setRotation(btQuaternion(btVector3(0.0, 0.0, 1.0), params[2]));
	break;
    case CONNECT_XYZ_QUAT:
	variable_transform.setOrigin(btVector3(params[0], params[1], params[2]));	
	variable_transform.setRotation(btQuaternion(params[3], params[4], params[5], params[6]));
	break;
    default:
	break;
    }
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
    global_link_transform.mult(parent_joint->parent_link ? parent_joint->parent_link->global_link_transform : owner->getRootTransform(), joint_origin_transform);
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

/* ------------------------ AttachedBody ------------------------ */

planning_models::KinematicModel::AttachedBody::AttachedBody(Link *link, const std::string& nid) : owner(link), id(nid)
{
}

planning_models::KinematicModel::AttachedBody::~AttachedBody(void)
{
    for (unsigned int i = 0 ; i < shapes.size() ; ++i)
	delete shapes[i];
}

void planning_models::KinematicModel::AttachedBody::computeTransform(void)
{
    for (unsigned int i = 0 ; i < global_collision_body_transform.size(); ++i)
	global_collision_body_transform[i] = owner->global_link_transform * attach_trans[i];
}

/* ------------------------ JointGroup ------------------------ */

planning_models::KinematicModel::JointGroup::JointGroup(KinematicModel *model, const std::string& groupName,
                                                        const std::vector<Joint*> &groupJoints) : owner(model)
{
    name = groupName;
    joints = groupJoints;
    joint_names.resize(joints.size());
    joint_index.resize(joints.size());
    dimension = 0;
    
    const std::vector<double> &allBounds = owner->getStateBounds();
    
    for (unsigned int i = 0 ; i < joints.size() ; ++i)
    {
	joint_names[i] = joints[i]->name;
	joint_index[i] = dimension;
	dimension += joints[i]->used_params;
	joint_map[joint_names[i]] = i;
	
	for (unsigned int k = 0 ; k < joints[i]->used_params ; ++k)
	{
	    const unsigned int si = joints[i]->state_index + k;
	    state_index.push_back(si);
	    state_bounds.push_back(allBounds[2 * si]);
	    state_bounds.push_back(allBounds[2 * si + 1]);
	}
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
	    jointRoots.push_back(joints[i]);
    }
    
    for (unsigned int i = 0 ; i < jointRoots.size() ; ++i)
    {
	updated_links.push_back(jointRoots[i]->child_link);
	owner->getChildLinks(jointRoots[i]->child_link, updated_links);
    }
}

planning_models::KinematicModel::JointGroup::~JointGroup(void)
{
}

bool planning_models::KinematicModel::JointGroup::hasJoint(const std::string &joint) const
{
    return joint_map.find(joint) != joint_map.end();
}

int planning_models::KinematicModel::JointGroup::getJointPosition(const std::string &joint) const
{
    std::map<std::string, unsigned int>::const_iterator it = joint_map.find(joint);
    if (it != joint_map.end())
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
	joints[i]->updateVariableTransform(params + joint_index[i]);
    
    const unsigned int ls = updated_links.size();
    for (unsigned int i = 0 ; i < ls ; ++i)
	updated_links[i]->computeTransform();
}

void planning_models::KinematicModel::JointGroup::defaultState(void)
{
    double params[dimension];
    for (unsigned int i = 0 ; i < dimension ; ++i)
    {
	if (state_bounds[2 * i] <= 0.0 && state_bounds[2 * i + 1] >= 0.0)
	    params[i] = 0.0;
	else
	    params[i] = (state_bounds[2 * i] + state_bounds[2 * i + 1]) / 2.0;
    }
    computeTransforms(params);
}
