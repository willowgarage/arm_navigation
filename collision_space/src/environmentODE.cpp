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

#include "collision_space/environmentODE.h"
#include <geometric_shapes/shape_operations.h>
#include <ros/console.h>
#include <cassert>
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <map>

#include <boost/thread.hpp>
static int          ODEInitCount = 0;
static boost::mutex ODEInitCountLock;

static std::map<boost::thread::id, int> ODEThreadMap;
static boost::mutex                     ODEThreadMapLock;

//all the threading stuff is necessary to check collision from different threads

static const int MAX_ODE_CONTACTS = 128;
static const int TEST_FOR_ALLOWED_NUM = 1;

static const std::string CONTACT_ONLY_NAME="contact_only";

collision_space::EnvironmentModelODE::EnvironmentModelODE(void) : EnvironmentModel()
{
  ODEInitCountLock.lock();
  if (ODEInitCount == 0)
  {
    int res = dInitODE2(0);
    ROS_DEBUG_STREAM("Calling ODE Init res " << res);
  }
  ODEInitCount++;
  ODEInitCountLock.unlock();
    
  checkThreadInit();

  ROS_DEBUG("Initializing ODE");

  model_geom_.env_space = dSweepAndPruneSpaceCreate(0, dSAP_AXES_XZY);
  model_geom_.self_space = dSweepAndPruneSpaceCreate(0, dSAP_AXES_XZY);
  
  previous_set_robot_model_ = false;
}

collision_space::EnvironmentModelODE::~EnvironmentModelODE(void)
{
  freeMemory();
  ODEInitCountLock.lock();
  ODEInitCount--;
  boost::thread::id id = boost::this_thread::get_id();
  ODEThreadMapLock.lock();
  ODEThreadMap.erase(id);
  ODEThreadMapLock.unlock();
  if (ODEInitCount == 0)
  {
    ODEThreadMap.clear();
    ROS_DEBUG("Closing ODE");
    dCloseODE();
  }
  ODEInitCountLock.unlock();
}

void collision_space::EnvironmentModelODE::freeMemory(void)
{ 
  for (unsigned int j = 0 ; j < model_geom_.link_geom.size() ; ++j)
    delete model_geom_.link_geom[j];
  model_geom_.link_geom.clear();
  if (model_geom_.env_space)
    dSpaceDestroy(model_geom_.env_space);
  if (model_geom_.self_space)
    dSpaceDestroy(model_geom_.self_space);
  for (std::map<std::string, CollisionNamespace*>::iterator it = coll_namespaces_.begin() ; it != coll_namespaces_.end() ; ++it) {
    delete it->second;
  }
  model_geom_.storage.clear();
  coll_namespaces_.clear();
}

void collision_space::EnvironmentModelODE::checkThreadInit(void) const
{
  boost::thread::id id = boost::this_thread::get_id();
  ODEThreadMapLock.lock();
  if (ODEThreadMap.find(id) == ODEThreadMap.end())
  {
    ODEThreadMap[id] = 1;
    ROS_DEBUG("Initializing new thread (%d total)", (int)ODEThreadMap.size());
    int res = dAllocateODEDataForThread(dAllocateMaskAll);
    ROS_DEBUG_STREAM("Init says " << res);
  } 
  ODEThreadMapLock.unlock();
}

void collision_space::EnvironmentModelODE::setRobotModel(const planning_models::KinematicModel* model, 
                                                         const AllowedCollisionMatrix& allowed_collision_matrix,
                                                         const std::map<std::string, double>& link_padding_map,
                                                         double default_padding,
                                                         double scale) 
{
  collision_space::EnvironmentModel::setRobotModel(model, allowed_collision_matrix, link_padding_map, default_padding, scale);
  if(previous_set_robot_model_) {
    for (unsigned int j = 0 ; j < model_geom_.link_geom.size() ; ++j)
      delete model_geom_.link_geom[j];
    model_geom_.link_geom.clear();
    dSpaceDestroy(model_geom_.env_space);
    dSpaceDestroy(model_geom_.self_space);
    model_geom_.env_space = dSweepAndPruneSpaceCreate(0, dSAP_AXES_XZY);
    model_geom_.self_space = dSweepAndPruneSpaceCreate(0, dSAP_AXES_XZY);
    attached_bodies_in_collision_matrix_.clear();
    geom_lookup_map_.clear();
  }
  createODERobotModel();
  previous_set_robot_model_ = true;
}

void collision_space::EnvironmentModelODE::getAttachedBodyPoses(std::map<std::string, std::vector<tf::Transform> >& pose_map) const
{
  pose_map.clear();

  const unsigned int n = model_geom_.link_geom.size();    
  for (unsigned int i = 0 ; i < n ; ++i)
  {
    LinkGeom *lg = model_geom_.link_geom[i];
    
    /* create new set of attached bodies */	
    const unsigned int nab = lg->att_bodies.size();
    std::vector<tf::Transform> nbt;
    for (unsigned int j = 0 ; j < nab ; ++j)
    {
      for(unsigned int k = 0; k < lg->att_bodies[j]->geom.size(); k++) {
        const dReal *pos = dGeomGetPosition(lg->att_bodies[j]->geom[k]);
        dQuaternion q;
        dGeomGetQuaternion(lg->att_bodies[j]->geom[k], q);
        //note that ODE puts w first (w,x,y,z)
        nbt.push_back(tf::Transform(tf::Quaternion(q[1], q[2], q[3], q[0]), tf::Vector3(pos[0], pos[1], pos[2])));
      }
      pose_map[lg->att_bodies[j]->att->getName()] = nbt;
    }
  }
}

void collision_space::EnvironmentModelODE::createODERobotModel()
{
  for (unsigned int i = 0 ; i < robot_model_->getLinkModels().size() ; ++i)
  {
    /* skip this link if we have no geometry or if the link
       name is not specified as enabled for collision
       checking */
    const planning_models::KinematicModel::LinkModel *link = robot_model_->getLinkModels()[i];
    if (!link || !link->getLinkShape())
      continue;
	
    LinkGeom *lg = new LinkGeom(model_geom_.storage);
    lg->link = link;
    if(!default_collision_matrix_.getEntryIndex(link->getName(), lg->index)) {
      ROS_WARN_STREAM("Link " << link->getName() << " not in provided collision matrix");
    } 
    double padd = default_robot_padding_;
    if(default_link_padding_map_.find(link->getName()) != default_link_padding_map_.end()) {
      padd = default_link_padding_map_.find(link->getName())->second;
    }
    ROS_DEBUG_STREAM("Link " << link->getName() << " padding " << padd);

    dGeomID unpadd_g = createODEGeom(model_geom_.self_space, model_geom_.storage, link->getLinkShape(), 1.0, 0.0);
    assert(unpadd_g);
    lg->geom.push_back(unpadd_g);
    geom_lookup_map_[unpadd_g] = std::pair<std::string, BodyType>(link->getName(), LINK);

    dGeomID padd_g = createODEGeom(model_geom_.env_space, model_geom_.storage, link->getLinkShape(), robot_scale_, padd);
    assert(padd_g);
    lg->padded_geom.push_back(padd_g);
    geom_lookup_map_[padd_g] = std::pair<std::string, BodyType>(link->getName(), LINK);
    const std::vector<planning_models::KinematicModel::AttachedBodyModel*>& attached_bodies = link->getAttachedBodyModels();
    for (unsigned int j = 0 ; j < attached_bodies.size() ; ++j) {
      padd = default_robot_padding_;
      if(default_link_padding_map_.find(attached_bodies[j]->getName()) != default_link_padding_map_.end()) {
        padd = default_link_padding_map_.find(attached_bodies[j]->getName())->second;
      } else if (default_link_padding_map_.find("attached") != default_link_padding_map_.end()) {
        padd = default_link_padding_map_.find("attached")->second;
      }
      addAttachedBody(lg, attached_bodies[j], padd);
    }
    model_geom_.link_geom.push_back(lg);
  } 
}

dGeomID collision_space::EnvironmentModelODE::createODEGeom(dSpaceID space, ODEStorage &storage, const shapes::StaticShape *shape)
{
  dGeomID g = NULL;
  switch (shape->type)
  {
  case shapes::PLANE:
    {
      const shapes::Plane *p = static_cast<const shapes::Plane*>(shape);
      g = dCreatePlane(space, p->a, p->b, p->c, p->d);
    }
    break;
  default:
    break;
  }
  return g;
}

dGeomID collision_space::EnvironmentModelODE::createODEGeom(dSpaceID space, ODEStorage &storage, const shapes::Shape *shape, double scale, double padding)
{
  dGeomID g = NULL;
  switch (shape->type)
  {
  case shapes::SPHERE:
    {
      g = dCreateSphere(space, static_cast<const shapes::Sphere*>(shape)->radius * scale + padding);
    }
    break;
  case shapes::BOX:
    {
      const double *size = static_cast<const shapes::Box*>(shape)->size;
      g = dCreateBox(space, size[0] * scale + padding * 2.0, size[1] * scale + padding * 2.0, size[2] * scale + padding * 2.0);
    }	
    break;
  case shapes::CYLINDER:
    {
      g = dCreateCylinder(space, static_cast<const shapes::Cylinder*>(shape)->radius * scale + padding,
                          static_cast<const shapes::Cylinder*>(shape)->length * scale + padding * 2.0);
    }
    break;
  case shapes::MESH:
    {
      const shapes::Mesh *mesh = static_cast<const shapes::Mesh*>(shape);
      if (mesh->vertexCount > 0 && mesh->triangleCount > 0)
      {		
        // copy indices for ODE
        int icount = mesh->triangleCount * 3;
        dTriIndex *indices = new dTriIndex[icount];
        for (int i = 0 ; i < icount ; ++i)
          indices[i] = mesh->triangles[i];
		
        // copt vertices for ODE
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
        for (unsigned int i = 0 ; i < mesh->vertexCount ; ++i)
        {
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
		
        dTriMeshDataID data = dGeomTriMeshDataCreate();
        dGeomTriMeshDataBuildDouble(data, vertices, sizeof(double) * 3, mesh->vertexCount, indices, icount, sizeof(dTriIndex) * 3);
        g = dCreateTriMesh(space, data, NULL, NULL, NULL);
        ODEStorage::Element& e = storage.meshes[g];
        e.vertices = vertices;
        e.indices = indices;
        e.data = data;
        e.n_vertices = mesh->vertexCount;
        e.n_indices = icount;
      }
    }
	
  default:
    break;
  }
  return g;
}

void collision_space::EnvironmentModelODE::updateGeom(dGeomID geom,  const tf::Transform &pose) const
{
  tf::Vector3 pos = pose.getOrigin();
  dGeomSetPosition(geom, pos.getX(), pos.getY(), pos.getZ());
  tf::Quaternion quat = pose.getRotation();
  dQuaternion q; 
  q[0] = quat.getW(); q[1] = quat.getX(); q[2] = quat.getY(); q[3] = quat.getZ();
  dGeomSetQuaternion(geom, q);
}

void collision_space::EnvironmentModelODE::updateAttachedBodies()
{
  updateAttachedBodies(default_link_padding_map_);
}

void collision_space::EnvironmentModelODE::updateAttachedBodies(const std::map<std::string, double>& link_padding_map)
{
  //getting rid of all entries associated with the current attached bodies
  for(std::map<std::string, bool>::iterator it = attached_bodies_in_collision_matrix_.begin();
      it != attached_bodies_in_collision_matrix_.end();
      it++) {
    if(!default_collision_matrix_.removeEntry(it->first)) {
      ROS_WARN_STREAM("No entry in default collision matrix for attached body " << it->first <<
                      " when there really should be.");
    }
  }
  attached_bodies_in_collision_matrix_.clear();
  for (unsigned int i = 0 ; i < model_geom_.link_geom.size() ; ++i) {
    LinkGeom *lg = model_geom_.link_geom[i];

    for(unsigned int j = 0; j < lg->att_bodies.size(); j++) {
      for(unsigned int k = 0; k < lg->att_bodies[j]->geom.size(); k++) {
        geom_lookup_map_.erase(lg->att_bodies[j]->geom[k]);
      }
      for(unsigned int k = 0; k < lg->att_bodies[j]->padded_geom.size(); k++) {
        geom_lookup_map_.erase(lg->att_bodies[j]->padded_geom[k]);
      }
    }
    lg->deleteAttachedBodies();

    /* create new set of attached bodies */
    const std::vector<planning_models::KinematicModel::AttachedBodyModel*>& attached_bodies = lg->link->getAttachedBodyModels();
    for (unsigned int j = 0 ; j < attached_bodies.size(); ++j) {
      double padd = default_robot_padding_;
      if(link_padding_map.find(attached_bodies[j]->getName()) != link_padding_map.end()) {
        padd = link_padding_map.find(attached_bodies[j]->getName())->second;
      } else if (link_padding_map.find("attached") != link_padding_map.end()) {
        padd = link_padding_map.find("attached")->second;
      }
      ROS_DEBUG_STREAM("Updating attached body " << attached_bodies[j]->getName());      
      addAttachedBody(lg, attached_bodies[j], padd);
    }
  }
}

void collision_space::EnvironmentModelODE::addAttachedBody(LinkGeom* lg, 
                                                           const planning_models::KinematicModel::AttachedBodyModel* attm,
                                                           double padd) {

  AttGeom* attg = new AttGeom(model_geom_.storage);
  attg->att = attm;

  if(!default_collision_matrix_.addEntry(attm->getName(), false)) {
    ROS_WARN_STREAM("Must already have an entry in allowed collision matrix for " << attm->getName());
  } else {
    ROS_DEBUG_STREAM("Adding entry for " << attm->getName());
  } 
  attached_bodies_in_collision_matrix_[attm->getName()] = true;
  default_collision_matrix_.getEntryIndex(attm->getName(), attg->index);
  //setting touch links
  for(unsigned int i = 0; i < attm->getTouchLinks().size(); i++) {
    if(default_collision_matrix_.hasEntry(attm->getTouchLinks()[i])) {
      if(!default_collision_matrix_.changeEntry(attm->getName(), attm->getTouchLinks()[i], true)) {
        ROS_WARN_STREAM("No entry in allowed collision matrix for " << attm->getName() << " and " << attm->getTouchLinks()[i]);
      } else {
        ROS_DEBUG_STREAM("Adding touch link for " << attm->getName() << " and " << attm->getTouchLinks()[i]);
      }
    }
  }
  for(unsigned int i = 0; i < attm->getShapes().size(); i++) {
    dGeomID ga = createODEGeom(model_geom_.self_space, model_geom_.storage, attm->getShapes()[i], 1.0, 0.0);
    assert(ga);
    attg->geom.push_back(ga);
    geom_lookup_map_[ga] = std::pair<std::string, BodyType>(attm->getName(), ATTACHED);    

    dGeomID padd_ga = createODEGeom(model_geom_.env_space, model_geom_.storage, attm->getShapes()[i], robot_scale_, padd);
    assert(padd_ga);
    attg->padded_geom.push_back(padd_ga);
    geom_lookup_map_[padd_ga] = std::pair<std::string, BodyType>(attm->getName(), ATTACHED);    
  }
  lg->att_bodies.push_back(attg);
}

void collision_space::EnvironmentModelODE::setAttachedBodiesLinkPadding() {
  for (unsigned int i = 0 ; i < model_geom_.link_geom.size() ; ++i) {
    LinkGeom *lg = model_geom_.link_geom[i];
    
    const std::vector<planning_models::KinematicModel::AttachedBodyModel*>& attached_bodies = lg->link->getAttachedBodyModels();
    for (unsigned int j = 0 ; j < attached_bodies.size(); ++j) {
      double new_padd = -1.0;
      if(altered_link_padding_map_.find(attached_bodies[j]->getName()) != altered_link_padding_map_.end()) {
        new_padd = altered_link_padding_map_.find(attached_bodies[j]->getName())->second;
      } else if (altered_link_padding_map_.find("attached") != altered_link_padding_map_.end()) {
        new_padd = altered_link_padding_map_.find("attached")->second;
      }
      if(new_padd != -1.0) {
        for(unsigned int k = 0; k < attached_bodies[j]->getShapes().size(); k++) {
          geom_lookup_map_.erase(lg->att_bodies[j]->padded_geom[k]);
          dGeomDestroy(lg->att_bodies[j]->padded_geom[k]);
          model_geom_.storage.remove(lg->att_bodies[j]->padded_geom[k]);
          dGeomID padd_ga = createODEGeom(model_geom_.env_space, model_geom_.storage, attached_bodies[j]->getShapes()[k], robot_scale_, new_padd);
          assert(padd_ga);
          lg->att_bodies[j]->padded_geom[k] = padd_ga;
          geom_lookup_map_[padd_ga] = std::pair<std::string, BodyType>(attached_bodies[j]->getName(), ATTACHED);
        }
      }
    }
  }
}

void collision_space::EnvironmentModelODE::revertAttachedBodiesLinkPadding() {
  for (unsigned int i = 0 ; i < model_geom_.link_geom.size() ; ++i) {
    LinkGeom *lg = model_geom_.link_geom[i];
    
    const std::vector<planning_models::KinematicModel::AttachedBodyModel*>& attached_bodies = lg->link->getAttachedBodyModels();
    for (unsigned int j = 0 ; j < attached_bodies.size(); ++j) {
      double new_padd = -1.0;
      if(altered_link_padding_map_.find(attached_bodies[j]->getName()) != altered_link_padding_map_.end()) {
        new_padd = default_link_padding_map_.find(attached_bodies[j]->getName())->second;
      } else if (altered_link_padding_map_.find("attached") != altered_link_padding_map_.end()) {
        new_padd = default_link_padding_map_.find("attached")->second;
      }
      if(new_padd != -1.0) {
        for(unsigned int k = 0; k < attached_bodies[j]->getShapes().size(); k++) {
          geom_lookup_map_.erase(lg->att_bodies[j]->padded_geom[k]);
          dGeomDestroy(lg->att_bodies[j]->padded_geom[k]);
          model_geom_.storage.remove(lg->att_bodies[j]->padded_geom[k]);
          dGeomID padd_ga = createODEGeom(model_geom_.env_space, model_geom_.storage, attached_bodies[j]->getShapes()[k], robot_scale_, new_padd);
          assert(padd_ga);
          lg->att_bodies[j]->padded_geom[k] = padd_ga;
          geom_lookup_map_[padd_ga] = std::pair<std::string, BodyType>(attached_bodies[j]->getName(), ATTACHED);
        }
      }
    }
  }
}

void collision_space::EnvironmentModelODE::updateRobotModel(const planning_models::KinematicState* state)
{ 
  const unsigned int n = model_geom_.link_geom.size();
    
  for (unsigned int i = 0 ; i < n ; ++i) {
    const planning_models::KinematicState::LinkState* link_state = state->getLinkState(model_geom_.link_geom[i]->link->getName());
    if(link_state == NULL) {
      ROS_WARN_STREAM("No link state for link " << model_geom_.link_geom[i]->link->getName());
      continue;
    }
    updateGeom(model_geom_.link_geom[i]->geom[0], link_state->getGlobalCollisionBodyTransform());
    updateGeom(model_geom_.link_geom[i]->padded_geom[0], link_state->getGlobalCollisionBodyTransform());
    const std::vector<planning_models::KinematicState::AttachedBodyState*>& attached_bodies = link_state->getAttachedBodyStateVector();
    for (unsigned int j = 0 ; j < attached_bodies.size(); ++j) {
      for(unsigned int k = 0; k < attached_bodies[j]->getGlobalCollisionBodyTransforms().size(); k++) {
        updateGeom(model_geom_.link_geom[i]->att_bodies[j]->geom[k], attached_bodies[j]->getGlobalCollisionBodyTransforms()[k]);
        updateGeom(model_geom_.link_geom[i]->att_bodies[j]->padded_geom[k], attached_bodies[j]->getGlobalCollisionBodyTransforms()[k]);
      }
    }
  }    
}

void collision_space::EnvironmentModelODE::setAlteredLinkPadding(const std::map<std::string, double>& new_link_padding) {
  
  //updating altered map
  collision_space::EnvironmentModel::setAlteredLinkPadding(new_link_padding);

  for(unsigned int i = 0; i < model_geom_.link_geom.size(); i++) {

    LinkGeom *lg = model_geom_.link_geom[i];

    if(altered_link_padding_map_.find(lg->link->getName()) != altered_link_padding_map_.end()) {
      double new_padding = altered_link_padding_map_.find(lg->link->getName())->second;
      const planning_models::KinematicModel::LinkModel *link = lg->link;
      if (!link || !link->getLinkShape()) {
        ROS_WARN_STREAM("Can't get kinematic model for link " << link->getName() << " to make new padding");
        continue;
      }
      ROS_DEBUG_STREAM("Setting padding for link " << lg->link->getName() << " from " 
                       << default_link_padding_map_[lg->link->getName()] 
                       << " to " << new_padding);
      //otherwise we clear out the data associated with the old one
      for (unsigned int j = 0 ; j < lg->padded_geom.size() ; ++j) {
        geom_lookup_map_.erase(lg->padded_geom[j]);
        dGeomDestroy(lg->padded_geom[j]);
        model_geom_.storage.remove(lg->padded_geom[j]);
      }
      lg->padded_geom.clear();
      dGeomID g = createODEGeom(model_geom_.env_space, model_geom_.storage, link->getLinkShape(), robot_scale_, new_padding);
      assert(g);
      lg->padded_geom.push_back(g);
      geom_lookup_map_[g] = std::pair<std::string, BodyType>(link->getName(), LINK);
    }
  }
  //this does all the work
  setAttachedBodiesLinkPadding();  
}

void collision_space::EnvironmentModelODE::revertAlteredLinkPadding() {
  for(unsigned int i = 0; i < model_geom_.link_geom.size(); i++) {
    
    LinkGeom *lg = model_geom_.link_geom[i];

    if(altered_link_padding_map_.find(lg->link->getName()) != altered_link_padding_map_.end()) {
      double old_padding = default_link_padding_map_.find(lg->link->getName())->second;
      const planning_models::KinematicModel::LinkModel *link = lg->link;
      if (!link || !link->getLinkShape()) {
        ROS_WARN_STREAM("Can't get kinematic model for link " << link->getName() << " to revert to old padding");
        continue;
      }
      //otherwise we clear out the data associated with the old one
      for (unsigned int j = 0 ; j < lg->padded_geom.size() ; ++j) {
        geom_lookup_map_.erase(lg->padded_geom[j]);
        dGeomDestroy(lg->padded_geom[j]);
        model_geom_.storage.remove(lg->padded_geom[j]);
      }
      ROS_DEBUG_STREAM("Reverting padding for link " << lg->link->getName() << " from " << altered_link_padding_map_[lg->link->getName()]
                      << " to " << old_padding);
      lg->padded_geom.clear();
      dGeomID g = createODEGeom(model_geom_.env_space, model_geom_.storage, link->getLinkShape(), robot_scale_, old_padding);
      assert(g);
      dGeomSetData(g, reinterpret_cast<void*>(lg));
      lg->padded_geom.push_back(g);
      geom_lookup_map_[g] = std::pair<std::string, BodyType>(link->getName(), LINK);
    }
  }
  revertAttachedBodiesLinkPadding();
  
  //clears altered map
  collision_space::EnvironmentModel::revertAlteredLinkPadding();
} 

bool collision_space::EnvironmentModelODE::ODECollide2::empty(void) const
{
  return geoms_x.empty();
}

void collision_space::EnvironmentModelODE::ODECollide2::registerSpace(dSpaceID space)
{
  int n = dSpaceGetNumGeoms(space);
  for (int i = 0 ; i < n ; ++i)
    registerGeom(dSpaceGetGeom(space, i));
}

void collision_space::EnvironmentModelODE::ODECollide2::unregisterGeom(dGeomID geom)
{
  setup();
    
  Geom g;
  g.id = geom;
  dGeomGetAABB(geom, g.aabb);
    
  Geom *found = NULL;
    
  std::vector<Geom*>::iterator posStart1 = std::lower_bound(geoms_x.begin(), geoms_x.end(), &g, SortByXTest());
  std::vector<Geom*>::iterator posEnd1   = std::upper_bound(posStart1, geoms_x.end(), &g, SortByXTest());
  while (posStart1 < posEnd1)
  {
    if ((*posStart1)->id == geom)
    {
      found = *posStart1;
      geoms_x.erase(posStart1);
      break;
    }
    ++posStart1;
  }

  std::vector<Geom*>::iterator posStart2 = std::lower_bound(geoms_y.begin(), geoms_y.end(), &g, SortByYTest());
  std::vector<Geom*>::iterator posEnd2   = std::upper_bound(posStart2, geoms_y.end(), &g, SortByYTest());
  while (posStart2 < posEnd2)
  {
    if ((*posStart2)->id == geom)
    {
      assert(found == *posStart2);
      geoms_y.erase(posStart2);
      break;
    }
    ++posStart2;
  }
    
  std::vector<Geom*>::iterator posStart3 = std::lower_bound(geoms_z.begin(), geoms_z.end(), &g, SortByZTest());
  std::vector<Geom*>::iterator posEnd3   = std::upper_bound(posStart3, geoms_z.end(), &g, SortByZTest());
  while (posStart3 < posEnd3)
  {
    if ((*posStart3)->id == geom)
    {
      assert(found == *posStart3);
      geoms_z.erase(posStart3);
      break;
    }
    ++posStart3;
  }
    
  assert(found);
  delete found;
}

void collision_space::EnvironmentModelODE::ODECollide2::registerGeom(dGeomID geom)
{
  Geom *g = new Geom();
  g->id = geom;
  dGeomGetAABB(geom, g->aabb);
  geoms_x.push_back(g);
  geoms_y.push_back(g);
  geoms_z.push_back(g);
  setup_ = false;
}
	
void collision_space::EnvironmentModelODE::ODECollide2::clear(void)
{
  for (unsigned int i = 0 ; i < geoms_x.size() ; ++i)
    delete geoms_x[i];
  geoms_x.clear();
  geoms_y.clear();
  geoms_z.clear();
  setup_ = false;
}

void collision_space::EnvironmentModelODE::ODECollide2::setup(void)
{
  if (!setup_)
  {
    std::sort(geoms_x.begin(), geoms_x.end(), SortByXLow());
    std::sort(geoms_y.begin(), geoms_y.end(), SortByYLow());
    std::sort(geoms_z.begin(), geoms_z.end(), SortByZLow());
    setup_ = true;
  }	    
}

void collision_space::EnvironmentModelODE::ODECollide2::getGeoms(std::vector<dGeomID> &geoms) const
{
  geoms.resize(geoms_x.size());
  for (unsigned int i = 0 ; i < geoms.size() ; ++i)
    geoms[i] = geoms_x[i]->id;
}

void collision_space::EnvironmentModelODE::ODECollide2::checkColl(std::vector<Geom*>::const_iterator posStart, std::vector<Geom*>::const_iterator posEnd,
                                                                  Geom *g, void *data, dNearCallback *nearCallback) const
{
  /* posStart now identifies the first geom which has an AABB
     that could overlap the AABB of geom on the X axis. posEnd
     identifies the first one that cannot overlap. */
    
  while (posStart < posEnd)
  {
    /* if the boxes are not disjoint along Y, Z, check further */
    if (!((*posStart)->aabb[2] > g->aabb[3] ||
          (*posStart)->aabb[3] < g->aabb[2] ||
          (*posStart)->aabb[4] > g->aabb[5] ||
          (*posStart)->aabb[5] < g->aabb[4]))
      dSpaceCollide2(g->id, (*posStart)->id, data, nearCallback);
    posStart++;
  }
}

void collision_space::EnvironmentModelODE::ODECollide2::collide(dGeomID geom, void *data, dNearCallback *nearCallback) const
{
  static const int CUTOFF = 100;

  assert(setup_);

  Geom g;
  g.id = geom;
  dGeomGetAABB(geom, g.aabb);
    
  std::vector<Geom*>::const_iterator posStart1 = std::lower_bound(geoms_x.begin(), geoms_x.end(), &g, SortByXTest());
  if (posStart1 != geoms_x.end())
  {
    std::vector<Geom*>::const_iterator posEnd1 = std::upper_bound(posStart1, geoms_x.end(), &g, SortByXTest());
    int                                d1      = posEnd1 - posStart1;
	
    /* Doing two binary searches on the sorted-by-y array takes
       log(n) time, which should be around 12 steps. Each step
       should be just a few ops, so a cut-off like 100 is
       appropriate. */
    if (d1 > CUTOFF)
    {
      std::vector<Geom*>::const_iterator posStart2 = std::lower_bound(geoms_y.begin(), geoms_y.end(), &g, SortByYTest());
      if (posStart2 != geoms_y.end())
      {
        std::vector<Geom*>::const_iterator posEnd2 = std::upper_bound(posStart2, geoms_y.end(), &g, SortByYTest());
        int                                d2      = posEnd2 - posStart2;
		
        if (d2 > CUTOFF)
        {
          std::vector<Geom*>::const_iterator posStart3 = std::lower_bound(geoms_z.begin(), geoms_z.end(), &g, SortByZTest());
          if (posStart3 != geoms_z.end())
          {
            std::vector<Geom*>::const_iterator posEnd3 = std::upper_bound(posStart3, geoms_z.end(), &g, SortByZTest());
            int                                d3      = posEnd3 - posStart3;
            if (d3 > CUTOFF)
            {
              if (d3 <= d2 && d3 <= d1)
                checkColl(posStart3, posEnd3, &g, data, nearCallback);
              else
                if (d2 <= d3 && d2 <= d1)
                  checkColl(posStart2, posEnd2, &g, data, nearCallback);
                else
                  checkColl(posStart1, posEnd1, &g, data, nearCallback);
            }
            else
              checkColl(posStart3, posEnd3, &g, data, nearCallback);   
          }
        }
        else
          checkColl(posStart2, posEnd2, &g, data, nearCallback);   
      }
    }
    else 
      checkColl(posStart1, posEnd1, &g, data, nearCallback);
  }
}

namespace collision_space
{

void nearCallbackFn(void *data, dGeomID o1, dGeomID o2)
{
  EnvironmentModelODE::CollisionData *cdata = reinterpret_cast<EnvironmentModelODE::CollisionData*>(data);
  
  if (cdata->done) {
    return;
  }
  
  //first figure out what check is happening
  bool check_in_allowed_collision_matrix = true;
  
  std::string object_name;

  std::map<dGeomID, std::pair<std::string, EnvironmentModelODE::BodyType> >::const_iterator it1 = cdata->geom_lookup_map->find(o1);
  std::map<dGeomID, std::pair<std::string, EnvironmentModelODE::BodyType> >::const_iterator it2 = cdata->geom_lookup_map->find(o2);
  
  if(it1 != cdata->geom_lookup_map->end()) {
    cdata->body_name_1 = it1->second.first;
    cdata->body_type_1 = it1->second.second;
  } else {
    for(std::map<std::string, dSpaceID>::const_iterator it = cdata->dspace_lookup_map->begin();
        it != cdata->dspace_lookup_map->end();
        it++) {
      if(dSpaceQuery(it->second, o1)) {
        object_name = it->first;
        break;
      }
    }
    if(object_name == "") {
      ROS_WARN_STREAM("Object does not have entry in dspace map");
    }
    cdata->body_name_1 = object_name;
    cdata->body_type_1 = EnvironmentModelODE::OBJECT;
    check_in_allowed_collision_matrix = false;
  }

  if(it2 != cdata->geom_lookup_map->end()) {
    cdata->body_name_2 = it2->second.first;
    cdata->body_type_2 = it2->second.second;
  } else {
    for(std::map<std::string, dSpaceID>::const_iterator it = cdata->dspace_lookup_map->begin();
        it != cdata->dspace_lookup_map->end();
        it++) {
      if(dSpaceQuery(it->second, o2)) {
        object_name = it->first;
        break;
      }
    }
    if(object_name == "") {
      ROS_WARN_STREAM("Object does not have entry in dspace map");
    }
    cdata->body_name_2 = object_name;
    cdata->body_type_2 = EnvironmentModelODE::OBJECT;
    check_in_allowed_collision_matrix = false;
  }

  //determine whether or not this collision is allowed in the self_collision matrix
  if (cdata->allowed_collision_matrix && check_in_allowed_collision_matrix) {
    bool allowed;
    if(!cdata->allowed_collision_matrix->getAllowedCollision(cdata->body_name_1, cdata->body_name_2, allowed)) {
      ROS_WARN_STREAM("No entry in allowed collision matrix for " << cdata->body_name_1 << " and " << cdata->body_name_2);
      return;
    }
    if(allowed) {
      ROS_DEBUG_STREAM("Will not test for collision between " << cdata->body_name_1 << " and " << cdata->body_name_2);
      return;
    } else {
      ROS_DEBUG_STREAM("Will test for collision between " << cdata->body_name_1 << " and " << cdata->body_name_2);
    }
  }

  //do the actual collision check to get the desired number of contacts
  int num_contacts = 1;
  if(cdata->contacts) {
    num_contacts = std::min(MAX_ODE_CONTACTS, (int) cdata->max_contacts_pair);
  } 
  if(cdata->allowed) {
    num_contacts = std::max(num_contacts, TEST_FOR_ALLOWED_NUM);
  }
  num_contacts = std::max(num_contacts, 1);
  
  ROS_DEBUG_STREAM("Testing " << cdata->body_name_1
                  << " and " << cdata->body_name_2 << " contact size " << num_contacts);

  dContactGeom contactGeoms[num_contacts];
  int numc = dCollide(o1, o2, num_contacts,
                      &(contactGeoms[0]), sizeof(dContactGeom));
  
  //no collisions, return
  if(!numc) 
    return;

  if(!cdata->contacts && !cdata->allowed) {
    //we don't care about contact information, so just set to true if there's been collision
    ROS_DEBUG_STREAM_NAMED(CONTACT_ONLY_NAME, "Detected collision between " << cdata->body_name_1 << " and " << cdata->body_name_2);
    cdata->collides = true;      
    cdata->done = true;
  } else {
    unsigned int num_not_allowed = 0;
    if(numc != num_contacts) {
      ROS_INFO_STREAM("Asked for " << num_contacts << " but only got " << numc);
    }
    for (int i = 0 ; i < numc ; ++i) {

      ROS_DEBUG_STREAM("Contact at " << contactGeoms[i].pos[0] << " " 
                       << contactGeoms[i].pos[1] << " " << contactGeoms[i].pos[2]);    
      const dReal *pos1 = dGeomGetPosition(o1);
      dQuaternion quat1, quat2;
      dGeomGetQuaternion(o1, quat1);
      const dReal *pos2 = dGeomGetPosition(o2);
      dGeomGetQuaternion(o2, quat2);

      ROS_DEBUG_STREAM("Body pos 1 " << pos1[0] << " " << pos1[1] << " " << pos1[2]);
      ROS_DEBUG_STREAM("Body quat 1 " << quat1[1] << " " << quat1[2] << " " << quat1[3] << " " << quat1[0]);
      ROS_DEBUG_STREAM("Body pos 2 " << pos2[0] << " " << pos2[1] << " " << pos2[2]);
      ROS_DEBUG_STREAM("Body quat 2 " << quat2[1] << " " << quat2[2] << " " << quat2[3] << " " << quat2[0]);

      tf::Vector3 pos(contactGeoms[i].pos[0], contactGeoms[i].pos[1], contactGeoms[i].pos[2]);
      
      //figure out whether the contact is allowed
      //allowed contacts only allowed with objects for now
      bool allowed = false;
      if(cdata->allowed) { 
        EnvironmentModel::AllowedContactMap::const_iterator it1 = cdata->allowed->find(cdata->body_name_1);
        if(it1 != cdata->allowed->end()) {
          std::map<std::string, std::vector<EnvironmentModel::AllowedContact> >::const_iterator it2 = it1->second.find(cdata->body_name_2);
          if(it2 != it1->second.end()) {
            ROS_DEBUG_STREAM("Testing allowed contact for " << cdata->body_name_1 << " and " << cdata->body_name_2 << " num " << i);
            ROS_DEBUG_STREAM("Contact at " << contactGeoms[i].pos[0] << " " 
                            << contactGeoms[i].pos[1] << " " << contactGeoms[i].pos[2]);      
            
            const std::vector<EnvironmentModel::AllowedContact>& av = it2->second;
            for(unsigned int j = 0; j < av.size(); j++) {
              if(av[j].bound->containsPoint(pos)) {
                if(av[j].depth >= fabs(contactGeoms[i].depth)) {
                  allowed = true;
                  ROS_DEBUG_STREAM("Contact allowed by allowed collision region");
                  break;
                } else {
                  ROS_DEBUG_STREAM("Depth check failing " << av[j].depth << " detected " << contactGeoms[i].depth);
                }
              }
            }
          }
        }
      }
      if(!allowed) {

        cdata->collides = true;
        num_not_allowed++;

        ROS_DEBUG_STREAM_NAMED(CONTACT_ONLY_NAME, "Detected collision between " << cdata->body_name_1 << " and " << cdata->body_name_2);

        if(cdata->contacts != NULL) {
          if(num_not_allowed <= cdata->max_contacts_pair) {
            collision_space::EnvironmentModelODE::Contact add;
            
            add.pos = pos;
            
            add.normal.setX(contactGeoms[i].normal[0]);
            add.normal.setY(contactGeoms[i].normal[1]);
            add.normal.setZ(contactGeoms[i].normal[2]);
            
            add.depth = contactGeoms[i].depth;
            
            add.body_name_1 = cdata->body_name_1;
            add.body_name_2 = cdata->body_name_2;
            add.body_type_1 = cdata->body_type_1;
            add.body_type_2 = cdata->body_type_2;
            
            cdata->contacts->push_back(add);
            if(cdata->contacts->size() >= cdata->max_contacts_total) {
              cdata->done = true;
            }
          }
        } else {
          cdata->done = true;    
        }
      }
    }
  }
}
}

bool collision_space::EnvironmentModelODE::getCollisionContacts(std::vector<Contact> &contacts, unsigned int max_total, unsigned int max_per_pair) const
{
  contacts.clear();
  CollisionData cdata;
  cdata.allowed_collision_matrix = &getCurrentAllowedCollisionMatrix();
  cdata.geom_lookup_map = &geom_lookup_map_;
  cdata.dspace_lookup_map = &dspace_lookup_map_;
  cdata.contacts = &contacts;
  cdata.max_contacts_total = max_total;
  cdata.max_contacts_pair = max_per_pair;
  if (!allowed_contacts_.empty())
    cdata.allowed = &allowed_contact_map_;
  contacts.clear();
  checkThreadInit();
  testCollision(&cdata);
  return cdata.collides;
}

bool collision_space::EnvironmentModelODE::getAllCollisionContacts(std::vector<Contact> &contacts, unsigned int num_contacts_per_pair) const
{
  contacts.clear();
  CollisionData cdata;
  cdata.geom_lookup_map = &geom_lookup_map_;
  cdata.dspace_lookup_map = &dspace_lookup_map_;
  cdata.allowed_collision_matrix = &getCurrentAllowedCollisionMatrix();
  cdata.contacts = &contacts;
  cdata.max_contacts_total = UINT_MAX;
  cdata.max_contacts_pair = num_contacts_per_pair;
  if (!allowed_contacts_.empty())
    cdata.allowed = &allowed_contact_map_;
  contacts.clear();
  checkThreadInit();
  testCollision(&cdata);
  return cdata.collides;
}

bool collision_space::EnvironmentModelODE::isCollision(void) const
{
  CollisionData cdata;
  cdata.allowed_collision_matrix = &getCurrentAllowedCollisionMatrix();
  cdata.geom_lookup_map = &geom_lookup_map_;
  cdata.dspace_lookup_map = &dspace_lookup_map_;
  if (!allowed_contacts_.empty()) {
    cdata.allowed = &allowed_contact_map_;
    ROS_DEBUG_STREAM("Got contacts size " << cdata.allowed->size());
  } else {
    ROS_DEBUG_STREAM("No allowed contacts");
  }
  checkThreadInit();
  testCollision(&cdata);
  return cdata.collides;
}

bool collision_space::EnvironmentModelODE::isSelfCollision(void) const
{
  CollisionData cdata; 
  cdata.geom_lookup_map = &geom_lookup_map_;
  cdata.dspace_lookup_map = &dspace_lookup_map_;
  cdata.allowed_collision_matrix = &getCurrentAllowedCollisionMatrix();
  if (!allowed_contacts_.empty())
    cdata.allowed = &allowed_contact_map_;
  checkThreadInit();
  testSelfCollision(&cdata);
  return cdata.collides;
}

bool collision_space::EnvironmentModelODE::isEnvironmentCollision(void) const
{
  CollisionData cdata; 
  cdata.geom_lookup_map = &geom_lookup_map_;
  cdata.dspace_lookup_map = &dspace_lookup_map_;
  cdata.allowed_collision_matrix = &getCurrentAllowedCollisionMatrix();
  if (!allowed_contacts_.empty())
    cdata.allowed = &allowed_contact_map_;
  checkThreadInit();
  testEnvironmentCollision(&cdata);
  return cdata.collides;
}

bool collision_space::EnvironmentModelODE::isObjectRobotCollision(const std::string& object_name) const {
  std::map<std::string, CollisionNamespace *>::const_iterator it =
    coll_namespaces_.find(object_name);
  if (it == coll_namespaces_.end()) {
    ROS_WARN("Attempt to check collision for %s and robot, but no such object exists", object_name.c_str());
    return false;
  }
  CollisionData cdata; 
  cdata.geom_lookup_map = &geom_lookup_map_;
  cdata.dspace_lookup_map = &dspace_lookup_map_;
  cdata.allowed_collision_matrix = &getCurrentAllowedCollisionMatrix();
  if (!allowed_contacts_.empty())
    cdata.allowed = &allowed_contact_map_;
  checkThreadInit();
  testObjectCollision(it->second, &cdata);
  return cdata.collides;
}

bool collision_space::EnvironmentModelODE::isObjectObjectCollision(const std::string& object1_name, 
                                                                   const std::string& object2_name) const
{
  CollisionData cdata; 
  cdata.geom_lookup_map = &geom_lookup_map_;
  cdata.dspace_lookup_map = &dspace_lookup_map_;
  cdata.allowed_collision_matrix = &getCurrentAllowedCollisionMatrix();
  if (!allowed_contacts_.empty())
    cdata.allowed = &allowed_contact_map_;
  checkThreadInit();
  testObjectObjectCollision(&cdata, object1_name, object2_name);
  return cdata.collides;
}

bool collision_space::EnvironmentModelODE::isObjectInEnvironmentCollision(const std::string& object_name) const
{
  CollisionData cdata; 
  cdata.geom_lookup_map = &geom_lookup_map_;
  cdata.dspace_lookup_map = &dspace_lookup_map_;
  cdata.allowed_collision_matrix = &getCurrentAllowedCollisionMatrix();
  if (!allowed_contacts_.empty())
    cdata.allowed = &allowed_contact_map_;
  checkThreadInit();
  testObjectEnvironmentCollision(&cdata, object_name);
  return cdata.collides;
}

bool collision_space::EnvironmentModelODE::getAllObjectEnvironmentCollisionContacts(const std::string& object_name, 
                                                                                    std::vector<Contact> &contacts,
                                                                                    unsigned int num_contacts_per_pair) const {
  contacts.clear();
  CollisionData cdata;
  cdata.geom_lookup_map = &geom_lookup_map_;
  cdata.dspace_lookup_map = &dspace_lookup_map_;
  cdata.allowed_collision_matrix = &getCurrentAllowedCollisionMatrix();
  cdata.contacts = &contacts;
  cdata.max_contacts_total = UINT_MAX;
  cdata.max_contacts_pair = num_contacts_per_pair;
  if (!allowed_contacts_.empty())
    cdata.allowed = &allowed_contact_map_;
  checkThreadInit();
  testObjectEnvironmentCollision(&cdata, object_name);
  return cdata.collides;
}

void collision_space::EnvironmentModelODE::testObjectEnvironmentCollision(CollisionData *cdata, const std::string& object_name) const {
  /* check collision with other ode bodies until done*/
  for (std::map<std::string, CollisionNamespace*>::const_iterator it = coll_namespaces_.begin() ; it != coll_namespaces_.end() && !cdata->done ; ++it) {
    if (it->first != object_name) // Don't check with itself.
    {
      testObjectObjectCollision(cdata, object_name, it->first);
    }
  }
}

void collision_space::EnvironmentModelODE::testObjectObjectCollision(CollisionData *cdata, 
                                                                     const std::string& object1_name, 
                                                                     const std::string& object2_name) const
{
  // Check if the given names are valid.
  std::map<std::string, CollisionNamespace*>::const_iterator it1 = coll_namespaces_.find(object1_name);
  if (it1 == coll_namespaces_.end())
  {
    ROS_WARN_STREAM("Failed to find object " << object1_name << " during collision check.");
    return;
  }
  std::map<std::string, CollisionNamespace*>::const_iterator it2 = coll_namespaces_.find(object2_name);
  if (it2 == coll_namespaces_.end())
  {
    ROS_WARN_STREAM("Failed to find object " << object2_name << " during collision check.");
    return;
  }

  // Check if these two objects are allowed to collide.
  bool allowed = false;
  if(cdata->allowed_collision_matrix) {
    if(!cdata->allowed_collision_matrix->getAllowedCollision(object1_name, object2_name, allowed)) {
      ROS_WARN_STREAM("No entry in cdata allowed collision matrix for " << object1_name << " and " << object2_name);
    } 
  }

  if (!allowed)
  {
    ROS_DEBUG_STREAM("Checking collision between " << object1_name << " and " << object2_name << ".");
    dSpaceCollide2((dxGeom *)it1->second->space, (dxGeom *)it2->second->space, cdata, nearCallbackFn);
  }
  else
  {
    ROS_DEBUG_STREAM("Not checking collision between " << object1_name << " and " << object2_name << " since collision is allowed between the two.");
    return;
  }
 
}

void collision_space::EnvironmentModelODE::testObjectCollision(CollisionNamespace *cn, CollisionData *cdata) const
{ 
  if (cn->collide2.empty()) {
    ROS_WARN_STREAM("Problem - collide2 required for body collision for " << cn->name);
    return;
  }
  
  cn->collide2.setup();
  for (int i = model_geom_.link_geom.size() - 1 ; i >= 0 && !cdata->done; --i) {
    LinkGeom *lg = model_geom_.link_geom[i];
    
    bool allowed = false;
    if(cdata->allowed_collision_matrix) {
      if(!cdata->allowed_collision_matrix->getAllowedCollision(cn->name, lg->link->getName(), allowed)) {
        ROS_WARN_STREAM("No entry in cdata allowed collision matrix for " << cn->name << " and " << lg->link->getName());
        return;
      } 
    }
    
    //have to test collisions with link
    if(!allowed) {
      ROS_DEBUG_STREAM("Will test for collision between object " << cn->name << " and link " << lg->link->getName());
      for(unsigned int j = 0; j < lg->padded_geom.size(); j++) {
        //have to figure
        unsigned int current_contacts_size = 0;
        if(cdata->contacts) {
          current_contacts_size = cdata->contacts->size();
        }
        cn->collide2.collide(lg->padded_geom[j], cdata, nearCallbackFn);
        if(cdata->contacts && cdata->contacts->size() > current_contacts_size) {
          //new contacts must mean collision
          for(unsigned int k = current_contacts_size; k < cdata->contacts->size(); k++) {
            if(cdata->contacts->at(k).body_type_1 == OBJECT) {
              cdata->contacts->at(k).body_name_1 = cn->name;
            } else if(cdata->contacts->at(k).body_type_2 == OBJECT) {
              cdata->contacts->at(k).body_name_2 = cn->name;
            } else {
              ROS_WARN_STREAM("New contacts really should have an object as one of the bodys");
            }
          }
        }
        if(cdata->done) {
          return;
        }
      }
    } else {
      ROS_DEBUG_STREAM("Will not test for allowed collision between object " << cn->name << " and link " << lg->link->getName());
    }
    //now we need to do the attached bodies
    for(unsigned int j = 0; j < lg->att_bodies.size(); j++) {
      std::string att_name = lg->att_bodies[j]->att->getName();
      allowed = false;
      if(cdata->allowed_collision_matrix) {
        if(!cdata->allowed_collision_matrix->getAllowedCollision(cn->name, att_name, allowed)) {
          ROS_WARN_STREAM("No entry in current allowed collision matrix for " << cn->name << " and " << att_name);
          return;
        }
      }
      if(!allowed) {
        ROS_DEBUG_STREAM("Will test for collision between object " << cn->name << " and attached object " << att_name);
        for(unsigned int k = 0; k < lg->att_bodies[j]->padded_geom.size(); k++) {
          //have to figure
          unsigned int current_contacts_size = 0;
          if(cdata->contacts) {
            current_contacts_size = cdata->contacts->size();
          }
          cn->collide2.collide(lg->att_bodies[j]->padded_geom[k], cdata, nearCallbackFn);
          if(cdata->contacts && cdata->contacts->size() > current_contacts_size) {
            //new contacts must mean collision
            for(unsigned int l = current_contacts_size; l < cdata->contacts->size(); l++) {
              if(cdata->contacts->at(l).body_type_1 == OBJECT) {
                cdata->contacts->at(l).body_name_1 = cn->name;
              } else if(cdata->contacts->at(l).body_type_2 == OBJECT) {
                cdata->contacts->at(l).body_name_2 = cn->name;
              } else {
                ROS_WARN_STREAM("New contacts really should have an object as one of the bodys");
              }
            }
          }
          if(cdata->done) {
            return;
          }
        }
      } else {
        ROS_DEBUG_STREAM("Will not test for allowed collision between object " << cn->name << " and attached object " << att_name);
      } 
    }
  }
}

void collision_space::EnvironmentModelODE::testCollision(CollisionData *cdata) const
{
  testSelfCollision(cdata);
  testEnvironmentCollision(cdata);    
}

void collision_space::EnvironmentModelODE::testSelfCollision(CollisionData *cdata) const
{
  dSpaceCollide(model_geom_.self_space, cdata, nearCallbackFn);
}

void collision_space::EnvironmentModelODE::testEnvironmentCollision(CollisionData *cdata) const
{
  /* check collision with other ode bodies until done*/
  for (std::map<std::string, CollisionNamespace*>::const_iterator it = coll_namespaces_.begin() ; it != coll_namespaces_.end() && !cdata->done ; ++it) {
    testObjectCollision(it->second, cdata);
  }
}

bool collision_space::EnvironmentModelODE::hasObject(const std::string& ns) const
{
  if(coll_namespaces_.find(ns) != coll_namespaces_.end()) {
    return true;
  }
  return false;
}

void collision_space::EnvironmentModelODE::addObjects(const std::string &ns, const std::vector<shapes::Shape*> &shapes, const std::vector<tf::Transform> &poses)
{
  assert(shapes.size() == poses.size());
  std::map<std::string, CollisionNamespace*>::iterator it = coll_namespaces_.find(ns);
  CollisionNamespace* cn = NULL;    
  if (it == coll_namespaces_.end())
  {
    cn = new CollisionNamespace(ns);
    dspace_lookup_map_[ns] = cn->space;
    coll_namespaces_[ns] = cn;
    default_collision_matrix_.addEntry(ns, false);
  }
  else {
     cn = it->second;
  }

  //we're going to create the namespace in objects_ even if it doesn't have anything in it
  objects_->addObjectNamespace(ns);

  unsigned int n = shapes.size();
  for (unsigned int i = 0 ; i < n ; ++i)
  {
    dGeomID g = createODEGeom(cn->space, cn->storage, shapes[i], 1.0, 0.0);
    assert(g);
    dGeomSetData(g, reinterpret_cast<void*>(shapes[i]));
    updateGeom(g, poses[i]);
    cn->collide2.registerGeom(g);
    objects_->addObject(ns, shapes[i], poses[i]);
  }
  cn->collide2.setup();
}

void collision_space::EnvironmentModelODE::addObject(const std::string &ns, shapes::Shape *shape, const tf::Transform &pose)
{
  std::map<std::string, CollisionNamespace*>::iterator it = coll_namespaces_.find(ns);
  CollisionNamespace* cn = NULL;    
  if (it == coll_namespaces_.end())
  {
    cn = new CollisionNamespace(ns);
    dspace_lookup_map_[ns] = cn->space;
    coll_namespaces_[ns] = cn;
    default_collision_matrix_.addEntry(ns, false);
  }
  else
    cn = it->second;

  dGeomID g = createODEGeom(cn->space, cn->storage, shape, 1.0, 0.0);
  assert(g);
  dGeomSetData(g, reinterpret_cast<void*>(shape));

  updateGeom(g, pose);
  cn->geoms.push_back(g);
  objects_->addObject(ns, shape, pose);
}

void collision_space::EnvironmentModelODE::addObject(const std::string &ns, shapes::StaticShape* shape)
{   
  std::map<std::string, CollisionNamespace*>::iterator it = coll_namespaces_.find(ns);
  CollisionNamespace* cn = NULL;    
  if (it == coll_namespaces_.end())
  {
    cn = new CollisionNamespace(ns);
    dspace_lookup_map_[ns] = cn->space;
    coll_namespaces_[ns] = cn;
    default_collision_matrix_.addEntry(ns, false);
  }
  else
    cn = it->second;

  dGeomID g = createODEGeom(cn->space, cn->storage, shape);
  assert(g);
  dGeomSetData(g, reinterpret_cast<void*>(shape));
  cn->geoms.push_back(g);
  objects_->addObject(ns, shape);
}

void collision_space::EnvironmentModelODE::clearObjects(void)
{
  for (std::map<std::string, CollisionNamespace*>::iterator it = coll_namespaces_.begin() ; it != coll_namespaces_.end() ; ++it) {
    default_collision_matrix_.removeEntry(it->first);
    delete it->second;
  }
  dspace_lookup_map_.clear();
  coll_namespaces_.clear();
  objects_->clearObjects();
}

void collision_space::EnvironmentModelODE::clearObjects(const std::string &ns)
{
  std::map<std::string, CollisionNamespace*>::iterator it = coll_namespaces_.find(ns);
  if (it != coll_namespaces_.end()) {
    default_collision_matrix_.removeEntry(ns);
    delete it->second;
    coll_namespaces_.erase(ns);
    dspace_lookup_map_.erase(ns);
  }
  objects_->clearObjects(ns);
}

dGeomID collision_space::EnvironmentModelODE::copyGeom(dSpaceID space, ODEStorage &storage, dGeomID geom, ODEStorage &sourceStorage) const
{
  int c = dGeomGetClass(geom);
  dGeomID ng = NULL;
  bool location = true;
  switch (c)
  {
  case dSphereClass:
    ng = dCreateSphere(space, dGeomSphereGetRadius(geom));
    break;
  case dBoxClass:
    {
      dVector3 r;
      dGeomBoxGetLengths(geom, r);
      ng = dCreateBox(space, r[0], r[1], r[2]);
    }
    break;
  case dCylinderClass:
    {
      dReal r, l;
      dGeomCylinderGetParams(geom, &r, &l);
      ng = dCreateCylinder(space, r, l);
    }
    break;
  case dPlaneClass:
    {
      dVector4 p;
      dGeomPlaneGetParams(geom, p);
      ng = dCreatePlane(space, p[0], p[1], p[2], p[3]);
      location = false;
    }
    break;
  case dTriMeshClass:
    {
      dTriMeshDataID tdata = dGeomTriMeshGetData(geom);
      dTriMeshDataID cdata = dGeomTriMeshDataCreate();
      for(std::map<dGeomID, ODEStorage::Element>::const_iterator it = sourceStorage.meshes.begin();
          it != sourceStorage.meshes.end();
          it++) {
        if (it->second.data == tdata)
        {
          ODEStorage::Element& e = storage.meshes[geom];
          e.n_vertices = it->second.n_vertices;
          e.n_indices = it->second.n_indices;
          e.indices = new dTriIndex[e.n_indices];
          for (int j = 0 ; j < e.n_indices ; ++j)
            e.indices[j] = it->second.indices[j];
          e.vertices = new double[e.n_vertices];
          for (int j = 0 ; j < e.n_vertices ; ++j)
            e.vertices[j] = e.vertices[j];
          dGeomTriMeshDataBuildDouble(cdata, e.vertices, sizeof(double) * 3, e.n_vertices, e.indices, e.n_indices, sizeof(dTriIndex) * 3);
          e.data = cdata;
          break;
        }
        ng = dCreateTriMesh(space, cdata, NULL, NULL, NULL);
      }
    }
    break;
  default:
    assert(0); // this should never happen
    break;
  }
    
  if (ng && location)
  {
    const dReal *pos = dGeomGetPosition(geom);
    dGeomSetPosition(ng, pos[0], pos[1], pos[2]);
    dQuaternion q;
    dGeomGetQuaternion(geom, q);
    dGeomSetQuaternion(ng, q);
  }
    
  return ng;
}

collision_space::EnvironmentModel* collision_space::EnvironmentModelODE::clone(void) const
{
  EnvironmentModelODE *env = new EnvironmentModelODE();
  env->default_collision_matrix_ = default_collision_matrix_;
  env->default_link_padding_map_ = default_link_padding_map_;
  env->verbose_ = verbose_;
  env->robot_scale_ = robot_scale_;
  env->default_robot_padding_ = default_robot_padding_;
  env->robot_model_ = new planning_models::KinematicModel(*robot_model_);
  env->createODERobotModel();

  for (std::map<std::string, CollisionNamespace*>::const_iterator it = coll_namespaces_.begin() ; it != coll_namespaces_.end() ; ++it) {
    // construct a map of the shape pointers we have; this points to the index positions where they are stored;
    std::map<void*, int> shapePtrs;
    const EnvironmentObjects::NamespaceObjects &ns = objects_->getObjects(it->first);
    unsigned int n = ns.static_shape.size();
    for (unsigned int i = 0 ; i < n ; ++i)
      shapePtrs[ns.static_shape[i]] = -1 - i;
    n = ns.shape.size();
    for (unsigned int i = 0 ; i < n ; ++i)
      shapePtrs[ns.shape[i]] = i;
    
    // copy the collision namespace structure, geom by geom
    CollisionNamespace *cn = new CollisionNamespace(it->first);
    env->coll_namespaces_[it->first] = cn;
    env->dspace_lookup_map_[cn->name] = cn->space;
    n = it->second->geoms.size();
    cn->geoms.reserve(n);
    for (unsigned int i = 0 ; i < n ; ++i)
    {
      dGeomID newGeom = copyGeom(cn->space, cn->storage, it->second->geoms[i], it->second->storage);
      int idx = shapePtrs[dGeomGetData(it->second->geoms[i])];
      if (idx < 0) // static geom
      {
        shapes::StaticShape *newShape = shapes::cloneShape(ns.static_shape[-idx - 1]);
        dGeomSetData(newGeom, reinterpret_cast<void*>(newShape));
        env->objects_->addObject(it->first, newShape);
      }
      else // movable geom
      {
        shapes::Shape *newShape = shapes::cloneShape(ns.shape[idx]);
        dGeomSetData(newGeom, reinterpret_cast<void*>(newShape));
        env->objects_->addObject(it->first, newShape, ns.shape_pose[idx]);
      }
      cn->geoms.push_back(newGeom);
    }
    std::vector<dGeomID> geoms;
    it->second->collide2.getGeoms(geoms);
    n = geoms.size();
    for (unsigned int i = 0 ; i < n ; ++i)
    {
      dGeomID newGeom = copyGeom(cn->space, cn->storage, geoms[i], it->second->storage);
      int idx = shapePtrs[dGeomGetData(geoms[i])];
      if (idx < 0) // static geom
      {
        shapes::StaticShape *newShape = shapes::cloneShape(ns.static_shape[-idx - 1]);
        dGeomSetData(newGeom, reinterpret_cast<void*>(newShape));
        env->objects_->addObject(it->first, newShape);
      }
      else // movable geom
      {
        shapes::Shape *newShape = shapes::cloneShape(ns.shape[idx]);
        dGeomSetData(newGeom, reinterpret_cast<void*>(newShape));
        env->objects_->addObject(it->first, newShape, ns.shape_pose[idx]);
      }
      cn->collide2.registerGeom(newGeom);
    }
  }
    
  return env;    
}
