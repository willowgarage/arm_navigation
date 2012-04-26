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

#ifndef COLLISION_SPACE_ENVIRONMENT_MODEL_ODE_
#define COLLISION_SPACE_ENVIRONMENT_MODEL_ODE_

#include "collision_space/environment.h"
#include <ode/ode.h>
#include <map>

namespace collision_space
{
    	
/** \brief A class describing an environment for a kinematic robot using ODE */
class EnvironmentModelODE : public EnvironmentModel
{     
  
  friend void nearCallbackFn(void *data, dGeomID o1, dGeomID o2);
	
public:
		
  EnvironmentModelODE(void);
  virtual ~EnvironmentModelODE(void);

  /** \brief Get the list of contacts (collisions). The maximum total number of contacts to be returned can be specified, and the max per pair of objects that's in collision*/
  virtual bool getCollisionContacts(std::vector<Contact> &contacts, unsigned int max_total = 1, unsigned int max_per_pair = 1) const;

  /** \brief This function will get the complete list of contacts between any two potentially colliding bodies.  The num per contacts specifies the number of contacts per pair that will be returned */
  virtual bool getAllCollisionContacts(std::vector<Contact> &contacts, unsigned int num_per_contact = 1) const;

  /** \brief Check if a model is in collision */
  virtual bool isCollision(void) const;

  /** \brief Check if a model is in self collision */
  virtual bool isSelfCollision(void) const;

  /** \brief Check if a model is in environment collision */
  virtual bool isEnvironmentCollision(void) const;

  /** \brief Check if a single static object is in collision with the robot. */
  virtual bool isObjectRobotCollision(const std::string& object_name) const;	

  /** \brief Check if two static objects are in collision. */
  virtual bool isObjectObjectCollision(const std::string& object1_name, 
                                       const std::string& object2_name) const;

  /** \brief Check if an object is in collision with the other static objects. */
  virtual bool isObjectInEnvironmentCollision(const std::string& object_name) const;

  virtual bool getAllObjectEnvironmentCollisionContacts (const std::string& object_name, 
                                                         std::vector<Contact> &contacts,
                                                         unsigned int num_contacts_per_pair) const;
	
  /** \brief Remove all objects from collision model */
  virtual void clearObjects(void);
	
  /** \brief Remove objects from a specific namespace in the collision model */
  virtual void clearObjects(const std::string &ns);

  /** \brief Tells whether or not there is an object with the given name in the collision model */
  virtual bool hasObject(const std::string& ns) const;
		
  /** \brief Add a static collision object to the map. The user releases ownership of the passed object. Memory allocated for the shape is freed by the collision environment. */
  virtual void addObject(const std::string &ns, shapes::StaticShape *shape);

  /** \brief Add a collision object to the map. The user releases ownership of the passed object. Memory allocated for the shape is freed by the collision environment. */
  virtual void addObject(const std::string &ns, shapes::Shape* shape, const tf::Transform &pose);

  /** \brief Add a set of collision objects to the map. The user releases ownership of the passed objects. Memory allocated for the shapes is freed by the collision environment. */
  virtual void addObjects(const std::string &ns, const std::vector<shapes::Shape*> &shapes, const std::vector<tf::Transform> &poses);

  virtual void getAttachedBodyPoses(std::map<std::string, std::vector<tf::Transform> >& pose_map) const;

  /** \brief Add a robot model. Ignore robot links if their name is not
      specified in the string vector. The scale argument can be
      used to increase or decrease the size of the robot's
      bodies (multiplicative factor). The padding can be used to
      increase or decrease the robot's bodies with by an
      additive term */
  virtual void setRobotModel(const planning_models::KinematicModel* model, 
                             const AllowedCollisionMatrix& allowed_collision_matrix,
                             const std::map<std::string, double>& link_padding_map,
                             double default_padding = 0.0,
                             double scale = 1.0); 

  /** \brief Update the positions of the geometry used in collision detection */
  virtual void updateRobotModel(const planning_models::KinematicState* state);

  /** \brief Update the set of bodies that are attached to the robot (re-creates them) */
  virtual void updateAttachedBodies(void);

  /** \brief Update the set of bodies that are attached to the robot (re-creates them) using the indicated padding or default if non-specified */
  virtual void updateAttachedBodies(const std::map<std::string, double>& link_padding_map);

  /** \briefs Sets a temporary robot padding on the indicated links */
  virtual void setAlteredLinkPadding(const std::map<std::string, double>& link_padding_map);

  /** \briefs Reverts link padding to that set at robot initialization */
  virtual void revertAlteredLinkPadding();

  /** \brief Clone the environment */
  virtual EnvironmentModel* clone(void) const;

protected:

  /** \brief Structure for maintaining ODE temporary data */
  struct ODEStorage
  {	
    struct Element
    {
      double *vertices;
      dTriIndex *indices;
      dTriMeshDataID data;
      int n_indices;
      int n_vertices;
    };
	    
    ~ODEStorage(void)
    {
      clear();
    }

    void remove(dGeomID id) {
      
      if(meshes.find(id) == meshes.end()) {
        return;
      }

      Element& e = meshes[id];
      delete[] e.indices;
      delete[] e.vertices;
      dGeomTriMeshDataDestroy(e.data);
      meshes.erase(id);
    }
	    
    void clear(void)
    {
      for(std::map<dGeomID, Element>::iterator it = meshes.begin();
          it != meshes.end();
          it++)
      {
        delete[] it->second.indices;
        delete[] it->second.vertices;
        dGeomTriMeshDataDestroy(it->second.data);
      }
      meshes.clear();
    }
	    
    /* Pointers for ODE indices; we need this around in ODE's assumed datatype */
    std::map<dGeomID, Element> meshes;
  };
	
  class ODECollide2
  {
  public:
	    
    ODECollide2(dSpaceID space = NULL)
    {	
      setup_ = false;
      if (space)
        registerSpace(space);
    }
	    
    ~ODECollide2(void)
    {
      clear();
    }
	    
    void registerSpace(dSpaceID space);
    void registerGeom(dGeomID geom);
    void unregisterGeom(dGeomID geom);
    void clear(void);
    void setup(void);
    void collide(dGeomID geom, void *data, dNearCallback *nearCallback) const;
    bool empty(void) const;
    void getGeoms(std::vector<dGeomID> &geoms) const;
	    
  private:
	    
    struct Geom
    {
      dGeomID id;
      dReal   aabb[6];
    };
	    
    struct SortByXLow
    {
      bool operator()(const Geom *a, const Geom *b) const
      {
        if (a->aabb[0] < b->aabb[0])
          return true;
        return false;
      }
    };
	    
    struct SortByYLow
    {
      bool operator()(const Geom *a, const Geom *b) const
      {
        if (a->aabb[2] < b->aabb[2])
          return true;
        return false;
      }
    };
	    
    struct SortByZLow
    {
      bool operator()(const Geom *a, const Geom *b) const
      {
        if (a->aabb[4] < b->aabb[4])
          return true;
        return false;
      }
    };
	    
    struct SortByXTest
    {
      bool operator()(const Geom *a, const Geom *b) const
      {
        if (a->aabb[1] < b->aabb[0])
          return true;
        return false;
      }
    };
	    
    struct SortByYTest
    {
      bool operator()(const Geom *a, const Geom *b) const
      {
        if (a->aabb[3] < b->aabb[2])
          return true;
        return false;
      }
    };
	    
    struct SortByZTest
    {
      bool operator()(const Geom *a, const Geom *b) const
      {
        if (a->aabb[5] < b->aabb[4])
          return true;
        return false;
      }
    };
	    
    bool setup_;
    std::vector<Geom*> geoms_x;
    std::vector<Geom*> geoms_y;
    std::vector<Geom*> geoms_z;
	    
    void checkColl(std::vector<Geom*>::const_iterator posStart, std::vector<Geom*>::const_iterator posEnd,
                   Geom *g, void *data, dNearCallback *nearCallback) const;
  };

  struct AttGeom
  {
    AttGeom(ODEStorage& s) : storage(s){
    }

    ~AttGeom() {

      for(unsigned int i = 0; i < geom.size(); i++) {
        dGeomDestroy(geom[i]);
        storage.remove(geom[i]);
      }
      for(unsigned int i = 0; i < padded_geom.size(); i++) {
        dGeomDestroy(padded_geom[i]);
        storage.remove(padded_geom[i]);
      }
    }

    ODEStorage& storage;
    std::vector<dGeomID> geom;
    std::vector<dGeomID> padded_geom;
    const planning_models::KinematicModel::AttachedBodyModel *att;
    unsigned int index;
  };

  struct LinkGeom
  {
    LinkGeom(ODEStorage& s) : storage(s){
    }

    ~LinkGeom() {
      for(unsigned int i = 0; i < geom.size(); i++) {
        dGeomDestroy(geom[i]);
      }
      for(unsigned int i = 0; i < padded_geom.size(); i++) {
        dGeomDestroy(padded_geom[i]);
      }
      deleteAttachedBodies();
    }
    
    void deleteAttachedBodies()
    {
      for(unsigned int i = 0; i < att_bodies.size(); i++) {
        delete att_bodies[i];
      }
      att_bodies.clear();
    }

    ODEStorage& storage;
    std::vector<dGeomID> geom;
    std::vector<dGeomID> padded_geom;
    std::vector<AttGeom*> att_bodies;
    const planning_models::KinematicModel::LinkModel *link;
    unsigned int index;
  };
	
  struct ModelInfo
  {
    ~ModelInfo() {
      storage.clear();
    }

    std::vector< LinkGeom* > link_geom;
    dSpaceID env_space;
    dSpaceID self_space;
    ODEStorage storage;
  };
	
  struct CollisionNamespace
  {
    CollisionNamespace(const std::string &nm) : name(nm)
    {
      space = dHashSpaceCreate(0);
    }
	    
    virtual ~CollisionNamespace(void)
    {
      if (space) {
        dSpaceDestroy(space);
      }
    }
	    
    void clear(void)
    {
      if (space) { 
        dSpaceDestroy(space);
      }
      space = dHashSpaceCreate(0);
      geoms.clear();
      collide2.clear();
      storage.clear();
    }
	    
    std::string name;
    dSpaceID space;
    std::vector<dGeomID> geoms;
    ODECollide2 collide2;
    ODEStorage storage;
  };
    
  struct CollisionData
  {
    CollisionData(void)
    {
      done = false;
      collides = false;
      max_contacts_total = 0;
      max_contacts_pair = 0;
      contacts = NULL;
      allowed_collision_matrix = NULL;
      allowed = NULL;
    }

    //these are parameters
    unsigned int max_contacts_total;
    unsigned int max_contacts_pair;
    const AllowedCollisionMatrix *allowed_collision_matrix;
    const std::map<dGeomID, std::pair<std::string, BodyType> >* geom_lookup_map;
    const std::map<std::string, dSpaceID>* dspace_lookup_map;
    const AllowedContactMap *allowed;
	    
    //these are for return info
    bool done;
    bool collides;
    std::vector<EnvironmentModelODE::Contact> *contacts;
    
    //for the last collision found
    std::string body_name_1;
    BodyType body_type_1;

    std::string body_name_2;
    BodyType body_type_2;



  };
	
	
  /** \brief Internal function for collision detection */
  void testCollision(CollisionData *data) const;

  /** \brief Internal function for collision detection */
  void testSelfCollision(CollisionData *data) const;

  /** \brief Internal function for collision detection */
  void testEnvironmentCollision(CollisionData *data) const;

  /** \brief Test collision between two static objects. */
  void testObjectObjectCollision(CollisionData *cdata, 
                                 const std::string& object1_name, 
                                 const std::string& object2_name) const;

  /** \brief Test collision between an object and all other static objects. */
  void testObjectEnvironmentCollision(CollisionData *cdata, 
                                      const std::string& object_name) const;

  /** \brief Internal function for collision detection */
  void testObjectCollision(CollisionNamespace *cn, CollisionData *data) const;
  
  dGeomID copyGeom(dSpaceID space, ODEStorage &storage, dGeomID geom, ODEStorage &sourceStorage) const;
  void createODERobotModel();	
  dGeomID createODEGeom(dSpaceID space, ODEStorage &storage, const shapes::Shape *shape, double scale, double padding);
  dGeomID createODEGeom(dSpaceID space, ODEStorage &storage, const shapes::StaticShape *shape);
  void updateGeom(dGeomID geom, const tf::Transform &pose) const;	

  void addAttachedBody(LinkGeom* lg, const planning_models::KinematicModel::AttachedBodyModel* attm,
                       double padd);

  std::map<std::string, bool> attached_bodies_in_collision_matrix_;

  void setAttachedBodiesLinkPadding();
  void revertAttachedBodiesLinkPadding();

  void freeMemory(void);	
	
  ModelInfo model_geom_;
  std::map<std::string, CollisionNamespace*> coll_namespaces_;

  std::map<dGeomID, std::pair<std::string, BodyType> > geom_lookup_map_;
  std::map<std::string, dSpaceID> dspace_lookup_map_;

  bool previous_set_robot_model_;

  void checkThreadInit(void) const;  	
};
}

#endif
