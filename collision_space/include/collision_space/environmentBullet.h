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

#ifndef COLLISION_SPACE_ENVIRONMENT_MODEL_BULLET_
#define COLLISION_SPACE_ENVIRONMENT_MODEL_BULLET_

#include "collision_space/environment.h"

#include "btBulletCollisionCommon.h"
#include <map>

namespace collision_space
{
    	
    /** \brief A class describing an environment for a kinematic robot using bullet. This class is still experimental, and methos such as cloning are not implemented. */
    class EnvironmentModelBullet : public EnvironmentModel
    {     
    public:
	
        EnvironmentModelBullet(void) : EnvironmentModel(),
				       m_selfCollisionFilterCallback(&m_selfCollisionTest),
				       m_genericCollisionFilterCallback(&m_selfCollisionTest, &m_selfCollision)
	{
	    m_config = new btDefaultCollisionConfiguration();
	    btCollisionDispatcher* dispatcher = new CollisionDispatcher(&m_selfCollisionTest, &m_selfCollision, m_config);
	    btBroadphaseInterface *broadphase = new btDbvtBroadphase();
	    m_world = new btCollisionWorld(dispatcher, broadphase, m_config);
	}
	
	virtual ~EnvironmentModelBullet(void)
	{ 
	    freeMemory();
	}
	
	/** \brief Get the list of contacts (collisions) */
	virtual bool getCollisionContacts(const std::vector<AllowedContact> &allowedContacts, std::vector<Contact> &contacts, unsigned int max_count = 1);

	/** \brief Check if a model is in collision */
	virtual bool isCollision(void);

	/** \brief Check if a model is in self collision */
	virtual bool isSelfCollision(void);
	
	/** \brief Remove all objects from collision model */
	virtual void clearObjects(void);
	
	/** \brief Remove objects from a specific namespace in the collision model */
	virtual void clearObjects(const std::string &ns);
	
	/** \brief Add a static collision object to the map. The user releases ownership of the passed object. Memory allocated for the shape is freed by the collision environment. */
	virtual void addObject(const std::string &ns, shapes::StaticShape *shape);

	/** \brief Add a collision object to the map. The user releases ownership of the passed object. Memory allocated for the shape is freed by the collision environment. */
	virtual void addObject(const std::string &ns, shapes::Shape* shape, const tf::Transform &pose);

	/** \brief Add a set of collision objects to the map. The user releases ownership of the passed objects. Memory allocated for the shapes is freed by the collision environment. */
	virtual void addObjects(const std::string &ns, const std::vector<shapes::Shape*> &shapes, const std::vector<tf::Transform> &poses);

	/** \brief Remove objects in the collision space that are collising with the object supplied as argument */
	virtual void removeCollidingObjects(const shapes::StaticShape *shape);

	/** \brief Remove objects in the collision space that are collising with the object supplied as argument */
	virtual void removeCollidingObjects(const shapes::Shape *shape, const tf::Transform &pose);

      virtual const std::vector<const planning_models::KinematicModel::AttachedBodyModel*> getAttachedBodies(void) const;
	
	/** \brief Add a robot model. Ignore robot links if their name is not
	    specified in the string vector. The scale argument can be
	    used to increase or decrease the size of the robot's
	    bodies (multiplicative factor). The padding can be used to
	    increase or decrease the robot's bodies with by an
	    additive term */
      virtual void setRobotModel(const boost::shared_ptr<const planning_models::KinematicModel> &model, 
                                 const std::vector<std::string> &links,
                                 const std::map<std::string, double>& link_padding_map,
                                 double default_padding = 0.0,
                                 double scale = 1.0);

	/** \brief Update the positions of the geometry used in collision detection */
	virtual void updateRobotModel(void);

	/** \brief Update the set of bodies that are attached to the robot (re-creates them) */
	virtual void updateAttachedBodies(void);

	/** \brief Enable/Disable collision checking for specific links. Return the previous value of the state (1 or 0) if succesful; -1 otherwise */
	virtual int setCollisionCheck(const std::string &link, bool state);

	/** \brief Enable/Disable collision checking for a set of links.*/
  void setCollisionCheckLinks(const std::vector<std::string> &links, bool state);

	/** \brief Set collision checking for the set of links to state, set collision checking for all other links to !state */
  void setCollisionCheckOnlyLinks(const std::vector<std::string> &links, bool state);

	/** \brief Set collision checking for all links to state */
  void setCollisionCheckAll(bool state);

	/** \brief Clone the environment */
	virtual EnvironmentModel* clone(void) const;
	
    protected:

	struct kGeom
	{
	    std::vector<btCollisionObject*>        geom;
	    bool                                   enabled;
          const 
planning_models::KinematicModel::LinkModel *link;
	    unsigned int                           index;
	};

	struct ModelInfo
	{
	    std::vector< kGeom* > linkGeom;
	    double                scale;
	    double                padding; 
	};
	
	struct SelfCollisionFilterCallback : public btOverlapFilterCallback
	{
	    SelfCollisionFilterCallback(std::vector< std::vector<bool> > *test) : selfCollisionTest(test)
	    {
	    }
	    
	    virtual bool needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const
	    {
		assert(static_cast<btCollisionObject*>(proxy0->m_clientObject) != NULL);
		assert(static_cast<btCollisionObject*>(proxy1->m_clientObject) != NULL);
		
		kGeom *k0 = reinterpret_cast<kGeom*>(reinterpret_cast<btCollisionObject*>(proxy0->m_clientObject)->getUserPointer());
		kGeom *k1 = reinterpret_cast<kGeom*>(reinterpret_cast<btCollisionObject*>(proxy1->m_clientObject)->getUserPointer());
		
		// only check collision between links
		if (!k0 || !k1)
		    return false;
		
		// only consider links that are enabled for self-collision checking
		return selfCollisionTest->at(k0->index)[k1->index];
	    }
	    
	    std::vector< std::vector<bool> > *selfCollisionTest;
	};

	struct GenericCollisionFilterCallback : public btOverlapFilterCallback
	{
	    GenericCollisionFilterCallback(std::vector< std::vector<bool> > *test, bool *checkSelf) : selfCollisionTest(test),
												      enableSelfCollision(checkSelf)
	    {
	    }
	    
	    virtual bool needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const
	    {
		assert(static_cast<btCollisionObject*>(proxy0->m_clientObject) != NULL);
		assert(static_cast<btCollisionObject*>(proxy1->m_clientObject) != NULL);
		
		kGeom *k0 = reinterpret_cast<kGeom*>(reinterpret_cast<btCollisionObject*>(proxy0->m_clientObject)->getUserPointer());
		kGeom *k1 = reinterpret_cast<kGeom*>(reinterpret_cast<btCollisionObject*>(proxy1->m_clientObject)->getUserPointer());

		// do not check collision between obstacles
		if (!k0 && !k1)
		    return false;

		// do not check disabled links
		if (k0)
		    if (k0->enabled == false)
			return false;
		if (k1)
		    if (k1->enabled == false)
			return false;
		
		// do not check collision between links that should not be self-collision checked
		if (k0 && k1)
		{
		    if (*enableSelfCollision)
			return selfCollisionTest->at(k0->index)[k1->index];
		    else
			return false;
		}
		
		return true;
	    }
	    
	    std::vector< std::vector<bool> > *selfCollisionTest;
	    bool                             *enableSelfCollision;
	};
	
	class CollisionDispatcher : public btCollisionDispatcher
	{
	public:
	    CollisionDispatcher(std::vector< std::vector<bool> > *test, bool *enableSelfCollision,
				btCollisionConfiguration *config) : btCollisionDispatcher(config),
								    m_selfCollisionTest(test),
								    m_enableSelfCollision(enableSelfCollision)
	    {
	    }
	    
	    virtual bool needsCollision(btCollisionObject* b0, btCollisionObject* b1)
	    {	
		kGeom *k0 = reinterpret_cast<kGeom*>(b0->getUserPointer());
		kGeom *k1 = reinterpret_cast<kGeom*>(b1->getUserPointer());
		
		if (k0 || k1)
		{
		    if (k0)
			if (k0->enabled == false)
			    return false;
		    if (k1)
			if (k1->enabled == false)
			    return false;
		    if (k0 && k1)
			return m_selfCollisionTest->at(k0->index)[k1->index];
		    return true;
		}
		else
		    return false;
	    }
	    
	protected:
	    
	    std::vector< std::vector<bool> > *m_selfCollisionTest;
	    bool                             *m_enableSelfCollision;
	};
	
	btCollisionObject* createCollisionBody(const shapes::Shape *shape, double scale, double padding);
	btCollisionObject* createCollisionBody(const shapes::StaticShape *shape);
	
	void freeMemory(void);
	
	SelfCollisionFilterCallback         m_selfCollisionFilterCallback;
	GenericCollisionFilterCallback      m_genericCollisionFilterCallback;
	
	ModelInfo                           m_modelGeom;
	std::map<std::string, std::vector<btCollisionObject*> > 
	                                    m_obstacles;
	btCollisionWorld                   *m_world;
	btDefaultCollisionConfiguration    *m_config;
	
    };
}

#endif
