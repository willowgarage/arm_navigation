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

#ifndef COLLISION_SPACE_ENVIRONMENT_MODEL_
#define COLLISION_SPACE_ENVIRONMENT_MODEL_

#include "collision_space/environment_objects.h"
#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
#include <geometric_shapes/bodies.h>
#include <LinearMath/btVector3.h>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <string>


/** \brief Main namespace */
namespace collision_space
{
    
/** \brief A class describing an environment for a kinematic
    robot. This is the base (abstract) definition. Different
    implementations are possible. The class is aware of a set of
    obstacles and a robot model. The obstacles are placed in different
    namespaces so they can be added and removed selectively.
*/
class EnvironmentModel
{
public:
	
  /** \brief Definition of a contact point */
  struct Contact
  {
    /** \brief contact position */
    btVector3                              pos;     
    /** \brief normal unit vector at contact */
    btVector3                              normal;  
    /** \brief depth (penetration between bodies) */
    double                                 depth;
    /** \brief first link involved in contact */
    const planning_models::KinematicModel::LinkModel *link1; 

    /** \brief if the contact with link1 is with an attached body this index will be non-zero*/
    unsigned int link1_attached_body_index;
    
    /** \brief if the contact is between two links, this is not NULL */
    const planning_models::KinematicModel::LinkModel *link2; 

    /** \brief if the contact with link2 is with an attached body this index will be non-zero*/
    unsigned int link2_attached_body_index;

    //if the contact is with an object, this will be non-empty, and link2 should be NULL
    std::string object_name; 
  };

  /** \brief Definition of a contact that is allowed */
  struct AllowedContact
  {
    /// the bound where the contact is allowed 
    boost::shared_ptr<bodies::Body> bound;
	    
    /// the set of link names that are allowed to make contact
    std::vector<std::string>        links;

    /// tha maximum depth for the contact
    double                          depth;
  };
    
  EnvironmentModel(void)
  {
    m_selfCollision = true;
    m_verbose = false;
    m_objects = new EnvironmentObjects();
    use_set_collision_matrix_ = false;
  }
	
  virtual ~EnvironmentModel(void)
  {
    if (m_objects)
      delete m_objects;
  }

  /**********************************************************************/
  /* Collision Environment Configuration                                */
  /**********************************************************************/
	
  /** \brief Set the status of self collision */
  void setSelfCollision(bool selfCollision);
	
  /** \brief Check if self collision is enabled */
  bool getSelfCollision(void) const;
			
  /** \brief Enable self-collision between all links in group 1 and all links in group 2 */
  virtual void addSelfCollisionGroup(const std::vector<std::string> &group1,
                                     const std::vector<std::string> &group2);

  /** \brief Disable self-collision between all links in group 1 and all links in group 2 */
  virtual void removeSelfCollisionGroup(const std::vector<std::string> &group1,
                                        const std::vector<std::string> &group2);

  /** \brief Enable/Disable collision checking for specific links. Return the previous value of the state (1 or 0) if succesful; -1 otherwise */
  virtual int setCollisionCheck(const std::string &link, bool state) = 0;

  /** \brief Enable/Disable collision checking for a set of links.*/
  virtual void setCollisionCheckLinks(const std::vector<std::string> &links, bool state) = 0;

  /** \brief Set collision checking for the set of links to state, set collision checking for all other links to !state */
  virtual void setCollisionCheckOnlyLinks(const std::vector<std::string> &links, bool state) = 0;

  /** \brief Set collision checking for all links to state */
  virtual void setCollisionCheckAll(bool state) = 0;

  /** \brief Add a robot model. Ignore robot links if their name is not
      specified in the string vector. The scale argument can be
      used to increase or decrease the size of the robot's
      bodies (multiplicative factor). The padding can be used to
      increase or decrease the robot's bodies with by an
      additive term */
  virtual void setRobotModel(const planning_models::KinematicModel* model, 
                             const std::vector<std::string> &links,
                             const std::map<std::string, double>& link_padding_map,
                             double default_padding = 0.0,
                             double scale = 1.0);

  /** \brief Get robot scale */
  double getRobotScale(void) const;
	
  /** \brief Get robot padding */
  double getRobotPadding(void) const;
	
  /** \brief Update the positions of the geometry used in collision detection */
  virtual void updateRobotModel(const planning_models::KinematicState* state) = 0;

  /** \brief Update the set of bodies that are attached to the robot (re-creates them) */
  virtual void updateAttachedBodies() = 0;
		
  /** \brief Get the robot model */
  const planning_models::KinematicModel* getRobotModel(void) const;
	
  /**********************************************************************/
  /* Collision Checking Routines                                        */
  /**********************************************************************/
	

  /** \brief Check if a model is in collision. Contacts are not computed */
  virtual bool isCollision(void) const = 0;
	
  /** \brief Check for self collision. Contacts are not computed */
  virtual bool isSelfCollision(void) const = 0;
	
  /** \brief Get the list of contacts (collisions). The maximum number of contacts to be returned can be specified. If the value is 0, all found contacts are returned. */
  virtual bool getCollisionContacts(const std::vector<AllowedContact> &allowedContacts, std::vector<Contact> &contacts, unsigned int max_count = 1) const = 0;
  bool getCollisionContacts(std::vector<Contact> &contacts, unsigned int max_count = 1) const;
	
  /**********************************************************************/
  /* Collision Bodies                                                   */
  /**********************************************************************/
	
  /** \brief Remove all objects from collision model */
  virtual void clearObjects(void) = 0;
	
  /** \brief Remove objects from a specific namespace in the collision model */
  virtual void clearObjects(const std::string &ns) = 0;
	
  /** \brief Add a static collision object to the map. The user releases ownership of the passed object. Memory allocated for the shape is freed by the collision environment. */
  virtual void addObject(const std::string &ns, shapes::StaticShape *shape) = 0;

  /** \brief Add a collision object to the map. The user releases ownership of the passed object. Memory allocated for the shape is freed by the collision environment.*/
  virtual void addObject(const std::string &ns, shapes::Shape* shape, const btTransform &pose) = 0;

  /** \brief Add a set of collision objects to the map. The user releases ownership of the passed objects. Memory allocated for the shapes is freed by the collision environment.*/
  virtual void addObjects(const std::string &ns, const std::vector<shapes::Shape*> &shapes, const std::vector<btTransform> &poses) = 0;

  /** \brief Remove objects in the collision space that are collising with the object supplied as argument. */
  virtual void removeCollidingObjects(const shapes::StaticShape *shape) = 0;

  /** \brief Remove objects in the collision space that are collising with the object supplied as argument. */
  virtual void removeCollidingObjects(const shapes::Shape *shape, const btTransform &pose) = 0;

  /** \brief Gets a vector of all the bodies currently attached to the robot.*/
  virtual const std::vector<const planning_models::KinematicModel::AttachedBodyModel*> getAttachedBodies(void) const = 0;

  /** \brief Gets a vector of all the bodies currently attached to a link.*/
  virtual const std::vector<const planning_models::KinematicModel::AttachedBodyModel*> getAttachedBodies(const std::string link_name) const = 0;

  /** \briefs Sets a temporary robot padding on the indicated links */
  virtual void setRobotLinkPadding(const std::map<std::string, double>& link_padding_map);

  /** \briefs Reverts link padding to that set at robot initialization */
  virtual void revertRobotLinkPadding();

  double getCurrentLinkPadding(std::string name) const;
		
  /** \brief Get the objects currently contained in the model */
  const EnvironmentObjects* getObjects(void) const;
  
  virtual void getDefaultAllowedCollisionMatrix(std::vector<std::vector<bool> > &matrix,
                                                std::map<std::string, unsigned int> &ind) const = 0;

  virtual void getCurrentAllowedCollisionMatrix(std::vector<std::vector<bool> > &matrix,
                                                std::map<std::string, unsigned int> &ind) const;
  
  /** \brief set the matrix for collision touch to use in lieu of the default settings */
  virtual void setAllowedCollisionMatrix(const std::vector<std::vector<bool> > &matrix,
                                 const std::map<std::string, unsigned int > &ind);

  /** \brief reverts to using default settings for allowed collisions */  
  virtual void revertAllowedCollisionMatrix();

  /**********************************************************************/
  /* Miscellaneous Routines                                             */
  /**********************************************************************/

  /** \brief Provide interface to a lock. Use carefully! */
  void lock(void);
	
  /** \brief Provide interface to a lock. Use carefully! */
  void unlock(void);

  /** \brief Enable/disable verbosity */
  void setVerbose(bool verbose);
	
  /** \brief Check the state of verbosity */
  bool getVerbose(void) const;
	
  /** \brief Clone the environment. */
  virtual EnvironmentModel* clone(void) const = 0;
	
protected:
        
  /** \brief Mutex used to lock the datastructure */
  boost::recursive_mutex                                             m_lock;

  /** \brief List of links (names) from the robot model that are considered for collision checking */
  std::vector<std::string>                                 m_collisionLinks;

  /** \brief Map used internally to find the index of a link that we do collision checking for */
  std::map<std::string, unsigned int>                      m_collisionLinkIndex;

  /** \brief Matrix of booleans indicating whether pairs of links can self collide */
  /* true means they can collide */
  std::vector< std::vector<bool> >                         m_selfCollisionTest;
	
  /** \brief Flag to indicate whether self collision checking is enabled */
  bool                                                     m_selfCollision;
	
  /** \brief Flag to indicate whether verbose mode is on */
  bool                                                     m_verbose;

  /** \brief Loaded robot model */	
  const planning_models::KinematicModel*       m_robotModel;

  /** \brief List of objects contained in the environment */
  EnvironmentObjects                                      *m_objects;
	
  /** \brief Scaling used for robot links */
  double                                                   m_robotScale;

  /** \brief Padding used for robot links */
  double                                                   m_robotPadd;	

  std::vector<std::vector<bool> > set_collision_matrix_;
  std::map<std::string, unsigned int> set_collision_ind_;

  bool use_set_collision_matrix_;

  std::map<std::string, double> link_padding_map_;
  std::map<std::string, double> altered_link_padding_;
	
};
}

#endif

