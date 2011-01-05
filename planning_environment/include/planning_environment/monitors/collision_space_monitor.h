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

#ifndef PLANNING_ENVIRONMENT_MONITORS_COLLISION_SPACE_MONITOR_
#define PLANNING_ENVIRONMENT_MONITORS_COLLISION_SPACE_MONITOR_

#include "planning_environment/models/collision_models.h"
#include "planning_environment/monitors/kinematic_model_state_monitor.h"

//#include <motion_planning_msgs/AcceptableContact.h>


#include <motion_planning_msgs/AllowedContactSpecification.h>
#include <motion_planning_msgs/OrderedCollisionOperations.h>
#include <motion_planning_msgs/LinkPadding.h>
#include <mapping_msgs/CollisionMap.h>
#include <mapping_msgs/CollisionObject.h>
#include <mapping_msgs/AttachedCollisionObject.h>
#include <boost/thread/mutex.hpp>
#include <planning_environment_msgs/GetAllowedCollisionMatrix.h>
#include <planning_environment_msgs/GetCollisionObjects.h>
#include <planning_environment_msgs/SetAllowedCollisions.h>
#include <std_srvs/Empty.h>

#include <geometric_shapes_msgs/Shape.h>

namespace planning_environment
{

/** \brief @b CollisionSpaceMonitor is a class which in addition to being aware of a robot model,
    is also aware of a collision space.
*/
class CollisionSpaceMonitor : public KinematicModelStateMonitor
{
public:	

  CollisionSpaceMonitor(CollisionModels *cm, tf::TransformListener *tf) : KinematicModelStateMonitor(static_cast<RobotModels*>(cm), tf)
  {
    cm_ = cm;
    setupCSM();

  }
	
  virtual ~CollisionSpaceMonitor(void)
  {
    if (collisionObjectFilter_)
      delete collisionObjectFilter_;
    if (collisionObjectSubscriber_)
      delete collisionObjectSubscriber_;
    if (collisionMapFilter_)
      delete collisionMapFilter_;
    if (collisionMapSubscriber_)
      delete collisionMapSubscriber_;
    if (collisionMapUpdateFilter_)
      delete collisionMapUpdateFilter_;
    if (collisionMapUpdateSubscriber_)
      delete collisionMapUpdateSubscriber_;
    if (attachedCollisionObjectSubscriber_)
      delete attachedCollisionObjectSubscriber_;

  }

  /** \brief Start the environment monitor. By default, the monitor is started after creation */
  void startEnvironmentMonitor(void);
	
  /** \brief Stop the environment monitor. */
  void stopEnvironmentMonitor(void);
	
  virtual void advertiseServices();

  /** \brief Check if the environment monitor is currently started */
  bool isEnvironmentMonitorStarted(void) const
  {
    return envMonitorStarted_;
  }	

  /** \brief Return the instance of the environment model maintained */
  collision_space::EnvironmentModel* getEnvironmentModel(void) const
  {
    return collisionSpace_;
  }
	
  /** \brief Return the instance of collision models that is being used */
  CollisionModels* getCollisionModels(void) const
  {
    return cm_;
  }
	
  /** \brief Define a callback for before updating a map */
  void setOnBeforeMapUpdateCallback(const boost::function<void(const mapping_msgs::CollisionMapConstPtr, bool)> &callback)
  {
    onBeforeMapUpdate_ = callback;
  }

  /** \brief Define a callback for after updating a map */
  void setOnAfterMapUpdateCallback(const boost::function<void(const mapping_msgs::CollisionMapConstPtr, bool)> &callback)
  {
    onAfterMapUpdate_ = callback;
  }

  /** \brief Define a callback for after updating a map */
  void setOnAfterAttachCollisionObjectCallback(const boost::function<void(const mapping_msgs::AttachedCollisionObjectConstPtr &attachedObject)> &callback)
  {
    onAfterAttachCollisionObject_ = callback;
  }

  /** \brief Define a callback for after updating objects */
  void setOnAfterCollisionObjectCallback(const boost::function<void(const mapping_msgs::CollisionObjectConstPtr &attachedObject)> &callback)
  {
    onCollisionObjectUpdate_ = callback;
  }

  /** \brief Return true if  map has been received */
  bool haveMap(void) const
  {
    return haveMap_;
  }
	
  /** \brief Return true if a map update has been received in the last sec seconds. If sec < 10us, this function always returns true. */
  bool isMapUpdated(double sec) const;
	
  /** \brief Wait until a map is received */
  void waitForMap(void) const;	

  /** \brief Return the last update time for the map */
  const ros::Time& lastMapUpdate(void) const
  {
    return lastMapUpdate_;
  }
	
  /** \brief Returns the padding used for pointclouds (for collision checking) */
  double getPointCloudPadd(void) const
  {
    return pointcloud_padd_;
  }
	
  /** \brief Convert an allowed contact message into the structure accepted by the collision space */
  //	bool computeAllowedContact(const motion_planning_msgs::AcceptableContact &allowedContactMsg, collision_space::EnvironmentModel::AllowedContact &allowedContact) const;
  bool computeAllowedContact(const motion_planning_msgs::AllowedContactSpecification &region,
                             collision_space::EnvironmentModel::AllowedContact &allowedContact) const;
	
  /** \brief This function provides the means to obtain a collision map (the set of boxes)
   *  from the current environment model */
  void recoverCollisionMap(mapping_msgs::CollisionMap &cmap);

  /** \brief If the user modified some instance of an environment model, this function provides the means to obtain a collision map (the set of boxes)
   *  from that environment model */
  void recoverCollisionMap(const collision_space::EnvironmentModel *env, mapping_msgs::CollisionMap &cmap);

	
  /** \brief This function provides the means to
      obtain a set of objects in map (all objects that are not
      in the namespace the collision map was added to) from the current environment */
  void recoverCollisionObjects(std::vector<mapping_msgs::CollisionObject> &omap);
	
  /** \brief If the user modified some instance of an
      environment model, this function provides the means to
      obtain a set of objects in map (all objects that are not
      in the namespace the collision map was added to) */
  void recoverCollisionObjects(const collision_space::EnvironmentModel *env, std::vector<mapping_msgs::CollisionObject> &omap);

  /** \brief This function provides the means to
      obtain a set of attached objects in map from the current environment */
  void recoverAttachedCollisionObjects(std::vector<mapping_msgs::AttachedCollisionObject> &avec);
	
  /** \brief If the user modified some instance of an
      environment model, this function provides the means to
      obtain a set of attached objects in map */
  void recoverAttachedCollisionObjects(const collision_space::EnvironmentModel *env, std::vector<mapping_msgs::AttachedCollisionObject> &avec);
  
  bool getObjectsService(planning_environment_msgs::GetCollisionObjects::Request &req,
                         planning_environment_msgs::GetCollisionObjects::Response &res);    
  /** \brief This function changes the allowed collisions 
      in the collision space to reflect the contents of the
      message.  This allowed collison set will be in place
      until revert is called.
   */
  bool applyOrderedCollisionOperationsToCollisionSpace(const motion_planning_msgs::OrderedCollisionOperations &ord, bool print = false);

  /** \brief This function gets whatever allowed collision matrix is currently in use
      by the collision space */
  bool getCurrentAllowedCollisionsService(planning_environment_msgs::GetAllowedCollisionMatrix::Request& req,
                                          planning_environment_msgs::GetAllowedCollisionMatrix::Response& res);
  
  /** \brief This service sets the allowed collisions using ordered collision operations.  
      The set collisions will remain in effect until a revert to default is called */
  bool setAllowedCollisionsService(planning_environment_msgs::SetAllowedCollisions::Request& req,
                                   planning_environment_msgs::SetAllowedCollisions::Response& res);

  /** \brief Applies ordered collision operations to a matrix with entries and indices specified
   */
  bool applyOrderedCollisionOperationsToMatrix(const motion_planning_msgs::OrderedCollisionOperations &ord,
                                               std::vector<std::vector<bool> > &curAllowed,
                                               std::map<std::string, unsigned int> &vecIndices);
  /** \brief Helper function for expanding collision operations given groups and attached bodies*/
  bool expandOrderedCollisionOperations(const motion_planning_msgs::OrderedCollisionOperations &ord,
                                        motion_planning_msgs::OrderedCollisionOperations &ex);

  /** \brief Service for reverting the allowed collision matrix to the default */
  bool revertAllowedCollisionMatrixToDefaultService(std_srvs::Empty::Request& req,
                                                    std_srvs::Empty::Response& res);
    
  /** \brief Applies indicated padding to links */  
  void applyLinkPaddingToCollisionSpace(const std::vector<motion_planning_msgs::LinkPadding>& link_padding);

  /** \brief Applies indicated padding to links */  
  void revertCollisionSpacePaddingToDefault(); 
  
  /** \brief This function reverts the allowed collisions in the 
      collision space to the default
   */
  void revertAllowedCollisionToDefault();
  
  /** \brief This gets the allowed collision information from the environment
      and prints it to std::out for debug purposes */
  void printAllowedCollisionMatrix(const std::vector<std::vector<bool> > &curAllowed,
                                   const std::map<std::string, unsigned int> &vecIndices) const;

  void setUseCollisionMap(bool use);

  void setCollisionSpace();

protected:

  struct KnownObject
  {
    KnownObject(void)
    {
    }
    
    ~KnownObject(void) {
      deleteBodies();
    }
    
    void deleteBodies() {
      for(unsigned int i = 0; i < bodies.size(); i++) {
        delete bodies[i];
      }
      bodies.clear();
    }
    
    void addBodyWithPose(bodies::Body* body, const btTransform &pose) {
      body->setPose(pose);
      bodies::BoundingSphere bsphere;
      body->computeBoundingSphere(bsphere);
      bodies.push_back(body);
      double rsquare = bsphere.radius * bsphere.radius;
      bspheres.push_back(bsphere);
      rsquares.push_back(rsquare);
    }
    
    void usePose(const unsigned int i, const btTransform &pose) {
      if(i >= bodies.size()) {
        ROS_WARN("Not enough bodies");
        return;
      }
      bodies[i]->setPose(pose);
      bodies[i]->computeBoundingSphere(bspheres[i]);
      rsquares[i] = bspheres[i].radius*bspheres[i].radius;
    }
    
    std::vector<bodies::Body*> bodies;
    std::vector<bodies::BoundingSphere>  bspheres;
    std::vector<double>                  rsquares;
  };

  void maskCollisionMapForCollisionObjects(std::vector<shapes::Shape*> &all_shapes,
                                           std::vector<btTransform> &all_poses,
                                           std::vector<bool> &mask);

  void updateStaticObjectBodies(const mapping_msgs::CollisionObjectConstPtr &collisionObject);
  
  void setupCSM(void);
  void updateCollisionSpace(const mapping_msgs::CollisionMapConstPtr &collisionMap, bool clear);
  void collisionMapAsSpheres(const mapping_msgs::CollisionMapConstPtr &collisionMap,
                             std::vector<shapes::Shape*> &spheres, std::vector<btTransform> &poses);
  void collisionMapAsBoxes(const mapping_msgs::CollisionMap &collisionMap,
                           std::vector<shapes::Shape*> &boxes, std::vector<btTransform> &poses);
  void collisionMapAsBoxes(const mapping_msgs::CollisionMapConstPtr &collisionMap,
                           std::vector<shapes::Shape*> &boxes, std::vector<btTransform> &poses);
  void collisionMapCallback(const mapping_msgs::CollisionMapConstPtr &collisionMap);
  void collisionMapUpdateCallback(const mapping_msgs::CollisionMapConstPtr &collisionMap);
  void collisionObjectCallback(const mapping_msgs::CollisionObjectConstPtr &collisionObject);
  virtual bool attachObjectCallback(const mapping_msgs::AttachedCollisionObjectConstPtr &attachedObject);
  void addAttachedCollisionObjects(std::vector<std::string>& svec) const;

  CollisionModels                                                *cm_;
  collision_space::EnvironmentModel                              *collisionSpace_;
  boost::mutex                                                    mapUpdateLock_;
  double                                                          pointcloud_padd_;
	
  bool                                                            envMonitorStarted_;
	
  bool                                                            haveMap_;
  ros::Time                                                       lastMapUpdate_;	
	
  message_filters::Subscriber<mapping_msgs::CollisionMap>        *collisionMapSubscriber_;
  tf::MessageFilter<mapping_msgs::CollisionMap>                  *collisionMapFilter_;
  message_filters::Subscriber<mapping_msgs::CollisionMap>        *collisionMapUpdateSubscriber_;
  tf::MessageFilter<mapping_msgs::CollisionMap>                  *collisionMapUpdateFilter_;
  message_filters::Subscriber<mapping_msgs::CollisionObject>         *collisionObjectSubscriber_;
  tf::MessageFilter<mapping_msgs::CollisionObject>                   *collisionObjectFilter_;

  message_filters::Subscriber<mapping_msgs::AttachedCollisionObject> *attachedCollisionObjectSubscriber_;

  ros::ServiceServer get_objects_service_;
  ros::ServiceServer get_current_collision_map_service_;
  ros::ServiceServer set_allowed_collisions_service_;
  ros::ServiceServer revert_allowed_collisions_service_;

  bool use_collision_map_;
  std::map<std::string, KnownObject*>                                 collisionObjects_;
  boost::recursive_mutex                                              collision_objects_lock_;

  boost::recursive_mutex collision_map_lock_;
  std::vector<shapes::Shape*> last_collision_map_shapes_;
  std::vector<btTransform>    last_collision_map_poses_;
  //true is include, false is currently masked
  std::vector<bool> last_collision_map_object_mask_;

  mapping_msgs::CollisionMap last_collision_map_;
  double                                                             scale_;
  double                                                             padd_;

  boost::function<void(const mapping_msgs::AttachedCollisionObjectConstPtr &attachedObject)> onAfterAttachCollisionObject_;	
  boost::function<void(const mapping_msgs::CollisionMapConstPtr, bool)> onBeforeMapUpdate_;
  boost::function<void(const mapping_msgs::CollisionMapConstPtr, bool)> onAfterMapUpdate_;
  boost::function<void(const mapping_msgs::CollisionObjectConstPtr)>        onCollisionObjectUpdate_;
    
};
    
	
}

#endif

