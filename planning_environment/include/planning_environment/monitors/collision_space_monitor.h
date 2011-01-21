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
    return have_map_;
  }
	
  /** \brief Return true if a map update has been received in the last sec seconds. If sec < 10us, this function always returns true. */
  bool isMapUpdated(double sec) const;
	
  /** \brief Wait until a map is received */
  void waitForMap(void) const;	

  /** \brief Return the last update time for the map */
  const ros::Time& lastMapUpdate(void) const
  {
    return last_map_update_;
  }
	
  /** \brief Returns the padding used for pointclouds (for collision checking) */
  double getPointCloudPadd(void) const
  {
    return pointcloud_padd_;
  }
	
  bool getObjectsService(planning_environment_msgs::GetCollisionObjects::Request &req,
                         planning_environment_msgs::GetCollisionObjects::Response &res);    
  /** \brief This function changes the allowed collisions 
      in the collision space to reflect the contents of the
      message.  This allowed collison set will be in place
      until revert is called.
   */

  /** \brief This function gets whatever allowed collision matrix is currently in use
      by the collision space */
  bool getCurrentAllowedCollisionsService(planning_environment_msgs::GetAllowedCollisionMatrix::Request& req,
                                          planning_environment_msgs::GetAllowedCollisionMatrix::Response& res);
  
  /** \brief This service sets the allowed collisions using ordered collision operations.  
      The set collisions will remain in effect until a revert to default is called */
  bool setAllowedCollisionsService(planning_environment_msgs::SetAllowedCollisions::Request& req,
                                   planning_environment_msgs::SetAllowedCollisions::Response& res);

  bool revertAllowedCollisionMatrixToDefaultService(std_srvs::Empty::Request& req,
                                                    std_srvs::Empty::Response& res);

  void setUseCollisionMap(bool use);

protected:
  
  bool createAndPoseShapes(const std::vector<geometric_shapes_msgs::Shape>& orig_shapes,
                           const std::vector<geometry_msgs::Pose>& orig_poses,
                           const std_msgs::Header& header, 
                           const std::string& frame_to,
                           std::vector<shapes::Shape*>& conv_shapes,
                           std::vector<btTransform>& conv_poses);
  

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
	
  bool                                                            have_map_;
  ros::Time                                                       last_map_update_;	
	
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

