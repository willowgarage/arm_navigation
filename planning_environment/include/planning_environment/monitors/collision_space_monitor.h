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

#include <planning_environment/models/collision_models.h>
#include <planning_environment/monitors/kinematic_model_state_monitor.h>
#include <arm_navigation_msgs/CollisionMap.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/AttachedCollisionObject.h>
#include <boost/thread/mutex.hpp>
#include <std_srvs/Empty.h>

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
	
  /** \brief Check if the environment monitor is currently started */
  bool isEnvironmentMonitorStarted(void) const
  {
    return envMonitorStarted_;
  }	

  /** \brief Return the instance of the environment model maintained */
  const collision_space::EnvironmentModel* getEnvironmentModel(void) const
  {
    return cm_->getCollisionSpace();
  }
	
  /** \brief Return the instance of collision models that is being used */
  CollisionModels* getCollisionModels(void) const
  {
    return cm_;
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
	
  void setUseCollisionMap(bool use);

protected:
  
  void setupCSM(void);
  void updateCollisionSpace(const arm_navigation_msgs::CollisionMapConstPtr &collisionMap, bool clear);
  void collisionMapAsSpheres(const arm_navigation_msgs::CollisionMapConstPtr &collisionMap,
                             std::vector<shapes::Shape*> &spheres, std::vector<tf::Transform> &poses);
  void collisionMapAsBoxes(const arm_navigation_msgs::CollisionMap &collisionMap,
                           std::vector<shapes::Shape*> &boxes, std::vector<tf::Transform> &poses);
  void collisionMapAsBoxes(const arm_navigation_msgs::CollisionMapConstPtr &collisionMap,
                           std::vector<shapes::Shape*> &boxes, std::vector<tf::Transform> &poses);
  void collisionMapCallback(const arm_navigation_msgs::CollisionMapConstPtr &collisionMap);
  void collisionMapUpdateCallback(const arm_navigation_msgs::CollisionMapConstPtr &collisionMap);
  void collisionObjectCallback(const arm_navigation_msgs::CollisionObjectConstPtr &collisionObject);
  virtual bool attachObjectCallback(const arm_navigation_msgs::AttachedCollisionObjectConstPtr &attachedObject);

  CollisionModels *cm_;
  double pointcloud_padd_;
	
  bool envMonitorStarted_;
	
  bool have_map_;
  ros::Time last_map_update_;	
	
  message_filters::Subscriber<arm_navigation_msgs::CollisionMap> *collisionMapSubscriber_;
  tf::MessageFilter<arm_navigation_msgs::CollisionMap> *collisionMapFilter_;
  message_filters::Subscriber<arm_navigation_msgs::CollisionMap> *collisionMapUpdateSubscriber_;
  tf::MessageFilter<arm_navigation_msgs::CollisionMap> *collisionMapUpdateFilter_;
  message_filters::Subscriber<arm_navigation_msgs::CollisionObject> *collisionObjectSubscriber_;
  tf::MessageFilter<arm_navigation_msgs::CollisionObject> *collisionObjectFilter_;

  message_filters::Subscriber<arm_navigation_msgs::AttachedCollisionObject> *attachedCollisionObjectSubscriber_;

  bool use_collision_map_;

  boost::recursive_mutex collision_map_lock_;
};
    
	
}

#endif

