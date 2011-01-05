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

#ifndef PLANNING_ENVIRONMENT_MONITORS_KINEMATIC_MODEL_STATE_MONITOR_
#define PLANNING_ENVIRONMENT_MONITORS_KINEMATIC_MODEL_STATE_MONITOR_

#include "planning_environment/models/robot_models.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/JointState.h>
#include <motion_planning_msgs/RobotState.h>
#include <boost/bind.hpp>
#include <vector>
#include <string>
#include <map>
#include <planning_models/kinematic_state.h>

namespace planning_environment
{

/** \brief @b KinematicModelStateMonitor is a class that monitors the robot state for the kinematic model defined in @b RobotModels
    If the pose is not included, the robot state is the frame of the link it attaches to the world. If the pose is included,
    the frame of the robot is the one in which the pose is published.
*/
class KinematicModelStateMonitor
{
public:

  KinematicModelStateMonitor(RobotModels *rm, tf::TransformListener *tf) : nh_("~")
  {
    rm_ = rm;
    tf_ = tf;
    setupRSM();
  }

  virtual ~KinematicModelStateMonitor(void)
  {
  }

  /** \brief Start the state monitor. By default, the monitor is started after creation */
  void startStateMonitor(void);
	
  /** \brief Stop the state monitor. */
  void stopStateMonitor(void);
	
  /** \brief Check if the state monitor is currently started */
  bool isStateMonitorStarted(void) const
  {
    return stateMonitorStarted_;
  }	
  
  /** Once everything is functional this advertises any services that the monitor may offer*/
  virtual void advertiseServices() {
    //kmsm doesn't have services to advertise
  }


  /** \brief Define a callback for when the state is changed */
  void setOnStateUpdateCallback(const boost::function<void(void)> &callback)
  {
    onStateUpdate_ = callback;
  }

  /** \brief Get the kinematic model that is being used to check for validity */
  boost::shared_ptr<planning_models::KinematicModel> getKinematicModel(void) const
  {
    return kmodel_;
  }

  /** \brief Get the instance of @b RobotModels that is being used */
  RobotModels* getRobotModels(void) const
  {
    return rm_;
  }

  /** \brief Return the maintained robot velocity (square root of sum of squares of velocity at each joint)*/
  double getTotalVelocity(void) const
  {
    return robotVelocity_;
  }

  /** \brief Return the transform listener */
  tf::TransformListener *getTransformListener(void) const
  {
    return tf_;
  }
	
  /** \brief Return the frame id of the state */
  const std::string& getRobotFrameId(void) const
  {
    return robot_frame_;
  }

  /** \brief Return the world frame id */
  const std::string& getWorldFrameId(void) const
  {
    return kmodel_->getRoot()->getParentFrameId();
  }

  /** \brief Return true if a full joint state has been received (including pose, if pose inclusion is enabled) */
  bool haveState(void) const
  {
    return haveJointState_ && havePose_;
  }
	
  /** \brief Return the time of the last state update */
  const ros::Time& lastJointStateUpdate(void) const
  {
    return lastJointStateUpdate_;
  }

  /** \brief Return the time of the last pose update */
  const ros::Time& lastPoseUpdate(void) const
  {
    return lastPoseUpdate_;
  }
	
  /** \brief Wait until a full joint state is received (including pose, if pose inclusion is enabled) */
  void waitForState(void) const;

  /** \brief Return true if a joint state has been received in the last sec seconds. If sec < 10us, this function always returns true. */
  bool isJointStateUpdated(double sec) const;

  /** \brief Return true if a pose has been received in the last
      sec seconds. If sec < 10us, this function always returns
      true. */
  bool isPoseUpdated(double sec) const;
	
  /** \brief Output the current state as ROS INFO */
  void printRobotState(void) const;
  
  /** \brief Sets the model used for collision/valditity checking to the current state values*/
  void setStateValuesFromCurrentValues(planning_models::KinematicState& state) const;

  void setRobotStateAndComputeTransforms(const motion_planning_msgs::RobotState &robot_state,
                                         planning_models::KinematicState& state) const;

  bool getCurrentRobotState(motion_planning_msgs::RobotState &robot_state) const;

  bool setKinematicStateToTime(const ros::Time& time,
                               planning_models::KinematicState& state) const;

  bool convertKinematicStateToRobotState(const planning_models::KinematicState& state, 
                                         motion_planning_msgs::RobotState &robot_state) const;

  bool getCachedJointStateValues(const ros::Time& time, std::map<std::string, double>& ret_map) const;

  bool allJointsUpdated(ros::Duration dur = ros::Duration()) const;

  //need to pass by value for thread safety
  std::map<std::string, double> getCurrentJointStateValues() const {
    current_joint_values_lock_.lock();
    std::map<std::string, double> ret_values = current_joint_state_map_;
    current_joint_values_lock_.unlock();
    return ret_values;
  }

protected:

  void setupRSM(void);
  void jointStateCallback(const sensor_msgs::JointStateConstPtr &jointState);

  std::list<std::pair<ros::Time, std::map<std::string, double> > >joint_state_map_cache_;

  std::map<std::string, ros::Time> last_joint_update_;
  std::map<std::string, double> current_joint_state_map_;

  mutable boost::recursive_mutex current_joint_values_lock_;

  double joint_state_cache_time_;
  double joint_state_cache_allowed_difference_;

  RobotModels                     *rm_;
      
  boost::shared_ptr<planning_models::KinematicModel> kmodel_;
  std::string                      planarJoint_;
  std::string                      floatingJoint_;

  bool                             stateMonitorStarted_;
  bool printed_out_of_date_;
	
  ros::NodeHandle                  nh_;
  ros::NodeHandle                  root_handle_;
  ros::Subscriber                  jointStateSubscriber_;
  tf::TransformListener           *tf_;

  double                           robotVelocity_;
  tf::Pose                         pose_;
  std::string                      robot_frame_;

  boost::function<void(void)>      onStateUpdate_;

  bool                             havePose_;
  bool                             haveJointState_;
  ros::Time                        lastJointStateUpdate_;
  ros::Time                        lastPoseUpdate_;
};

}

#endif
