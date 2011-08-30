/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2010, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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

/** \author Sachin Chitta */

#ifndef OMPL_ROS_JOINT_STATE_VALIDITY_CHECKER_
#define OMPL_ROS_JOINT_STATE_VALIDITY_CHECKER_

#include <ompl_ros_interface/ompl_ros_state_validity_checker.h>

namespace ompl_ros_interface
{
/**
 * @class OmplRosJointStateValidityChecker
 * @brief This class implements a state validity checker in joint space
 */
class OmplRosJointStateValidityChecker : public ompl_ros_interface::OmplRosStateValidityChecker
{
public:
  /**
   * @brief Default constructor
   * @param si - The space information for this validity checker
   * @param planning_monitor - The planning monitor 
   * @param mapping - The mapping between the ompl state and the  kinematic state that the planning monitor will use
   */
  OmplRosJointStateValidityChecker(ompl::base::SpaceInformation *si, 
                                   planning_environment::CollisionModelsInterface* cmi,
                                   const ompl_ros_interface::OmplStateToKinematicStateMapping &mapping) : 
    ompl_ros_interface::OmplRosStateValidityChecker(si,cmi), 
    ompl_state_to_kinematic_state_mapping_(mapping)
  {
  }
  ~OmplRosJointStateValidityChecker(){}
	
  /**
   * @brief The callback function used by the planners to determine if a state is valid
   * @param ompl_state The state that needs to be checked
   */
  virtual bool 	isValid  (const ompl::base::State *ompl_state) const;	

  /**
   * @brief A non-const version of isValid designed to fill in the last error code 
   * @param ompl_state The state that needs to be checked
   */
  virtual bool isStateValid(const ompl::base::State *ompl_state);
	
protected:	
  ompl_ros_interface::OmplStateToKinematicStateMapping ompl_state_to_kinematic_state_mapping_;    

  //a cached pose that will be multiplied to every input pose
  //necessary since the input may be in a frame that's not the one that the kinematics solver is working in
  geometry_msgs::Pose cached_transform_pose_;
};
}
#endif
