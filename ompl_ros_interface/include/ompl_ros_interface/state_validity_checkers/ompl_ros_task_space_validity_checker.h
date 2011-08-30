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

#ifndef OMPL_ROS_TASK_SPACE_VALIDITY_CHECKER_
#define OMPL_ROS_TASK_SPACE_VALIDITY_CHECKER_

#include <ompl_ros_interface/ompl_ros_state_validity_checker.h>
#include <ompl_ros_interface/ompl_ros_state_transformer.h>

namespace ompl_ros_interface
{
/**
 * @class OmplRosTaskSpaceValidityChecker
 * @brief This class implements a state validity checker in task space
 */
class OmplRosTaskSpaceValidityChecker : public ompl_ros_interface::OmplRosStateValidityChecker
{
public:
  /**
   * @brief Default constructor
   * @param si - The space information for this validity checker
   * @param planning_monitor - The planning monitor 
   * @param mapping - The mapping between the ompl state and the  kinematic state that the planning monitor will use
   */
  OmplRosTaskSpaceValidityChecker(ompl::base::SpaceInformation *si, 
                                  planning_environment::CollisionModelsInterface *cmi,
                                  const std::string &parent_frame):
    ompl_ros_interface::OmplRosStateValidityChecker(si,cmi),parent_frame_(parent_frame)
  {
  }
  ~OmplRosTaskSpaceValidityChecker(){}
	
  /**
   * @brief The callback function used by the planners to determine if a state is valid
   * @param ompl_state The state that needs to be checked
   */
  virtual bool 	isValid  (const ompl::base::State *ompl_state) const;	

  /*
    @brief A non-const version of isValid designed to fill in the last error code 
    @param ompl_state The state that needs to be checked
   */
  virtual bool isStateValid(const ompl::base::State *ompl_state);

  /**
   * @brief Instantiate the state transformer
   * @param ompl_state An instance of the state transformer passed in for initialization
   * @return false if an error has occured
   */
  bool setStateTransformer(boost::shared_ptr<ompl_ros_interface::OmplRosStateTransformer> &state_transformer);
	
  /**
   * @brief Return a shared pointer to the state transformer used by this validity checker
   * @return A shared pointer to the state transformer
   */
  boost::shared_ptr<ompl_ros_interface::OmplRosStateTransformer>& getStateTransformer()
  {
    return state_transformer_;
  };

  /**
   * @brief Configure the transformer when a request is received. This is typically a one time configuration 
   * for each planning request.
   * @param kinematic_state - The kinematic state corresponding to the current state of the robot
   * @param joint_state_group - The state of the joint group on which this validity checker will operate
   * @param request - The motion planning request
   */ 
  virtual void configureOnRequest(planning_models::KinematicState *kinematic_state,
                                  planning_models::KinematicState::JointStateGroup *joint_state_group,
                                  const arm_navigation_msgs::GetMotionPlan::Request &request);

protected:	

  arm_navigation_msgs::RobotState robot_state_msg_;
  std::string parent_frame_;
  virtual bool initialize();
  boost::shared_ptr<ompl_ros_interface::OmplRosStateTransformer> state_transformer_;
  ompl_ros_interface::RobotStateToKinematicStateMapping robot_state_to_joint_state_group_mapping_;
};
}
#endif
