/*********************************************************************
*
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
*
* Author: Sachin Chitta
*********************************************************************/
#ifndef MOTION_PLANNING_STATE_REFINEMENT_
#define MOTION_PLANNING_STATE_REFINEMENT_

#include <motion_planning_msgs/RobotState.h>
#include <motion_planning_msgs/Constraints.h>

namespace motion_planning_state_refinement {
  /**
   * @class MotionPlanningStateRefinement
   * @brief Provides an interface for state refinement, e.g. projection onto constraints, 
   * motion away from contact, etc.
   */
  class MotionPlanningStateRefinement
  {
    public:
    MotionPlanningStateRefinement(){};

    bool setGroupName(const std::string &group_name);

      /**
       * @brief Given a robot state, refine the state. Examples of refinement could include projection 
       * onto a constraint manifold, motion away from contact.
       * @param robot_state The full robot state.
       * @param constraints A set of constraints that may need to be applied
       * @param group_state The group state that needs to be refined
       * @return True if a valid refinement was found, false otherwise
       */
      virtual bool refineState(const motion_planning_msgs::RobotState &robot_state,
                               const motion_planning_msgs::Constraints &constraints,
                               motion_planning_msgs::RobotState &group_state) = 0;
      /**
       * @brief  Virtual destructor for the interface
       */
      virtual ~MotionPlanningStateRefinement(){}

  };
};

#endif
