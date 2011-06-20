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
*
* Author: Sachin Chitta
*********************************************************************/

#ifndef COLLISION_PROXIMITY_PLANNER_PLUGIN_H_
#define COLLISION_PROXIMITY_PLANNER_PLUGIN_H_


// Plugin
#include <motion_planning_state_refinement/motion_planning_state_refinement.h>

// Other
#include <collision_proximity_planner/collision_proximity_planner.h>

namespace collision_proximity_planner
{
  class CollisionProximityPlannerPlugin : public  motion_planning_state_refinement::MotionPlanningStateRefinement
  {
    public:    
    /** @class
     *  @brief Plugin-able interface for state refinement
     * This class has two main interfaces: 
     * 1. refineState(const motion_planning_msgs::RobotState &robot_state,
     *                const motion_planning_msgs::Constraints &constraints,
     *                motion_planning_msgs::RobotState &group_state)
     * is a single function call that will do everything for you including setting up the collision space, computing 
     * the collision increments and returning a fully refined state. If it cannot achieve this in max_iterations_, it will 
     * return false, otherwise it will return true.
     * 2. The second interface exposes more of the internal functionality and is intended for use with planners where 
     * you might want to setup the collision space and make multiple queries in the most efficient manner possible. The steps
     * you would go through are the following
     * (a) setRobotState - set the robot state (the entire state of the robot) for which you want to perform checks/refinement
     * (b) setGroupState - set the group state and then always use the same order of joints in calling the class. For efficiency, this will cache a mapping from your definition of a joint state and its internal representation. This cache will only be cleared when the clear() function is called.     * 
     */
    CollisionProximityPlannerPlugin();

    bool setGroupName(const std::string &group_name);

    /**
     * @brief Given a robot state, refine the state. Examples of refinement could include projection 
     * onto a constraint manifold, motion away from contact.
     * @param robot_state The full robot state.
     * @param constraints A set of constraints that may need to be applied
     * @param group_state The group state that needs to be refined
     * @return True if a valid refinement was found, false otherwise
     */
    bool refineState(const motion_planning_msgs::RobotState &robot_state,
                     const motion_planning_msgs::Constraints &constraints,
                     motion_planning_msgs::RobotState &group_state);

    /**
     * @brief Set the robot state that you want to check. Note that this locks the collision space 
     * and you will have to call clear() to unlock the space.
     * @param robot_state The full robot state.
     * @return True if setting robot state was successful
     */
    bool setRobotState(const motion_planning_msgs::RobotState &robot_state);

    /**
     * @brief Set a group state. This must be called before you can make multiple queries to refineState below.
     * This function can be used to define a mapping between the group state specified in the argument and 
     * the internal group state for more efficiency.
     * @param group_state 
     */
    bool setGroupState(const motion_planning_msgs::RobotState &group_state);

    /**
     * @brief Given a robot state, refine the state. Examples of refinement could include projection 
     * onto a constraint manifold, motion away from contact, etc.
     * @param constraints A set of constraints that may need to be applied
     * @return True if setting constraints was successful
     */
    bool setConstraints(const motion_planning_msgs::Constraints &constraints);

    /**
     * @brief Given a robot state, refine the state. Examples of refinement could include projection 
     * onto a constraint manifold, motion away from contact.
     * @param group_state The group state that needs to be refined.
     * @return True if a valid refinement was found, false otherwise.
     */
    bool refineState(motion_planning_msgs::RobotState &group_state,
                     motion_planning_msgs::RobotTrajectory &robot_trajectory);

    /**
     * @brief Given a robot state, get the gradient direction to be moved in.
     * @param group_state The group state for which the gradient is expected.
     */
    void getStateGradient(const motion_planning_msgs::RobotState &group_state,
                          double &distance,
                          motion_planning_msgs::RobotState &gradient);

    /**
     * @brief  Clear everything internally and get ready for a whole new request. 
     * This must be called after setRobotState() has been called if a completely new request 
     * needs to be serviced. 
     */
    void clear();

  private:
    collision_proximity_planner::CollisionProximityPlanner planner_;
  };
}

#endif
