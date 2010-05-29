/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Ioan Sucan */

#include <planning_models/kinematic_state.h>
#include <motion_planning_msgs/RobotState.h>
#include <motion_planning_msgs/RobotTrajectory.h>

/** \brief Routines for converting between state representations */
namespace robot_state_conversions
{

    
    /** \brief Convert a RobotState to a KinematicState
	\param rs the robot state to convert from
	\param kstate the kinematic state to convert to
	\todo What to do with header information ? */
    void robotStateToKinematicState(const motion_planning_msgs::RobotState &rs, planning_models::KinematicState &kstate);
    

    /** \brief Convert a KinematicState to a RobotState. All joints in the model are set.
	\param kmodel the kinematic model that corresponds to the kinematic state we are reading from
	\param kstate the kinematic state to convert from
	\param rs the robot state to convert to
	\todo What to do with header information ? */
    void kinematicStateToRobotState(const planning_models::KinematicModel &kmodel, const planning_models::KinematicState &kstate,
				    motion_planning_msgs::RobotState &rs);
    
    /** \brief Convert a JointGroup from a KinematicState to a RobotState. Only joints in the group are set
	\param group the JointGroup that corresponds to the data we want to set in the robot state
	\param kstate the kinematic state to convert from
	\param rs the robot state to convert to
	\todo What to do with header information ? */
    void kinematicStateToRobotState(const planning_models::KinematicModel::JointGroup &group, const planning_models::KinematicState &kstate,
				    motion_planning_msgs::RobotState &rs);
    
    /** \brief Convert a point along a trajectory to a KinematicState  
	\param rt the robot trajectory we are converting from
	\param pos the position along the trajectory where this point is to be retrieved
	\param kstate the kinematic state we are converting to
	\todo What to do with header information ? */
    void robotTrajectoryPointToKinematicState(const motion_planning_msgs::RobotTrajectory &rt, const unsigned int pos,
					      planning_models::KinematicState &kstate);
    
    /** \brief Convert a kinematic state to a point along a robot trajectory. 
	\param kmodel the kinematic model that corresponds to the state we are reading from
	\param kstate the kinematic state we are converting from
	\param rt the robot trajectory we are converting to
	\param pos the position along the trajectory where this point is to be set 
	\param length the length of the complete trajectory 
	\todo What to do with header information ? */
    void kinematicStateToRobotTrajectoryPoint(const planning_models::KinematicModel &kmodel, const planning_models::KinematicState &kstate,
					      motion_planning_msgs::RobotTrajectory &rt, const unsigned int pos, const unsigned int length);

    /** \brief Convert a JointGroup to a point along a robot trajectory. 
	\param group the JointGroup that corresponds to the data we want to set in the robot state
	\param kstate the kinematic state we are converting from
	\param rt the robot trajectory we are converting to
	\param pos the position along the trajectory where this point is to be set 
	\param length the length of the complete trajectory 
	\todo What to do with header information ? */
    void kinematicStateToRobotTrajectoryPoint(const planning_models::KinematicModel::JointGroup &group, const planning_models::KinematicState &kstate,
					      motion_planning_msgs::RobotTrajectory &rt, const unsigned int pos, const unsigned int length);

}
