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

/** \author Ioan Sucan, Sachin Chitta */

#ifndef PLANNING_STATE_
#define PLANNING_STATE_

#include "planning_models/model.h"

/** \brief Main namespace */
namespace planning_models
{
    
/** \brief A class that can hold the named joint values of this planning model */
class RobotState
{
public:
  RobotState(boost::shared_ptr<const PlanningModel> model);
  RobotState(const RobotState &sp);

  ~RobotState(void);
	
  RobotState &operator=(const RobotState &rhs);
	
	bool operator==(const RobotState &rhs) const;
	
	/** \brief Construct a default state: each value at 0.0, if
	    within bounds. Otherwise, select middle point. */
	void defaultState(void);
	
	/** \brief Construct a random state (within bounds) */
	void randomState(void);

	/** \brief Construct a random state for a group */
	void randomStateGroup(const std::string &group);

	/** \brief Construct a random state for a group */
	void randomStateGroup(boost::shared_ptr<const JointGroup> group);

	/** \brief Perturb state. Each dimension is perturbed by a factor of its range */
	void perturbState(double factor);

	/** \brief Perturb state of a group. Each dimension is perturbed by a factor of its range */
	void perturbStateGroup(double factor, const std::string &group);

	/** \brief Perturb state of a group. Each dimension is perturbed by a factor of its range */
	void perturbStateGroup(double factor, boost::shared_ptr<const JointGroup> group);
	
	/** \brief Update joint values so that they are within the specified bounds */
	void enforceJointLimits(void);
	
	/** \brief Update joint values so that they are within the specified bounds (for a specific group) */
	void enforceJointLimits(const std::string &group);

	/** \brief Update joint values so that they are within the specified bounds (for a specific group) */
	void enforceJointLimits(boost::shared_ptr<const JointGroup> group);

	/** \brief check whether joint values are within the specified bounds (for a specific group) */
	bool checkJointLimits(const std::string &group) const;

	/** \brief check whether joint values are within the specified bounds (for a specific group) */
	bool checkJointLimits(boost::shared_ptr<const JointGroup> group) const;
	
	/** \brief check whether joint values are within the specified bounds (for a specified set of joints) */
  bool checkJointLimits(const std::vector<std::string> &joint_names) const;

	/** \brief check whether joint values are within the specified bounds (for all joints) */
  bool checkJointLimits() const;

	/** \brief Clear all updated flags */
	void clearUpdatedFlags(void);
	
	/** \brief Clear all updated flags in a group */
	void clearUpdatedFlags(boost::shared_ptr<const JointGroup> group);
	
	/** \brief Clear all updated flags in a group */
	void clearUpdatedFlags(const std::string &group);




	/** \brief Set all the joint values to a given value */
	void setJointState(const double value);
	
	/** \brief Set all the joint values from a group to a given value */
	void setJointState(const double value, const std::string &group);
	
	/** \brief Set all the joint values from a group to a given value */
	void setJointState(const double value, boost::shared_ptr<const planning_models::JointGroup> group);
			
	/** \brief Set the joint values for a given group. Return true if
	    any change was observed in either of the set
	    values. */
	bool setJointState(const std::vector<double> &values, const std::string &group);
	
	/** \brief Set the joint values for a given group. Return true if
	    any change was observed in either of the set
	    values. */
	bool setJointState(const std::vector<double> &values, boost::shared_ptr<const planning_models::JointGroup> group);
		
	/** \brief Set the joint values for a given group. Return true if
	    any change was observed in either of the set
	    values. */
	bool setJointState(const double *values, boost::shared_ptr<const planning_models::JointGroup> group);
	
	/** \brief Given the name of a joint, set the joint values. Return true if any
	    change was observed in the set value */
	bool setJointState(const double value, const std::string &name);
	
	/** \brief Given the names of some joints, set the values of the
	    joint values describing the joints. Return true if any
	    change was observed in the set values */
	bool setJointState(const std::vector<double> &values, const std::vector<std::string> &names);

	
	/** \brief Set all the planar joint values from a group to a given state */
	void setGroupPlanarJointState(const PlanarJointState &state, const std::string &group);
	
	/** \brief Set all the joint values from a group to a given value */
	void setPlanarJointState(const PlanarJointState &state, boost::shared_ptr<const planning_models::JointGroup> group);
			
	/** \brief Set the joint values for a given group. Return true if
	    any change was observed in either of the set
	    values. */
	bool setPlanarJointState(const std::vector<PlanarJointState> &states, const std::string &group);
	
	/** \brief Set the joint values for a given group. Return true if
	    any change was observed in either of the set
	    values. */
	bool setPlanarJointState(const std::vector<PlanarJointState> &states, boost::shared_ptr<const planning_models::JointGroup> group);
	
	/** \brief Given the name of a joint, set the joint values. Return true if any
	    change was observed in the set value */
	bool setPlanarJointState(const PlanarJointState &state, const std::string &name);
	
	/** \brief Given the names of some joints, set the values of the
	    joint values describing the joints. Return true if any
	    change was observed in the set values */
	bool setPlanarJointState(const std::vector<PlanarJointState> &states, const std::vector<std::string> &names);


	/** \brief Set all the planar joint values from a group to a given state */
	void setGroupFloatingJointState(const FloatingJointState &state, const std::string &group);
	
	/** \brief Set all the joint values from a group to a given value */
	void setFloatingJointState(const FloatingJointState &state, boost::shared_ptr<const planning_models::JointGroup> group);
			
	/** \brief Set the joint values for a given group. Return true if
	    any change was observed in either of the set
	    values. */
	bool setFloatingJointState(const std::vector<FloatingJointState> &states, const std::string &group);
	
	/** \brief Set the joint values for a given group. Return true if
	    any change was observed in either of the set
	    values. */
	bool setFloatingJointState(const std::vector<FloatingJointState> &states, boost::shared_ptr<const planning_models::JointGroup> group);
	
	/** \brief Given the name of a joint, set the joint values. Return true if any
	    change was observed in the set value */
	bool setFloatingJointState(const FloatingJointState &state, const std::string &name);
	
	/** \brief Given the names of some joints, set the values of the
	    joint values describing the joints. Return true if any
	    change was observed in the set values */
	bool setFloatingJointState(const std::vector<FloatingJointState> &states, const std::vector<std::string> &names);



	
	/** \brief Get the joint values for a given group*/
	void getGroupJointState(std::vector<double> &values, const std::string &group) const;
	
	/** \brief Get the joint values for a given group*/
	void getGroupJointState(std::vector<double> &values, boost::shared_ptr<const planning_models::JointGroup> group) const;
	
	/** \brief Get the joint values for a given group*/
	void getGroupJointState(double *values, const std::string &group) const;
	
	/** \brief Get the joint values for a given group*/
	void getGroupJointState(double *values, boost::shared_ptr<const planning_models::JointGroup> group) const;
	
	/** \brief Get all the joint values*/
	void getJointState(double *values) const;
	
	/** \brief Get all the joint values*/
	void getJointState(std::vector<double> &values) const;
	
	/** \brief Get the joint value for a single joint*/
	void getJointState(double &value, const std::string &name) const;
	
	/** \brief Get the joint value for a set of joints*/
	void getJointState(double *values, const std::vector<std::string> &names) const;

	/** \brief Get the joint value for a set of joints*/
	void getJointState(std::vector<double> &values, const std::vector<std::string> &names) const;




	/** \brief Get the planar joint states for a given joint*/
	void getPlanarJointState(planning_models::PlanarJointState &state, const std::string &name) const;
	
	/** \brief Get the planar joint states for a set of joints*/
	void getPlanarJointState(std::vector<planning_models::PlanarJointState> &states, const std::vector<std::string> &names) const;

	/** \brief Get the planar joint states for a given group*/
	void getGroupPlanarJointState(std::vector<planning_models::PlanarJointState> &states, const std::string &group) const;
	
	/** \brief Get the planar joint states for a given group*/
	void getGroupPlanarJointState(std::vector<planning_models::PlanarJointState> &states, boost::shared_ptr<const planning_models::JointGroup> group) const;



	/** \brief Get the floating joint states for a given joint*/
	void getFloatingJointState(planning_models::FloatingJointState &state, const std::string &name) const;
	
	/** \brief Get the floating joint states for a set of joints*/
	void getFloatingJointState(std::vector<planning_models::FloatingJointState> &states, const std::vector<std::string> &names) const;

	/** \brief Get the floating joint states for a given group*/
	void getGroupFloatingJointState(std::vector<planning_models::FloatingJointState> &states, const std::string &group) const;
	
	/** \brief Get the planar joint states for a given group*/
	void getGroupFloatingJointState(std::vector<planning_models::FloatingJointState> &values, boost::shared_ptr<const planning_models::JointGroup> group) const;


	
	/** \brief Check if a joint state was updated */
	bool isJointStateUpdated(const std::string &name) const;
	
	/** \brief Check if all joint states in a group were updated */
	bool isGroupStateUpdated(const std::string &group) const;
	
	/** \brief Check if all joint states in a group were updated */
	bool isGroupStateUpdated(boost::shared_ptr<const planning_models::JointGroup> group) const;
i	
	/** \brief Check if all joint states in a model were updated */
	bool isModelStateUpdated(bool print_errors = false) const;

  bool isFixedJoint(const std::string &name) const;
	
  bool isFixedJoint(unsigned int i) const;

	/** \brief Print the data from the state to screen */
	void print(std::ostream &out = std::cout) const;
	
	/** \brief Print the missing joint index values */
	void missing(std::ostream &out = std::cout);
	
protected:
	
	/** \brief Copy data from another instance of this class */
	void copyFrom(const RobotState &sp);
	
  boost::shared_ptr<PlanningModel> parent_model_;
  std::vector<planning_models::JointState> joint_state_;
  std::vector<planning_models::PlanarJointState> planar_joint_state_;
  std::vector<planning_models::FloatingJointState> floating_joint_state_;
  std::vector<bool> updated_joints_;
};
    
}

#endif
