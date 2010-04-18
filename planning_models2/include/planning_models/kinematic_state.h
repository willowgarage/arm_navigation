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

#ifndef PLANNING_MODELS_KINEMATIC_STATE_
#define PLANNING_MODELS_KINEMATIC_STATE_

#include "planning_models/kinematic_model.h"

/** \brief Main namespace */
namespace planning_models
{
    
    /** \brief A class that can hold the named parameters of this planning model */
    class KinematicState
    {
    public:
	KinematicState(const KinematicModel *model);
	KinematicState(const KinematicState &sp);

	~KinematicState(void);
	
	KinematicState &operator=(const KinematicState &rhs);
	
	bool operator==(const KinematicState &rhs) const;

	/** \brief Get the dimension of a state */
	unsigned int getDimension(void) const;
	
	/** \brief Construct a default state: each value at 0.0, if
	    within bounds. Otherwise, select middle point. */
	void defaultState(void);
	
	/** \brief Construct a random state (within bounds) */
	void randomState(void);

	/** \brief Construct a random state for a group */
	void randomStateGroup(const std::string &group);

	/** \brief Construct a random state for a group */
	void randomStateGroup(const KinematicModel::JointGroup *group);

	/** \brief Perturb state. Each dimension is perturbed by a factor of its range */
	void perturbState(double factor);

	/** \brief Perturb state of a group. Each dimension is perturbed by a factor of its range */
	void perturbStateGroup(double factor, const std::string &group);

	/** \brief Perturb state of a group. Each dimension is perturbed by a factor of its range */
	void perturbStateGroup(double factor, const KinematicModel::JointGroup *group);
	
	/** \brief Update parameters so that they are within the specified bounds */
	void enforceBounds(void);
	
	/** \brief Update parameters so that they are within the specified bounds (for a specific group) */
	void enforceBoundsGroup(const std::string &group);

	/** \brief Update parameters so that they are within the specified bounds (for a specific group) */
	void enforceBoundsGroup(const KinematicModel::JointGroup *group);

	/** \brief Check whether parameters are within the specified bounds (for a specific group) */
	bool checkBoundsGroup(const std::string &group) const;

	/** \brief Check whether parameters are within the specified bounds (for a specific group) */
	bool checkBoundsGroup(const KinematicModel::JointGroup *group) const;
	
	/** \brief Check whether parameters are within the specified bounds (for a specific joint) */
	bool checkBoundsJoint(const std::string &name) const;
	
	/** \brief Check whether parameters are within the specified bounds (for a specified set of joints) */
	bool checkBoundsJoints(const std::vector<std::string> &names) const;
	
	/** \brief Check whether parameters are within the specified bounds (for all joints) */
	bool checkBounds(void) const;
		
	/** \brief Mark all values as unseen */
	void reset(void);
	
	/** \brief Mark all values in a group as unseen */
	void resetGroup(const KinematicModel::JointGroup *group);
	
	/** \brief Mark all values in a group as unseen */
	void resetGroup(const std::string &group);
	
	/** \brief Set all the parameters to a given value */
	void setAll(const double value);
	
	/** \brief Set all the parameters from a group to a given value */
	void setAllInGroup(const double value, const std::string &group);
	
	/** \brief Set all the parameters from a group to a given value */
	void setAllInGroup(const double value, const KinematicModel::JointGroup *group);
	
	/** \brief Set the parameters for the complete robot. */
	bool setParams(const std::vector<double> &params);
	
	/** \brief Set the parameters for the complete robot. */
	bool setParams(const double *params);
	
	/** \brief Set the parameters for a given group. Return true if
	    any change was observed in either of the set
	    values. */
	bool setParamsGroup(const std::vector<double> &params, const std::string &group);
	
	/** \brief Set the parameters for a given group. Return true if
	    any change was observed in either of the set
	    values. */
	bool setParamsGroup(const std::vector<double> &params, const KinematicModel::JointGroup *group);
	
	/** \brief Set the parameters for a given group. Return true if
	    any change was observed in either of the set
	    values. */
	bool setParamsGroup(const double *params, const std::string &group);
	
	/** \brief Set the parameters for a given group. Return true if
	    any change was observed in either of the set
	    values. */
	bool setParamsGroup(const double *params, const KinematicModel::JointGroup *group);
	
	/** \brief Given the name of a joint, set the values of the
	    parameters describing the joint. Return true if any
	    change was observed in the set value */
	bool setParamsJoint(const double *params, const std::string &name);
	
	/** \brief Given the name of a joint, set the values of the
	    parameters describing the joint. Return true if any
	    change was observed in the set value */
	bool setParamsJoint(const std::vector<double> &params, const std::string &name);

	/** \brief Given the names of some joints, set the values of the
	    parameters describing the joints. Return true if any
	    change was observed in the set values */
	bool setParamsJoints(const double *params, const std::vector<std::string> &names);

	/** \brief Given the names of some joints, set the values of the
	    parameters describing the joints. Return true if any
	    change was observed in the set values */
	bool setParamsJoints(const std::vector<double> &params, const std::vector<std::string> &names);
	
	/** \brief Given the name of a joint, get the values of the
	    parameters describing the joint. */
	const double* getParamsJoint(const std::string &name) const;
	
	/** \brief Return the current value of the params */
	const double* getParams(void) const;
	
	/** \brief Copy the parameters for a given group to a destination address */
	void copyParamsGroup(std::vector<double> &params, const std::string &group) const;
	
	/** \brief Copy the parameters for a given group to a destination address */
	void copyParamsGroup(std::vector<double> &params, const KinematicModel::JointGroup *group) const;
	
	/** \brief Copy the parameters for a given group to a destination address */
	void copyParamsGroup(double *params, const std::string &group) const;
	
	/** \brief Copy the parameters for a given group to a destination address */
	void copyParamsGroup(double *params, const KinematicModel::JointGroup *group) const;
	
	/** \brief Copy all parameters to a destination address */
	void copyParams(double *params) const;
	
	/** \brief Copy all parameters to a destination address */
	void copyParams(std::vector<double> &params) const;
	
	/** \brief Copy the parameters describing a given joint */
	void copyParamsJoint(double *params, const std::string &name) const;
	
	/** \brief Copy the parameters describing a given joint */
	void copyParamsJoint(std::vector<double> &params, const std::string &name) const;
	
	/** \brief Copy the parameters describing a given set of joints */
	void copyParamsJoints(double *params, const std::vector<std::string> &names) const;

	/** \brief Copy the parameters describing a given set of joints */
	void copyParamsJoints(std::vector<double> &params, const std::vector<std::string> &names) const;
	
	/** \brief Check if all params for a joint were seen */
	bool seenJoint(const std::string &name) const;
	
	/** \brief Check if all params in a group were seen */
	bool seenAllGroup(const std::string &group) const;
	
	/** \brief Check if all params in a group were seen */
	bool seenAllGroup(const KinematicModel::JointGroup *group) const;
	
	/** \brief Check if all params were seen */
	bool seenAll(void) const;

	/** \brief Print the data from the state to screen */
	void print(std::ostream &out = std::cout) const;
	
	/** \brief Print the missing joint index values */
	void missing(std::ostream &out = std::cout);
	
    protected:
	
	/** \brief Copy data from another instance of this class */
	void copyFrom(const KinematicState &sp);
	
	const KinematicModel *owner_;
	double               *params_;
	std::vector<bool>     seen_;
    };
    
}

#endif
