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

#ifndef OMPL_ROS_PROJECTION_EVALUATOR_
#define OMPL_ROS_PROJECTION_EVALUATOR_

#include <ompl_ros_interface/helpers/ompl_ros_conversions.h>
#include <ompl_ros_interface/helpers/ompl_ros_exception.h>

#include <ompl/base/ProjectionEvaluator.h>

#include <boost/math/constants/constants.hpp>

namespace ompl_ros_interface
{

/**
 * @class OmplRosProjectionEvaluator
 * @brief A projection evaluator specifically designed for the ROS interface to OMPL while using a compound state space
*/
class OmplRosProjectionEvaluator : public ompl::base::ProjectionEvaluator
{
public:
  /**
   * @brief Default constructor
   * @param state_space - A pointer to the state space to use in construction of the projection evaluator
   * @param evaluator_name - A name to be given to the evaluator
   */  
	OmplRosProjectionEvaluator(const ompl::base::StateSpace *state_space, 
                             const std::string &evaluator_name);
	
  /**
   * @brief Get the dimension of the state used by this projection evaluator
   */
	virtual unsigned int getDimension(void) const;
	
  /**
   * @brief Carry out the projection
   */
	virtual void project(const ompl::base::State *state, 
                       ompl::base::EuclideanProjection &projection) const;

private:
  unsigned int dimension_;
  unsigned int mapping_index_;
  ompl_ros_interface::MAPPING_TYPE mapping_type_;
};
}

#endif
