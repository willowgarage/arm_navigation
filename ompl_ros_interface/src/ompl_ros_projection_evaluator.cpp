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

#include <ompl_ros_interface/ompl_ros_projection_evaluator.h>

namespace ompl_ros_interface
{ 
 
OmplRosProjectionEvaluator::OmplRosProjectionEvaluator(const ompl::base::StateSpace *state_space, 
                                                       const std::string &evaluator_name) : ompl::base::ProjectionEvaluator(state_space)
{
  if(!state_space->as<ompl::base::CompoundStateSpace>()->hasSubspace(evaluator_name) && 
     !(evaluator_name == "joint_state"))
  {
    ROS_ERROR("Evaluator name %s does not match any state space name",evaluator_name.c_str());
    return;
  }

  if(evaluator_name == "joint_state")
  {
    if(!state_space->as<ompl::base::CompoundStateSpace>()->hasSubspace("real_vector"))
    {      
      ROS_ERROR("Could not find subspace for defining projection evaluator");
      throw new OMPLROSException();
    }
    mapping_index_ = state_space->as<ompl::base::CompoundStateSpace>()->getSubspaceIndex("real_vector");
    dimension_ = std::min<unsigned int>(state_space->as<ompl::base::CompoundStateSpace>()->as<ompl::base::RealVectorStateSpace>(mapping_index_)->getDimension(),2);
    cellSizes_.resize(dimension_);
    const ompl::base::RealVectorBounds &b = state_space->as<ompl::base::CompoundStateSpace>()->as<ompl::base::RealVectorStateSpace>(mapping_index_)->getBounds();
    for(unsigned int i=0; i < dimension_; i++)
      cellSizes_[i] = (b.high[i] - b.low[i]) / 10.0;
    mapping_type_ = ompl_ros_interface::REAL_VECTOR;
    ROS_DEBUG("Choosing projection evaluator for real vector joints with dimension %d",dimension_);
    return;
  }

  mapping_index_ = state_space->as<ompl::base::CompoundStateSpace>()->getSubspaceIndex(evaluator_name);
  mapping_type_ = ompl_ros_interface::getMappingType(state_space->as<ompl::base::CompoundStateSpace>()->getSubspace(mapping_index_).get());

  if(mapping_type_ == ompl_ros_interface::SO2)
  {
    dimension_ = 1;
    cellSizes_.resize(1);
    cellSizes_[0] = boost::math::constants::pi<double>() / 10.0;
    ROS_DEBUG("Choosing projection evaluator for SO2 state space %s",evaluator_name.c_str());
  }
  else if(mapping_type_ == ompl_ros_interface::SE2)
  {
    dimension_ = 2;
    cellSizes_.resize(2);
    const ompl::base::RealVectorBounds &b = state_space->as<ompl::base::CompoundStateSpace>()->as<ompl::base::SE2StateSpace>(mapping_index_)->as<ompl::base::RealVectorStateSpace>(0)->getBounds();
    cellSizes_[0] = (b.high[0] - b.low[0]) / 10.0;
    cellSizes_[1] = (b.high[1] - b.low[1]) / 10.0;      
    ROS_INFO("Choosing projection evaluator for SE2 state space %s",evaluator_name.c_str());
  }
  else if(mapping_type_ == ompl_ros_interface::SO3)
  {
    dimension_ = 3;
    cellSizes_.resize(3);
    cellSizes_[0] = boost::math::constants::pi<double>() / 10.0;
    cellSizes_[1] = boost::math::constants::pi<double>() / 10.0;
    cellSizes_[2] = boost::math::constants::pi<double>() / 10.0;
    ROS_INFO("Choosing projection evaluator for SO3 state space %s",evaluator_name.c_str());
  }
  else if(mapping_type_ == ompl_ros_interface::SE3)
  {
    dimension_ = 3;
    cellSizes_.resize(3);
    const ompl::base::RealVectorBounds &b = state_space->as<ompl::base::CompoundStateSpace>()->as<ompl::base::SE3StateSpace>(mapping_index_)->as<ompl::base::RealVectorStateSpace>(0)->getBounds();
    cellSizes_[0] = (b.high[0] - b.low[0]) / 10.0;
    cellSizes_[1] = (b.high[1] - b.low[1]) / 10.0;
    cellSizes_[2] = (b.high[2] - b.low[2]) / 10.0;
    ROS_INFO("Choosing projection evaluator for SE3 state space %s",evaluator_name.c_str());
  }
  else
  {
    ROS_ERROR("Could not initialize projection evaluator. A projection evaluator needs to be defined as either a combination of revolute joints with joint limits, or a continuous, spherical, planar of floating joint. ");
    throw new OMPLROSException();
  }
};
	
unsigned int OmplRosProjectionEvaluator::getDimension(void) const
{
  return dimension_;
};
	
void OmplRosProjectionEvaluator::project(const ompl::base::State *state, ompl::base::EuclideanProjection &projection) const
{
  if(mapping_type_ == ompl_ros_interface::REAL_VECTOR)
  {
    for(unsigned int i=0; i < dimension_; i++)
      projection[i] = state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(mapping_index_)->values[i];
  }
  if(mapping_type_ == ompl_ros_interface::SO2)
  {
    projection[0] = state->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(mapping_index_)->value;
  }
  else if(mapping_type_ == ompl_ros_interface::SE2)
  {
    const double *se2_vals = state->as<ompl::base::CompoundState>()->as<ompl::base::SE2StateSpace::StateType>(mapping_index_)->as<ompl::base::RealVectorStateSpace::StateType>(0)->values;
    projection[0] = se2_vals[0];
    projection[1] = se2_vals[1];
  }
  else if(mapping_type_ == ompl_ros_interface::SO3)
  {
    projection[0] = state->as<ompl::base::CompoundState>()->as<ompl::base::SO3StateSpace::StateType>(mapping_index_)->x;
    projection[1] = state->as<ompl::base::CompoundState>()->as<ompl::base::SO3StateSpace::StateType>(mapping_index_)->y;
    projection[2] = state->as<ompl::base::CompoundState>()->as<ompl::base::SO3StateSpace::StateType>(mapping_index_)->z;
  }
  else if(mapping_type_ == ompl_ros_interface::SE3)
  {
    const double *se3_vals = state->as<ompl::base::CompoundState>()->as<ompl::base::SE3StateSpace::StateType>(mapping_index_)->as<ompl::base::RealVectorStateSpace::StateType>(0)->values;
    projection[0] = se3_vals[0];
    projection[1] = se3_vals[1];
    projection[2] = se3_vals[2];
  }
};

}
