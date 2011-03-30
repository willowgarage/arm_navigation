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
 
OmplRosProjectionEvaluator::OmplRosProjectionEvaluator(const ompl::base::StateManifold *manifold, 
                                                       const std::string &evaluator_name) : ompl::base::ProjectionEvaluator(manifold)
{
  if(!manifold->as<ompl::base::CompoundStateManifold>()->hasSubManifold(evaluator_name) && 
     !(evaluator_name == "joint_state"))
  {
    ROS_ERROR("Evaluator name %s does not match any manifold name",evaluator_name.c_str());
    return;
  }

  if(evaluator_name == "joint_state")
  {
    if(!manifold->as<ompl::base::CompoundStateManifold>()->hasSubManifold("real_vector"))
    {      
      ROS_ERROR("Could not find sub manifold for defining projection evaluator");
      throw new OMPLROSException();
    }
    mapping_index_ = manifold->as<ompl::base::CompoundStateManifold>()->getSubManifoldIndex("real_vector");
    dimension_ = std::min<unsigned int>(manifold->as<ompl::base::CompoundStateManifold>()->as<ompl::base::RealVectorStateManifold>(mapping_index_)->getDimension(),2);
    cellDimensions_.resize(dimension_);
    const ompl::base::RealVectorBounds &b = manifold->as<ompl::base::CompoundStateManifold>()->as<ompl::base::RealVectorStateManifold>(mapping_index_)->getBounds();
    for(unsigned int i=0; i < dimension_; i++)
      cellDimensions_[i] = (b.high[i] - b.low[i]) / 10.0;
    mapping_type_ = ompl_ros_interface::REAL_VECTOR;
    ROS_DEBUG("Choosing projection evaluator for real vector joints with dimension %d",dimension_);
    return;
  }

  mapping_index_ = manifold->as<ompl::base::CompoundStateManifold>()->getSubManifoldIndex(evaluator_name);
  mapping_type_ = ompl_ros_interface::getMappingType(manifold->as<ompl::base::CompoundStateManifold>()->getSubManifold(mapping_index_).get());

  if(mapping_type_ == ompl_ros_interface::SO2)
  {
    dimension_ = 1;
    cellDimensions_.resize(1);
    cellDimensions_[0] = boost::math::constants::pi<double>() / 10.0;
    ROS_DEBUG("Choosing projection evaluator for SO2 manifold %s",evaluator_name.c_str());
  }
  else if(mapping_type_ == ompl_ros_interface::SE2)
  {
    dimension_ = 2;
    cellDimensions_.resize(2);
    const ompl::base::RealVectorBounds &b = manifold->as<ompl::base::CompoundStateManifold>()->as<ompl::base::SE2StateManifold>(mapping_index_)->as<ompl::base::RealVectorStateManifold>(0)->getBounds();
    cellDimensions_[0] = (b.high[0] - b.low[0]) / 10.0;
    cellDimensions_[1] = (b.high[1] - b.low[1]) / 10.0;      
    ROS_DEBUG("Choosing projection evaluator for SE2 manifold %s",evaluator_name.c_str());
  }
  else if(mapping_type_ == ompl_ros_interface::SO3)
  {
    dimension_ = 3;
    cellDimensions_.resize(3);
    cellDimensions_[0] = boost::math::constants::pi<double>() / 10.0;
    cellDimensions_[1] = boost::math::constants::pi<double>() / 10.0;
    cellDimensions_[2] = boost::math::constants::pi<double>() / 10.0;
    ROS_DEBUG("Choosing projection evaluator for SO3 manifold %s",evaluator_name.c_str());
  }
  else if(mapping_type_ == ompl_ros_interface::SE3)
  {
    dimension_ = 3;
    cellDimensions_.resize(3);
    const ompl::base::RealVectorBounds &b = manifold->as<ompl::base::CompoundStateManifold>()->as<ompl::base::SE3StateManifold>(mapping_index_)->as<ompl::base::RealVectorStateManifold>(0)->getBounds();
    cellDimensions_[0] = (b.high[0] - b.low[0]) / 10.0;
    cellDimensions_[1] = (b.high[1] - b.low[1]) / 10.0;
    cellDimensions_[2] = (b.high[2] - b.low[2]) / 10.0;
    ROS_DEBUG("Choosing projection evaluator for SE3 manifold %s",evaluator_name.c_str());
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
      projection.values[i] = state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateManifold::StateType>(mapping_index_)->values[i];
  }
  if(mapping_type_ == ompl_ros_interface::SO2)
  {
    projection.values[0] = state->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateManifold::StateType>(mapping_index_)->value;
  }
  else if(mapping_type_ == ompl_ros_interface::SE2)
  {
    memcpy(projection.values, state->as<ompl::base::CompoundState>()->as<ompl::base::SE2StateManifold::StateType>(mapping_index_)->as<ompl::base::RealVectorStateManifold::StateType>(0)->values, 2 * sizeof(double));
  }
  else if(mapping_type_ == ompl_ros_interface::SO3)
  {
    projection.values[0] = state->as<ompl::base::CompoundState>()->as<ompl::base::SO3StateManifold::StateType>(mapping_index_)->x;
    projection.values[1] = state->as<ompl::base::CompoundState>()->as<ompl::base::SO3StateManifold::StateType>(mapping_index_)->y;
    projection.values[2] = state->as<ompl::base::CompoundState>()->as<ompl::base::SO3StateManifold::StateType>(mapping_index_)->z;
  }
  else if(mapping_type_ == ompl_ros_interface::SE3)
  {
    memcpy(projection.values, state->as<ompl::base::CompoundState>()->as<ompl::base::SE3StateManifold::StateType>(mapping_index_)->as<ompl::base::RealVectorStateManifold::StateType>(0)->values, 3 * sizeof(double));
  }
};

}
