/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

/** \author Ken Anderson */

#ifndef PARABOLIC_LINEAR_BLEND_H_
#define PARABOLIC_LINEAR_BLEND_H_

#include <spline_smoother/spline_smoother.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace constraint_aware_spline_smoother
{

/// @brief This is a wrapper around Tobias Kunz and Mike Stilman's trajectory smoother using
/// parabolic and linear phases.
/// See http://www.golems.org/node/1570 for more details.
/// This smoother chooses timing intervals between trajectory points that respects
/// both velocity and acceleration constraints.
/// The resulting trajectory uses linear segments parametric blends to smooth the trajectory between points.
template <typename T>
class ParabolicLinearBlendSmoother : public spline_smoother::SplineSmoother<T>
{
public:
  ParabolicLinearBlendSmoother(){};
  ~ParabolicLinearBlendSmoother(){};

  /// \brief Configures the filter
  virtual bool configure() { return true; }

  /// \brief Calculates a smooth trajectory based on parabolic blends
  virtual bool smooth(const T& trajectory_in, T& trajectory_out) const;
};

}

#endif
