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

#ifndef PARABOLIC_BLEND_FAST_H_
#define PARABOLIC_BLEND_FAST_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include <spline_smoother/spline_smoother.h>
#include <spline_smoother/cubic_trajectory.h>
#include <planning_environment/models/collision_models_interface.h>
#include <planning_environment/models/model_utils.h>
#include <arm_navigation_msgs/RobotState.h>
#include <arm_navigation_msgs/ArmNavigationErrorCodes.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <constraint_aware_spline_smoother/Trajectory.h>

namespace constraint_aware_spline_smoother
{

/// \brief This is a parametric smoother that modifies the timestamps of a trajectory to respect
/// velocity and acceleration constraints.
template <typename T>
class ParabolicBlendFastSmoother : public spline_smoother::SplineSmoother<T>
{
public:
  ParabolicBlendFastSmoother(){};
  ~ParabolicBlendFastSmoother(){};

  /// \brief Configures the filter
  virtual bool configure() { return true; }

  /// \brief Calculates a smooth trajectory based on parabolic blends
  virtual bool smooth(const T& trajectory_in, T& trajectory_out) const;

private:
  void ApplyVelocityConstraints(T& trajectory, std::vector<double> &time_diff) const;
  void PrintStats(const T& trajectory) const;
  void PrintPoint(const trajectory_msgs::JointTrajectoryPoint& point, unsigned int i) const;
  void ApplyAccelerationConstraints(const T& trajectory, std::vector<double> & time_diff) const;
  double expandInterval( const double d1, const double d2, const double t, const double a_max) const;
  //double getExpandedT1( const double d1, const double d2, const double t1, const double t2, const double a_max ) const;
};

}

#endif
