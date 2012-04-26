/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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

/** \author Mrinal Kalakrishnan */
#include <arm_navigation_msgs/FilterJointTrajectory.h>
#include <arm_navigation_msgs/FilterJointTrajectoryWithConstraints.h>
#include <spline_smoother/spline_smoother.h>
#include <spline_smoother/linear_spline_velocity_scaler.h>
#include <spline_smoother/clamped_cubic_spline_smoother.h>
#include <spline_smoother/fritsch_butland_spline_smoother.h>
#include <spline_smoother/numerical_differentiation_spline_smoother.h>
#include <spline_smoother/cubic_spline_velocity_scaler.h>
#include <spline_smoother/cubic_parameterized_spline_velocity_scaler.h>

//  PLUGINLIB_REGISTER_CLASS(class_name, class_type, filters::FilterBase<T>)

PLUGINLIB_DECLARE_CLASS(spline_smoother,LinearSplineVelocityScalerFilterJointTrajectory, 
                        spline_smoother::LinearSplineVelocityScaler<arm_navigation_msgs::FilterJointTrajectory>, 
                        filters::FilterBase<arm_navigation_msgs::FilterJointTrajectory>)
PLUGINLIB_DECLARE_CLASS(spline_smoother,ClampedCubicSplineSmootherFilterJointTrajectory, 
                        spline_smoother::ClampedCubicSplineSmoother<arm_navigation_msgs::FilterJointTrajectory>, 
                        filters::FilterBase<arm_navigation_msgs::FilterJointTrajectory>)
PLUGINLIB_DECLARE_CLASS(spline_smoother,FritschButlandSplineSmootherFilterJointTrajectory, spline_smoother::FritschButlandSplineSmoother<arm_navigation_msgs::FilterJointTrajectory>, filters::FilterBase<arm_navigation_msgs::FilterJointTrajectory>)
PLUGINLIB_DECLARE_CLASS(spline_smoother,NumericalDifferentiationSplineSmootherFilterJointTrajectory, spline_smoother::NumericalDifferentiationSplineSmoother<arm_navigation_msgs::FilterJointTrajectory>, filters::FilterBase<arm_navigation_msgs::FilterJointTrajectory>)
PLUGINLIB_DECLARE_CLASS(spline_smoother,CubicSplineVelocityScalerFilterJointTrajectory, spline_smoother::CubicSplineVelocityScaler<arm_navigation_msgs::FilterJointTrajectory>, filters::FilterBase<arm_navigation_msgs::FilterJointTrajectory>)
PLUGINLIB_DECLARE_CLASS(spline_smoother,CubicParameterizedSplineVelocityScalerFilterJointTrajectory, spline_smoother::CubicParameterizedSplineVelocityScaler<arm_navigation_msgs::FilterJointTrajectory>, filters::FilterBase<arm_navigation_msgs::FilterJointTrajectory>)

PLUGINLIB_DECLARE_CLASS(spline_smoother,LinearSplineVelocityScalerFilterJointTrajectoryWithConstraints, spline_smoother::LinearSplineVelocityScaler<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>, filters::FilterBase<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>)
PLUGINLIB_DECLARE_CLASS(spline_smoother,ClampedCubicSplineSmootherFilterJointTrajectoryWithConstraints, spline_smoother::ClampedCubicSplineSmoother<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>, filters::FilterBase<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>)
PLUGINLIB_DECLARE_CLASS(spline_smoother,FritschButlandSplineSmootherFilterJointTrajectoryWithConstraints, spline_smoother::FritschButlandSplineSmoother<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>, filters::FilterBase<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>)
PLUGINLIB_DECLARE_CLASS(spline_smoother,NumericalDifferentiationSplineSmootherFilterJointTrajectoryWithConstraints, spline_smoother::NumericalDifferentiationSplineSmoother<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>, filters::FilterBase<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>)
PLUGINLIB_DECLARE_CLASS(spline_smoother,CubicSplineVelocityScalerFilterJointTrajectoryWithConstraints, spline_smoother::CubicSplineVelocityScaler<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>, filters::FilterBase<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>)
PLUGINLIB_DECLARE_CLASS(spline_smoother,CubicParameterizedSplineVelocityScalerFilterJointTrajectoryWithConstraints, spline_smoother::CubicParameterizedSplineVelocityScaler<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>, filters::FilterBase<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>)
