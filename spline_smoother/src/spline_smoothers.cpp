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

#include <spline_smoother/spline_smoother.h>
#include <spline_smoother/linear_spline_velocity_scaler.h>
#include <spline_smoother/unnormalize_trajectory.h>
#include <spline_smoother/clamped_cubic_spline_smoother.h>
#include <spline_smoother/fritsch_butland_spline_smoother.h>
#include <spline_smoother/numerical_differentiation_spline_smoother.h>
#include <spline_smoother/cubic_spline_velocity_scaler.h>

REGISTER_SPLINE_SMOOTHER(LinearSplineVelocityScaler, spline_smoother::LinearSplineVelocityScaler)
REGISTER_SPLINE_SMOOTHER(ClampedCubicSplineSmoother, spline_smoother::ClampedCubicSplineSmoother)
REGISTER_SPLINE_SMOOTHER(FritschButlandSplineSmoother, spline_smoother::FritschButlandSplineSmoother)
REGISTER_SPLINE_SMOOTHER(NumericalDifferentiationSplineSmoother, spline_smoother::NumericalDifferentiationSplineSmoother)
REGISTER_SPLINE_SMOOTHER(UnNormalizeTrajectory, spline_smoother::UnNormalizeTrajectory)
REGISTER_SPLINE_SMOOTHER(CubicSplineVelocityScaler, spline_smoother::CubicSplineVelocityScaler)
