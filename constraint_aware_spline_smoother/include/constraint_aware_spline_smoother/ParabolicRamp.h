/*****************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2009, the Trustees of Indiana University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Indiana University nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE TRUSTEES OF INDIANA UNIVERSITY ''AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE TRUSTEES OF INDIANA UNIVERSITY BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 ***************************************************************************/

#ifndef PARABOLIC_RAMP_H
#define PARABOLIC_RAMP_H

#include <math.h>
#include <assert.h>
#include "constraint_aware_spline_smoother/Math.h"

/** @file ParabolicRamp.h
 * @brief Functions for optimal acceleration-bounded trajectories.
 */

/** @brief Stores optimal trajectores for an acceleration and
 * velocity-bounded 1D system.
 *
 * Initialize the members x0 (start position), x1 (end position),
 * dx0 (start velocity), and dx1 (end velocity) before calling the
 * SolveX functions.
 */
class ParabolicRamp1D
{
 public:
  /// Sets the ramp to a constant function
  void SetConstant(Real x);
  /// Solves for minimum time given acceleration and velocity bounds
  bool SolveMinTime(Real amax,Real vmax);
  /// Solves for minimum acceleration given end time and velocity bounds
  bool SolveMinAccel(Real endTime,Real vmax);
  /// Evaluates the trajectory
  Real Evaluate(Real t) const;
  /// Evaluates the derivative of the trajectory
  Real Derivative(Real t) const;
  /// Evaluates the second derivative of the trajectory
  Real Accel(Real t) const;
  /// Returns the time at which x1 is reached
  Real EndTime() const { return ttotal; }
  /// Scales time to slow down (value > 1) or speed up (< 1) the trajectory
  void Dilate(Real timeScale);
  /// Trims off the front [0,tcut] of the trajectory
  void TrimFront(Real tcut);
  /// Trims off the front [T-tcut,T] of the trajectory
  void TrimBack(Real tcut);
  /// Sanity check
  bool IsValid() const;

  /// Input
  Real x0,dx0;
  Real x1,dx1;

  /// Calculated upon SolveX
  Real tswitch1,tswitch2;  //time to switch between ramp/flat/ramp
  Real ttotal;
  Real a1,v,a2;   // accel of first ramp, velocity of linear section, accel of second ramp
};

/** @brief Solves for optimal trajectores for a velocity-bounded ND system.
 *
 * Methods are essentially the same as for ParabolicRamp1D.
 */
class ParabolicRampND
{
 public:
  void SetConstant(const Vector& x);
  bool SolveMinTimeLinear(const Vector& amax,const Vector& vmax);
  bool SolveMinTime(const Vector& amax,const Vector& vmax);
  bool SolveMinAccel(const Vector& vmax,Real time);
  void Evaluate(Real t,Vector& x) const;
  void Derivative(Real t,Vector& x) const;
  void Output(Real dt,std::vector<Vector>& path) const;
  void Dilate(Real timeScale);
  void TrimFront(Real tcut);
  void TrimBack(Real tcut);
  bool IsValid() const;

  /// Input
  Vector x0,dx0;
  Vector x1,dx1;

  /// Calculated upon SolveX
  Real endTime;
  std::vector<ParabolicRamp1D> ramps;
};

#endif
