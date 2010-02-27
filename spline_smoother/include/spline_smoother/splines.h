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


#ifndef SPLINE_SMOOTHER_SPLINES_H_
#define SPLINE_SMOOTHER_SPLINES_H_

#include <vector>

namespace spline_smoother
{

/**
 * \brief Calculates quintic spline coefficients given the start and end way-points
 *
 * The input to this function is the start and end way-point, with position, velocity and acceleration,
 * and the duration of the spline segment. (assumes that the spline runs from 0 to time)
 *
 * Returns 6 coefficients of the quintic polynomial in the "coefficients" vector. The spline can then
 * be sampled as:
 * x = coefficients[0]*t^0 + coefficients[1]*t^1 ... coefficients[5]*t^5
 */
void getQuinticSplineCoefficients(double start_pos, double start_vel, double start_acc,
    double end_pos, double end_vel, double end_acc, double time, std::vector<double>& coefficients);

/**
 * \brief Calculates cubic spline coefficients given the start and end way-points
 *
 * The input to this function is the start and end way-point, with position, velocity
 * and the duration of the spline segment. (assumes that the spline runs from 0 to time)
 *
 * Returns 4 coefficients of the quintic polynomial in the "coefficients" vector. The spline can then
 * be sampled as:
 * x = coefficients[0]*t^0 + coefficients[1]*t^1 ... coefficients[3]*t^3
 */
void getCubicSplineCoefficients(double start_pos, double start_vel,
    double end_pos, double end_vel, double time, std::vector<double>& coefficients);

/**
 * \brief Samples a quintic spline segment at a particular time
 */
void sampleQuinticSpline(const std::vector<double>& coefficients, double time,
    double& position, double& velocity, double& acceleration);

/**
 * \brief Samples a cubic spline segment at a particular time
 */
void sampleCubicSpline(const std::vector<double>& coefficients, double time,
    double& position, double& velocity, double& acceleration);


/////////////////////////// inline implementations follow //////////////////////////////


static inline void generatePowers(int n, double x, double* powers)
{
  powers[0] = 1.0;
  for (int i=1; i<=n; i++)
  {
    powers[i] = powers[i-1]*x;
  }
}

inline void getQuinticSplineCoefficients(double start_pos, double start_vel, double start_acc,
    double end_pos, double end_vel, double end_acc, double time, std::vector<double>& coefficients)
{
  coefficients.resize(6);

  double T[6];
  generatePowers(5, time, T);

  coefficients[0] = start_pos;
  coefficients[1] = start_vel;
  coefficients[2] = 0.5*start_acc;
  coefficients[3] = (-20.0*start_pos + 20.0*end_pos - 3.0*start_acc*T[2] + end_acc*T[2] -
      12.0*start_vel*T[1] - 8.0*end_vel*T[1]) / (2.0*T[3]);
  coefficients[4] = (30.0*start_pos - 30.0*end_pos + 3.0*start_acc*T[2] - 2.0*end_acc*T[2] +
      16.0*start_vel*T[1] + 14.0*end_vel*T[1]) / (2.0*T[4]);
  coefficients[5] = (-12.0*start_pos + 12.0*end_pos - start_acc*T[2] + end_acc*T[2] -
      6.0*start_vel*T[1] - 6.0*end_vel*T[1]) / (2.0*T[5]);

}

/**
 * \brief Samples a quintic spline segment at a particular time
 */
inline void sampleQuinticSpline(const std::vector<double>& coefficients, double time,
    double& position, double& velocity, double& acceleration)
{
  // create powers of time:
  double t[6];
  generatePowers(5, time, t);

  position = t[0]*coefficients[0] +
      t[1]*coefficients[1] +
      t[2]*coefficients[2] +
      t[3]*coefficients[3] +
      t[4]*coefficients[4] +
      t[5]*coefficients[5];

  velocity = t[0]*coefficients[1] +
      2.0*t[1]*coefficients[2] +
      3.0*t[2]*coefficients[3] +
      4.0*t[3]*coefficients[4] +
      5.0*t[4]*coefficients[5];

  acceleration = 2.0*t[0]*coefficients[2] +
      6.0*t[1]*coefficients[3] +
      12.0*t[2]*coefficients[4] +
      20.0*t[3]*coefficients[5];
}

inline void getCubicSplineCoefficients(double start_pos, double start_vel,
    double end_pos, double end_vel, double time, std::vector<double>& coefficients)
{
  coefficients.resize(4);

  double T[4];
  generatePowers(3, time, T);

  coefficients[0] = start_pos;
  coefficients[1] = start_vel;
  coefficients[2] = (-3.0*start_pos + 3.0*end_pos - 2.0*start_vel*T[1] - end_vel*T[1]) / T[2];
  coefficients[3] = (2.0*start_pos - 2.0*end_pos + start_vel*T[1] + end_vel*T[1]) / T[3];
}

inline void sampleCubicSpline(const std::vector<double>& coefficients, double time,
    double& position, double& velocity, double& acceleration)
{
  double t[4];
  generatePowers(3, time, t);

  position = t[0]*coefficients[0] +
      t[1]*coefficients[1] +
      t[2]*coefficients[2] +
      t[3]*coefficients[3];

  velocity = t[0]*coefficients[1] +
      2.0*t[1]*coefficients[2] +
      3.0*t[2]*coefficients[3];

  acceleration = 2.0*t[0]*coefficients[2] +
      6.0*t[1]*coefficients[3];
}

} // namespace spline_smoother

#endif /* SPLINES_H_ */
