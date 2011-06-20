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

#include <gtest/gtest.h>
#include <spline_smoother/splines.h>
#include <stdlib.h>

using namespace spline_smoother;

static double getRandomNumber(double min, double max)
{
  return ((double)rand() / RAND_MAX)*(max-min) + min;
}

TEST(TestSplines, testQuinticCoefficients)
{
  // seed the random number generator:
  srand(0);

  // generate random boundary conditions:
  double bc[6];
  for (int i=0; i<6; i++)
    bc[i] = getRandomNumber(-1.0, 1.0);

  // and a random time:
  double time = getRandomNumber(0.5,10.0);

  // get the spline coefficients:
  std::vector<double> coeffs;
  getQuinticSplineCoefficients(bc[0], bc[1], bc[2], bc[3], bc[4], bc[5], time, coeffs);

  // now sample the spline at t=0 and t=time to cross-check
  double test_bc[6];
  sampleQuinticSpline(coeffs, 0, test_bc[0], test_bc[1], test_bc[2]);
  sampleQuinticSpline(coeffs, time, test_bc[3], test_bc[4], test_bc[5]);

  double tolerance=1e-10;

  EXPECT_NEAR(bc[0], test_bc[0], tolerance);
  EXPECT_NEAR(bc[1], test_bc[1], tolerance);
  EXPECT_NEAR(bc[2], test_bc[2], tolerance);
  EXPECT_NEAR(bc[3], test_bc[3], tolerance);
  EXPECT_NEAR(bc[4], test_bc[4], tolerance);
  EXPECT_NEAR(bc[5], test_bc[5], tolerance);
}

TEST(TestSplines, testCubicCoefficients)
{
  // seed the random number generator:
  srand(1);

  // generate random boundary conditions:
  double bc[4];
  for (int i=0; i<4; i++)
    bc[i] = getRandomNumber(-1.0, 1.0);

  // and a random time:
  double time = getRandomNumber(0.5,10.0);

  // get the spline coefficients:
  std::vector<double> coeffs;
  getCubicSplineCoefficients(bc[0], bc[1], bc[2], bc[3], time, coeffs);

  // now sample the spline at t=0 and t=time to cross-check
  double test_bc[4];
  double dummy;
  sampleCubicSpline(coeffs, 0, test_bc[0], test_bc[1], dummy);
  sampleCubicSpline(coeffs, time, test_bc[2], test_bc[3], dummy);

  double tolerance=1e-10;

  EXPECT_NEAR(bc[0], test_bc[0], tolerance);
  EXPECT_NEAR(bc[1], test_bc[1], tolerance);
  EXPECT_NEAR(bc[2], test_bc[2], tolerance);
  EXPECT_NEAR(bc[3], test_bc[3], tolerance);
}

int main(int argc, char** argv)
{
 testing::InitGoogleTest(&argc, argv);
 return RUN_ALL_TESTS();
}
