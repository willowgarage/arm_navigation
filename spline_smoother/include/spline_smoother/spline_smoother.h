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

#ifndef SPLINE_SMOOTHER_H_
#define SPLINE_SMOOTHER_H_

#include <vector>
#include <filters/filter_base.h>
#include <pluginlib/class_list_macros.h>

namespace spline_smoother
{

/**
 * \brief Abstract base class for spline smoothing.
 *
 * A spline smoother is a class that takes in a "WaypointTrajWithLimits" message, potentially
 * containing no velocities / accelerations, and fills them in using some rules. Example
 * implementations are in "ClampedCubicSplineSmoother", "FritschButlandSplineSmoother" and
 * "NumericalDifferentiationSplineSmoother", each of which uses a different set of rules
 * to fill in the velocities and accelerations for spline creation.
 *
 * To implement a smoother, just override the virtual "smooth" method, and call the
 * REGISTER_SPLINE_SMOOTHER macro with the class name (anywhere in the cpp file)
 */
template <typename T>
class SplineSmoother: public filters::FilterBase<T>
{
public:
  SplineSmoother(){};
  virtual ~SplineSmoother(){};

  virtual bool configure();

  /**
   * \brief Implementation of the update() function for the filter
   */
  virtual bool update(const T& data_in, T& data_out);

  /**
   * \brief Smooths the input position trajectory by generating velocities and accelerations at the waypoints.
   *
   * This virtual method needs to implemented by the derived class.
   * \return true if successful, false if not
   */
  virtual bool smooth(const T& trajectory_in, T& trajectory_out) const = 0;
};

/////////////////////////// inline functions follow //////////////////////////

template <typename T>
inline bool SplineSmoother<T>::configure()
{
  return true;
}

template <typename T>
inline bool SplineSmoother<T>::update(const T& data_in, 
                                   T& data_out)
{
  return smooth(data_in, data_out);
}

}

//#define REGISTER_SPLINE_SMOOTHER(class_name, class_type)              
//  PLUGINLIB_REGISTER_CLASS(class_name, class_type, filters::FilterBase<T>)

#endif /* SPLINE_SMOOTHER_H_ */
