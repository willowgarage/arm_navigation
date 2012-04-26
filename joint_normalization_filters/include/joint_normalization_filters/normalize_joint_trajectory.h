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

#ifndef NORMALIZE_JOINT_TRAJECTORY_H_
#define NORMALIZE_JOINT_TRAJECTORY_H_

#include <spline_smoother/spline_smoother.h>
#include <spline_smoother/spline_smoother_utils.h>
#include <angles/angles.h>

namespace joint_normalization_filters
{

/**
 * \brief This class converts a trajectory to remove all wrap-arounds, so that spline processing is easier
 */
template <typename T>
class NormalizeJointTrajectory: public spline_smoother::SplineSmoother<T>
{
public:
  NormalizeJointTrajectory();
  virtual ~NormalizeJointTrajectory();

  virtual bool smooth(const T& trajectory_in, 
                      T& trajectory_out) const;

};

template <typename T>
bool NormalizeJointTrajectory<T>::smooth(const T& data_in, 
                                         T& data_out) const
{
  data_out = data_in;

  int size = data_in.request.trajectory.points.size();
  int num_joints = data_in.request.trajectory.joint_names.size();

  if (!spline_smoother::checkTrajectoryConsistency(data_out))
    return false;

  for (int i=0; i<num_joints; ++i)
  {
    if (!data_out.request.limits[i].has_position_limits)
    {
      for (int j=1; j<size; ++j)
      {
        double& cur = data_out.request.trajectory.points[j].positions[i];
        double prev = data_out.request.trajectory.points[j-1].positions[i];

        cur = prev + angles::shortest_angular_distance(prev, cur);
      }
    }
  }
  return true;
}

template <typename T>
NormalizeJointTrajectory<T>::NormalizeJointTrajectory()
{
}

template <typename T>
NormalizeJointTrajectory<T>::~NormalizeJointTrajectory()
{
}

}

#endif /* NORMALIZE_JOINT_TRAJECTORY_H */
