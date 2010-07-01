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

/** \author Sachin Chitta, Ioan Sucan */

#ifndef IK_CONSTRAINT_EVALUATORS_
#define IK_CONSTRAINT_EVALUATORS_

#include <motion_planning_msgs/Constraints.h>
#include <geometric_shapes_msgs/Shape.h>
#include <geometric_shapes/bodies.h>
#include <geometry_msgs/Pose.h>

#include <boost/scoped_ptr.hpp>

#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <angles/angles.h>

namespace constraint_evaluators
{    

  class PositionConstraintEvaluator
  {
  public:
    PositionConstraintEvaluator(){}

    PositionConstraintEvaluator(const motion_planning_msgs::PositionConstraint &pc)
    {
      setup(pc);
    }
	
    virtual ~PositionConstraintEvaluator(void)
    {
    }
    
    void setup(const motion_planning_msgs::PositionConstraint &pc);
   
    bool isSatisfied(const geometry_msgs::Point &position) const
    {
      btVector3 position_tf;
      tf::pointMsgToTF(position,position_tf);
      return isSatisfied(position_tf);
    }

    bool isSatisfied(const btVector3 &position) const
    {
      bool result = constraint_region_->containsPoint(position,false);
      return result;
    }

    double distance(const btVector3 &position) const
    {
      return (position-nominal_position_).length();
    }

  private:
    boost::scoped_ptr<bodies::Body> constraint_region_;
      btVector3 nominal_position_;
  };

  class OrientationConstraintEvaluator
  {
    public:
    OrientationConstraintEvaluator():active_(false)
    {

    }

    OrientationConstraintEvaluator(const motion_planning_msgs::OrientationConstraint &oc) 
    {
      setup(oc);
    }
	
    virtual ~OrientationConstraintEvaluator(void)
    {
    }

    void setup(const motion_planning_msgs::OrientationConstraint &oc)
    {
      btQuaternion q;
      tf::quaternionMsgToTF(oc.orientation,q);
      nominal_orientation_ = btMatrix3x3(q);
      nominal_orientation_inverse_ = nominal_orientation_.inverse();
      if(oc.type == oc.HEADER_FRAME) 
        body_fixed_orientation_constraint_ = false;
      else
        body_fixed_orientation_constraint_ = true;
      absolute_roll_tolerance_ = oc.absolute_roll_tolerance;
      absolute_pitch_tolerance_ = oc.absolute_pitch_tolerance;
      absolute_yaw_tolerance_ = oc.absolute_yaw_tolerance;      
      active_ = true;
    }
	
    bool isSatisfied(const geometry_msgs::Quaternion &quaternion)
    {
      btQuaternion q;
      tf::quaternionMsgToTF(quaternion,q);
      return isSatisfied(btMatrix3x3(q));
    }

    bool isSatisfied(const btMatrix3x3 &orientation_matrix) const
    {
      double roll, pitch, yaw;
      getRPYDistance(orientation_matrix,roll,pitch,yaw);
      if(fabs(roll) < absolute_roll_tolerance_ &&
         fabs(pitch) < absolute_pitch_tolerance_ &&
         fabs(yaw) < absolute_yaw_tolerance_)
        return true;
      else
        return false;
    }

    double distance(const btMatrix3x3 &orientation_matrix) const
    {
      double roll, pitch, yaw;
      getRPYDistance(orientation_matrix,roll,pitch,yaw);
      return std::max<double>(std::max<double>(fabs(roll),fabs(pitch)),fabs(yaw));
    }

  private:
    double absolute_roll_tolerance_, absolute_pitch_tolerance_, absolute_yaw_tolerance_;
    btMatrix3x3 nominal_orientation_, nominal_orientation_inverse_;
    bool body_fixed_orientation_constraint_;    
    bool active_;

    void getRPYDistance(const btMatrix3x3 &orientation_matrix, double &roll, double &pitch, double &yaw) const
    {
      if(!body_fixed_orientation_constraint_)
      {
        btMatrix3x3 result = orientation_matrix * nominal_orientation_inverse_;
        result.getRPY(roll, pitch, yaw);
      }
      else
      {
        btMatrix3x3 result = nominal_orientation_inverse_ * orientation_matrix;
        result.getRPY(roll, pitch, yaw);
      }
    }
  };

  class JointConstraintEvaluator
  {
    public:
    JointConstraintEvaluator():active_(false)
    {
    }

    JointConstraintEvaluator(const motion_planning_msgs::JointConstraint &jc)
    {
      setup(jc);
    }
    virtual ~JointConstraintEvaluator(void)
    {      
    }

    void setup(const motion_planning_msgs::JointConstraint &jc, bool continuous=false)
    {
      tolerance_above_  = jc.tolerance_above;
      tolerance_below_  = jc.tolerance_below;
      nominal_position_ = jc.position;
      continuous_ = continuous;
      active_ = true;
    }

    bool isSatisfied(const double &position) const
    {
      if(!active_)
        return true;

      double dif;
      if(continuous_)
        dif = angles::shortest_angular_distance(nominal_position_,position);
      else
        dif = position - nominal_position_;
  
      if (dif > tolerance_above_ || dif < - tolerance_below_)
        return false;
      else
        return true;
    }

    double distance(const double &position) const
    {
      if(!active_)
        return 0.0;
      double dif;
      if(continuous_)
        dif = angles::shortest_angular_distance(nominal_position_,position);
      else
        dif = position - nominal_position_;
      return dif;
    }

  private:
    double tolerance_above_, tolerance_below_;
    double nominal_position_;
    bool continuous_;
    bool active_;
  };
}
#endif
