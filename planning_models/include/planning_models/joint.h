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
 *   * Neither the name of Willow Garage nor the names of its
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

/** \author Ioan Sucan, Sachin Chitta*/

#ifndef PLANNING_JOINT_H_
#define PLANNING_JOINT_H_

#include <geometric_shapes/shapes.h>
#include <urdf/model.h>
#include <LinearMath/btTransform.h>
#include <boost/thread/mutex.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <map>

/** \brief Main namespace */
namespace planning_models
{
class PlanningModel;
class Link;

/** \brief A joint from the robot. Contains the transform applied by the joint type */
class Joint
{
public:
  Joint(boost::shared_ptr<Model> model, const unsigned int &num_dofs);
  virtual ~Joint(void);
  boost::shared_ptr<PlanningModel> parent_model_;
  /** \brief the local transform (computed by forward kinematics) */
  btTransform  local_transform_;
  unsigned int num_dofs_;/** number of degrees of freedom */
  unsigned int state_index_;
  boost::shared_ptr<Link> child_link_;
  boost::shared_ptr<Link> parent_link_;
};

/** \brief A fixed joint */
class FixedJoint : public Joint
{
public:	    
  FixedJoint(boost::shared_ptr<Model> owner): Joint(owner, 0)
  {
  }	  
  virtual void updateTransform();
};

class JointState
{
public:
  JointState();
  virtual ~JointState(void);
  boost::shared_ptr<Joint> parent_joint_;
};

class SingleDOFJointState : public JointState
{
public:
  SingleDOFJointState():JointState();
  virtual ~SingleDOFJointState(void);
  double position_;
  double velocity_;
  double acceleration_;
};

class SingleDOFJointLimits
{
public:
  SingleDOFJointLimits(const urdf::JointLimits &limits);
  double lower_;
  double upper_;
  double effort_;
  double velocity_;
  void clear()
  {
    lower_ = 0;
    upper_ = 0;
    effort_ = 0;
    velocity_ = 0;
  };
};

class SingleDOFJoint : public Joint
{
public:
  SingleDOFJoint(boost::shared_ptr<Model> owner): Joint(owner,1)
  {
  }	  
  virtual void updateTransform();
};

/** \brief A prismatic joint */
class PrismaticJoint : public SingleDOFJoint
{
public:

  PrismaticJoint(boost::shared_ptr<Model> owner) : SingleDOFJoint(owner)
  {
  }  
  virtual void updateTransform();
};
	
/** \brief A revolute joint */
class RevoluteJoint : public SingleDOFJoint
{
public:

  RevoluteJoint(boost::shared_ptr<Model> owner) : SingleDOFJoint(owner)
  {
  }	    	    
  virtual void updateTransform();
};

class PlanarJointState : public JointState
{
public:
  PlanarJointState():JointState();
  virtual ~PlanarJointState(void);
  double x_;
  double y_;
  double yaw_;
  double x_velocity_;
  double y_velocity_;
  double yaw_velocity_;
};

class PlanarJointLimits
{
public:
  PlanarJointLimits(const shapes::Shape &position_tolerance_region, const double &yaw_upper, const double &yaw_lower);
  boost::shared_ptr<shapes::Shape> position_tolerance_region_;
  double yaw_upper_;
  double yaw_lower_;
  void clear()
  {
    yaw_upper_ = 0;
    yaw_lower_ = 0;
    position_tolerance_region_->reset();
  };
};

/** \brief A planar joint */
class PlanarJoint : public Joint
{
public:	    
  PlanarJoint(boost::shared_ptr<Model> owner) : Joint(owner,3)
  {        
  }
  virtual void updateTransform();
};

class FloatingJointLimits
{
public:
  FloatingJointLimits(const shapes::Shape &position_tolerance_region, const shapes::Shape &orientation_tolerance_region);
  boost::shared_ptr<bodies::Body> position_tolerance_region_;
  boost::shared_ptr<shapes::Shape> orientation_tolerance_region_;
  void clear()
  {
    position_tolerance_region_->reset();
    orientation_tolerance_region_->reset();
  };
};

class FloatingJointState : public JointState
{
public:
  PlanarJointState():JointState();
  virtual ~PlanarJointState(void);
  btVector3 position_;
  btQuaternion orientation_;
};

/** \brief A floating joint */
class FloatingJoint : public Joint
{
public:	    
  FloatingJoint(boost::shared_ptr<Model> owner) : Joint(owner,6)
  {
  }
  virtual void updateTransform();
};

}

#endif
