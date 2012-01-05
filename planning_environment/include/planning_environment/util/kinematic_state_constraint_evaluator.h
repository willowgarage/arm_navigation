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

/** \author Ioan Sucan */

#ifndef PLANNING_ENVIRONMENT_UTIL_KINEMATIC_STATE_CONSTRAINT_EVALUATOR_
#define PLANNING_ENVIRONMENT_UTIL_KINEMATIC_STATE_CONSTRAINT_EVALUATOR_

#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
#include <arm_navigation_msgs/Constraints.h>
#include <arm_navigation_msgs/Shape.h>
#include <geometric_shapes/bodies.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <vector>

#include <boost/scoped_ptr.hpp>

namespace planning_environment
{
bool createConstraintRegionFromMsg(const arm_navigation_msgs::Shape &constraint_region_shape, 
                                   const geometry_msgs::Pose &constraint_region_pose, 
                                   boost::scoped_ptr<bodies::Body> &body);

class KinematicConstraintEvaluator
{
public:
	
  KinematicConstraintEvaluator(void)
  {
  }
	
  virtual ~KinematicConstraintEvaluator(void)
  {
  }
	
  /** \brief Clear the stored constraint */
  virtual void clear(void) = 0;

  /** \brief Decide whether the constraint is satisfied in the indicated state or group, if specified */
  virtual bool decide(const planning_models::KinematicState *state,
                      bool verbose=false) const = 0;

  /** \brief Print the constraint data */
  virtual void print(std::ostream &out = std::cout) const
  {
  }
};
    
class JointConstraintEvaluator : public KinematicConstraintEvaluator
{
public:  

  JointConstraintEvaluator(void) : KinematicConstraintEvaluator()
  {
    m_joint = NULL;
  }
	
  /** \brief This function assumes the constraint has been transformed into the proper frame, if such a transform is needed */
  bool use(const arm_navigation_msgs::JointConstraint &jc);

  /** \brief Decide whether the constraint is satisfied in the indicated state or group, if specified */
  virtual bool decide(const planning_models::KinematicState  *state,
                      bool verbose=false) const;

  /** \brief Clear the stored constraint */
  virtual void clear(void);

  /** \brief Print the constraint data */
  virtual void print(std::ostream &out = std::cout) const;

  /** \brief Get the constraint message */
  const arm_navigation_msgs::JointConstraint& getConstraintMessage(void) const;

protected:
	
  arm_navigation_msgs::JointConstraint         m_jc;
  const planning_models::KinematicModel::JointModel *m_joint;    
};
    
	
class OrientationConstraintEvaluator : public KinematicConstraintEvaluator
{
public:
	
  OrientationConstraintEvaluator(void) : KinematicConstraintEvaluator()
  {
  }
	
  /** \brief This function assumes the constraint has been transformed into the proper frame, if such a transform is needed */
  bool use(const arm_navigation_msgs::OrientationConstraint &pc);

  /** \brief Clear the stored constraint */
  virtual void clear(void);
	
  /** \brief Decide whether the constraint is satisfied in the indicated state or group, if specified */
  virtual bool decide(const planning_models::KinematicState* state, 
                      bool verbose=false) const;

  /** \brief Evaluate the distances to the position and to the orientation are given. */
  void evaluate(const planning_models::KinematicState* state, double &distAng, bool verbose=false) const;
  
  /** \brief Decide whether the constraint is satisfied. The distances to the position and to the orientation are given. */
  bool decide(double dAng, bool verbose=false) const;
  
  /** \brief Print the constraint data */
  void print(std::ostream &out = std::cout) const;

  /** \brief Get the constraint message */
  const arm_navigation_msgs::OrientationConstraint& getConstraintMessage(void) const;
	
protected:
	
  arm_navigation_msgs::OrientationConstraint  m_oc;
  double m_roll, m_pitch, m_yaw;
  tf::Matrix3x3 m_rotation_matrix;
  boost::scoped_ptr<bodies::Body> m_constraint_region;
	
};

class VisibilityConstraintEvaluator : public KinematicConstraintEvaluator
{
public:
	
  VisibilityConstraintEvaluator(void) : KinematicConstraintEvaluator()
  {
  }
	
  /** \brief This function assumes the constraint has been transformed into the proper frame, if such a transform is needed */
  bool use(const arm_navigation_msgs::VisibilityConstraint &vc);

  /** \brief Clear the stored constraint */
  virtual void clear(void);
	
  /** \brief Decide whether the constraint is satisfied in the indicated state or group, if specified */
  virtual bool decide(const planning_models::KinematicState* state,
                      bool verbose=false) const;

  /** \brief Print the constraint data */
  void print(std::ostream &out = std::cout) const;

  /** \brief Get the constraint message */
  const arm_navigation_msgs::VisibilityConstraint& getConstraintMessage(void) const;
	
protected:	
  arm_navigation_msgs::VisibilityConstraint  m_vc;
  tf::Transform m_sensor_offset_pose;
};

class PositionConstraintEvaluator : public KinematicConstraintEvaluator
{
public:
	
  PositionConstraintEvaluator(void) : KinematicConstraintEvaluator()
  {
  }
	
  /** \brief This function assumes the constraint has been transformed into the proper frame, if such a transform is needed */
  bool use(const arm_navigation_msgs::PositionConstraint &pc);

  /** \brief Clear the stored constraint */
  virtual void clear(void);
	
  /** \brief Decide whether the constraint is satisfied in the indicated state or group, if specified */
  virtual bool decide(const planning_models::KinematicState* state,
                      bool verbose=false) const;

  /** \brief Evaluate the distances to the position and to the orientation are given. */
  void evaluate(const planning_models::KinematicState* state, double& distPos, bool verbose=false) const;
	
  /** \brief Decide whether the constraint is satisfied. The distances to the position and to the orientation are given. */
  bool decide(double dPos, bool verbose=false) const;

  /** \brief Print the constraint data */
  void print(std::ostream &out = std::cout) const;

  /** \brief Get the constraint message */
  const arm_navigation_msgs::PositionConstraint& getConstraintMessage(void) const;
	
protected:
	
  arm_navigation_msgs::PositionConstraint     m_pc;
  double                                       m_x, m_y, m_z;
  tf::Vector3                                    m_offset;
  boost::scoped_ptr<bodies::Body> m_constraint_region;
};
        
class KinematicConstraintEvaluatorSet
{
public:
	
  KinematicConstraintEvaluatorSet(void)
  {
  }
	
  ~KinematicConstraintEvaluatorSet(void)
  {
    clear();
  }
	
  /** \brief Clear the stored constraints */
  void clear(void);
	
  /** \brief Add a set of joint constraints */
  bool add(const std::vector<arm_navigation_msgs::JointConstraint> &jc);

  /** \brief Add a set of position constraints */
  bool add(const std::vector<arm_navigation_msgs::PositionConstraint> &pc);

  /** \brief Add a set of orientation constraints */
  bool add(const std::vector<arm_navigation_msgs::OrientationConstraint> &pc);

  /** \brief Add a set of orientation constraints */
  bool add(const std::vector<arm_navigation_msgs::VisibilityConstraint> &pc);
	
  /** \brief Decide whether the set of constraints is satisfied  */
  bool decide(const planning_models::KinematicState* state,
              bool verbose=false) const;

  /** \brief Print the constraint data */
  void print(std::ostream &out = std::cout) const;
	
  /** \brief Get the active position constraints */
  const std::vector<arm_navigation_msgs::PositionConstraint>& getPositionConstraints(void) const
  {
    return m_pc;
  }

  /** \brief Get the active orientation constraints */
  const std::vector<arm_navigation_msgs::OrientationConstraint>& getOrientationConstraints(void) const
  {
    return m_oc;
  }

  /** \brief Get the active pose constraints */
  const std::vector<arm_navigation_msgs::JointConstraint>& getJointConstraints(void) const
  {
    return m_jc;
  }
	
protected:
	
  std::vector<KinematicConstraintEvaluator*>         m_kce;
  std::vector<arm_navigation_msgs::JointConstraint> m_jc;

  std::vector<arm_navigation_msgs::PositionConstraint>  m_pc;
  std::vector<arm_navigation_msgs::OrientationConstraint>  m_oc;
  std::vector<arm_navigation_msgs::VisibilityConstraint> m_vc;
};
} // planning_environment


#endif
