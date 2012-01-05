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

#include "planning_environment/util/kinematic_state_constraint_evaluator.h"
#include <geometric_shapes/shape_operations.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <cassert>

bool planning_environment::JointConstraintEvaluator::use(const arm_navigation_msgs::JointConstraint &jc)
{
  m_jc     = jc;
  return true;
}

bool planning_environment::JointConstraintEvaluator::decide(const planning_models::KinematicState* state, 
                                                            bool verbose) const
{
  std::vector<double> cur_joint_values;
  const planning_models::KinematicState::JointState* joint = state->getJointState(m_jc.joint_name);
  if(!joint) {
    ROS_WARN_STREAM("No joint in state with name " << m_jc.joint_name);
  }
  cur_joint_values = joint->getJointStateValues();

  if(cur_joint_values.size() == 0) {
    ROS_WARN_STREAM("Trying to decide joint with no value " << joint->getName());
  }
  if(cur_joint_values.size() > 1) {
    ROS_WARN_STREAM("Trying to decide joint value with more than one value " << joint->getName());
  }

  double current_joint_position = cur_joint_values[0];
  double dif;
  
  const planning_models::KinematicModel::RevoluteJointModel *revolute_joint = dynamic_cast<const planning_models::KinematicModel::RevoluteJointModel*>(joint->getJointModel());

  if(revolute_joint && revolute_joint->continuous_)
    dif = angles::shortest_angular_distance(m_jc.position,current_joint_position);
  else
    dif = current_joint_position - m_jc.position;
  
  if(verbose)
    ROS_DEBUG("Joint name:%s, value: %f, Constraint: %f, tolerance_above: %f, tolerance_below: %f",joint->getName().c_str(),current_joint_position,m_jc.position,m_jc.tolerance_above,m_jc.tolerance_below);
  if (dif > m_jc.tolerance_above || dif < - m_jc.tolerance_below)
  {
    if(verbose) {
      ROS_INFO("Constraint violated:: Joint name:%s, value: %f, Constraint: %f, tolerance_above: %f, tolerance_below: %f",joint->getName().c_str(),current_joint_position,m_jc.position,m_jc.tolerance_above,m_jc.tolerance_below);
    }
    return false;
  }
  return true;
}

void planning_environment::JointConstraintEvaluator::clear(void)
{
}

const arm_navigation_msgs::JointConstraint& planning_environment::JointConstraintEvaluator::getConstraintMessage(void) const
{
  return m_jc;
}

void planning_environment::JointConstraintEvaluator::print(std::ostream &out) const
{		
  if (m_joint)
  {
    out << "Joint constraint for joint " << m_jc.joint_name << ": " << std::endl;
    out << "  value = ";	    
    out << m_jc.position << "; ";
    out << "  tolerance below = ";	    
    out << m_jc.tolerance_below << "; ";	
    out << "  tolerance above = ";
    out << m_jc.tolerance_above << "; ";
    out << std::endl;
  }
  else
    out << "No constraint" << std::endl;    
}

bool planning_environment::createConstraintRegionFromMsg(const arm_navigation_msgs::Shape &constraint_region_shape, 
                                                         const geometry_msgs::Pose &constraint_region_pose, 
                                                         boost::scoped_ptr<bodies::Body> &body)
{
  if(constraint_region_shape.type == arm_navigation_msgs::Shape::SPHERE)
  {
    if(constraint_region_shape.dimensions.empty())
      return false;
    shapes::Sphere shape(constraint_region_shape.dimensions[0]);
    body.reset(new bodies::Sphere(&shape));
  }
  else if(constraint_region_shape.type == arm_navigation_msgs::Shape::BOX)
  {
    if((int) constraint_region_shape.dimensions.size() < 3)
      return false;
    shapes::Box shape(constraint_region_shape.dimensions[0],constraint_region_shape.dimensions[1],constraint_region_shape.dimensions[2]);
    body.reset(new bodies::Box(&shape));
  }
  else if(constraint_region_shape.type == arm_navigation_msgs::Shape::CYLINDER)
  {
    if((int) constraint_region_shape.dimensions.size() < 2)
      return false;
    shapes::Cylinder shape(constraint_region_shape.dimensions[0],constraint_region_shape.dimensions[1]);
    body.reset(new bodies::Cylinder(&shape));
  }
  else if(constraint_region_shape.type == arm_navigation_msgs::Shape::MESH)
  {
    std::vector<tf::Vector3> vertices;
    std::vector<unsigned int> triangles; 
    for(unsigned int i=0; i < constraint_region_shape.triangles.size(); i++)
    {
      triangles.push_back((unsigned int) constraint_region_shape.triangles[i]);
    }
    for(unsigned int i=0; i < constraint_region_shape.triangles.size(); i++)
    {
      tf::Vector3 tmp;
      tf::pointMsgToTF(constraint_region_shape.vertices[i],tmp);
      vertices.push_back(tmp);
    }
    shapes::Mesh *shape = shapes::createMeshFromVertices(vertices,triangles);
    body.reset(new bodies::ConvexMesh(shape));    
  }
  else
  {
    ROS_ERROR("Could not recognize constraint type");
    return false;
  }
  tf::Transform pose_tf;
  tf::poseMsgToTF(constraint_region_pose,pose_tf);
  body->setPose(pose_tf);
  return true;
}

bool planning_environment::PositionConstraintEvaluator::use(const arm_navigation_msgs::PositionConstraint &pc)
{
  m_pc   = pc;

  m_x = m_pc.position.x;
  m_y = m_pc.position.y;
  m_z = m_pc.position.z;
  m_offset= tf::Vector3(m_pc.target_point_offset.x,m_pc.target_point_offset.y,m_pc.target_point_offset.z);

  geometry_msgs::Pose constraint_region_pose;
  constraint_region_pose.orientation = pc.constraint_region_orientation;
  constraint_region_pose.position = m_pc.position;
  createConstraintRegionFromMsg(pc.constraint_region_shape,constraint_region_pose,m_constraint_region);
  ROS_DEBUG("Position Constraint: frame_id: %s",pc.header.frame_id.c_str());
  ROS_DEBUG("Position Constraint Desired position: (%f,%f,%f)",m_pc.position.x,m_pc.position.y,m_pc.position.z);
  ROS_DEBUG("Position Constraint Region: orientation: (%f,%f,%f,%f)",
           m_pc.constraint_region_orientation.x,
           m_pc.constraint_region_orientation.y,
           m_pc.constraint_region_orientation.z,
           m_pc.constraint_region_orientation.w);    
  ROS_DEBUG("Offset (%f,%f,%f)", m_pc.target_point_offset.x,m_pc.target_point_offset.y,m_pc.target_point_offset.z);
  return true;
}
	
bool planning_environment::OrientationConstraintEvaluator::use(const arm_navigation_msgs::OrientationConstraint &oc)
{
  m_oc   = oc;
  tf::Quaternion q;
  tf::quaternionMsgToTF(m_oc.orientation,q);
  m_rotation_matrix = tf::Matrix3x3(q);
  geometry_msgs::Pose id;
  id.orientation.w = 1.0;
  ROS_DEBUG("Orientation constraint: %f %f %f %f",m_oc.orientation.x,m_oc.orientation.y,m_oc.orientation.z,m_oc.orientation.w);
  return true;
}

void planning_environment::PositionConstraintEvaluator::clear(void)
{
}

void planning_environment::OrientationConstraintEvaluator::clear(void)
{
}

bool planning_environment::PositionConstraintEvaluator::decide(const planning_models::KinematicState  *state,
                                                               bool verbose) const
{
  const planning_models::KinematicState::LinkState* link_state = state->getLinkState(m_pc.link_name);

  if(!link_state) 
  {
    ROS_WARN_STREAM("No link in state with name " << m_pc.link_name);
    return false;
  }

  bool result =  m_constraint_region->containsPoint(link_state->getGlobalLinkTransform()(m_offset),false);
  if(!result)
  {
    ROS_DEBUG("Position constraint violated : desired:: %f, %f, %f, current:: %f, %f, %f, tolerance: %f, %f, %f",m_x,m_y,m_z,
              link_state->getGlobalLinkTransform().getOrigin().x(),link_state->getGlobalLinkTransform().getOrigin().y(),link_state->getGlobalLinkTransform().getOrigin().z(),
             m_pc.constraint_region_shape.dimensions[0],m_pc.constraint_region_shape.dimensions[1],m_pc.constraint_region_shape.dimensions[2]);
    if(verbose) {
      ROS_INFO("Link name %s Position constraint satisfied: desired:: %f, %f, %f, current:: %f, %f, %f, tolerance: %f, %f, %f",link_state->getName().c_str(), m_x,m_y,m_z,
               link_state->getGlobalLinkTransform().getOrigin().x(),link_state->getGlobalLinkTransform().getOrigin().y(),link_state->getGlobalLinkTransform().getOrigin().z(),
               m_pc.constraint_region_shape.dimensions[0],m_pc.constraint_region_shape.dimensions[1],m_pc.constraint_region_shape.dimensions[2]);
    }
  }
  return result;
}

bool planning_environment::OrientationConstraintEvaluator::decide(const planning_models::KinematicState *state,
                                                                  bool verbose) const
{
  const planning_models::KinematicState::LinkState* link_state = state->getLinkState(m_oc.link_name);

  if(!link_state) 
  {
    ROS_WARN_STREAM("No link in state with name " << m_oc.link_name);
    return false;
  }
  /*  tfScalar yaw, pitch, roll;
      m_link->global_collision_body_transform.getBasis().getEulerYPR(yaw, pitch, roll);
      tf::Vector3 orientation(roll,pitch,yaw);
  */
  tfScalar yaw, pitch, roll;
  if(m_oc.type == m_oc.HEADER_FRAME)
  {
    tf::Matrix3x3 result = link_state->getGlobalLinkTransform().getBasis() *  m_rotation_matrix.inverse();
    result.getRPY(roll, pitch, yaw);
    //    result.getEulerYPR(yaw, pitch, roll);
  }
  else
  {
    tf::Matrix3x3 result = m_rotation_matrix.inverse() * link_state->getGlobalLinkTransform().getBasis();
    result.getRPY(roll, pitch, yaw);
  }
  if(fabs(roll) < m_oc.absolute_roll_tolerance &&
     fabs(pitch) < m_oc.absolute_pitch_tolerance &&
     fabs(yaw) < m_oc.absolute_yaw_tolerance)
  {
    return true;
  }
  else
  {
    tf::Quaternion quat;
    link_state->getGlobalLinkTransform().getBasis().getRotation(quat);
    geometry_msgs::Quaternion quat_msg;
    tf::quaternionTFToMsg(quat,quat_msg);
    if(!verbose) {
      ROS_DEBUG("Orientation constraint: violated");
      ROS_DEBUG("Orientation Constraint: Quaternion desired: %f %f %f %f",m_oc.orientation.x,m_oc.orientation.y,m_oc.orientation.z,m_oc.orientation.w);
      ROS_DEBUG("Orientation Constraint: Quaternion actual: %f %f %f %f",quat_msg.x,quat_msg.y,quat_msg.z,quat_msg.w);
      ROS_DEBUG("Orientation Constraint: Error: Roll: %f, Pitch: %f, Yaw: %f",roll,pitch,yaw);
      ROS_DEBUG("Orientation Constraint: Tolerance: Roll: %f, Pitch: %f, Yaw: %f",m_oc.absolute_roll_tolerance,m_oc.absolute_pitch_tolerance,m_oc.absolute_yaw_tolerance);
    } else {
      ROS_INFO("Orientation constraint: violated");
      ROS_INFO("Orientation Constraint: Quaternion desired: %f %f %f %f",m_oc.orientation.x,m_oc.orientation.y,m_oc.orientation.z,m_oc.orientation.w);
      ROS_INFO("Orientation Constraint: Quaternion actual: %f %f %f %f",quat_msg.x,quat_msg.y,quat_msg.z,quat_msg.w);
      ROS_INFO("Orientation Constraint: Error: Roll: %f, Pitch: %f, Yaw: %f",roll,pitch,yaw);
      ROS_INFO("Orientation Constraint: Tolerance: Roll: %f, Pitch: %f, Yaw: %f",m_oc.absolute_roll_tolerance,m_oc.absolute_pitch_tolerance,m_oc.absolute_yaw_tolerance);
    }
    return false;
  }
}

void planning_environment::PositionConstraintEvaluator::evaluate(const planning_models::KinematicState* state, double& distPos, bool verbose) const
{
  const planning_models::KinematicState::LinkState* link_state = state->getLinkState(m_pc.link_name);

  if(!link_state) 
  {
    ROS_WARN_STREAM("No link in state with name " << m_pc.link_name);
    distPos = DBL_MAX;
  }

  double dx = link_state->getGlobalLinkTransform().getOrigin().x() - m_pc.position.x;
  double dy = link_state->getGlobalLinkTransform().getOrigin().y() - m_pc.position.y;
  double dz = link_state->getGlobalLinkTransform().getOrigin().z() - m_pc.position.z;
  
  distPos = sqrt(dx*dx+dy*dy+dz*dz);
}

void planning_environment::OrientationConstraintEvaluator::evaluate(const planning_models::KinematicState* state, double& distAng, bool verbose) const
{
  const planning_models::KinematicState::LinkState* link_state = state->getLinkState(m_oc.link_name);

  if(!link_state) 
  {
    ROS_WARN_STREAM("No link in state with name " << m_oc.link_name);
    distAng = DBL_MAX;
  }

  distAng = 0.0;
  tfScalar yaw, pitch, roll;
  if(m_oc.type == m_oc.HEADER_FRAME)
  {
    tf::Matrix3x3 result = m_rotation_matrix.inverse() * link_state->getGlobalLinkTransform().getBasis();
    result.getEulerYPR(yaw, pitch, roll);
  }
  else
  {
    tf::Matrix3x3 result = link_state->getGlobalLinkTransform().getBasis() *  m_rotation_matrix.inverse();
    result.getRPY(roll, pitch, yaw);
  }
  distAng += fabs(yaw); 
  distAng += fabs(pitch);
  distAng += fabs(roll);
  if(verbose) {
    std::cout << "Dist angle is " << distAng << std::endl;
  }
}

bool planning_environment::PositionConstraintEvaluator::decide(double dPos, bool verbose) const
{
  return dPos < 1e-12;
}

bool planning_environment::OrientationConstraintEvaluator::decide(double dAng, bool verbose) const
{
  return dAng < 1e-12;
}

const arm_navigation_msgs::PositionConstraint& planning_environment::PositionConstraintEvaluator::getConstraintMessage(void) const
{
  return m_pc;
}

const arm_navigation_msgs::OrientationConstraint& planning_environment::OrientationConstraintEvaluator::getConstraintMessage(void) const
{
  return m_oc;
}

void planning_environment::PositionConstraintEvaluator::print(std::ostream &out) const
{
  out << "Position constraint on link '" << m_pc.link_name << "'" << std::endl;
  if (m_pc.constraint_region_shape.type == arm_navigation_msgs::Shape::SPHERE)
  {
    if(m_pc.constraint_region_shape.dimensions.empty())
    {
      out << "No radius specified for spherical constraint region.";
    }
    else
    {
      out << "Spherical constraint region with radius " << m_pc.constraint_region_shape.dimensions[0] << std::endl;
    }   
  }
  else if (m_pc.constraint_region_shape.type == arm_navigation_msgs::Shape::BOX)
  {
    if((int) m_pc.constraint_region_shape.dimensions.size() < 3)
    {
      out << "Length, width, height must be specified for box constraint region.";
    }
    else
    {
      out << "Box constraint region with dimensions " << m_pc.constraint_region_shape.dimensions[0] << " x "  
          << m_pc.constraint_region_shape.dimensions[1] << " x "  <<  m_pc.constraint_region_shape.dimensions[2] << std::endl;
    }   
  }
  else if (m_pc.constraint_region_shape.type == arm_navigation_msgs::Shape::CYLINDER)
  {
    if((int) m_pc.constraint_region_shape.dimensions.size() < 2)
    {
      out << "Radius and height must be specified for cylinder constraint region.";
    }
    else
    {
      out << "Cylinder constraint region with radius " << m_pc.constraint_region_shape.dimensions[0] << " and length "  
          << m_pc.constraint_region_shape.dimensions[1] << std::endl;
    }   
  }
  else if(m_pc.constraint_region_shape.type == arm_navigation_msgs::Shape::MESH)
  {
    out << "Mesh type constraint region.";         
  }
}

void planning_environment::OrientationConstraintEvaluator::print(std::ostream &out) const
{
  out << "Orientation constraint on link '" << m_oc.link_name << "'" << std::endl;
  out << "Desired orientation:" << m_oc.orientation.x << "," <<  m_oc.orientation.y << "," <<  m_oc.orientation.z << "," << m_oc.orientation.w << std::endl;
}

void planning_environment::KinematicConstraintEvaluatorSet::clear(void)
{
  for (unsigned int i = 0 ; i < m_kce.size() ; ++i)
    delete m_kce[i];
  m_kce.clear();	
  m_jc.clear();
  m_pc.clear();
  m_oc.clear();
}
	
bool planning_environment::KinematicConstraintEvaluatorSet::add(const std::vector<arm_navigation_msgs::JointConstraint> &jc)
{
  bool result = true;
  for (unsigned int i = 0 ; i < jc.size() ; ++i)
  {
    JointConstraintEvaluator *ev = new JointConstraintEvaluator();
    result = result && ev->use(jc[i]);
    m_kce.push_back(ev);
    m_jc.push_back(jc[i]);
  }
  return result;
}

bool planning_environment::KinematicConstraintEvaluatorSet::add(const std::vector<arm_navigation_msgs::PositionConstraint> &pc)
{
  bool result = true;
  for (unsigned int i = 0 ; i < pc.size() ; ++i)
  {
    PositionConstraintEvaluator *ev = new PositionConstraintEvaluator();
    result = result && ev->use(pc[i]);
    m_kce.push_back(ev);
    m_pc.push_back(pc[i]);
  }
  return result;
}

bool planning_environment::KinematicConstraintEvaluatorSet::add(const std::vector<arm_navigation_msgs::OrientationConstraint> &oc)
{
  bool result = true;
  for (unsigned int i = 0 ; i < oc.size() ; ++i)
  {
    OrientationConstraintEvaluator *ev = new OrientationConstraintEvaluator();
    result = result && ev->use(oc[i]);
    m_kce.push_back(ev);
    m_oc.push_back(oc[i]);
  }
  return result;
}


bool planning_environment::KinematicConstraintEvaluatorSet::add(const std::vector<arm_navigation_msgs::VisibilityConstraint> &vc)
{
  bool result = true;
  for (unsigned int i = 0 ; i < vc.size() ; ++i)
  {
    VisibilityConstraintEvaluator *ev = new VisibilityConstraintEvaluator();
    result = result && ev->use(vc[i]);
    m_kce.push_back(ev);
    m_vc.push_back(vc[i]);
  }
  return result;
}

bool planning_environment::KinematicConstraintEvaluatorSet::decide(const planning_models::KinematicState* state, 
                                                                   bool verbose) const
{
  for (unsigned int i = 0 ; i < m_kce.size() ; ++i) {
    if (!m_kce[i]->decide(state, verbose)) {
      return false;            
    }
  }
  return true;
}

void planning_environment::KinematicConstraintEvaluatorSet::print(std::ostream &out) const
{
  out << m_kce.size() << " kinematic constraints" << std::endl;
  for (unsigned int i = 0 ; i < m_kce.size() ; ++i)
    m_kce[i]->print(out);
}

void planning_environment::VisibilityConstraintEvaluator::print(std::ostream &out) const
{
  out << "Visibility constraint for sensor on link '" << m_vc.sensor_pose.header.frame_id << "'" << std::endl;
}
const arm_navigation_msgs::VisibilityConstraint& planning_environment::VisibilityConstraintEvaluator::getConstraintMessage(void) const
{
  return m_vc;
}
void planning_environment::VisibilityConstraintEvaluator::clear(void)
{
}
bool planning_environment::VisibilityConstraintEvaluator::use(const arm_navigation_msgs::VisibilityConstraint &vc)
{
  m_vc   = vc;
  tf::poseMsgToTF(m_vc.sensor_pose.pose,m_sensor_offset_pose);
  return true;
}
bool planning_environment::VisibilityConstraintEvaluator::decide(const planning_models::KinematicState* state,
                                                                 bool verbose) const
{
  std::string link_name = m_vc.sensor_pose.header.frame_id;
  const planning_models::KinematicState::LinkState* link_state = state->getLinkState(link_name);
  if(!link_state) {
    ROS_WARN_STREAM("No link state for link " << link_name);
    return false;
  }

  tf::Transform sensor_pose = link_state->getGlobalLinkTransform() * m_sensor_offset_pose;
  double dx = m_vc.target.point.x - sensor_pose.getOrigin().x();
  double dy = m_vc.target.point.y - sensor_pose.getOrigin().y();
  double dz = m_vc.target.point.z - sensor_pose.getOrigin().z();

  tf::Vector3 x_axis(1,0,0);
  tf::Vector3 target_vector(dx,dy,dz);
  tf::Vector3 sensor_x_axis = sensor_pose.getBasis()*x_axis;

  double angle = fabs(target_vector.angle(sensor_x_axis));
  if(angle < m_vc.absolute_tolerance)
    return true;
  else
    return false;
}

