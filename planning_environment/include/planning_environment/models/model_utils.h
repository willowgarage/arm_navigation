/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2010, Willow Garage, Inc.
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

/** \author E. Gil Jones */

#ifndef _MODEL_UTILS_H_
#define _MODEL_UTILS_H_

#include <tf/tf.h>
#include <planning_models/kinematic_state.h>
#include <motion_planning_msgs/RobotState.h>
#include <motion_planning_msgs/Constraints.h>
#include <motion_planning_msgs/OrderedCollisionOperations.h>

#include <planning_environment/util/kinematic_state_constraint_evaluator.h>
#include <geometric_shapes_msgs/Shape.h>
#include <visualization_msgs/Marker.h>
#include <motion_planning_msgs/LinkPadding.h>
#include <collision_space/environment.h>
#include <planning_environment_msgs/AllowedCollisionMatrix.h>

namespace planning_environment {

//returns true if the joint_state_map sets all the joints in the state, 
inline bool setRobotStateAndComputeTransforms(const motion_planning_msgs::RobotState &robot_state,
                                              planning_models::KinematicState& state)
{
  std::map<std::string, double> joint_state_map;
  for (unsigned int i = 0 ; i < robot_state.joint_state.name.size(); ++i)
  {
    joint_state_map[robot_state.joint_state.name[i]] = robot_state.joint_state.position[i];
  }

  //first we are going to apply transforms
  for(unsigned int i = 0; i < robot_state.multi_dof_joint_state.joint_names.size(); i++) {
    std::string joint_name = robot_state.multi_dof_joint_state.joint_names[i];
    if(!state.hasJointState(joint_name)) {
      ROS_WARN_STREAM("No joint matching multi-dof joint " << joint_name);
      continue;
    }
    planning_models::KinematicState::JointState* joint_state = state.getJointState(joint_name);
    if(robot_state.multi_dof_joint_state.frame_ids.size() <= i) {
      ROS_INFO_STREAM("Robot state msg had bad multi_dof transform - not enough frame ids");
    } else if(robot_state.multi_dof_joint_state.child_frame_ids.size() <= i) {
      ROS_INFO_STREAM("Robot state msg had bad multi_dof transform - not enough child frame ids");
    } else if(robot_state.multi_dof_joint_state.frame_ids[i] != joint_state->getParentFrameId() ||
              robot_state.multi_dof_joint_state.child_frame_ids[i] != joint_state->getChildFrameId()) {
      ROS_WARN_STREAM("Robot state msg has bad multi_dof transform - frame ids or child frame_ids do not match up with joint");
    } else {
      tf::StampedTransform transf;
      tf::poseMsgToTF(robot_state.multi_dof_joint_state.poses[i], transf);
      joint_state->setJointStateValues(transf);
    }
  }
  return(state.setKinematicState(joint_state_map));
}

inline void convertKinematicStateToRobotState(const planning_models::KinematicState& kinematic_state,
                                       const ros::Time& timestamp,
                                       const std::string& header_frame,
                                       motion_planning_msgs::RobotState &robot_state)
{
  robot_state.joint_state.position.clear();
  robot_state.joint_state.name.clear();

  const std::vector<planning_models::KinematicState::JointState*>& joints = kinematic_state.getJointStateVector();
  for(std::vector<planning_models::KinematicState::JointState*>::const_iterator it = joints.begin();
      it != joints.end();
      it++) {
    const std::vector<double>& joint_state_values = (*it)->getJointStateValues();
    const std::vector<std::string>& joint_state_value_names = (*it)->getJointStateNameOrder();
    for(unsigned int i = 0; i < joint_state_values.size(); i++) {
      robot_state.joint_state.name.push_back(joint_state_value_names[i]);
      robot_state.joint_state.position.push_back(joint_state_values[i]);
    }
    if(!(*it)->getParentFrameId().empty() && !(*it)->getChildFrameId().empty()) {
      robot_state.multi_dof_joint_state.stamp = ros::Time::now();
      robot_state.multi_dof_joint_state.joint_names.push_back((*it)->getName());
      robot_state.multi_dof_joint_state.frame_ids.push_back((*it)->getParentFrameId());
      robot_state.multi_dof_joint_state.child_frame_ids.push_back((*it)->getChildFrameId());
      tf::Pose p((*it)->getVariableTransform());
      geometry_msgs::Pose pose;
      tf::poseTFToMsg(p, pose);
      robot_state.multi_dof_joint_state.poses.push_back(pose);
    }
  }
  robot_state.joint_state.header.stamp = timestamp;
  robot_state.joint_state.header.frame_id = header_frame;
  return;
}

inline void applyOrderedCollisionOperationsToMatrix(const motion_planning_msgs::OrderedCollisionOperations &ord,
                                                    collision_space::EnvironmentModel::AllowedCollisionMatrix& acm) {
  for(size_t i = 0; i < ord.collision_operations.size(); i++) {
    
    bool allowed = (ord.collision_operations[i].operation == motion_planning_msgs::CollisionOperation::DISABLE);
    
    if(ord.collision_operations[i].object1 == ord.collision_operations[i].COLLISION_SET_ALL &&
       ord.collision_operations[i].object2 == ord.collision_operations[i].COLLISION_SET_ALL) {
      acm.changeEntry(allowed);
    } else if(ord.collision_operations[i].object1 == ord.collision_operations[i].COLLISION_SET_ALL ||
              ord.collision_operations[i].object2 == ord.collision_operations[i].COLLISION_SET_ALL) {
      //second case - only one all
      std::string other;
      if(ord.collision_operations[i].object1 == ord.collision_operations[i].COLLISION_SET_ALL) {
        other = ord.collision_operations[i].object2;
      } else {
        other = ord.collision_operations[i].object1;
      }
      bool ok = acm.changeEntry(other,allowed);
      if(!ok) {
        ROS_WARN_STREAM("No allowed collision entry for " << other);
      }
    } else {
      //third case - must be both objects
      bool ok = acm.changeEntry(ord.collision_operations[i].object1,
                                 ord.collision_operations[i].object2,
                                 allowed);
      if(!ok) {
        ROS_WARN_STREAM("No entry in allowed collision matrix for at least one of " 
                        << ord.collision_operations[i].object1 << " and " 
                        << ord.collision_operations[i].object2);
      }
    }
  }
}

inline void convertFromACMToACMMsg(const collision_space::EnvironmentModel::AllowedCollisionMatrix& acm,
                                   planning_environment_msgs::AllowedCollisionMatrix& matrix) {
  if(!acm.getValid()) return;
  matrix.link_names.resize(acm.getSize());
  matrix.entries.resize(acm.getSize());
  const collision_space::EnvironmentModel::AllowedCollisionMatrix::entry_type& bm = acm.getEntriesBimap();
  for(collision_space::EnvironmentModel::AllowedCollisionMatrix::entry_type::right_const_iterator it = bm.right.begin();
      it != bm.right.end();
      it++) {
    matrix.link_names[it->first] = it->second;
    for(unsigned int i = 0; i < acm.getSize(); i++) {
      bool allowed;
      matrix.entries[i].enabled.resize(acm.getSize());
      acm.getAllowedCollision(it->first, i, allowed); 
      if(it->first >= matrix.entries[i].enabled.size()) {
        ROS_WARN_STREAM("Trouble size " << matrix.entries[i].enabled.size() << " ind " << it->first); 
      } else {
        matrix.entries[it->first].enabled[i] = allowed;
      }
    }
  }
}

inline collision_space::EnvironmentModel::AllowedCollisionMatrix convertFromACMMsgToACM(const planning_environment_msgs::AllowedCollisionMatrix& matrix)
{
  std::map<std::string, unsigned int> indices;
  std::vector<std::vector<bool> > vecs;
  unsigned int ns = matrix.link_names.size();
  if(matrix.entries.size() != ns) {
    ROS_WARN_STREAM("Matrix messed up");
  }
  vecs.resize(ns);
  for(unsigned int i = 0; i < std::min(ns, (unsigned int) matrix.entries.size()); i++) {
    indices[matrix.link_names[i]] = i;
    if(matrix.entries[i].enabled.size() != ns) {
      ROS_WARN_STREAM("Matrix messed up");
    }
    vecs[i].resize(ns);
    for(unsigned int j = 0; j < std::min(ns, (unsigned int) matrix.entries[i].enabled.size()); j++) {
      vecs[i][j] = matrix.entries[i].enabled[j];
    }
  }
  collision_space::EnvironmentModel::AllowedCollisionMatrix acm(vecs,indices);
  return acm;
}

inline bool applyOrderedCollisionOperationsListToACM(const motion_planning_msgs::OrderedCollisionOperations& ordered_coll,
                                                     const std::vector<std::string>& object_names,
                                                     const std::vector<std::string>& att_names,
                                                     const planning_models::KinematicModel* model,
                                                     collision_space::EnvironmentModel::AllowedCollisionMatrix& matrix)
{
  bool all_ok = true;
  for(std::vector<motion_planning_msgs::CollisionOperation>::const_iterator it = ordered_coll.collision_operations.begin();
      it != ordered_coll.collision_operations.end();
      it++) {
    bool op = (*it).operation != motion_planning_msgs::CollisionOperation::ENABLE;
    std::vector<std::string> svec1, svec2;
    bool special1 = false;
    bool special2 = false;
    if((*it).object1 == (*it).COLLISION_SET_OBJECTS) {
      svec1 = object_names;
      special1 = true;
    }
    if((*it).object2 == (*it).COLLISION_SET_OBJECTS) {
      svec2 = object_names;
      special2 = true;
    }
    if((*it).object1 == (*it).COLLISION_SET_ATTACHED_OBJECTS) {
      svec1 = att_names;
      special1 = true;
    }
    if((*it).object2 == (*it).COLLISION_SET_ATTACHED_OBJECTS) {
      svec2 = att_names;
      special2 = true;
    }
    if(model->getModelGroup((*it).object1) != NULL) {
      svec1 = model->getModelGroup((*it).object1)->getGroupLinkNames();
      special1 = true;
    }
    if(model->getModelGroup((*it).object2)) {
      svec2 = model->getModelGroup((*it).object1)->getGroupLinkNames();
      special2 = true;
    }
    if(!special1) {
      svec1.push_back((*it).object1);
    }
    if(!special2) {
      svec2.push_back((*it).object2);
    }

    bool first_all = false;
    bool second_all = true;
    for(unsigned int j = 0; j < svec1.size(); j++) {
      if(svec1[j] == (*it).COLLISION_SET_ALL) {
        first_all = true;
        if(svec1.size() > 1) {
          ROS_WARN("Shouldn't have collision object all in multi-item list");
          all_ok = false;
        }
        break;
      }
    }
    for(unsigned int j = 0; j < svec2.size(); j++) {
      if(svec2[j] == (*it).COLLISION_SET_ALL) {
        second_all = true;
        if(svec2.size() > 1) {
          ROS_WARN("Shouldn't have collision object all in multi-item list");
          all_ok = false;
        }
        break;
      }
    }
    if(first_all && second_all) {
      matrix.changeEntry(op);
    } else if(first_all) {
      for(unsigned int j = 0; j < svec2.size(); j++) {
        matrix.changeEntry(svec2[j], op);
      }
    } else if(second_all) {
      for(unsigned int j = 0; j < svec1.size(); j++) {
        matrix.changeEntry(svec1[j], op);
      }
    } else {
      bool ok = matrix.changeEntry(svec1, svec2, (*it).operation != motion_planning_msgs::CollisionOperation::ENABLE);
      if(!ok) {
        ROS_INFO_STREAM("No entry in acm for some member of " << (*it).object1 << " and " << (*it).object2);
        all_ok = false;
      }
    }
  }
  return all_ok;
}               
       
/*
inline void printAllowedCollisionMatrix(const std::vector<std::vector<bool> > &curAllowed,
                                 const std::map<std::string, unsigned int> &vecIndices) {
  size_t all_size = curAllowed.size();
  for(unsigned int i = 0; i < vecIndices.size(); i++) {
    std::string n;
    for(std::map<std::string, unsigned int>::const_iterator it = vecIndices.begin();
        it != vecIndices.end();
        it++) {
      if(it->second == i) {
        n = it->first; 
      }
    }
    if(n.empty()) {
      ROS_WARN_STREAM("Can't find index " << i << " in vecIndex");
      return;
    }
    std::cout << std::setw(40) << n;
    std::cout << " | ";
    for(size_t j = 0; j < all_size; j++) {
      std::cout << std::setw(3) << curAllowed[i][j];
    }
    std::cout << std::endl;
  }
}
*/
inline bool doesKinematicStateObeyConstraints(const planning_models::KinematicState& state,
                                       const motion_planning_msgs::Constraints& constraints,
                                       bool verbose = false) {
  planning_environment::KinematicConstraintEvaluatorSet constraint_evaluator;
  
  constraint_evaluator.add(constraints.joint_constraints);
  constraint_evaluator.add(constraints.position_constraints);
  constraint_evaluator.add(constraints.orientation_constraints);
  constraint_evaluator.add(constraints.visibility_constraints);
  return(constraint_evaluator.decide(&state, verbose));
}

inline void setMarkerShapeFromShape(const geometric_shapes_msgs::Shape &obj, visualization_msgs::Marker &mk)
{
  switch (obj.type)
  {
  case geometric_shapes_msgs::Shape::SPHERE:
    mk.type = visualization_msgs::Marker::SPHERE;
    mk.scale.x = mk.scale.y = mk.scale.z = obj.dimensions[0] * 2.0;
    break;
    
  case geometric_shapes_msgs::Shape::BOX:
    mk.type = visualization_msgs::Marker::CUBE;
    mk.scale.x = obj.dimensions[0];
    mk.scale.y = obj.dimensions[1];
    mk.scale.z = obj.dimensions[2];
    break;
    
  case geometric_shapes_msgs::Shape::CYLINDER:
    mk.type = visualization_msgs::Marker::CYLINDER;
    mk.scale.x = obj.dimensions[0] * 2.0;
    mk.scale.y = obj.dimensions[0] * 2.0;
    mk.scale.z = obj.dimensions[1];
    break;
    
  case geometric_shapes_msgs::Shape::MESH:
    mk.type = visualization_msgs::Marker::LINE_LIST;
    mk.scale.x = mk.scale.y = mk.scale.z = 0.001;
    {
      unsigned int nt = obj.triangles.size() / 3;
      for (unsigned int i = 0 ; i < nt ; ++i)
      {
        mk.points.push_back(obj.vertices[obj.triangles[3*i]]);
        mk.points.push_back(obj.vertices[obj.triangles[3*i+ 1]]);
        mk.points.push_back(obj.vertices[obj.triangles[3*i]]);
        mk.points.push_back(obj.vertices[obj.triangles[3*i+2]]);
        mk.points.push_back(obj.vertices[obj.triangles[3*i+1]]);
        mk.points.push_back(obj.vertices[obj.triangles[3*i+2]]);
      }
    }
    
    break;
    
  default:
    ROS_ERROR("Unknown object type: %d", (int)obj.type);
  }
}

inline void setMarkerShapeFromShape(const shapes::Shape *obj, visualization_msgs::Marker &mk)
{
  switch (obj->type)
  {
  case shapes::SPHERE:
    mk.type = visualization_msgs::Marker::SPHERE;
    mk.scale.x = mk.scale.y = mk.scale.z = static_cast<const shapes::Sphere*>(obj)->radius * 2.0;
    break;
    
  case shapes::BOX:
    mk.type = visualization_msgs::Marker::CUBE;
    {
      const double *size = static_cast<const shapes::Box*>(obj)->size;
      mk.scale.x = size[0];
      mk.scale.y = size[1];
      mk.scale.z = size[2];
    }
    break;
    
  case shapes::CYLINDER:
    mk.type = visualization_msgs::Marker::CYLINDER;
    mk.scale.x = static_cast<const shapes::Cylinder*>(obj)->radius * 2.0;
    mk.scale.y = mk.scale.x;
    mk.scale.z = static_cast<const shapes::Cylinder*>(obj)->length;
    break;
    
  case shapes::MESH:
    mk.type = visualization_msgs::Marker::LINE_LIST;
    mk.scale.x = mk.scale.y = mk.scale.z = 0.001;
    {	   
      const shapes::Mesh *mesh = static_cast<const shapes::Mesh*>(obj);
      unsigned int nt = mesh->triangleCount / 3;
      for (unsigned int i = 0 ; i < nt ; ++i)
      {
        unsigned int v = mesh->triangles[3*i];
        geometry_msgs::Point pt1;
        pt1.x = mesh->vertices[v];
        pt1.y = mesh->vertices[v+1];
        pt1.z = mesh->vertices[v+2];
        mk.points.push_back(pt1);
        
        v = mesh->triangles[3*i + 1];
        geometry_msgs::Point pt2;
        pt2.x = mesh->vertices[v];
        pt2.y = mesh->vertices[v+1];
        pt2.z = mesh->vertices[v+2];
        mk.points.push_back(pt2);
        
        mk.points.push_back(pt1);
        
        v = mesh->triangles[3*i + 2];
        geometry_msgs::Point pt3;
        pt3.x = mesh->vertices[v];
        pt3.y = mesh->vertices[v+1];
        pt3.z = mesh->vertices[v+2];
        mk.points.push_back(pt3);
        
        mk.points.push_back(pt2);
        mk.points.push_back(pt3);
      }
    }
    
    break;
    
  default:
    ROS_ERROR("Unknown object type: %d", (int)obj->type);
  }
}

inline void convertFromLinkPaddingMapToLinkPaddingVector(const std::map<std::string, double>& link_padding_map,
                                                         std::vector<motion_planning_msgs::LinkPadding>& link_padding_vector)
{
  for(std::map<std::string, double>::const_iterator it = link_padding_map.begin();
      it != link_padding_map.end();
      it++) {
    motion_planning_msgs::LinkPadding lp;
    lp.link_name = it->first;
    lp.padding = it->second;
    link_padding_vector.push_back(lp);
  }
}

}
#endif
