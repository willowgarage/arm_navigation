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

#include <planning_environment/models/model_utils.h>
#include <geometric_shapes/bodies.h>
#include <planning_environment/util/construct_object.h>

//returns true if the joint_state_map sets all the joints in the state, 
bool planning_environment::setRobotStateAndComputeTransforms(const arm_navigation_msgs::RobotState &robot_state,
                                                             planning_models::KinematicState& state)
{
  if(robot_state.joint_state.name.size() != robot_state.joint_state.position.size()) {
    ROS_INFO_STREAM("Different number of names and positions: " << robot_state.joint_state.name.size() 
                    << " " << robot_state.joint_state.position.size());
    return false;
  }
  std::map<std::string, double> joint_state_map;
  for (unsigned int i = 0 ; i < robot_state.joint_state.name.size(); ++i)
  {
    joint_state_map[robot_state.joint_state.name[i]] = robot_state.joint_state.position[i];
  }
  std::vector<std::string> missing_states;
  bool complete = state.setKinematicState(joint_state_map, missing_states);
  if(!complete) {
    if(missing_states.empty()) {
      ROS_INFO("Incomplete, but no missing states");
    }
  }
  std::map<std::string, bool> has_missing_state_map;
  for(unsigned int i = 0; i < missing_states.size(); i++) {
    has_missing_state_map[missing_states[i]] = false;
  }
  bool need_to_update = false;
  if(robot_state.multi_dof_joint_state.joint_names.size() > 1) {
    ROS_INFO("More than 1 joint names");
  }
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
      need_to_update = true;
      //setting true for all individual joint names because we're setting the transform
      for(unsigned int i = 0; i < joint_state->getJointStateNameOrder().size(); i++) {
        has_missing_state_map[joint_state->getJointStateNameOrder()[i]] = true;
      }
    }
  }
  if(need_to_update) {
    state.updateKinematicLinks();
  }
  if(!complete) {
    for(std::map<std::string, bool>::iterator it = has_missing_state_map.begin();
        it != has_missing_state_map.end();
        it++) {
      if(!it->second) {
        return false;
      }
    }
    return true;
  }
  return true;
}

void planning_environment::convertKinematicStateToRobotState(const planning_models::KinematicState& kinematic_state,
                                                             const ros::Time& timestamp,
                                                             const std::string& header_frame,
                                                             arm_navigation_msgs::RobotState &robot_state)
{
  robot_state.joint_state.position.clear();
  robot_state.joint_state.name.clear();

  robot_state.multi_dof_joint_state.joint_names.clear();
  robot_state.multi_dof_joint_state.frame_ids.clear();
  robot_state.multi_dof_joint_state.child_frame_ids.clear();
  robot_state.multi_dof_joint_state.poses.clear();

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

void planning_environment::applyOrderedCollisionOperationsToMatrix(const arm_navigation_msgs::OrderedCollisionOperations &ord,
                                                                   collision_space::EnvironmentModel::AllowedCollisionMatrix& acm) {
  if(ord.collision_operations.size() == 0) {
    ROS_INFO_STREAM("No allowed collision operations");
  }
  
  for(size_t i = 0; i < ord.collision_operations.size(); i++) {
    
    bool allowed = (ord.collision_operations[i].operation == arm_navigation_msgs::CollisionOperation::DISABLE);

    ROS_INFO_STREAM("Setting collision operation between " << ord.collision_operations[i].object1
                    << " and " << ord.collision_operations[i].object2 << " allowed " << allowed);

    if(ord.collision_operations[i].object1 == ord.collision_operations[i].COLLISION_SET_ALL &&
       ord.collision_operations[i].object2 == ord.collision_operations[i].COLLISION_SET_ALL) {
      acm.changeEntry(allowed);
      ROS_INFO_STREAM("Should be setting all");
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

void planning_environment::convertFromACMToACMMsg(const collision_space::EnvironmentModel::AllowedCollisionMatrix& acm,
                                                  arm_navigation_msgs::AllowedCollisionMatrix& matrix) {
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

collision_space::EnvironmentModel::AllowedCollisionMatrix 
planning_environment::convertFromACMMsgToACM(const arm_navigation_msgs::AllowedCollisionMatrix& matrix)
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

bool planning_environment::applyOrderedCollisionOperationsListToACM(const arm_navigation_msgs::OrderedCollisionOperations& ordered_coll,
                                                                    const std::vector<std::string>& object_names,
                                                                    const std::vector<std::string>& att_names,
                                                                    const planning_models::KinematicModel* model,
                                                                    collision_space::EnvironmentModel::AllowedCollisionMatrix& matrix)
{
  bool all_ok = true;
  for(std::vector<arm_navigation_msgs::CollisionOperation>::const_iterator it = ordered_coll.collision_operations.begin();
      it != ordered_coll.collision_operations.end();
      it++) {
    bool op = (*it).operation != arm_navigation_msgs::CollisionOperation::ENABLE;
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
    if(model->getModelGroup((*it).object2) != NULL) {
      svec2 = model->getModelGroup((*it).object2)->getGroupLinkNames();
      special2 = true;
    }
    if(!special1) {
      svec1.push_back((*it).object1);
    }
    if(!special2) {
      svec2.push_back((*it).object2);
    }

    bool first_all = false;
    bool second_all = false;
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

    ROS_DEBUG_STREAM("Coll op first and second all " << first_all << " " << second_all);
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
      bool ok = matrix.changeEntry(svec1, svec2, (*it).operation != arm_navigation_msgs::CollisionOperation::ENABLE);
      if(!ok) {
        ROS_DEBUG_STREAM("No entry in acm for some member of " << (*it).object1 << " and " << (*it).object2);
        all_ok = false;
      }
    }
  }
  return all_ok;
}               

arm_navigation_msgs::AllowedCollisionMatrix 
planning_environment::applyOrderedCollisionOperationsToCollisionsModel(const planning_environment::CollisionModels* cm,
                                                                       const arm_navigation_msgs::OrderedCollisionOperations& ordered_coll,
                                                                       const std::vector<std::string>& object_names,
                                                                       const std::vector<std::string>& att_names)
{
  collision_space::EnvironmentModel::AllowedCollisionMatrix acm = cm->getDefaultAllowedCollisionMatrix();
  
  for(unsigned int i = 0; i < object_names.size(); i++) {
    if(!acm.hasEntry(object_names[i])) {
      acm.addEntry(object_names[i], false);
    } 
  }

  for(unsigned int i = 0; i < att_names.size(); i++) {
    if(!acm.hasEntry(att_names[i])) {
      acm.addEntry(att_names[i], false);
    } 
  }

  applyOrderedCollisionOperationsListToACM(ordered_coll, object_names, att_names, cm->getKinematicModel(), acm);
                         
  arm_navigation_msgs::AllowedCollisionMatrix ret_msg;
  convertFromACMToACMMsg(acm, ret_msg);
  return ret_msg;
}

bool planning_environment::doesKinematicStateObeyConstraints(const planning_models::KinematicState& state,
                                                             const arm_navigation_msgs::Constraints& constraints,
                                                             bool verbose) {
  planning_environment::KinematicConstraintEvaluatorSet constraint_evaluator;
  
  constraint_evaluator.add(constraints.joint_constraints);
  constraint_evaluator.add(constraints.position_constraints);
  constraint_evaluator.add(constraints.orientation_constraints);
  constraint_evaluator.add(constraints.visibility_constraints);
  return(constraint_evaluator.decide(&state, verbose));
}

void planning_environment::setMarkerShapeFromShape(const arm_navigation_msgs::Shape &obj, visualization_msgs::Marker &mk)
{
  switch (obj.type)
  {
  case arm_navigation_msgs::Shape::SPHERE:
    mk.type = visualization_msgs::Marker::SPHERE;
    mk.scale.x = mk.scale.y = mk.scale.z = obj.dimensions[0] * 2.0;
    break;
    
  case arm_navigation_msgs::Shape::BOX:
    mk.type = visualization_msgs::Marker::CUBE;
    mk.scale.x = obj.dimensions[0];
    mk.scale.y = obj.dimensions[1];
    mk.scale.z = obj.dimensions[2];
    break;
    
  case arm_navigation_msgs::Shape::CYLINDER:
    mk.type = visualization_msgs::Marker::CYLINDER;
    mk.scale.x = obj.dimensions[0] * 2.0;
    mk.scale.y = obj.dimensions[0] * 2.0;
    mk.scale.z = obj.dimensions[1];
    break;
    
  case arm_navigation_msgs::Shape::MESH:
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

void planning_environment::setMarkerShapeFromShape(const shapes::Shape *obj, visualization_msgs::Marker &mk, double padding)
{
  switch (obj->type)
  {
  case shapes::SPHERE:
    mk.type = visualization_msgs::Marker::SPHERE;
    mk.scale.x = mk.scale.y = mk.scale.z = static_cast<const shapes::Sphere*>(obj)->radius * 2.0 + padding;
    break;
    
  case shapes::BOX:
    mk.type = visualization_msgs::Marker::CUBE;
    {
      const double *size = static_cast<const shapes::Box*>(obj)->size;
      mk.scale.x = size[0] + padding*2.0;
      mk.scale.y = size[1] + padding*2.0;
      mk.scale.z = size[2] + padding*2.0;
    }
    break;
    
  case shapes::CYLINDER:
    mk.type = visualization_msgs::Marker::CYLINDER;
    mk.scale.x = static_cast<const shapes::Cylinder*>(obj)->radius * 2.0 + padding;
    mk.scale.y = mk.scale.x;
    mk.scale.z = static_cast<const shapes::Cylinder*>(obj)->length + padding*2.0;
    break;
    
  case shapes::MESH:
    mk.type = visualization_msgs::Marker::LINE_LIST;
    mk.scale.x = mk.scale.y = mk.scale.z = 0.001;
    {	   
      const shapes::Mesh *mesh = static_cast<const shapes::Mesh*>(obj);
      double* vertices = new double[mesh->vertexCount * 3];
      double sx = 0.0, sy = 0.0, sz = 0.0;
      for(unsigned int i = 0; i < mesh->vertexCount; ++i) {
        unsigned int i3 = i * 3;
        vertices[i3] = mesh->vertices[i3];
        vertices[i3 + 1] = mesh->vertices[i3 + 1];
        vertices[i3 + 2] = mesh->vertices[i3 + 2];
        sx += vertices[i3];
        sy += vertices[i3 + 1];
        sz += vertices[i3 + 2];
      }
      // the center of the mesh
      sx /= (double)mesh->vertexCount;
      sy /= (double)mesh->vertexCount;
      sz /= (double)mesh->vertexCount;
      
      for (unsigned int i = 0 ; i < mesh->vertexCount ; ++i)
      {
        unsigned int i3 = i * 3;
	
        // vector from center to the vertex
        double dx = vertices[i3] - sx;
        double dy = vertices[i3 + 1] - sy;
        double dz = vertices[i3 + 2] - sz;
	
        // length of vector
        //double norm = sqrt(dx * dx + dy * dy + dz * dz);
	
        double ndx = ((dx > 0) ? dx+padding : dx-padding);
        double ndy = ((dy > 0) ? dy+padding : dy-padding);
        double ndz = ((dz > 0) ? dz+padding : dz-padding);
        
        // the new distance of the vertex from the center
        //double fact = scale + padding/norm;
        vertices[i3] = sx + ndx; //dx * fact;
        vertices[i3 + 1] = sy + ndy; //dy * fact;
        vertices[i3 + 2] = sz + ndz; //dz * fact;		    
      }
      
      tf::Transform trans;
      tf::poseMsgToTF(mk.pose, trans);

      for (unsigned int j = 0 ; j < mesh->triangleCount; ++j) {
        unsigned int t1ind = mesh->triangles[3*j];
        unsigned int t2ind = mesh->triangles[3*j + 1];
        unsigned int t3ind = mesh->triangles[3*j + 2];
        
        tf::Vector3 vec1(vertices[t1ind*3],
                       vertices[t1ind*3+1],
                       vertices[t1ind*3+2]);
        
        tf::Vector3 vec2(vertices[t2ind*3],
                       vertices[t2ind*3+1],
                       vertices[t2ind*3+2]);
        
        tf::Vector3 vec3(vertices[t3ind*3],
                       vertices[t3ind*3+1],
                       vertices[t3ind*3+2]);
        
        //vec1 = trans*vec1;
        //vec2 = trans*vec2;
        //vec3 = trans*vec3;
        
        geometry_msgs::Point pt1;
        pt1.x = vec1.x();
        pt1.y = vec1.y();
        pt1.z = vec1.z();

        geometry_msgs::Point pt2;
        pt2.x = vec2.x();
        pt2.y = vec2.y();
        pt2.z = vec2.z();
        
        geometry_msgs::Point pt3;
        pt3.x = vec3.x();
        pt3.y = vec3.y();
        pt3.z = vec3.z();
        
        mk.points.push_back(pt1);
        mk.points.push_back(pt2);
        
        mk.points.push_back(pt1);
        mk.points.push_back(pt3);
        
        mk.points.push_back(pt2);
        mk.points.push_back(pt3);
      }
      delete[] vertices;
    }
    break;
    
  default:
    ROS_ERROR("Unknown object type: %d", (int)obj->type);
  }
}

void planning_environment::convertFromLinkPaddingMapToLinkPaddingVector(const std::map<std::string, double>& link_padding_map,
                                                                        std::vector<arm_navigation_msgs::LinkPadding>& link_padding_vector)
{
  for(std::map<std::string, double>::const_iterator it = link_padding_map.begin();
      it != link_padding_map.end();
      it++) {
    arm_navigation_msgs::LinkPadding lp;
    lp.link_name = it->first;
    lp.padding = it->second;
    link_padding_vector.push_back(lp);
  }
}

void planning_environment::getAllKinematicStateStampedTransforms(const planning_models::KinematicState& state,
                                                                 std::vector<geometry_msgs::TransformStamped>& trans_vector,
                                                                 const ros::Time& stamp)
{
  trans_vector.clear();
  for(unsigned int i = 0; i < state.getLinkStateVector().size(); i++) {      
    const planning_models::KinematicState::LinkState* ls = state.getLinkStateVector()[i];
    geometry_msgs::TransformStamped ts;
    ts.header.stamp = stamp;
    ts.header.frame_id = state.getKinematicModel()->getRoot()->getParentFrameId();
    ts.child_frame_id = ls->getName();
    if(ts.header.frame_id == ts.child_frame_id) continue; 
    tf::transformTFToMsg(ls->getGlobalLinkTransform(),ts.transform);
    trans_vector.push_back(ts);
  }
} 

void planning_environment::convertAllowedContactSpecificationMsgToAllowedContactVector(const std::vector<arm_navigation_msgs::AllowedContactSpecification>& acmv,
                                                                                       std::vector<collision_space::EnvironmentModel::AllowedContact>& acv)
{
  //assumes that poses are in the global frame
  acv.clear();
  for(unsigned int i = 0; i < acmv.size(); i++) {
    const arm_navigation_msgs::AllowedContactSpecification& acs = acmv[i];
    if(acs.link_names.size() != 2) {
      ROS_WARN_STREAM("Allowed collision specification has link_names size " << acs.link_names.size()
                      << " while the only supported size is 2");
      continue;
    }
    shapes::Shape* shape = constructObject(acs.shape);
    boost::shared_ptr<bodies::Body> bodysp(bodies::createBodyFromShape(shape));
    delete shape;
    tf::Transform trans;
    tf::poseMsgToTF(acs.pose_stamped.pose, trans);
    bodysp->setPose(trans);

    collision_space::EnvironmentModel::AllowedContact allc;
    allc.bound = bodysp;
    allc.body_name_1 = acs.link_names[0];
    allc.body_name_2 = acs.link_names[1];
    allc.depth = acs.penetration_depth;
    acv.push_back(allc);
  }
}

void planning_environment::getCollisionMarkersFromContactInformation(const std::vector<arm_navigation_msgs::ContactInformation>& coll_info_vec,
                                                                     const std::string& world_frame_id,
                                                                     visualization_msgs::MarkerArray& arr,
                                                                     const std_msgs::ColorRGBA& color,
                                                                     const ros::Duration& lifetime)
{
  std::map<std::string, unsigned> ns_counts;
  for(unsigned int i = 0; i < coll_info_vec.size(); i++) {
    std::string ns_name;
    ns_name = coll_info_vec[i].contact_body_1;
    ns_name +="+";
    ns_name += coll_info_vec[i].contact_body_2;
    if(ns_counts.find(ns_name) == ns_counts.end()) {
      ns_counts[ns_name] = 0;
    } else {
      ns_counts[ns_name]++;
    }
    visualization_msgs::Marker mk;
    mk.header.stamp = ros::Time::now();
    mk.header.frame_id = world_frame_id;
    mk.ns = ns_name;
    mk.id = ns_counts[ns_name];
    mk.type = visualization_msgs::Marker::SPHERE;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.position.x = coll_info_vec[i].position.x;
    mk.pose.position.y = coll_info_vec[i].position.y;
    mk.pose.position.z = coll_info_vec[i].position.z;
    mk.pose.orientation.w = 1.0;
    
    mk.scale.x = mk.scale.y = mk.scale.z = 0.035;

    mk.color.a = color.a;
    if(mk.color.a == 0.0) {
      mk.color.a = 1.0;
    }
    mk.color.r = color.r;
    mk.color.g = color.g;
    mk.color.b = color.b;
    
    mk.lifetime = lifetime;
    arr.markers.push_back(mk);
  }
}
