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

#ifndef COLLISION_PROXIMITY_SPACE_
#define COLLISION_PROXIMITY_SPACE_

#include <vector>
#include <string>
#include <algorithm>
#include <sstream>

#include <ros/ros.h>

#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
#include <planning_environment/models/collision_models_interface.h>

#include <collision_proximity/collision_proximity_types.h>

namespace collision_proximity
{
//A class for implementation of proximity queries and proximity-based
//collision queries

class CollisionProximitySpace
{

public:

  CollisionProximitySpace(const std::string& robot_description_name);
  ~CollisionProximitySpace();

  //this function sets up the collision proximity space for making a series of 
  //proximity collision or gradient queries for the indicated group
  void setupForGroupQueries(const std::string& group_name,
                            const motion_planning_msgs::RobotState& rob_state);

  //returns the updating objects lock and destroys the current kinematic state
  void revertAfterGroupQueries();

  // sets the current group given the kinematic state
  void setCurrentGroupState(const planning_models::KinematicState& state);

  // returns true if the current group is in collision in the indicated state.
  // This doesn't affect the distance field or other robot links not in the group
  bool isStateInCollision() const;

  // returns the full set of collision information for each group link
  bool getStateCollisions(std::vector<std::string>& link_names, 
                          std::vector<std::string>& attached_body_names,
                          bool& in_collision, 
                          std::vector<CollisionType>& collisions) const;
  
  // returns the full gradient information for each group_link
  bool getStateGradients(std::vector<std::string>& link_names,
                         std::vector<std::string>& attached_body_names,
                         std::vector<GradientInfo>& gradients, 
                         bool subtract_radii = false) const;

  // returns true if current setup is in environment collision
  bool isEnvironmentCollision() const;

  // returns true if current setup is in intra-group collision
  bool isIntraGroupCollision() const;

  // returns true if current setup is in self collision
  bool isSelfCollision() const;

  // returns the single closest proximity for the group previously configured
  //bool getEnvironmentProximity(ProximityInfo& prox) const;
  
  // returns true or false for environment collisions for the group that's been configured
  //bool getEnvironmentCollision() const;

  //
  //visualization functions
  //

  void getProximityGradientMarkers(const std::vector<std::string>& link_names, 
                                   const std::vector<std::string>& attached_body_names,
                                   const std::vector<GradientInfo>& gradients,
                                   const std::string& ns, 
                                   visualization_msgs::MarkerArray& arr) const;

  void visualizeDistanceField(distance_field::PropagationDistanceField* distance_field) const;

  //void visualizeClosestCollisionSpheres(const std::vector<std::string>& link_names) const;

  void visualizeCollisions(const std::vector<std::string>& link_names, 
                           const std::vector<std::string>& attached_body_names, 
                           const std::vector<CollisionType> collisions) const;

  void visualizeObjectVoxels(const std::vector<std::string>& object_names) const;

  void visualizeObjectSpheres(const std::vector<std::string>& object_names) const;

  void visualizeBoundingCylinders(const std::vector<std::string>& object_names) const;

  const planning_environment::CollisionModelsInterface* getCollisionModelsInterface() const {
    return collision_models_interface_;
  }

  bool setPlanningScene(const planning_environment_msgs::PlanningScene& planning_scene);

  std::vector<std::string> getCurrentLinkNames() const
  {
    return current_link_names_;
  }

  std::vector<std::string> getCurrentAttachedBodyNames() const
  {
    return current_attached_body_names_;
  }

private:

  void setPlanningSceneCallback(const planning_environment_msgs::PlanningScene& scene); 
  void revertPlanningSceneCallback();

  void deleteAllStaticObjectDecompositions();
  void deleteAllAttachedObjectDecompositions();

  // sets the poses of the body to those held in the kinematic state
  void setBodyPosesToCurrent();

  // sets the body poses given the indicated kinematic state
  void setBodyPosesGivenKinematicState(const planning_models::KinematicState& state);

  void setDistanceFieldForGroupQueries(const std::string& group_name,
                                       const planning_models::KinematicState& state);

  bool getIntraGroupCollisions(std::vector<bool>& collisions,
                               bool stop_at_first = false) const;
  
  bool getIntraGroupProximityGradients(std::vector<GradientInfo>& gradients,
                                       bool subtract_radii = false) const;

  bool getSelfCollisions(std::vector<bool>& collisions,
                               bool stop_at_first = false) const;
  
  bool getSelfProximityGradients(std::vector<GradientInfo>& gradients,
                                       bool subtract_radii = false) const;

  bool getEnvironmentCollisions(std::vector<bool>& collisions,
                                bool stop_at_first = false) const;
  
  bool getEnvironmentProximityGradients(std::vector<GradientInfo>& gradients,
                                        bool subtract_radii = false) const;

  bool getGroupLinkAndAttachedBodyNames(const std::string& group_name,
                                        const planning_models::KinematicState& state,
                                        std::vector<std::string>& link_names,
                                        std::vector<unsigned int>& link_indices,
                                        std::vector<std::string>& attached_body_names,
                                        std::vector<unsigned int>& attached_body_link_indices) const; 
  
  bool setupGradientStructures(const std::vector<std::string>& link_names,
                               const std::vector<std::string>& attached_body_names, 
                               std::vector<GradientInfo>& gradients) const;

  void prepareEnvironmentDistanceField(const planning_models::KinematicState& state);

  void prepareSelfDistanceField(const std::vector<std::string>& link_names, 
                                const planning_models::KinematicState& state);


  //double getCollisionSphereProximity(const std::vector<CollisionSphere>& sphere_list, 
  //                                  unsigned int& closest, btVector3& grad) const;

  btTransform getInverseWorldTransform(const planning_models::KinematicState& state) const;

  void syncObjectsWithCollisionSpace(const planning_models::KinematicState& state);

  //configuration convenience functions
  void loadRobotBodyDecompositions();
  void loadDefaultCollisionOperations();

  mutable std::vector<std::vector<double> > colors_;

  distance_field::PropagationDistanceField* environment_distance_field_;
  distance_field::PropagationDistanceField* self_distance_field_;

  planning_environment::CollisionModelsInterface* collision_models_interface_;

  ros::NodeHandle root_handle_, priv_handle_;

  ros::Publisher vis_marker_publisher_;
  ros::Publisher vis_marker_array_publisher_;

  mutable boost::recursive_mutex group_queries_lock_;

  std::map<std::string, BodyDecomposition*> body_decomposition_map_;
  std::map<std::string, BodyDecompositionVector*> static_object_map_;
  std::map<std::string, BodyDecompositionVector*> attached_object_map_;

  std::map<std::string, std::map<std::string, bool> > enabled_self_collision_links_;
  std::map<std::string, std::map<std::string, bool> > intra_group_collision_links_;
  std::map<std::string, std::map<std::string, bool> > attached_object_collision_links_;
  std::map<std::string, bool> self_excludes_;

  //current entries to avoid map lookups during collision checks
  std::string current_group_name_;
  std::vector<std::string> current_link_names_;
  std::vector<std::string> current_attached_body_names_;
  std::vector<unsigned int> current_link_indices_;
  std::vector<unsigned int> current_attached_body_indices_;
  std::vector<BodyDecomposition*> current_link_body_decompositions_;
  std::vector<BodyDecompositionVector*> current_attached_body_decompositions_;
  std::vector<std::vector<bool> > current_intra_group_collision_links_;
  std::vector<bool> current_self_excludes_;

  //just for initializing input
  std::vector<GradientInfo> current_gradients_;
  
  //distance field configuration
  double size_x_, size_y_, size_z_;
  double origin_x_, origin_y_, origin_z_;
  double resolution_, tolerance_;

  double max_environment_distance_;
  double max_self_distance_;
  double undefined_distance_;

};

}
#endif
