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

#ifndef PLANNING_ENVIRONMENT_MODELS_COLLISION_MODELS_
#define PLANNING_ENVIRONMENT_MODELS_COLLISION_MODELS_

#include "planning_environment/models/robot_models.h"

#include <collision_space/environment.h>
#include <motion_planning_msgs/AllowedContactSpecification.h>
#include <motion_planning_msgs/OrderedCollisionOperations.h>
#include <motion_planning_msgs/LinkPadding.h>
#include <mapping_msgs/CollisionMap.h>
#include <mapping_msgs/CollisionObject.h>
#include <mapping_msgs/AttachedCollisionObject.h>
#include <geometric_shapes_msgs/Shape.h>
#include <planning_environment_msgs/AllowedCollisionMatrix.h>
#include <geometric_shapes/bodies.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <motion_planning_msgs/RobotState.h>
#include <motion_planning_msgs/Constraints.h>
#include <motion_planning_msgs/ArmNavigationErrorCodes.h>
#include <planning_environment_msgs/ContactInformation.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

static const std::string COLLISION_MAP_NAME="collision_map";

namespace planning_environment
{

/** \brief A class capable of loading a robot model from the parameter server */
    
class CollisionModels : public RobotModels
{
public:

  //
  // Constructors
  //
	
  CollisionModels(const std::string &description);

  virtual ~CollisionModels(void);
 
  /** \brief Reload the robot description and recreate the model */	
  virtual void reload(void);

  //
  // Manipulating bodies and objects
  //

  planning_models::KinematicState* 
  setPlanningScene(const motion_planning_msgs::RobotState& complete_robot_state,
                   const planning_environment_msgs::AllowedCollisionMatrix& allowed_collision_matrix,
                   const std::vector<motion_planning_msgs::AllowedContactSpecification>& transformed_allowed_contacts,
                   const std::vector<motion_planning_msgs::LinkPadding>& all_link_paddings,
                   const std::vector<mapping_msgs::CollisionObject>& all_collision_objects,
                   const std::vector<mapping_msgs::AttachedCollisionObject>& all_attached_collision_objects,
                   const mapping_msgs::CollisionMap& unmasked_collision_map);

  void revertPlanningScene(planning_models::KinematicState* state);
    
  void updateRobotModelPose(const planning_models::KinematicState& state);

  //this function will fail if the header is not in the world frame
  bool addStaticObject(const mapping_msgs::CollisionObject& obj);

  void addStaticObject(const std::string& name,
                       std::vector<shapes::Shape*>& shapes,
                       const std::vector<btTransform>& poses);

  void deleteStaticObject(const std::string& name);
  
  void deleteAllStaticObjects();

  //this function will fail if the header is not in the world frame
  void setCollisionMap(const mapping_msgs::CollisionMap& map,
                       bool mask_before_insertion=true);

  void setCollisionMap(std::vector<shapes::Shape*>& shapes,
                       const std::vector<btTransform>& poses,
                       bool mask_before_insertion=true);
  
  void remaskCollisionMap();

  void maskAndDeleteShapeVector(std::vector<shapes::Shape*>& shapes,
                                std::vector<btTransform>& poses);
  
  //this function will fail if the header is not in the world frame
  bool addAttachedObject(const mapping_msgs::AttachedCollisionObject& att);

  //fails if the link_name is not a valid link
  bool addAttachedObject(const std::string& object_name,
                         const std::string& link_name,
                         std::vector<shapes::Shape*>& shapes,
                         const std::vector<btTransform>& poses,
                         const std::vector<std::string>& touch_links);

  bool deleteAttachedObject(const std::string& object_id,
                            const std::string& link_name);

  void deleteAllAttachedObjects(const std::string& link_name="");

  bool convertStaticObjectToAttachedObject(const std::string& object_name,
                                           const std::string& link_name,
                                           const std::vector<std::string>& touch_links);
  
  bool convertAttachedObjectToStaticObject(const std::string& object_name,
                                           const std::string& link_name);

  void applyLinkPaddingToCollisionSpace(const std::vector<motion_planning_msgs::LinkPadding>& link_padding);

  void getCurrentLinkPadding(std::vector<motion_planning_msgs::LinkPadding>& link_padding);

  void revertCollisionSpacePaddingToDefault();

  void revertAllowedCollisionToDefault();

  bool applyOrderedCollisionOperationsToCollisionSpace(const motion_planning_msgs::OrderedCollisionOperations &ord,
                                                       bool print=false);
  
  bool computeAllowedContact(const motion_planning_msgs::AllowedContactSpecification& al,
                             collision_space::EnvironmentModel::AllowedContact& allowed_contact) const;

  void getCollisionSpaceCollisionMap(mapping_msgs::CollisionMap& cmap) const;

  
  void getCollisionSpaceAllowedCollisions(planning_environment_msgs::AllowedCollisionMatrix& matrix) const;

  void getCollisionSpaceCollisionObjects(std::vector<mapping_msgs::CollisionObject>& omap) const;

  void getCollisionSpaceAttachedCollisionObjects(std::vector<mapping_msgs::AttachedCollisionObject>& avec) const;
  
  bool isKinematicStateInCollision(const planning_models::KinematicState& state);

  bool isKinematicStateInSelfCollision(const planning_models::KinematicState& state);

  bool isKinematicStateInEnvironmentCollision(const planning_models::KinematicState& state);

  void getAllCollisionsForState(const planning_models::KinematicState& state,
                                std::vector<planning_environment_msgs::ContactInformation>& contacts,
                                unsigned int num_per_pair = 1);

  bool isTrajectoryValid(const trajectory_msgs::JointTrajectory &trajectory,
                         const motion_planning_msgs::RobotState& robot_state,
                         const motion_planning_msgs::Constraints& path_constraints,
                         const motion_planning_msgs::Constraints& goal_constraints, 
                         motion_planning_msgs::ArmNavigationErrorCodes& error_code,
                         std::vector<motion_planning_msgs::ArmNavigationErrorCodes>& trajectory_error_codes,
                         const bool evaluate_entire_trajectory);

  void getAllCollisionSpaceObjectMarkers(visualization_msgs::MarkerArray& arr,
                                         const std_msgs::ColorRGBA static_color,
                                         const std_msgs::ColorRGBA attached_color,
                                         const ros::Duration& lifetime) const;

  void getAttachedCollisionObjectMarkers(visualization_msgs::MarkerArray& arr,
                                         const std_msgs::ColorRGBA color,
                                         const ros::Duration& lifetime) const;;
  
  void getStaticCollisionObjectMarkers(visualization_msgs::MarkerArray& arr,
                                       const std_msgs::ColorRGBA color,
                                       const ros::Duration& lifetime) const;
  
  //can't be const because it poses in state
  void getAllCollisionPointMarkers(const planning_models::KinematicState& state,
                                   visualization_msgs::MarkerArray& arr,
                                   const std_msgs::ColorRGBA color,
                                   const ros::Duration& lifetime);

  void getRobotTrimeshMarkersGivenState(const planning_models::KinematicState& state,
                                        visualization_msgs::MarkerArray& arr,
                                        bool use_default_padding,
                                        const ros::Duration& lifetime) const;
  
  /** \brief Return the instance of the constructed ODE collision model */  
  const collision_space::EnvironmentModel* getCollisionSpace() const {
    return ode_collision_model_;
  }

  /** \brief Get the scaling to be used for the robot parts when inserted in the collision space */
  double getDefaultScale(void)
  {
    return default_scale_;
  }
	
  /** \brief Get the padding to be used for the robot parts when inserted in the collision space */
  double getDefaultPadding(void)
  {
    return default_padd_;
  }

  void getDefaultOrderedCollisionOperations(std::vector<motion_planning_msgs::CollisionOperation> &self_collision)
  {
    self_collision = default_collision_operations_;
  }
      
  const std::map<std::string,double>& getDefaultLinkPaddingMap() const{
    return default_link_padding_map_;
  }
  
protected:

  std::vector<shapes::Shape*> collision_map_shapes_;
  std::vector<btTransform> collision_map_poses_;

  std::map<std::string, bodies::BodyVector*> static_object_map_;

  std::map<std::string, std::map<std::string, bodies::BodyVector*> > link_attached_objects_;
	
  void loadCollision();
  void setupModel(collision_space::EnvironmentModel* model);
	
  collision_space::EnvironmentModel* ode_collision_model_;

  double default_scale_;
  double default_padd_;
  std::vector<double> bounding_planes_;

  std::vector<motion_planning_msgs::CollisionOperation> default_collision_operations_;
  std::map<std::string, double> default_link_padding_map_;
};
    
	
}

#endif

