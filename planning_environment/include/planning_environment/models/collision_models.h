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
#include <tf/tf.h>
#include <collision_space/environment.h>
#include <planning_environment_msgs/PlanningScene.h>
#include <geometric_shapes_msgs/Shape.h>
#include <geometric_shapes/bodies.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <motion_planning_msgs/Constraints.h>
#include <motion_planning_msgs/ArmNavigationErrorCodes.h>
#include <motion_planning_msgs/MotionPlanRequest.h>
#include <planning_environment_msgs/ContactInformation.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <motion_planning_msgs/OrderedCollisionOperations.h>

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

  CollisionModels(boost::shared_ptr<urdf::Model> urdf,
                  planning_models::KinematicModel* kmodel,
                  collision_space::EnvironmentModel* ode_collision_model_);

  virtual ~CollisionModels(void);

  //
  // Planning scene functions
  //
  planning_models::KinematicState* setPlanningScene(const planning_environment_msgs::PlanningScene& planning_scene);

  void revertPlanningScene(planning_models::KinematicState* state);

  // 
  // Planning scene and state based transform functions
  //
  bool convertAttachedCollisionObjectToNewWorldFrame(const planning_models::KinematicState& state,
                                                     mapping_msgs::AttachedCollisionObject& att_obj) const;
  
  bool convertCollisionObjectToNewWorldFrame(const planning_models::KinematicState& state,
                                             mapping_msgs::CollisionObject& obj) const;
  
  bool convertConstraintsGivenNewWorldTransform(const planning_models::KinematicState& state,
                                                motion_planning_msgs::Constraints& constraints,
                                                const std::string& opt_frame="") const;
  
  bool convertPoseGivenWorldTransform(const planning_models::KinematicState& state,
                                      const std::string& des_frame_id,
                                      const std_msgs::Header& header,
                                      const geometry_msgs::Pose& pose,
                                      geometry_msgs::PoseStamped& ret_pose) const;

  bool convertPointGivenWorldTransform(const planning_models::KinematicState& state,
                                       const std::string& des_frame_id,
                                       const std_msgs::Header& header,
                                       const geometry_msgs::Point& point,
                                       geometry_msgs::PointStamped& ret_point) const;
  
  bool convertQuaternionGivenWorldTransform(const planning_models::KinematicState& state,
                                            const std::string& des_frame_id,
                                            const std_msgs::Header& header,
                                            const geometry_msgs::Quaternion& quat,
                                            geometry_msgs::QuaternionStamped& ret_quat) const;

  //
  // Functions for updating the position of attached objects
  //

  bool updateAttachedBodyPosesForLink(const planning_models::KinematicState& state,
                                      const std::string& link_name);

  bool updateAttachedBodyPoses(const planning_models::KinematicState& state);

  //this function will fail if the header is not in the world frame
  bool addStaticObject(const mapping_msgs::CollisionObject& obj);

  void addStaticObject(const std::string& name,
                       std::vector<shapes::Shape*>& shapes,
                       const std::vector<btTransform>& poses,
                       double padding);

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
                         const std::vector<std::string>& touch_links,
                         double padding);

  bool deleteAttachedObject(const std::string& object_id,
                            const std::string& link_name);

  void deleteAllAttachedObjects(const std::string& link_name="");

  bool convertStaticObjectToAttachedObject(const std::string& object_name,
                                           const std::string& link_name,
                                           const btTransform& link_pose,
                                           const std::vector<std::string>& touch_links);
  
  bool convertAttachedObjectToStaticObject(const std::string& object_name,
                                           const std::string& link_name,
                                           const btTransform& link_pose);

  const std::map<std::string, std::map<std::string, bodies::BodyVector*> >& getLinkAttachedObjects() const
  {
    return link_attached_objects_;
  }

  //
  // Handling collision space functions
  //

  void applyLinkPaddingToCollisionSpace(const std::vector<motion_planning_msgs::LinkPadding>& link_padding);

  void getCurrentLinkPadding(std::vector<motion_planning_msgs::LinkPadding>& link_padding);

  void revertCollisionSpacePaddingToDefault();

  void revertAllowedCollisionToDefault();

  bool applyOrderedCollisionOperationsToCollisionSpace(const motion_planning_msgs::OrderedCollisionOperations &ord,
                                                       bool print=false);
  bool disableCollisionsForNonUpdatedLinks(const std::string& group_name);

  bool setAlteredAllowedCollisionMatrix(const collision_space::EnvironmentModel::AllowedCollisionMatrix& acm);

  bool computeAllowedContact(const motion_planning_msgs::AllowedContactSpecification& al,
                             collision_space::EnvironmentModel::AllowedContact& allowed_contact) const;

  //
  // Collision space accessors
  //

  const collision_space::EnvironmentModel::AllowedCollisionMatrix& getCurrentAllowedCollisionMatrix() const;

  const collision_space::EnvironmentModel::AllowedCollisionMatrix& getDefaultAllowedCollisionMatrix() const;

  void getCollisionSpaceCollisionMap(mapping_msgs::CollisionMap& cmap) const;

  void getLastCollisionMap(mapping_msgs::CollisionMap& cmap) const;
  
  void getCollisionSpaceAllowedCollisions(planning_environment_msgs::AllowedCollisionMatrix& matrix) const;

  void getCollisionSpaceCollisionObjects(std::vector<mapping_msgs::CollisionObject>& omap) const;

  void getCollisionSpaceAttachedCollisionObjects(std::vector<mapping_msgs::AttachedCollisionObject>& avec) const;

  //
  // Functions for checking collisions and validity
  //
  
  bool isKinematicStateInCollision(const planning_models::KinematicState& state);

  bool isKinematicStateInSelfCollision(const planning_models::KinematicState& state);

  bool isKinematicStateInEnvironmentCollision(const planning_models::KinematicState& state);

  void getPlanningSceneGivenState(const planning_models::KinematicState& state,
                                  planning_environment_msgs::PlanningScene& scene);

  void getAllCollisionsForState(const planning_models::KinematicState& state,
                                std::vector<planning_environment_msgs::ContactInformation>& contacts,
                                unsigned int num_per_pair = 1);

  bool isKinematicStateValid(const planning_models::KinematicState& state,
                             const std::vector<std::string>& names,
                             motion_planning_msgs::ArmNavigationErrorCodes& error_code,
                             const motion_planning_msgs::Constraints goal_constraints,
                             const motion_planning_msgs::Constraints path_constraints,
			     bool verbose = false);

  bool isJointTrajectoryValid(const planning_environment_msgs::PlanningScene& planning_scene,
                              const trajectory_msgs::JointTrajectory &trajectory,
                              const motion_planning_msgs::Constraints& goal_constraints,
                              const motion_planning_msgs::Constraints& path_constraints,
                              motion_planning_msgs::ArmNavigationErrorCodes& error_code,
                              std::vector<motion_planning_msgs::ArmNavigationErrorCodes>& trajectory_error_codes,
                              const bool evaluate_entire_trajectory);

  bool isJointTrajectoryValid(planning_models::KinematicState& state,
                              const trajectory_msgs::JointTrajectory &trajectory,
                              const motion_planning_msgs::Constraints& goal_constraints,
                              const motion_planning_msgs::Constraints& path_constraints,
                              motion_planning_msgs::ArmNavigationErrorCodes& error_code,
                              std::vector<motion_planning_msgs::ArmNavigationErrorCodes>& trajectory_error_codes,
                              const bool evaluate_entire_trajectory);  

  // bool isRobotTrajectoryValid(const planning_environment_msgs::PlanningScene& planning_scene,
  //                             const motion_planning_msgs::RobotTrajectory &trajectory,
  //                             const motion_planning_msgs::Constraints& goal_constraints,
  //                             const motion_planning_msgs::Constraints& path_constraints,
  //                             motion_planning_msgs::ArmNavigationErrorCodes& error_code,
  //                             std::vector<motion_planning_msgs::ArmNavigationErrorCodes>& trajectory_error_codes,
  //                             const bool evaluate_entire_trajectory);

  // bool isRobotTrajectoryValid(planning_models::KinematicState& state,
  //                             const motion_planning_msgs::RobotTrajectory &trajectory,
  //                             const motion_planning_msgs::Constraints& goal_constraints,
  //                             const motion_planning_msgs::Constraints& path_constraints,
  //                             motion_planning_msgs::ArmNavigationErrorCodes& error_code,
  //                             std::vector<motion_planning_msgs::ArmNavigationErrorCodes>& trajectory_error_codes,
  //                             const bool evaluate_entire_trajectory);  


  double getTotalTrajectoryJointLength(planning_models::KinematicState& state,
                                       const trajectory_msgs::JointTrajectory &trajectory) const;                         

  //
  // Visualization functions
  //

  void getCollisionMapAsMarkers(visualization_msgs::MarkerArray& arr,
                                const std_msgs::ColorRGBA& color,
                                const ros::Duration& lifetime);

  void getAllCollisionSpaceObjectMarkers(const planning_models::KinematicState& state,
                                         visualization_msgs::MarkerArray& arr,
                                         const std::string& ns, 
                                         const std_msgs::ColorRGBA& static_color,
                                         const std_msgs::ColorRGBA& attached_color,
                                         const ros::Duration& lifetime);

  void getAttachedCollisionObjectMarkers(const planning_models::KinematicState& state,
                                         visualization_msgs::MarkerArray& arr,
                                         const std::string& ns, 
                                         const std_msgs::ColorRGBA& color,
                                         const ros::Duration& lifetime);
  
  void getStaticCollisionObjectMarkers(visualization_msgs::MarkerArray& arr,
                                       const std::string& ns, 
                                       const std_msgs::ColorRGBA& color,
                                       const ros::Duration& lifetime) const;
  
  //can't be const because it poses in state
  void getAllCollisionPointMarkers(const planning_models::KinematicState& state,
                                   visualization_msgs::MarkerArray& arr,
                                   const std_msgs::ColorRGBA& color,
                                   const ros::Duration& lifetime);

  void getRobotTrimeshMarkersGivenState(const planning_models::KinematicState& state,
                                        visualization_msgs::MarkerArray& arr,
                                        bool use_default_padding,
                                        const ros::Duration& lifetime) const;
  void getRobotMarkersGivenState(const planning_models::KinematicState& state,
                                             visualization_msgs::MarkerArray& arr,
                                             const std_msgs::ColorRGBA& color,
                                             const std::string& name, 
                                             const ros::Duration& lifetime,
                                             const std::vector<std::string>* names = NULL) const;

  void getGroupAndUpdatedJointMarkersGivenState(const planning_models::KinematicState& state,
                                                visualization_msgs::MarkerArray& arr,
                                                const std::string& group_name, 
                                                const std_msgs::ColorRGBA& group_color,
                                                const std_msgs::ColorRGBA& updated_color,
                                                const ros::Duration& lifetime) const;
  ///
  /// Functions for bag manipulation
  ///

  void writePlanningSceneBag(const std::string& filename,
                             const planning_environment_msgs::PlanningScene& planning_scene) const;
  
  bool readPlanningSceneBag(const std::string& filename,
                            planning_environment_msgs::PlanningScene& planning_scene) const;

  bool appendMotionPlanRequestToPlanningSceneBag(const std::string& filename,
                                                 const std::string& topic_name,
                                                 const motion_planning_msgs::MotionPlanRequest& req);

  bool loadMotionPlanRequestsInPlanningSceneBag(const std::string& filename,
                                                const std::string& topic_name,
                                                std::vector<motion_planning_msgs::MotionPlanRequest>& motion_plan_reqs);

  bool loadJointTrajectoriesInPlanningSceneBag(const std::string& filename,
                                               const std::string& topic_name,
                                               std::vector<trajectory_msgs::JointTrajectory>& traj_vec);

  
  bool appendJointTrajectoryToPlanningSceneBag(const std::string& filename,
                                               const std::string& topic_name,
                                               const trajectory_msgs::JointTrajectory& traj);

  ///
  /// Accessors
  ///

  /** \brief Return the instance of the constructed ODE collision model */  
  const collision_space::EnvironmentModel* getCollisionSpace() const {
    return ode_collision_model_;
  }

  /** \brief Get the scaling to be used for the robot parts when inserted in the collision space */
  double getDefaultScale(void) const
  {
    return default_scale_;
  }
	
  /** \brief Get the padding to be used for the robot parts when inserted in the collision space */
  double getDefaultPadding(void) const
  {
    return default_padd_;
  }

  double getDefaultObjectPadding(void) const
  {
    return object_padd_;
  }

  void getDefaultOrderedCollisionOperations(std::vector<motion_planning_msgs::CollisionOperation> &self_collision) const
  {
    self_collision = default_collision_operations_;
  }
      
  const std::map<std::string,double>& getDefaultLinkPaddingMap() const {
    return ode_collision_model_->getDefaultLinkPaddingMap();
  }
  
  bool isPlanningSceneSet() const {
    return planning_scene_set_;
  }

  const std::map<std::string, geometry_msgs::TransformStamped>& getSceneTransformMap() const {
    return scene_transform_map_;
  }

  const std::vector<shapes::Shape*>& getCollisionMapShapes() const {
    return collision_map_shapes_;
  }

  const std::vector<btTransform>& getCollisionMapPoses() const {
    return collision_map_poses_;
  }

  void bodiesLock() const {
    bodies_lock_.lock();
  }

  void bodiesUnlock() const {
    bodies_lock_.unlock();
  }

protected:

  mutable boost::recursive_mutex bodies_lock_;

  std::vector<shapes::Shape*> collision_map_shapes_;
  std::vector<btTransform> collision_map_poses_;

  std::map<std::string, bodies::BodyVector*> static_object_map_;

  std::map<std::string, std::map<std::string, bodies::BodyVector*> > link_attached_objects_;
	
  void loadCollisionFromParamServer();
  void setupModelFromParamServer(collision_space::EnvironmentModel* model);
	
  collision_space::EnvironmentModel* ode_collision_model_;

  bool planning_scene_set_;

  double default_scale_;
  double default_padd_;
  double object_padd_;
  double attached_padd_;
  std::vector<double> bounding_planes_;

  std::vector<motion_planning_msgs::CollisionOperation> default_collision_operations_;

  std::map<std::string, geometry_msgs::TransformStamped> scene_transform_map_;

};
    
	
}

#endif

