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
#include <arm_navigation_msgs/RobotState.h>
#include <arm_navigation_msgs/Constraints.h>
#include <arm_navigation_msgs/OrderedCollisionOperations.h>

#include <planning_environment/util/kinematic_state_constraint_evaluator.h>
#include <arm_navigation_msgs/Shape.h>
#include <visualization_msgs/Marker.h>
#include <arm_navigation_msgs/LinkPadding.h>
#include <collision_space/environment.h>
#include <arm_navigation_msgs/AllowedCollisionMatrix.h>
#include <planning_environment/models/collision_models.h>

namespace planning_environment {

//returns true if the joint_state_map sets all the joints in the state, 
bool setRobotStateAndComputeTransforms(const arm_navigation_msgs::RobotState &robot_state,
                                       planning_models::KinematicState& state);

void convertKinematicStateToRobotState(const planning_models::KinematicState& kinematic_state,
                                       const ros::Time& timestamp,
                                       const std::string& header_frame,
                                       arm_navigation_msgs::RobotState &robot_state);

void applyOrderedCollisionOperationsToMatrix(const arm_navigation_msgs::OrderedCollisionOperations &ord,
                                             collision_space::EnvironmentModel::AllowedCollisionMatrix& acm);

void convertFromACMToACMMsg(const collision_space::EnvironmentModel::AllowedCollisionMatrix& acm,
                            arm_navigation_msgs::AllowedCollisionMatrix& matrix);

collision_space::EnvironmentModel::AllowedCollisionMatrix convertFromACMMsgToACM(const arm_navigation_msgs::AllowedCollisionMatrix& matrix);

bool applyOrderedCollisionOperationsListToACM(const arm_navigation_msgs::OrderedCollisionOperations& ordered_coll,
                                              const std::vector<std::string>& object_names,
                                              const std::vector<std::string>& att_names,
                                              const planning_models::KinematicModel* model,
                                              collision_space::EnvironmentModel::AllowedCollisionMatrix& matrix);

arm_navigation_msgs::AllowedCollisionMatrix 
applyOrderedCollisionOperationsToCollisionsModel(const CollisionModels* cm,
                                                 const arm_navigation_msgs::OrderedCollisionOperations& ordered_coll,
                                                 const std::vector<std::string>& object_names,
                                                 const std::vector<std::string>& att_names);

void getAllKinematicStateStampedTransforms(const planning_models::KinematicState& state,
                                           std::vector<geometry_msgs::TransformStamped>& trans_vector,
                                           const ros::Time& stamp); 
       
bool doesKinematicStateObeyConstraints(const planning_models::KinematicState& state,
                                       const arm_navigation_msgs::Constraints& constraints,
                                       bool verbose = false);

void setMarkerShapeFromShape(const arm_navigation_msgs::Shape &obj, visualization_msgs::Marker &mk);

void setMarkerShapeFromShape(const shapes::Shape *obj, visualization_msgs::Marker &mk, double padding = 0.0);

void convertFromLinkPaddingMapToLinkPaddingVector(const std::map<std::string, double>& link_padding_map,
                                                  std::vector<arm_navigation_msgs::LinkPadding>& link_padding_vector);

void convertAllowedContactSpecificationMsgToAllowedContactVector(const std::vector<arm_navigation_msgs::AllowedContactSpecification>& acmv,
                                                                 std::vector<collision_space::EnvironmentModel::AllowedContact>& acv);

void getCollisionMarkersFromContactInformation(const std::vector<arm_navigation_msgs::ContactInformation>& contacts,
                                               const std::string& world_frame_id,
                                               visualization_msgs::MarkerArray& arr,
                                               const std_msgs::ColorRGBA& color,
                                               const ros::Duration& lifetime);
}
#endif
