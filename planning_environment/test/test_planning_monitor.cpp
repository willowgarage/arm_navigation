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

/** E. Gil Jones */

#include <ros/time.h>
#include <gtest/gtest.h>
#include <iostream>
#include <sstream>
#include <ros/package.h>

#include <planning_environment/monitors/planning_monitor.h>

static const double VERY_SMALL = .0001;

class PlanningMonitorTest : public testing::Test {
protected:

  virtual void SetUp() {

    group_name_ = "right_arm";

    std::string robot_description_name = nh_.resolveName("robot_description", true);

    ros::WallRate h(10.0);
    
    while(nh_.ok() && !nh_.hasParam(robot_description_name)) {
      ros::spinOnce();
      h.sleep();
    }
    
    collision_models_ = new planning_environment::CollisionModels(robot_description_name);
    planning_monitor_ = new planning_environment::PlanningMonitor(collision_models_, &tf_);
    planning_monitor_->setUseCollisionMap(false);
    planning_monitor_->waitForState();
    planning_monitor_->startEnvironmentMonitor();
  }

  virtual void TearDown()
  {
    delete collision_models_;
    delete planning_monitor_;
  }

  bool CallPlanningScene() {
    return planning_monitor_->getCompletePlanningScene(group_name_,
                                                       state_diff,
                                                       goal_constraints,
                                                       path_constraints,
                                                       allowed_contacts_diffs,
                                                       ordered_collision_operations_diff,
                                                       link_padding_diff,
                                                       collision_object_diffs,
                                                       attached_collision_object_diffs,
                                                       complete_robot_state,
                                                       transformed_goal_constraints,
                                                       transformed_path_constraints,
                                                       allowed_collision_matrix,
                                                       transformed_allowed_contacts,
                                                       all_link_paddings,
                                                       all_collision_objects,
                                                       all_attached_collision_objects,
                                                       unmasked_collision_map);
  }

protected:

  std::string group_name_;
  ros::NodeHandle nh_;
  tf::TransformListener tf_;
  planning_environment::CollisionModels *collision_models_;
  planning_environment::PlanningMonitor *planning_monitor_;
  
  motion_planning_msgs::RobotState state_diff;
  motion_planning_msgs::Constraints goal_constraints;
  motion_planning_msgs::Constraints path_constraints;
  std::vector<motion_planning_msgs::AllowedContactSpecification> allowed_contacts_diffs;
  motion_planning_msgs::OrderedCollisionOperations ordered_collision_operations_diff;    
  std::vector<motion_planning_msgs::LinkPadding> link_padding_diff;
  std::vector<mapping_msgs::CollisionObject> collision_object_diffs;
  std::vector<mapping_msgs::AttachedCollisionObject> attached_collision_object_diffs;

  motion_planning_msgs::RobotState complete_robot_state;
  motion_planning_msgs::Constraints transformed_goal_constraints;
  motion_planning_msgs::Constraints transformed_path_constraints;
  planning_environment_msgs::AllowedCollisionMatrix allowed_collision_matrix;
  std::vector<motion_planning_msgs::AllowedContactSpecification> transformed_allowed_contacts;
  std::vector<motion_planning_msgs::LinkPadding> all_link_paddings;
  std::vector<mapping_msgs::CollisionObject> all_collision_objects;
  std::vector<mapping_msgs::AttachedCollisionObject> all_attached_collision_objects;
  mapping_msgs::CollisionMap unmasked_collision_map;
  
};

TEST_F(PlanningMonitorTest, ChangingObjects)
{
  state_diff.multi_dof_joint_state.stamp = ros::Time::now();
  state_diff.multi_dof_joint_state.joint_names.push_back("base_joint");
  state_diff.multi_dof_joint_state.frame_ids.push_back("odom_combined");
  state_diff.multi_dof_joint_state.child_frame_ids.push_back("base_footprint");
  state_diff.multi_dof_joint_state.poses.resize(1);
  state_diff.multi_dof_joint_state.poses[0].position.x = 4.0;
  state_diff.multi_dof_joint_state.poses[0].orientation.w = 1.0;

  mapping_msgs::CollisionObject obj1;
  obj1.header.stamp = ros::Time::now();
  obj1.header.frame_id = "base_footprint";
  obj1.id = "obj1";
  obj1.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
  obj1.shapes.resize(1);
  obj1.shapes[0].type = geometric_shapes_msgs::Shape::BOX;
  obj1.shapes[0].dimensions.resize(3);
  obj1.shapes[0].dimensions[0] = .1;
  obj1.shapes[0].dimensions[1] = .1;
  obj1.shapes[0].dimensions[2] = .75;
  obj1.poses.resize(1);
  obj1.poses[0].position.x = .5;
  obj1.poses[0].position.y = .5;
  obj1.poses[0].position.z = 0;
  obj1.poses[0].orientation.w = 1.0;

  mapping_msgs::AttachedCollisionObject att_obj;
  att_obj.object = obj1;
  att_obj.object.header.stamp = ros::Time::now();
  att_obj.object.header.frame_id = "r_gripper_r_finger_tip_link";
  att_obj.link_name = "r_gripper_palm_link";
  att_obj.touch_links.push_back("r_gripper_palm_link");
  att_obj.touch_links.push_back("r_gripper_r_finger_link");
  att_obj.touch_links.push_back("r_gripper_l_finger_link");
  att_obj.touch_links.push_back("r_gripper_r_finger_tip_link");
  att_obj.touch_links.push_back("r_gripper_l_finger_tip_link");
  att_obj.touch_links.push_back("r_wrist_roll_link");
  att_obj.touch_links.push_back("r_wrist_flex_link");
  att_obj.touch_links.push_back("r_forearm_link");
  att_obj.touch_links.push_back("r_gripper_motor_accelerometer_link");
  att_obj.object.id = "obj2";
  att_obj.object.shapes[0].type = geometric_shapes_msgs::Shape::CYLINDER;
  att_obj.object.shapes[0].dimensions.resize(2);
  att_obj.object.shapes[0].dimensions[0] = .025;
  att_obj.object.shapes[0].dimensions[1] = .5;
  att_obj.object.poses.resize(1);
  att_obj.object.poses[0].position.x = 0.0;
  att_obj.object.poses[0].position.y = 0.0;
  att_obj.object.poses[0].position.z = 0.0;
  att_obj.object.poses[0].orientation.w = 1.0;

  collision_object_diffs.push_back(obj1);
  attached_collision_object_diffs.push_back(att_obj);

  att_obj.object.header.frame_id = "odom_combined";
  attached_collision_object_diffs.push_back(att_obj);
  
  CallPlanningScene();

  ASSERT_EQ(all_collision_objects.size(), 1);
  //extra object with same id should have overwritten first
  ASSERT_EQ(all_attached_collision_objects.size(), 1);

  //last one should replace other
  EXPECT_EQ(all_attached_collision_objects[0].object.header.frame_id, "r_gripper_palm_link");
  //the pose when connected to odom_combined should be negative 
  //TOdo - figure this out
  //EXPECT_LE(all_attached_collision_objects[0].object.poses[0].position.x,0.0); 
  
  att_obj.link_name = "base_footprint";
  att_obj.object.header.frame_id = "base_footprint";
  att_obj.object.id = "obj3";
  att_obj.object.poses[0].position.x = 0.12;
  attached_collision_object_diffs.pop_back();
  attached_collision_object_diffs.push_back(att_obj);
  
  CallPlanningScene();

  EXPECT_EQ(all_collision_objects[0].header.frame_id, "odom_combined");
  EXPECT_LE(fabs(all_collision_objects[0].poses[0].position.x-4.5),VERY_SMALL);
  EXPECT_LE(fabs(all_collision_objects[0].poses[0].position.y-.5),VERY_SMALL);

  //now they should be different
  EXPECT_EQ(all_attached_collision_objects[0].object.header.frame_id, "r_gripper_palm_link");
  EXPECT_EQ(all_attached_collision_objects[1].object.header.frame_id, "base_footprint");
  //now the first one in should be slightly forward of the palm
  EXPECT_GE(all_attached_collision_objects[0].object.poses[0].position.x,0.0);
  EXPECT_LE(fabs(all_attached_collision_objects[1].object.poses[0].position.x-.12), VERY_SMALL); 
}

TEST_F(PlanningMonitorTest, ConvertPlanningConstraints)
{
  //todo - figure out a way to test transforms involving external frames
  state_diff.multi_dof_joint_state.stamp = ros::Time::now();
  state_diff.multi_dof_joint_state.joint_names.push_back("base_joint");
  state_diff.multi_dof_joint_state.frame_ids.push_back("odom_combined");
  state_diff.multi_dof_joint_state.child_frame_ids.push_back("base_footprint");
  state_diff.multi_dof_joint_state.poses.resize(1);
  state_diff.multi_dof_joint_state.poses[0].position.x = 4.0;
  state_diff.multi_dof_joint_state.poses[0].orientation.w = 1.0;
  
  motion_planning_msgs::PositionConstraint pos;
  pos.header.frame_id = "base_footprint";
  pos.header.stamp = ros::Time::now();
  pos.link_name = "r_wrist_roll_link";
  pos.target_point_offset.x = .5;
  pos.constraint_region_orientation.w = 1.0;

  goal_constraints.position_constraints.push_back(pos);

  CallPlanningScene();

  EXPECT_TRUE(transformed_goal_constraints.position_constraints[0].header.frame_id == "odom_combined");
  EXPECT_LE(fabs(transformed_goal_constraints.position_constraints[0].target_point_offset.x-4.5), VERY_SMALL) ;
  
  //checking that if we turn around then we should subtract .5 in x

  btQuaternion turn(btVector3(0,0,1), M_PI);

  state_diff.multi_dof_joint_state.poses[0].orientation.x = turn.x(); 
  state_diff.multi_dof_joint_state.poses[0].orientation.y = turn.y(); 
  state_diff.multi_dof_joint_state.poses[0].orientation.z = turn.z(); 
  state_diff.multi_dof_joint_state.poses[0].orientation.w = turn.w(); 
  
  CallPlanningScene();

  EXPECT_TRUE(transformed_goal_constraints.position_constraints[0].header.frame_id == "odom_combined");
  EXPECT_LE(fabs(transformed_goal_constraints.position_constraints[0].target_point_offset.x-3.5), VERY_SMALL) ;

}

TEST_F(PlanningMonitorTest, ChangingRobotState)
{
  mapping_msgs::CollisionObject obj1;
  obj1.header.stamp = ros::Time::now();
  obj1.header.frame_id = "odom_combined";
  obj1.id = "table";
  obj1.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
  obj1.shapes.resize(1);
  obj1.shapes[0].type = geometric_shapes_msgs::Shape::BOX;
  obj1.shapes[0].dimensions.resize(3);
  obj1.shapes[0].dimensions[0] = 1.0;
  obj1.shapes[0].dimensions[1] = 1.0;
  obj1.shapes[0].dimensions[2] = .2;
  obj1.poses.resize(1);
  obj1.poses[0].position.x = 4.25;
  obj1.poses[0].position.y = 0.0;
  obj1.poses[0].position.z = .8;
  obj1.poses[0].orientation.w = 1.0;

  collision_object_diffs.push_back(obj1);

  CallPlanningScene();

  planning_environment::CollisionModels cm("robot_description");

  planning_models::KinematicState* state = cm.setPlanningScene(complete_robot_state,
                                                               allowed_collision_matrix,
                                                               transformed_allowed_contacts,
                                                               all_link_paddings,
                                                               all_collision_objects,
                                                               all_attached_collision_objects,
                                                               unmasked_collision_map);
  //without transforming state nowhere near the table
  ASSERT_TRUE(state != NULL);
  EXPECT_FALSE(cm.isKinematicStateInCollision(*state));

  cm.revertPlanningScene(state);

  state_diff.multi_dof_joint_state.stamp = ros::Time::now();
  state_diff.multi_dof_joint_state.joint_names.push_back("base_joint");
  state_diff.multi_dof_joint_state.frame_ids.push_back("odom_combined");
  state_diff.multi_dof_joint_state.child_frame_ids.push_back("base_footprint");
  state_diff.multi_dof_joint_state.poses.resize(1);
  state_diff.multi_dof_joint_state.poses[0].position.x = 4.0;
  state_diff.multi_dof_joint_state.poses[0].orientation.w = 1.0;

  CallPlanningScene();

  state = cm.setPlanningScene(complete_robot_state,
                              allowed_collision_matrix,
                              transformed_allowed_contacts,
                              all_link_paddings,
                              all_collision_objects,
                              all_attached_collision_objects,
                              unmasked_collision_map);

  //expect collisions with table
  ASSERT_TRUE(state != NULL);
  std::map<std::string, double> joint_state_values;
  state->getKinematicStateValues(joint_state_values);
  EXPECT_EQ(joint_state_values["floating_trans_x"], 4.0);
  EXPECT_TRUE(cm.isKinematicStateInCollision(*state));
  EXPECT_FALSE(cm.isKinematicStateInSelfCollision(*state));
  EXPECT_TRUE(cm.isKinematicStateInEnvironmentCollision(*state));

  cm.revertPlanningScene(state);

  //bad multi-dof
  state_diff.multi_dof_joint_state.stamp = ros::Time::now();
  state_diff.multi_dof_joint_state.joint_names.push_back("base_joint");
  state_diff.multi_dof_joint_state.frame_ids[0] = "";
  state_diff.multi_dof_joint_state.child_frame_ids.push_back("base_footprint");
  state_diff.multi_dof_joint_state.poses.resize(1);
  state_diff.multi_dof_joint_state.poses[0].position.x = 4.0;
  state_diff.multi_dof_joint_state.poses[0].orientation.w = 1.0;

  CallPlanningScene();

  //but now we shouldn't be in collision
  state = cm.setPlanningScene(complete_robot_state,
                              allowed_collision_matrix,
                              transformed_allowed_contacts,
                              all_link_paddings,
                              all_collision_objects,
                              all_attached_collision_objects,
                              unmasked_collision_map);
  ASSERT_TRUE(state != NULL);
  //expect collisions with table
  state->getKinematicStateValues(joint_state_values);
  EXPECT_EQ(joint_state_values["floating_trans_x"], 0.0);
  EXPECT_FALSE(cm.isKinematicStateInCollision(*state));

  cm.revertPlanningScene(state);

  state_diff.multi_dof_joint_state.joint_names.clear();
  state_diff.multi_dof_joint_state.poses.clear();
  state_diff.multi_dof_joint_state.frame_ids.clear();
  state_diff.multi_dof_joint_state.child_frame_ids.clear();
  
  state_diff.joint_state.name.push_back("floating_trans_x");
  state_diff.joint_state.position.push_back(3.3);

  CallPlanningScene();

  //back in collision
  state = cm.setPlanningScene(complete_robot_state,
                              allowed_collision_matrix,
                              transformed_allowed_contacts,
                              all_link_paddings,
                              all_collision_objects,
                              all_attached_collision_objects,
                              unmasked_collision_map);
  ASSERT_TRUE(state != NULL);
  state->getKinematicStateValues(joint_state_values);
  EXPECT_EQ(joint_state_values["floating_trans_x"], 3.3);
  EXPECT_TRUE(cm.isKinematicStateInCollision(*state));
  EXPECT_FALSE(cm.isKinematicStateInSelfCollision(*state));
  EXPECT_TRUE(cm.isKinematicStateInEnvironmentCollision(*state));

  // cm.writePlanningSceneBag(ros::package::getPath("planning_environment")+"/test.bag",
  //                          complete_robot_state,
  //                          allowed_collision_matrix,
  //                          transformed_allowed_contacts,
  //                          all_link_paddings,
  //                          all_collision_objects,
  //                          all_attached_collision_objects,
  //                          unmasked_collision_map);

  cm.revertPlanningScene(state);

  //now we turn out of collision
  state_diff.joint_state.name.push_back("floating_rot_z");
  state_diff.joint_state.name.push_back("floating_rot_w");
  state_diff.joint_state.position.push_back(.7071);
  state_diff.joint_state.position.push_back(.7071);

  CallPlanningScene();

  state = cm.setPlanningScene(complete_robot_state,
                              allowed_collision_matrix,
                              transformed_allowed_contacts,
                              all_link_paddings,
                              all_collision_objects,
                              all_attached_collision_objects,
                              unmasked_collision_map);
  ASSERT_TRUE(state != NULL);
  EXPECT_FALSE(cm.isKinematicStateInCollision(*state));

  cm.revertPlanningScene(state);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_planning_monitor");
  return RUN_ALL_TESTS();
}
