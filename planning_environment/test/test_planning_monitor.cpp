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
#include <planning_environment/models/collision_models_interface.h>
#include <actionlib/client/simple_action_client.h>

static const double VERY_SMALL = .0001;

class PlanningMonitorTest : public testing::Test {
public:

  void actionFeedbackCallback(const planning_environment_msgs::SetPlanningSceneFeedbackConstPtr& feedback) {
    ready_ = true;  
  }

  void actionDoneCallback(const actionlib::SimpleClientGoalState& state,
                          const planning_environment_msgs::SetPlanningSceneResultConstPtr& result)
  {
    EXPECT_TRUE(state == actionlib::SimpleClientGoalState::PREEMPTED);
    ROS_INFO("Got preempted");
  }

  void setPlanningSceneCallback(const planning_environment_msgs::PlanningScene& scene) {
    got_set_callback_ = true;
  }

  void revertPlanningSceneCallback() {
    got_revert_callback_ = true;
  }

protected:

  virtual void SetUp() {
    
    got_set_callback_ = false;
    got_revert_callback_ = false;
    ready_ = false;

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
                                                       planning_scene_diff,
                                                       operations,
                                                       goal_constraints,
                                                       path_constraints,
                                                       planning_scene,
                                                       transformed_goal_constraints,
                                                       transformed_path_constraints);
  }

protected:

  bool ready_;
  bool got_set_callback_;
  bool got_revert_callback_;

  std::string group_name_;
  ros::NodeHandle nh_;
  tf::TransformListener tf_;
  planning_environment::CollisionModels *collision_models_;
  planning_environment::PlanningMonitor *planning_monitor_;
  
  planning_environment_msgs::PlanningScene planning_scene_diff;
  planning_environment_msgs::PlanningScene planning_scene;

  motion_planning_msgs::OrderedCollisionOperations operations;
  
  motion_planning_msgs::Constraints goal_constraints;
  motion_planning_msgs::Constraints path_constraints;

  motion_planning_msgs::Constraints transformed_goal_constraints;
  motion_planning_msgs::Constraints transformed_path_constraints;
};

TEST_F(PlanningMonitorTest, ChangingObjects)
{
  planning_scene_diff.robot_state.multi_dof_joint_state.stamp = ros::Time::now();
  planning_scene_diff.robot_state.multi_dof_joint_state.joint_names.push_back("base_joint");
  planning_scene_diff.robot_state.multi_dof_joint_state.frame_ids.push_back("odom_combined");
  planning_scene_diff.robot_state.multi_dof_joint_state.child_frame_ids.push_back("base_footprint");
  planning_scene_diff.robot_state.multi_dof_joint_state.poses.resize(1);
  planning_scene_diff.robot_state.multi_dof_joint_state.poses[0].position.x = 4.0;
  planning_scene_diff.robot_state.multi_dof_joint_state.poses[0].orientation.w = 1.0;

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

  planning_scene_diff.collision_objects.push_back(obj1);
  planning_scene_diff.attached_collision_objects.push_back(att_obj);

  att_obj.object.header.frame_id = "odom_combined";
  planning_scene_diff.attached_collision_objects.push_back(att_obj);
  
  CallPlanningScene();

  ASSERT_EQ(planning_scene.collision_objects.size(), 1);
  //extra object with same id should have overwritten first
  ASSERT_EQ(planning_scene.attached_collision_objects.size(), 1);

  //last one should replace other
  EXPECT_EQ(planning_scene.attached_collision_objects[0].object.header.frame_id, "r_gripper_palm_link");
  //the pose when connected to odom_combined should be negative 
  //TOdo - figure this out
  //EXPECT_LE(all_attached_collision_objects[0].object.poses[0].position.x,0.0); 
  
  att_obj.link_name = "base_footprint";
  att_obj.object.header.frame_id = "base_footprint";
  att_obj.object.id = "obj3";
  att_obj.object.poses[0].position.x = 0.12;
  planning_scene_diff.attached_collision_objects.pop_back();
  planning_scene_diff.attached_collision_objects.push_back(att_obj);
  
  CallPlanningScene();

  EXPECT_EQ(planning_scene.collision_objects[0].header.frame_id, "odom_combined");
  EXPECT_LE(fabs(planning_scene.collision_objects[0].poses[0].position.x-4.5),VERY_SMALL);
  EXPECT_LE(fabs(planning_scene.collision_objects[0].poses[0].position.y-.5),VERY_SMALL);

  //now they should be different
  EXPECT_EQ(planning_scene.attached_collision_objects[0].object.header.frame_id, "r_gripper_palm_link");
  EXPECT_EQ(planning_scene.attached_collision_objects[1].object.header.frame_id, "base_footprint");
  //now the first one in should be slightly forward of the palm
  EXPECT_GE(planning_scene.attached_collision_objects[0].object.poses[0].position.x,0.0);
  EXPECT_LE(fabs(planning_scene.attached_collision_objects[1].object.poses[0].position.x-.12), VERY_SMALL); 
}

TEST_F(PlanningMonitorTest, ConvertPlanningConstraints)
{
  //todo - figure out a way to test transforms involving external frames
  planning_scene_diff.robot_state.multi_dof_joint_state.stamp = ros::Time::now();
  planning_scene_diff.robot_state.multi_dof_joint_state.joint_names.push_back("base_joint");
  planning_scene_diff.robot_state.multi_dof_joint_state.frame_ids.push_back("odom_combined");
  planning_scene_diff.robot_state.multi_dof_joint_state.child_frame_ids.push_back("base_footprint");
  planning_scene_diff.robot_state.multi_dof_joint_state.poses.resize(1);
  planning_scene_diff.robot_state.multi_dof_joint_state.poses[0].position.x = 4.0;
  planning_scene_diff.robot_state.multi_dof_joint_state.poses[0].orientation.w = 1.0;
  
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

  planning_scene_diff.robot_state.multi_dof_joint_state.poses[0].orientation.x = turn.x(); 
  planning_scene_diff.robot_state.multi_dof_joint_state.poses[0].orientation.y = turn.y(); 
  planning_scene_diff.robot_state.multi_dof_joint_state.poses[0].orientation.z = turn.z(); 
  planning_scene_diff.robot_state.multi_dof_joint_state.poses[0].orientation.w = turn.w(); 
  
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

  planning_scene_diff.collision_objects.push_back(obj1);

  CallPlanningScene();

  planning_environment::CollisionModels cm("robot_description");

  planning_models::KinematicState* state = cm.setPlanningScene(planning_scene);

  //without transforming state nowhere near the table
  ASSERT_TRUE(state != NULL);
  EXPECT_FALSE(cm.isKinematicStateInCollision(*state));

  cm.revertPlanningScene(state);

  planning_scene_diff.robot_state.multi_dof_joint_state.stamp = ros::Time::now();
  planning_scene_diff.robot_state.multi_dof_joint_state.joint_names.push_back("base_joint");
  planning_scene_diff.robot_state.multi_dof_joint_state.frame_ids.push_back("odom_combined");
  planning_scene_diff.robot_state.multi_dof_joint_state.child_frame_ids.push_back("base_footprint");
  planning_scene_diff.robot_state.multi_dof_joint_state.poses.resize(1);
  planning_scene_diff.robot_state.multi_dof_joint_state.poses[0].position.x = 4.0;
  planning_scene_diff.robot_state.multi_dof_joint_state.poses[0].orientation.w = 1.0;

  CallPlanningScene();

  state = cm.setPlanningScene(planning_scene);

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
  planning_scene_diff.robot_state.multi_dof_joint_state.stamp = ros::Time::now();
  planning_scene_diff.robot_state.multi_dof_joint_state.joint_names.push_back("base_joint");
  planning_scene_diff.robot_state.multi_dof_joint_state.frame_ids[0] = "";
  planning_scene_diff.robot_state.multi_dof_joint_state.child_frame_ids.push_back("base_footprint");
  planning_scene_diff.robot_state.multi_dof_joint_state.poses.resize(1);
  planning_scene_diff.robot_state.multi_dof_joint_state.poses[0].position.x = 4.0;
  planning_scene_diff.robot_state.multi_dof_joint_state.poses[0].orientation.w = 1.0;

  CallPlanningScene();

  //but now we shouldn't be in collision
  state = cm.setPlanningScene(planning_scene);

  ASSERT_TRUE(state != NULL);
  //expect collisions with table
  state->getKinematicStateValues(joint_state_values);
  EXPECT_EQ(joint_state_values["floating_trans_x"], 0.0);
  EXPECT_FALSE(cm.isKinematicStateInCollision(*state));

  cm.revertPlanningScene(state);

  planning_scene_diff.robot_state.multi_dof_joint_state.joint_names.clear();
  planning_scene_diff.robot_state.multi_dof_joint_state.poses.clear();
  planning_scene_diff.robot_state.multi_dof_joint_state.frame_ids.clear();
  planning_scene_diff.robot_state.multi_dof_joint_state.child_frame_ids.clear();
  
  planning_scene_diff.robot_state.joint_state.name.push_back("floating_trans_x");
  planning_scene_diff.robot_state.joint_state.position.push_back(3.3);

  CallPlanningScene();

  //back in collision
  state = cm.setPlanningScene(planning_scene);

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
  planning_scene_diff.robot_state.joint_state.name.push_back("floating_rot_z");
  planning_scene_diff.robot_state.joint_state.name.push_back("floating_rot_w");
  planning_scene_diff.robot_state.joint_state.position.push_back(.7071);
  planning_scene_diff.robot_state.joint_state.position.push_back(.7071);

  CallPlanningScene();

  state = cm.setPlanningScene(planning_scene);

  ASSERT_TRUE(state != NULL);
  EXPECT_FALSE(cm.isKinematicStateInCollision(*state));

  cm.revertPlanningScene(state);
}

TEST_F(PlanningMonitorTest, PlanningMonitorWithCollisionInterface)
{
  //this test is important because calling the service calls collision checking from a different thread

  planning_environment::CollisionModelsInterface cm("robot_description");

  cm.addSetPlanningSceneCallback(boost::bind(&PlanningMonitorTest::setPlanningSceneCallback, this, _1));
  cm.addRevertPlanningSceneCallback(boost::bind(&PlanningMonitorTest::revertPlanningSceneCallback, this));

  CallPlanningScene();

  actionlib::SimpleActionClient<planning_environment_msgs::SetPlanningSceneAction> ac("set_planning_scene", true);

  planning_environment_msgs::SetPlanningSceneGoal goal;
  
  goal.planning_scene = planning_scene;

  ros::AsyncSpinner async(2);
  async.start();

  ASSERT_TRUE(ac.waitForServer());

  ac.sendGoal(goal, boost::bind(&PlanningMonitorTest::actionDoneCallback, this, _1, _2), NULL, boost::bind(&PlanningMonitorTest::actionFeedbackCallback, this, _1));

  ros::Rate r(10.0);
  while(ros::ok()) {
    if(ready_ == true) {
      break;
    }
  }
  ASSERT_TRUE(ready_);
  EXPECT_TRUE(got_set_callback_);
  EXPECT_FALSE(got_revert_callback_);

  EXPECT_FALSE(cm.isKinematicStateInCollision(*(cm.getPlanningSceneState())));
  
  ac.cancelGoal();

  //waiting for a second to take effect
  ros::WallDuration(1.0).sleep();

  EXPECT_TRUE(got_revert_callback_);

  EXPECT_TRUE(cm.getPlanningSceneState() == NULL);
  
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_planning_monitor");
  return RUN_ALL_TESTS();
}
