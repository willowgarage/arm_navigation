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

/** \author E. Gil Jones */

#include <planning_environment/models/collision_models.h>
#include <planning_environment/models/collision_models_interface.h>
#include <planning_models/kinematic_state.h>
#include <ros/time.h>
#include <gtest/gtest.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <ros/package.h>
#include <planning_environment/models/model_utils.h>

static const std::string rel_path = "/test_urdf/robot.xml";
static const double VERY_SMALL = .0001;

class TestCollisionModels : public testing::Test 
{
protected:
  
  virtual void SetUp() {

    full_path_ = ros::package::getPath("planning_models")+rel_path;
    
    std::string com = "rosparam set robot_description -t "+full_path_;

    int ok = system(com.c_str());
    
    if(ok != 0) {
      ROS_WARN_STREAM("Setting parameter system call not ok");
    }

    static_object_1_.header.stamp = ros::Time::now();
    static_object_1_.header.frame_id = "odom_combined";
    static_object_1_.id = "object_1";
    static_object_1_.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
    static_object_1_.shapes.resize(1);
    static_object_1_.shapes[0].type = arm_navigation_msgs::Shape::CYLINDER;
    static_object_1_.shapes[0].dimensions.resize(2);
    static_object_1_.shapes[0].dimensions[0] = .1;
    static_object_1_.shapes[0].dimensions[1] = 1.5;
    static_object_1_.poses.resize(1);
    static_object_1_.poses[0].position.x = .75;
    static_object_1_.poses[0].position.y = -.37;
    static_object_1_.poses[0].position.z = .81;
    static_object_1_.poses[0].orientation.w = 1.0;
      
    static_object_2_.header.stamp = ros::Time::now();
    static_object_2_.header.frame_id = "odom_combined";
    static_object_2_.id = "object_2";
    static_object_2_.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
    static_object_2_.shapes.resize(2);
    static_object_2_.shapes[0].type = arm_navigation_msgs::Shape::BOX;
    static_object_2_.shapes[0].dimensions.resize(3);
    static_object_2_.shapes[0].dimensions[0] = 1.0;
    static_object_2_.shapes[0].dimensions[1] = 1.0;
    static_object_2_.shapes[0].dimensions[2] = .05;
    static_object_2_.shapes[1].type = arm_navigation_msgs::Shape::BOX;
    static_object_2_.shapes[1].dimensions.resize(3);
    static_object_2_.shapes[1].dimensions[0] = 1.0;
    static_object_2_.shapes[1].dimensions[1] = 1.0;
    static_object_2_.shapes[1].dimensions[2] = .05;
    static_object_2_.poses.resize(2);
    static_object_2_.poses[0].position.x = 1.0;
    static_object_2_.poses[0].position.y = 0;
    static_object_2_.poses[0].position.z = .5;
    static_object_2_.poses[0].orientation.x = 0;
    static_object_2_.poses[0].orientation.y = 0;
    static_object_2_.poses[0].orientation.z = 0;
    static_object_2_.poses[0].orientation.w = 1;
    static_object_2_.poses[1].position.x = 1.0;
    static_object_2_.poses[1].position.y = 0;
    static_object_2_.poses[1].position.z = .75;
    static_object_2_.poses[1].orientation.x = 0;
    static_object_2_.poses[1].orientation.y = 0;
    static_object_2_.poses[1].orientation.z = 0;
    static_object_2_.poses[1].orientation.w = 1;
    
    static_object_3_.header.stamp = ros::Time::now();
    static_object_3_.header.frame_id = "odom_combined";
    static_object_3_.id = "object_3";
    static_object_3_.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
    static_object_3_.shapes.resize(1);
    static_object_3_.shapes[0].type = arm_navigation_msgs::Shape::BOX;
    static_object_3_.shapes[0].dimensions.resize(3);
    static_object_3_.shapes[0].dimensions[0] = 1.0;
    static_object_3_.shapes[0].dimensions[1] = 1.0;
    static_object_3_.shapes[0].dimensions[2] = .05;
    static_object_3_.poses.resize(1);
    static_object_3_.poses[0].position.x = .15;
    static_object_3_.poses[0].position.y = 0;
    static_object_3_.poses[0].position.z = .5;
    static_object_3_.poses[0].orientation.x = 0;
    static_object_3_.poses[0].orientation.y = 0;
    static_object_3_.poses[0].orientation.z = 0;
    static_object_3_.poses[0].orientation.w = 1;

    att_object_1_.object.header.stamp = ros::Time::now();
    att_object_1_.object.header.frame_id = "odom_combined";
    att_object_1_.link_name = "r_gripper_r_finger_tip_link";
    att_object_1_.object.id = "object_4";
    att_object_1_.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
    att_object_1_.object.shapes.resize(1);
    att_object_1_.object.shapes[0].type = arm_navigation_msgs::Shape::BOX;
    att_object_1_.object.shapes[0].dimensions.resize(3);
    att_object_1_.object.shapes[0].dimensions[0] = 1.0;
    att_object_1_.object.shapes[0].dimensions[1] = 1.0;
    att_object_1_.object.shapes[0].dimensions[2] = .05;
    att_object_1_.object.poses.resize(1);
    att_object_1_.object.poses[0].position.x = .15;
    att_object_1_.object.poses[0].position.y = 0;
    att_object_1_.object.poses[0].position.z = .5;
    att_object_1_.object.poses[0].orientation.x = 0;
    att_object_1_.object.poses[0].orientation.y = 0;
    att_object_1_.object.poses[0].orientation.z = 0;
    att_object_1_.object.poses[0].orientation.w = 1;

  }
  
protected:

  arm_navigation_msgs::CollisionObject static_object_1_;
  arm_navigation_msgs::CollisionObject static_object_2_;
  arm_navigation_msgs::CollisionObject static_object_3_;

  arm_navigation_msgs::AttachedCollisionObject att_object_1_;

  ros::NodeHandle nh_;
  std::string full_path_;
};


TEST_F(TestCollisionModels, NotInCollisionByDefault) 
{
  //this mostly tests that the planning_description file is correct
  planning_environment::CollisionModels cm("robot_description");
  
  planning_models::KinematicState state(cm.getKinematicModel());

  state.setKinematicStateToDefault();

  EXPECT_FALSE(cm.isKinematicStateInCollision(state));
  
  std::vector<arm_navigation_msgs::ContactInformation> contacts;
  cm.getAllCollisionsForState(state, contacts,1);
  
  EXPECT_EQ(contacts.size(),0);

  arm_navigation_msgs::OrderedCollisionOperations ord;
  ord.collision_operations.resize(1);
  ord.collision_operations[0].object1 = ord.collision_operations[0].COLLISION_SET_ALL;
  ord.collision_operations[0].object2 = ord.collision_operations[0].COLLISION_SET_ALL;
  ord.collision_operations[0].operation = ord.collision_operations[0].ENABLE;

  cm.applyOrderedCollisionOperationsToCollisionSpace(ord);

  EXPECT_TRUE(cm.isKinematicStateInCollision(state));
  
  cm.getAllCollisionsForState(state, contacts,1);
  
  EXPECT_GE(contacts.size(),0);

}

//Testing a couple important functions in model_utils.h
TEST_F(TestCollisionModels, TestModelUtils) 
{
  //this mostly tests that the planning_description file is correct
  planning_environment::CollisionModels cm("robot_description");
  
  planning_models::KinematicState state(cm.getKinematicModel());
  
  state.setKinematicStateToDefault();

  arm_navigation_msgs::RobotState rs;
  planning_environment::convertKinematicStateToRobotState(state,
                                                          ros::Time::now(),
                                                          cm.getWorldFrameId(),
                                                          rs);

  ASSERT_EQ(rs.multi_dof_joint_state.poses[0].position.x, 0.0);
  ASSERT_EQ(rs.multi_dof_joint_state.poses[0].position.y, 0.0);
  ASSERT_EQ(rs.multi_dof_joint_state.poses[0].position.z, 0.0);
  ASSERT_EQ(rs.multi_dof_joint_state.poses[0].orientation.x, 0.0);
  ASSERT_EQ(rs.multi_dof_joint_state.poses[0].orientation.y, 0.0);
  ASSERT_EQ(rs.multi_dof_joint_state.poses[0].orientation.z, 0.0);
  ASSERT_EQ(rs.multi_dof_joint_state.poses[0].orientation.w, 1.0);
  ASSERT_EQ(rs.multi_dof_joint_state.joint_names[0], std::string("base_joint"));
  ASSERT_EQ(rs.multi_dof_joint_state.frame_ids[0], cm.getWorldFrameId());
  ASSERT_EQ(rs.multi_dof_joint_state.child_frame_ids[0], cm.getRobotFrameId());
  
  ASSERT_TRUE(planning_environment::setRobotStateAndComputeTransforms(rs,state));

  EXPECT_EQ(rs.joint_state.name[0],std::string("floating_trans_x"));
  rs.joint_state.name.erase(rs.joint_state.name.begin());
  rs.joint_state.position.erase(rs.joint_state.position.begin());

  EXPECT_NE(rs.joint_state.name[0],std::string("floating_trans_x"));

  //should still be ok because the multi-dof is set
  ASSERT_TRUE(planning_environment::setRobotStateAndComputeTransforms(rs,state));

  //now we mess with the multi-dof, and now something's not getting set
  rs.multi_dof_joint_state.frame_ids.clear();
  ASSERT_FALSE(planning_environment::setRobotStateAndComputeTransforms(rs,state));
}

//Functional equivalent of test_collision_objects
TEST_F(TestCollisionModels,TestCollisionObjects)
{
  planning_environment::CollisionModels cm("robot_description");
   
  cm.addStaticObject(static_object_2_);
  cm.addStaticObject(static_object_3_);
  
  std::vector<arm_navigation_msgs::CollisionObject> space_objs;
  cm.getCollisionSpaceCollisionObjects(space_objs);

  std::vector<arm_navigation_msgs::AttachedCollisionObject> space_atts;
  cm.getCollisionSpaceAttachedCollisionObjects(space_atts);

  ASSERT_EQ(space_objs.size(),2);
  ASSERT_EQ(space_atts.size(),0);

  std::vector<std::string> touch_links;
  tf::Transform ident;
  ident.setIdentity();
  cm.convertStaticObjectToAttachedObject("object_3", "base_link", ident, touch_links);

  cm.getCollisionSpaceCollisionObjects(space_objs);
  cm.getCollisionSpaceAttachedCollisionObjects(space_atts);

  ASSERT_EQ(space_objs.size(),1);
  ASSERT_EQ(space_atts.size(),1);

  const collision_space::EnvironmentModel::AllowedCollisionMatrix& acm = cm.getCollisionSpace()->getCurrentAllowedCollisionMatrix();
  
  bool allowed;
  EXPECT_TRUE(acm.getAllowedCollision("object_3", "base_link", allowed));
  EXPECT_TRUE(allowed);

  cm.addAttachedObject(att_object_1_);

  cm.getCollisionSpaceCollisionObjects(space_objs);
  cm.getCollisionSpaceAttachedCollisionObjects(space_atts);

  ASSERT_EQ(space_objs.size(),1);
  ASSERT_EQ(space_atts.size(),2);

  cm.deleteAllAttachedObjects("base_link");

  cm.getCollisionSpaceAttachedCollisionObjects(space_atts);
  ASSERT_EQ(space_atts.size(),1);
  ASSERT_EQ(space_atts[0].object.id, "object_4");

  //non-existent second link has no effect
  cm.convertAttachedObjectToStaticObject("object_4","r_gripper_finger_tip_link", ident);

  cm.getCollisionSpaceCollisionObjects(space_objs);
  cm.getCollisionSpaceAttachedCollisionObjects(space_atts);

  ASSERT_EQ(space_objs.size(),1);
  ASSERT_EQ(space_atts.size(),1);

  //now we do it right
  cm.convertAttachedObjectToStaticObject("object_4","r_gripper_r_finger_tip_link", ident);

  cm.getCollisionSpaceCollisionObjects(space_objs);
  cm.getCollisionSpaceAttachedCollisionObjects(space_atts);

  ASSERT_EQ(space_objs.size(),2);
  ASSERT_EQ(space_atts.size(),0);
}

//Functional equivalent of test_alter_padding
TEST_F(TestCollisionModels,TestAlterLinkPadding)
{
  planning_environment::CollisionModels cm("robot_description");
  
  planning_models::KinematicState state(cm.getKinematicModel());

  state.setKinematicStateToDefault();

  EXPECT_FALSE(cm.isKinematicStateInCollision(state));

  cm.addStaticObject(static_object_1_);

  //with 0.01 padding, shouldn't be in collision
  EXPECT_FALSE(cm.isKinematicStateInCollision(state));
  
  std::vector<arm_navigation_msgs::LinkPadding> padd_vec;
  padd_vec.resize(3);
  padd_vec[0].link_name = "r_gripper_palm_link";
  padd_vec[0].padding = .1;
  padd_vec[1].link_name = "r_gripper_r_finger_link";
  padd_vec[1].padding = .1;
  padd_vec[2].link_name = "r_gripper_l_finger_link";
  padd_vec[2].padding = .1;


  cm.applyLinkPaddingToCollisionSpace(padd_vec);

  EXPECT_TRUE(cm.isKinematicStateInCollision(state));
  EXPECT_FALSE(cm.isKinematicStateInSelfCollision(state));
  EXPECT_TRUE(cm.isKinematicStateInEnvironmentCollision(state));

  std::vector<arm_navigation_msgs::ContactInformation> contacts;
  cm.getAllCollisionsForState(state, contacts,1);
  
  EXPECT_GE(contacts.size(),1);

  for(unsigned int i = 0; i < contacts.size(); i++) {
    EXPECT_TRUE(contacts[i].contact_body_1 == "object_1" || contacts[i].contact_body_2 == "object_1") << contacts[i].contact_body_1 << " " << contacts[i].contact_body_2;
    EXPECT_TRUE(contacts[i].contact_body_1 != "object_1" || contacts[i].contact_body_2 != "object_1");
    EXPECT_TRUE(contacts[i].body_type_1 == arm_navigation_msgs::ContactInformation::ROBOT_LINK ||
                contacts[i].body_type_2 == arm_navigation_msgs::ContactInformation::ROBOT_LINK);    
    EXPECT_TRUE(contacts[i].body_type_1 == arm_navigation_msgs::ContactInformation::OBJECT ||
                contacts[i].body_type_2 == arm_navigation_msgs::ContactInformation::OBJECT);
  }
  
  cm.revertCollisionSpacePaddingToDefault();

  EXPECT_FALSE(cm.isKinematicStateInCollision(state));
}

//Functional equivalent of test_allowed_collision_operations
TEST_F(TestCollisionModels,TestAllowedCollisions)
{
  planning_environment::CollisionModels cm("robot_description");

  {
    planning_models::KinematicState state(cm.getKinematicModel());
    
    state.setKinematicStateToDefault();
  }
  collision_space::EnvironmentModel::AllowedCollisionMatrix check_acm = cm.getDefaultAllowedCollisionMatrix();

  bool allowed;
  ASSERT_TRUE(check_acm.getAllowedCollision("r_gripper_palm_link","l_gripper_palm_link", allowed));
  EXPECT_FALSE(allowed);

  arm_navigation_msgs::OrderedCollisionOperations ord;
  ord.collision_operations.resize(1);
  ord.collision_operations[0].object1 = "r_gripper_palm_link";
  ord.collision_operations[0].object2 = "l_gripper_palm_link";
  ord.collision_operations[0].operation = ord.collision_operations[0].DISABLE;

  cm.applyOrderedCollisionOperationsToCollisionSpace(ord);  

  check_acm = cm.getCurrentAllowedCollisionMatrix();

  ASSERT_TRUE(check_acm.getAllowedCollision("r_gripper_palm_link","l_gripper_palm_link", allowed));
  EXPECT_TRUE(allowed);

  cm.revertAllowedCollisionToDefault();

  check_acm = cm.getCurrentAllowedCollisionMatrix();

  ASSERT_TRUE(check_acm.getAllowedCollision("r_gripper_palm_link","l_gripper_palm_link", allowed));
  EXPECT_FALSE(allowed);

  check_acm.changeEntry("r_gripper_palm_link", "l_gripper_palm_link", true);

  cm.setAlteredAllowedCollisionMatrix(check_acm);

  ASSERT_TRUE(check_acm.getAllowedCollision("r_gripper_palm_link","l_gripper_palm_link", allowed));
  EXPECT_TRUE(allowed);

  cm.revertAllowedCollisionToDefault();

  //now testing with objects

  cm.addStaticObject(static_object_1_);

  check_acm = cm.getCurrentAllowedCollisionMatrix();  
  
  ASSERT_TRUE(check_acm.getAllowedCollision("r_gripper_palm_link","object_1", allowed));
  EXPECT_FALSE(allowed);

  ord.collision_operations[0].object1 = "r_gripper_palm_link";
  ord.collision_operations[0].object2 = ord.collision_operations[0].COLLISION_SET_OBJECTS;
  ord.collision_operations[0].operation = ord.collision_operations[0].DISABLE;

  cm.applyOrderedCollisionOperationsToCollisionSpace(ord);  

  check_acm = cm.getCurrentAllowedCollisionMatrix();  
  
  ASSERT_TRUE(check_acm.getAllowedCollision("r_gripper_palm_link","object_1", allowed));
  EXPECT_TRUE(allowed);

  //now with attached objects
  att_object_1_.touch_links.push_back("r_gripper_palm_link");
  cm.addAttachedObject(att_object_1_);

  //this should also revert the allowed collision matrix
  cm.revertAllowedCollisionToDefault();

  check_acm = cm.getCurrentAllowedCollisionMatrix();  
  
  ASSERT_TRUE(check_acm.getAllowedCollision("r_gripper_palm_link","object_1", allowed));
  EXPECT_FALSE(allowed);

  check_acm = cm.getCurrentAllowedCollisionMatrix();
 
  ASSERT_TRUE(check_acm.getAllowedCollision("r_gripper_palm_link","object_4", allowed));
  EXPECT_TRUE(allowed);

  ASSERT_TRUE(check_acm.getAllowedCollision("r_gripper_r_finger_tip_link","object_4", allowed));
  EXPECT_TRUE(allowed);

  ASSERT_TRUE(check_acm.getAllowedCollision("r_gripper_l_finger_tip_link","object_4", allowed));
  EXPECT_FALSE(allowed);

  ord.collision_operations[0].object1 = "r_gripper_palm_link";
  ord.collision_operations[0].object2 = ord.collision_operations[0].COLLISION_SET_ATTACHED_OBJECTS;
  ord.collision_operations[0].operation = ord.collision_operations[0].DISABLE;

  cm.applyOrderedCollisionOperationsToCollisionSpace(ord);  
  
  check_acm = cm.getCurrentAllowedCollisionMatrix();

  //this should override touch_links

  ASSERT_TRUE(check_acm.getAllowedCollision("r_gripper_palm_link","object_4", allowed));
  EXPECT_TRUE(allowed);

}

//Functional equivalent of test_allowed_collision_operations
TEST_F(TestCollisionModels,TestAttachedObjectCollisions)
{
  planning_environment::CollisionModels cm("robot_description");

  arm_navigation_msgs::CollisionObject table;
  
  table.header.stamp = ros::Time::now();
  table.header.frame_id = "odom_combined";
  table.id = "wall";
  table.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  table.shapes.resize(1);
  table.shapes[0].type = arm_navigation_msgs::Shape::BOX;
  table.shapes[0].dimensions.resize(3);
  table.shapes[0].dimensions[0] = 0.05;
  table.shapes[0].dimensions[1] = 4.0;
  table.shapes[0].dimensions[2] = 4.0;
  table.poses.resize(1);
  table.poses[0].position.x = .7;
  table.poses[0].position.y = 0;
  table.poses[0].position.z = 1.0;
  table.poses[0].orientation.w = 1.0;

  cm.addStaticObject(table);

  arm_navigation_msgs::AttachedCollisionObject pole;

  pole.object.header.stamp = ros::Time::now();
  pole.object.header.frame_id = "r_gripper_r_finger_tip_link";
  pole.link_name = "r_gripper_r_finger_tip_link";
  pole.object.id = "pole";
  pole.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  pole.object.shapes.resize(1);
  pole.object.shapes[0].type = arm_navigation_msgs::Shape::CYLINDER;
  pole.object.shapes[0].dimensions.resize(2);
  pole.object.shapes[0].dimensions[0] = .05;
  pole.object.shapes[0].dimensions[1] = .4;
  pole.object.poses.resize(1);
  pole.object.poses[0].position.x = 0;
  pole.object.poses[0].position.y = 0;
  pole.object.poses[0].position.z = 0;
  pole.object.poses[0].orientation.x = 0;
  pole.object.poses[0].orientation.y = 0;
  pole.object.poses[0].orientation.z = 0;
  pole.object.poses[0].orientation.w = 1;
  
  std::vector<std::string> touch_links;

  touch_links.push_back("r_gripper_palm_link");
  touch_links.push_back("r_gripper_r_finger_link");
  touch_links.push_back("r_gripper_l_finger_link");
  touch_links.push_back("r_gripper_l_finger_tip_link");
  
  cm.addAttachedObject(pole);

  //with no touch links should be in collision
  {
    planning_models::KinematicState state(cm.getKinematicModel());
    
    state.setKinematicStateToDefault();

    EXPECT_TRUE(cm.isKinematicStateInCollision(state));

    std::vector<arm_navigation_msgs::ContactInformation> contacts;
    cm.getAllCollisionsForState(state, contacts,1);
    
    EXPECT_GE(contacts.size(),1);    

    //should get contacts between pole and touch_links and pole and table
    bool got_object = false;
    std::map<std::string, bool> touch_links_map;
    for(unsigned int i = 0; i < touch_links.size(); i++) {
      touch_links_map[touch_links[i]] = false;
    }
    for(unsigned int i = 0; i < contacts.size(); i++) {
      std::string other_body_name;
      char other_body_type;
      if(contacts[i].body_type_1 == arm_navigation_msgs::ContactInformation::ATTACHED_BODY) {
        other_body_name = contacts[i].contact_body_2;
        other_body_type = contacts[i].body_type_2;
      } else if(contacts[i].body_type_2 == arm_navigation_msgs::ContactInformation::ATTACHED_BODY) {
        other_body_name = contacts[i].contact_body_1;
        other_body_type = contacts[i].body_type_1;
      } else {
        ASSERT_TRUE(false) << "Collision other than with attached object " << contacts[i].contact_body_1 << " and " << contacts[i].contact_body_2;
      }
      if(other_body_type == arm_navigation_msgs::ContactInformation::ROBOT_LINK) {
        EXPECT_FALSE(touch_links_map.find(other_body_name) == touch_links_map.end()) << contacts[i].contact_body_1 << " and " << contacts[i].contact_body_2;
        touch_links_map[other_body_name] = true;
      } else {
        got_object = true;
      }
    }
    EXPECT_TRUE(got_object);
    for(std::map<std::string, bool>::iterator it = touch_links_map.begin();
        it != touch_links_map.end();
        it++) {
      EXPECT_TRUE(it->second) << it->first << " not in collision";
    }
  }
  pole.touch_links = touch_links;
  cm.addAttachedObject(pole);

  {
    planning_models::KinematicState state(cm.getKinematicModel());
    
    state.setKinematicStateToDefault();  

    EXPECT_TRUE(cm.isKinematicStateInCollision(state));
    EXPECT_TRUE(cm.isKinematicStateInEnvironmentCollision(state));
    EXPECT_FALSE(cm.isKinematicStateInSelfCollision(state));
  }
  
  //Testing group touch links
  pole.touch_links.clear();
  pole.touch_links.push_back("r_end_effector");
  cm.addAttachedObject(pole);

  {
    planning_models::KinematicState state(cm.getKinematicModel());
    
    state.setKinematicStateToDefault();  

    EXPECT_TRUE(cm.isKinematicStateInCollision(state));
    EXPECT_TRUE(cm.isKinematicStateInEnvironmentCollision(state));
    EXPECT_FALSE(cm.isKinematicStateInSelfCollision(state));
  }
  
}

TEST_F(TestCollisionModels, TestTrajectoryValidity)
{

  planning_environment::CollisionModels cm("robot_description");

  static_object_1_.poses[0].position.x = .45;
  static_object_1_.poses[0].position.y = -.5;
  cm.addStaticObject(static_object_1_);

  planning_models::KinematicState kin_state(cm.getKinematicModel());
  kin_state.setKinematicStateToDefault();

  std::map<std::string, double> jm;
  jm["r_shoulder_pan_joint"] = -2.0;
  kin_state.setKinematicState(jm);
  ASSERT_FALSE(cm.isKinematicStateInCollision(kin_state));
  
  arm_navigation_msgs::Constraints goal_constraints;
  goal_constraints.joint_constraints.resize(1);
  goal_constraints.joint_constraints[0].joint_name = "r_shoulder_pan_joint";
  goal_constraints.joint_constraints[0].position = -2.0;
  goal_constraints.joint_constraints[0].tolerance_below = 0.1;
  goal_constraints.joint_constraints[0].tolerance_above = 0.1;

  //empty path_constraints
  arm_navigation_msgs::Constraints path_constraints;

  //just testing goal constraints
  trajectory_msgs::JointTrajectory trajectory;
  trajectory.joint_names.push_back("r_shoulder_pan_joint");
  trajectory.points.resize(1);
  trajectory.points[0].positions.resize(1);
  trajectory.points[0].positions[0] = -2.0;
  
  std::vector<arm_navigation_msgs::ArmNavigationErrorCodes> trajectory_error_codes;
  arm_navigation_msgs::ArmNavigationErrorCodes error_code;
  ASSERT_TRUE(cm.isJointTrajectoryValid(kin_state, trajectory, goal_constraints, path_constraints,
                                        error_code, trajectory_error_codes, false)) << error_code.val;
  EXPECT_EQ(error_code.val, error_code.SUCCESS);

  //should be out of bounds
  trajectory.points[0].positions[0] = -1.8;

  ASSERT_FALSE(cm.isJointTrajectoryValid(kin_state, trajectory, goal_constraints, path_constraints,
                                    error_code, trajectory_error_codes, false));
  EXPECT_EQ(error_code.val, error_code.GOAL_CONSTRAINTS_VIOLATED);

  //valid goal constraint, but out of joint limits
  goal_constraints.joint_constraints[0].position = -2.3;
  trajectory.points[0].positions[0] = -2.0;

  ASSERT_FALSE(cm.isJointTrajectoryValid(kin_state, trajectory, goal_constraints, path_constraints,
                                    error_code, trajectory_error_codes, false));
  EXPECT_EQ(error_code.val, error_code.INVALID_GOAL_JOINT_CONSTRAINTS);

  //now we discretize and check collisions

  goal_constraints.joint_constraints[0].position = -2.0;

  unsigned int num_points = fabs(100.0/-2.0);
  trajectory.points.resize(num_points);
  for(unsigned int i = 1; i <= num_points; i++) {
    trajectory.points[i-1].positions.resize(1);
    trajectory.points[i-1].positions[0] = -2.0*((i*1.0)/(1.0*num_points));
  }
  EXPECT_EQ(trajectory.points.back().positions[0], -2.0);
  
  ASSERT_FALSE(cm.isJointTrajectoryValid(kin_state, trajectory, goal_constraints, path_constraints,
                                         error_code, trajectory_error_codes, false));
  EXPECT_EQ(error_code.val, error_code.COLLISION_CONSTRAINTS_VIOLATED);
}

TEST_F(TestCollisionModels, TestConversionFunctionsForObjects)
{

  planning_environment::CollisionModels cm("robot_description");
  
  planning_models::KinematicState kin_state(cm.getKinematicModel());
  kin_state.setKinematicStateToDefault();

  arm_navigation_msgs::RobotState robot_state;

  planning_environment::convertKinematicStateToRobotState(kin_state,
                                                          ros::Time::now(),
                                                          cm.getWorldFrameId(),
                                                          robot_state);

  robot_state.multi_dof_joint_state.poses[0].position.x = 4.0;
  robot_state.multi_dof_joint_state.poses[0].orientation.w = 1.0;

  planning_environment::setRobotStateAndComputeTransforms(robot_state, kin_state);

  arm_navigation_msgs::CollisionObject obj1;
  obj1.header.stamp = ros::Time::now();
  obj1.header.frame_id = "base_footprint";
  obj1.id = "obj1";
  obj1.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  obj1.shapes.resize(1);
  obj1.shapes[0].type = arm_navigation_msgs::Shape::BOX;
  obj1.shapes[0].dimensions.resize(3);
  obj1.shapes[0].dimensions[0] = .1;
  obj1.shapes[0].dimensions[1] = .1;
  obj1.shapes[0].dimensions[2] = .75;
  obj1.poses.resize(1);
  obj1.poses[0].position.x = .5;
  obj1.poses[0].position.y = .5;
  obj1.poses[0].position.z = 0;
  obj1.poses[0].orientation.w = 1.0;

  arm_navigation_msgs::AttachedCollisionObject att_obj;
  att_obj.object = obj1;
  att_obj.object.header.stamp = ros::Time::now();
  att_obj.object.header.frame_id = "r_gripper_r_finger_tip_link";
  att_obj.link_name = "r_gripper_palm_link";
  att_obj.touch_links.push_back("r_end_effector");
  att_obj.touch_links.push_back("r_wrist_roll_link");
  att_obj.touch_links.push_back("r_wrist_flex_link");
  att_obj.touch_links.push_back("r_forearm_link");
  att_obj.object.id = "obj2";
  att_obj.object.shapes[0].type = arm_navigation_msgs::Shape::CYLINDER;
  att_obj.object.shapes[0].dimensions.resize(2);
  att_obj.object.shapes[0].dimensions[0] = .025;
  att_obj.object.shapes[0].dimensions[1] = .5;
  att_obj.object.poses.resize(1);
  att_obj.object.poses[0].position.x = 0.0;
  att_obj.object.poses[0].position.y = 0.0;
  att_obj.object.poses[0].position.z = 0.0;
  att_obj.object.poses[0].orientation.w = 1.0;
  
  cm.convertAttachedCollisionObjectToNewWorldFrame(kin_state,
                                                   att_obj);
  //last one should replace other
  EXPECT_EQ(att_obj.object.header.frame_id, "r_gripper_palm_link");

  cm.convertAttachedCollisionObjectToNewWorldFrame(kin_state,
                                                   att_obj);
  
  att_obj.link_name = "base_footprint";
  att_obj.object.header.frame_id = "base_footprint";
  att_obj.object.id = "obj3";
  att_obj.object.poses[0].position.x = 0.12;
  
  cm.convertAttachedCollisionObjectToNewWorldFrame(kin_state,
                                                   att_obj);

  EXPECT_GE(att_obj.object.poses[0].position.x,0.0);
  EXPECT_LE(fabs(att_obj.object.poses[0].position.x-.12), VERY_SMALL); 

  cm.convertCollisionObjectToNewWorldFrame(kin_state,
                                        obj1);
  
  EXPECT_EQ(obj1.header.frame_id, "odom_combined");
  EXPECT_LE(fabs(obj1.poses[0].position.x-4.5),VERY_SMALL);
  EXPECT_LE(fabs(obj1.poses[0].position.y-.5),VERY_SMALL);
}

TEST_F(TestCollisionModels, TestConversionFunctionsForConstraints)
{

  planning_environment::CollisionModels cm("robot_description");

  planning_models::KinematicState kin_state(cm.getKinematicModel());
  kin_state.setKinematicStateToDefault();

  arm_navigation_msgs::RobotState robot_state;

  robot_state.multi_dof_joint_state.stamp = ros::Time::now();
  robot_state.multi_dof_joint_state.joint_names.push_back("base_joint");
  robot_state.multi_dof_joint_state.frame_ids.push_back("odom_combined");
  robot_state.multi_dof_joint_state.child_frame_ids.push_back("base_footprint");
  robot_state.multi_dof_joint_state.poses.resize(1);
  robot_state.multi_dof_joint_state.poses[0].position.x = 4.0;
  robot_state.multi_dof_joint_state.poses[0].orientation.w = 1.0;

  planning_environment::setRobotStateAndComputeTransforms(robot_state, kin_state);

  arm_navigation_msgs::Constraints goal_constraints;
  arm_navigation_msgs::PositionConstraint pos;
  pos.header.frame_id = "base_footprint";
  pos.header.stamp = ros::Time::now();
  pos.link_name = "r_wrist_roll_link";
  pos.position.x = .5;
  pos.constraint_region_orientation.w = 1.0;

  goal_constraints.position_constraints.push_back(pos);

  cm.convertConstraintsGivenNewWorldTransform(kin_state,
                                              goal_constraints);

  EXPECT_TRUE(goal_constraints.position_constraints[0].header.frame_id == "odom_combined");
  EXPECT_LE(fabs(goal_constraints.position_constraints[0].position.x-4.5), VERY_SMALL) ;
  
  //checking that if we turn around then we should subtract .5 in x

  tf::Quaternion turn(tf::Vector3(0,0,1), M_PI);

  robot_state.multi_dof_joint_state.poses[0].orientation.x = turn.x(); 
  robot_state.multi_dof_joint_state.poses[0].orientation.y = turn.y(); 
  robot_state.multi_dof_joint_state.poses[0].orientation.z = turn.z(); 
  robot_state.multi_dof_joint_state.poses[0].orientation.w = turn.w(); 

  planning_environment::setRobotStateAndComputeTransforms(robot_state, kin_state);
  
  goal_constraints.position_constraints.clear();
  goal_constraints.position_constraints.push_back(pos);

  cm.convertConstraintsGivenNewWorldTransform(kin_state,
                                              goal_constraints);
  

  EXPECT_TRUE(goal_constraints.position_constraints[0].header.frame_id == "odom_combined");
  EXPECT_LE(fabs(goal_constraints.position_constraints[0].position.x-3.5), VERY_SMALL) ;
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_collision_models");
    
  return RUN_ALL_TESTS();
}

