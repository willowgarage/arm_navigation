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

#include <planning_environment/models/collision_models.h>
#include <planning_models/kinematic_state.h>
#include <ros/time.h>
#include <gtest/gtest.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <ros/package.h>

static const std::string rel_path = "/test_urdf/robot.xml";

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
    static_object_1_.id = "static_object_1";
    static_object_1_.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
    static_object_1_.shapes.resize(1);
    static_object_1_.shapes[0].type = geometric_shapes_msgs::Shape::CYLINDER;
    static_object_1_.shapes[0].dimensions.resize(2);
    static_object_1_.shapes[0].dimensions[0] = .1;
    static_object_1_.shapes[0].dimensions[1] = 1.5;
    static_object_1_.poses.resize(1);
    static_object_1_.poses[0].position.x = .75;
    static_object_1_.poses[0].position.y = -.37;
    static_object_1_.poses[0].position.z = .81;
    static_object_1_.poses[0].orientation.w = 1.0;
    
  }
  
protected:

  mapping_msgs::CollisionObject static_object_1_;

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
  
  std::vector<planning_environment_msgs::ContactInformation> contacts;
  cm.getAllCollisionsForState(state, contacts,1);
  
  EXPECT_EQ(contacts.size(),0);

  motion_planning_msgs::OrderedCollisionOperations ord;
  ord.collision_operations.resize(1);
  ord.collision_operations[0].object1 = ord.collision_operations[0].COLLISION_SET_ALL;
  ord.collision_operations[0].object2 = ord.collision_operations[0].COLLISION_SET_ALL;
  ord.collision_operations[0].operation = ord.collision_operations[0].ENABLE;

  cm.applyOrderedCollisionOperationsToCollisionSpace(ord);

  EXPECT_TRUE(cm.isKinematicStateInCollision(state));
  
  cm.getAllCollisionsForState(state, contacts,1);
  
  EXPECT_GE(contacts.size(),0);

}

TEST_F(TestCollisionModels,TestAlterLinkPadding)
{
  planning_environment::CollisionModels cm("robot_description");
  
  planning_models::KinematicState state(cm.getKinematicModel());

  state.setKinematicStateToDefault();

  EXPECT_FALSE(cm.isKinematicStateInCollision(state));

  cm.addStaticObject(static_object_1_);

  //with 0.01 padding, shouldn't be in collision
  EXPECT_FALSE(cm.isKinematicStateInCollision(state));
  
  std::vector<motion_planning_msgs::LinkPadding> padd_vec;
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

  std::vector<planning_environment_msgs::ContactInformation> contacts;
  cm.getAllCollisionsForState(state, contacts,1);
  
  EXPECT_GE(contacts.size(),1);

  for(unsigned int i = 0; i < contacts.size(); i++) {
    EXPECT_TRUE(contacts[i].contact_body_1 == "static_object_1" || contacts[i].contact_body_2 == "static_object_1") << contacts[i].contact_body_1 << " " << contacts[i].contact_body_2;
    EXPECT_TRUE(contacts[i].contact_body_1 != "static_object_1" || contacts[i].contact_body_2 != "static_object_1");
    EXPECT_TRUE(contacts[i].body_type_1 == planning_environment_msgs::ContactInformation::ROBOT_LINK ||
                contacts[i].body_type_2 == planning_environment_msgs::ContactInformation::ROBOT_LINK);    
    EXPECT_TRUE(contacts[i].body_type_1 == planning_environment_msgs::ContactInformation::OBJECT ||
                contacts[i].body_type_2 == planning_environment_msgs::ContactInformation::OBJECT);
  }
  
  cm.revertCollisionSpacePaddingToDefault();

  EXPECT_FALSE(cm.isKinematicStateInCollision(state));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_collision_models");
    
  return RUN_ALL_TESTS();
}

