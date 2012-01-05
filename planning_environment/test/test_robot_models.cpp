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

#include <planning_environment/models/robot_models.h>
#include <planning_models/kinematic_state.h>
#include <ros/time.h>
#include <gtest/gtest.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <ros/package.h>

static const std::string rel_path = "/test_urdf/robot.xml";

static const std::string FLOATING_JOINT_XML = 
  "<value>"
  "<array>"
  "<data>"
  "<value>"
  "<struct>"
  "<member>"
  "<name>name</name>"
  "<value><string>base_joint</string></value>"
  "</member>"
  "<member>"
  "<name>parent_frame_id</name>"
  "<value><string>base_footprint</string></value>"
  "</member>"
  "<member>"
  "<name>child_frame_id</name>"
  "<value><string>base_footprint</string></value>"
  "</member>"
  "<member>"
  "<name>type</name>"
  "<value><string>Floating</string></value>"
  "</member>"
  "</struct>"
  "</value>"
  "</data>"
  "</array>"
  "</value>";

static const std::string RIGHT_ARM_GROUP_XML = 
  "<value>"
  "<array>"
  "<data>"
  "<value>"
  "<struct>"
  "<member>"
  "<name>name</name>"
  "<value><string>right_arm</string></value>"
  "</member>"
  "<member>"
  "<name>base_link</name>"
  "<value><string>torso_lift_link</string></value>"
  "</member>"
  "<member>"
  "<name>tip_link</name>"
  "<value><string>r_wrist_roll_link</string></value>"
  "</member>"
  "</struct>"
  "</value>"
  "</data>"
  "</array>"
  "</value>";

class TestRobotModels : public testing::Test 
{
protected:
  
  virtual void SetUp() {

    full_path_ = ros::package::getPath("planning_models")+rel_path;
    
    std::string com = "rosparam set robot_description -t "+full_path_;

    int ok = system(com.c_str());
    
    if(ok != 0) {
      ROS_WARN_STREAM("Setting parameter system call not ok");
    }
  }
  
protected:

  ros::NodeHandle nh_;
  std::string full_path_;
};

TEST_F(TestRobotModels, Loading)
{
  int offset1=0;

  //this will be used by other tests unless another joint is pushed
  XmlRpc::XmlRpcValue floating_multi_dof_joint(FLOATING_JOINT_XML, &offset1);

  ASSERT_TRUE(floating_multi_dof_joint.valid());
  ASSERT_EQ(floating_multi_dof_joint.getType(),XmlRpc::XmlRpcValue::TypeArray); 

  nh_.setParam("robot_description_planning/multi_dof_joints", floating_multi_dof_joint);

  //and these groups
  offset1 = 0;
  XmlRpc::XmlRpcValue planning_groups(RIGHT_ARM_GROUP_XML, &offset1);

  ASSERT_TRUE(planning_groups.valid());
  ASSERT_EQ(planning_groups.getType(),XmlRpc::XmlRpcValue::TypeArray); 

  nh_.setParam("robot_description_planning/groups", planning_groups);
  
  planning_environment::RobotModels m("robot_description");

  ASSERT_TRUE(m.getKinematicModel() != NULL);

  //now we test that the root transform has all the expected values
  const planning_models::KinematicModel* kmodel = m.getKinematicModel();

  const planning_models::KinematicModel::JointModel* j = kmodel->getRoot();
    
  ASSERT_TRUE(j != NULL);
  
  //check if it's the right type - this means that yaml parsing also works
  const planning_models::KinematicModel::FloatingJointModel* pj = dynamic_cast<const planning_models::KinematicModel::FloatingJointModel*>(j);
  EXPECT_TRUE(pj != NULL);
}

TEST_F(TestRobotModels, SetGetOperations)
{
  planning_environment::RobotModels m("robot_description");
  const planning_models::KinematicModel* kmodel = m.getKinematicModel();

  planning_models::KinematicState state(kmodel);

  state.setKinematicStateToDefault();

  //all positive
  std::map<std::string, double> vals; 
  state.getKinematicStateValues(vals);
  for(std::map<std::string, double>::iterator it = vals.begin();
      it != vals.end();
      it++) {
    it->second = .1;
  }
  state.setKinematicState(vals);

  std::map<std::string, double> test_vals;
  state.getKinematicStateValues(test_vals);
  for(std::map<std::string, double>::iterator it = vals.begin();
      it != vals.end();
      it++) {
    EXPECT_TRUE(test_vals.find(it->first) != test_vals.end()) << "no value set for joint " << it->first;
    EXPECT_LE(fabs(it->second-test_vals[it->first]),.00001) << "Value for " << it->first << " is " << test_vals[it->first] << " instead of " << it->second;
  }

  //all negative
  for(std::map<std::string, double>::iterator it = vals.begin();
      it != vals.end();
      it++) {
    it->second = -.1;
  }
  state.setKinematicState(vals);

  state.getKinematicStateValues(test_vals);
  for(std::map<std::string, double>::iterator it = vals.begin();
      it != vals.end();
      it++) {
    EXPECT_TRUE(test_vals.find(it->first) != test_vals.end()) << "no value set for joint " << it->first;
    EXPECT_LE(fabs(it->second-test_vals[it->first]),.00001) << "Value for " << it->first << " is " << test_vals[it->first] << " instead of " << it->second;
  }

  //using root transform
  const planning_models::KinematicModel::JointModel* j = kmodel->getRoot();
  tf::Transform bt;
  bt.setOrigin(tf::Vector3(5.0, 5.0, 5.0));
  bt.setRotation(tf::Quaternion(0,0,.7071,.7071));
  state.getJointState(j->getName())->setJointStateValues(bt);
  
  test_vals.clear();

  state.getKinematicStateValues(test_vals);
  //for(std::map<std::string, double>::iterator it = test_vals.begin();
  //    it != test_vals.end();
  //    it++) {
  // ROS_INFO_STREAM("Var " << it->first << " val " << it->second);
  //}

  EXPECT_EQ(test_vals["floating_trans_x"],5.0);
  EXPECT_EQ(test_vals["floating_trans_y"], 5.0);
  EXPECT_LE(fabs(test_vals["floating_rot_z"]-.7071), .001); 

}

TEST_F(TestRobotModels, SetGetBounds)
{
  planning_environment::RobotModels m("robot_description");
  const planning_models::KinematicModel* kmodel = m.getKinematicModel();

  planning_models::KinematicState state(kmodel);

  state.setKinematicStateToDefault();

  std::map<std::string, double> test_vals;
  test_vals["r_shoulder_pan_joint"] =  4.0;
  test_vals["r_wrist_roll_link"] = 4.0;
  state.setKinematicState(test_vals);
  EXPECT_FALSE(state.isJointWithinBounds("r_shoulder_pan_joint"));
  EXPECT_TRUE(state.isJointWithinBounds("r_wrist_roll_joint"));
}

TEST_F(TestRobotModels, ForwardKinematics)
{
  planning_environment::RobotModels m("robot_description");
  const planning_models::KinematicModel* kmodel = m.getKinematicModel();
  
  planning_models::KinematicState state(kmodel);
  
  state.setKinematicStateToDefault();

  planning_models::KinematicState::JointStateGroup *group = state.getJointStateGroup("right_arm");
  
  ASSERT_TRUE(group != NULL);

  std::map<std::string, double> vals;
  group->getKinematicStateValues(vals);
  for(std::map<std::string, double>::iterator it = vals.begin();
      it != vals.end();
      it++) {
    it->second = .1;
  }
  std::map<std::string, double> vals2;
  group->getKinematicStateValues(vals2);
  for(std::map<std::string, double>::iterator it = vals2.begin();
      it != vals2.end();
      it++) {
    it->second = .1;
  }
  ros::WallTime tm = ros::WallTime::now();
  const unsigned int NT = 100000;  
  for (unsigned int i = 0 ; i < NT ; ++i) {
    if(i%2 == 0) {
      group->setKinematicState(vals);
    } else {
      group->setKinematicState(vals2);
    }
  }
  double fps = (double)NT / (ros::WallTime::now() - tm).toSec();
  ROS_INFO("Map %f forward kinematics steps per second", fps);

  EXPECT_GT(fps,5000.0);
  
  std::vector<double> jv(group->getDimension(), 0.1);
  std::vector<double> jv2(group->getDimension(), 0.2);
  tm = ros::WallTime::now();
  for (unsigned int i = 0 ; i < NT ; ++i) {
    if(i%2 == 0) {
      group->setKinematicState(jv);
    } else {
      group->setKinematicState(jv2);
    }
  }
  fps = (double)NT / (ros::WallTime::now() - tm).toSec();
  ROS_INFO("Vector %f forward kinematics steps per second", fps);
  EXPECT_GT(fps,5000.0);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_robot_models");
    
  return RUN_ALL_TESTS();
}
