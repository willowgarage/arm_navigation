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

#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
#include <gtest/gtest.h>
#include <sstream>
#include <ctype.h>
#include <ros/package.h>
#include <collision_space/environmentODE.h>
#include <boost/thread.hpp>

//urdf location relative to the planning_models path
static const std::string rel_path = "/test_urdf/robot.xml";

class TestCollisionSpace : public testing::Test {
public:

  void spinThread() {
    lock_.lock();
    coll_space_->isCollision();
    lock_.unlock();
  }

protected:
  
  virtual void SetUp() {

    full_path_ = ros::package::getPath("planning_models")+rel_path;
    
    urdf_ok_ = urdf_model_.initFile(full_path_);

    std::vector<planning_models::KinematicModel::MultiDofConfig> multi_dof_configs;
    //now this should work with an identity transform
    planning_models::KinematicModel::MultiDofConfig config("base_joint");
    config.type = "Planar";
    config.parent_frame_id = "base_footprint";
    config.child_frame_id = "base_footprint";
    multi_dof_configs.push_back(config);

    std::vector<planning_models::KinematicModel::GroupConfig> gcs;
    planning_models::KinematicModel::GroupConfig left_arm("left_arm",
                                                          "torso_lift_link",
                                                          "l_wrist_roll_link");

    planning_models::KinematicModel::GroupConfig right_arm("right_arm",
                                                           "torso_lift_link",
                                                           "r_wrist_roll_link");

    kinematic_model_ = new planning_models::KinematicModel(urdf_model_,
                                                           gcs,
                                                           multi_dof_configs);
    coll_space_ = new collision_space::EnvironmentModelODE();
  };

  virtual void TearDown() {
    delete kinematic_model_;
    delete coll_space_;
  }
protected:

  boost::mutex lock_;

  urdf::Model urdf_model_;
  bool urdf_ok_;
  std::string full_path_;
  collision_space::EnvironmentModelODE* coll_space_;
  planning_models::KinematicModel* kinematic_model_;
};


TEST_F(TestCollisionSpace, TestInit) {
  std::vector<std::string> links;
  kinematic_model_->getLinkModelNames(links);
  std::map<std::string, double> link_padding_map;

  {
    collision_space::EnvironmentModel::AllowedCollisionMatrix acm(links,true);
    
    coll_space_->setRobotModel(kinematic_model_, acm, link_padding_map);
    
    //all AllowedCollisions set to true, so no collision
    ASSERT_FALSE(coll_space_->isCollision());
  }

  {
    collision_space::EnvironmentModel::AllowedCollisionMatrix acm(links,false);

    coll_space_->setRobotModel(kinematic_model_, acm, link_padding_map);
  
    //now we are in collision with nothing disabled
    ASSERT_TRUE(coll_space_->isCollision());
  }

  //one more time for good measure
  {
    collision_space::EnvironmentModel::AllowedCollisionMatrix acm(links,false);

    coll_space_->setRobotModel(kinematic_model_, acm, link_padding_map);
  
    //now we are in collision with nothing disabled
    ASSERT_TRUE(coll_space_->isCollision());
  }
}


TEST_F(TestCollisionSpace, TestACM) {
  std::vector<std::string> links;
  kinematic_model_->getLinkModelNames(links);
  std::map<std::string, double> link_padding_map;
  
  //first we get
  {
    collision_space::EnvironmentModel::AllowedCollisionMatrix acm(links, false);
    coll_space_->setRobotModel(kinematic_model_, acm, link_padding_map);

    planning_models::KinematicState state(kinematic_model_);
    state.setKinematicStateToDefault();

    coll_space_->updateRobotModel(&state);

    //at default state in collision
    ASSERT_TRUE(coll_space_->isCollision());

    //now we get the full set of collisions in the default state
    std::vector<collision_space::EnvironmentModel::Contact> contacts;
      
    coll_space_->getAllCollisionContacts(contacts, 1);

    ASSERT_TRUE(contacts.size() > 1);
    //now we change all these pairwise to true
    for(unsigned int i = 0; i < contacts.size(); i++) {
      ASSERT_TRUE(contacts[i].body_type_1 == collision_space::EnvironmentModel::LINK);
      ASSERT_TRUE(contacts[i].body_type_2 == collision_space::EnvironmentModel::LINK);
      ASSERT_TRUE(acm.changeEntry(contacts[i].body_name_1,contacts[i].body_name_2, true));
    }

    coll_space_->setAlteredCollisionMatrix(acm);
    
    //with all of these disabled, no more collisions
    ASSERT_FALSE(coll_space_->isCollision());

    coll_space_->revertAlteredCollisionMatrix();
    ASSERT_TRUE(coll_space_->isCollision());
  }
}

TEST_F(TestCollisionSpace, TestAttachedObjects)
{
  std::vector<std::string> links;
  kinematic_model_->getLinkModelNames(links);
  std::map<std::string, double> link_padding_map;
  
  collision_space::EnvironmentModel::AllowedCollisionMatrix acm(links, false);
  coll_space_->setRobotModel(kinematic_model_, acm, link_padding_map);  

  {
    //indented cause the state needs to cease to exist before we add the attached body
    planning_models::KinematicState state(kinematic_model_);
    state.setKinematicStateToDefault();
    
    coll_space_->updateRobotModel(&state);

    //now we get the full set of collisions in the default state
    std::vector<collision_space::EnvironmentModel::Contact> contacts;
      
    coll_space_->getAllCollisionContacts(contacts, 1);

    //now we change all these pairwise to true
    for(unsigned int i = 0; i < contacts.size(); i++) {
      ASSERT_TRUE(contacts[i].body_type_1 == collision_space::EnvironmentModel::LINK);
      ASSERT_TRUE(contacts[i].body_type_2 == collision_space::EnvironmentModel::LINK);
      ASSERT_TRUE(acm.changeEntry(contacts[i].body_name_1,contacts[i].body_name_2, true));
    }

    coll_space_->setRobotModel(kinematic_model_, acm, link_padding_map);
    coll_space_->updateRobotModel(&state);
  }
  
  //now we shouldn't be in collision
  ASSERT_FALSE(coll_space_->isCollision());  

  const planning_models::KinematicModel::LinkModel *link = kinematic_model_->getLinkModel("base_link");

  //first a single box
  shapes::Sphere* sphere1 = new shapes::Sphere();
  sphere1->radius = .1;

  shapes::Box* box2 = new shapes::Box();
  box2->size[0] = .05;
  box2->size[1] = .05;
  box2->size[2] = .05;
  
  std::vector<shapes::Shape*> shape_vector;
  shape_vector.push_back(sphere1);

  tf::Transform pose;
  pose.setIdentity();

  std::vector<tf::Transform> poses;
  poses.push_back(pose);
  
  std::vector<std::string> touch_links;

  planning_models::KinematicModel::AttachedBodyModel* ab1 = 
    new planning_models::KinematicModel::AttachedBodyModel(link, "box_1",
                                                           poses,
                                                           touch_links,
                                                           shape_vector);

  kinematic_model_->addAttachedBodyModel(link->getName(), ab1);
  coll_space_->updateAttachedBodies();

  const collision_space::EnvironmentModel::AllowedCollisionMatrix& aft_attached 
    = coll_space_->getDefaultAllowedCollisionMatrix();
  
  ASSERT_TRUE(aft_attached.hasEntry("box_1"));
  bool allowed;
  EXPECT_TRUE(aft_attached.getAllowedCollision("box_1", link->getName(), allowed));
  EXPECT_FALSE(allowed);

  {
    //indented cause the state needs to cease to exist before we add the attached body
    planning_models::KinematicState state(kinematic_model_);
    state.setKinematicStateToDefault();
    
    coll_space_->updateRobotModel(&state);
  }

  //now it collides
  ASSERT_TRUE(coll_space_->isCollision());  

  kinematic_model_->clearLinkAttachedBodyModel(link->getName(), "box_1");
  coll_space_->updateAttachedBodies();

  ASSERT_FALSE(aft_attached.hasEntry("box_1"));

  //now adding an attached object with two boxes, this time with two objects
  shape_vector.clear();
  shape_vector.push_back(box2);
  pose.getOrigin().setX(.1);
  poses.clear();
  poses.push_back(pose);
  touch_links.push_back("r_gripper_palm_link");
  touch_links.push_back("r_gripper_r_finger_link");
  touch_links.push_back("r_gripper_l_finger_link");
  touch_links.push_back("r_gripper_r_finger_tip_link");
  touch_links.push_back("r_gripper_l_finger_tip_link");
  touch_links.push_back("base_link");
  
  planning_models::KinematicModel::AttachedBodyModel* ab2 = 
    new planning_models::KinematicModel::AttachedBodyModel(link, "box_2",
                                                           poses,
                                                           touch_links,
                                                           shape_vector);
  kinematic_model_->addAttachedBodyModel(link->getName(), ab2);
  coll_space_->updateAttachedBodies();

  ASSERT_TRUE(aft_attached.hasEntry("box_2"));
  EXPECT_TRUE(aft_attached.getAllowedCollision("box_2", link->getName(), allowed));
  EXPECT_TRUE(allowed);

  {
    //indented cause the state needs to cease to exist before we add the attached body
    planning_models::KinematicState state(kinematic_model_);
    state.setKinematicStateToDefault();
    
    coll_space_->updateRobotModel(&state);
  }

  //now it doesn't collide
  ASSERT_FALSE(coll_space_->isCollision());  
}

TEST_F(TestCollisionSpace, TestStaticObjects)
{
  std::vector<std::string> links;
  kinematic_model_->getLinkModelNames(links);
  std::map<std::string, double> link_padding_map;
  
  collision_space::EnvironmentModel::AllowedCollisionMatrix acm(links, false);
  coll_space_->setRobotModel(kinematic_model_, acm, link_padding_map);  
  
  {
    //indented cause the state needs to cease to exist before we add the attached body
    planning_models::KinematicState state(kinematic_model_);
    state.setKinematicStateToDefault();
    
    coll_space_->updateRobotModel(&state);
  }

  ASSERT_FALSE(coll_space_->isEnvironmentCollision());

  shapes::Sphere* sphere1 = new shapes::Sphere();
  sphere1->radius = .2;

  tf::Transform pose;
  pose.setIdentity();

  std::vector<tf::Transform> poses;
  poses.push_back(pose);

  std::vector<shapes::Shape*> shape_vector;
  shape_vector.push_back(sphere1);

  coll_space_->addObjects("obj1", shape_vector, poses);

  ASSERT_TRUE(coll_space_->isEnvironmentCollision());



  //Now test interactions between static and attached objects

  const planning_models::KinematicModel::LinkModel *link = kinematic_model_->getLinkModel("base_link");

  shapes::Box* att_box = new shapes::Box();
  att_box->size[0] = .05;
  att_box->size[1] = .05;
  att_box->size[2] = .05;
  
  std::vector<shapes::Shape*> att_shapes;
  att_shapes.push_back(att_box);

  tf::Transform att_pose;
  att_pose.setIdentity();

  std::vector<tf::Transform> att_poses;
  att_poses.push_back(att_pose);
  
  std::vector<std::string> touch_links;
  touch_links.push_back("base_link");
  touch_links.push_back("base_footprint");

  planning_models::KinematicModel::AttachedBodyModel* ab1 = 
    new planning_models::KinematicModel::AttachedBodyModel(link, "att1",
                                                           att_poses,
                                                           touch_links,
                                                           att_shapes);

  kinematic_model_->addAttachedBodyModel(link->getName(), ab1);
  coll_space_->updateAttachedBodies();

  {
    //indented cause the state needs to cease to exist before we add the attached body
    planning_models::KinematicState state(kinematic_model_);
    state.setKinematicStateToDefault();
    
    coll_space_->updateRobotModel(&state);
  }

  ASSERT_TRUE(coll_space_->isEnvironmentCollision());

  //now we get the full set of collisions in the default state
  std::vector<collision_space::EnvironmentModel::Contact> contacts;
  
  coll_space_->getAllCollisionContacts(contacts, 1);

  //now we change all these pairwise to true
  for(unsigned int i = 0; i < contacts.size(); i++) {
    if(contacts[i].body_type_1 == collision_space::EnvironmentModel::OBJECT) {
      ASSERT_TRUE(contacts[i].body_name_1 == "obj1");
    }
    if(contacts[i].body_type_2 == collision_space::EnvironmentModel::OBJECT) {
      ASSERT_TRUE(contacts[i].body_name_2 == "obj1");
    }
  }

  acm = coll_space_->getDefaultAllowedCollisionMatrix();
  bool allowed;
  ASSERT_TRUE(acm.getAllowedCollision("obj1","att1",allowed));
  EXPECT_FALSE(allowed);
  
  ASSERT_TRUE(acm.changeEntry(link->getName(), "obj1", true));
  ASSERT_TRUE(acm.changeEntry("base_footprint", "obj1", true));
  coll_space_->setAlteredCollisionMatrix(acm);

  {
    //indented cause the state needs to cease to exist before we add the attached body
    planning_models::KinematicState state(kinematic_model_);
    state.setKinematicStateToDefault();
    
    coll_space_->updateRobotModel(&state);
  }

  EXPECT_TRUE(coll_space_->isEnvironmentCollision());
  
  ASSERT_TRUE(acm.changeEntry("att1", "obj1", true));
  coll_space_->setAlteredCollisionMatrix(acm);
  
  allowed = false;
  ASSERT_TRUE(coll_space_->getCurrentAllowedCollisionMatrix().getAllowedCollision("att1","obj1", allowed));
  EXPECT_TRUE(allowed);

  ASSERT_TRUE(coll_space_->getCurrentAllowedCollisionMatrix().getAllowedCollision("obj1","att1", allowed));
  EXPECT_TRUE(allowed);

  ASSERT_TRUE(coll_space_->getCurrentAllowedCollisionMatrix().getAllowedCollision("base_link","obj1", allowed));
  EXPECT_TRUE(allowed);

  ASSERT_TRUE(coll_space_->getCurrentAllowedCollisionMatrix().getAllowedCollision("obj1","base_link", allowed));
  EXPECT_TRUE(allowed);

  ASSERT_TRUE(coll_space_->getCurrentAllowedCollisionMatrix().getAllowedCollision("obj1","base_footprint", allowed));
  EXPECT_TRUE(allowed);
  
  EXPECT_FALSE(coll_space_->isEnvironmentCollision());
  contacts.clear();

  coll_space_->getAllCollisionContacts(contacts, 1);

  //now we change all these pairwise to true
  for(unsigned int i = 0; i < contacts.size(); i++) {
    if(contacts[i].body_type_1 == collision_space::EnvironmentModel::OBJECT) {
      ASSERT_TRUE(contacts[i].body_name_1 == "obj1");
      ROS_INFO_STREAM(contacts[i].body_name_2);
    }
    if(contacts[i].body_type_2 == collision_space::EnvironmentModel::OBJECT) {
      ASSERT_TRUE(contacts[i].body_name_2 == "obj1");
      ROS_INFO_STREAM(contacts[i].body_name_1);
    }
  }

}

TEST_F(TestCollisionSpace, TestAllowedContacts)
{
  std::vector<std::string> links;
  kinematic_model_->getLinkModelNames(links);
  std::map<std::string, double> link_padding_map;
  
  collision_space::EnvironmentModel::AllowedCollisionMatrix acm(links, true);
  coll_space_->setRobotModel(kinematic_model_, acm, link_padding_map);  
  
  {
    //indented cause the state needs to cease to exist before we add the attached body
    planning_models::KinematicState state(kinematic_model_);
    state.setKinematicStateToDefault();
    
    coll_space_->updateRobotModel(&state);
  }

  ASSERT_FALSE(coll_space_->isEnvironmentCollision());

  shapes::Sphere* sphere1 = new shapes::Sphere(.2);
  shapes::Box* box1 = new shapes::Box(.4, .4, .4);
  shapes::Box* box1a = new shapes::Box(.4, .4, .4);

  tf::Transform pose;
  pose.setIdentity();

  std::vector<tf::Transform> poses;
  poses.push_back(pose);

  std::vector<shapes::Shape*> shape_vector_1;
  shape_vector_1.push_back(sphere1);

  coll_space_->addObjects("obj1", shape_vector_1, poses);

  std::vector<shapes::Shape*> shape_vector_2;
  shape_vector_2.push_back(box1);
  shape_vector_2.push_back(box1a);
  pose.getOrigin().setX(.25);
  poses.push_back(pose);

  coll_space_->addObjects("obj2", shape_vector_2, poses);

  ASSERT_TRUE(coll_space_->isEnvironmentCollision());

  std::vector<collision_space::EnvironmentModel::Contact> contacts;
  ASSERT_TRUE(coll_space_->getAllCollisionContacts(contacts));

  std::vector<collision_space::EnvironmentModel::AllowedContact> allowed;

  for(unsigned int i = 0; i < contacts.size(); i++) {
    //now we place an allowed contact region around this sphere for each contact

    if(contacts[i].body_name_1 == "obj1" || contacts[i].body_name_2 == "obj1") {

      shapes::Sphere* sphere2 = new shapes::Sphere(.3);

      tf::Transform trans(tf::Quaternion(0,0,0,1.0), contacts[i].pos);
      
      boost::shared_ptr<bodies::Sphere> bodysp(new bodies::Sphere(sphere2));
      bodysp->setPose(trans);

      delete sphere2;

      collision_space::EnvironmentModel::AllowedContact allc;

      allc.bound = bodysp;
      allc.body_name_1 = contacts[i].body_name_1;
      allc.body_name_2 = contacts[i].body_name_2;
      allc.depth = 100.0;//contacts[i].depth;

      allowed.push_back(allc);
    }
    if(contacts[i].body_name_1 == "obj2" || contacts[i].body_name_2 == "obj2") {

      shapes::Box* box2 = new shapes::Box(.01, .01, .01);

      tf::Transform trans(tf::Quaternion(0,0,0,1.0), contacts[i].pos);

      ROS_DEBUG_STREAM("Making allowed contact for " << contacts[i].body_name_1 
                       << " and " << contacts[i].body_name_2 
                       << " at " << contacts[i].pos.x() << " " 
                       << contacts[i].pos.y() << " " << contacts[i].pos.z());      

      
      boost::shared_ptr<bodies::Box> bodysp(new bodies::Box(box2));
      bodysp->setPose(trans);

      delete box2;

      collision_space::EnvironmentModel::AllowedContact allc;

      allc.bound = bodysp;
      allc.body_name_1 = contacts[i].body_name_1;
      allc.body_name_2 = contacts[i].body_name_2;
      allc.depth = 100.0;//contacts[i].depth;

      allowed.push_back(allc);
    }
  }
  coll_space_->setAllowedContacts(allowed);

  ASSERT_FALSE(coll_space_->isEnvironmentCollision());

  coll_space_->clearAllowedContacts();
}

TEST_F(TestCollisionSpace, TestThreading)
{
  boost::thread thread1(boost::bind(&TestCollisionSpace::spinThread, this));
  boost::thread thread2(boost::bind(&TestCollisionSpace::spinThread, this));
  boost::thread thread3(boost::bind(&TestCollisionSpace::spinThread, this));
  boost::thread thread4(boost::bind(&TestCollisionSpace::spinThread, this));
  thread1.join();
  thread2.join();
  thread3.join();
  thread4.join();
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

