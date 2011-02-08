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

//urdf location relative to the planning_models path
static const std::string rel_path = "/test_urdf/robot.xml";

class TestCollisionSpace : public testing::Test {
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
    std::vector<collision_space::EnvironmentModel::AllowedContact> ac;
    std::vector<collision_space::EnvironmentModel::Contact> contacts;
      
    coll_space_->getAllCollisionContacts(ac, contacts, 1);

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

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

