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
  };

  virtual void TearDown() {
    delete kinematic_model_;
  }

protected:

  urdf::Model urdf_model_;
  bool urdf_ok_;
  std::string full_path_;
  collision_space::EnvironmentModelODE coll_space_;
  planning_models::KinematicModel* kinematic_model_;
};

TEST_F(TestCollisionSpace, TestInit) {
  std::vector<std::string> links;
  std::map<std::string, double> link_padding_map;
  coll_space_.setRobotModel(kinematic_model_, links, link_padding_map);

  //no links, so no collision
  ASSERT_FALSE(coll_space_.isCollision());

  //all links, should be collision
  kinematic_model_->getLinkModelNames(links);

  coll_space_.setRobotModel(kinematic_model_, links, link_padding_map);
  
  //now we are in collision with no default collision operations
  ASSERT_TRUE(coll_space_.isCollision());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

