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

#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
#include <gtest/gtest.h>
#include <sstream>
#include <ctype.h>

static bool sameStringIgnoringWS(const std::string &s1, const std::string &s2)
{
    unsigned int i1 = 0;
    unsigned int i2 = 0;
    while (i1 < s1.size() && isspace(s1[i1])) i1++;
    while (i2 < s2.size() && isspace(s2[i2])) i2++;
    while (i1 < s1.size() && i2 < s2.size())
    {
	if (i1 < s1.size() && i2 < s2.size())
	{
	    if (s1[i1] != s2[i2])
		return false;
	    i1++;
	    i2++;
	}
	while (i1 < s1.size() && isspace(s1[i1])) i1++;
	while (i2 < s2.size() && isspace(s2[i2])) i2++;
    }
    return i1 == s1.size() && i2 == s2.size();
}

TEST(Loading, SimpleRobot)
{
    static const std::string MODEL0 = 
	"<?xml version=\"1.0\" ?>" 
	"<robot name=\"myrobot\">" 
	"<link name=\"base_link\">"
	"  <collision>"
        "    <origin rpy=\"0 0 0\" xyz=\"0 0 0.165\"/>"
        "    <geometry>"
	"      <box size=\"0.65 0.65 0.23\"/>"
        "    </geometry>"
	"  </collision>"
	"</link>"
	"</robot>";

    urdf::Model urdfModel;
    urdfModel.initString(MODEL0);
    
    std::map < std::string, std::vector<std::string> > groups;
    planning_models::KinematicModel *model = new planning_models::KinematicModel(urdfModel, groups, planning_models::KinematicModel::CONNECT_XY_YAW);
    
    EXPECT_EQ(std::string("myrobot"), model->getName());
    EXPECT_EQ((unsigned int)3, model->getDimension());
    
    std::vector<const planning_models::KinematicModel::Link*> links;
    model->getLinks(links);
    EXPECT_EQ((unsigned int)1, links.size());
    
    std::vector<const planning_models::KinematicModel::Joint*> joints;
    model->getJoints(joints);
    EXPECT_EQ((unsigned int)1, joints.size());
    
    std::vector<std::string> pgroups;
    model->getGroupNames(pgroups);    
    EXPECT_EQ((unsigned int)0, pgroups.size());
    
    delete model;
}

TEST(LoadingAndFK, SimpleRobot)
{   
    static const std::string MODEL1 = 
	"<?xml version=\"1.0\" ?>" 
	"<robot name=\"one_robot\">"
	"<link name=\"base_link\">"
	"  <inertial>"
	"    <mass value=\"2.81\"/>"
	"    <origin rpy=\"0 0 0\" xyz=\"0.0 0.099 .0\"/>"
	"    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
	"  </inertial>"
	"  <collision>"
	"    <origin rpy=\"0 0 -1\" xyz=\"-0.1 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </collision>"
	"  <visual>"
	"    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </visual>"
	"</link>"
	"</robot>";
    
    static const std::string MODEL1_INFO = 
	"Model one_robot connecting to the world using connection type 1\n"
	"Complete model state dimension = 3\n"
	"State bounds: [0.00000, 0.00000] [0.00000, 0.00000] [-3.14159, 3.14159] \n"
	"Available groups: base \n"
	"Group base has 1 roots: world \n"
	"The state components for this group are: 0 1 2 \n";
    
    urdf::Model urdfModel;
    urdfModel.initString(MODEL1);
    
    std::map < std::string, std::vector<std::string> > groups;
    groups["base"].push_back("world");

    planning_models::KinematicModel *model = new planning_models::KinematicModel(urdfModel, groups, planning_models::KinematicModel::CONNECT_XY_YAW);
        
    EXPECT_EQ((unsigned int)3, model->getDimension());
    
    model->defaultState();
    
    std::vector<const planning_models::KinematicModel::Joint*> joints;
    model->getJoints(joints);
    EXPECT_EQ((unsigned int)1, joints.size());
    if (joints.size() > 0)
	EXPECT_EQ((unsigned int)3, joints[0]->used_params);

    EXPECT_TRUE(dynamic_cast<planning_models::KinematicModel::WorldJoint*>(model->getRoot()) != NULL);

    std::stringstream ssi;
    model->printModelInfo(ssi);
    EXPECT_TRUE(sameStringIgnoringWS(MODEL1_INFO, ssi.str()));
    
    double param[3] = { 10, 8, 0 };
    model->computeTransforms(param);
    if (model->hasLink("base_link"))
    {
	EXPECT_NEAR(9.9, model->getLink("base_link")->global_collision_body_transform.getOrigin().x(), 1e-5);
	EXPECT_NEAR(8.0, model->getLink("base_link")->global_collision_body_transform.getOrigin().y(), 1e-5);
	EXPECT_NEAR(0.0, model->getLink("base_link")->global_collision_body_transform.getOrigin().z(), 1e-5);
	EXPECT_NEAR(0.0, model->getLink("base_link")->global_collision_body_transform.getRotation().x(), 1e-5);
	EXPECT_NEAR(0.0, model->getLink("base_link")->global_collision_body_transform.getRotation().y(), 1e-5);
	EXPECT_NEAR(-0.479426, model->getLink("base_link")->global_collision_body_transform.getRotation().z(), 1e-5);
	EXPECT_NEAR(0.877583, model->getLink("base_link")->global_collision_body_transform.getRotation().w(), 1e-5);
    }
    
    delete model;    
}


TEST(FK, OneRobot)
{
    static const std::string MODEL2 = 
	"<?xml version=\"1.0\" ?>" 
	"<robot name=\"one_robot\">"
	"<link name=\"base_link\">"
	"  <inertial>"
	"    <mass value=\"2.81\"/>"
	"    <origin rpy=\"0 0 0\" xyz=\"0.0 0.0 .0\"/>"
	"    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
	"  </inertial>"
	"  <collision>"
	"    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </collision>"
	"  <visual>"
	"    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </visual>"
	"</link>"
	"<joint name=\"joint_a\" type=\"continuous\">"
	"   <axis xyz=\"0 0 1\"/>"
	"   <parent link=\"base_link\"/>"
	"   <child link=\"link_a\"/>"
	"   <origin rpy=\" 0.0 0 0 \" xyz=\"0.0 0 0 \"/>"
	"</joint>"
	"<link name=\"link_a\">"
	"  <inertial>"
	"    <mass value=\"1.0\"/>"
	"    <origin rpy=\"0 0 0\" xyz=\"0.0 0.0 .0\"/>"
	"    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
	"  </inertial>"
	"  <collision>"
	"    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </collision>"
	"  <visual>"
	"    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </visual>"
	"</link>"
	"<joint name=\"joint_b\" type=\"fixed\">"
	"  <parent link=\"link_a\"/>"
	"  <child link=\"link_b\"/>"
	"  <origin rpy=\" 0.0 -0.42 0 \" xyz=\"0.0 0.5 0 \"/>"
	"</joint>"
	"<link name=\"link_b\">"
	"  <inertial>"
	"    <mass value=\"1.0\"/>"
	"    <origin rpy=\"0 0 0\" xyz=\"0.0 0.0 .0\"/>"
	"    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
	"  </inertial>"
	"  <collision>"
	"    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </collision>"
	"  <visual>"
	"    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </visual>"
	"</link>"
	"  <joint name=\"joint_c\" type=\"prismatic\">"
	"    <axis xyz=\"1 0 0\"/>"
	"    <limit effort=\"100.0\" lower=\"0.0\" upper=\"0.09\" velocity=\"0.2\"/>"
	"    <safety_controller k_position=\"20.0\" k_velocity=\"500.0\" soft_lower_limit=\"0.0\" soft_upper_limit=\"0.089\"/>"
	"    <parent link=\"link_b\"/>"
	"    <child link=\"link_c\"/>"
	"    <origin rpy=\" 0.0 0.42 0.0 \" xyz=\"0.0 -0.1 0 \"/>"
	"  </joint>"
	"<link name=\"link_c\">"
	"  <inertial>"
	"    <mass value=\"1.0\"/>"
	"    <origin rpy=\"0 0 0\" xyz=\"0.0 0 .0\"/>"
	"    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
	"  </inertial>"
	"  <collision>"
	"    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </collision>"
	"  <visual>"
	"    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
	"    <geometry>"
	"      <box size=\"1 2 1\" />"
	"    </geometry>"
	"  </visual>"
	"</link>"
	"</robot>";
    
    static const std::string MODEL2_INFO = 
	"Model one_robot connecting to the world using connection type 1\n"
	"Complete model state dimension = 5\n"
	"State bounds: [0.00000, 0.00000] [0.00000, 0.00000] [-3.14159, 3.14159] [-3.14159, 3.14159] [0.00000, 0.08900] \n"
	"Available groups: base \n"
	"Group base has 1 roots: world \n"
	"The state components for this group are: 0 1 2 3 4 \n";

    urdf::Model urdfModel;
    urdfModel.initString(MODEL2);
    
    std::map < std::string, std::vector<std::string> > groups;
    groups["base"].push_back("world");
    groups["base"].push_back("joint_a");
    groups["base"].push_back("joint_b");
    groups["base"].push_back("joint_c");
        
    planning_models::KinematicModelPtr model(new planning_models::KinematicModel(urdfModel, groups,  planning_models::KinematicModel::CONNECT_XY_YAW));
    
    
    EXPECT_EQ((unsigned int)5, model->getDimension());

    double param[5] = { 1, 1, 0.5, -0.5, 0.1 };
    model->getGroup("base")->computeTransforms(param);
    
    std::stringstream ss1;
    model->printModelInfo(ss1);
    EXPECT_TRUE(sameStringIgnoringWS(MODEL2_INFO, ss1.str()));

    // make sure applying the state works for the entire robot
    model->printTransforms(ss1);
    
    model->computeTransforms(param);
    
    std::stringstream ss2;
    model->printModelInfo(ss2);

    // make sure copying of data is accurate
    planning_models::KinematicModel tempModel = *model;
    tempModel.printTransforms(ss2);
    
    EXPECT_EQ(ss1.str(), ss2.str());
    
    EXPECT_NEAR(1.0, model->getLink("base_link")->global_collision_body_transform.getOrigin().x(), 1e-5);
    EXPECT_NEAR(1.0, model->getLink("base_link")->global_collision_body_transform.getOrigin().y(), 1e-5);
    EXPECT_NEAR(0.0, model->getLink("base_link")->global_collision_body_transform.getOrigin().z(), 1e-5);
    EXPECT_NEAR(0.0, model->getLink("base_link")->global_collision_body_transform.getRotation().x(), 1e-5);
    EXPECT_NEAR(0.0, model->getLink("base_link")->global_collision_body_transform.getRotation().y(), 1e-5);
    EXPECT_NEAR(0.247404, model->getLink("base_link")->global_collision_body_transform.getRotation().z(), 1e-5);
    EXPECT_NEAR(0.968912, model->getLink("base_link")->global_collision_body_transform.getRotation().w(), 1e-5);

    EXPECT_NEAR(1.0, model->getLink("link_a")->global_collision_body_transform.getOrigin().x(), 1e-5);
    EXPECT_NEAR(1.0, model->getLink("link_a")->global_collision_body_transform.getOrigin().y(), 1e-5);
    EXPECT_NEAR(0.0, model->getLink("link_a")->global_collision_body_transform.getOrigin().z(), 1e-5);
    EXPECT_NEAR(0.0, model->getLink("link_a")->global_collision_body_transform.getRotation().x(), 1e-5);
    EXPECT_NEAR(0.0, model->getLink("link_a")->global_collision_body_transform.getRotation().y(), 1e-5);
    EXPECT_NEAR(0.0, model->getLink("link_a")->global_collision_body_transform.getRotation().z(), 1e-5);
    EXPECT_NEAR(1.0, model->getLink("link_a")->global_collision_body_transform.getRotation().w(), 1e-5);

    EXPECT_NEAR(1.0, model->getLink("link_b")->global_collision_body_transform.getOrigin().x(), 1e-5);
    EXPECT_NEAR(1.5, model->getLink("link_b")->global_collision_body_transform.getOrigin().y(), 1e-5);
    EXPECT_NEAR(0.0, model->getLink("link_b")->global_collision_body_transform.getOrigin().z(), 1e-5);
    EXPECT_NEAR(0.0, model->getLink("link_b")->global_collision_body_transform.getRotation().x(), 1e-5);
    EXPECT_NEAR(-0.20846, model->getLink("link_b")->global_collision_body_transform.getRotation().y(), 1e-5);
    EXPECT_NEAR(0.0, model->getLink("link_b")->global_collision_body_transform.getRotation().z(), 1e-5);
    EXPECT_NEAR(0.978031, model->getLink("link_b")->global_collision_body_transform.getRotation().w(), 1e-5);

    EXPECT_NEAR(1.1, model->getLink("link_c")->global_collision_body_transform.getOrigin().x(), 1e-5);
    EXPECT_NEAR(1.4, model->getLink("link_c")->global_collision_body_transform.getOrigin().y(), 1e-5);
    EXPECT_NEAR(0.0, model->getLink("link_c")->global_collision_body_transform.getOrigin().z(), 1e-5);
    EXPECT_NEAR(0.0, model->getLink("link_c")->global_collision_body_transform.getRotation().x(), 1e-5);
    EXPECT_NEAR(0.0, model->getLink("link_c")->global_collision_body_transform.getRotation().y(), 1e-5);
    EXPECT_NEAR(0.0, model->getLink("link_c")->global_collision_body_transform.getRotation().z(), 1e-5);
    EXPECT_NEAR(1.0, model->getLink("link_c")->global_collision_body_transform.getRotation().w(), 1e-5);

    planning_models::KinematicState *sp = new planning_models::KinematicState(model);
    EXPECT_FALSE(sp->seenAll());

    double tmpParam[3];
    tmpParam[0] = 0.1;
    sp->setParamsJoint(tmpParam, "joint_a");
    EXPECT_FALSE(sp->seenAll());
    EXPECT_TRUE(sp->seenJoint("joint_a"));

    tmpParam[0] = -1.0;    
    sp->setParamsJoint(tmpParam, "joint_c");
    EXPECT_FALSE(sp->seenAll());
    
    tmpParam[0] = 0.5; tmpParam[1] = 0.4; tmpParam[2] = 1.1;
    sp->setParamsJoint(tmpParam, "world");
    EXPECT_TRUE(sp->seenAll());
    
    EXPECT_EQ(0.5, sp->getParams()[0]);
    EXPECT_EQ(0.4, sp->getParams()[1]);
    EXPECT_EQ(1.1, sp->getParams()[2]);
    EXPECT_EQ(0.1, sp->getParams()[3]);
    EXPECT_EQ(-1.0, sp->getParams()[4]);
    
    planning_models::KinematicState sp_copy = *sp;
    EXPECT_TRUE(sp_copy == *sp);

    planning_models::KinematicModel m_copy = *model;
    EXPECT_TRUE(m_copy == *model);

    delete sp;
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
