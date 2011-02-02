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
      "  <link name=\"base_link\">"
      "    <collision name=\"base_collision\">"
      "    <origin rpy=\"0 0 0\" xyz=\"0 0 0.165\"/>"
      "    <geometry name=\"base_collision_geom\">"
      "      <box size=\"0.65 0.65 0.23\"/>"
      "    </geometry>"
      "    </collision>"
      "   </link>"
      "</robot>";

    std::vector<planning_models::KinematicModel::MultiDofConfig> multi_dof_configs;
    planning_models::KinematicModel::MultiDofConfig config("base_joint");
    config.type = "Planar";
    config.parent_frame_id = "odom_combined";
    config.child_frame_id = "base_link";
    multi_dof_configs.push_back(config);

    urdf::Model urdfModel;
    urdfModel.initString(MODEL0);
    
    std::map < std::string, std::vector<std::string> > groups;
    boost::shared_ptr<planning_models::KinematicModel> model(new planning_models::KinematicModel(urdfModel,groups,multi_dof_configs));
    
    planning_models::KinematicState state(model);

    EXPECT_EQ(std::string("myrobot"), model->getName());
    EXPECT_EQ((unsigned int)3, state.getDimension());

    const std::vector<planning_models::KinematicModel::LinkModel*>& links = model->getLinkModels();
    EXPECT_EQ((unsigned int)1, links.size());
    
    const std::vector<planning_models::KinematicModel::JointModel*>& joints = model->getJointModels();
    EXPECT_EQ((unsigned int)1, joints.size());
    
    std::vector<std::string> pgroups;
    model->getModelGroupNames(pgroups);    
    EXPECT_EQ((unsigned int)0, pgroups.size());    

}

TEST(LoadingAndFK, SimpleRobot)
{   
    static const std::string MODEL1 = 
	"<?xml version=\"1.0\" ?>" 
      "<robot name=\"myrobot\">" 
	"<link name=\"base_link\">"
	"  <inertial>"
	"    <mass value=\"2.81\"/>"
	"    <origin rpy=\"0 0 0\" xyz=\"0.0 0.099 .0\"/>"
	"    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
	"  </inertial>"
	"  <collision name=\"my_collision\">"
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
      "Complete model state dimension = 3\n"
      "State bounds: [-3.14159, 3.14159] [-DBL_MAX, DBL_MAX] [-DBL_MAX, DBL_MAX]\n"
      "Root joint : base_joint \n"
      "Available groups: base \n"
      "Group base has 1 roots: base_joint \n";

    std::vector<planning_models::KinematicModel::MultiDofConfig> multi_dof_configs;
    planning_models::KinematicModel::MultiDofConfig config("base_joint");
    config.type = "Planar";
    config.parent_frame_id = "odom_combined";
    config.child_frame_id = "base_link";
    multi_dof_configs.push_back(config);
    
    urdf::Model urdfModel;
    urdfModel.initString(MODEL1);
    
    std::map < std::string, std::vector<std::string> > groups;
    groups["base"].push_back("base_joint");

    boost::shared_ptr<planning_models::KinematicModel> model(new planning_models::KinematicModel(urdfModel,groups,multi_dof_configs));
    
    planning_models::KinematicState state(model);

    EXPECT_EQ((unsigned int)3, state.getDimension());
    
    state.setKinematicStateToDefault();
    
    const std::vector<planning_models::KinematicState::JointState*>& joint_states = state.getJointStateVector();
    EXPECT_EQ((unsigned int)1, joint_states.size());
    EXPECT_EQ((unsigned int)3, joint_states[0]->getJointStateValues().size());

    std::stringstream ssi;
    state.printStateInfo(ssi);
    EXPECT_TRUE(sameStringIgnoringWS(MODEL1_INFO, ssi.str())) << ssi.str();
        
    
    std::map<std::string, double> joint_values;
    joint_values["planar_x"]=10.0;
    joint_values["planar_y"]=8.0;
    joint_values["planar_th"]=0.0;
    state.setKinematicState(joint_values);

    EXPECT_NEAR(10.0, state.getLinkState("base_link")->getGlobalLinkTransform().getOrigin().x(), 1e-5);
    EXPECT_NEAR(8.0, state.getLinkState("base_link")->getGlobalLinkTransform().getOrigin().y(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("base_link")->getGlobalLinkTransform().getOrigin().z(), 1e-5);

    const std::map<std::string, unsigned int>& ind_map = state.getKinematicStateIndexMap();
    std::vector<double> jv(state.getDimension(), 0.0);
    jv[ind_map.at("planar_x")] = 10.0;
    jv[ind_map.at("planar_y")] = 8.0;
    jv[ind_map.at("planar_th")] = 0.0;

    state.setKinematicState(jv);
    EXPECT_NEAR(10.0, state.getLinkState("base_link")->getGlobalLinkTransform().getOrigin().x(), 1e-5);
    EXPECT_NEAR(8.0, state.getLinkState("base_link")->getGlobalLinkTransform().getOrigin().y(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("base_link")->getGlobalLinkTransform().getOrigin().z(), 1e-5);
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
      "  <collision name=\"my_collision\">"
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
      "Complete model state dimension = 5\n"
      "State bounds: [-3.14159, 3.14159] [-DBL_MAX, DBL_MAX] [-DBL_MAX, DBL_MAX] [-3.14159, 3.14159] [0.00000, 0.08900]\n"
      "Root joint : base_joint \n"
      "Available groups: base \n"
      "Group base has 1 roots: base_joint \n";

    urdf::Model urdfModel;
    urdfModel.initString(MODEL2);
    
    std::map < std::string, std::vector<std::string> > groups;
    groups["base"].push_back("base_joint");
    groups["base"].push_back("joint_a");
    groups["base"].push_back("joint_b");
    groups["base"].push_back("joint_c");

    std::vector<planning_models::KinematicModel::MultiDofConfig> multi_dof_configs;
    planning_models::KinematicModel::MultiDofConfig config("base_joint");
    config.type = "Planar";
    config.parent_frame_id = "odom_combined";
    config.child_frame_id = "base_link";
    multi_dof_configs.push_back(config);
    
    boost::shared_ptr<planning_models::KinematicModel> model(new planning_models::KinematicModel(urdfModel,groups,multi_dof_configs));
    
    planning_models::KinematicState state(model);

    EXPECT_EQ((unsigned int)5, state.getDimension());

    std::map<std::string, double> joint_values;
    joint_values["planar_x"]=1.0;
    joint_values["planar_y"]=1.0;
    joint_values["planar_th"]=0.5;
    joint_values["joint_a"] = -0.5;
    joint_values["joint_c"] = 0.1;
    state.getJointStateGroup("base")->setKinematicState(joint_values);

    //double param[5] = { 1, 1, 0.5, -0.5, 0.1 };
    //model->getGroup("base")->computeTransforms(param);
    
    std::stringstream ss1;
    state.printStateInfo(ss1);
    EXPECT_TRUE(sameStringIgnoringWS(MODEL2_INFO, ss1.str())) << ss1.str();
    state.printTransforms(ss1);

    //make sure it works the same way for the whole robot
    state.setKinematicState(joint_values);
    std::stringstream ss2;
    state.printStateInfo(ss2);
    state.printTransforms(ss2);
    
    EXPECT_EQ(ss1.str(), ss2.str());
    
    EXPECT_NEAR(1.0, state.getLinkState("base_link")->getGlobalLinkTransform().getOrigin().x(), 1e-5);
    EXPECT_NEAR(1.0, state.getLinkState("base_link")->getGlobalLinkTransform().getOrigin().y(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("base_link")->getGlobalLinkTransform().getOrigin().z(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("base_link")->getGlobalLinkTransform().getRotation().x(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("base_link")->getGlobalLinkTransform().getRotation().y(), 1e-5);
    EXPECT_NEAR(0.247404, state.getLinkState("base_link")->getGlobalLinkTransform().getRotation().z(), 1e-5);
    EXPECT_NEAR(0.968912, state.getLinkState("base_link")->getGlobalLinkTransform().getRotation().w(), 1e-5);

    EXPECT_NEAR(1.0, state.getLinkState("link_a")->getGlobalLinkTransform().getOrigin().x(), 1e-5);
    EXPECT_NEAR(1.0, state.getLinkState("link_a")->getGlobalLinkTransform().getOrigin().y(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("link_a")->getGlobalLinkTransform().getOrigin().z(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("link_a")->getGlobalLinkTransform().getRotation().x(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("link_a")->getGlobalLinkTransform().getRotation().y(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("link_a")->getGlobalLinkTransform().getRotation().z(), 1e-5);
    EXPECT_NEAR(1.0, state.getLinkState("link_a")->getGlobalLinkTransform().getRotation().w(), 1e-5);

    EXPECT_NEAR(1.0, state.getLinkState("link_b")->getGlobalLinkTransform().getOrigin().x(), 1e-5);
    EXPECT_NEAR(1.5, state.getLinkState("link_b")->getGlobalLinkTransform().getOrigin().y(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("link_b")->getGlobalLinkTransform().getOrigin().z(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("link_b")->getGlobalLinkTransform().getRotation().x(), 1e-5);
    EXPECT_NEAR(-0.20846, state.getLinkState("link_b")->getGlobalLinkTransform().getRotation().y(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("link_b")->getGlobalLinkTransform().getRotation().z(), 1e-5);
    EXPECT_NEAR(0.978031, state.getLinkState("link_b")->getGlobalLinkTransform().getRotation().w(), 1e-5);

    EXPECT_NEAR(1.1, state.getLinkState("link_c")->getGlobalLinkTransform().getOrigin().x(), 1e-5);
    EXPECT_NEAR(1.4, state.getLinkState("link_c")->getGlobalLinkTransform().getOrigin().y(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("link_c")->getGlobalLinkTransform().getOrigin().z(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("link_c")->getGlobalLinkTransform().getRotation().x(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("link_c")->getGlobalLinkTransform().getRotation().y(), 1e-5);
    EXPECT_NEAR(0.0, state.getLinkState("link_c")->getGlobalLinkTransform().getRotation().z(), 1e-5);
    EXPECT_NEAR(1.0, state.getLinkState("link_c")->getGlobalLinkTransform().getRotation().w(), 1e-5);

    //bonus bounds lookup test
    std::vector<std::string> jn;
    jn.push_back("planar_x");
    jn.push_back("planar_th");
    jn.push_back("base_joint");
    EXPECT_TRUE(state.areJointsWithinBounds(jn));

    jn.push_back("monkey");
    EXPECT_FALSE(state.areJointsWithinBounds(jn));
}
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
