#include <kinematics_msgs/utils.h>
#include <gtest/gtest.h>

using namespace kinematics_msgs;

TEST(TestUtils,DefaultLinks)
{
  std::vector<std::string> collision_enable, collision_disable, default_links, result;
  default_links.push_back("wrist");
  default_links.push_back("elbow");
  default_links.push_back("shoulder");
  getCollisionLinks(default_links,collision_enable,collision_disable,result);
  EXPECT_TRUE((int)result.size() == 3);
  EXPECT_TRUE(result[0] == default_links[0]);
  EXPECT_TRUE(result[1] == default_links[1]);
  EXPECT_TRUE(result[2] == default_links[2]);
}

TEST(TestUtils,CollisionEnable)
{
  std::vector<std::string> collision_enable, collision_disable, default_links, result;

  default_links.push_back("wrist");
  default_links.push_back("elbow");
  default_links.push_back("shoulder");

  collision_enable.push_back("head");

  getCollisionLinks(default_links,collision_enable,collision_disable,result);
  EXPECT_TRUE((int)result.size() == 1);
  EXPECT_TRUE(result[0] == collision_enable[0]);
}

TEST(TestUtils,CollisionDisable)
{
  std::vector<std::string> collision_enable, collision_disable, default_links, result;

  default_links.push_back("wrist");
  default_links.push_back("elbow");
  default_links.push_back("shoulder");

  collision_disable.push_back("elbow");

  getCollisionLinks(default_links,collision_enable,collision_disable,result);
  EXPECT_TRUE((int)result.size() == 2);
  EXPECT_TRUE(result[0] == default_links[0] || result[0] == default_links[2]);
  EXPECT_TRUE(result[1] == default_links[0] || result[1] == default_links[2]);
}

TEST(TestUtils,CollisionEnableDisable)
{
  std::vector<std::string> collision_enable, collision_disable, default_links, result;

  default_links.push_back("wrist");
  default_links.push_back("elbow");
  default_links.push_back("shoulder");

  collision_disable.push_back("elbow");
  collision_disable.push_back("foot");

  collision_enable.push_back("gripper");
  collision_enable.push_back("head");
  collision_enable.push_back("wrist");

  getCollisionLinks(default_links,collision_enable,collision_disable,result);
  EXPECT_TRUE((int)result.size() == 4);

  for(unsigned int i=0; i < result.size(); i++)
  {
    EXPECT_TRUE(result[i] == "wrist" || result[i] == "shoulder" || result[i] == "gripper" || result[i] == "head");
  }
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
