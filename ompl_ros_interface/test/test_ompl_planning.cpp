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
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <arm_navigation_msgs/GetMotionPlan.h>
#include <planning_environment/models/model_utils.h>
#include <planning_environment/models/collision_models_interface.h>
#include <actionlib/client/simple_action_client.h>

static const std::string SET_PLANNING_SCENE_DIFF_SERVICE="/environment_server/set_planning_scene_diff";
static const std::string PLANNER_SERVICE_NAME="/ompl_planning/plan_kinematic_path";

class OmplPlanningTest : public testing::Test {
protected:

  virtual void SetUp() {

    ready_ = false;
    done_ = false;

    cm_ = new planning_environment::CollisionModels("robot_description");

    ros::service::waitForService(SET_PLANNING_SCENE_DIFF_SERVICE);
    ros::service::waitForService(PLANNER_SERVICE_NAME);

    set_planning_scene_diff_client_ = nh_.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_SERVICE);
    planning_service_client_ = nh_.serviceClient<arm_navigation_msgs::GetMotionPlan>(PLANNER_SERVICE_NAME);

    mplan_req.motion_plan_request.group_name = "right_arm";
    mplan_req.motion_plan_request.num_planning_attempts = 1;
    mplan_req.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
    const std::vector<std::string>& joint_names = cm_->getKinematicModel()->getModelGroup("right_arm")->getJointModelNames();
    mplan_req.motion_plan_request.goal_constraints.joint_constraints.resize(joint_names.size());
    for(unsigned int i = 0; i < joint_names.size(); i++) {
      mplan_req.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = joint_names[i];
      mplan_req.motion_plan_request.goal_constraints.joint_constraints[i].position = 0.0;
      mplan_req.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.001;
      mplan_req.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.001;
    }      
  }

  virtual void TearDown() {
    delete cm_;
  }

  void GetAndSetPlanningScene() {
    ASSERT_TRUE(set_planning_scene_diff_client_.call(get_req, get_res));
  }
      
protected:

  ros::NodeHandle nh_;

  bool ready_, done_;

  planning_environment::CollisionModels* cm_;

  arm_navigation_msgs::SetPlanningSceneDiff::Request get_req;
  arm_navigation_msgs::SetPlanningSceneDiff::Response get_res;
  arm_navigation_msgs::GetMotionPlan::Request mplan_req;

  ros::ServiceClient set_planning_scene_diff_client_;
  ros::ServiceClient planning_service_client_;
};

TEST_F(OmplPlanningTest, TestPole)
{
  arm_navigation_msgs::CollisionObject pole;
  
  pole.header.stamp = ros::Time::now();
  pole.header.frame_id = "odom_combined";
  pole.id = "pole";
  pole.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  pole.shapes.resize(1);
  pole.shapes[0].type = arm_navigation_msgs::Shape::CYLINDER;
  pole.shapes[0].dimensions.resize(2);
  pole.shapes[0].dimensions[0] = 0.1;
  pole.shapes[0].dimensions[1] = 1.5;
  pole.poses.resize(1);
  pole.poses[0].position.x = .6;
  pole.poses[0].position.y = -.6;
  pole.poses[0].position.z = .75;
  pole.poses[0].orientation.w = 1.0;

  get_req.planning_scene_diff.collision_objects.push_back(pole);

  GetAndSetPlanningScene();

  mplan_req.motion_plan_request.goal_constraints.joint_constraints[0].position = -2.0;
  mplan_req.motion_plan_request.goal_constraints.joint_constraints[3].position = -.2;
  mplan_req.motion_plan_request.goal_constraints.joint_constraints[5].position = -.2;

  for(unsigned int i = 0; i < 10; i++) {
    arm_navigation_msgs::GetMotionPlan::Response mplan_res;
    ASSERT_TRUE(planning_service_client_.call(mplan_req, mplan_res));
    
    ASSERT_EQ(mplan_res.error_code.val,mplan_res.error_code.SUCCESS);
    
    EXPECT_GT(mplan_res.trajectory.joint_trajectory.points.size(), 0);
    
    arm_navigation_msgs::ArmNavigationErrorCodes error_code;
    std::vector<arm_navigation_msgs::ArmNavigationErrorCodes> trajectory_error_codes;
    
    EXPECT_TRUE(cm_->isJointTrajectoryValid(get_res.planning_scene,
                                            mplan_res.trajectory.joint_trajectory,
                                            mplan_req.motion_plan_request.goal_constraints,                              
                                            mplan_req.motion_plan_request.path_constraints,
                                            error_code,
                                            trajectory_error_codes, false));
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_ompl_planning");
    
  return RUN_ALL_TESTS();
}
