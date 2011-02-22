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
#include <planning_environment_msgs/GetPlanningScene.h>
#include <motion_planning_msgs/GetMotionPlan.h>
#include <planning_environment_msgs/SetPlanningSceneAction.h>
#include <planning_environment/models/model_utils.h>
#include <planning_environment/models/collision_models_interface.h>
#include <actionlib/client/simple_action_client.h>

static const std::string GET_PLANNING_SCENE_SERVICE="/environment_server/get_planning_scene";
static const std::string SET_PLANNING_SCENE_NAME="/ompl_planning/set_planning_scene";
static const std::string PLANNER_SERVICE_NAME="/ompl_planning/plan_kinematic_path";

class OmplPlanningTest : public testing::Test {
public: 

  void actionFeedbackCallback(const planning_environment_msgs::SetPlanningSceneFeedbackConstPtr& feedback) {
    ready_ = true;  
  }

  void actionDoneCallback(const actionlib::SimpleClientGoalState& state,
                          const planning_environment_msgs::SetPlanningSceneResultConstPtr& result)
  {
    EXPECT_TRUE(state == actionlib::SimpleClientGoalState::PREEMPTED);
    done_ = true;
  }

protected:

  virtual void SetUp() {

    ready_ = false;
    done_ = false;

    cm_ = new planning_environment::CollisionModels("robot_description");

    ros::service::waitForService(GET_PLANNING_SCENE_SERVICE);
    ros::service::waitForService(PLANNER_SERVICE_NAME);

    get_planning_scene_client_ = nh_.serviceClient<planning_environment_msgs::GetPlanningScene>(GET_PLANNING_SCENE_SERVICE);
    planning_service_client_ = nh_.serviceClient<motion_planning_msgs::GetMotionPlan>(PLANNER_SERVICE_NAME);

    set_planning_scene_action_ = new actionlib::SimpleActionClient<planning_environment_msgs::SetPlanningSceneAction>(SET_PLANNING_SCENE_NAME, true);

    set_planning_scene_action_->waitForServer();

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
    delete set_planning_scene_action_;
  }

  void GetAndSetPlanningScene() {
    ASSERT_TRUE(get_planning_scene_client_.call(get_req, get_res));

    planning_environment_msgs::SetPlanningSceneGoal planning_scene_goal;
    planning_scene_goal.planning_scene = get_res.planning_scene;

    //set_planning_scene_action_->sendGoal(planning_scene_goal, boost::bind(&OmplPlanningTest::actionDoneCallback, this, _1, _2), NULL, boost::bind(&OmplPlanningTest::actionFeedbackCallback, this, _1));

    actionlib::SimpleClientGoalState gs = set_planning_scene_action_->sendGoalAndWait(planning_scene_goal);
    
    EXPECT_TRUE(gs == actionlib::SimpleClientGoalState::SUCCEEDED);
  }
      
protected:

  ros::NodeHandle nh_;

  bool ready_, done_;

  planning_environment::CollisionModels* cm_;

  planning_environment_msgs::GetPlanningScene::Request get_req;
  planning_environment_msgs::GetPlanningScene::Response get_res;
  motion_planning_msgs::GetMotionPlan::Request mplan_req;

  ros::ServiceClient get_planning_scene_client_;
  ros::ServiceClient planning_service_client_;

  actionlib::SimpleActionClient<planning_environment_msgs::SetPlanningSceneAction>* set_planning_scene_action_;  

};

TEST_F(OmplPlanningTest, TestPole)
{
  mapping_msgs::CollisionObject pole;
  
  pole.header.stamp = ros::Time::now();
  pole.header.frame_id = "odom_combined";
  pole.id = "pole";
  pole.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
  pole.shapes.resize(1);
  pole.shapes[0].type = geometric_shapes_msgs::Shape::CYLINDER;
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
    motion_planning_msgs::GetMotionPlan::Response mplan_res;
    ASSERT_TRUE(planning_service_client_.call(mplan_req, mplan_res));
    
    ASSERT_EQ(mplan_res.error_code.val,mplan_res.error_code.SUCCESS);
    
    EXPECT_GT(mplan_res.trajectory.joint_trajectory.points.size(), 0);
    
    motion_planning_msgs::ArmNavigationErrorCodes error_code;
    std::vector<motion_planning_msgs::ArmNavigationErrorCodes> trajectory_error_codes;
    
    EXPECT_TRUE(cm_->isTrajectoryValid(get_res.planning_scene,
                                       mplan_res.trajectory.joint_trajectory,
                                       get_res.transformed_goal_constraints,
                                       get_res.transformed_path_constraints,
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
