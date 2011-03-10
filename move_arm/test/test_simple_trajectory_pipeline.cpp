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
#include <motion_planning_msgs/FilterJointTrajectoryWithConstraints.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <motion_planning_msgs/convert_messages.h>
#include <ros/package.h>

static const std::string GET_PLANNING_SCENE_SERVICE="/environment_server/get_planning_scene";
static const std::string SET_PLANNING_SCENE_NAME_1="/ompl_planning/set_planning_scene";
static const std::string SET_PLANNING_SCENE_NAME_2="/trajectory_filter/set_planning_scene";
static const std::string SET_PLANNING_SCENE_NAME_3="/pr2_right_arm_kinematics/set_planning_scene";
static const std::string PLANNER_SERVICE_NAME="/ompl_planning/plan_kinematic_path";
static const std::string TRAJECTORY_FILTER_SERVICE_NAME="/trajectory_filter/filter_trajectory_with_constraints";
static const std::string IK_NAME="/pr2_right_arm_kinematics/get_constraint_aware_ik";

class TrajectoryPipelineTest : public testing::Test {
protected:

  virtual void SetUp() {

    cm_ = new planning_environment::CollisionModels("robot_description");
    planning_scene_state_ = NULL;

    ros::service::waitForService(GET_PLANNING_SCENE_SERVICE);
    ros::service::waitForService(PLANNER_SERVICE_NAME);
    ros::service::waitForService(TRAJECTORY_FILTER_SERVICE_NAME);
    ros::service::waitForService(IK_NAME);

    get_planning_scene_client_ = nh_.serviceClient<planning_environment_msgs::GetPlanningScene>(GET_PLANNING_SCENE_SERVICE);
    planning_service_client_ = nh_.serviceClient<motion_planning_msgs::GetMotionPlan>(PLANNER_SERVICE_NAME);
    trajectory_filter_client_ = nh_.serviceClient<motion_planning_msgs::FilterJointTrajectoryWithConstraints>(TRAJECTORY_FILTER_SERVICE_NAME);
    ik_service_client_ = nh_.serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>(IK_NAME);

    set_planning_scene_action_1_ = new actionlib::SimpleActionClient<planning_environment_msgs::SetPlanningSceneAction>(SET_PLANNING_SCENE_NAME_1, true);
    set_planning_scene_action_1_->waitForServer();

    set_planning_scene_action_2_ = new actionlib::SimpleActionClient<planning_environment_msgs::SetPlanningSceneAction>(SET_PLANNING_SCENE_NAME_2, true);
    set_planning_scene_action_2_->waitForServer();

    set_planning_scene_action_3_ = new actionlib::SimpleActionClient<planning_environment_msgs::SetPlanningSceneAction>(SET_PLANNING_SCENE_NAME_3, true);
    set_planning_scene_action_3_->waitForServer();

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
    if(planning_scene_state_ != NULL) {
      delete planning_scene_state_;
    }
    delete cm_;
    delete set_planning_scene_action_1_;
    delete set_planning_scene_action_2_;
    delete set_planning_scene_action_3_;
  }

  void GetAndSetPlanningScene() {
    ASSERT_TRUE(get_planning_scene_client_.call(get_req, get_res));

    planning_scene_state_ = cm_->setPlanningScene(get_res.planning_scene);

    planning_environment_msgs::SetPlanningSceneGoal planning_scene_goal;
    planning_scene_goal.planning_scene = get_res.planning_scene;

    //set_planning_scene_action_->sendGoal(planning_scene_goal, boost::bind(&OmplPlanningTest::actionDoneCallback, this, _1, _2), NULL, boost::bind(&OmplPlanningTest::actionFeedbackCallback, this, _1));
    set_planning_scene_action_1_->sendGoal(planning_scene_goal);
    set_planning_scene_action_2_->sendGoal(planning_scene_goal);
    set_planning_scene_action_3_->sendGoal(planning_scene_goal);

    set_planning_scene_action_1_->waitForResult();
    set_planning_scene_action_2_->waitForResult();
    set_planning_scene_action_3_->waitForResult();

    actionlib::SimpleClientGoalState gs = set_planning_scene_action_1_->getState();
    EXPECT_TRUE(gs == actionlib::SimpleClientGoalState::SUCCEEDED);

    gs = set_planning_scene_action_2_->getState();
    EXPECT_TRUE(gs == actionlib::SimpleClientGoalState::SUCCEEDED);

    gs = set_planning_scene_action_3_->getState();
    EXPECT_TRUE(gs == actionlib::SimpleClientGoalState::SUCCEEDED);

  }

protected:

  ros::NodeHandle nh_;

  planning_environment::CollisionModels* cm_;

  planning_models::KinematicState* planning_scene_state_;

  planning_environment_msgs::GetPlanningScene::Request get_req;
  planning_environment_msgs::GetPlanningScene::Response get_res;
  motion_planning_msgs::GetMotionPlan::Request mplan_req;

  ros::ServiceClient get_planning_scene_client_;
  ros::ServiceClient planning_service_client_;
  ros::ServiceClient trajectory_filter_client_;
  ros::ServiceClient ik_service_client_;

  actionlib::SimpleActionClient<planning_environment_msgs::SetPlanningSceneAction>* set_planning_scene_action_1_;  
  actionlib::SimpleActionClient<planning_environment_msgs::SetPlanningSceneAction>* set_planning_scene_action_2_;  
  actionlib::SimpleActionClient<planning_environment_msgs::SetPlanningSceneAction>* set_planning_scene_action_3_;  

};

TEST_F(TrajectoryPipelineTest, TestPole)
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

  cm_->writePlanningSceneBag(ros::package::getPath("move_arm")+"/test_pole.bag",
                             get_res.planning_scene);

  mplan_req.motion_plan_request.goal_constraints.joint_constraints[0].position = -2.0;
  mplan_req.motion_plan_request.goal_constraints.joint_constraints[3].position = -.2;
  mplan_req.motion_plan_request.goal_constraints.joint_constraints[5].position = -.2;

  for(unsigned int i = 0; i < 1; i++) {
    motion_planning_msgs::GetMotionPlan::Response mplan_res;
    ASSERT_TRUE(planning_service_client_.call(mplan_req, mplan_res));
    
    ASSERT_EQ(mplan_res.error_code.val,mplan_res.error_code.SUCCESS);
    
    EXPECT_GT(mplan_res.trajectory.joint_trajectory.points.size(), 0);
    
    motion_planning_msgs::ArmNavigationErrorCodes error_code;
    std::vector<motion_planning_msgs::ArmNavigationErrorCodes> trajectory_error_codes;
    
    EXPECT_TRUE(cm_->isJointTrajectoryValid(*planning_scene_state_, 
                                       mplan_res.trajectory.joint_trajectory,
                                       mplan_req.motion_plan_request.goal_constraints,
                                       mplan_req.motion_plan_request.path_constraints,
                                       error_code,
                                       trajectory_error_codes, false)) << error_code;

    planning_environment::setRobotStateAndComputeTransforms(get_res.planning_scene.robot_state, *planning_scene_state_);
    double planner_length = cm_->getTotalTrajectoryJointLength(*planning_scene_state_, mplan_res.trajectory.joint_trajectory);

    motion_planning_msgs::FilterJointTrajectoryWithConstraints::Request filter_req;
    motion_planning_msgs::FilterJointTrajectoryWithConstraints::Response filter_res;
    
    filter_req.trajectory = mplan_res.trajectory.joint_trajectory;
  
    filter_req.goal_constraints = mplan_req.motion_plan_request.goal_constraints;
    filter_req.path_constraints = mplan_req.motion_plan_request.path_constraints;
    filter_req.allowed_time = ros::Duration(1.0);

    EXPECT_TRUE(trajectory_filter_client_.call(filter_req, filter_res));

    EXPECT_TRUE(cm_->isJointTrajectoryValid(*planning_scene_state_, 
                                       filter_res.trajectory,
                                       mplan_req.motion_plan_request.goal_constraints,
                                       mplan_req.motion_plan_request.path_constraints,
                                       error_code,
                                       trajectory_error_codes, false)) << error_code;

    planning_environment::setRobotStateAndComputeTransforms(get_res.planning_scene.robot_state, *planning_scene_state_);
    double filter_length = cm_->getTotalTrajectoryJointLength(*planning_scene_state_, filter_res.trajectory);
  
    EXPECT_GE(planner_length, filter_length);

    ROS_INFO_STREAM("Planner points " << mplan_res.trajectory.joint_trajectory.points.size() << " filter points " << filter_res.trajectory.points.size() << " planner length " << planner_length << " filter length " << filter_length);
  }
  cm_->revertPlanningScene(planning_scene_state_);
}

TEST_F(TrajectoryPipelineTest, TestObjectTable)
{
  mapping_msgs::CollisionObject table;

  table.header.stamp = ros::Time::now();
  table.header.frame_id = "odom_combined";
  table.id = "table";
  table.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
  table.shapes.resize(1);
  table.shapes[0].type = geometric_shapes_msgs::Shape::BOX;
  table.shapes[0].dimensions.resize(3);
  table.shapes[0].dimensions[0] = .5;
  table.shapes[0].dimensions[1] = .5;
  table.shapes[0].dimensions[2] = .1;
  table.poses.resize(1);
  table.poses[0].position.x = 4.0;
  table.poses[0].position.y = -4.0;
  table.poses[0].position.z = .5;
  table.poses[0].orientation.w = 1.0;

  geometry_msgs::PoseStamped obj_pose;
  obj_pose.header.frame_id = "odom_combined";
  obj_pose.pose.position.x = 3.77;
  obj_pose.pose.position.y = -4.2;
  obj_pose.pose.position.z = .851;
  //obj_pose.header.frame_id = "base_link";
  //obj_pose.pose.position.x = .52;
  //obj_pose.pose.position.y = -.2;
  //obj_pose.pose.position.z = .8;
  obj_pose.pose.orientation.x = 0.0;
  obj_pose.pose.orientation.y = 0.7071;
  obj_pose.pose.orientation.z = 0.0;
  obj_pose.pose.orientation.w = 0.7071;

  motion_planning_msgs::Constraints goal_constraints;
  goal_constraints.position_constraints.resize(1);
  goal_constraints.orientation_constraints.resize(1);

  motion_planning_msgs::poseStampedToPositionOrientationConstraints(obj_pose,
                                                                    "r_wrist_roll_link",
                                                                    goal_constraints.position_constraints[0],
                                                                    goal_constraints.orientation_constraints[0]);
 
  get_req.planning_scene_diff.collision_objects.push_back(table);
  
  GetAndSetPlanningScene();

  motion_planning_msgs::RobotState rs;
  planning_environment::convertKinematicStateToRobotState(*planning_scene_state_, ros::Time::now(), cm_->getWorldFrameId(), rs);

  rs.multi_dof_joint_state.joint_names.resize(1);
  rs.multi_dof_joint_state.frame_ids.resize(1);  
  rs.multi_dof_joint_state.child_frame_ids.resize(1);
  rs.multi_dof_joint_state.stamp = ros::Time::now();
  rs.multi_dof_joint_state.joint_names[0] = "base_joint";
  rs.multi_dof_joint_state.frame_ids[0] = "odom_combined";
  rs.multi_dof_joint_state.child_frame_ids[0] = "base_footprint";
  rs.multi_dof_joint_state.poses.resize(1);

  //first pose - straight on

  //X is 0.57 y -0.2 z -0.097925

  rs.multi_dof_joint_state.poses[0].position.x = 3.25;
  rs.multi_dof_joint_state.poses[0].position.y = -4.0;
  rs.multi_dof_joint_state.poses[0].position.z = 0.0;
  rs.multi_dof_joint_state.poses[0].orientation.w = 1.0;
  
  planning_environment_msgs::PlanningScene temp_scene = get_res.planning_scene;
  temp_scene.robot_state = rs;

  cm_->writePlanningSceneBag(ros::package::getPath("planning_environment")+"/test_table_moved.bag",
                             temp_scene);

  kinematics_msgs::PositionIKRequest ik_request;
  ik_request.ik_link_name = "r_wrist_roll_link";
  ik_request.pose_stamped = obj_pose;
  ik_request.robot_state = rs;
  ik_request.ik_seed_state = rs;

  kinematics_msgs::GetConstraintAwarePositionIK::Request ik_req;
  kinematics_msgs::GetConstraintAwarePositionIK::Response ik_res;
  ik_req.ik_request = ik_request;
  ik_req.timeout = ros::Duration(1.0);


  ASSERT_TRUE(ik_service_client_.call(ik_req, ik_res));
  ASSERT_TRUE(ik_res.error_code.val == ik_res.error_code.SUCCESS);

  planning_environment::setRobotStateAndComputeTransforms(ik_res.solution, *planning_scene_state_);

  motion_planning_msgs::ArmNavigationErrorCodes error_code;
  motion_planning_msgs::Constraints path_constraints;
  ASSERT_TRUE(cm_->isKinematicStateValid(*planning_scene_state_,
                                         ik_res.solution.joint_state.name,
                                         error_code,
                                         goal_constraints,
                                         path_constraints));

  mplan_req.motion_plan_request.goal_constraints.joint_constraints.clear();
  mplan_req.motion_plan_request.goal_constraints.position_constraints.clear();

  for(unsigned int i = 0; i < ik_res.solution.joint_state.name.size(); i++) {
    motion_planning_msgs::JointConstraint jc;
    jc.joint_name = ik_res.solution.joint_state.name[i];
    jc.position = ik_res.solution.joint_state.position[i];
    jc.tolerance_below = 0.01;
    jc.tolerance_above = 0.01;
    mplan_req.motion_plan_request.goal_constraints.joint_constraints.push_back(jc);
  }
  mplan_req.motion_plan_request.start_state = rs;
  
  motion_planning_msgs::GetMotionPlan::Response mplan_res;
  ASSERT_TRUE(planning_service_client_.call(mplan_req, mplan_res));
  
  ASSERT_EQ(mplan_res.error_code.val,mplan_res.error_code.SUCCESS);
  
  EXPECT_GT(mplan_res.trajectory.joint_trajectory.points.size(), 0);
  
  std::vector<motion_planning_msgs::ArmNavigationErrorCodes> trajectory_error_codes;  
  EXPECT_TRUE(cm_->isJointTrajectoryValid(*planning_scene_state_, 
                                     mplan_res.trajectory.joint_trajectory,
                                     mplan_req.motion_plan_request.goal_constraints,
                                     mplan_req.motion_plan_request.path_constraints,
                                     error_code,
                                     trajectory_error_codes, false)) << error_code;
  
  planning_environment::setRobotStateAndComputeTransforms(get_res.planning_scene.robot_state, *planning_scene_state_);
  double planner_length = cm_->getTotalTrajectoryJointLength(*planning_scene_state_, mplan_res.trajectory.joint_trajectory);
  
  motion_planning_msgs::FilterJointTrajectoryWithConstraints::Request filter_req;
  motion_planning_msgs::FilterJointTrajectoryWithConstraints::Response filter_res;
  
  filter_req.trajectory = mplan_res.trajectory.joint_trajectory;
  
  filter_req.goal_constraints = mplan_req.motion_plan_request.goal_constraints;
  filter_req.path_constraints = mplan_req.motion_plan_request.path_constraints;
  filter_req.allowed_time = ros::Duration(1.0);
  
  EXPECT_TRUE(trajectory_filter_client_.call(filter_req, filter_res));
  
  EXPECT_TRUE(cm_->isJointTrajectoryValid(*planning_scene_state_, 
                                     filter_res.trajectory,
                                     mplan_req.motion_plan_request.goal_constraints,
                                     mplan_req.motion_plan_request.path_constraints,
                                     error_code,
                                     trajectory_error_codes, false)) << error_code;
  
  planning_environment::setRobotStateAndComputeTransforms(get_res.planning_scene.robot_state, *planning_scene_state_);
  double filter_length = cm_->getTotalTrajectoryJointLength(*planning_scene_state_, filter_res.trajectory);
  
  EXPECT_GE(planner_length, filter_length);
  
  ROS_INFO_STREAM("Planner points " << mplan_res.trajectory.joint_trajectory.points.size() << " filter points " << filter_res.trajectory.points.size() << " planner length " << planner_length << " filter length " << filter_length);
}
  
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_simple_trajectory_pipeline");
    
  return RUN_ALL_TESTS();
}
