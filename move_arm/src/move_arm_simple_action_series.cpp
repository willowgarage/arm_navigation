#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/MoveArmAction.h>

//#include <arm_navigation_msgs/GetPlanningScene.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <planning_environment/models/collision_models.h>

static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";

// positions for right arm
double points[][7] =
{
// bent down
{-0.62882814703429735, 0.8418347450080067, -1.9799863031444187, -1.6658535208371843, 25.110489253195208, -1.9402143481790481, -3.3328334417445808},
// down diagonal
{-1.0213518629144611, 0.79781678347001694, -0.21997346810980545, -0.46313835939973602, -129.69683375982004, -0.2090908168158423, -43.029370768714301},
// bent up
{-1.6557731956287212, -0.12427141761710289, 0.27000326732522195, -1.6920980865205064, 0.2123976349290348, -1.7229959856671484, 0.45571628193275693},
// side up/out
{-1.964319710777449, -0.20129802400737434, 0.28000326732521985, -1.8126503280142443, -2.9063492821957446, -1.9440657888765194, -1.6092749252553165},
// forward out
{0.12365848504908895, -0.31925879050609901, 0.26002653189019498, -0.60416685029232786, -127.10443321064356, -0.14674235854525863, -49.120638472343479},
// side up
{-2.1133781512819105, -0.34097140919884783, 0.060003267325219906, -1.4142123996403146, 2.2710161038551959, -0.16913725241878641, -1.7351318586839262},
//  { -2.0, 0.0, 0.0, -0.2, 0.0, -0.15, 0.0 },
//  { -2.0, 0.5, 0.0, -0.2, 0.0, -0.15, 0.0 },
//  { -2.0, 1.0, 0.0, -0.2, 0.0, -0.15, 0.0 },
};
const int SIZE = 4;

void plan_filter_execute_function(
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> & move_arm,
  arm_navigation_msgs::MoveArmGoal & goal
)
{
   bool finished_within_time = false;
   move_arm.sendGoal(goal);
   finished_within_time = move_arm.waitForResult(ros::Duration(20.0));
   if (!finished_within_time)
   {
     move_arm.cancelGoal();
     ROS_INFO("Timed out achieving goal");
   }
   else
   {
     actionlib::SimpleClientGoalState state = move_arm.getState();
     bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
     if(success)
       ROS_INFO("Action finished: %s",state.toString().c_str());
     else
       ROS_INFO("Action failed: %s",state.toString().c_str());
   }
}

arm_navigation_msgs::CollisionObject getPole(bool right)
{
  //add the cylinder into the collision space
  arm_navigation_msgs::CollisionObject cylinder_object;
  if(right)
  {
    cylinder_object.id = "r_pole";
  }
  else
  {
    cylinder_object.id = "l_pole";
  }
  cylinder_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  cylinder_object.header.frame_id = "odom_combined";
  cylinder_object.header.stamp = ros::Time::now();
  arm_navigation_msgs::Shape object;
  object.type = arm_navigation_msgs::Shape::CYLINDER;
  object.dimensions.resize(2);
  object.dimensions[0] = .1;
  object.dimensions[1] = 1.0;
  geometry_msgs::Pose pose;
  pose.position.x = .5;
  if(right)
  {
    pose.position.y = -.6;
  }
  else
  {
    pose.position.y = +.6;
  }
  pose.position.z = .5;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  cylinder_object.shapes.push_back(object);
  cylinder_object.poses.push_back(pose);

  return cylinder_object;
 }


int main(int argc, char **argv){
  ros::init (argc, argv, "move_arm_joint_goal_test");
  ros::NodeHandle nh;

  // Planning scene
  ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
  ros::ServiceClient set_planning_scene_client = nh.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);
  arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
  arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;

  // Add cylender
  planning_scene_req.planning_scene_diff.collision_objects.push_back(getPole(true));
  planning_scene_req.planning_scene_diff.collision_objects.push_back(getPole(false));

  if(!set_planning_scene_client.call(planning_scene_req, planning_scene_res)) {
    ROS_WARN("Can't get planning scene");
    return -1;
  }


  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm_right("move_right_arm",true);
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm_left("move_left_arm",true);
  move_arm_right.waitForServer();
  move_arm_left.waitForServer();
  ROS_INFO("Connected to server");

  arm_navigation_msgs::MoveArmGoal start;
  arm_navigation_msgs::MoveArmGoal r_goal;
  std::vector<std::string> r_names(7);
  r_names[0] = "r_shoulder_pan_joint";
  r_names[1] = "r_shoulder_lift_joint";
  r_names[2] = "r_upper_arm_roll_joint";
  r_names[3] = "r_elbow_flex_joint";
  r_names[4] = "r_forearm_roll_joint";
  r_names[5] = "r_wrist_flex_joint";
  r_names[6] = "r_wrist_roll_joint";

  r_goal.motion_plan_request.group_name = "right_arm";
  r_goal.motion_plan_request.num_planning_attempts = 1;
  r_goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
  r_goal.motion_plan_request.planner_id= std::string("");
  r_goal.planner_service_name = std::string("/ompl_planning/plan_kinematic_path");
  //r_goal.planner_service_name = std::string("/chomp_planner_longrange/plan_path");
  r_goal.motion_plan_request.goal_constraints.joint_constraints.resize(r_names.size());
  for (unsigned int i = 0 ; i < r_goal.motion_plan_request.goal_constraints.joint_constraints.size(); ++i)
  {
    r_goal.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = r_names[i];
    r_goal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
    r_goal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
  }

  arm_navigation_msgs::MoveArmGoal l_goal;
  std::vector<std::string> l_names(7);
  l_names[0] = "l_shoulder_pan_joint";
  l_names[1] = "l_shoulder_lift_joint";
  l_names[2] = "l_upper_arm_roll_joint";
  l_names[3] = "l_elbow_flex_joint";
  l_names[4] = "l_forearm_roll_joint";
  l_names[5] = "l_wrist_flex_joint";
  l_names[6] = "l_wrist_roll_joint";

  l_goal.motion_plan_request.group_name = "left_arm";
  l_goal.motion_plan_request.num_planning_attempts = 1;
  l_goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
  l_goal.motion_plan_request.planner_id= std::string("");
  //l_goal.planner_service_name = std::string("/ompl_planning/plan_kinematic_path");
  l_goal.planner_service_name = std::string("/chomp_planner_longrange/plan_path");
  l_goal.motion_plan_request.goal_constraints.joint_constraints.resize(l_names.size());
  for (unsigned int i = 0 ; i < l_goal.motion_plan_request.goal_constraints.joint_constraints.size(); ++i)
  {
    l_goal.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = l_names[i];
    l_goal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
    l_goal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
  }


  for(int i=0; ros::ok(); i++)
  {
//   goal.motion_plan_request.goal_constraints.joint_constraints[1].position = points[i%SIZE][1];
    for (unsigned int j = 0 ; j < r_goal.motion_plan_request.goal_constraints.joint_constraints.size(); ++j)
    {
      r_goal.motion_plan_request.goal_constraints.joint_constraints[j].position = points[i%SIZE][j];
      l_goal.motion_plan_request.goal_constraints.joint_constraints[j].position = points[i%SIZE][j];
      if( j==0 || j==2 || j==4 || j==6 )
      {
        l_goal.motion_plan_request.goal_constraints.joint_constraints[j].position *= -1;
      }
    }

    if (nh.ok())
    {
      //plan_filter_execute_function(move_arm_right, r_goal);
      boost::thread right_arm_thread(&plan_filter_execute_function, boost::ref(move_arm_right), boost::ref(r_goal));
      //right_arm_thread.join();

      //plan_filter_execute_function(move_arm_left, l_goal);
      boost::thread left_arm_thread(&plan_filter_execute_function, boost::ref(move_arm_left), boost::ref(l_goal));
      //left_arm_thread.join();

      right_arm_thread.join();
      left_arm_thread.join();
    }
  }

  ros::shutdown();
}
