#include <ros/ros.h>
#include <collision_proximity_planner/GetFreePath.h>

int main(int argc, char **argv){
  ros::init (argc, argv, "cp_test");
  ros::NodeHandle rh;

  ros::service::waitForService("collision_proximity_planner/plan");
  ros::ServiceClient ik_client = rh.serviceClient<collision_proximity_planner::GetFreePath>("collision_proximity_planner/plan");

  // define the service messages
  collision_proximity_planner::GetFreePath::Request request;
  collision_proximity_planner::GetFreePath::Response response;

  request.robot_state.joint_state.name.push_back("r_shoulder_pan_joint");
  request.robot_state.joint_state.name.push_back("r_shoulder_lift_joint");
  request.robot_state.joint_state.name.push_back("r_upper_arm_roll_joint");

  request.robot_state.joint_state.name.push_back("r_elbow_flex_joint");
  request.robot_state.joint_state.name.push_back("r_forearm_roll_joint");
  request.robot_state.joint_state.name.push_back("r_wrist_flex_joint");

  request.robot_state.joint_state.name.push_back("r_wrist_roll_joint");

  request.robot_state.joint_state.position.push_back(-0.330230);
  request.robot_state.joint_state.position.push_back(0.008300);
  request.robot_state.joint_state.position.push_back(-1.550000);

  request.robot_state.joint_state.position.push_back(-0.859908);
  request.robot_state.joint_state.position.push_back(3.139403);
  request.robot_state.joint_state.position.push_back(-0.529580);

  request.robot_state.joint_state.position.push_back(-1.591270);

  if(ik_client.call(request, response))
  {
    ROS_INFO("Service call was a success");
  }
  else
    ROS_ERROR("Service call failed");
  ros::shutdown();
}
