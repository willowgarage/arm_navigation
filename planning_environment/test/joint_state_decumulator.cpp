#include <map>
#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <planning_environment/models/robot_models.h>

class JointStateDecumulator {

public:

  JointStateDecumulator() : priv_handle_("~") 
  {
    robot_model_ = new planning_environment::RobotModels("robot_description");
    
    priv_handle_.param<bool>("publish_ungrouped_joints", publish_ungrouped_joints_, false);
    priv_handle_.param<std::string>("group_name", group_name_, "");
    priv_handle_.param<double>("publish_rate",publish_rate_,10.0);

    if(publish_ungrouped_joints_) {
      const std::vector<std::string>& joint_union = robot_model_->getGroupJointUnion();
      for(unsigned int i = 0; i < joint_union.size(); i++) {
        ungrouped_joints_[joint_union[i]] = true;
      }
    } else {
      if(robot_model_->getPlanningGroupJoints().find(group_name_) == robot_model_->getPlanningGroupJoints().end()) {
        ROS_WARN_STREAM("No group " << group_name_ << " in planning groups");
        exit(0);
      }
      std::vector<std::string> group_joints = robot_model_->getPlanningGroupJoints().find(group_name_)->second;
      for(unsigned int i = 0; i < group_joints.size(); i++) {
        joint_values_[group_joints[i]] = 0.0;
        joint_velocities_[group_joints[i]] = 0.0;
      }
    } 
    joint_state_subscriber_ = root_handle_.subscribe("joint_states", 1, &JointStateDecumulator::jointStateCallback, this);

    joint_state_publisher_ = root_handle_.advertise<sensor_msgs::JointState>("separate_joint_states", 1);
  }

  ~JointStateDecumulator()
  {
    delete robot_model_;
  }

  void jointStateCallback(const sensor_msgs::JointStateConstPtr &jointState) {
    if (jointState->name.size() != jointState->position.size() || jointState->name.size() !=jointState->velocity.size())
    {
      ROS_ERROR("Planning environment received invalid joint state");
      return;
    }
    for (unsigned int i = 0 ; i < jointState->name.size(); ++i)
    {
      if(publish_ungrouped_joints_) {
        if(ungrouped_joints_.find(jointState->name[i]) == ungrouped_joints_.end()) {
          joint_values_[jointState->name[i]] = jointState->position[i];
          joint_velocities_[jointState->name[i]] = jointState->velocity[i];
        }
      } else if(joint_values_.find(jointState->name[i]) != joint_values_.end()) {
        joint_values_[jointState->name[i]] = jointState->position[i];
        joint_velocities_[jointState->name[i]] = jointState->velocity[i];
      }
    }
  }

  void publishSeparateJointState() {
    
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(joint_values_.size());
    joint_state.position.resize(joint_values_.size());
    joint_state.velocity.resize(joint_values_.size());
    unsigned int i = 0;
    for(std::map<std::string, double>::iterator it = joint_values_.begin(); it != joint_values_.end(); it++, i++) {
      joint_state.name[i] = it->first;
      joint_state.position[i] = it->second;
      joint_state.velocity[i] = joint_velocities_[it->first];
    }
    joint_state_publisher_.publish(joint_state);
  }
  
  double getPublishRate() const {
    return publish_rate_;
  }

private:
  
  ros::NodeHandle root_handle_;
  ros::NodeHandle priv_handle_;

  planning_environment::RobotModels* robot_model_;

  std::string group_name_;
  double publish_rate_;
  bool publish_ungrouped_joints_;

  ros::Publisher joint_state_publisher_;
  ros::Subscriber joint_state_subscriber_;

  std::map<std::string, double> joint_values_;
  std::map<std::string, double> joint_velocities_;
  std::map<std::string, bool> ungrouped_joints_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_state_accumulator");
  
  ros::AsyncSpinner spinner(2); // Use 1 thread
  spinner.start();

  JointStateDecumulator jsd;

  ros::Rate pub_rate(jsd.getPublishRate());
  
  while (ros::ok()) 
  {
    jsd.publishSeparateJointState();
    pub_rate.sleep();
  }
  ros::shutdown();
  return 0;
}
