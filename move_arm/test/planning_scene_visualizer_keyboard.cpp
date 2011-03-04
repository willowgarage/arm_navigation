/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: E. Gil Jones

#include <termios.h>
#include <signal.h>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <math.h>

#include <ros/ros.h>

#include <planning_environment/models/collision_models.h>
#include <planning_environment_msgs/PlanningScene.h>
#include <planning_environment/models/model_utils.h>
#include <planning_environment_msgs/SetPlanningSceneAction.h>
#include <rosgraph_msgs/Clock.h>
#include <tf/transform_broadcaster.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <actionlib/client/simple_action_client.h>

int kfd = 0;
struct termios cooked, raw;

static const std::string VIS_TOPIC_NAME = "planning_scene_visualizer";

//in 100 hz ticks
static const unsigned int CONTROL_SPEED = 10;

static const double BASE_TRANS_SPEED=.3;
static const double BASE_ROT_SPEED = .15;

static const double HAND_TRANS_SPEED=.05;
static const double HAND_ROT_SPEED = .15;

static const std::string SET_PLANNING_SCENE_NAME_1="/pr2_right_arm_kinematics/set_planning_scene";
static const std::string SET_PLANNING_SCENE_NAME_2="/pr2_left_arm_kinematics/set_planning_scene";

static const std::string LEFT_IK_NAME="/pr2_left_arm_kinematics/get_constraint_aware_ik";
static const std::string RIGHT_IK_NAME="/pr2_right_arm_kinematics/get_constraint_aware_ik";

static const std::string RIGHT_ARM_GROUP = "right_arm";
static const std::string LEFT_ARM_GROUP = "left_arm";

static const std::string LEFT_IK_LINK = "l_wrist_roll_link";
static const std::string RIGHT_IK_LINK = "r_wrist_roll_link";

class PlanningSceneVisualizer {
public:  
  PlanningSceneVisualizer()
  {
    std::string robot_description_name = nh_.resolveName("robot_description", true);
    cm_ = new planning_environment::CollisionModels(robot_description_name);
    
    vis_marker_publisher_ = nh_.advertise<visualization_msgs::Marker>(VIS_TOPIC_NAME, 128);
    vis_marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(VIS_TOPIC_NAME+"_array", 128);
    
    ros::service::waitForService(LEFT_IK_NAME);
    ros::service::waitForService(RIGHT_IK_NAME);
    
    left_ik_service_client_ = nh_.serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>(LEFT_IK_NAME);
    right_ik_service_client_ = nh_.serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>(RIGHT_IK_NAME);

    set_planning_scene_action_1_ = new actionlib::SimpleActionClient<planning_environment_msgs::SetPlanningSceneAction>(SET_PLANNING_SCENE_NAME_1, true);
    set_planning_scene_action_1_->waitForServer(ros::Duration(.1));

    set_planning_scene_action_2_ = new actionlib::SimpleActionClient<planning_environment_msgs::SetPlanningSceneAction>(SET_PLANNING_SCENE_NAME_2, true);
    set_planning_scene_action_2_->waitForServer(ros::Duration(.1));

    //clock_publisher_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 128);

    send_collision_markers_ = false;
    send_ik_solution_ = false;
    send_end_effector_goal_markers_ = false;

    point_color_.a = 1.0;
    point_color_.r = 1.0;
    point_color_.g = .8;
    point_color_.b = 0.04;

    stat_color_.a = 0.5;
    stat_color_.r = 0.1;
    stat_color_.g = 0.8;
    stat_color_.b = 0.3;

    attached_color_.a = 0.5;
    attached_color_.r = 0.6;
    attached_color_.g = 0.4;
    attached_color_.b = 0.3;

    end_effector_color_.a = .9;
    end_effector_color_.r = 1.0;
    end_effector_color_.b = 0.0;
    end_effector_color_.g = 0.0;

    planning_state_ = NULL;
    end_effector_state_ = NULL;
  }
  
  ~PlanningSceneVisualizer() {
    delete set_planning_scene_action_1_;
    delete set_planning_scene_action_2_;
    deleteKinematicStates();
    delete cm_;
  }

  void deleteKinematicStates() {
    if(planning_state_ != NULL) {
      delete planning_state_;
    }
    if(end_effector_state_ != NULL) {
      delete end_effector_state_;
    }
  }

  bool loadPlanningScene(const std::string& filename) {
    if(!cm_->readPlanningSceneBag(filename,
                                  planning_scene_)) {
      ROS_ERROR("Bag file doesn't exist or doesn't contain a planning scene");
      return false;
    }
    deleteKinematicStates();
    setEndEffectorGoal(false);
    planning_state_ = cm_->setPlanningScene(planning_scene_);
    if(planning_state_ == NULL) {
      ROS_ERROR("Something wrong with planning scene");
      return false;
    }
    planning_state_joint_values_.clear();
    planning_state_->getKinematicStateValues(planning_state_joint_values_);
    updateCurrentCollisionSet();
    ros::WallTime cur_time = ros::WallTime::now();
    rosgraph_msgs::Clock c;
    c.clock.nsec = cur_time.nsec;
    c.clock.sec = cur_time.sec;
    getAllRobotStampedTransforms(*planning_state_, planning_scene_robot_transforms_, c.clock);

    planning_environment_msgs::SetPlanningSceneGoal planning_scene_goal;
    planning_scene_goal.planning_scene = planning_scene_;

    set_planning_scene_action_1_->sendGoal(planning_scene_goal);
    set_planning_scene_action_2_->sendGoal(planning_scene_goal);
    set_planning_scene_action_1_->waitForResult(ros::Duration(.1));
    set_planning_scene_action_2_->waitForResult(ros::Duration(.1));

    send_start_state_ = true;
    return true;
  }
  
  void sendTransformsAndClock() {
    ros::WallTime cur_time = ros::WallTime::now();
    rosgraph_msgs::Clock c;
    c.clock.nsec = cur_time.nsec;
    c.clock.sec = cur_time.sec;
    //clock_publisher_.publish(c);
    getAllRobotStampedTransforms(*planning_state_, planning_scene_robot_transforms_, c.clock);
    transform_broadcaster_.sendTransform(planning_scene_robot_transforms_);
  };

  void resetPlanningStateToStartState() {
    planning_state_->setKinematicState(planning_state_joint_values_);
    if(send_collision_markers_) {
      updateCurrentCollisionSet();
    }
  }

  void getAllRobotStampedTransforms(const planning_models::KinematicState& state,
                                    std::vector<geometry_msgs::TransformStamped>& trans_vector,
                                    const ros::Time& stamp) 
  {
    trans_vector.clear();
    const std::map<std::string, geometry_msgs::TransformStamped>& transforms = cm_->getSceneTransformMap();
    geometry_msgs::TransformStamped transvec;
    for(std::map<std::string, geometry_msgs::TransformStamped>::const_iterator it = transforms.begin();
        it != transforms.end();
        it++) {
      trans_vector.push_back(it->second);
    }
    for(unsigned int i = 0; i < state.getLinkStateVector().size(); i++) {
      const planning_models::KinematicState::LinkState* ls = state.getLinkStateVector()[i];
      geometry_msgs::TransformStamped ts;
      ts.header.stamp = stamp;
      ts.header.frame_id = cm_->getWorldFrameId();
      ts.child_frame_id = ls->getName();
      tf::transformTFToMsg(ls->getGlobalLinkTransform(),ts.transform);
      trans_vector.push_back(ts);
    }
  }

  void moveBase(double vx, double vy, double vt) 
  {
    btTransform cur = planning_state_->getRootTransform();
    double mult = CONTROL_SPEED/100.0;
    cur.setOrigin(btVector3(cur.getOrigin().x()+(vx*mult), cur.getOrigin().y()+(vy*mult), 0.0));
    cur.setRotation(btQuaternion(btVector3(0.0,0.0,1.0),cur.getRotation().getAngle()*cur.getRotation().getAxis().z()+(vt*mult)));
    planning_state_->getJointStateVector()[0]->setJointStateValues(cur);
    planning_state_->updateKinematicLinks();
    if(send_collision_markers_) {
      updateCurrentCollisionSet();
    }
  }

  void moveEndEffectorMarkers(double vx, double vy, double vz,
                              double vr, double vp, double vw)
  {
    btTransform cur = end_effector_state_->getLinkState(end_effector_link_)->getGlobalLinkTransform();
    double mult = CONTROL_SPEED/100.0;
    cur.setOrigin(btVector3(cur.getOrigin().x()+(vx*mult), cur.getOrigin().y()+(vy*mult), cur.getOrigin().z()+(vz*mult))); 
    btScalar roll, pitch, yaw;
    cur.getBasis().getRPY(roll,pitch,yaw);
    roll += vr*mult;
    pitch += vp*mult;
    yaw += vw*mult;
    cur.getBasis().setRPY(roll,pitch,yaw);
    if(!end_effector_state_->updateKinematicStateWithLinkAt(end_effector_link_, cur)) {
      ROS_INFO("Problem");
    }
  }
  
  void solveIKForEndEffectorPose()
  {
    bool right = (end_effector_link_ == RIGHT_IK_LINK);
    kinematics_msgs::PositionIKRequest ik_request;
    ik_request.ik_link_name = end_effector_link_;
    ik_request.pose_stamped.header.frame_id = cm_->getWorldFrameId();
    ik_request.pose_stamped.header.stamp = ros::Time::now();
    tf::poseTFToMsg(end_effector_state_->getLinkState(end_effector_link_)->getGlobalLinkTransform(), ik_request.pose_stamped.pose);
    planning_environment::convertKinematicStateToRobotState(*planning_state_, ros::Time::now(), 
                                                            cm_->getWorldFrameId(), ik_request.robot_state);
    ik_request.ik_seed_state = ik_request.robot_state;

    ros::ServiceClient* s;
    if(right) {
      s = &right_ik_service_client_;
    } else {
      s = &left_ik_service_client_;
    }
    kinematics_msgs::GetConstraintAwarePositionIK::Request ik_req;
    kinematics_msgs::GetConstraintAwarePositionIK::Response ik_res;
    ik_req.ik_request = ik_request;
    ik_req.timeout = ros::Duration(1.0);
    if(!s->call(ik_req, ik_res)) {
      ROS_INFO("Problem with ik service call");
      return;
    }
    if(ik_res.error_code.val != ik_res.error_code.SUCCESS) {
      ROS_INFO_STREAM("Call yields bad error code " << ik_res.error_code.val);
      return;
    }
    send_ik_solution_ = true;
    std::map<std::string, double> joint_values;
    for(unsigned int i = 0; i < ik_res.solution.joint_state.name.size(); i++) {
      joint_values[ik_res.solution.joint_state.name[i]] = ik_res.solution.joint_state.position[i];
    }
    end_effector_state_->setKinematicState(joint_values);
  }

  void sendMarkers() 
  {
    visualization_msgs::MarkerArray arr;
    if(send_start_state_) {
      cm_->getAllCollisionSpaceObjectMarkers(arr,
                                             stat_color_,
                                             attached_color_,
                                             ros::Duration(.2));
    }
    if(send_collision_markers_) {
      arr.markers.insert(arr.markers.end(), collision_markers_.markers.begin(), collision_markers_.markers.end());
    }
    if(send_ik_solution_) {
      const std::vector<const planning_models::KinematicModel::LinkModel*>& updated_links = 
        cm_->getKinematicModel()->getModelGroup(arm_group_name_)->getUpdatedLinkModels();
      std::vector<std::string> lnames;
      lnames.resize(updated_links.size());
      for(unsigned int i = 0; i < updated_links.size(); i++) {
        lnames[i] = updated_links[i]->getName();
      }
      cm_->getRobotMeshResourceMarkersGivenState(*end_effector_state_,
                                                 arr,
                                                 end_effector_color_,
                                                 ros::Duration(.2),
                                                 &lnames);
    } else if(send_end_effector_goal_markers_) {
      std::vector<std::string> lnames = 
        cm_->getKinematicModel()->getChildLinkModelNames(cm_->getKinematicModel()->getLinkModel(end_effector_link_));
      cm_->getRobotMeshResourceMarkersGivenState(*end_effector_state_,
                                                 arr,
                                                 end_effector_color_,
                                                 ros::Duration(.2),
                                                 &lnames);
    }
    vis_marker_array_publisher_.publish(arr);
  }

  void setEndEffectorGoal(bool on, bool right = false) {
    send_end_effector_goal_markers_ = on;
    send_ik_solution_ = false;
    if(send_end_effector_goal_markers_) {
      if(end_effector_state_ == NULL) {
        end_effector_state_ = new planning_models::KinematicState(*planning_state_);
      }
      if(right) {
        end_effector_link_ = RIGHT_IK_LINK;
        arm_group_name_ = RIGHT_ARM_GROUP;
      } else {
        end_effector_link_ = LEFT_IK_LINK;
        arm_group_name_ = LEFT_ARM_GROUP;
      }
    } else if(end_effector_state_ != NULL) {
      delete end_effector_state_;
      end_effector_state_ = NULL;
    }
  }

  void toggleShowCollisions() {
    send_collision_markers_ = !send_collision_markers_;
    if(send_collision_markers_) {
      updateCurrentCollisionSet();
    } else {
      collision_markers_.markers.clear();
    }
  }

  void updateCurrentCollisionSet()
  {
    collision_markers_.markers.clear();
    cm_->getAllCollisionPointMarkers(*planning_state_,
                                     collision_markers_,
                                     point_color_,
                                     ros::Duration(.2)); 
  }

protected:
  ros::NodeHandle nh_;
  planning_environment::CollisionModels* cm_;
  planning_models::KinematicState* planning_state_;
  planning_models::KinematicState* end_effector_state_;
  planning_environment_msgs::PlanningScene planning_scene_;
  std_msgs::ColorRGBA point_color_;
  std_msgs::ColorRGBA stat_color_;
  std_msgs::ColorRGBA attached_color_;
  std_msgs::ColorRGBA end_effector_color_;
  ros::Publisher vis_marker_publisher_;
  ros::Publisher vis_marker_array_publisher_;
  ros::Publisher clock_publisher_;

  std::string end_effector_link_;
  std::string arm_group_name_;

  ros::ServiceClient left_ik_service_client_;
  ros::ServiceClient right_ik_service_client_;

  actionlib::SimpleActionClient<planning_environment_msgs::SetPlanningSceneAction>* set_planning_scene_action_1_;  
  actionlib::SimpleActionClient<planning_environment_msgs::SetPlanningSceneAction>* set_planning_scene_action_2_;  

  bool send_start_state_;
  bool send_collision_markers_;
  bool send_end_effector_goal_markers_;
  bool send_ik_solution_;
  tf::TransformBroadcaster transform_broadcaster_;
  std::vector<geometry_msgs::TransformStamped> planning_scene_robot_transforms_;  
  std::map<std::string, double> planning_state_joint_values_;  

  visualization_msgs::MarkerArray collision_markers_;
};

PlanningSceneVisualizer* psv = NULL;
bool inited = false;

void spin_function() {
  ros::WallRate r(100.0);
  unsigned int counter = 0;
  while(ros::ok()) {
    if(inited) {
      psv->sendTransformsAndClock();
      if(counter%CONTROL_SPEED == 0) {
        counter = 1;
        psv->sendMarkers();
      } else {
        counter++;
      }
    }
    r.sleep();
    ros::spinOnce();
  }
}

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  if(psv != NULL) {
    delete psv;
  }
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planning_scene_keyboard", ros::init_options::NoSigintHandler);
  if(argc <= 1) {
    ROS_ERROR("Must give address of bag file");
    exit(0);
  }

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  //ros::NodeHandle loc("~");
  //loc.setParam("tf_prefix", "planning_scene");

  signal(SIGINT,quit);
  boost::thread spin_thread(boost::bind(&spin_function));

  psv = new PlanningSceneVisualizer();

  if(!psv->loadPlanningScene(argv[1])) {
    exit(0);
  }

  inited = true;
    
  char c;
  
  bool stop = false;
  bool reprint = true;
  while(!stop)
  {
    if(reprint) {
      reprint = false;
      puts("------------------");
      puts("Use 'i/k' for forward/back");
      puts("Use 'j/l' for strafe left/right");
      puts("Use 'n/m' for turning left and right");
      puts("Use 'c' to toggle collision markers");
      puts("Use 'r' to reset to original planning state");
      puts("Use 'w/e' for left/right end effector pose submenu");
      puts("Use 'q' to quit");
    }
    
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }
    switch(c)
    {
    case 'w': case 'e':
      {
        bool right = (c == 'e');
        psv->setEndEffectorGoal(true, right);
        puts("------------------");
        puts("Use 'i/k' for end effector forward/back");
        puts("Use 'j/l' for end effector left/right");
        puts("Use 'h/n' for end effector up/down");
        puts("Use 'r/f' for pitch up/down");
        puts("Use 'd/g' for yaw left/right");
        puts("Use 'v/b' for roll up/down");
        puts("Use 's' to solve for collision free ik given the current end effector position");
        puts("Use 'q' to exit submenu");
        bool end_effector_stop = false;
        while(!end_effector_stop) {
          if(read(kfd, &c, 1) < 0)
          {
            perror("read():");
            exit(-1);
          }
          switch(c) {
          case 'i':
            psv->moveEndEffectorMarkers(HAND_TRANS_SPEED, 0.0, 0.0,
                                        0.0,0.0,0.0);
            break;
          case 'k':
            psv->moveEndEffectorMarkers(-HAND_TRANS_SPEED, 0.0, 0.0,
                                        0.0,0.0,0.0);
            break;
          case 'j':
            psv->moveEndEffectorMarkers(0.0, HAND_TRANS_SPEED, 0.0,
                                        0.0,0.0,0.0);
            break;
          case 'l':
            psv->moveEndEffectorMarkers(0.0, -HAND_TRANS_SPEED, 0.0,
                                        0.0,0.0,0.0);
            break;
          case 'h':
            psv->moveEndEffectorMarkers(0.0, 0.0, HAND_TRANS_SPEED,
                                        0.0,0.0,0.0);
            break;
          case 'n':
            psv->moveEndEffectorMarkers(0.0, 0.0, -HAND_TRANS_SPEED,
                                        0.0,0.0,0.0);
            break;
          case 'r':
            psv->moveEndEffectorMarkers(0.0, 0.0, 0.0,
                                        0.0, HAND_ROT_SPEED, 0.0);
            break;
          case 'f':
            psv->moveEndEffectorMarkers(0.0, 0.0, 0.0,
                                        0.0, -HAND_ROT_SPEED, 0.0);
            break;
          case 'd':
            psv->moveEndEffectorMarkers(0.0, 0.0, 0.0,
                                        0.0, 0.0, HAND_ROT_SPEED);
            break;
          case 'g':
            psv->moveEndEffectorMarkers(0.0, 0.0, 0.0,
                                        0.0, 0.0, -HAND_ROT_SPEED);
            break;
          case 'v':
            psv->moveEndEffectorMarkers(0.0, 0.0, 0.0,
                                        HAND_ROT_SPEED, 0.0, 0.0);
            break;
          case 'b':
            psv->moveEndEffectorMarkers(0.0, 0.0, 0.0,
                                        -HAND_ROT_SPEED, 0.0, 0.0);
            break;
          case 's':
            psv->solveIKForEndEffectorPose();
            break;
          case 'q':
            psv->setEndEffectorGoal(false);
            end_effector_stop = true;
            reprint = true;
            break;
          }
        }
      }
      break;
    case 'i':
      psv->moveBase(BASE_TRANS_SPEED, 0.0, 0.0);
      break;
    case 'k':
      psv->moveBase(-BASE_TRANS_SPEED, 0.0, 0.0);
      break;
    case 'j':
      psv->moveBase(0.0, BASE_TRANS_SPEED, 0.0);
      break;
    case 'l':
      psv->moveBase(0.0, -BASE_TRANS_SPEED, 0.0);
      break;
    case 'n':
      psv->moveBase(0.0, 0.0, BASE_ROT_SPEED);
      break;
    case 'm':
      psv->moveBase(0.0, 0.0, -BASE_ROT_SPEED);
      break;
    case 'c':
      psv->toggleShowCollisions();
      break;
    case 'r':
      psv->resetPlanningStateToStartState();
      break;
    case 'q':
      stop = true;
      break;
    default:
      //ROS_INFO_STREAM("Keycode is " << c);
      break;
    }
  }
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  spin_thread.join();
  return(0);
}


