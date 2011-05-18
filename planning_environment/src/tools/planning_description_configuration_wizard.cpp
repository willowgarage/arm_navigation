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
#include <planning_environment/models/model_utils.h>
#include <tf/transform_broadcaster.h>
#include <collision_space/environmentODE.h>
#include <rosgraph_msgs/Clock.h>
#include <planning_environment/util/collision_operations_generator.h>

#include <ncurses.h>

//in 100 hz ticks
static const unsigned int CONTROL_SPEED = 10;
static const std::string VIS_TOPIC_NAME = "planning_description_configuration_wizard";

class PlanningDescriptionConfigurationWizard {
public:  

  PlanningDescriptionConfigurationWizard(const std::string& full_path_name) :
    inited_(false)
  {
    ROS_INFO_STREAM("full path name is " << full_path_name);

    urdf_ = boost::shared_ptr<urdf::Model>(new urdf::Model());
    bool urdf_ok = urdf_->initFile(full_path_name);
    
    if(!urdf_ok) {
      ROS_WARN_STREAM("Urdf file " << full_path_name << " not ok");
      return;
    }
    
    //pushing to the param server
    std::string com = "rosparam set robot_description -t "+full_path_name;

    int ok = system(com.c_str());
    
    if(ok != 0) {
      ROS_WARN_STREAM("Setting parameter system call not ok");
      return;
    }

    cm_ = NULL;
    ops_gen_ = NULL;

    if(!setupWithWorldFixedFrame("", "Floating")) {
      return;
    }

    vis_marker_publisher_ = nh_.advertise<visualization_msgs::Marker>(VIS_TOPIC_NAME, 128);
    vis_marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(VIS_TOPIC_NAME+"_array", 128);

    inited_ = true;
  }

  ~PlanningDescriptionConfigurationWizard() {
    if(cm_ != NULL) { 
      delete cm_;
    }
    if(ops_gen_ != NULL) {
      delete ops_gen_;
    }
  }

  void deleteKinematicStates() {
    if(robot_state_ != NULL) {
      delete robot_state_;
      robot_state_ = NULL;
    }
  }

  bool setupWithWorldFixedFrame(const std::string& world_fixed_frame, const std::string& joint_type) 
  {
    lock_.lock();

    std::vector<planning_models::KinematicModel::GroupConfig> gcs;
    std::vector<planning_models::KinematicModel::MultiDofConfig> multi_dof_configs;
    const urdf::Link *root = urdf_->getRoot().get();
 
    std::string ff;

    if(world_fixed_frame.empty()) {
      ff = root->name;
    } else {
      ff = world_fixed_frame;
    }
    
    ROS_INFO_STREAM("Setting world frame to " << ff);

    //now this should work with an n=on-identity transform
    planning_models::KinematicModel::MultiDofConfig config("world_joint");
    config.type = joint_type;
    config.parent_frame_id = ff;
    config.child_frame_id = root->name;
    multi_dof_configs.push_back(config);

    deleteKinematicStates();

    if(cm_ != NULL) { 
      delete cm_;
    }
    if(ops_gen_ != NULL) {
      delete ops_gen_;
    }
    planning_models::KinematicModel* kmodel = new planning_models::KinematicModel(*urdf_, gcs, multi_dof_configs);

    if(kmodel->getRoot() == NULL) {
      ROS_INFO_STREAM("Kinematic root is NULL");
      lock_.unlock();
      return false;
    }

    robot_state_ = new planning_models::KinematicState(kmodel);
    robot_state_->setKinematicStateToDefault();

    ode_collision_model_ = new collision_space::EnvironmentModelODE();

    const std::vector<planning_models::KinematicModel::LinkModel*>& coll_links = kmodel->getLinkModelsWithCollisionGeometry();
    
    std::vector<std::string> coll_names;
    for(unsigned int i = 0; i < coll_links.size(); i++) {
      coll_names.push_back(coll_links[i]->getName());
    }
    collision_space::EnvironmentModel::AllowedCollisionMatrix default_collision_matrix(coll_names,false);
    std::map<std::string, double> default_link_padding_map;
    ode_collision_model_->setRobotModel(kmodel, default_collision_matrix, 
                                        default_link_padding_map, 0.0, 1.0);
    
    cm_ = new planning_environment::CollisionModels(urdf_,
                                                    kmodel, 
                                                    ode_collision_model_);
    ops_gen_ = new planning_environment::CollisionOperationsGenerator(cm_);

    lock_.unlock();

    return true;
  }

  void setAlwaysAndDefaultInCollisionMarkers(std::vector<planning_environment::CollisionOperationsGenerator::StringPair>& default_in_collision) {
    lock_.lock();

    std::vector<planning_environment::CollisionOperationsGenerator::StringPair> always_in_collision;
    std::vector<planning_environment::CollisionOperationsGenerator::CollidingJointValues> in_collision_joint_values;

    ops_gen_->generateAlwaysInCollisionPairs(always_in_collision, in_collision_joint_values);

    robot_state_->setKinematicStateToDefault();
    
    std_msgs::ColorRGBA always_color;
    always_color.a = 1.0;
    always_color.r = 1.0;
    always_color.g = .8;
    always_color.b = 0.04;

    collision_markers_.markers.clear();
    cm_->getAllCollisionPointMarkers(*robot_state_,
                                     collision_markers_,
                                     always_color,
                                     ros::Duration(.2));

    ops_gen_->disablePairCollisionChecking(always_in_collision);
    ops_gen_->generateDefaultInCollisionPairs(default_in_collision, in_collision_joint_values);

    std_msgs::ColorRGBA default_color;
    default_color.a = 1.0;
    default_color.r = 0.0;
    default_color.g = .8;
    default_color.b = 0.04;
    
    cm_->getAllCollisionPointMarkers(*robot_state_,
                                     collision_markers_,
                                     default_color,
                                     ros::Duration(.2));

    lock_.unlock();
  }

  visualization_msgs::Marker transformEnvironmentModelContactInfoMarker(const collision_space::EnvironmentModel::Contact& c) {
    std::string ns_name;
    ns_name = c.body_name_1;
    ns_name +="+";
    ns_name += c.body_name_2;
    visualization_msgs::Marker mk;
    mk.header.stamp = ros::Time::now();
    mk.header.frame_id = cm_->getWorldFrameId();
    mk.ns = ns_name;
    mk.id = 0;
    mk.type = visualization_msgs::Marker::SPHERE;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.position.x = c.pos.x();
    mk.pose.position.y = c.pos.y();
    mk.pose.position.z = c.pos.z();
    mk.pose.orientation.w = 1.0;
    
    mk.scale.x = mk.scale.y = mk.scale.z = 0.1;
    return mk;
  }

  void considerOftenInCollisionPairs() {

    std::vector<planning_environment::CollisionOperationsGenerator::StringPair> often_in_collision;
    std::vector<double> percentages;
    std::vector<planning_environment::CollisionOperationsGenerator::CollidingJointValues> in_collision_joint_values;

    ops_gen_->generateOftenInCollisionPairs(often_in_collision, percentages, in_collision_joint_values);

    if(often_in_collision.size() == 0) {
      printw("No additional often in collision pairs");
      refresh();
      return;
    }

    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.r = 1.0;
    color.g = 0.0;
    color.b = 1.0;

    considerInCollisionPairs(often_in_collision,
                             percentages,
                             in_collision_joint_values,
                             color);

  }

  void considerOccasionallyInCollisionPairs() {

    std::vector<planning_environment::CollisionOperationsGenerator::StringPair> in_collision;
    std::vector<planning_environment::CollisionOperationsGenerator::StringPair> not_in_collision;
    std::vector<double> percentages;
    std::vector<planning_environment::CollisionOperationsGenerator::CollidingJointValues> in_collision_joint_values;

    ops_gen_->generateOccasionallyAndNeverInCollisionPairs(in_collision, not_in_collision, percentages, in_collision_joint_values);

    if(in_collision.size() == 0) {
      printw("No additional often in collision pairs");
      refresh();
      return;
    }

    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.r = 1.0;
    color.g = 0.0;
    color.b = 1.0;

    considerInCollisionPairs(in_collision,
                             percentages,
                             in_collision_joint_values,
                             color);

  }
  
  void considerInCollisionPairs(std::vector<planning_environment::CollisionOperationsGenerator::StringPair>& in_collision_pairs,
                                std::vector<double>& percentages,
                                std::vector<planning_environment::CollisionOperationsGenerator::CollidingJointValues>& in_collision_joint_values,
                                const std_msgs::ColorRGBA& color
) {
    for(unsigned int i = 0; i < in_collision_pairs.size(); i++) {
      lock_.lock();
      collision_markers_.markers.clear();
      robot_state_->setKinematicState(in_collision_joint_values[i]);
      if(!cm_->isKinematicStateInCollision(*robot_state_)) {
        ROS_INFO_STREAM("Really should be in collision");
      }
      std::vector<collision_space::EnvironmentModel::AllowedContact> allowed_contacts;
      std::vector<collision_space::EnvironmentModel::Contact> coll_space_contacts;
      cm_->getCollisionSpace()->getAllCollisionContacts(allowed_contacts,
                                                        coll_space_contacts,
                                                        1);
      bool found = false;
      visualization_msgs::Marker marker;
      for(unsigned int j = 0; j < coll_space_contacts.size(); j++) {
        if((coll_space_contacts[j].body_name_1 == in_collision_pairs[i].first &&
            coll_space_contacts[j].body_name_2 == in_collision_pairs[i].second) ||
           (coll_space_contacts[j].body_name_1 == in_collision_pairs[i].second &&
            coll_space_contacts[j].body_name_2 == in_collision_pairs[i].first)) {
          found = true;
          marker = transformEnvironmentModelContactInfoMarker(coll_space_contacts[j]);
          marker.color = color;
          marker.lifetime = ros::Duration(.2);
          collision_markers_.markers.push_back(marker);
        }
      }
      lock_.unlock();
      if(!found) { 
        ROS_WARN_STREAM("Collision that should be there not found");
      } else {
        printw("Disable all collisions between %s and %s (frequency in collision %g) (y or n)?", in_collision_pairs[i].first.c_str(), in_collision_pairs[i].second.c_str(), percentages[i]);
        refresh();
        char str[80];
        getstr(str);
        if(str[0] != 'n') {
          ops_gen_->disablePairCollisionChecking(in_collision_pairs[i]);
        }
      }
    }
  }
 

  void updateCollisionsInCurrentState() {
    lock_.lock();
    std_msgs::ColorRGBA default_color;
    default_color.a = 1.0;
    default_color.r = 0.0;
    default_color.g = .8;
    default_color.b = 0.04;

    collision_markers_.markers.clear();
    
    cm_->getAllCollisionPointMarkers(*robot_state_,
                                     collision_markers_,
                                     default_color,
                                     ros::Duration(.2));
    lock_.unlock();
  }

  void sendMarkers() 
  {
    lock_.lock();
    vis_marker_array_publisher_.publish(collision_markers_);
    lock_.unlock();
  }

  void sendTransforms() {
    lock_.lock();
    ros::WallTime cur_time = ros::WallTime::now();
    rosgraph_msgs::Clock c;
    c.clock.nsec = cur_time.nsec;
    c.clock.sec = cur_time.sec;
    std::vector<geometry_msgs::TransformStamped> trans_vector;
    getAllRobotStampedTransforms(*robot_state_, trans_vector, c.clock);
    transform_broadcaster_.sendTransform(trans_vector);
    lock_.unlock();
  };

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
      if(ts.header.frame_id == ts.child_frame_id) continue; 
      tf::transformTFToMsg(ls->getGlobalLinkTransform(),ts.transform);
      trans_vector.push_back(ts);
    }
  }

  bool isInited() const {
    return inited_;
  }

  planning_environment::CollisionOperationsGenerator* getOperationsGenerator() 
  { 
    return ops_gen_;
  }

protected:

  bool inited_;

  ros::NodeHandle nh_;
  boost::shared_ptr<urdf::Model> urdf_;  
  planning_environment::CollisionModels* cm_;
  planning_environment::CollisionOperationsGenerator* ops_gen_;
  planning_models::KinematicState* robot_state_;
  collision_space::EnvironmentModel* ode_collision_model_;
  visualization_msgs::MarkerArray collision_markers_;

  tf::TransformBroadcaster transform_broadcaster_;
  ros::Publisher vis_marker_publisher_;
  ros::Publisher vis_marker_array_publisher_;
  
  boost::recursive_mutex lock_;

};

PlanningDescriptionConfigurationWizard* pdcw;

bool inited = false;

void spin_function() {
  ros::WallRate r(100.0);
  unsigned int counter = 0;
  while(ros::ok()) {
    if(inited) {
      pdcw->sendTransforms();
      if(counter%CONTROL_SPEED == 0) {
        counter = 1;
        pdcw->sendMarkers();
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
  if(pdcw != NULL) {
    delete pdcw;
  }
  endwin();
  exit(0);
}

int main(int argc, char** argv)
{

  srand(time(NULL));
  ros::init(argc, argv, "planning_description_configuration_wizard", ros::init_options::NoSigintHandler);

  if(argc < 2) {
    ROS_INFO_STREAM("Must specify a urdf file");
    exit(0);
  }

  std::string urdf_file = argv[1];
  pdcw = new PlanningDescriptionConfigurationWizard(urdf_file);

  if(!pdcw->isInited()) {
    ROS_WARN_STREAM("Can't init. Exiting");
    exit(0);
  }

  inited = true;

  boost::thread spin_thread(boost::bind(&spin_function));

  initscr();
  use_default_colors();
  start_color();

  std::vector<planning_environment::CollisionOperationsGenerator::StringPair> default_in_collision;
  pdcw->setAlwaysAndDefaultInCollisionMarkers(default_in_collision);

  // printw("In rviz collisions resulting from the pairs of links that are always in collision are shown in yellow.\n");
  // printw("These will be disabled\n");
  // printw("Link pairs that are in collision in the default state are shown in green\n");
  // for(unsigned int i = 0; i < default_in_collision.size(); i++) {
  //   printw("Disable all collisions between %s and %s (y or n)?", default_in_collision[i].first.c_str(), default_in_collision[i].second.c_str());
  //   refresh();
  //   char str[80];
  //   getstr(str);
  //   if(str[0] != 'n') {
  //     pdcw->getOperationsGenerator()->disablePairCollisionChecking(default_in_collision[i]);
  //   }
  //   pdcw->updateCollisionsInCurrentState();
  // }
  printw("Finding often in collision pairs\n");
  refresh();
  pdcw->considerOftenInCollisionPairs();

  printw("Finding occasionally in collision pairs\n");
  refresh();
  pdcw->considerOccasionallyInCollisionPairs();

  refresh();
  getch();
  endwin();
  ros::shutdown();
  return 0;
}





  

