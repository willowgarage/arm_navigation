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

#include <ncurses.h>

//in 100 hz ticks
static const unsigned int CONTROL_SPEED = 10;

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

    if(!setupWithWorldFixedFrame("", "Floating")) {
      return;
    }

    //vis_marker_publisher_ = nh_.advertise<visualization_msgs::Marker>(VIS_TOPIC_NAME, 128);
    //vis_marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(VIS_TOPIC_NAME+"_array", 128);

    inited_ = true;
  }

  ~PlanningDescriptionConfigurationWizard() {
    delete cm_;
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
    lock_.unlock();

    return true;
  }

  void sendMarkers() 
  {
    lock_.lock();
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

protected:

  bool inited_;

  ros::NodeHandle nh_;
  boost::shared_ptr<urdf::Model> urdf_;  
  planning_environment::CollisionModels* cm_;
  planning_models::KinematicState* robot_state_;
  collision_space::EnvironmentModel* ode_collision_model_;

  tf::TransformBroadcaster transform_broadcaster_;
  
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

  refresh();
  getch();
  endwin();
  ros::shutdown();
  return 0;
}





  

