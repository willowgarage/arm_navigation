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

#include <planning_environment/tools/planning_description_configuration_wizard.h>
#include <qt4/QtGui/qapplication.h>
#include <qt4/QtGui/qradiobutton.h>
#include <qt4/QtGui/qdialogbuttonbox.h>
#include <qt4/QtCore/qtextstream.h>
#include <QHeaderView>
#include <boost/thread.hpp>

using namespace std;
using namespace planning_environment;
using namespace planning_models;
using namespace visualization_msgs;
using namespace collision_space;

PlanningDescriptionConfigurationWizard::PlanningDescriptionConfigurationWizard(const string& urdf_package,
                                                                               const string& urdf_path, QWidget* parent) :
  QWizard(parent), inited_(false), world_joint_config_("world_joint"), urdf_package_(urdf_package),
  urdf_path_(urdf_path)
{
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  emitter_ = NULL;
  progress_ = 0;
  package_directory_ = "";
  string full_urdf_path = ros::package::getPath(urdf_package_)+"/"+urdf_path_;

  ROS_INFO_STREAM("full path name is " << full_urdf_path);

  urdf_ = boost::shared_ptr<urdf::Model>(new urdf::Model());
  bool urdf_ok = urdf_->initFile(full_urdf_path);

  if(!urdf_ok)
  {
    ROS_WARN_STREAM("Urdf file " << full_urdf_path << " not ok");
    return;
  }

  //making directories
  dir_name_ = getRobotName() + "_arm_navigation";

  yaml_outfile_name_ = getRobotName() + "_planning_description.yaml";
  full_yaml_outfile_name_ = dir_name_ + "/config/" + yaml_outfile_name_;
  launch_outfile_name_ = getRobotName() + "_planning_environment.launch";
  full_launch_outfile_name_ = dir_name_ + "/launch/" + launch_outfile_name_;

  //pushing to the param server
  string com = "rosparam set robot_description -t " + full_urdf_path;

  int ok = system(com.c_str());

  if(ok != 0)
  {
    ROS_WARN_STREAM("Setting parameter system call not ok");
    return;
  }

  kmodel_ = NULL;
  cm_ = NULL;
  ops_gen_ = NULL;
  robot_state_ = NULL;
  ode_collision_model_ = NULL;

  if(!setupWithWorldFixedFrame("", "Floating"))
  {
    return;
  }

  outputConfigAndLaunchRviz();

  vis_marker_publisher_ = nh_.advertise<Marker> (VIS_TOPIC_NAME, 128);
  vis_marker_array_publisher_ = nh_.advertise<MarkerArray> (VIS_TOPIC_NAME + "_array", 128);

  wizard_mode_ = PlanningDescriptionConfigurationWizard::Easy;
  ode_collision_model_ = new EnvironmentModelODE();

  const vector<KinematicModel::LinkModel*>& coll_links = kmodel_->getLinkModelsWithCollisionGeometry();

  vector<string> coll_names;
  for(unsigned int i = 0; i < coll_links.size(); i++)
  {
    coll_names.push_back(coll_links[i]->getName());
  }
  EnvironmentModel::AllowedCollisionMatrix default_collision_matrix(coll_names, false);
  map<string, double> default_link_padding_map;
  ode_collision_model_->setRobotModel(kmodel_, default_collision_matrix, default_link_padding_map, 0.0, 1.0);

  cm_ = new CollisionModels(urdf_, kmodel_, ode_collision_model_);
  ops_gen_ = new CollisionOperationsGenerator(cm_);


  setupQtPages();
  inited_ = true;
}

PlanningDescriptionConfigurationWizard::~PlanningDescriptionConfigurationWizard()
{
  if(cm_ != NULL)
  {
    delete cm_;
  }
  if(ops_gen_ != NULL)
  {
    delete ops_gen_;
  }
  if(emitter_ != NULL) {
    delete emitter_;
  }
}

void PlanningDescriptionConfigurationWizard::deleteKinematicStates()
{
  if(robot_state_ != NULL)
  {
    delete robot_state_;
    robot_state_ = NULL;
  }
}

bool PlanningDescriptionConfigurationWizard::setupWithWorldFixedFrame(const string& world_fixed_frame,
                                                                      const string& joint_type)
{
  lock_.lock();

  vector<KinematicModel::GroupConfig> gcs;
  vector<KinematicModel::MultiDofConfig> multi_dof_configs;
  const urdf::Link *root = urdf_->getRoot().get();

  string ff;

  if(world_fixed_frame.empty())
  {
    ff = root->name;
  }
  else
  {
    ff = world_fixed_frame;
  }

  ROS_INFO_STREAM("Setting world frame to " << ff);

  //now this should work with an n=on-identity transform
  world_joint_config_.type = joint_type;
  world_joint_config_.parent_frame_id = ff;
  world_joint_config_.child_frame_id = root->name;
  multi_dof_configs.push_back(world_joint_config_);

  deleteKinematicStates();

  if(cm_ != NULL)
  {
    delete cm_;
  }
  if(ops_gen_ != NULL)
  {
    delete ops_gen_;
  }
  kmodel_ = new KinematicModel(*urdf_, gcs, multi_dof_configs);

  if(kmodel_->getRoot() == NULL)
  {
    ROS_INFO_STREAM("Kinematic root is NULL");
    lock_.unlock();
    return false;
  }

  robot_state_ = new KinematicState(kmodel_);
  robot_state_->setKinematicStateToDefault();

  lock_.unlock();

  return true;
}

void PlanningDescriptionConfigurationWizard::emitWorldJointYAML()
{

  (*emitter_) << YAML::Key << "multi_dof_joints";
  (*emitter_) << YAML::Value << YAML::BeginSeq;
  (*emitter_) << YAML::BeginMap;
  (*emitter_) << YAML::Key << "name" << YAML::Value << world_joint_config_.name;
  (*emitter_) << YAML::Key << "type" << YAML::Value << world_joint_config_.type;
  (*emitter_) << YAML::Key << "parent_frame_id" << YAML::Value << world_joint_config_.parent_frame_id;
  (*emitter_) << YAML::Key << "child_frame_id" << YAML::Value << world_joint_config_.child_frame_id;
  (*emitter_) << YAML::EndMap;
  (*emitter_) << YAML::EndSeq;
}

void PlanningDescriptionConfigurationWizard::outputConfigAndLaunchRviz()
{

  string template_name = ros::package::getPath("planning_environment")
    + "/config/planning_description_configuration_wizard.vcg";

  ifstream template_file(template_name.c_str());

  ofstream ofile("planning_description_configuration_wizard.vcg");

  char ch;
  char buf[80];
  while(template_file && template_file.get(ch))
  {
    if(ch != '$')
    {
      ofile.put(ch);
    }
    else
    {
      template_file.getline(buf, 80, '$');
      if(template_file.eof() || template_file.bad())
      {
        ROS_ERROR_STREAM("Bad template file");
        break;
      }
      ofile << kmodel_->getRoot()->getParentFrameId();
    }
  }

  ofstream touch_file("vcg_ready");
}

PlanningDescriptionConfigurationWizard::GroupAddStatus 
PlanningDescriptionConfigurationWizard::addGroup(const KinematicModel::GroupConfig& group_config)
{
  lock_.lock();
  
  if(kmodel_->hasModelGroup(group_config.name_))
  {
    QString t;
    QTextStream(&t) << "There is already a planning group named " << group_config.name_.c_str() << ". Replace this group?";
    confirm_group_text_->setText(t);
    int val = confirm_group_replace_dialog_->exec();
    if(val == QDialog::Rejected) {
      lock_.unlock();
      return GroupAddCancel;
    } 
    kmodel_->removeModelGroup(group_config.name_);
  }
  deleteKinematicStates();
  bool group_ok = kmodel_->addModelGroup(group_config);
  robot_state_ = new KinematicState(kmodel_);
  robot_state_->setKinematicStateToDefault();

  if(group_ok)
  {
    setup_groups_page_->updateGroupTable();
    current_show_group_ = group_config.name_;
  }
  else
  {
    current_show_group_ = "";
  }

  lock_.unlock();
  if(group_ok) {
    return GroupAddSuccess;
  }
  return GroupAddFailed;
}

void PlanningDescriptionConfigurationWizard::removeGroup(const std::string& name) 
{
  lock_.lock();
  if(!kmodel_->hasModelGroup(name)) {
    lock_.unlock();
    return;
  }
  deleteKinematicStates();
  kmodel_->removeModelGroup(name);

  robot_state_ = new KinematicState(kmodel_);
  robot_state_->setKinematicStateToDefault();

  if(current_show_group_ == name) {
    current_show_group_ = "";
  }
  
  lock_.unlock();
}

void PlanningDescriptionConfigurationWizard::popupFileFailure(const char* reason)
{
  file_failure_reason_->setText(reason);
  file_failure_dialog_->exec();
}

void PlanningDescriptionConfigurationWizard::emitGroupYAML()
{
  (*emitter_) << YAML::Key << "groups";
  (*emitter_) << YAML::Value << YAML::BeginSeq;

  const map<string, KinematicModel::GroupConfig>& group_config_map = kmodel_->getJointModelGroupConfigMap();

  for(map<string, KinematicModel::GroupConfig>::const_iterator it = group_config_map.begin(); it
        != group_config_map.end(); it++)
  {
    (*emitter_) << YAML::BeginMap;
    (*emitter_) << YAML::Key << "name" << YAML::Value << it->first;
    if(!it->second.base_link_.empty())
    {
      (*emitter_) << YAML::Key << "base_link" << YAML::Value << it->second.base_link_;
      (*emitter_) << YAML::Key << "tip_link" << YAML::Value << it->second.tip_link_;
    }
    else
    {
      if(!it->second.subgroups_.empty())
      {
        (*emitter_) << YAML::Key << "subgroups";
        (*emitter_) << YAML::Value << YAML::BeginSeq;
        for(unsigned int i = 0; i < it->second.subgroups_.size(); i++)
        {
          (*emitter_) << it->second.subgroups_[i];
        }
        (*emitter_) << YAML::EndSeq;
      }
      if(!it->second.joints_.empty())
      {
        (*emitter_) << YAML::Key << "joints";
        (*emitter_) << YAML::Value << YAML::BeginSeq;
        for(unsigned int i = 0; i < it->second.joints_.size(); i++)
        {
          (*emitter_) << it->second.joints_[i];
        }
        (*emitter_) << YAML::EndSeq;
      }
    }
    (*emitter_) << YAML::EndMap;
  }
  (*emitter_) << YAML::EndSeq;
}

// void setupGroupSubgroupCollection(const string& new_group_name) {
//   while(1) {
//     clear();
//     vector<string> group_names;
//     kmodel_->getModelGroupNames(group_names);
//     vector<bool> is_included(group_names.size(), false);
//     for(unsigned int i = 0; i < group_names.size(); i++) {
//       printw("%d) ", i);
//       if(is_included[i]) {
//         printw("(X)");
//       } else {
//         printw("( )");
//       }
//       printw(" %s ", group_names[i].c_str());
//     }
//     printw("Enter a subgroup number to toggle inclusion ");
//     printw("Enter 'v' to visualize current subgroup ");
//     printw("Enter 'x' to accept current subgroup ");
//     refresh();
//     char str[80];
//     getstr(str);
//     unsigned int entry;
//     stringstream ss(str);
//     ss >> entry;
//     if(entry == 0) {

//     }
//   }
// }

void PlanningDescriptionConfigurationWizard::outputJointLimitsYAML()
{
  map<string, bool> already;
  boost::shared_ptr<urdf::Model> robot_model = urdf_;
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;
  emitter << YAML::Key << "joint_limits";
  emitter << YAML::Value << YAML::BeginMap;
  const map<string, KinematicModel::JointModelGroup*>& group_map = kmodel_->getJointModelGroupMap();
  for(map<string, KinematicModel::JointModelGroup*>::const_iterator it = group_map.begin(); it != group_map.end(); it++)
  {
    const vector<const KinematicModel::JointModel*>& jmv = it->second->getJointModels();

    for(unsigned int i = 0; i < jmv.size(); i++)
    {
      boost::shared_ptr<const urdf::Joint> urdf_joint = robot_model->getJoint(jmv[i]->getName());
      double max_vel = 0.0;
      if(urdf_joint && urdf_joint->limits)
      {
        max_vel = urdf_joint->limits->velocity;
      } 
      const map<string, pair<double, double> >& bounds_map = jmv[i]->getAllVariableBounds();
      for(map<string, pair<double, double> >::const_iterator it2 = bounds_map.begin(); it2 != bounds_map.end(); it2++)
      {
        if(already.find(it2->first) == already.end())
        {
          already[it2->first] = true;
          emitter << YAML::Key << it2->first;
          emitter << YAML::Value << YAML::BeginMap;
          emitter << YAML::Key << "has_position_limits";
          const KinematicModel::RevoluteJointModel* rev =
            dynamic_cast<const KinematicModel::RevoluteJointModel*> (jmv[i]);
          bool has_limits = (rev == NULL || !rev->continuous_);
          emitter << YAML::Value << has_limits;
          if(max_vel > 0.0) {
            emitter << YAML::Key << "has_velocity_limits" << YAML::Value << "true";
            emitter << YAML::Key << "max_velocity" << YAML::Value << max_vel;
            emitter << YAML::Key << "has_acceleration_limits" << YAML::Value << "true";
            emitter << YAML::Key << "max_acceleration" << YAML::Value << DEFAULT_ACCELERATION;
          } else {
            emitter << YAML::Key << "has_velocity_limits" << YAML::Value << "false";
            emitter << YAML::Key << "has_acceleration_limits" << YAML::Value << "false";
          }
          emitter << YAML::Key << "angle_wraparound" << YAML::Value << !has_limits;
          emitter << YAML::EndMap;
        }
      }
    }
  }
  emitter << YAML::EndMap;
  emitter << YAML::EndMap;
  ofstream outf((dir_name_ + "/config/joint_limits.yaml").c_str(), ios_base::trunc);

  outf << emitter.c_str();
}

Marker PlanningDescriptionConfigurationWizard::transformEnvironmentModelContactInfoMarker(const EnvironmentModel::Contact& c)
{
  string ns_name;
  ns_name = c.body_name_1;
  ns_name += "+";
  ns_name += c.body_name_2;
  Marker mk;
  mk.header.stamp = ros::Time::now();
  mk.header.frame_id = cm_->getWorldFrameId();
  mk.ns = ns_name;
  mk.id = 0;
  mk.type = Marker::SPHERE;
  mk.action = Marker::ADD;
  mk.pose.position.x = c.pos.x();
  mk.pose.position.y = c.pos.y();
  mk.pose.position.z = c.pos.z();
  mk.pose.orientation.w = 1.0;

  mk.scale.x = mk.scale.y = mk.scale.z = 0.05;
  return mk;
}


void PlanningDescriptionConfigurationWizard::outputPlanningDescriptionYAML()
{
  //initial map
  if(emitter_ != NULL) {
    delete emitter_;
  }
  emitter_ = new YAML::Emitter;
  (*emitter_) << YAML::BeginMap;
  emitWorldJointYAML();
  emitGroupYAML();
  //ops_gen_->performanceTestSavedResults(disable_map_);
  ops_gen_->outputYamlStringOfSavedResults((*emitter_), disable_map_);
  //end map
  (*emitter_) << YAML::EndMap;
  ofstream outf(full_yaml_outfile_name_.c_str(), ios_base::trunc);

  outf << emitter_->c_str();
}

void PlanningDescriptionConfigurationWizard::outputOMPLGroupYAML()
{
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;
  emitter << YAML::Key << "planner_configs";
  
  emitter << YAML::Value << YAML::BeginMap;
  
  emitter << YAML::Key << "SBLkConfig1";
  emitter << YAML::Value << YAML::BeginMap;
  emitter << YAML::Key << "type" << YAML::Value << "kinematic::SBL";
  emitter << YAML::EndMap;
  
  emitter << YAML::Key << "LBKPIECEkConfig1";
  emitter << YAML::Value << YAML::BeginMap;
  emitter << YAML::Key << "type" << YAML::Value << "kinematic::LBKPIECE";
  emitter << YAML::EndMap;
  
  emitter << YAML::EndMap;
  
  emitter << YAML::Key << "groups";
  const std::map<std::string, planning_models::KinematicModel::GroupConfig>& group_config_map = kmodel_->getJointModelGroupConfigMap();
  
  emitter << YAML::Value << YAML::BeginSeq;
  for(std::map<std::string, planning_models::KinematicModel::GroupConfig>::const_iterator it = group_config_map.begin();
      it != group_config_map.end();
      it++) {
    emitter << it->first;
    if(!it->second.base_link_.empty()) {
      emitter << it->first+"_cartesian";
    }
  }
  emitter << YAML::EndSeq;
  
  for(std::map<std::string, planning_models::KinematicModel::GroupConfig>::const_iterator it = group_config_map.begin();
      it != group_config_map.end();
      it++) {
    emitter << YAML::Key << it->first;
    emitter << YAML::Value << YAML::BeginMap;
    emitter << YAML::Key << "planner_type" << YAML::Value << "JointPlanner";
    emitter << YAML::Key << "planner_configs" << YAML::Value << YAML::BeginSeq;
    emitter << "SBLkConfig1" << "LBKPIECEkConfig1" << YAML::EndSeq;
    emitter << YAML::Key << "projection_evaluator" << YAML::Value << "joint_state";
    emitter << YAML::EndMap;
  }
  
  //now doing cartesian for any groups that are kinematic chains
  for(std::map<std::string, planning_models::KinematicModel::GroupConfig>::const_iterator it = group_config_map.begin();
      it != group_config_map.end();
      it++) {
    if(!it->second.base_link_.empty()) {
      emitter << YAML::Key << it->first+"_cartesian";
      emitter << YAML::Value << YAML::BeginMap;
      emitter << YAML::Key << "parent_frame" << YAML::Value << it->second.base_link_;
      emitter << YAML::Key << "physical_group" << YAML::Value << it->first;
      emitter << YAML::Key << "planner_type" << YAML::Value << "RPYIKTaskSpacePlanner";
      
      emitter << YAML::Key << "state_spaces" << YAML::Value << YAML::BeginSeq;
      emitter << "x" << "y" << "z" << "roll" << "pitch" << "yaw" << YAML::EndSeq;
      
      emitter << YAML::Key << "x" << YAML::Value << YAML::BeginMap;
      emitter << YAML::Key << "type" << YAML::Value << "Linear";
      emitter << YAML::Key << "min" << YAML::Value << "-2.0";
      emitter << YAML::Key << "max" << YAML::Value << "2.0";
      emitter << YAML::EndMap;
      
      emitter << YAML::Key << "y" << YAML::Value << YAML::BeginMap;
      emitter << YAML::Key << "type" << YAML::Value << "Linear";
      emitter << YAML::Key << "min" << YAML::Value << "-2.0";
      emitter << YAML::Key << "max" << YAML::Value << "2.0";
      emitter << YAML::EndMap;
      
      emitter << YAML::Key << "z" << YAML::Value << YAML::BeginMap;
      emitter << YAML::Key << "type" << YAML::Value << "Linear";
      emitter << YAML::Key << "min" << YAML::Value << "-2.0";
      emitter << YAML::Key << "max" << YAML::Value << "2.0";
      emitter << YAML::EndMap;
      
      emitter << YAML::Key << "roll" << YAML::Value << YAML::BeginMap;
      emitter << YAML::Key << "type" << YAML::Value << "Revolute";
      emitter << YAML::EndMap;
      
      emitter << YAML::Key << "pitch" << YAML::Value << YAML::BeginMap;
      emitter << YAML::Key << "type" << YAML::Value << "Revolute";
      emitter << YAML::EndMap;
      
      emitter << YAML::Key << "yaw" << YAML::Value << YAML::BeginMap;
      emitter << YAML::Key << "type" << YAML::Value << "Revolute";
      emitter << YAML::EndMap;
      
      emitter << YAML::Key << "planner_configs" << YAML::Value << YAML::BeginSeq;
      emitter << "SBLkConfig1" << "LBKPIECEkConfig1" << YAML::EndSeq;
      
      emitter << YAML::Key << "kinematics_solver" << YAML::Value 
              << "arm_kinematics_constraint_aware/KDLArmKinematicsPlugin";
      emitter << YAML::Key << "tip_name" << YAML::Value << it->second.tip_link_;
      emitter << YAML::Key << "root_name" << YAML::Value << it->second.base_link_;
      emitter << YAML::Key << "projection_evaluator" << YAML::Value << "joint_state";
      emitter << YAML::Key << "longest_valid_segment_fraction" << YAML::Value << "0.001"; 
      emitter << YAML::EndMap;
    }
  }
  emitter << YAML::EndMap;
  std::ofstream outf((dir_name_+"/config/ompl_planning.yaml").c_str(), std::ios_base::trunc);
  
  outf << emitter.c_str();
}

void PlanningDescriptionConfigurationWizard::outputOMPLLaunchFile()
{
  TiXmlDocument doc;
  TiXmlElement* launch_root = new TiXmlElement("launch");
  doc.LinkEndChild(launch_root);

  TiXmlElement *inc = new TiXmlElement("include");
  launch_root->LinkEndChild(inc);
  inc->SetAttribute("file", "$(find " + dir_name_ + ")/launch/" + launch_outfile_name_);

  TiXmlElement *node = new TiXmlElement("node");
  launch_root->LinkEndChild(node);
  node->SetAttribute("pkg", "ompl_ros_interface");
  node->SetAttribute("type", "ompl_ros");
  node->SetAttribute("name", "ompl_planning");

  TiXmlElement *p = new TiXmlElement("param");
  node->LinkEndChild(p);
  p->SetAttribute("name", "default_planner_config");
  p->SetAttribute("type", "string");
  p->SetAttribute("value", "SBLkConfig1");

  TiXmlElement *rp = new TiXmlElement("rosparam");
  node->LinkEndChild(rp);
  rp->SetAttribute("command", "load");
  rp->SetAttribute("file", "$(find " + dir_name_ + ")/config/ompl_planning.yaml");
  doc.SaveFile(dir_name_ + "/launch/ompl_planning.launch");
}

void PlanningDescriptionConfigurationWizard::outputTrajectoryFilterLaunch()
{
  TiXmlDocument doc;
  TiXmlElement* launch_root = new TiXmlElement("launch");
  doc.LinkEndChild(launch_root);

  TiXmlElement *inc = new TiXmlElement("include");
  launch_root->LinkEndChild(inc);
  inc->SetAttribute("file", "$(find " + dir_name_ + ")/launch/" + launch_outfile_name_);

  TiXmlElement *node = new TiXmlElement("node");
  launch_root->LinkEndChild(node);
  node->SetAttribute("pkg", "trajectory_filter_server");
  node->SetAttribute("type", "trajectory_filter_server");
  node->SetAttribute("name", "trajectory_filter_server");

  TiXmlElement *rp = new TiXmlElement("rosparam");
  node->LinkEndChild(rp);
  rp->SetAttribute("command", "load");
  rp->SetAttribute("file", "$(find trajectory_filter_server)/config/filters.yaml");

  TiXmlElement *rp2 = new TiXmlElement("rosparam");
  node->LinkEndChild(rp2);
  rp2->SetAttribute("command", "load");
  rp2->SetAttribute("file", "$(find " + dir_name_ + ")/config/joint_limits.yaml");

  doc.SaveFile(dir_name_ + "/launch/trajectory_filter_server.launch");
}

void PlanningDescriptionConfigurationWizard::outputPlanningEnvironmentLaunch()
{
  TiXmlDocument doc;
  TiXmlElement* launch_root = new TiXmlElement("launch");
  doc.LinkEndChild(launch_root);

  TiXmlElement *rd = new TiXmlElement("param");
  launch_root->LinkEndChild(rd);
  rd->SetAttribute("name", "robot_description");
  rd->SetAttribute("textfile", "$(find " + urdf_package_ + ")/" + urdf_path_);

  TiXmlElement *rp = new TiXmlElement("rosparam");
  launch_root->LinkEndChild(rp);
  rp->SetAttribute("command", "load");
  rp->SetAttribute("ns", "robot_description_planning");
  rp->SetAttribute("file", "$(find " + dir_name_ + ")/config/" + yaml_outfile_name_);
  doc.SaveFile(full_launch_outfile_name_);

}

void PlanningDescriptionConfigurationWizard::outputKinematicsLaunchFiles()
{
  TiXmlDocument doc;
  TiXmlElement* launch_root = new TiXmlElement("launch");
  doc.LinkEndChild(launch_root);

  TiXmlElement *inc = new TiXmlElement("include");
  launch_root->LinkEndChild(inc);
  inc->SetAttribute("file", "$(find " + dir_name_ + ")/launch/" + launch_outfile_name_);

  const map<string, KinematicModel::GroupConfig>& group_config_map = kmodel_->getJointModelGroupConfigMap();

  for(map<string, KinematicModel::GroupConfig>::const_iterator it = group_config_map.begin(); it
        != group_config_map.end(); it++)
  {
    if(!it->second.base_link_.empty())
    {
      TiXmlElement *node = new TiXmlElement("node");
      launch_root->LinkEndChild(node);
      node->SetAttribute("pkg", "arm_kinematics_constraint_aware");
      node->SetAttribute("type", "arm_kinematics_constraint_aware");
      node->SetAttribute("name", getRobotName() + "_" + it->first + "_kinematics");

      TiXmlElement *group_param = new TiXmlElement("param");
      node->LinkEndChild(group_param);
      group_param->SetAttribute("name", "group");
      group_param->SetAttribute("type", "string");
      group_param->SetAttribute("value", it->first);

      TiXmlElement *base_param = new TiXmlElement("param");
      node->LinkEndChild(base_param);
      base_param->SetAttribute("name", it->first + "/root_name");
      base_param->SetAttribute("type", "string");
      base_param->SetAttribute("value", it->second.base_link_);

      TiXmlElement *tip_param = new TiXmlElement("param");
      node->LinkEndChild(tip_param);
      tip_param->SetAttribute("name", it->first + "/tip_name");
      tip_param->SetAttribute("type", "string");
      tip_param->SetAttribute("value", it->second.tip_link_);

      TiXmlElement *solver_param = new TiXmlElement("param");
      node->LinkEndChild(solver_param);
      solver_param->SetAttribute("name", "kinematics_solver");
      solver_param->SetAttribute("type", "string");
      solver_param->SetAttribute("value", "arm_kinematics_constraint_aware/KDLArmKinematicsPlugin");
    }
  }

  doc.SaveFile(dir_name_ + "/launch/constraint_aware_kinematics.launch");
}

void PlanningDescriptionConfigurationWizard::outputMoveGroupLaunchFiles()
{
  TiXmlDocument doc;
  TiXmlElement* launch_root = new TiXmlElement("launch");
  doc.LinkEndChild(launch_root);

  const map<string, KinematicModel::GroupConfig>& group_config_map = kmodel_->getJointModelGroupConfigMap();

  for(map<string, KinematicModel::GroupConfig>::const_iterator it = group_config_map.begin(); it
        != group_config_map.end(); it++)
  {
    TiXmlElement *inc = new TiXmlElement("include");
    launch_root->LinkEndChild(inc);
    inc->SetAttribute("file", "$(find " + dir_name_ + ")/launch/move_"+it->first+".launch");

    TiXmlDocument group_doc;
    TiXmlElement* group_launch_root = new TiXmlElement("launch");
    group_doc.LinkEndChild(group_launch_root);
    
    TiXmlElement *node = new TiXmlElement("node");
    group_launch_root->LinkEndChild(node);
    node->SetAttribute("pkg", "move_arm");
    node->SetAttribute("type", "move_arm_simple_action");
    node->SetAttribute("name", getRobotName() + "_move_" + it->first);
    
    TiXmlElement *group_param = new TiXmlElement("param");
    node->LinkEndChild(group_param);
    group_param->SetAttribute("name", "group");
    group_param->SetAttribute("type", "string");
    group_param->SetAttribute("value", it->first);

    TiXmlElement *remap1 = new TiXmlElement("remap");
    node->LinkEndChild(remap1);
    remap1->SetAttribute("from", "arm_ik");
    remap1->SetAttribute("to", getRobotName()+"_"+it->first+"_kinematics/get_constraint_aware_ik");

    TiXmlElement *base_param = new TiXmlElement("param");
    node->LinkEndChild(base_param);
    base_param->SetAttribute("name", "controller_action_name");
    base_param->SetAttribute("type", "string");
    base_param->SetAttribute("value", it->first+"_controller/follow_joint_trajectory");   
    group_doc.SaveFile(dir_name_ + "/launch/move_"+it->first+".launch");
  }
  doc.SaveFile(dir_name_ + "/launch/move_groups.launch");
}

void PlanningDescriptionConfigurationWizard::outputPlanningComponentVisualizerLaunchFile()
{
  //now doing planning components .vcg file
  string template_name_2 = ros::package::getPath("planning_environment") + "/config/planning_components_visualizer.vcg";

  ifstream template_file_2(template_name_2.c_str());

  ofstream ofile_2((dir_name_ + "/config/planning_components_visualizer.vcg").c_str());

  char ch;
  char buf[80];
  while(template_file_2 && template_file_2.get(ch))
  {
    if(ch != '$')
    {
      ofile_2.put(ch);
    }
    else
    {
      template_file_2.getline(buf, 80, '$');
      if(template_file_2.eof() || template_file_2.bad())
      {
        ROS_ERROR_STREAM("Bad template file");
        break;
      }
      ofile_2 << kmodel_->getRoot()->getParentFrameId();
    }
  }

  TiXmlDocument doc;
  TiXmlElement* launch_root = new TiXmlElement("launch");
  doc.LinkEndChild(launch_root);

  TiXmlElement *inc = new TiXmlElement("include");
  launch_root->LinkEndChild(inc);
  inc->SetAttribute("file", "$(find " + dir_name_ + ")/launch/" + launch_outfile_name_);

  TiXmlElement *pre = new TiXmlElement("include");
  launch_root->LinkEndChild(pre);
  pre->SetAttribute("file",
                    "$(find planning_environment)/launch/planning_environment_visualization_prerequisites.launch");

  TiXmlElement *kin = new TiXmlElement("include");
  launch_root->LinkEndChild(kin);
  kin->SetAttribute("file", "$(find " + dir_name_ + ")/launch/constraint_aware_kinematics.launch");

  TiXmlElement *ompl = new TiXmlElement("include");
  launch_root->LinkEndChild(ompl);
  ompl->SetAttribute("file", "$(find " + dir_name_ + ")/launch/ompl_planning.launch");

  TiXmlElement *fil = new TiXmlElement("include");
  launch_root->LinkEndChild(fil);
  fil->SetAttribute("file", "$(find " + dir_name_ + ")/launch/trajectory_filter_server.launch");

  TiXmlElement *rviz = new TiXmlElement("node");
  launch_root->LinkEndChild(rviz);
  rviz->SetAttribute("pkg", "rviz");
  rviz->SetAttribute("type", "rviz");
  rviz->SetAttribute("name", "rviz_planning_components");
  rviz->SetAttribute("args", "-d $(find " + dir_name_ + ")/config/planning_components_visualizer.vcg");

  TiXmlElement *vis = new TiXmlElement("node");
  launch_root->LinkEndChild(vis);
  vis->SetAttribute("pkg", "move_arm");
  vis->SetAttribute("type", "planning_components_visualizer");
  vis->SetAttribute("name", "planning_components_visualizer");
  vis->SetAttribute("output", "screen");

  TiXmlElement *state_publisher = new TiXmlElement("node");
  launch_root->LinkEndChild(state_publisher);
  state_publisher->SetAttribute("pkg", "robot_state_publisher");
  state_publisher->SetAttribute("type", "state_publisher");
  state_publisher->SetAttribute("name", "rob_st_pub");

  doc.SaveFile(dir_name_ + "/launch/planning_components_visualizer.launch");
}

void PlanningDescriptionConfigurationWizard::outputArmNavigationLaunchFile()
{
  TiXmlDocument doc;
  TiXmlElement* launch_root = new TiXmlElement("launch");
  doc.LinkEndChild(launch_root);

  TiXmlElement *inc = new TiXmlElement("include");
  launch_root->LinkEndChild(inc);
  inc->SetAttribute("file", "$(find " + dir_name_ + ")/launch/" + launch_outfile_name_);

  TiXmlElement *inc_env = new TiXmlElement("include");
  launch_root->LinkEndChild(inc_env);
  inc_env->SetAttribute("file", "$(find planning_environment)/launch/environment_server.launch");

  TiXmlElement *arg1 = new TiXmlElement("arg");
  inc_env->LinkEndChild(arg1);
  arg1->SetAttribute("name", "use_monitor");
  arg1->SetAttribute("value", "true");

  TiXmlElement *arg2 = new TiXmlElement("arg");
  inc_env->LinkEndChild(arg2);
  arg2->SetAttribute("name", "use_collision_map");
  arg2->SetAttribute("value", "false");

  TiXmlElement *kin = new TiXmlElement("include");
  launch_root->LinkEndChild(kin);
  kin->SetAttribute("file", "$(find " + dir_name_ + ")/launch/constraint_aware_kinematics.launch");

  TiXmlElement *ompl = new TiXmlElement("include");
  launch_root->LinkEndChild(ompl);
  ompl->SetAttribute("file", "$(find " + dir_name_ + ")/launch/ompl_planning.launch");

  TiXmlElement *fil = new TiXmlElement("include");
  launch_root->LinkEndChild(fil);
  fil->SetAttribute("file", "$(find " + dir_name_ + ")/launch/trajectory_filter_server.launch");

  TiXmlElement *incgr = new TiXmlElement("include");
  launch_root->LinkEndChild(incgr);
  incgr->SetAttribute("file", "$(find " + dir_name_ + ")/launch/move_groups.launch");

  doc.SaveFile(dir_name_ + "/launch/"+getRobotName()+"_arm_navigation.launch");
}

void PlanningDescriptionConfigurationWizard::updateCollisionsInCurrentState()
{
  lock_.lock();
  std_msgs::ColorRGBA default_color;
  default_color.a = 1.0;
  default_color.r = 0.0;
  default_color.g = .8;
  default_color.b = 0.04;

  collision_markers_.markers.clear();

  cm_->getAllCollisionPointMarkers(*robot_state_, collision_markers_, default_color, ros::Duration(.2));
  lock_.unlock();
}

void PlanningDescriptionConfigurationWizard::sendMarkers()
{
  lock_.lock();
  vis_marker_array_publisher_.publish(collision_markers_);
  if(!current_show_group_.empty())
  {
    MarkerArray arr;
    std_msgs::ColorRGBA default_color;
    default_color.a = 1.0;
    default_color.r = 0.0;
    default_color.g = .8;
    default_color.b = 0.04;

    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.r = 1.0;
    color.g = 0.0;
    color.b = 1.0;

    const KinematicModel::JointModelGroup* jmg = kmodel_->getModelGroup(current_show_group_);

    if(jmg != NULL)
    {
      vector<string> group_link_names = jmg->getGroupLinkNames();
      cm_->getRobotMarkersGivenState(*robot_state_, arr, default_color, current_show_group_, ros::Duration(.2),
                                     &group_link_names, 1.02);

      vector<string> updated_link_model_names = jmg->getUpdatedLinkModelNames();
      map<string, bool> dont_include;
      for(unsigned int i = 0; i < group_link_names.size(); i++)
      {
        dont_include[group_link_names[i]] = true;
      }

      vector<string> ex_list;
      for(unsigned int i = 0; i < updated_link_model_names.size(); i++)
      {
        if(dont_include.find(updated_link_model_names[i]) == dont_include.end())
        {
          ex_list.push_back(updated_link_model_names[i]);
        }
      }
      cm_->getRobotMarkersGivenState(*robot_state_, arr, color, current_show_group_ + "_updated_links",
                                     ros::Duration(.2), &ex_list);
      vis_marker_array_publisher_.publish(arr);
    }
    else
    {
      ROS_ERROR("The joint model group %s did not exist!", current_show_group_.c_str());
    }
  } else if(!current_show_link_.empty()) {
    MarkerArray arr;
    std_msgs::ColorRGBA default_color;
    default_color.a = 1.0;
    default_color.r = 0.0;
    default_color.g = 0.0;
    default_color.b = 1.0;
    
    vector<string> single_link_name(1, current_show_link_);
    cm_->getRobotPaddedMarkersGivenState(*robot_state_, arr, default_color, current_show_link_, ros::Duration(.2),
                                         &single_link_name);
    vis_marker_array_publisher_.publish(arr);
  }
  lock_.unlock();
}

void PlanningDescriptionConfigurationWizard::sendTransforms()
{
  lock_.lock();
  ros::WallTime cur_time = ros::WallTime::now();
  rosgraph_msgs::Clock c;
  c.clock.nsec = cur_time.nsec;
  c.clock.sec = cur_time.sec;
  vector<geometry_msgs::TransformStamped> trans_vector;
  getAllKinematicStateStampedTransforms(*robot_state_, trans_vector, c.clock);
  transform_broadcaster_.sendTransform(trans_vector);
  lock_.unlock();
}
;

bool PlanningDescriptionConfigurationWizard::isInited() const
{
  return inited_;
}

CollisionOperationsGenerator* PlanningDescriptionConfigurationWizard::getOperationsGenerator()
{
  return ops_gen_;
}

string PlanningDescriptionConfigurationWizard::getRobotName()
{
  return urdf_->getName();
}

void PlanningDescriptionConfigurationWizard::dofTogglePushed()
{
  ROS_INFO_STREAM("Pushed");
  unsigned int column = 3;
  vector<int> rows = getSelectedRows(dof_selection_table_);
  for(size_t i = 0; i < rows.size(); i++)
  {
    QCheckBox* box = dynamic_cast<QCheckBox*> (dof_selection_table_->cellWidget(rows[i], column));

    if(box != NULL)
    {
      if(box->isChecked())
      {
        box->setChecked(false);
      }
      else
      {
        box->setChecked(true);
      }
    }
  }
  dofSelectionTableChanged();
}

void PlanningDescriptionConfigurationWizard::dofSelectionTableChanged()
{
  int xind = 0;
  map<string, bool> cdof_map;
  const vector<KinematicModel::JointModel*>& jmv = cm_->getKinematicModel()->getJointModels();
  for(unsigned int i = 1; i < jmv.size(); i++)
  {
    const map<string, pair<double, double> >& joint_bounds = jmv[i]->getAllVariableBounds();
    for(map<string, pair<double, double> >::const_iterator it = joint_bounds.begin(); it != joint_bounds.end(); it++)
    {
      QCheckBox* checkBox = dynamic_cast<QCheckBox*> (dof_selection_table_->cellWidget(xind, 3));

      if(checkBox != NULL)
      {
        cdof_map[it->first] = checkBox->isChecked();
      }
      xind++;
    }
  }
  ops_gen_->generateSamplingStructures(cdof_map);
}

void PlanningDescriptionConfigurationWizard::popupGenericWarning(const char* text)
{
  generic_dialog_label_->setText(text);
  generic_dialog_->show();
  generic_dialog_->setVisible(true);
  generic_dialog_->setModal(true);
  ROS_INFO("Showing warning: %s", text);
}

void PlanningDescriptionConfigurationWizard::writeFiles()
{
  package_directory_ = output_wizard_page_->getPackagePathField();
  int ok = chdir(package_directory_.c_str());

  if(ok != 0)
  {
    ROS_WARN("Unable to change directory to %s", package_directory_.c_str());
    string warning = "Failed to change directory to ";
    warning += package_directory_;
    popupFileFailure(warning.c_str());
    return;
  }

  string del_com = "rm -rf " + dir_name_;
  ok = system(del_com.c_str());

  if(ok != 0)
  {
    ROS_WARN_STREAM("Failed to delete old package!");
    popupFileFailure("Failed to delete existing package.");
  }

  string mdir = "roscreate-pkg " + dir_name_;
  mdir += " planning_environment kinematics_base arm_kinematics_constraint_aware ompl_ros_interface ";
  mdir += "trajectory_filter_server constraint_aware_spline_smoother move_arm";
  ok = system(mdir.c_str());

  if(ok != 0)
  {
    ROS_WARN_STREAM("Failed to create ros package!");
    popupFileFailure("Creating ros package failed.");
    return;
  }

  mdir = "mkdir -p " + dir_name_ + "/config";
  ok = system(mdir.c_str());
  if(ok != 0)
  {
    ROS_WARN_STREAM("Making subdirectory not ok");
    popupFileFailure("Creating subdirectory /config failed.");
    return;
  }

  mdir = "mkdir -p " + dir_name_ + "/launch";
  ok = system(mdir.c_str());
  if(ok != 0)
  {
    ROS_WARN_STREAM("Making subdirectory not ok");
    popupFileFailure("Creating subdirectory /launch failed");
    return;
  }

  if(wizard_mode_ == PlanningDescriptionConfigurationWizard::Easy)
  {
    ROS_INFO("Automatically configuring.");
    AutoConfigureThread* thread = new AutoConfigureThread(this);
    thread->start();
  }

  else
  {
    outputJointLimitsYAML();
    output_wizard_page_->updateProgressBar(10);
    outputOMPLGroupYAML();
    output_wizard_page_->updateProgressBar(20);
    outputPlanningDescriptionYAML();
    output_wizard_page_->updateProgressBar(30);
    outputOMPLLaunchFile();
    output_wizard_page_->updateProgressBar(40);
    outputKinematicsLaunchFiles();
    output_wizard_page_->updateProgressBar(50);
    outputTrajectoryFilterLaunch();
    output_wizard_page_->updateProgressBar(65);
    outputPlanningEnvironmentLaunch();
    output_wizard_page_->updateProgressBar(75);
    outputMoveGroupLaunchFiles();
    outputArmNavigationLaunchFile();
    outputPlanningComponentVisualizerLaunchFile();
    output_wizard_page_->updateProgressBar(100);

    output_wizard_page_->setSuccessfulGeneration();
  }
}

void PlanningDescriptionConfigurationWizard::labelChanged(const char* label)
{
  output_wizard_page_->setProgressLabel(label);
}

void PlanningDescriptionConfigurationWizard::autoConfigure()
{
  progress_ = 0;
  //////////////////////
  // SETUP JOINTS
  //////////////////////
  const vector<KinematicModel::JointModel*>& jmv = cm_->getKinematicModel()->getJointModels();
  vector<bool> consider_dof;
  //assuming that 0th is world joint, which we don't want to include
  for(unsigned int i = 1; i < jmv.size(); i++)
  {
    const map<string, pair<double, double> >& joint_bounds = jmv[i]->getAllVariableBounds();
    for(map<string, pair<double, double> >::const_iterator it = joint_bounds.begin(); it != joint_bounds.end(); it++)
    {
      consider_dof.push_back(true);
    }
  }
  int xind = 0;
  map<string, bool> cdof_map;
  for(unsigned int i = 1; i < jmv.size(); i++)
  {
    const map<string, pair<double, double> >& joint_bounds = jmv[i]->getAllVariableBounds();
    for(map<string, pair<double, double> >::const_iterator it = joint_bounds.begin(); it != joint_bounds.end(); it++)
    {
      cdof_map[it->first] = consider_dof[xind++];
    }
  }
  ops_gen_->generateSamplingStructures(cdof_map);

  progress_ = 1;
  emit changeProgress(1);
  /////////////////////////
  // ADJACENT PAIRS IN COLLISION
  ////////////////////////
  vector<CollisionOperationsGenerator::StringPair> adj_links; 
  ops_gen_->generateAdjacentInCollisionPairs(adj_links);
  ops_gen_->disablePairCollisionChecking(adj_links);
  disable_map_[CollisionOperationsGenerator::ADJACENT] = adj_links;

  emit changeLabel("Finding always in collision pairs....");
  /////////////////////////
  // ALWAYS-DEFAULT IN COLLISION
  ////////////////////////

  vector<CollisionOperationsGenerator::StringPair> always_in_collision;
  vector<CollisionOperationsGenerator::CollidingJointValues> in_collision_joint_values;

  ops_gen_->generateAlwaysInCollisionPairs(always_in_collision, in_collision_joint_values);

  lock_.lock();
  robot_state_->setKinematicStateToDefault();
  lock_.unlock();

  progress_ = 10;
  emit changeProgress(10);
  emit changeLabel("Finding Default In Collision Pairs....");

  ops_gen_->disablePairCollisionChecking(always_in_collision);
  disable_map_[CollisionOperationsGenerator::ALWAYS] = always_in_collision;
  vector<CollisionOperationsGenerator::StringPair> default_in_collision;
  ops_gen_->generateDefaultInCollisionPairs(default_in_collision, in_collision_joint_values);
  ops_gen_->disablePairCollisionChecking(default_in_collision);
  disable_map_[CollisionOperationsGenerator::DEFAULT] = default_in_collision;

  progress_ = 33;
  emit changeProgress(33);
  emit changeLabel("Finding Often In Collision Pairs....");
  ////////////////////////
  // OFTEN IN COLLISION
  ///////////////////////

  vector<CollisionOperationsGenerator::StringPair> often_in_collision;
  vector<double> percentages;
  ops_gen_->generateOftenInCollisionPairs(often_in_collision, percentages, in_collision_joint_values);
  ops_gen_->disablePairCollisionChecking(often_in_collision);
  disable_map_[CollisionOperationsGenerator::OFTEN] = often_in_collision;

  progress_ = 60;
  emit changeProgress(60);
  emit changeLabel("Finding Never In Collision Pairs....");
  ////////////////////////
  // OCC-NEVER IN COLLISION
  ///////////////////////

  vector<CollisionOperationsGenerator::StringPair> in_collision;
  vector<CollisionOperationsGenerator::StringPair> not_in_collision;

  ops_gen_->generateOccasionallyAndNeverInCollisionPairs(in_collision, not_in_collision, percentages,
                                                         in_collision_joint_values);
  disable_map_[CollisionOperationsGenerator::NEVER] = not_in_collision;
  ops_gen_->disablePairCollisionChecking(not_in_collision);
  progress_ = 90;
  emit changeProgress(90);
  emit changeLabel("Writing Files....");

  outputJointLimitsYAML();
  outputOMPLGroupYAML();
  outputPlanningDescriptionYAML();
  outputOMPLLaunchFile();
  outputKinematicsLaunchFiles();
  outputTrajectoryFilterLaunch();
  outputPlanningEnvironmentLaunch();
  outputMoveGroupLaunchFiles();
  outputArmNavigationLaunchFile();
  outputPlanningComponentVisualizerLaunchFile();
  progress_ = 100;
  emit changeProgress(100);
}

int PlanningDescriptionConfigurationWizard::nextId() const
{
  switch (currentId())
  {
  case StartPage:
    return SetupGroupsPage;
  case SetupGroupsPage:
    return setup_groups_page_->nextId();
  case KinematicChainsPage:
    {
      if(!kinematic_chain_page_->getReturnToGroups()) {
        if(wizard_mode_ == PlanningDescriptionConfigurationWizard::Advanced)
        {
          return SelectDOFPage;
        }
        else
        {
          return OutputFilesPage;
        }
      } else {
        return SetupGroupsPage;
      }
    }
  case JointCollectionsPage:
    {
      if(!joint_collections_page_->getReturnToGroups()) {
        if(wizard_mode_ == PlanningDescriptionConfigurationWizard::Advanced)
        {
          return SelectDOFPage;
        }
        else
        {
          return OutputFilesPage;
        }
      } else {
        return SetupGroupsPage;
      }
    }
  case SelectDOFPage:
    return AdjacentLinkPage;
    
  case AdjacentLinkPage:
    return AlwaysInCollisionPage;
    
  case AlwaysInCollisionPage:
    return DefaultInCollisionPage;
    
  case DefaultInCollisionPage:
    return OftenInCollisionPage;
    
  case OftenInCollisionPage:
    return OccasionallyInCollisionPage;
    
  case OccasionallyInCollisionPage:
    return OutputFilesPage;
    
  case OutputFilesPage:
    return -1;
      
  default:
    return -1;
  }
}

void PlanningDescriptionConfigurationWizard::update()
{
  output_wizard_page_->updateProgressBar(progress_);
  if(progress_ >= 100)
  {
    output_wizard_page_->setSuccessfulGeneration();
  }

  QWizard::update();
}

void PlanningDescriptionConfigurationWizard::easyButtonToggled(bool checkState)
{
  if(checkState)
  {
    wizard_mode_ = PlanningDescriptionConfigurationWizard::Easy;
  }
}

void PlanningDescriptionConfigurationWizard::hardButtonToggled(bool checkState)
{
  if(checkState)
  {
    wizard_mode_ = PlanningDescriptionConfigurationWizard::Advanced;
  }
}

void PlanningDescriptionConfigurationWizard::verySafeButtonToggled(bool checkState)
{
  if(checkState)
  {
    ops_gen_->setSafety(CollisionOperationsGenerator::VerySafe);
  }
}

void PlanningDescriptionConfigurationWizard::safeButtonToggled(bool checkState)
{
  if(checkState)
  {
    ops_gen_->setSafety(CollisionOperationsGenerator::Safe);
  }
}

void PlanningDescriptionConfigurationWizard::normalButtonToggled(bool checkState)
{
  if(checkState)
  {
    ops_gen_->setSafety(CollisionOperationsGenerator::Normal);
  }
}

void PlanningDescriptionConfigurationWizard::fastButtonToggled(bool checkState)
{
  if(checkState)
  {
    ops_gen_->setSafety(CollisionOperationsGenerator::Fast);
  }
}

void PlanningDescriptionConfigurationWizard::veryFastButtonToggled(bool checkState)
{
  if(checkState)
  {
    ops_gen_->setSafety(CollisionOperationsGenerator::VeryFast);
  }
}

void PlanningDescriptionConfigurationWizard::visualizeCollision(vector<CollisionOperationsGenerator::CollidingJointValues>& jointValues,
                                                                vector<CollisionOperationsGenerator::StringPair>& pairs,
                                                                int& index, std_msgs::ColorRGBA& color)
{

  if(index >= (int)(pairs.size()) || index < 0 || pairs.size() == 0)
  {
    return;
  }
  lock_.lock();

  robot_state_->setKinematicState(jointValues[index]);

  if(!cm_->isKinematicStateInCollision(*robot_state_))
  {
    ROS_INFO_STREAM("Really should be in collision");
  }

  collision_markers_.markers.clear();

  vector<EnvironmentModel::Contact> coll_space_contacts;
  cm_->getCollisionSpace()->getAllCollisionContacts(coll_space_contacts, 1);

  Marker marker;
  Marker marker2;
  for(unsigned int j = 0; j < coll_space_contacts.size(); j++)
  {
    if((coll_space_contacts[j].body_name_1 == pairs[index].first && coll_space_contacts[j].body_name_2
        == pairs[index].second) || (coll_space_contacts[j].body_name_1 == pairs[index].second
                                    && coll_space_contacts[j].body_name_2 == pairs[index].first))
    {
      marker = transformEnvironmentModelContactInfoMarker(coll_space_contacts[j]);
      marker2 = transformEnvironmentModelContactInfoMarker(coll_space_contacts[j]);
      marker.id = 0;
      marker.id = 1;
      marker.color = color;
      marker.lifetime = ros::Duration(0.2);
      marker2.type = Marker::ARROW;
      marker2.color.r = 1.0;
      marker2.color.g = 0.0;
      marker2.color.b = 0.0;
      marker2.color.a = 1.0;
      marker2.scale.x = 0.5;
      marker2.scale.y = 0.5;
      marker2.scale.z = 0.5;
      marker2.pose.position.z = marker2.pose.position.z+.65;
      marker2.pose.orientation.x = 0.0;
      marker2.pose.orientation.y = 1.0;
      marker2.pose.orientation.z = 0.0;
      marker2.pose.orientation.w = 1.0;
      marker2.lifetime = ros::Duration(0.2);
      collision_markers_.markers.push_back(marker);
      collision_markers_.markers.push_back(marker2);
    } 
  }
  lock_.unlock();

}

void PlanningDescriptionConfigurationWizard::setupQtPages()
{
  initStartPage();

  setup_groups_page_ = new SetupGroupsWizardPage(this);
  setPage(SetupGroupsPage, setup_groups_page_);

  kinematic_chain_page_ = new KinematicChainWizardPage(this);
  setPage(KinematicChainsPage, kinematic_chain_page_);

  joint_collections_page_ = new JointCollectionWizardPage(this);
  setPage(JointCollectionsPage, joint_collections_page_);

  initSelectDofPage();

  adjacent_link_page_ = new CollisionsWizardPage(this, CollisionOperationsGenerator::ADJACENT);
  setPage(AdjacentLinkPage, adjacent_link_page_);

  always_in_collision_page_ = new CollisionsWizardPage(this, CollisionOperationsGenerator::ALWAYS);
  setPage(AlwaysInCollisionPage, always_in_collision_page_);

  default_in_collision_page_ = new CollisionsWizardPage(this, CollisionOperationsGenerator::DEFAULT);
  setPage(DefaultInCollisionPage, default_in_collision_page_);

  often_in_collision_page_ = new CollisionsWizardPage(this, CollisionOperationsGenerator::OFTEN);
  setPage(OftenInCollisionPage, often_in_collision_page_);

  occasionally_in_collision_page_ = new CollisionsWizardPage(this, CollisionOperationsGenerator::NEVER);
  setPage(OccasionallyInCollisionPage, occasionally_in_collision_page_);

  output_wizard_page_ = new OutputWizardPage(this);
  setPage(OutputFilesPage, output_wizard_page_);
 
  generic_dialog_ = new QDialog(this);
  QVBoxLayout* gDialogLayout = new QVBoxLayout(generic_dialog_);
  generic_dialog_label_ = new QLabel(generic_dialog_);
  generic_dialog_label_->setText("Warning!");
  gDialogLayout->addWidget(generic_dialog_label_);
  generic_dialog_->setLayout(gDialogLayout);

  file_failure_dialog_ = new QDialog(this);
  QVBoxLayout* filefailureLayout = new QVBoxLayout(file_failure_dialog_);
  QLabel* fileFailureText = new QLabel(file_failure_dialog_);
  fileFailureText->setText("Failed to create files! Reason: ");
  file_failure_reason_ = new QLabel(this);
  file_failure_reason_->setText("unspecified.");
  filefailureLayout->addWidget(fileFailureText);
  filefailureLayout->addWidget(file_failure_reason_);
  QDialogButtonBox* file_failure_box = new QDialogButtonBox(QDialogButtonBox::Ok);
  filefailureLayout->addWidget(file_failure_box);
  connect(file_failure_box, SIGNAL(accepted()), file_failure_dialog_, SLOT(accept()));
  file_failure_dialog_->setLayout(filefailureLayout);

  confirm_group_replace_dialog_ = new QDialog(this);
  QVBoxLayout* confirm_group_replace_layout = new QVBoxLayout(confirm_group_replace_dialog_);
  confirm_group_text_ = new QLabel(this);
  confirm_group_text_->setText("Really replace group ");
  confirm_group_replace_layout->addWidget(confirm_group_text_);
  QDialogButtonBox* group_button_box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
  confirm_group_replace_layout->addWidget(group_button_box);
  connect(group_button_box, SIGNAL(accepted()), confirm_group_replace_dialog_, SLOT(accept()));
  connect(group_button_box, SIGNAL(rejected()), confirm_group_replace_dialog_, SLOT(reject()));
  confirm_group_replace_dialog_->setLayout(confirm_group_replace_layout);
}

void PlanningDescriptionConfigurationWizard::initStartPage()
{
  start_page_ = new QWizardPage(this);
  start_page_->setTitle("Planning Components Configuration Wizard");
  QHBoxLayout* layout = new QHBoxLayout(start_page_);

  QImage* image = new QImage();
  if(chdir(ros::package::getPath("planning_environment").c_str()) != 0)
  {
    ROS_ERROR("FAILED TO CHANGE PACKAGE TO %s", ros::package::getPath("planning_environment").c_str());
  }

  if(!image->load("./resources/wizard.png"))
  {
    ROS_ERROR("FAILED TO LOAD ./resources/wizard.png");
  }
  ROS_INFO("Loaded Image with %d bytes.", image->byteCount());

  QLabel* imageLabel = new QLabel(start_page_);
  imageLabel->setPixmap(QPixmap::fromImage(*image));
  layout->addWidget(imageLabel);
  imageLabel->setMinimumHeight(10);  
  imageLabel->setMinimumWidth(10);  
  start_page_->setSubTitle(
                                                    "Welcome to the ROS planning components configuration wizard! This wizard will guide you through"
                           " creating a planning configuration for your robot.\nThe robot's URDF location should have been passed into the"
                           " program as a command line on startup.");

  QGroupBox* descBox = new QGroupBox(start_page_);

  QLabel* label = new QLabel(descBox);
  QVBoxLayout* descLayout = new QVBoxLayout(descBox);
  label->setText("After you've selected your robot's planning groups,"
                 "\nand set up collision information this wizard will"
                 "\nautomatically generate a planning stack for your"
                 "\nrobot, and in no time your robot's arms will be "
                 "\nable to plan around obstacles efficiently!");
  //label->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
  label->setMinimumWidth(1); 
  label->setMinimumHeight(1);

  layout->addWidget(imageLabel);
  descLayout->addWidget(label);

  label->setAlignment(Qt::AlignTop);

  //descBox->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Maximum);
  descBox->setAlignment(Qt::AlignTop);

  QGroupBox* modeGroupBox = new QGroupBox(descBox);
  modeGroupBox->setTitle("Select Mode");

  QVBoxLayout* modeGroupBoxLayout = new QVBoxLayout(modeGroupBox);
  QLabel* modeGroupBoxDesc = new QLabel(modeGroupBox);
  modeGroupBoxDesc->setMinimumWidth(1);
  modeGroupBoxDesc->setMinimumHeight(1);
  modeGroupBoxDesc->setText("In Easy mode, your robot will be automatically tested for self-collisions,\nand a planning"
                            " parameter file will be automatically generated.\nAdvanced mode allows you to manually configure collision checking.");
  modeGroupBoxLayout->addWidget(modeGroupBoxDesc);

  QRadioButton* easyButton = new QRadioButton(modeGroupBox);
  easyButton->setText("Easy");
  modeGroupBoxLayout->addWidget(easyButton);
  easyButton->setChecked(true);

  connect(easyButton, SIGNAL(toggled(bool)), this, SLOT(easyButtonToggled(bool)));

  QRadioButton* hardButton = new QRadioButton(modeGroupBox);
  hardButton->setText("Advanced");
  modeGroupBoxLayout->addWidget(hardButton);

  connect(hardButton, SIGNAL(toggled(bool)), this, SLOT(hardButtonToggled(bool)));

  modeGroupBox->setLayout(modeGroupBoxLayout);
  modeGroupBox->setAlignment(Qt::AlignTop | Qt::AlignLeft);

  descLayout->addWidget(modeGroupBox);
  modeGroupBox->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Maximum);

  QGroupBox* safetyGroupBox = new QGroupBox(descBox);
  safetyGroupBox->setTitle("Select Self-collision Sampling Density");

  QVBoxLayout* safetyGroupBoxLayout = new QVBoxLayout(safetyGroupBox);
  QLabel* safetyGroupBoxDesc = new QLabel(safetyGroupBox);
  safetyGroupBoxDesc->setText("Parameterizes the number of joint space samples used to "
                              "\ndetermine whether certain robot poses are in collision."
                              "\nDenser setting swill sample longer, while sparser settings" 
                              "\nare more likely to give efficient collision checking that "
                              "\nmay not detect rare self-collisions.");

  safetyGroupBoxDesc->setMinimumWidth(1);
  safetyGroupBoxDesc->setMinimumHeight(1);
  safetyGroupBoxDesc->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  safetyGroupBoxLayout->addWidget(safetyGroupBoxDesc);

  QRadioButton* verySafeButton = new QRadioButton(safetyGroupBox);
  verySafeButton->setText("Very Dense");
  safetyGroupBoxLayout->addWidget(verySafeButton);
  verySafeButton->setChecked(true);

  connect(verySafeButton, SIGNAL(toggled(bool)), this, SLOT(verySafeButtonToggled(bool)));

  QRadioButton* safeButton = new QRadioButton(safetyGroupBox);
  safeButton->setText("Dense");
  safetyGroupBoxLayout->addWidget(safeButton);

  connect(safeButton, SIGNAL(toggled(bool)), this, SLOT(safeButtonToggled(bool)));

  QRadioButton* normalButton = new QRadioButton(safetyGroupBox);
  normalButton->setText("Normal");
  safetyGroupBoxLayout->addWidget(normalButton);
  normalButton->setChecked(true);

  connect(normalButton, SIGNAL(toggled(bool)), this, SLOT(normalButtonToggled(bool)));

  QRadioButton* fastButton = new QRadioButton(safetyGroupBox);
  fastButton->setText("Sparse");
  safetyGroupBoxLayout->addWidget(fastButton);

  connect(fastButton, SIGNAL(toggled(bool)), this, SLOT(fastButtonToggled(bool)));

  QRadioButton* veryFastButton = new QRadioButton(safetyGroupBox);
  veryFastButton->setText("Very Sparse");
  safetyGroupBoxLayout->addWidget(veryFastButton);

  connect(veryFastButton, SIGNAL(toggled(bool)), this, SLOT(veryFastButtonToggled(bool)));


  safetyGroupBox->setLayout(safetyGroupBoxLayout);
  safetyGroupBox->setAlignment(Qt::AlignTop | Qt::AlignLeft);

  descLayout->addWidget(safetyGroupBox);
  //safetyGroupBox->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Maximum);

  layout->addWidget(descBox);

  setPage(StartPage, start_page_);
  start_page_->setLayout(layout);
  start_page_->setMinimumWidth(400);
  start_page_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
}

void PlanningDescriptionConfigurationWizard::initSelectDofPage()
{
  select_dof_page_ = new QWizardPage(this);
  select_dof_page_->setTitle("DOF Sampling");
  QVBoxLayout* layout = new QVBoxLayout(select_dof_page_);
  select_dof_page_->setSubTitle("Select degrees of freedom to sample for collisions. The wizard will run your robot"
                                " through a set of subsamples of these joints and check each pair of links for"
                                " collisions. Unselected joints will remain in their default positions"
                                " during these tests.");

  dof_selection_table_ = new QTableWidget(select_dof_page_);
  layout->addWidget(dof_selection_table_);

  QPushButton* toggleSelected = new QPushButton(select_dof_page_);
  toggleSelected->setText("Toggle Selected");
  layout->addWidget(toggleSelected);
  toggleSelected->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  connect(toggleSelected, SIGNAL(clicked()), this, SLOT(dofTogglePushed()));

  setPage(SelectDOFPage, select_dof_page_);
  select_dof_page_->setLayout(layout);

  createDofPageTable();

}

void PlanningDescriptionConfigurationWizard::createDofPageTable()
{
  const vector<KinematicModel::JointModel*>& jmv = cm_->getKinematicModel()->getJointModels();
  vector<bool> consider_dof;

  int numDofs = 0;
  //assuming that 0th is world joint, which we don't want to include
  for(unsigned int i = 1; i < jmv.size(); i++)
  {
    const map<string, pair<double, double> >& joint_bounds = jmv[i]->getAllVariableBounds();
    for(map<string, pair<double, double> >::const_iterator it = joint_bounds.begin(); it != joint_bounds.end(); it++)
    {
      consider_dof.push_back(true);
      numDofs++;
    }
  }

  dof_selection_table_->clear();
  dof_selection_table_->setRowCount(numDofs);
  dof_selection_table_->setColumnCount(4);
  dof_selection_table_->setColumnWidth(0, 300);
  dof_selection_table_->setColumnWidth(1, 300);
  dof_selection_table_->setColumnWidth(2, 300);
  dof_selection_table_->setColumnWidth(3, 300);

  QStringList horizontalLabels;
  horizontalLabels.append("DOF Name");
  horizontalLabels.append("Lower Bound (radians)");
  horizontalLabels.append("Upper Bound (radians)");
  horizontalLabels.append("Consider DOF?");
  dof_selection_table_->setHorizontalHeaderLabels(horizontalLabels);

  int ind = 1;
  for(unsigned int i = 1; i < jmv.size(); i++)
  {
    const map<string, pair<double, double> >& joint_bounds = jmv[i]->getAllVariableBounds();

    for(map<string, pair<double, double> >::const_iterator it = joint_bounds.begin(); it != joint_bounds.end(); it++)
    {
      stringstream lowerStream;
      stringstream upperStream;
      lowerStream << it->second.first;
      upperStream << it->second.second;

      QTableWidgetItem* nameItem = new QTableWidgetItem(it->first.c_str());
      nameItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

      QTableWidgetItem* lowerItem = new QTableWidgetItem(lowerStream.str().c_str());
      lowerItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

      QTableWidgetItem* upperItem = new QTableWidgetItem(upperStream.str().c_str());
      upperItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

      QCheckBox* check = new QCheckBox(dof_selection_table_);
      check->setChecked(consider_dof[ind - 1]);

      connect(check, SIGNAL(toggled(bool)), this, SLOT(dofSelectionTableChanged()));
      dof_selection_table_->setItem(ind - 1, 0, nameItem);
      dof_selection_table_->setItem(ind - 1, 1, lowerItem);
      dof_selection_table_->setItem(ind - 1, 2, upperItem);
      dof_selection_table_->setCellWidget(ind - 1, 3, check);
      ind++;
    }
  }

  dofSelectionTableChanged();
}
          
SetupGroupsWizardPage::SetupGroupsWizardPage(PlanningDescriptionConfigurationWizard *parent) 
  : QWizardPage(parent), next_from_groups_id_(PlanningDescriptionConfigurationWizard::OutputFilesPage)
{
  parent_ = parent;

  setTitle("Planning Group Setup");

  QGridLayout* layout = new QGridLayout(this);
  setSubTitle("Select planning groups for your robot based on kinematic chains, or joint collections."
              " When you are finished, please check the checkbox and you can move on by pressing Next.");
  
  QGroupBox* selectGroupsBox = new QGroupBox(this);
  selectGroupsBox->setTitle("Current Groups");

  current_group_table_ = new QTableWidget(selectGroupsBox);
  QVBoxLayout* groupBoxLayout = new QVBoxLayout(selectGroupsBox);
  groupBoxLayout->addWidget(current_group_table_);
  selectGroupsBox->setLayout(groupBoxLayout);
  layout->addWidget(selectGroupsBox, 0, 0, 1, 1);
  QPushButton* deleteButton = new QPushButton(selectGroupsBox);
  deleteButton->setText("Delete");
  groupBoxLayout->addWidget(deleteButton);
  deleteButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  
  connect(current_group_table_, SIGNAL(itemClicked(QTableWidgetItem*)), this, SLOT(groupTableClicked()));
  connect(deleteButton, SIGNAL(clicked()), this, SLOT(deleteGroupButtonClicked()));

  QGroupBox* modeBox = new QGroupBox(this);
  modeBox->setTitle("Add groups");
  QVBoxLayout* modeBoxLayout = new QVBoxLayout(modeBox);
  QPushButton* add_kinematic_chain_button = new QPushButton(tr("&Add Kinematic Chain Group"));
  QPushButton* add_joint_collection_button = new QPushButton(tr("&Add Joint Collection Group"));
  modeBoxLayout->addWidget(add_kinematic_chain_button);
  modeBoxLayout->addWidget(add_joint_collection_button);

  first_group_field_ = new QLineEdit(this);
  registerField("first_group*", first_group_field_);
  first_group_field_->hide();
  
  connect(add_kinematic_chain_button, SIGNAL(clicked()), this, SLOT(addKinematicChainGroup()));
  connect(add_joint_collection_button, SIGNAL(clicked()), this, SLOT(addJointCollectionGroup()));
  
  modeBox->setLayout(modeBoxLayout);
  layout->addWidget(modeBox, 0, 2, 1, 1);
  
  setLayout(layout);
  layout->setAlignment(modeBox, Qt::AlignTop);
  
  setButtonText(QWizard::NextButton, "Done with groups");
}


void SetupGroupsWizardPage::groupTableClicked()
{
  QList<QTableWidgetItem*> selected = current_group_table_->selectedItems();
  
  if(selected.size() == 0)
  {
    return;
  }
  else
  {
    QTableWidgetItem* first = selected[0];
    int row = first->row();
    QTableWidgetItem* groupName = current_group_table_->item(row, 0);
    parent_->setCurrentShowGroup(groupName->text().toStdString());
  }
}


void SetupGroupsWizardPage::updateGroupTable()
{
  current_group_table_->clear();
  first_group_field_->clear();
  
  vector<string> modelNames;
  parent_->getKinematicModel()->getModelGroupNames(modelNames);
  current_group_table_->setRowCount((int)modelNames.size());
  current_group_table_->setColumnCount(1);
  current_group_table_->setColumnWidth(0, 300);
  current_group_table_->setDragEnabled(false);
  current_group_table_->setHorizontalHeaderItem(0, new QTableWidgetItem("Planning Groups"));
  if(!modelNames.empty()) {
    first_group_field_->setText(modelNames[0].c_str());
  }
  for(size_t i = 0; i < modelNames.size(); i++)
  {
    QTableWidgetItem* item = new QTableWidgetItem(modelNames[i].c_str());
    current_group_table_->setItem((int)i, 0, item);
    item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
  }
}

void SetupGroupsWizardPage::deleteGroupButtonClicked()
{
  QList<QTableWidgetItem*> itemList = current_group_table_->selectedItems();

  for(int i = 0; i < itemList.size(); i++)
  {
    if(parent_->getKinematicModel()->hasModelGroup(itemList[i]->text().toStdString()))
    {
      parent_->removeGroup(itemList[i]->text().toStdString());
    }
  }
  updateGroupTable();
}

int SetupGroupsWizardPage::nextId() const {
  if(next_from_groups_id_ == PlanningDescriptionConfigurationWizard::OutputFilesPage)
  {
    if(parent_->getWizardMode() == PlanningDescriptionConfigurationWizard::Advanced)
    {
      return PlanningDescriptionConfigurationWizard::SelectDOFPage;
    }
    else
    {
      return PlanningDescriptionConfigurationWizard::OutputFilesPage;
    }
  }
  return next_from_groups_id_;
}

void SetupGroupsWizardPage::addKinematicChainGroup() {
  next_from_groups_id_ = PlanningDescriptionConfigurationWizard::KinematicChainsPage;
  parent_->next();
  next_from_groups_id_ = PlanningDescriptionConfigurationWizard::OutputFilesPage;
}

void SetupGroupsWizardPage::addJointCollectionGroup() {
  next_from_groups_id_ = PlanningDescriptionConfigurationWizard::JointCollectionsPage;
  parent_->next();
  next_from_groups_id_ = PlanningDescriptionConfigurationWizard::OutputFilesPage;
}

KinematicChainWizardPage::KinematicChainWizardPage(PlanningDescriptionConfigurationWizard *parent) 
  : QWizardPage(parent), return_to_groups_(false)
{
  parent_ = parent;
  setTitle("Select Kinematic Chain");

  QGridLayout* layout = new QGridLayout(this);
  setSubTitle("Select a planning group based on a kinematic chain."
              " Select a base link (the first link in the chain) and a tip link."
              " They must be connected by a direct line of joints.");
  
  QImage* image = new QImage();
  if(!image->load("./resources/chains.png"))
  {
    ROS_ERROR("FAILED TO LOAD ./resources/chains.png");
  }
  ROS_INFO("Loaded Image with %d bytes.", image->byteCount());

  QLabel* imageLabel = new QLabel(this);
  imageLabel->setPixmap(QPixmap::fromImage(*image));
  imageLabel->setAlignment(Qt::AlignTop);
  imageLabel->setMinimumHeight(10);  
  imageLabel->setMinimumWidth(10);  
  layout->addWidget(imageLabel, 0, 0, 0, 1);
  QGroupBox* treeBox = new QGroupBox(this);
  treeBox->setTitle("Links");
  layout->addWidget(treeBox, 0, 1, 1, 1);

  QVBoxLayout* treeLayout = new QVBoxLayout(treeBox);
  link_tree_ = new QTreeWidget(treeBox);
  connect(link_tree_, SIGNAL(itemSelectionChanged()), this, SLOT(showTreeLink()));
  treeLayout->addWidget(link_tree_);
  QPushButton* baseLinkButton = new QPushButton(treeBox);
  baseLinkButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  baseLinkButton->setText("Select Base Link");
  treeLayout->addWidget(baseLinkButton);
  QPushButton* tipLinkButton = new QPushButton(treeBox);
  tipLinkButton->setText("Select Tip Link");
  tipLinkButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  treeLayout->addWidget(tipLinkButton);
  treeBox->setLayout(treeLayout);
  createLinkTree(parent->getKinematicModel());

  connect(baseLinkButton, SIGNAL(clicked()), this, SLOT(baseLinkTreeClick()));
  connect(tipLinkButton, SIGNAL(clicked()), this, SLOT(tipLinkTreeClick()));
  QGroupBox* chainBox = new QGroupBox(this);
  chainBox->setTitle("Kinematic Chain");
  QFormLayout* chainLayout = new QFormLayout(chainBox);
  chain_name_field_ = new QLineEdit(chainBox);
  base_link_field_ = new QLineEdit(chainBox);
  tip_link_field_ = new QLineEdit(chainBox);
  chainLayout->addRow("Chain Name", chain_name_field_);
  chainLayout->addRow("Base Link", base_link_field_);
  chainLayout->addRow("Tip Link", tip_link_field_);
  layout->addWidget(chainBox, 1, 1, 1, 2);

  registerField("chain_name*", chain_name_field_);
  registerField("base_link*", base_link_field_);
  registerField("tip_link*", tip_link_field_);

  setLayout(layout);

  good_group_dialog_ = new QDialog(this);
  QVBoxLayout* good_group_layout = new QVBoxLayout(good_group_dialog_);
  QLabel* good_dialog_text = new QLabel(good_group_dialog_);
  good_dialog_text->setText("Your group is valid and has been added to the planning groups.\nIt should be visualized in Rviz");
  good_group_layout->addWidget(good_dialog_text);
  QPushButton* return_to_groups_button = new QPushButton(tr("&Return To Groups"));
  QPushButton* add_another_kinematic_chain_button = new QPushButton(tr("&Add Another Kinematic Chain"));
  QPushButton* done_with_groups_button = new QPushButton(tr("&Done Adding Groups"));
  QDialogButtonBox* good_button_box = new QDialogButtonBox();
  good_group_layout->addWidget(good_button_box);
  good_button_box->addButton(return_to_groups_button, QDialogButtonBox::RejectRole);
  good_button_box->addButton(add_another_kinematic_chain_button, QDialogButtonBox::ResetRole);
  good_button_box->addButton(done_with_groups_button, QDialogButtonBox::AcceptRole);
  connect(good_button_box, SIGNAL(rejected()), good_group_dialog_, SLOT(reject()));
  connect(good_button_box, SIGNAL(rejected()), parent, SLOT(back()));
  connect(good_button_box, SIGNAL(accepted()), good_group_dialog_, SLOT(accept()));
  connect(add_another_kinematic_chain_button, SIGNAL(clicked()), this, SLOT(resetPage()));
  good_group_dialog_->setLayout(good_group_layout);

  not_ok_dialog_ = new QDialog(this);
  QVBoxLayout* notOkDialogLayout = new QVBoxLayout(not_ok_dialog_);
  QLabel* notOkText = new QLabel(not_ok_dialog_);
  notOkText->setText("Error! The planning group was invalid!");
  notOkDialogLayout->addWidget(notOkText);
  QDialogButtonBox* ok_button_box = new QDialogButtonBox(QDialogButtonBox::Ok);
  notOkDialogLayout->addWidget(ok_button_box);
  connect(ok_button_box, SIGNAL(accepted()), not_ok_dialog_, SLOT(accept()));
  not_ok_dialog_->setLayout(notOkDialogLayout);

  setButtonText(QWizard::NextButton, "Add Group");
}

bool KinematicChainWizardPage::validatePage() {
  
  KinematicModel::GroupConfig gc(getChainNameField(), getBaseLinkField(), getTipLinkField());

  PlanningDescriptionConfigurationWizard::GroupAddStatus stat = 
    parent_->addGroup(gc);

  if(stat == PlanningDescriptionConfigurationWizard::GroupAddSuccess) {
    int val = good_group_dialog_->exec();
    parent_->setCurrentShowGroup("");
    parent_->setCurrentShowLink("");
    if(val == QDialog::Accepted) {
      return true;
    } else if(val == QDialog::Rejected) {
      return false;
    } else {
      return false;
    }
  } else if(stat == PlanningDescriptionConfigurationWizard::GroupAddFailed) {
    not_ok_dialog_->exec();
    return false;
  } else {
    //cancelled duplicated
    return false;
  }
}

void KinematicChainWizardPage::showTreeLink() {
  QTreeWidgetItem* item = link_tree_->currentItem();
  if(item != NULL)
  {
    std::string link_name = item->text(0).toStdString();
    if(!link_name.empty()) {
      parent_->setCurrentShowGroup("");
      parent_->setCurrentShowLink(link_name);
    }
  }
}

void KinematicChainWizardPage::baseLinkTreeClick() {
  QTreeWidgetItem* item = link_tree_->currentItem();
  if(item != NULL)
  {
    base_link_field_->setText(item->text(0));
  }
}

void KinematicChainWizardPage::tipLinkTreeClick()
{
  QTreeWidgetItem* item = link_tree_->currentItem();
  if(item != NULL)
  {
    tip_link_field_->setText(item->text(0));
  }
}

void KinematicChainWizardPage::createLinkTree(const planning_models::KinematicModel* kmodel)
{
  const KinematicModel::JointModel* rootJoint = kmodel->getRoot();
  addLinktoTreeRecursive(rootJoint->getChildLinkModel(), NULL);

  link_tree_->expandToDepth(0);
  link_tree_->header()->setResizeMode(0, QHeaderView::ResizeToContents);
  link_tree_->header()->setStretchLastSection(false);
}



void KinematicChainWizardPage::addLinktoTreeRecursive(const KinematicModel::LinkModel* link,
                                                      const KinematicModel::LinkModel* parent)
{
  QTreeWidgetItem* toAdd = new QTreeWidgetItem(link_tree_);
  if(parent == NULL)
  {
    toAdd->setText(0, link->getName().c_str());
    link_tree_->addTopLevelItem(toAdd);
  }
  else
  {
    for(int i = 0; i < link_tree_->topLevelItemCount(); i++)
    {
      if(addLinkChildRecursive(link_tree_->topLevelItem(i), link, parent->getName()))
      {
        break;
      }
    }
  }
  for(size_t i = 0; i < link->getChildJointModels().size(); i++)
  {
    addLinktoTreeRecursive(link->getChildJointModels()[i]->getChildLinkModel(), link);
  }
}

bool KinematicChainWizardPage::addLinkChildRecursive(QTreeWidgetItem* parent,
                                                     const KinematicModel::LinkModel* link,
                                                     const string& parentName)
{
  if(parent->text(0).toStdString() == parentName)
  {
    QTreeWidgetItem* toAdd = new QTreeWidgetItem(parent);
    toAdd->setText(0, link->getName().c_str());
    parent->addChild(toAdd);
    return true;
  }
  else
  {
    for(int i = 0; i < parent->childCount(); i++)
    {
      if(addLinkChildRecursive(parent->child(i), link, parentName))
      {
        return true;
      }
    }
  }

  return false;
}

JointCollectionWizardPage::JointCollectionWizardPage(PlanningDescriptionConfigurationWizard *parent) 
  : QWizardPage(parent), return_to_groups_(false)
{ 
  parent_ = parent;
  setTitle("Select Joint Collections");

  QGridLayout* layout = new QGridLayout(this);

  setSubTitle("Select an arbitrary group of joints to form a planning group.");

  QImage* image = new QImage();
  if(!image->load("./resources/groups.png"))
  {
    ROS_ERROR("FAILED TO LOAD ./resources/groups.png");
  }

  QLabel* imageLabel = new QLabel(this);
  imageLabel->setPixmap(QPixmap::fromImage(*image));
  imageLabel->setAlignment(Qt::AlignTop);
  imageLabel->setMinimumHeight(10);  
  imageLabel->setMinimumWidth(10);  

  layout->addWidget(imageLabel, 0, 1, 1, 1);

  QGroupBox* jointBox = new QGroupBox(this);
  jointBox->setTitle("Joints");


  QVBoxLayout* jointLayout = new QVBoxLayout(jointBox);
  joint_table_ = new QTableWidget(jointBox);
  jointLayout->addWidget(joint_table_);
  jointBox->setLayout(jointLayout);
  layout->addWidget(jointBox, 0, 0, 3, 1);

  QPushButton* selectButton = new QPushButton(jointBox);
  selectButton->setText("v Select v");
  jointLayout->addWidget(selectButton);
  selectButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  first_joint_field_ = new QLineEdit(this);
  registerField("first_joint*", first_joint_field_);
  first_joint_field_->hide();

  connect(selectButton, SIGNAL(clicked()), this, SLOT(selectJointButtonClicked()));

  QGroupBox* selectedBox = new QGroupBox(this);
  selectedBox->setTitle("Joint Group");

  QVBoxLayout* selectedLayout = new QVBoxLayout(selectedBox);
  selected_joint_table_ = new QTableWidget(jointBox);
  jointLayout->addWidget(selected_joint_table_);

  QPushButton* deselectButton = new QPushButton(jointBox);
  deselectButton->setText("^ Deselect ^");
  jointLayout->addWidget(deselectButton);
  deselectButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  connect(deselectButton, SIGNAL(clicked()), this, SLOT(deselectJointButtonClicked()));

  QLabel* jointGroupLabel = new QLabel(selectedBox);
  jointGroupLabel->setText(" Joint Group Name: ");
  joint_group_name_field_ = new QLineEdit(selectedBox);
  selectedLayout->addWidget(jointGroupLabel);
  selectedLayout->addWidget(joint_group_name_field_);
  selectedBox->setLayout(selectedLayout);
  layout->addWidget(selectedBox, 1, 1, 1, 1);
  setLayout(layout);

  createJointCollectionTables();

  good_group_dialog_ = new QDialog(this);
  QVBoxLayout* good_group_layout = new QVBoxLayout(good_group_dialog_);
  QLabel* good_dialog_text = new QLabel(good_group_dialog_);
  good_dialog_text->setText("Your group is valid and has been added to the planning groups.\nIt should be visualized in Rviz");
  good_group_layout->addWidget(good_dialog_text);
  QPushButton* return_to_groups_button = new QPushButton(tr("&Return To Groups"));
  QPushButton* add_another_kinematic_chain_button = new QPushButton(tr("&Add Another Joint Collection"));
  QPushButton* done_with_groups_button = new QPushButton(tr("&Done Adding Groups"));
  QDialogButtonBox* good_button_box = new QDialogButtonBox();
  good_group_layout->addWidget(good_button_box);
  good_button_box->addButton(return_to_groups_button, QDialogButtonBox::RejectRole);
  good_button_box->addButton(add_another_kinematic_chain_button, QDialogButtonBox::ResetRole);
  good_button_box->addButton(done_with_groups_button, QDialogButtonBox::AcceptRole);
  connect(good_button_box, SIGNAL(rejected()), good_group_dialog_, SLOT(reject()));
  connect(good_button_box, SIGNAL(rejected()), parent, SLOT(back()));
  connect(good_button_box, SIGNAL(accepted()), good_group_dialog_, SLOT(accept()));
  connect(add_another_kinematic_chain_button, SIGNAL(clicked()), this, SLOT(resetPage()));
  good_group_dialog_->setLayout(good_group_layout);

  not_ok_dialog_ = new QDialog(this);
  QVBoxLayout* notOkDialogLayout = new QVBoxLayout(not_ok_dialog_);
  QLabel* notOkText = new QLabel(not_ok_dialog_);
  notOkText->setText("Error! The planning group was invalid!");
  notOkDialogLayout->addWidget(notOkText);
  QDialogButtonBox* ok_button_box = new QDialogButtonBox(QDialogButtonBox::Ok);
  notOkDialogLayout->addWidget(ok_button_box);
  connect(ok_button_box, SIGNAL(accepted()), not_ok_dialog_, SLOT(accept()));
  not_ok_dialog_->setLayout(notOkDialogLayout);

  setButtonText(QWizard::NextButton, "Add Group");
  registerField("collection_name*", joint_group_name_field_);


}

bool JointCollectionWizardPage::validatePage() {
  
  string new_group_name = joint_group_name_field_->text().toStdString();
  vector<string> joints;
  for(int i = 0; i < selected_joint_table_->rowCount(); i++)
  {
    joints.push_back(selected_joint_table_->item(i, 0)->text().toStdString());
  }
  
  vector<string> subgroups;
  KinematicModel::GroupConfig gc(new_group_name, joints, subgroups);

  PlanningDescriptionConfigurationWizard::GroupAddStatus stat = 
    parent_->addGroup(gc);

  if(stat == PlanningDescriptionConfigurationWizard::GroupAddSuccess) {
    int val = good_group_dialog_->exec();
    parent_->setCurrentShowGroup("");
    if(val == QDialog::Accepted) {
      return true;
    } else if(val == QDialog::Rejected) {
      return false;
    } else {
      return false;
    }
  } else if(stat == PlanningDescriptionConfigurationWizard::GroupAddFailed) {
    not_ok_dialog_->exec();
    return false;
  } else {
    //cancelled duplicated
    return false;
  }
}

void JointCollectionWizardPage::selectJointButtonClicked()
{
  QList<QTableWidgetItem*> selected = joint_table_->selectedItems();

  for(int i = 0; i < selected.size(); i++)
  {
    string name = selected[i]->text().toStdString();
    bool alreadyExists = false;
    int rowToAdd = 0;
    for(int r = 0; r < selected_joint_table_->rowCount(); r++)
    {
      QTableWidgetItem* item = selected_joint_table_->item(r, 0);

      if(item->text().toStdString() == name)
      {
        alreadyExists = true;
        break;
      }
      rowToAdd = r + 1;
    }

    if(!alreadyExists)
    {
      selected_joint_table_->setRowCount(selected_joint_table_->rowCount() + 1);
      QTableWidgetItem* newItem = new QTableWidgetItem(name.c_str());
      newItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
      selected_joint_table_->setItem(rowToAdd, 0, newItem);
    }
  }
  if(selected_joint_table_->rowCount() > 0) {
    first_joint_field_->setText("has");
  }
}

void JointCollectionWizardPage::deselectJointButtonClicked()
{
  QList<QTableWidgetItem*> deselected = selected_joint_table_->selectedItems();

  for(int i = 0; i < deselected.size(); i++)
  {
    selected_joint_table_->removeRow(deselected[i]->row());
  }
  if(selected_joint_table_->rowCount() == 0) {
    first_joint_field_->clear();
  }
}

void JointCollectionWizardPage::createJointCollectionTables()
{
  const vector<KinematicModel::JointModel*>& jmv = parent_->getKinematicModel()->getJointModels();

  joint_table_->setRowCount((int)jmv.size());
  joint_table_->setColumnCount(1);
  joint_table_->setColumnWidth(0, 300);
  selected_joint_table_->setColumnCount(1);
  selected_joint_table_->setColumnWidth(0, 300);
  QStringList headerLabels;
  headerLabels.append("Joint");
  QStringList headerLabels2;
  headerLabels2.append("Selected Joints");
  joint_table_->setHorizontalHeaderLabels(headerLabels);
  selected_joint_table_->setHorizontalHeaderLabels(headerLabels2);
  for(size_t i = 1; i < jmv.size(); i++)
  {
    QTableWidgetItem* item = new QTableWidgetItem(jmv[i]->getName().c_str());
    item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    joint_table_->setItem((int)i - 1, 0, item);
  }
}

CollisionsWizardPage::CollisionsWizardPage(PlanningDescriptionConfigurationWizard *parent,
                                           CollisionOperationsGenerator::DisableType disable_type)
  : QWizardPage(parent), disable_type_(disable_type), 
    allow_enable_(true), show_percentages_(false), is_clickable_(true), coll_default_enabled_(false)
{
  parent_ = parent;
  std::string title = "Links ";
  std::string subtitle;
  if(disable_type_ == CollisionOperationsGenerator::ALWAYS) {
    title += "Always";
    subtitle = "The following links are always in collision over the sample space.\nClick on a link name to visualize the collision in Rviz";
    allow_enable_ = false;
  } else if(disable_type_ == CollisionOperationsGenerator::DEFAULT) {
    subtitle = "The following links are in collision in the default state, and will be disabled by default.\nClick on a link name to visualize the collision in Rviz, and check Enable to force self-collision checking for the indicated pair.";
    title += "Default";
  } else if(disable_type_ == CollisionOperationsGenerator::OFTEN) {
    title += "Often";
    subtitle = "The following links are in collision in the majority of sampled states.  Enabling them will substantially slow collision checking.\nClick on a link name to visualize the collision in Rviz, and check Enable to force self-collision checking for the indicated pair.";
    show_percentages_ = true;
  } else if(disable_type == CollisionOperationsGenerator::ADJACENT) { 
    is_clickable_ = false;
    title += "Adjacent";
    subtitle = "The following links are adjacent in the kinematic tree and will be disabled by default.\nCheck Enable to force self-collision checking for the indicated pair";
  } else {
    coll_default_enabled_ = true;
    show_percentages_ = true;
    subtitle = "The following links are in the indicated percentage of samples. By default, link pairs that were found to be in collision in one or more samples have collision checking enabled.\nLink pairs that were never found to be in collision have been disabled.\nClick on a link name to visualize the collision in Rviz.";
    title += "Occasionally and Never";
  }
  title += " In Collision";
  setTitle(title.c_str());

  QVBoxLayout* layout = new QVBoxLayout(this);
  setSubTitle(subtitle.c_str());
  QPushButton* generateButton = new QPushButton(this);
  generateButton->setText("Generate List (May take a minute)");
  layout->addWidget(generateButton);
  connect(generateButton, SIGNAL(clicked()), this, SLOT(generateCollisionTable()));
  generateButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  collision_table_ = new QTableWidget(this);
  layout->addWidget(collision_table_);

  if(allow_enable_) {
    QPushButton* toggle_selected = new QPushButton(this);
    toggle_selected->setText("Toggle Selected");
    layout->addWidget(toggle_selected);
    connect(toggle_selected, SIGNAL(clicked()), this, SLOT(toggleTable()));
    toggle_selected->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  }
  if(is_clickable_) {
    connect(collision_table_, SIGNAL(cellClicked(int, int)), this, SLOT(tableClicked()));
  }
  setLayout(layout);
}

void CollisionsWizardPage::generateCollisionTable() {
  vector<CollisionOperationsGenerator::StringPair> not_in_collision;
  vector<double> percentages;
  if(disable_type_ == CollisionOperationsGenerator::ALWAYS) {
    parent_->getOperationsGenerator()->generateAlwaysInCollisionPairs(collision_pairs_, in_collision_joint_values_);
  } else if(disable_type_ == CollisionOperationsGenerator::DEFAULT) {
    parent_->getOperationsGenerator()->generateDefaultInCollisionPairs(collision_pairs_, in_collision_joint_values_);
  } else if(disable_type_ == CollisionOperationsGenerator::OFTEN) {
    parent_->getOperationsGenerator()->generateOftenInCollisionPairs(collision_pairs_, percentages, in_collision_joint_values_);
  } else if(disable_type_ == CollisionOperationsGenerator::ADJACENT) { 
    parent_->getOperationsGenerator()->generateAdjacentInCollisionPairs(collision_pairs_);
  } else {
    parent_->getOperationsGenerator()->generateOccasionallyAndNeverInCollisionPairs(collision_pairs_, not_in_collision,
                                                                                    percentages, in_collision_joint_values_);
  }

  if(disable_type_ == CollisionOperationsGenerator::NEVER ||
     disable_type_ == CollisionOperationsGenerator::OCCASIONALLY) {
    disable_pairs_ = not_in_collision;
    enable_pairs_ = collision_pairs_;
  } else {
    disable_pairs_ = collision_pairs_;
  }

  collision_table_->clear();
  collision_table_->setRowCount((int)collision_pairs_.size()+(int)not_in_collision.size());
  collision_table_->setColumnCount(2);
  collision_table_->setColumnWidth(0, 500);
  collision_table_->setColumnWidth(1, 500);
  QStringList titleList;
  titleList.append("Link A");
  titleList.append("Link B");
  if(show_percentages_) {
    titleList.append("% Colliding");
  }
  if(allow_enable_) {
    titleList.append("Enable?");
  }
  if(show_percentages_ && allow_enable_) {
    collision_table_->setColumnCount(4);
    
    collision_table_->setColumnWidth(0, 300);
    collision_table_->setColumnWidth(1, 300);
    collision_table_->setColumnWidth(2, 200);
    collision_table_->setColumnWidth(3, 200);
  } else if(allow_enable_) {
    collision_table_->setColumnCount(3);
    
    collision_table_->setColumnWidth(0, 300);
    collision_table_->setColumnWidth(1, 300);
    collision_table_->setColumnWidth(2, 300);
  }
  //shouldn't ever have percentages without allowed enable

  collision_table_->setHorizontalHeaderLabels(titleList);
  if(collision_pairs_.size() + not_in_collision.size() == 0) {
    collision_table_->setRowCount(1);
    QTableWidgetItem* no_collide = new QTableWidgetItem("No Link Pairs Of This Kind");
    collision_table_->setItem(0, 0, no_collide);
  }

  bool bad_percentages = false;
  if(show_percentages_ && collision_pairs_.size() != percentages.size()) {
    ROS_WARN_STREAM("Percentages enabled but percentage size " << percentages.size() 
                    << " not equal to collision pairs size " << collision_pairs_.size());
    bad_percentages = true;
  }

  for(size_t i = 0; i < collision_pairs_.size(); i++)
  {
    QTableWidgetItem* linkA = new QTableWidgetItem(collision_pairs_[i].first.c_str());
    linkA->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    QTableWidgetItem* linkB = new QTableWidgetItem(collision_pairs_[i].second.c_str());
    linkB->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

    collision_table_->setItem((int)i, 0, linkA);
    collision_table_->setItem((int)i, 1, linkB);

    if(show_percentages_) {
      stringstream percentageStream;
      if(bad_percentages) {
        percentageStream << 1.0;
      } else {
        percentageStream << percentages[i];
      }
      QTableWidgetItem* percentage = new QTableWidgetItem(percentageStream.str().c_str());
      percentage->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
      collision_table_->setItem((int)i, 2, percentage);
    }

    if(allow_enable_) {
      QCheckBox* enable_box = new QCheckBox(collision_table_);
      if(coll_default_enabled_) {
        enable_box->setChecked(true);
      } else {
        enable_box->setChecked(false);
      }
      connect(enable_box, SIGNAL(toggled(bool)), this, SLOT(tableChanged()));
      if(show_percentages_) {
        collision_table_->setCellWidget((int)i, 3, enable_box); 
      } else {
        collision_table_->setCellWidget((int)i, 2, enable_box);
      }
    }
  }
  for(size_t i = 0; i < not_in_collision.size(); i++)
  {
    QTableWidgetItem* linkA = new QTableWidgetItem(not_in_collision[i].first.c_str());
    linkA->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    QTableWidgetItem* linkB = new QTableWidgetItem(not_in_collision[i].second.c_str());
    linkB->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

    collision_table_->setItem((int)i+collision_pairs_.size(), 0, linkA);
    collision_table_->setItem((int)i+collision_pairs_.size(), 1, linkB);

    if(show_percentages_) {
      stringstream percentageStream;
      percentageStream << 0.0;
      QTableWidgetItem* percentage = new QTableWidgetItem(percentageStream.str().c_str());
      percentage->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
      collision_table_->setItem((int)i+collision_pairs_.size(), 2, percentage);
    }

    if(allow_enable_) {
      QCheckBox* enable_box = new QCheckBox(collision_table_);
      enable_box->setChecked(false);
      connect(enable_box, SIGNAL(toggled(bool)), this, SLOT(tableChanged()));
      if(show_percentages_) {
        collision_table_->setCellWidget((int)i+collision_pairs_.size(), 3, enable_box); 
      } else {
        collision_table_->setCellWidget((int)i+collision_pairs_.size(), 2, enable_box);
      }
    }
  }
  tableChanged();
}

void CollisionsWizardPage::toggleTable()
{
  unsigned int column = 2;
  if(show_percentages_) {
    column = 3;
  }
  vector<int> rows = getSelectedRows(collision_table_);
  for(size_t i = 0; i < rows.size(); i++)
  {
    QCheckBox* box = dynamic_cast<QCheckBox*> (collision_table_->cellWidget(rows[i], column));

    if(box != NULL)
    {
      if(box->isChecked())
      {
        box->setChecked(false);
      }
      else
      {
        box->setChecked(true);
      }
    }
  }
  tableChanged();
}

void CollisionsWizardPage::tableClicked() {
  QList<QTableWidgetItem*> selected = collision_table_->selectedItems();

  ROS_DEBUG_STREAM("Table clicked");

  if(selected.size() == 0)
  {
    return;
  }

  int row = selected[0]->row();
  ROS_DEBUG_STREAM("Showing selection " << row);

  std_msgs::ColorRGBA color;
  color.r = 1.0;
  color.g = 1.0;
  color.b = 0.2;
  color.a = 1.0;
  parent_->visualizeCollision(in_collision_joint_values_, collision_pairs_, row, color);
}

void CollisionsWizardPage::tableChanged() {
  unsigned int column = 2;
  if(show_percentages_) {
    column = 3;
  }
  for(int i = 0; i < collision_table_->rowCount(); i++)
  {
    QCheckBox* box = dynamic_cast<QCheckBox*> (collision_table_->cellWidget(i, column));
    if(box != NULL)
    {
      pair<string, string> link_pair;
      link_pair.first = collision_table_->item(i, 0)->text().toStdString();
      link_pair.second = collision_table_->item(i, 1)->text().toStdString();
      bool in_disabled = false;
      vector<pair<string, string> >::iterator it = disable_pairs_.begin();
      while(it != disable_pairs_.end())
      {
        if((*it) == link_pair) {
          in_disabled = true;
          if(box->isChecked()) {
            disable_pairs_.erase(it);
          }
          break;
        }
        it++;
      }
      if(!in_disabled) {
        if(box->isChecked()) {
          it = extra_disable_pairs_.begin();
          while(it != extra_disable_pairs_.end()) {
            if((*it) == link_pair) {
              extra_disable_pairs_.erase(it);
              break;
            }
            it++;
          }
        }
        if(!box->isChecked()) {
          bool in_enable = false;
          it = enable_pairs_.begin();
          while(it != enable_pairs_.end()) {
            if((*it) == link_pair) {
              in_enable = true;
              break;
            }
            it++;
          }
          if(!in_enable) {
            disable_pairs_.push_back(link_pair);
          } else {
            bool found = false;
            it = extra_disable_pairs_.begin();
            while(it != extra_disable_pairs_.end()) {
              if((*it) == link_pair) {
                found = true;
                break;
              }
              it++;
            }
            if(!found) {
              extra_disable_pairs_.push_back(link_pair);
            }
          }
        }
      }
    }
  }
}

bool CollisionsWizardPage::validatePage() {
  parent_->getOperationsGenerator()->disablePairCollisionChecking(disable_pairs_);
  //for occasionally and never 
  if(!extra_disable_pairs_.empty()) {
    parent_->setDisableMap(CollisionOperationsGenerator::OCCASIONALLY, extra_disable_pairs_);
  }
  parent_->setDisableMap(disable_type_, disable_pairs_);
  return true;
}

OutputWizardPage::OutputWizardPage(PlanningDescriptionConfigurationWizard *parent) 
  : QWizardPage(parent), successful_generation_(false)
{
  setTitle("Output Files");
  QVBoxLayout* layout = new QVBoxLayout(this);

  setSubTitle("Done! The wizard will auto-generate a stack called <your_robot_name>_arm_navigation "
              "in the selected folder when you click the generate button below.");

  package_path_field_ = new QLineEdit(this);
  package_path_field_->setText("<absolute directory path>");
  layout->addWidget(package_path_field_);

  QPushButton* selectFileButton = new QPushButton(this);
  selectFileButton->setText("Select Directory ...");
  selectFileButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  file_selector_ = new QFileDialog(this);
  file_selector_->setFileMode(QFileDialog::Directory);
  file_selector_->setOption(QFileDialog::ShowDirsOnly, true);

  connect(file_selector_, SIGNAL(fileSelected(const QString&)), this, SLOT(fileSelected(const QString&)));
  connect(selectFileButton, SIGNAL(clicked()), file_selector_, SLOT(open()));
  layout->addWidget(selectFileButton);

  QPushButton* generateButton = new QPushButton(this);
  generateButton->setText("Generate Config Files...");
  generateButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  layout->addWidget(generateButton);

  progress_bar_ = new QProgressBar(this);
  progress_label_ = new QLabel(this);
  layout->addWidget(progress_bar_);
  layout->addWidget(progress_label_);

  really_exit_dialog_ = new QDialog(this);
  QVBoxLayout* really_exit_layout = new QVBoxLayout(really_exit_dialog_);
  QLabel* really_exit_text = new QLabel(really_exit_dialog_);
  really_exit_text->setText("You haven't generated an application - do you really want to exit?");
  really_exit_layout->addWidget(really_exit_text);
  QDialogButtonBox* exit_button_box = new QDialogButtonBox(QDialogButtonBox::Yes | QDialogButtonBox::No);
  connect(exit_button_box, SIGNAL(accepted()), really_exit_dialog_, SLOT(accept()));
  connect(exit_button_box, SIGNAL(rejected()), really_exit_dialog_, SLOT(reject()));
  really_exit_layout->addWidget(exit_button_box);
  really_exit_dialog_->setLayout(really_exit_layout);

  successful_creation_dialog_ = new QDialog(this);
  QVBoxLayout* successful_creation_layout = new QVBoxLayout(successful_creation_dialog_);
  QLabel* successful_creation_text = new QLabel(successful_creation_dialog_);
  successful_creation_text->setText("Your application was successfully created.");
  successful_creation_layout->addWidget(successful_creation_text);
  QPushButton* exit_now_button = new QPushButton(tr("&Exit Wizard"));
  QPushButton* stay_in_application_button = new QPushButton(tr("&Stay In Wizard"));
  QDialogButtonBox* success_button_box = new QDialogButtonBox();
  successful_creation_layout->addWidget(success_button_box);
  success_button_box->addButton(stay_in_application_button, QDialogButtonBox::RejectRole);
  success_button_box->addButton(exit_now_button, QDialogButtonBox::AcceptRole);
  connect(success_button_box, SIGNAL(rejected()), successful_creation_dialog_, SLOT(reject()));
  connect(success_button_box, SIGNAL(accepted()), successful_creation_dialog_, SLOT(accept()));
  connect(success_button_box, SIGNAL(accepted()), parent, SLOT(accept()));
  successful_creation_dialog_->setLayout(successful_creation_layout);

  connect(generateButton, SIGNAL(clicked()), parent, SLOT(writeFiles()));
  //addPage(output_files_page_);
  setLayout(layout);
}

PlanningDescriptionConfigurationWizard* pdcw;

bool inited = false;

void spin_function()
{
  ros::WallRate r(100.0);
  unsigned int counter = 0;
  while(ros::ok())
  {
    if(inited)
    {
      pdcw->sendTransforms();
      if(counter % CONTROL_SPEED == 0)
      {
        counter = 1;
        pdcw->sendMarkers();
      }
      else
      {
        counter++;
      }
    }
    r.sleep();
    ros::spinOnce();
  }
}

void quit(int sig)
{
  if(pdcw != NULL)
  {
    delete pdcw;
  }
  exit(0);
}

int main(int argc, char** argv)
{

  QApplication qtApp(argc, argv);

  srand(time(NULL));
  ros::init(argc, argv, "planning_description_configuration_wizard", ros::init_options::NoSigintHandler);

  if(argc < 3)
  {
    ROS_INFO_STREAM("Must specify a package and relative urdf file");
    exit(0);
  }

  string urdf_package = argv[1];
  string urdf_path = argv[2];
  pdcw = new PlanningDescriptionConfigurationWizard(urdf_package, urdf_path, NULL);
  pdcw->setUpdatesEnabled(true);

  qtApp.setActiveWindow(pdcw);

  pdcw->show();
  if(!pdcw->isInited())
  {
    ROS_WARN_STREAM("Can't init. Exiting");
    exit(0);
  }

  inited = true;

  boost::thread spin_thread(boost::bind(&spin_function));
  return qtApp.exec();
}

