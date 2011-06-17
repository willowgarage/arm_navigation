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

//in 100 hz ticks
WINDOW* left_win;
WINDOW* right_win;

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
  progress_ = 0;
  package_directory_ = "";
  string full_urdf_path = ros::package::getPath(urdf_package_) + urdf_path_;

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

  cm_ = NULL;
  ops_gen_ = NULL;

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

  emitter_ << YAML::Key << "multi_dof_joints";
  emitter_ << YAML::Value << YAML::BeginSeq;
  emitter_ << YAML::BeginMap;
  emitter_ << YAML::Key << "name" << YAML::Value << world_joint_config_.name;
  emitter_ << YAML::Key << "type" << YAML::Value << world_joint_config_.type;
  emitter_ << YAML::Key << "parent_frame_id" << YAML::Value << world_joint_config_.parent_frame_id;
  emitter_ << YAML::Key << "child_frame_id" << YAML::Value << world_joint_config_.child_frame_id;
  emitter_ << YAML::EndMap;
  emitter_ << YAML::EndSeq;
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

void PlanningDescriptionConfigurationWizard::setupGroups()
{

  while(1)
  {
    clear();
    vector<string> group_names;
    kmodel_->getModelGroupNames(group_names);
    printw("Current groups: ");
    for(unsigned int i = 0; i < group_names.size(); i++)
    {
      printw("%d) %s ", i, group_names[i].c_str());
    }
    printw("Enter 0 to accept current group set ");
    printw("Enter an 'x' followed by the group number to delete a current group. ");
    printw("Enter 1 to add a group based on kinematic chain ");
    printw("Enter 2 to add a group based on a joint collection ");
    printw("Enter 3 to add a group based on a subgroup collection ");
    refresh();
    char str[80];
    getstr(str);
    if(str[0] == 'x')
    {
      unsigned int entry;
      stringstream ss(&str[0]);
      ss >> entry;
      lock_.lock();
      deleteKinematicStates();
      kmodel_->removeModelGroup(group_names[entry]);
      robot_state_ = new KinematicState(kmodel_);
      robot_state_->setKinematicStateToDefault();
      lock_.unlock();
    }
    else
    {
      unsigned int entry;
      stringstream ss(str);
      ss >> entry;
      if(entry == 0)
        break;
      printw("Enter name for new group: ");
      getstr(str);
      stringstream ss2(str);
      string new_group_name;
      ss2 >> new_group_name;
      if(entry == 0)
      {
        break;
      }
      else if(entry == 1)
      {
        setupGroupKinematicChain(new_group_name);
      }
      else if(entry == 2)
      {
        setupGroupJointCollection(new_group_name);
      } // else {
      //   setupGroupSubgroupCollection(new_group_name);
      // }
    }
  }
}

void PlanningDescriptionConfigurationWizard::setupGroupKinematicChain(const string& new_group_name)
{
  const vector<KinematicModel::LinkModel*>& lmv = kmodel_->getLinkModels();
  bool has_base = false;
  unsigned int base_num = 0;
  bool has_tip = false;
  unsigned int tip_num = 0;
  bool group_ok = false;
  string last_status;
  while(1)
  {
    clear();
    refresh();
    for(unsigned int i = 0; i < lmv.size(); i++)
    {
      printw("%-3d ", i);
      if(has_base && i == base_num)
      {
        printw("(B) ");
      }
      else if(has_tip && i == tip_num)
      {
        printw("(T) ");
      }
      else
      {
        printw("( )");
      }
      printw("%s ", lmv[i]->getName().c_str());
    }
    printw("New group name: %s ", new_group_name.c_str());
    printw("Enter 'b' followed by a link number to set the base link for the group. ");
    printw("Enter 't' followed by a link number to set the base link for the group. ");
    printw("Enter 'q' to exit ");
    if(has_tip && has_base)
    {
      printw("Enter 'x' to validate/visualize the group ");
    }
    if(group_ok)
    {
      printw("Visualization shows group links in red and updated links in green ");
      printw("Enter 'a' to accept group ");
    }
    if(!last_status.empty())
    {
      printw("Last status msg: %s ", last_status.c_str());
    }
    refresh();
    char str[80];
    getstr(str);
    if(str[0] == 'q')
    {
      break;
    }
    else if(str[0] == 'b' || str[0] == 't')
    {
      stringstream ss(&str[1]);
      unsigned int entry;
      ss >> entry;
      if(str[0] == 'b')
      {
        base_num = entry;
        has_base = true;
      }
      else
      {
        tip_num = entry;
        has_tip = true;
      }
    }
    else if(has_tip && has_base)
    {
      if(str[0] == 'a')
      {
        if(!group_ok)
        {
          last_status = "Must validate group before accepting";
          continue;
        }
        else
        {
          current_show_group_ = "";
          break;
        }
      }
      lock_.lock();
      deleteKinematicStates();
      if(kmodel_->hasModelGroup(new_group_name))
      {
        kmodel_->removeModelGroup(new_group_name);
      }
      KinematicModel::GroupConfig gc(new_group_name, lmv[base_num]->getName(), lmv[tip_num]->getName());
      group_ok = kmodel_->addModelGroup(gc);
      robot_state_ = new KinematicState(kmodel_);
      robot_state_->setKinematicStateToDefault();
      if(group_ok)
      {
        current_show_group_ = new_group_name;
        last_status = "Group " + current_show_group_ + " ok";
      }
      else
      {
        current_show_group_ = "";
        last_status = "Group not ok";
      }
      lock_.unlock();
    }
  }
}

bool PlanningDescriptionConfigurationWizard::addGroup(string new_group_name, string base, string tip)
{
  lock_.lock();
  if(kmodel_->hasModelGroup(new_group_name))
  {
    kmodel_->removeModelGroup(new_group_name);
  }
  KinematicModel::GroupConfig gc(new_group_name, base, tip);
  bool group_ok = kmodel_->addModelGroup(gc);
  robot_state_ = new KinematicState(kmodel_);
  robot_state_->setKinematicStateToDefault();

  if(!group_ok)
  {
    popupNotOkayWarning();
    ROS_ERROR("%s %s %s invalid!", new_group_name.c_str(), base.c_str(), tip.c_str());
  }
  else
  {
    popupOkayWarning();
    updateGroupTable();
  }

  if(group_ok)
  {
    current_show_group_ = new_group_name;
  }
  else
  {
    current_show_group_ = "";
  }

  lock_.unlock();
  return group_ok;
}

void PlanningDescriptionConfigurationWizard::popupNotOkayWarning()
{
  not_ok_dialog_->show();
}

void PlanningDescriptionConfigurationWizard::popupOkayWarning()
{
  ok_dialog_->show();
}

void PlanningDescriptionConfigurationWizard::popupFileFailure(const char* reason)
{
  file_failure_reason_->setText(reason);
  file_failure_dialog_->show();
}

void PlanningDescriptionConfigurationWizard::popupFileSuccess()
{
  file_success_dialog_->show();
}

void PlanningDescriptionConfigurationWizard::updateGroupTable()
{
  current_group_table_->clear();
  vector<string> modelNames;
  kmodel_->getModelGroupNames(modelNames);
  current_group_table_->setRowCount((int)modelNames.size());
  current_group_table_->setColumnCount(1);
  current_group_table_->setColumnWidth(0, 300);
  current_group_table_->setDragEnabled(false);
  current_group_table_->setHorizontalHeaderItem(0, new QTableWidgetItem("Planning Groups"));
  for(size_t i = 0; i < modelNames.size(); i++)
  {
    QTableWidgetItem* item = new QTableWidgetItem(modelNames[i].c_str());
    current_group_table_->setItem((int)i, 0, item);
    item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
  }
}

void PlanningDescriptionConfigurationWizard::setupGroupJointCollection(const string& new_group_name)
{
  const vector<KinematicModel::JointModel*>& jmv = kmodel_->getJointModels();
  vector<bool> is_included(jmv.size(), false);
  while(1)
  {
    clear();
    for(unsigned int i = 0; i < jmv.size(); i++)
    {
      printw("%-3d ", i);
      if(is_included[i])
      {
        printw("(X) ");
      }
      else
      {
        printw("( )");
      }
      printw("%s ", jmv[i]->getName().c_str());
    }
    printw("New group name: %s ", new_group_name.c_str());
    printw("Enter a joint number or two numbers seperated by a ':' to toggle inclusion ");
    printw("Enter an 'a' followed by a joint number to toggle that joint and all downstream joints ");
    printw("Enter 'r' to reset all entries ");
    printw("Enter 'v' to visualize all member and updated links of the current selection (shown in green) ");
    printw("Enter 'x' to accept this joint collection ");
    refresh();
    char str[80];
    getstr(str);
    if(str[0] == 'x' || str[0] == 'v')
    {
      lock_.lock();
      vector<string> joints;
      for(unsigned int i = 0; i < is_included.size(); i++)
      {
        if(is_included[i])
        {
          joints.push_back(jmv[i]->getName());
        }
      }
      deleteKinematicStates();
      if(kmodel_->hasModelGroup(new_group_name))
      {
        kmodel_->removeModelGroup(new_group_name);
      }
      vector<string> emp;
      KinematicModel::GroupConfig gc(new_group_name, joints, emp);
      bool group_ok = kmodel_->addModelGroup(gc);
      robot_state_ = new KinematicState(kmodel_);
      robot_state_->setKinematicStateToDefault();
      if(!group_ok)
      {
        ROS_ERROR_STREAM("Joint collection group really should be ok");
        current_show_group_ = "";
      }
      else
      {
        if(str[0] == 'v')
        {
          current_show_group_ = new_group_name;
        }
      }
      lock_.unlock();
      if(str[0] == 'x')
      {
        break;
      }
    }
    else if(str[0] == 'r')
    {
      for(unsigned int i = 0; i < is_included.size(); i++)
      {
        is_included[i] = false;
      }
    }
    else if(str[0] == 'a')
    {
      stringstream ss(&str[1]);
      unsigned int entry;
      ss >> entry;
      vector<string> joints = kmodel_->getChildJointModelNames(jmv[entry]);
      for(unsigned int i = 0; i < joints.size(); i++)
      {
        for(unsigned int j = 0; j < jmv.size(); j++)
        {
          if(joints[i] == jmv[j]->getName())
          {
            is_included[j] = !is_included[j];
            break;
          }
        }
      }
    }
    else
    {
      unsigned int entry;
      stringstream ss(str);
      ss >> entry;
      char c;
      ss >> c;
      if(c != ':')
      {
        is_included[entry] = !is_included[entry];
      }
      else
      {
        unsigned int entry2;
        ss >> entry2;
        for(unsigned int q = entry; q <= entry2; q++)
        {
          is_included[q] = !is_included[q];
        }
      }
    }
  }
  lock_.lock();
  current_show_group_ = "";
  lock_.unlock();
}

void PlanningDescriptionConfigurationWizard::emitGroupYAML()
{
  emitter_ << YAML::Key << "groups";
  emitter_ << YAML::Value << YAML::BeginSeq;

  const map<string, KinematicModel::GroupConfig>& group_config_map = kmodel_->getJointModelGroupConfigMap();

  for(map<string, KinematicModel::GroupConfig>::const_iterator it = group_config_map.begin(); it
      != group_config_map.end(); it++)
  {
    emitter_ << YAML::BeginMap;
    emitter_ << YAML::Key << "name" << YAML::Value << it->first;
    if(!it->second.base_link_.empty())
    {
      emitter_ << YAML::Key << "base_link" << YAML::Value << it->second.base_link_;
      emitter_ << YAML::Key << "tip_link" << YAML::Value << it->second.tip_link_;
    }
    else
    {
      if(!it->second.subgroups_.empty())
      {
        emitter_ << YAML::Key << "subgroups";
        emitter_ << YAML::Value << YAML::BeginSeq;
        for(unsigned int i = 0; i < it->second.subgroups_.size(); i++)
        {
          emitter_ << it->second.subgroups_[i];
        }
        emitter_ << YAML::EndSeq;
      }
      if(!it->second.joints_.empty())
      {
        emitter_ << YAML::Key << "joints";
        emitter_ << YAML::Value << YAML::BeginSeq;
        for(unsigned int i = 0; i < it->second.joints_.size(); i++)
        {
          emitter_ << it->second.joints_[i];
        }
        emitter_ << YAML::EndSeq;
      }
    }
    emitter_ << YAML::EndMap;
  }
  emitter_ << YAML::EndSeq;
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
      if(urdf_joint)
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
          emitter << YAML::Key << "has_velocity_limits" << YAML::Value << "true";
          emitter << YAML::Key << "max_velocity" << YAML::Value << max_vel;
          emitter << YAML::Key << "has_acceleration_limits" << YAML::Value << "true";
          emitter << YAML::Key << "max_acceleration" << YAML::Value << DEFAULT_ACCELERATION;
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

void PlanningDescriptionConfigurationWizard::setJointsForCollisionSampling()
{
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

  while(1)
  {
    clear();
    refresh();
    int ind = 1;
    for(unsigned int i = 1; i < jmv.size(); i++)
    {
      const map<string, pair<double, double> >& joint_bounds = jmv[i]->getAllVariableBounds();
      for(map<string, pair<double, double> >::const_iterator it = joint_bounds.begin(); it != joint_bounds.end(); it++)
      {
        printw("%d) (%c) Dof name: %s  Lower bound: %g   Upper bound: %g ", ind, (consider_dof[ind - 1] ? 'X' : ' '),
               it->first.c_str(), it->second.first, it->second.second);
        ind++;
      }
    }
    printw(
           "Enter a number to toggle DOF for collision sampling purposes, two numbers seperated by a ':' to toggle a range(inclusive), or 0 to accept ");
    refresh();
    char str[80];
    getstr(str);
    unsigned int entry;
    stringstream ss(str);
    ss >> entry;
    if(entry == 0)
      break;
    char c;
    ss >> c;
    if(c != ':')
    {
      consider_dof[entry - 1] = !consider_dof[entry - 1];
    }
    else
    {
      unsigned int entry2;
      ss >> entry2;
      for(unsigned int q = entry - 1; q < entry2; q++)
      {
        consider_dof[q] = !consider_dof[q];
      }
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
  clear();
  refresh();
  ops_gen_->generateSamplingStructures(cdof_map);
}

Marker PlanningDescriptionConfigurationWizard::transformEnvironmentModelContactInfoMarker(
                                                                                          const EnvironmentModel::Contact& c)
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

  mk.scale.x = mk.scale.y = mk.scale.z = 0.1;
  return mk;
}

void PlanningDescriptionConfigurationWizard::considerAlwaysAndDefaultInCollisionMarkers()
{
  vector<CollisionOperationsGenerator::StringPair> always_in_collision;
  vector<CollisionOperationsGenerator::CollidingJointValues> in_collision_joint_values;

  ops_gen_->generateAlwaysInCollisionPairs(always_in_collision, in_collision_joint_values);

  lock_.lock();
  robot_state_->setKinematicStateToDefault();

  std_msgs::ColorRGBA always_color;
  always_color.a = 1.0;
  always_color.r = 1.0;
  always_color.g = .8;
  always_color.b = 0.04;

  collision_markers_.markers.clear();
  cm_->getAllCollisionPointMarkers(*robot_state_, collision_markers_, always_color, ros::Duration(.2));
  clear();
  refresh();
  printw("These pairs (with yellow collision markers) are always in collision.  Collisions will be disabled. ");
  printw("Press any key to continue. ");
  refresh();
  lock_.unlock();
  getch();

  ops_gen_->disablePairCollisionChecking(always_in_collision);
  disable_map_[CollisionOperationsGenerator::ALWAYS] = always_in_collision;
  vector<CollisionOperationsGenerator::StringPair> default_in_collision;
  ops_gen_->generateDefaultInCollisionPairs(default_in_collision, in_collision_joint_values);

  std_msgs::ColorRGBA default_color;
  default_color.a = 1.0;
  default_color.r = 0.0;
  default_color.g = .8;
  default_color.b = 0.04;

  vector<double> percentages(default_in_collision.size(), 1.0);
  clear();
  refresh();
  printw(
         "These pairs (with green collision markers) are in collision in the default state.  Collisions will be optionally disabled. ");
  considerInCollisionPairs(default_in_collision, percentages, in_collision_joint_values, default_color);
  disable_map_[CollisionOperationsGenerator::DEFAULT] = default_in_collision;

}

void PlanningDescriptionConfigurationWizard::considerOftenInCollisionPairs()
{
  vector<CollisionOperationsGenerator::StringPair> often_in_collision;
  vector<double> percentages;
  vector<CollisionOperationsGenerator::CollidingJointValues> in_collision_joint_values;

  ops_gen_->generateOftenInCollisionPairs(often_in_collision, percentages, in_collision_joint_values);

  if(often_in_collision.size() == 0)
  {
    printw("No additional often in collision pairs ");
    refresh();
    return;
  }

  std_msgs::ColorRGBA color;
  color.a = 1.0;
  color.r = 1.0;
  color.g = 0.0;
  color.b = 1.0;

  clear();
  refresh();
  printw("These pairs (with magenta collision markers) are often in collision. Collisions will be optionally disabled. ");

  considerInCollisionPairs(often_in_collision, percentages, in_collision_joint_values, color);
  disable_map_[CollisionOperationsGenerator::OFTEN] = often_in_collision;
}

void PlanningDescriptionConfigurationWizard::considerOccasionallyInCollisionPairs()
{

  vector<CollisionOperationsGenerator::StringPair> in_collision;
  vector<CollisionOperationsGenerator::StringPair> not_in_collision;
  vector<double> percentages;
  vector<CollisionOperationsGenerator::CollidingJointValues> in_collision_joint_values;

  ops_gen_->generateOccasionallyAndNeverInCollisionPairs(in_collision, not_in_collision, percentages,
                                                         in_collision_joint_values);

  if(in_collision.size() == 0)
  {
    printw("No additional often in collision pairs");
    refresh();
    return;
  }

  std_msgs::ColorRGBA color;
  color.a = 1.0;
  color.r = 1.0;
  color.g = 0.0;
  color.b = 1.0;

  clear();
  refresh();
  //printw("These pairs (with magenta collision markers) are ooccasionally in collision.  Collisions will be optionally disabled. ");

  // considerInCollisionPairs(in_collision,
  //                          percentages,
  //                          in_collision_joint_values,
  //                          color);

  //disable_map_[CollisionOperationsGenerator::OCCASIONALLY] = in_collision;
  disable_map_[CollisionOperationsGenerator::NEVER] = not_in_collision;

}

void PlanningDescriptionConfigurationWizard::considerInCollisionPairs(
                                                                      vector<CollisionOperationsGenerator::StringPair>& in_collision_pairs,
                                                                      vector<double>& percentages,
                                                                      vector<
                                                                          CollisionOperationsGenerator::CollidingJointValues>& in_collision_joint_values,
                                                                      const std_msgs::ColorRGBA& color)
{
  vector<CollisionOperationsGenerator::StringPair> actually_disabling;
  for(unsigned int i = 0; i < in_collision_pairs.size(); i++)
  {
    lock_.lock();
    collision_markers_.markers.clear();
    robot_state_->setKinematicState(in_collision_joint_values[i]);
    if(!cm_->isKinematicStateInCollision(*robot_state_))
    {
      ROS_INFO_STREAM("Really should be in collision");
    }
    vector<EnvironmentModel::AllowedContact> allowed_contacts;
    vector<EnvironmentModel::Contact> coll_space_contacts;
    cm_->getCollisionSpace()->getAllCollisionContacts(allowed_contacts, coll_space_contacts, 1);
    bool found = false;
    Marker marker;
    for(unsigned int j = 0; j < coll_space_contacts.size(); j++)
    {

      if((coll_space_contacts[j].body_name_1 == in_collision_pairs[i].first && coll_space_contacts[j].body_name_2
          == in_collision_pairs[i].second) || (coll_space_contacts[j].body_name_1 == in_collision_pairs[i].second
          && coll_space_contacts[j].body_name_2 == in_collision_pairs[i].first))
      {
        found = true;
        marker = transformEnvironmentModelContactInfoMarker(coll_space_contacts[j]);
        marker.color = color;
        marker.lifetime = ros::Duration(.2);
        collision_markers_.markers.push_back(marker);
      }
    }
    lock_.unlock();
    if(!found)
    {
      ROS_WARN_STREAM("Collision that should be there not found between " << in_collision_pairs[i].first << " and " << in_collision_pairs[i].second
          << " " << cm_->isKinematicStateInCollision(*robot_state_));
      for(unsigned int j = 0; j < coll_space_contacts.size(); j++)
      {
        ROS_INFO_STREAM("Contacts between " << coll_space_contacts[j].body_name_1 << " and " << coll_space_contacts[j].body_name_2);
      }
    }
    else
    {
      printw("Disable all collisions between %s and %s (frequency in collision %g) (y or n)?",
             in_collision_pairs[i].first.c_str(), in_collision_pairs[i].second.c_str(), percentages[i]);
      refresh();
      char str[80];
      getstr(str);
      if(str[0] != 'n')
      {
        ops_gen_->disablePairCollisionChecking(in_collision_pairs[i]);
        actually_disabling.push_back(in_collision_pairs[i]);
      }
    }
  }
  in_collision_pairs = actually_disabling;
}

void PlanningDescriptionConfigurationWizard::outputPlanningDescriptionYAML()
{
  //initial map
  emitter_ << YAML::BeginMap;
  emitWorldJointYAML();
  emitGroupYAML();
  //ops_gen_->performanceTestSavedResults(disable_map_);
  ops_gen_->outputYamlStringOfSavedResults(emitter_, disable_map_);
  //end map
  emitter_ << YAML::EndMap;
  ofstream outf(full_yaml_outfile_name_.c_str(), ios_base::trunc);

  outf << emitter_.c_str();
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
      
      emitter << YAML::Key << "manifolds" << YAML::Value << YAML::BeginSeq;
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
  rd->SetAttribute("textfile", "$(find " + urdf_package_ + ")" + urdf_path_);

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
                                            &group_link_names);

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
      //first n will be actually in group
      cm_->getRobotMarkersGivenState(*robot_state_, arr, color, current_show_group_ + "_updated_links",
                                            ros::Duration(.2), &ex_list);
      vis_marker_array_publisher_.publish(arr);
    }
    else
    {
      ROS_ERROR("The joint model group %s did not exist!", current_show_group_.c_str());
    }
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

/* Functionally replicated from Collision Model ?
void PlanningDescriptionConfigurationWizard::getRobotMeshResourceMarkersGivenState(const KinematicState& state,
                                                                                   MarkerArray& arr,
                                                                                   const std_msgs::ColorRGBA& color,
                                                                                   const string& name,
                                                                                   const ros::Duration& lifetime,
                                                                                   const vector<string>* names) const
{
  boost::shared_ptr<urdf::Model> robot_model = urdf_;

  vector<string> link_names;
  if(names == NULL)
  {
    kmodel_->getLinkModelNames(link_names);
  }
  else
  {
    link_names = *names;
  }

  for(unsigned int i = 0; i < link_names.size(); i++)
  {
    boost::shared_ptr<const urdf::Link> urdf_link = robot_model->getLink(link_names[i]);
    if(!urdf_link)
    {
      ROS_INFO_STREAM("Invalid urdf name " << link_names[i]);
      continue;
    }
    if(!urdf_link->collision)
    {
      continue;
    }
    const urdf::Geometry *geom = urdf_link->collision->geometry.get();
    if(!geom)
    {
      continue;
    }
    const urdf::Mesh *mesh = dynamic_cast<const urdf::Mesh*> (geom);
    if(!mesh)
    {
      continue;
    }
    if(mesh->filename.empty())
    {
      continue;
    }
    const KinematicState::LinkState* ls = state.getLinkState(link_names[i]);
    if(ls == NULL)
    {
      ROS_WARN_STREAM("No link state for name " << names << " though there's a mesh");
      continue;
    }
    Marker mark;
    mark.header.frame_id = kmodel_->getRoot()->getParentFrameId();
    mark.header.stamp = ros::Time::now();
    mark.ns = name;
    mark.id = i;
    mark.type = mark.MESH_RESOURCE;
    mark.scale.x = 1.0;
    mark.scale.y = 1.0;
    mark.scale.z = 1.0;
    mark.color = color;
    mark.mesh_resource = mesh->filename;
    mark.lifetime = lifetime;
    tf::poseTFToMsg(ls->getGlobalCollisionBodyTransform(), mark.pose);
    arr.markers.push_back(mark);
  }
}
*/
vector<int> PlanningDescriptionConfigurationWizard::getSelectedRows(QTableWidget* table)
{
  QList<QTableWidgetItem*> selected = table->selectedItems();

  vector<int> rows;
  for(int i = 0; i < selected.size(); i++)
  {
    bool rowExists = false;
    int r = selected[i]->row();
    for(size_t j = 0; j < rows.size(); j++)
    {
      if((int)j == r)
      {
        rowExists = true;
      }
    }

    if(!rowExists)
    {
      rows.push_back(r);
    }
  }
  return rows;
}

void PlanningDescriptionConfigurationWizard::toggleTable(QTableWidget* table, int column)
{
  vector<int> rows = getSelectedRows(table);
  for(size_t i = 0; i < rows.size(); i++)
  {
    QCheckBox* box = dynamic_cast<QCheckBox*> (table->cellWidget(rows[i], column));

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
}

void PlanningDescriptionConfigurationWizard::defaultTogglePushed()
{
  toggleTable(default_collision_table_, 2);
  defaultCollisionTableChanged();
}

void PlanningDescriptionConfigurationWizard::oftenTogglePushed()
{
  toggleTable(often_collision_table_);
  oftenCollisionTableChanged();
}

void PlanningDescriptionConfigurationWizard::occasionallyTogglePushed()
{
  toggleTable(occasionally_collision_table_);
  occasionallyCollisionTableChanged();
}

void PlanningDescriptionConfigurationWizard::dofTogglePushed()
{
  toggleTable(dof_selection_table_);
  dofSelectionTableChanged();
}

void PlanningDescriptionConfigurationWizard::selectJointButtonClicked()
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
}

void PlanningDescriptionConfigurationWizard::deselectJointButtonClicked()
{
  QList<QTableWidgetItem*> deselected = selected_joint_table_->selectedItems();

  for(int i = 0; i < deselected.size(); i++)
  {
    selected_joint_table_->removeRow(deselected[i]->row());
  }
}

void PlanningDescriptionConfigurationWizard::deleteGroupButtonClicked()
{
  QList<QTableWidgetItem*> itemList = current_group_table_->selectedItems();

  for(int i = 0; i < itemList.size(); i++)
  {
    if(current_show_group_ == itemList[i]->text().toStdString())
    {
      current_show_group_ = "";
    }
    if(kmodel_->hasModelGroup(itemList[i]->text().toStdString()))
    {
      kmodel_->removeModelGroup(itemList[i]->text().toStdString());
    }
  }

  updateGroupTable();

  if(current_group_table_->rowCount() == 0)
  {
    validateDoneBox();
  }
}

void PlanningDescriptionConfigurationWizard::acceptChainClicked()
{
  addGroup(chain_name_field_->text().toStdString(), base_link_field_->text().toStdString(),
           tip_link_field_->text().toStdString());
  createDofPageTable();
}

void PlanningDescriptionConfigurationWizard::acceptGroupClicked()
{
  lock_.lock();
  string new_group_name = joint_group_name_field_->text().toStdString();
  vector<string> joints;
  for(int i = 0; i < selected_joint_table_->rowCount(); i++)
  {
    joints.push_back(selected_joint_table_->item(i, 0)->text().toStdString());
  }

  deleteKinematicStates();
  if(kmodel_->hasModelGroup(new_group_name))
  {
    kmodel_->removeModelGroup(new_group_name);
  }

  vector<string> emp;
  KinematicModel::GroupConfig gc(new_group_name, joints, emp);
  bool group_ok = kmodel_->addModelGroup(gc);

  robot_state_ = new KinematicState(kmodel_);
  robot_state_->setKinematicStateToDefault();
  if(!group_ok)
  {
    ROS_ERROR_STREAM("Joint collection group really should be ok");
    current_show_group_ = "";
    popupNotOkayWarning();
  }
  else
  {
    current_show_group_ = new_group_name;
    sendMarkers();
  }

  popupOkayWarning();
  updateGroupTable();
  lock_.unlock();

  createDofPageTable();
}

void PlanningDescriptionConfigurationWizard::baseLinkTreeClick()
{
  QTreeWidgetItem* item = link_tree_->currentItem();
  if(item != NULL)
  {
    base_link_field_->setText(item->text(0));
  }
}

void PlanningDescriptionConfigurationWizard::tipLinkTreeClick()
{
  QTreeWidgetItem* item = link_tree_->currentItem();
  if(item != NULL)
  {
    tip_link_field_->setText(item->text(0));
  }
}

void PlanningDescriptionConfigurationWizard::validateDoneBox()
{
  if(group_selection_done_box_->isChecked())
  {
    if(current_group_table_->rowCount() <= 0)
    {
      group_selection_done_box_->setChecked(false);
      need_groups_dialog_->show();
      setup_groups_page_->setButtonText(QWizard::NextButton, "New Group...");
    }
    else
    {
      setup_groups_page_->setButtonText(QWizard::NextButton, "Next >");
      current_show_group_ = "";
    }
  }
  else
  {
    setup_groups_page_->setButtonText(QWizard::NextButton, "New Group...");
  }
}

void PlanningDescriptionConfigurationWizard::groupTableClicked()
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
    current_show_group_ = groupName->text().toStdString();
  }
}

void PlanningDescriptionConfigurationWizard::defaultTableClicked()
{
  QList<QTableWidgetItem*> selected = default_collision_table_->selectedItems();

  if(selected.size() == 0)
  {
    return;
  }

  int row = selected[0]->row();
  std_msgs::ColorRGBA color;
  color.r = 1.0;
  color.g = 1.0;
  color.b = 0.2;
  color.a = 1.0;
  visualizeCollision(default_in_collision_joint_values_, default_collision_pairs_, row, color);

}

void PlanningDescriptionConfigurationWizard::oftenTableClicked()
{
  QList<QTableWidgetItem*> selected = often_collision_table_->selectedItems();

  if(selected.size() == 0)
  {
    return;
  }

  int row = selected[0]->row();
  std_msgs::ColorRGBA color;
  color.r = 1.0;
  color.g = 1.0;
  color.b = 0.2;
  color.a = 1.0;
  visualizeCollision(often_in_collision_joint_values_, often_collision_pairs_, row, color);

}

void PlanningDescriptionConfigurationWizard::occasionallyTableClicked()
{
  QList<QTableWidgetItem*> selected = occasionally_collision_table_->selectedItems();

  if(selected.size() == 0)
  {
    return;
  }

  int row = selected[0]->row();
  std_msgs::ColorRGBA color;
  color.r = 1.0;
  color.g = 1.0;
  color.b = 0.2;
  color.a = 1.0;
  visualizeCollision(occasionally_collision_joint_values_, occasionally_collision_pairs_, row, color);
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
      else
      {
        xind++;
      }
    }
  }
  ops_gen_->generateSamplingStructures(cdof_map);
}

void PlanningDescriptionConfigurationWizard::oftenCollisionTableChanged()
{
  vector<pair<string, string> >& disableVector = disable_map_[CollisionOperationsGenerator::OFTEN];
  for(int i = 0; i < often_collision_table_->rowCount(); i++)
  {
    QCheckBox* box = dynamic_cast<QCheckBox*> (often_collision_table_->cellWidget(i, 3));
    if(box != NULL)
    {
      bool alreadyDisabled = false;
      vector<pair<string, string> >::iterator pos;
      pair<string, string> linkPair;
      linkPair.first = often_collision_table_->item(i, 0)->text().toStdString();
      linkPair.second = often_collision_table_->item(i, 1)->text().toStdString();
      for(vector<pair<string, string> >::iterator it = disableVector.begin(); it != disableVector.end(); it++)
      {
        if((*it) == linkPair)
        {
          alreadyDisabled = true;
          pos = it;
          break;
        }
      }

      if(box->isChecked())
      {
        if(alreadyDisabled)
        {
          disableVector.erase(pos);
          ops_gen_->enablePairCollisionChecking(linkPair);
        }
      }
      else
      {
        if(!alreadyDisabled)
        {
          disableVector.push_back(linkPair);
          ops_gen_->disablePairCollisionChecking(linkPair);
        }
      }
    }
  }
}

void PlanningDescriptionConfigurationWizard::defaultCollisionTableChanged()
{
  vector<pair<string, string> >& disableVector = disable_map_[CollisionOperationsGenerator::DEFAULT];
  for(int i = 0; i < default_collision_table_->rowCount(); i++)
  {
    QCheckBox* box = dynamic_cast<QCheckBox*> (default_collision_table_->cellWidget(i, 2));
    if(box != NULL)
    {
      bool alreadyDisabled = false;
      vector<pair<string, string> >::iterator pos;
      pair<string, string> linkPair;
      linkPair.first = default_collision_table_->item(i, 0)->text().toStdString();
      linkPair.second = default_collision_table_->item(i, 1)->text().toStdString();
      for(vector<pair<string, string> >::iterator it = disableVector.begin(); it != disableVector.end(); it++)
      {
        if((*it) == linkPair)
        {
          alreadyDisabled = true;
          pos = it;
          break;
        }
      }

      if(box->isChecked())
      {
        if(alreadyDisabled)
        {
          disableVector.erase(pos);
          ops_gen_->enablePairCollisionChecking(linkPair);
        }
      }
      else
      {
        if(!alreadyDisabled)
        {
          disableVector.push_back(linkPair);
          ops_gen_->disablePairCollisionChecking(linkPair);
        }
      }
    }
  }
}

void PlanningDescriptionConfigurationWizard::occasionallyCollisionTableChanged()
{
  vector<pair<string, string> >& disableVector = disable_map_[CollisionOperationsGenerator::OCCASIONALLY];
  for(int i = 0; i < occasionally_collision_table_->rowCount(); i++)
  {
    QCheckBox* box = dynamic_cast<QCheckBox*> (occasionally_collision_table_->cellWidget(i, 3));
    if(box != NULL)
    {
      bool alreadyDisabled = false;
      vector<pair<string, string> >::iterator pos = disableVector.end();
      pair<string, string> linkPair;
      linkPair.first = occasionally_collision_table_->item(i, 0)->text().toStdString();
      linkPair.second = occasionally_collision_table_->item(i, 1)->text().toStdString();
      for(vector<pair<string, string> >::iterator it = disableVector.begin(); it != disableVector.end(); it++)
      {
        if((*it) == linkPair)
        {
          alreadyDisabled = true;
          pos = it;
          break;
        }
      }

      if(box->isChecked())
      {
        if(alreadyDisabled)
        {
          if(pos != disableVector.end())
          {
            disableVector.erase(pos);
          }
          ops_gen_->enablePairCollisionChecking(linkPair);
        }
      }
      else
      {
        if(!alreadyDisabled)
        {
          disableVector.push_back(linkPair);
          ops_gen_->disablePairCollisionChecking(linkPair);
        }
      }
    }
  }
}

void PlanningDescriptionConfigurationWizard::generateOccasionallyInCollisionTable()
{
  lock_.lock();
  vector<CollisionOperationsGenerator::StringPair> not_in_collision;
  vector<double> percentages;

  ops_gen_->generateOccasionallyAndNeverInCollisionPairs(occasionally_collision_pairs_, not_in_collision, percentages,
                                                         occasionally_collision_joint_values_);

  occasionally_collision_table_->clear();
  occasionally_collision_table_->setRowCount((int)(occasionally_collision_pairs_.size() + not_in_collision.size()));
  occasionally_collision_table_->setColumnCount(4);

  occasionally_collision_table_->setColumnWidth(0, 300);
  occasionally_collision_table_->setColumnWidth(1, 300);
  occasionally_collision_table_->setColumnWidth(2, 300);
  occasionally_collision_table_->setColumnWidth(3, 300);

  QStringList titleList;
  titleList.append("Link A");
  titleList.append("Link B");
  titleList.append("% Colliding");
  titleList.append("Enable?");

  occasionally_collision_table_->setHorizontalHeaderLabels(titleList);

  if(occasionally_collision_pairs_.size() + not_in_collision.size() == 0)
  {
    occasionally_collision_table_->setRowCount(1);
    QTableWidgetItem* noCollide = new QTableWidgetItem("No Collisions");
    occasionally_collision_table_->setItem(0, 0, noCollide);
  }

  ROS_INFO("%lu links occasionally in collision.", occasionally_collision_pairs_.size());

  for(size_t i = 0; i < occasionally_collision_pairs_.size(); i++)
  {
    QTableWidgetItem* linkA = new QTableWidgetItem(occasionally_collision_pairs_[i].first.c_str());
    linkA->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

    QTableWidgetItem* linkB = new QTableWidgetItem(occasionally_collision_pairs_[i].second.c_str());
    linkB->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

    stringstream percentageStream;
    percentageStream << percentages[i];
    QTableWidgetItem* percentage = new QTableWidgetItem(percentageStream.str().c_str());
    percentage->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

    QCheckBox* enableBox = new QCheckBox(occasionally_collision_table_);
    enableBox->setChecked(false);
    connect(enableBox, SIGNAL(toggled(bool)), this, SLOT(occasionallyCollisionTableChanged()));

    occasionally_collision_table_->setItem((int)i, 0, linkA);
    occasionally_collision_table_->setItem((int)i, 1, linkB);
    occasionally_collision_table_->setItem((int)i, 2, percentage);
    occasionally_collision_table_->setCellWidget((int)i, 3, enableBox);
  }

  for(size_t i = 0; i < not_in_collision.size(); i++)
  {
    QTableWidgetItem* linkA = new QTableWidgetItem(not_in_collision[i].first.c_str());
    linkA->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

    QTableWidgetItem* linkB = new QTableWidgetItem(not_in_collision[i].second.c_str());
    linkB->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

    stringstream percentageStream;
    percentageStream << 0.0;
    QTableWidgetItem* percentage = new QTableWidgetItem(percentageStream.str().c_str());
    percentage->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

    QCheckBox* enableBox = new QCheckBox(occasionally_collision_table_);
    enableBox->setChecked(false);
    connect(enableBox, SIGNAL(toggled(bool)), this, SLOT(occasionallyCollisionTableChanged()));

    occasionally_collision_table_->setItem((int)(i + occasionally_collision_pairs_.size()), 0, linkA);
    occasionally_collision_table_->setItem((int)(i + occasionally_collision_pairs_.size()), 1, linkB);
    occasionally_collision_table_->setItem((int)(i + occasionally_collision_pairs_.size()), 2, percentage);
    occasionally_collision_table_->setCellWidget((int)(i + occasionally_collision_pairs_.size()), 3, enableBox);
  }
  occasionallyCollisionTableChanged();

  lock_.unlock();
}

void PlanningDescriptionConfigurationWizard::generateOftenInCollisionTable()
{
  lock_.lock();
  vector<double> percentages;

  ops_gen_->generateOftenInCollisionPairs(often_collision_pairs_, percentages, often_in_collision_joint_values_);

  often_collision_table_->clear();
  often_collision_table_->setRowCount((int)often_collision_pairs_.size());
  often_collision_table_->setColumnCount(4);

  often_collision_table_->setColumnWidth(0, 250);
  often_collision_table_->setColumnWidth(1, 250);
  often_collision_table_->setColumnWidth(2, 250);
  often_collision_table_->setColumnWidth(3, 250);

  QStringList titleList;
  titleList.append("Link A");
  titleList.append("Link B");
  titleList.append("% Colliding");
  titleList.append("Enable?");

  often_collision_table_->setHorizontalHeaderLabels(titleList);

  if(often_collision_pairs_.size() == 0)
  {
    often_collision_table_->setRowCount(1);
    QTableWidgetItem* noCollide = new QTableWidgetItem("No Collisions");
    often_collision_table_->setItem(0, 0, noCollide);
  }

  ROS_INFO("%lu links often in collision.", often_collision_pairs_.size());

  for(size_t i = 0; i < often_collision_pairs_.size(); i++)
  {
    QTableWidgetItem* linkA = new QTableWidgetItem(often_collision_pairs_[i].first.c_str());
    linkA->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

    QTableWidgetItem* linkB = new QTableWidgetItem(often_collision_pairs_[i].second.c_str());
    linkB->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

    stringstream percentageStream;
    percentageStream << percentages[i];
    QTableWidgetItem* percentage = new QTableWidgetItem(percentageStream.str().c_str());
    percentage->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

    QCheckBox* enableBox = new QCheckBox(often_collision_table_);
    enableBox->setChecked(false);
    connect(enableBox, SIGNAL(toggled(bool)), this, SLOT(oftenCollisionTableChanged()));

    often_collision_table_->setItem((int)i, 0, linkA);
    often_collision_table_->setItem((int)i, 1, linkB);
    often_collision_table_->setItem((int)i, 2, percentage);
    often_collision_table_->setCellWidget((int)i, 3, enableBox);
  }
  oftenCollisionTableChanged();

  lock_.unlock();
}

void PlanningDescriptionConfigurationWizard::generateAlwaysInCollisionTable()
{
  vector<CollisionOperationsGenerator::StringPair> always_in_collision;
  vector<CollisionOperationsGenerator::CollidingJointValues> in_collision_joint_values;

  ops_gen_->generateAlwaysInCollisionPairs(always_in_collision, in_collision_joint_values);

  lock_.lock();
  robot_state_->setKinematicStateToDefault();

  std_msgs::ColorRGBA always_color;
  always_color.a = 1.0;
  always_color.r = 1.0;
  always_color.g = .8;
  always_color.b = 0.04;

  collision_markers_.markers.clear();
  cm_->getAllCollisionPointMarkers(*robot_state_, collision_markers_, always_color, ros::Duration(.2));

  always_collision_table_->clear();
  always_collision_table_->setRowCount((int)always_in_collision.size());
  always_collision_table_->setColumnCount(2);
  always_collision_table_->setColumnWidth(0, 500);
  always_collision_table_->setColumnWidth(1, 500);
  QStringList titleList;
  titleList.append("Link A");
  titleList.append("Link B");
  always_collision_table_->setHorizontalHeaderLabels(titleList);

  for(size_t i = 0; i < always_in_collision.size(); i++)
  {
    QTableWidgetItem* linkA = new QTableWidgetItem(always_in_collision[i].first.c_str());
    linkA->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    QTableWidgetItem* linkB = new QTableWidgetItem(always_in_collision[i].second.c_str());
    linkB->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

    always_collision_table_->setItem((int)i, 0, linkA);
    always_collision_table_->setItem((int)i, 1, linkB);

  }
  lock_.unlock();

  ops_gen_->disablePairCollisionChecking(always_in_collision);
  disable_map_[CollisionOperationsGenerator::ALWAYS] = always_in_collision;
}

void PlanningDescriptionConfigurationWizard::generateDefaultInCollisionTable()
{
  lock_.lock();

  ops_gen_->generateDefaultInCollisionPairs(default_collision_pairs_, default_in_collision_joint_values_);

  default_collision_table_->clear();
  default_collision_table_->setRowCount((int)default_collision_pairs_.size());
  default_collision_table_->setColumnCount(3);

  default_collision_table_->setColumnWidth(0, 300);
  default_collision_table_->setColumnWidth(1, 300);
  default_collision_table_->setColumnWidth(2, 300);

  QStringList titleList;
  titleList.append("Link A");
  titleList.append("Link B");
  titleList.append("Enable?");

  default_collision_table_->setHorizontalHeaderLabels(titleList);

  if(default_collision_pairs_.size() == 0)
  {
    default_collision_table_->setRowCount(1);
    QTableWidgetItem* noCollide = new QTableWidgetItem("No Collisions");
    default_collision_table_->setItem(0, 0, noCollide);
  }

  ROS_INFO("%lu links often in collision.", default_collision_pairs_.size());

  for(size_t i = 0; i < default_collision_pairs_.size(); i++)
  {
    QTableWidgetItem* linkA = new QTableWidgetItem(default_collision_pairs_[i].first.c_str());
    linkA->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

    QTableWidgetItem* linkB = new QTableWidgetItem(default_collision_pairs_[i].second.c_str());
    linkB->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

    QCheckBox* enableBox = new QCheckBox(default_collision_table_);
    enableBox->setChecked(false);
    connect(enableBox, SIGNAL(toggled(bool)), this, SLOT(defaultCollisionTableChanged()));

    default_collision_table_->setItem((int)i, 0, linkA);
    default_collision_table_->setItem((int)i, 1, linkB);
    default_collision_table_->setCellWidget((int)i, 2, enableBox);
  }
  oftenCollisionTableChanged();

  lock_.unlock();
}

void PlanningDescriptionConfigurationWizard::popupGenericWarning(const char* text)
{
  generic_dialog_label_->setText(text);
  generic_dialog_->show();
  generic_dialog_->setVisible(true);
  generic_dialog_->setModal(true);
  ROS_INFO("Showing warning: %s", text);
}

void PlanningDescriptionConfigurationWizard::fileSelected(const QString& file)
{
  package_path_field_->setText(file);
}

void PlanningDescriptionConfigurationWizard::writeFiles()
{
  package_directory_ = package_path_field_->text().toStdString();
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
  mdir += " planning_environment arm_kinematics_constraint_aware ompl_ros_interface ";
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
    progress_bar_->setValue(10);
    outputOMPLGroupYAML();
    progress_bar_->setValue(20);
    outputPlanningDescriptionYAML();
    progress_bar_->setValue(30);
    outputOMPLLaunchFile();
    progress_bar_->setValue(40);
    outputKinematicsLaunchFiles();
    progress_bar_->setValue(50);
    outputTrajectoryFilterLaunch();
    progress_bar_->setValue(65);
    outputPlanningEnvironmentLaunch();
    progress_bar_->setValue(75);
    outputPlanningComponentVisualizerLaunchFile();
    progress_bar_->setValue(100);

    popupFileSuccess();
  }
}

void PlanningDescriptionConfigurationWizard::labelChanged(const char* label)
{
  progress_label_->setText(label);
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
      if(group_selection_done_box_->isChecked())
      {
        if(wizard_mode_ == PlanningDescriptionConfigurationWizard::Advanced)
        {
          return SelectDOFPage;
        }
        else
        {
          return OutputFilesPage;
        }
      }
      else
      {
        if(group_selection_mode_box_->itemText(group_selection_mode_box_->currentIndex()) == "Kinematic Chains")
        {
          return KinematicChainsPage;
        }
        else
        {
          return JointCollectionsPage;
        }
      }
      return -1;

    case KinematicChainsPage:
      return SetupGroupsPage;

    case JointCollectionsPage:
      return SetupGroupsPage;

    case SelectDOFPage:
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
  progress_bar_->setValue(progress_);
  if(progress_ >= 100)
  {
    popupFileSuccess();
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

  if(index > (int)(pairs.size()) || index < 0 || pairs.size() == 0)
  {
    return;
  }
  lock_.lock();

  ROS_INFO("Visualizing collision index %d", index);

  collision_markers_.markers.clear();
  robot_state_->setKinematicState(jointValues[index]);

  if(!cm_->isKinematicStateInCollision(*robot_state_))
  {
    ROS_INFO_STREAM("Really should be in collision");
  }

  vector<EnvironmentModel::AllowedContact> allowed_contacts;
  vector<EnvironmentModel::Contact> coll_space_contacts;
  cm_->getCollisionSpace()->getAllCollisionContacts(allowed_contacts, coll_space_contacts, 1);

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
      marker.color = color;
      marker.lifetime = ros::Duration(.2);
      marker2.type = Marker::ARROW;
      marker2.color.r = 1.0;
      marker2.color.g = 0.0;
      marker2.color.b = 0.0;
      marker2.color.a = 1.0;
      marker2.scale.x = 0.5;
      marker2.scale.y = 0.5;
      marker2.scale.z = 0.5;
      marker2.pose.position.z = marker2.pose.position.z + 0.8;
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
  initSetupGroupsPage();
  initKinematicChainsPage();
  initJointCollectionsPage();
  initSelectDofPage();
  initAlwaysInCollisionPage();
  initDefaultInCollisionPage();
  initOftenInCollisionPage();
  initOccasionallyInCollisionPage();
  initOutputFilesPage();

  generic_dialog_ = new QDialog(this);
  QVBoxLayout* gDialogLayout = new QVBoxLayout(generic_dialog_);
  generic_dialog_label_ = new QLabel(generic_dialog_);
  generic_dialog_label_->setText("Warning!");
  gDialogLayout->addWidget(generic_dialog_label_);
  generic_dialog_->setLayout(gDialogLayout);

  need_groups_dialog_ = new QDialog(this);
  QVBoxLayout* needsGroupsDialogLayout = new QVBoxLayout(need_groups_dialog_);
  QLabel* needsGroupsText = new QLabel(need_groups_dialog_);
  needsGroupsText->setText("Cannot continue without planning groups!");
  needsGroupsDialogLayout->addWidget(needsGroupsText);
  need_groups_dialog_->setLayout(needsGroupsDialogLayout);

  ok_dialog_ = new QDialog(this);
  QVBoxLayout* okDialogLayout = new QVBoxLayout(ok_dialog_);
  QLabel* okText = new QLabel(ok_dialog_);
  okText->setText("The planning group was valid! Visualize in Rviz.");
  okDialogLayout->addWidget(okText);
  ok_dialog_->setLayout(okDialogLayout);

  not_ok_dialog_ = new QDialog(this);
  QVBoxLayout* notOkDialogLayout = new QVBoxLayout(not_ok_dialog_);
  QLabel* notOkText = new QLabel(not_ok_dialog_);
  notOkText->setText("Error! The planning group was invalid!");
  notOkDialogLayout->addWidget(notOkText);
  not_ok_dialog_->setLayout(notOkDialogLayout);

  file_failure_dialog_ = new QDialog(this);
  QVBoxLayout* filefailureLayout = new QVBoxLayout(file_failure_dialog_);
  QLabel* fileFailureText = new QLabel(file_failure_dialog_);
  fileFailureText->setText("Failed to create files! Reason: ");
  file_failure_reason_ = new QLabel(this);
  file_failure_reason_->setText("unspecified.");
  filefailureLayout->addWidget(fileFailureText);
  filefailureLayout->addWidget(file_failure_reason_);
  file_failure_dialog_->setLayout(filefailureLayout);

  file_success_dialog_ = new QDialog(this);
  QVBoxLayout* filesuccessLayout = new QVBoxLayout(file_success_dialog_);
  QLabel* filesuccessText = new QLabel(file_success_dialog_);
  filesuccessText->setText("Successfully created files!");
  filesuccessLayout->addWidget(filesuccessText);
  file_success_dialog_->setLayout(filesuccessLayout);

}

void PlanningDescriptionConfigurationWizard::initStartPage()
{
  start_page_ = new QWizardPage(this);
  start_page_->setTitle("Planning Components Configuration Wizard");
  QHBoxLayout* layout = new QHBoxLayout(setup_groups_page_);

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
  imageLabel->setMinimumHeight(image->height());
  imageLabel->setMinimumWidth(image->width());
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
  layout->addWidget(imageLabel);
  descLayout->addWidget(label);

  label->setAlignment(Qt::AlignTop);

  descBox->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  descBox->setAlignment(Qt::AlignTop);

  QGroupBox* modeGroupBox = new QGroupBox(descBox);
  modeGroupBox->setTitle("Select Mode");

  QVBoxLayout* modeGroupBoxLayout = new QVBoxLayout(modeGroupBox);
  QLabel* modeGroupBoxDesc = new QLabel(modeGroupBox);
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
  modeGroupBox->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Maximum);




  QGroupBox* safetyGroupBox = new QGroupBox(descBox);
   safetyGroupBox->setTitle("Select Safety");

   QVBoxLayout* safetyGroupBoxLayout = new QVBoxLayout(safetyGroupBox);
   QLabel* safetyGroupBoxDesc = new QLabel(safetyGroupBox);
   safetyGroupBoxDesc->setText("Defines the sampling density used to determine whether"
                               "\ncertain robot poses are in collision. Safer settings"
                               "\nwill sample longer, while faster settings are more"
                               "\nlikely to be inaccurate.");

   safetyGroupBoxLayout->addWidget(safetyGroupBoxDesc);

   QRadioButton* verySafeButton = new QRadioButton(safetyGroupBox);
   verySafeButton->setText("Very Safe");
   safetyGroupBoxLayout->addWidget(verySafeButton);
   verySafeButton->setChecked(true);

   connect(verySafeButton, SIGNAL(toggled(bool)), this, SLOT(verySafeButtonToggled(bool)));

   QRadioButton* safeButton = new QRadioButton(safetyGroupBox);
   safeButton->setText("Safe");
   safetyGroupBoxLayout->addWidget(safeButton);

   connect(safeButton, SIGNAL(toggled(bool)), this, SLOT(safeButtonToggled(bool)));

   QRadioButton* normalButton = new QRadioButton(safetyGroupBox);
   normalButton->setText("Normal");
   safetyGroupBoxLayout->addWidget(normalButton);
   normalButton->setChecked(true);

   connect(normalButton, SIGNAL(toggled(bool)), this, SLOT(normalButtonToggled(bool)));

   QRadioButton* fastButton = new QRadioButton(safetyGroupBox);
   fastButton->setText("Fast");
   safetyGroupBoxLayout->addWidget(fastButton);

   connect(fastButton, SIGNAL(toggled(bool)), this, SLOT(fastButtonToggled(bool)));

   QRadioButton* veryFastButton = new QRadioButton(safetyGroupBox);
   veryFastButton->setText("Very Fast");
   safetyGroupBoxLayout->addWidget(veryFastButton);

   connect(veryFastButton, SIGNAL(toggled(bool)), this, SLOT(veryFastButtonToggled(bool)));


   safetyGroupBox->setLayout(safetyGroupBoxLayout);
   safetyGroupBox->setAlignment(Qt::AlignTop | Qt::AlignLeft);

   descLayout->addWidget(safetyGroupBox);
   safetyGroupBox->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Maximum);

  layout->addWidget(descBox);


  setPage(StartPage, start_page_);
  start_page_->setLayout(layout);
  start_page_->setMinimumWidth(1000);
  start_page_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
}

void PlanningDescriptionConfigurationWizard::initSetupGroupsPage()
{
  setup_groups_page_ = new QWizardPage(this);
  setup_groups_page_->setTitle("Planning Group Setup");

  QGridLayout* layout = new QGridLayout(setup_groups_page_);
  setup_groups_page_->setSubTitle(
                                  "Select planning groups for your robot based on kinematic chains, or joint collections."
                                    " When you are finished, please check the checkbox and you can move on by pressing Next.");

  QGroupBox* selectGroupsBox = new QGroupBox(setup_groups_page_);
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

  connect(current_group_table_, SIGNAL(itemClicked(QTableWidgetItem*)), SLOT(groupTableClicked()));
  connect(deleteButton, SIGNAL(clicked()), this, SLOT(deleteGroupButtonClicked()));

  QGroupBox* modeBox = new QGroupBox(setup_groups_page_);
  modeBox->setTitle("Select Mode");
  QVBoxLayout* modeBoxLayout = new QVBoxLayout(modeBox);
  group_selection_mode_box_ = new QComboBox(modeBox);
  group_selection_done_box_ = new QCheckBox(modeBox);
  group_selection_done_box_->setText("Done Selecting Groups");

  connect(group_selection_done_box_, SIGNAL(toggled(bool)), this, SLOT(validateDoneBox()));

  QStringList texts;
  texts.append("Kinematic Chains");
  texts.append("Joint Collections");
  group_selection_mode_box_->addItems(texts);

  modeBoxLayout->addWidget(group_selection_mode_box_);
  modeBoxLayout->addWidget(group_selection_done_box_);
  modeBox->setLayout(modeBoxLayout);
  layout->addWidget(modeBox, 0, 2, 1, 1);

  //addPage(setup_groups_page_);
  setPage(SetupGroupsPage, setup_groups_page_);
  setup_groups_page_->setLayout(layout);
  modeBoxLayout->setAlignment(group_selection_mode_box_, Qt::AlignTop);
  layout->setAlignment(modeBox, Qt::AlignTop);

  setup_groups_page_->setButtonText(QWizard::NextButton, "New Group...");
}

void PlanningDescriptionConfigurationWizard::initKinematicChainsPage()
{
  kinematic_chains_page_ = new QWizardPage(this);
  kinematic_chains_page_->setTitle("Select Kinematic Chain");

  QGridLayout* layout = new QGridLayout(kinematic_chains_page_);
  kinematic_chains_page_->setSubTitle("Select a planning group based on a kinematic chain."
    " Select a base link (the first link in the chain) and a tip link."
    " They must be connected by a direct line of joints.");

  QImage* image = new QImage();
  if(!image->load("./resources/chains.png"))
  {
    ROS_ERROR("FAILED TO LOAD ./resources/chains.png");
  }
  ROS_INFO("Loaded Image with %d bytes.", image->byteCount());

  QLabel* imageLabel = new QLabel(kinematic_chains_page_);
  imageLabel->setPixmap(QPixmap::fromImage(*image));
  imageLabel->setAlignment(Qt::AlignTop);
  layout->addWidget(imageLabel, 0, 0, 0, 1);
  QGroupBox* treeBox = new QGroupBox(kinematic_chains_page_);
  treeBox->setTitle("Links");
  layout->addWidget(treeBox, 0, 1, 1, 1);

  QVBoxLayout* treeLayout = new QVBoxLayout(treeBox);
  link_tree_ = new QTreeWidget(treeBox);
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
  createLinkTree();

  connect(baseLinkButton, SIGNAL(clicked()), this, SLOT(baseLinkTreeClick()));
  connect(tipLinkButton, SIGNAL(clicked()), this, SLOT(tipLinkTreeClick()));
  QGroupBox* chainBox = new QGroupBox(kinematic_chains_page_);
  chainBox->setTitle("Kinematic Chain");
  QFormLayout* chainLayout = new QFormLayout(chainBox);
  chain_name_field_ = new QLineEdit(chainBox);
  base_link_field_ = new QLineEdit(chainBox);
  tip_link_field_ = new QLineEdit(chainBox);
  chainLayout->addRow("Chain Name", chain_name_field_);
  chainLayout->addRow("Base Link", base_link_field_);
  chainLayout->addRow("Tip Link", tip_link_field_);

  QPushButton* acceptButton = new QPushButton(chainBox);
  acceptButton->setText("Accept Chain");
  chainLayout->addRow(acceptButton);
  chainBox->setLayout(chainLayout);
  acceptButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  connect(acceptButton, SIGNAL(clicked()), this, SLOT(acceptChainClicked()));

  layout->addWidget(chainBox, 1, 1, 1, 2);

  //addPage(kinematic_chains_page_);
  setPage(KinematicChainsPage, kinematic_chains_page_);
  kinematic_chains_page_->setLayout(layout);

  kinematic_chains_page_->setButtonText(QWizard::NextButton, "-");
}

void PlanningDescriptionConfigurationWizard::createJointCollectionTables()
{
  const vector<KinematicModel::JointModel*>& jmv = kmodel_->getJointModels();

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

void PlanningDescriptionConfigurationWizard::initJointCollectionsPage()
{
  joint_collections_page_ = new QWizardPage(this);
  joint_collections_page_->setTitle("Select Joint Collections");

  QGridLayout* layout = new QGridLayout(joint_collections_page_);

  joint_collections_page_->setSubTitle("Select an arbitrary group of joints to form a planning group.");

  QImage* image = new QImage();
  if(!image->load("./resources/groups.png"))
  {
    ROS_ERROR("FAILED TO LOAD ./resources/groups.png");
  }
  ROS_INFO("Loaded Image with %d bytes.", image->byteCount());

  QLabel* imageLabel = new QLabel(kinematic_chains_page_);
  imageLabel->setPixmap(QPixmap::fromImage(*image));
  imageLabel->setAlignment(Qt::AlignTop);
  layout->addWidget(imageLabel, 0, 1, 1, 1);

  QGroupBox* jointBox = new QGroupBox(joint_collections_page_);
  jointBox->setTitle("Joints");
  jointBox->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);

  QVBoxLayout* jointLayout = new QVBoxLayout(jointBox);
  joint_table_ = new QTableWidget(jointBox);
  jointLayout->addWidget(joint_table_);
  jointBox->setLayout(jointLayout);
  layout->addWidget(jointBox, 0, 0, 3, 1);

  QPushButton* selectButton = new QPushButton(jointBox);
  selectButton->setText("v Select v");
  jointLayout->addWidget(selectButton);
  selectButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);


  connect(selectButton, SIGNAL(clicked()), this, SLOT(selectJointButtonClicked()));

  QGroupBox* selectedBox = new QGroupBox(joint_collections_page_);
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
  QPushButton* acceptButton = new QPushButton(selectedBox);
  acceptButton->setText("Accept Joint Group");
  acceptButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  selectedLayout->addWidget(acceptButton);
  selectedBox->setLayout(selectedLayout);
  layout->addWidget(selectedBox, 1, 1, 1, 1);

  connect(acceptButton, SIGNAL(clicked()), this, SLOT(acceptGroupClicked()));

  //addPage(joint_collections_page_);
  setPage(JointCollectionsPage, joint_collections_page_);
  joint_collections_page_->setLayout(layout);

  createJointCollectionTables();
  joint_collections_page_->setButtonText(QWizard::NextButton, "-");
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

  connect(toggleSelected, SIGNAL(clicked()), SLOT(dofTogglePushed()));

  //addPage(select_dof_page_);
  setPage(SelectDOFPage, select_dof_page_);
  select_dof_page_->setLayout(layout);

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

void PlanningDescriptionConfigurationWizard::initAlwaysInCollisionPage()
{
  always_in_collision_page_ = new QWizardPage(this);
  always_in_collision_page_->setTitle("Links Always In Collision");

  QVBoxLayout* layout = new QVBoxLayout(always_in_collision_page_);
  always_in_collision_page_->setSubTitle("The following links are always in collision over the sample space. "
    "By default, collisions will be disabled for them. Collisions are visualized as yellow spheres in rviz.");
  QPushButton* generateButton = new QPushButton(always_in_collision_page_);
  generateButton->setText("Generate List (May take a minute)");
  layout->addWidget(generateButton);
  connect(generateButton, SIGNAL(clicked()), this, SLOT(generateAlwaysInCollisionTable()));
  generateButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  always_collision_table_ = new QTableWidget(always_in_collision_page_);
  layout->addWidget(always_collision_table_);

  //addPage(always_in_collision_page_);
  setPage(AlwaysInCollisionPage, always_in_collision_page_);
  always_in_collision_page_->setLayout(layout);
}

void PlanningDescriptionConfigurationWizard::initDefaultInCollisionPage()
{
  default_collision_page_ = new QWizardPage(this);
  default_collision_page_->setTitle("Links Default In Collision");

  QVBoxLayout* layout = new QVBoxLayout(default_collision_page_);
  default_collision_page_->setSubTitle("The following links are in collision in the default robot state. "
    "By default, collisions will be disabled for them.  Select items to visualize collisions in rviz.");

  QPushButton* generateButton = new QPushButton(default_collision_page_);
  generateButton->setText("Generate List (May take a minute)");
  layout->addWidget(generateButton);
  generateButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  default_collision_table_ = new QTableWidget(default_collision_page_);
  layout->addWidget(default_collision_table_);

  connect(default_collision_table_, SIGNAL(cellClicked(int, int)), this, SLOT(defaultTableClicked()));
  QPushButton* toggleSelected = new QPushButton(default_collision_page_);
  toggleSelected->setText("Toggle Selected");
  toggleSelected->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  layout->addWidget(toggleSelected);
  connect(toggleSelected, SIGNAL(clicked()), this, SLOT(defaultTogglePushed()));

  connect(generateButton, SIGNAL(clicked()), this, SLOT(generateDefaultInCollisionTable()));

  //addPage(default_collision_page_);
  setPage(DefaultInCollisionPage, default_collision_page_);
  default_collision_page_->setLayout(layout);
}

void PlanningDescriptionConfigurationWizard::initOftenInCollisionPage()
{
  often_in_collision_page_ = new QWizardPage(this);
  often_in_collision_page_->setTitle("Links Often In Collision");

  QVBoxLayout* layout = new QVBoxLayout(often_in_collision_page_);
  often_in_collision_page_->setSubTitle("The following links are often in collision over the sample space. "
    " By default, collisions will be disabled for them. Select items to visualize collisions in rviz.");

  QPushButton* generateButton = new QPushButton(often_in_collision_page_);
  generateButton->setText("Generate List (May take a minute)");
  layout->addWidget(generateButton);
  generateButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  often_collision_table_ = new QTableWidget(often_in_collision_page_);
  layout->addWidget(often_collision_table_);

  QPushButton* toggleSelected = new QPushButton(often_in_collision_page_);
  toggleSelected->setText("Toggle Selected");
  layout->addWidget(toggleSelected);
  connect(toggleSelected, SIGNAL(clicked()), this, SLOT(oftenTogglePushed()));
  toggleSelected->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  connect(generateButton, SIGNAL(clicked()), this, SLOT(generateOftenInCollisionTable()));

  //addPage(often_in_collision_page_);
  setPage(OftenInCollisionPage, often_in_collision_page_);
  often_in_collision_page_->setLayout(layout);

  connect(often_collision_table_, SIGNAL(cellClicked(int, int)), this, SLOT(oftenTableClicked()));
}

void PlanningDescriptionConfigurationWizard::initOccasionallyInCollisionPage()
{
  occasionally_in_collision_page_ = new QWizardPage(this);
  occasionally_in_collision_page_->setTitle("Links Occasionally In Collision");

  QVBoxLayout* layout = new QVBoxLayout(occasionally_in_collision_page_);

  occasionally_in_collision_page_->setSubTitle(
                                               "The following links are occasionally (or never) in collision over the sample space. "
                                                 "By default, collisions will be disabled. Select a pair to visualize in rviz.");

  QPushButton* generateButton = new QPushButton(occasionally_in_collision_page_);
  generateButton->setText("Generate List (May take a minute)");
  layout->addWidget(generateButton);
  connect(generateButton, SIGNAL(clicked()), this, SLOT(generateOccasionallyInCollisionTable()));
  generateButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);


  occasionally_collision_table_ = new QTableWidget(occasionally_in_collision_page_);
  layout->addWidget(occasionally_collision_table_);

  connect(occasionally_collision_table_, SIGNAL(itemClicked(QTableWidgetItem*)), SLOT(occasionallyTableClicked()));

  QPushButton* toggleSelected = new QPushButton(occasionally_in_collision_page_);
  toggleSelected->setText("Toggle Selected");
  layout->addWidget(toggleSelected);
  connect(toggleSelected, SIGNAL(clicked()), this, SLOT(occasionallyTogglePushed()));
  toggleSelected->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  //addPage(occasionally_in_collision_page_);
  setPage(OccasionallyInCollisionPage, occasionally_in_collision_page_);
  occasionally_in_collision_page_->setLayout(layout);
}

void PlanningDescriptionConfigurationWizard::initOutputFilesPage()
{
  output_files_page_ = new QWizardPage(this);
  output_files_page_->setTitle("Output Files");
  QVBoxLayout* layout = new QVBoxLayout(output_files_page_);

  output_files_page_->setSubTitle("Done! The wizard will auto-generate a stack called <your_robot_name>_arm_navigation "
                                    "in the selected folder when you click the generate button below.");

  package_path_field_ = new QLineEdit(output_files_page_);
  package_path_field_->setText("<absolute directory path>");
  layout->addWidget(package_path_field_);

  QPushButton* selectFileButton = new QPushButton(output_files_page_);
  selectFileButton->setText("Select Directory ...");
  selectFileButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  file_selector_ = new QFileDialog(this);
  file_selector_->setFileMode(QFileDialog::Directory);
  file_selector_->setOption(QFileDialog::ShowDirsOnly, true);

  connect(file_selector_, SIGNAL(fileSelected(const QString&)), this, SLOT(fileSelected(const QString&)));
  connect(selectFileButton, SIGNAL(clicked()), file_selector_, SLOT(open()));
  layout->addWidget(selectFileButton);

  QPushButton* generateButton = new QPushButton(output_files_page_);
  generateButton->setText("Generate Config Files...");
  generateButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  layout->addWidget(generateButton);

  progress_bar_ = new QProgressBar(output_files_page_);
  progress_label_ = new QLabel(output_files_page_);
  layout->addWidget(progress_bar_);
  layout->addWidget(progress_label_);

  connect(generateButton, SIGNAL(clicked()), this, SLOT(writeFiles()));
  //addPage(output_files_page_);
  setPage(OutputFilesPage, output_files_page_);
  output_files_page_->setLayout(layout);
}

void PlanningDescriptionConfigurationWizard::createLinkTree()
{
  const KinematicModel::JointModel* rootJoint = kmodel_->getRoot();
  addLinktoTreeRecursive(rootJoint->getChildLinkModel(), NULL);

  link_tree_->expandToDepth(0);
}

void PlanningDescriptionConfigurationWizard::addLinktoTreeRecursive(const KinematicModel::LinkModel* link,
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

bool PlanningDescriptionConfigurationWizard::addLinkChildRecursive(QTreeWidgetItem* parent,
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
  endwin();
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

  /*
   initscr();
   use_default_colors();
   start_color();

   pdcw->outputPlanningEnvironmentLaunch();

   pdcw->setupGroups();

   pdcw->outputKinematicsLaunchFiles();
   pdcw->outputPlanningComponentVisualizerLaunchFile();
   pdcw->outputOMPLGroupYAML();
   pdcw->outputOMPLLaunchFile();
   pdcw->outputTrajectoryFilterLaunch();
   pdcw->outputJointLimitsYAML();

   pdcw->setJointsForCollisionSampling();

   pdcw->considerAlwaysAndDefaultInCollisionMarkers();

   printw("Finding often in collision pairs\n");
   refresh();
   pdcw->considerOftenInCollisionPairs();

   printw("Finding occasionally in collision pairs\n");
   refresh();
   pdcw->considerOccasionallyInCollisionPairs();

   printw("Performance testing and writing to file\n");
   refresh();
   pdcw->outputPlanningDescriptionYAML();
   printw("Press any key to exit\n");
   refresh();
   getch();
   endwin();
   ros::shutdown();
   */

  return qtApp.exec();
}

