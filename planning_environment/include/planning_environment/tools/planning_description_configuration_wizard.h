#ifndef PLANNING_DESCRIPTION_CONFIGURATION_thisH
#define PLANNING_DESCRIPTION_CONFIGURATION_thisH
#include <termios.h>
#include <signal.h>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <math.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <planning_environment/models/collision_models.h>
#include <planning_environment/models/model_utils.h>
#include <tf/transform_broadcaster.h>
#include <collision_space/environmentODE.h>
#include <rosgraph_msgs/Clock.h>
#include <planning_environment/util/collision_operations_generator.h>
#include <qt4/QtGui/qwidget.h>
#include <qt4/QtGui/qwizard.h>
#include <qt4/QtGui/qlabel.h>
#include <qt4/QtGui/qcombobox.h>
#include <qt4/QtGui/qtreewidget.h>
#include <qt4/QtGui/qtablewidget.h>
#include <qt4/QtGui/qlineedit.h>
#include <qt4/QtGui/qgridlayout.h>
#include <qt4/QtGui/qgroupbox.h>
#include <qt4/QtGui/qformlayout.h>
#include <qt4/QtGui/qpushbutton.h>
#include <qt4/QtGui/qcheckbox.h>
#include <qt4/QtGui/qdialog.h>
#include <ncurses.h>
#include <tinyxml/tinyxml.h>

static const std::string VIS_TOPIC_NAME = "planning_description_configuration_wizard";
static const unsigned int CONTROL_SPEED = 10;
static const double DEFAULT_ACCELERATION = 1.0;

class PlanningDescriptionConfigurationWizard : public QWizard {
    Q_OBJECT

public:
  enum WizardPage
  {
    SetupGroupsPage,
    KinematicChainsPage,
    JointCollectionsPage,
    SelectDOFPage,
    AlwaysInCollisionPage,
    OftenInCollisionPage,
    OccasionallyInCollisionPage,
    OutputFilesPage
  };

  PlanningDescriptionConfigurationWizard(const std::string& urdf_package, const std::string& urdf_path, QWidget* parent = NULL) :
    inited_(false), world_joint_config_("world_joint"), urdf_package_(urdf_package), urdf_path_(urdf_path), QWizard(parent)
  {
    std::string full_urdf_path = ros::package::getPath(urdf_package_)+urdf_path_;

    ROS_INFO_STREAM("full path name is " << full_urdf_path);

    urdf_ = boost::shared_ptr<urdf::Model>(new urdf::Model());
    bool urdf_ok = urdf_->initFile(full_urdf_path);

    if(!urdf_ok) {
      ROS_WARN_STREAM("Urdf file " << full_urdf_path << " not ok");
      return;
    }

    //making directories
    dir_name_ = getRobotName()+"_arm_navigation";

    std::string del_com = "rm -rf " + dir_name_;
    int ok = system(del_com.c_str());

    std::string mdir = "roscreate-pkg "+dir_name_;
    mdir += " planning_environment arm_kinematics_constraint_aware ompl_ros_interface ";
    mdir += "trajectory_filter_server constraint_aware_spline_smoother move_arm";
    ok = system(mdir.c_str());

    mdir ="mkdir -p "+dir_name_+"/config";
    ok = system(mdir.c_str());
    if(ok != 0) {
      ROS_WARN_STREAM("Making subdirectory not ok");
      return;
    }

    mdir = "mkdir -p "+dir_name_+"/launch";
    ok = system(mdir.c_str());
    if(ok != 0) {
      ROS_WARN_STREAM("Making subdirectory not ok");
      return;
    }

    yaml_outfile_name_ = getRobotName()+"_planning_description.yaml";
    full_yaml_outfile_name_ = dir_name_+"/config/"+yaml_outfile_name_;
    launch_outfile_name_ = getRobotName()+"_planning_environment.launch";
    full_launch_outfile_name_ = dir_name_+"/launch/"+launch_outfile_name_;

    //pushing to the param server
    std::string com = "rosparam set robot_description -t "+full_urdf_path;

    ok = system(com.c_str());

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

    setupQtPages();
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
    world_joint_config_.type = joint_type;
    world_joint_config_.parent_frame_id = ff;
    world_joint_config_.child_frame_id = root->name;
    multi_dof_configs.push_back(world_joint_config_);

    deleteKinematicStates();

    if(cm_ != NULL) {
      delete cm_;
    }
    if(ops_gen_ != NULL) {
      delete ops_gen_;
    }
    kmodel_ = new planning_models::KinematicModel(*urdf_, gcs, multi_dof_configs);

    if(kmodel_->getRoot() == NULL) {
      ROS_INFO_STREAM("Kinematic root is NULL");
      lock_.unlock();
      return false;
    }

    robot_state_ = new planning_models::KinematicState(kmodel_);
    robot_state_->setKinematicStateToDefault();

    lock_.unlock();

    return true;
  }

  void emitWorldJointYAML() {

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

  void setupGroups() {

    while(1) {
      clear();
      std::vector<std::string> group_names;
      kmodel_->getModelGroupNames(group_names);
      printw("Current groups:\n");
      for(unsigned int i = 0; i < group_names.size(); i++) {
        printw("%d) %s\n", i, group_names[i].c_str());
      }
      printw("Enter 0 to accept current group set\n");
      printw("Enter an 'x' followed by the group number to delete a current group.\n");
      printw("Enter 1 to add a group based on kinematic chain\n");
      printw("Enter 2 to add a group based on a joint collection\n");
      printw("Enter 3 to add a group based on a subgroup collection\n");
      refresh();
      char str[80];
      getstr(str);
      if(str[0] == 'x') {
        unsigned int entry;
        std::stringstream ss(&str[0]);
        ss >> entry;
        lock_.lock();
        deleteKinematicStates();
        kmodel_->removeModelGroup(group_names[entry]);
        robot_state_ = new planning_models::KinematicState(kmodel_);
        robot_state_->setKinematicStateToDefault();
        lock_.unlock();
      } else {
        unsigned int entry;
        std::stringstream ss(str);
        ss >> entry;
        if(entry == 0) break;
        printw("Enter name for new group: ");
        getstr(str);
        std::stringstream ss2(str);
        std::string new_group_name;
        ss2 >> new_group_name;
        if(entry == 0) {
          break;
        } else if(entry == 1) {
          setupGroupKinematicChain(new_group_name);
        } else if(entry == 2) {
          setupGroupJointCollection(new_group_name);
        } // else {
        //   setupGroupSubgroupCollection(new_group_name);
        // }
      }
    }
  }

  void setupGroupKinematicChain(const std::string& new_group_name) {
    const std::vector<planning_models::KinematicModel::LinkModel*>& lmv = kmodel_->getLinkModels();
    bool has_base = false;
    unsigned int base_num = 0;
    bool has_tip = false;
    unsigned int tip_num = 0;
    bool group_ok = false;
    std::string last_status;
    while(1) {
      clear();
      refresh();
      for(unsigned int i = 0; i < lmv.size(); i++) {
        printw("%-3d ", i);
        if(has_base && i == base_num) {
          printw("(B) ");
        } else if(has_tip && i == tip_num) {
          printw("(T) ");
        } else {
          printw("( )");
        }
        printw("%s\n", lmv[i]->getName().c_str());
      }
      printw("New group name: %s\n", new_group_name.c_str());
      printw("Enter 'b' followed by a link number to set the base link for the group.\n");
      printw("Enter 't' followed by a link number to set the base link for the group.\n");
      printw("Enter 'q' to exit\n");
      if(has_tip && has_base) {
        printw("Enter 'x' to validate/visualize the group\n");
      }
      if(group_ok) {
        printw("Visualization shows group links in red and updated links in green\n");
        printw("Enter 'a' to accept group\n");
      }
      if(!last_status.empty()) {
        printw("Last status msg: %s\n", last_status.c_str());
      }
      refresh();
      char str[80];
      getstr(str);
      if(str[0] == 'q') {
        break;
      } else if(str[0] == 'b' || str[0] == 't') {
        std::stringstream ss(&str[1]);
        unsigned int entry;
        ss >> entry;
        if(str[0] == 'b') {
          base_num = entry;
          has_base = true;
        } else {
          tip_num = entry;
          has_tip = true;
        }
      } else if(has_tip && has_base) {
        if(str[0] == 'a') {
          if(!group_ok) {
            last_status = "Must validate group before accepting";
            continue;
          } else {
            current_show_group_ = "";
            break;
          }
        }
        lock_.lock();
        deleteKinematicStates();
        if(kmodel_->hasModelGroup(new_group_name)) {
          kmodel_->removeModelGroup(new_group_name);
        }
        planning_models::KinematicModel::GroupConfig gc(new_group_name,
                                                        lmv[base_num]->getName(),
                                                        lmv[tip_num]->getName());
        group_ok = kmodel_->addModelGroup(gc);
        robot_state_ = new planning_models::KinematicState(kmodel_);
        robot_state_->setKinematicStateToDefault();
        if(group_ok) {
          current_show_group_ = new_group_name;
          last_status = "Group " + current_show_group_ + " ok";
        } else {
          current_show_group_ = "";
          last_status = "Group not ok";
        }
        lock_.unlock();
      }
    }
  }

  bool addGroup(std::string new_group_name, std::string base, std::string tip)
  {
    lock_.lock();
    if(kmodel_->hasModelGroup(new_group_name)) {
      kmodel_->removeModelGroup(new_group_name);
    }
    planning_models::KinematicModel::GroupConfig gc(new_group_name,
                                                    base,
                                                    tip);
    bool group_ok = kmodel_->addModelGroup(gc);
    robot_state_ = new planning_models::KinematicState(kmodel_);
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

    if(group_ok) {
      current_show_group_ = new_group_name;
    } else {
      current_show_group_ = "";
    }

    lock_.unlock();
    return group_ok;
  }

  void popupNotOkayWarning()
  {
    not_ok_dialog_->show();
  }

  void popupOkayWarning()
  {
    ok_dialog_->show();
  }

  void updateGroupTable()
  {
    current_group_table_->clear();
    std::vector<std::string> modelNames;
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

  void setupGroupJointCollection(const std::string& new_group_name) {
    const std::vector<planning_models::KinematicModel::JointModel*>& jmv = kmodel_->getJointModels();
    std::vector<bool> is_included(jmv.size(), false);
    while(1) {
      clear();
      for(unsigned int i = 0; i < jmv.size(); i++) {
        printw("%-3d ", i);
        if(is_included[i]) {
          printw("(X) ");
        } else {
          printw("( )");
        }
        printw("%s\n", jmv[i]->getName().c_str());
      }
      printw("New group name: %s\n", new_group_name.c_str());
      printw("Enter a joint number or two numbers seperated by a ':' to toggle inclusion\n");
      printw("Enter an 'a' followed by a joint number to toggle that joint and all downstream joints\n");
      printw("Enter 'r' to reset all entries\n");
      printw("Enter 'v' to visualize all member and updated links of the current selection (shown in green)\n");
      printw("Enter 'x' to accept this joint collection\n");
      refresh();
      char str[80];
      getstr(str);
      if(str[0] == 'x' || str[0] == 'v') {
        lock_.lock();
        std::vector<std::string> joints;
        for(unsigned int i = 0; i < is_included.size(); i++) {
          if(is_included[i]) {
            joints.push_back(jmv[i]->getName());
          }
        }
        deleteKinematicStates();
        if(kmodel_->hasModelGroup(new_group_name)) {
          kmodel_->removeModelGroup(new_group_name);
        }
        std::vector<std::string> emp;
        planning_models::KinematicModel::GroupConfig gc(new_group_name,
                                                        joints,
                                                        emp);
        bool group_ok = kmodel_->addModelGroup(gc);
        robot_state_ = new planning_models::KinematicState(kmodel_);
        robot_state_->setKinematicStateToDefault();
        if(!group_ok) {
          ROS_ERROR_STREAM("Joint collection group really should be ok");
          current_show_group_ = "";
        } else {
          if(str[0] == 'v') {
            current_show_group_ = new_group_name;
          }
        }
        lock_.unlock();
        if(str[0] == 'x') {
          break;
        }
      } else if(str[0] == 'r') {
        for(unsigned int i = 0; i < is_included.size(); i++) {
          is_included[i] = false;
        }
      } else if(str[0] == 'a') {
        std::stringstream ss(&str[1]);
        unsigned int entry;
        ss >> entry;
        std::vector<std::string> joints = kmodel_->getChildJointModelNames(jmv[entry]);
        for(unsigned int i = 0; i < joints.size(); i++) {
          for(unsigned int j = 0; j < jmv.size(); j++) {
            if(joints[i] == jmv[j]->getName()) {
              is_included[j] = !is_included[j];
              break;
            }
          }
        }
      } else {
        unsigned int entry;
        std::stringstream ss(str);
        ss >> entry;
        char c;
        ss >> c;
        if(c != ':') {
          is_included[entry] = !is_included[entry];
        } else {
          unsigned int entry2;
          ss >> entry2;
          for(unsigned int q = entry; q <= entry2; q++) {
            is_included[q] = !is_included[q];
          }
        }
      }
    }
    lock_.lock();
    current_show_group_ = "";
    lock_.unlock();
  }


  void emitGroupYAML() {
    emitter_ << YAML::Key << "groups";
    emitter_ << YAML::Value << YAML::BeginSeq;

    const std::map<std::string, planning_models::KinematicModel::GroupConfig>& group_config_map = kmodel_->getJointModelGroupConfigMap();

    for(std::map<std::string, planning_models::KinematicModel::GroupConfig>::const_iterator it = group_config_map.begin();
        it != group_config_map.end();
        it++) {
      emitter_ << YAML::BeginMap;
      emitter_ << YAML::Key << "name" << YAML::Value << it->first;
      if(!it->second.base_link_.empty()) {
        emitter_ << YAML::Key << "base_link" << YAML::Value << it->second.base_link_;
        emitter_ << YAML::Key << "tip_link" << YAML::Value << it->second.tip_link_;
      } else {
        if(!it->second.subgroups_.empty()) {
          emitter_ << YAML::Key << "subgroups";
          emitter_ << YAML::Value << YAML::BeginSeq;
          for(unsigned int i = 0; i < it->second.subgroups_.size(); i++) {
            emitter_ << it->second.subgroups_[i];
          }
          emitter_ << YAML::EndSeq;
        }
        if(!it->second.joints_.empty()) {
          emitter_ << YAML::Key << "joints";
          emitter_ << YAML::Value << YAML::BeginSeq;
          for(unsigned int i = 0; i < it->second.joints_.size(); i++) {
            emitter_ << it->second.joints_[i];
          }
          emitter_ << YAML::EndSeq;
        }
      }
      emitter_ << YAML::EndMap;
    }
    emitter_ << YAML::EndSeq;
  }

  // void setupGroupSubgroupCollection(const std::string& new_group_name) {
  //   while(1) {
  //     clear();
  //     std::vector<std::string> group_names;
  //     kmodel_->getModelGroupNames(group_names);
  //     std::vector<bool> is_included(group_names.size(), false);
  //     for(unsigned int i = 0; i < group_names.size(); i++) {
  //       printw("%d) ", i);
  //       if(is_included[i]) {
  //         printw("(X)");
  //       } else {
  //         printw("( )");
  //       }
  //       printw(" %s\n", group_names[i].c_str());
  //     }
  //     printw("Enter a subgroup number to toggle inclusion\n");
  //     printw("Enter 'v' to visualize current subgroup\n");
  //     printw("Enter 'x' to accept current subgroup\n");
  //     refresh();
  //     char str[80];
  //     getstr(str);
  //     unsigned int entry;
  //     std::stringstream ss(str);
  //     ss >> entry;
  //     if(entry == 0) {

  //     }
  //   }
  // }

  void outputJointLimitsYAML() {
    std::map<std::string, bool> already;
    boost::shared_ptr<urdf::Model> robot_model = urdf_;
    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "joint_limits";
    emitter << YAML::Value << YAML::BeginMap;
    const std::map<std::string, planning_models::KinematicModel::JointModelGroup*>& group_map = kmodel_->getJointModelGroupMap();
    for(std::map<std::string, planning_models::KinematicModel::JointModelGroup*>::const_iterator it = group_map.begin();
        it != group_map.end();
        it++) {
      const std::vector<const planning_models::KinematicModel::JointModel*>& jmv = it->second->getJointModels();

      for(unsigned int i = 0; i < jmv.size(); i++) {
        boost::shared_ptr<const urdf::Joint> urdf_joint = robot_model->getJoint(jmv[i]->getName());
        double max_vel = 0.0;
        if(urdf_joint) {
          max_vel = urdf_joint->limits->velocity;
        }
        const std::map<std::string, std::pair<double, double> >& bounds_map = jmv[i]->getAllVariableBounds();
        for(std::map<std::string, std::pair<double, double> >::const_iterator it2 = bounds_map.begin();
            it2 != bounds_map.end();
            it2++) {
          if(already.find(it2->first) == already.end()) {
            already[it2->first] = true;
            emitter << YAML::Key << it2->first;
            emitter << YAML::Value << YAML::BeginMap;
            emitter << YAML::Key << "has_position_limits";
            const planning_models::KinematicModel::RevoluteJointModel* rev
              = dynamic_cast<const planning_models::KinematicModel::RevoluteJointModel*>(jmv[i]);
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
    std::ofstream outf((dir_name_+"/config/joint_limits.yaml").c_str(), std::ios_base::trunc);

    outf << emitter.c_str();
  }

  void setJointsForCollisionSampling() {
    ode_collision_model_ = new collision_space::EnvironmentModelODE();

    const std::vector<planning_models::KinematicModel::LinkModel*>& coll_links = kmodel_->getLinkModelsWithCollisionGeometry();

    std::vector<std::string> coll_names;
    for(unsigned int i = 0; i < coll_links.size(); i++) {
      coll_names.push_back(coll_links[i]->getName());
    }
    collision_space::EnvironmentModel::AllowedCollisionMatrix default_collision_matrix(coll_names,false);
    std::map<std::string, double> default_link_padding_map;
    ode_collision_model_->setRobotModel(kmodel_, default_collision_matrix,
                                        default_link_padding_map, 0.0, 1.0);


    cm_ = new planning_environment::CollisionModels(urdf_,
                                                    kmodel_,
                                                    ode_collision_model_);
    ops_gen_ = new planning_environment::CollisionOperationsGenerator(cm_);

    const std::vector<planning_models::KinematicModel::JointModel*>& jmv = cm_->getKinematicModel()->getJointModels();
    std::vector<bool> consider_dof;
    //assuming that 0th is world joint, which we don't want to include
    for(unsigned int i = 1; i < jmv.size(); i++) {
      const std::map<std::string, std::pair<double, double> >& joint_bounds = jmv[i]->getAllVariableBounds();
      for(std::map<std::string, std::pair<double, double> >::const_iterator it = joint_bounds.begin();
          it != joint_bounds.end();
          it++) {
        consider_dof.push_back(true);
      }
    }

    while(1) {
      clear();
      refresh();
      int ind = 1;
      for(unsigned int i = 1; i < jmv.size(); i++) {
        const std::map<std::string, std::pair<double, double> >& joint_bounds = jmv[i]->getAllVariableBounds();
        for(std::map<std::string, std::pair<double, double> >::const_iterator it = joint_bounds.begin();
            it != joint_bounds.end();
            it++) {
          printw("%d) (%c) Dof name: %s  Lower bound: %g   Upper bound: %g\n", ind, (consider_dof[ind-1] ? 'X' : ' '),
                 it->first.c_str(), it->second.first, it->second.second);
          ind++;
        }
      }
      printw("Enter a number to toggle DOF for collision sampling purposes, two numbers seperated by a ':' to toggle a range(inclusive), or 0 to accept\n");
      refresh();
      char str[80];
      getstr(str);
      unsigned int entry;
      std::stringstream ss(str);
      ss >> entry;
      if(entry == 0) break;
      char c;
      ss >> c;
      if(c != ':') {
        consider_dof[entry-1] = !consider_dof[entry-1];
      } else {
        unsigned int entry2;
        ss >> entry2;
        for(unsigned int q = entry-1; q < entry2; q++) {
          consider_dof[q] = !consider_dof[q];
        }
      }
    }
    int xind = 0;
    std::map<std::string, bool> cdof_map;
    for(unsigned int i = 1; i < jmv.size(); i++) {
      const std::map<std::string, std::pair<double, double> >& joint_bounds = jmv[i]->getAllVariableBounds();
      for(std::map<std::string, std::pair<double, double> >::const_iterator it = joint_bounds.begin();
          it != joint_bounds.end();
          it++) {
        cdof_map[it->first] = consider_dof[xind++];
      }
    }
    clear();
    refresh();
    ops_gen_->generateSamplingStructures(cdof_map);
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

  void considerAlwaysAndDefaultInCollisionMarkers() {
    std::vector<planning_environment::CollisionOperationsGenerator::StringPair> always_in_collision;
    std::vector<planning_environment::CollisionOperationsGenerator::CollidingJointValues> in_collision_joint_values;

    ops_gen_->generateAlwaysInCollisionPairs(always_in_collision, in_collision_joint_values);

    lock_.lock();
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
    clear();
    refresh();
    printw("These pairs (with yellow collision markers) are always in collision.  Collisions will be disabled.\n");
    printw("Press any key to continue.\n");
    refresh();
    lock_.unlock();
    getch();

    ops_gen_->disablePairCollisionChecking(always_in_collision);
    disable_map_[planning_environment::CollisionOperationsGenerator::ALWAYS] = always_in_collision;
    std::vector<planning_environment::CollisionOperationsGenerator::StringPair> default_in_collision;
    ops_gen_->generateDefaultInCollisionPairs(default_in_collision, in_collision_joint_values);

    std_msgs::ColorRGBA default_color;
    default_color.a = 1.0;
    default_color.r = 0.0;
    default_color.g = .8;
    default_color.b = 0.04;

    std::vector<double> percentages(default_in_collision.size(), 1.0);
    clear();
    refresh();
    printw("These pairs (with green collision markers) are in collision in the default state.  Collisions will be optionally disabled.\n");
    considerInCollisionPairs(default_in_collision,
                             percentages,
                             in_collision_joint_values,
                             default_color);
    disable_map_[planning_environment::CollisionOperationsGenerator::DEFAULT] = default_in_collision;

  }

  void considerOftenInCollisionPairs() {

    std::vector<planning_environment::CollisionOperationsGenerator::StringPair> often_in_collision;
    std::vector<double> percentages;
    std::vector<planning_environment::CollisionOperationsGenerator::CollidingJointValues> in_collision_joint_values;

    ops_gen_->generateOftenInCollisionPairs(often_in_collision, percentages, in_collision_joint_values);

    if(often_in_collision.size() == 0) {
      printw("No additional often in collision pairs\n");
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
    printw("These pairs (with magenta collision markers) are often in collision.  Collisions will be optionally disabled.\n");


    considerInCollisionPairs(often_in_collision,
                             percentages,
                             in_collision_joint_values,
                             color);
    disable_map_[planning_environment::CollisionOperationsGenerator::OFTEN] = often_in_collision;
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

    clear();
    refresh();
    //printw("These pairs (with magenta collision markers) are ooccasionally in collision.  Collisions will be optionally disabled.\n");

    // considerInCollisionPairs(in_collision,
    //                          percentages,
    //                          in_collision_joint_values,
    //                          color);

    //disable_map_[planning_environment::CollisionOperationsGenerator::OCCASIONALLY] = in_collision;
    disable_map_[planning_environment::CollisionOperationsGenerator::NEVER] = not_in_collision;

  }

  void considerInCollisionPairs(std::vector<planning_environment::CollisionOperationsGenerator::StringPair>& in_collision_pairs,
                                std::vector<double>& percentages,
                                std::vector<planning_environment::CollisionOperationsGenerator::CollidingJointValues>& in_collision_joint_values,
                                const std_msgs::ColorRGBA& color
) {
    std::vector<planning_environment::CollisionOperationsGenerator::StringPair> actually_disabling;
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
        ROS_WARN_STREAM("Collision that should be there not found between " << in_collision_pairs[i].first << " and " << in_collision_pairs[i].second
                        << " " << cm_->isKinematicStateInCollision(*robot_state_));
        for(unsigned int j = 0; j < coll_space_contacts.size(); j++) {
          ROS_INFO_STREAM("Contacts between " << coll_space_contacts[j].body_name_1 << " and " << coll_space_contacts[j].body_name_2);
        }
      } else {
        printw("Disable all collisions between %s and %s (frequency in collision %g) (y or n)?", in_collision_pairs[i].first.c_str(), in_collision_pairs[i].second.c_str(), percentages[i]);
        refresh();
        char str[80];
        getstr(str);
        if(str[0] != 'n') {
          ops_gen_->disablePairCollisionChecking(in_collision_pairs[i]);
          actually_disabling.push_back(in_collision_pairs[i]);
        }
      }
    }
    in_collision_pairs = actually_disabling;
  }

  void outputPlanningDescriptionYAML() {
    //initial map
    emitter_ << YAML::BeginMap;
    emitWorldJointYAML();
    emitGroupYAML();
    //ops_gen_->performanceTestSavedResults(disable_map_);
    ops_gen_->outputYamlStringOfSavedResults(emitter_, disable_map_);
    //end map
    emitter_ << YAML::EndMap;
    std::ofstream outf(full_yaml_outfile_name_.c_str(), std::ios_base::trunc);

    outf << emitter_.c_str();
  }

  void outputOMPLGroupYAML() {
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
    emitter << YAML::EndMap;
    std::ofstream outf((dir_name_+"/config/ompl_planning.yaml").c_str(), std::ios_base::trunc);

    outf << emitter.c_str();
  }

  void outputOMPLLaunchFile() {
    TiXmlDocument doc;
    TiXmlElement* launch_root = new TiXmlElement("launch");
    doc.LinkEndChild(launch_root);

    TiXmlElement *inc = new TiXmlElement("include");
    launch_root->LinkEndChild(inc);
    inc->SetAttribute("file","$(find "+dir_name_+")/launch/"+launch_outfile_name_);

    TiXmlElement *node = new TiXmlElement("node");
    launch_root->LinkEndChild(node);
    node->SetAttribute("pkg","ompl_ros_interface");
    node->SetAttribute("type", "ompl_ros");
    node->SetAttribute("name", "ompl_planning");

    TiXmlElement *rp = new TiXmlElement("rosparam");
    node->LinkEndChild(rp);
    rp->SetAttribute("command","load");
    rp->SetAttribute("file", "$(find "+dir_name_+")/config/ompl_planning.yaml");
    doc.SaveFile(dir_name_+"/launch/ompl_planning.launch");
  }

  void outputTrajectoryFilterLaunch() {
    TiXmlDocument doc;
    TiXmlElement* launch_root = new TiXmlElement("launch");
    doc.LinkEndChild(launch_root);

    TiXmlElement *inc = new TiXmlElement("include");
    launch_root->LinkEndChild(inc);
    inc->SetAttribute("file","$(find "+dir_name_+")/launch/"+launch_outfile_name_);

    TiXmlElement *node = new TiXmlElement("node");
    launch_root->LinkEndChild(node);
    node->SetAttribute("pkg","trajectory_filter_server");
    node->SetAttribute("type","trajectory_filter_server");
    node->SetAttribute("name", "trajectory_filter_server");

    TiXmlElement *rp = new TiXmlElement("rosparam");
    node->LinkEndChild(rp);
    rp->SetAttribute("command","load");
    rp->SetAttribute("file", "$(find trajectory_filter_server)/config/filters.yaml");

    TiXmlElement *rp2 = new TiXmlElement("rosparam");
    node->LinkEndChild(rp2);
    rp2->SetAttribute("command","load");
    rp2->SetAttribute("file", "$(find "+dir_name_+")/config/joint_limits.yaml");

    doc.SaveFile(dir_name_+"/launch/trajectory_filter_server.launch");
  }

  void outputPlanningEnvironmentLaunch() {
    TiXmlDocument doc;
    TiXmlElement* launch_root = new TiXmlElement("launch");
    doc.LinkEndChild(launch_root);

    TiXmlElement *rd = new TiXmlElement("param");
    launch_root->LinkEndChild(rd);
    rd->SetAttribute("name", "robot_description");
    rd->SetAttribute("textfile", "$(find "+urdf_package_+")"+urdf_path_);

    TiXmlElement *rp = new TiXmlElement("rosparam");
    launch_root->LinkEndChild(rp);
    rp->SetAttribute("command","load");
    rp->SetAttribute("ns", "robot_description_planning");
    rp->SetAttribute("textfile", "$(find "+dir_name_+")/config/"+yaml_outfile_name_);
    doc.SaveFile(full_launch_outfile_name_);

  }

  void outputKinematicsLaunchFiles() {
    TiXmlDocument doc;
    TiXmlElement* launch_root = new TiXmlElement("launch");
    doc.LinkEndChild(launch_root);

    TiXmlElement *inc = new TiXmlElement("include");
    launch_root->LinkEndChild(inc);
    inc->SetAttribute("file","$(find "+dir_name_+")/launch/"+launch_outfile_name_);

    const std::map<std::string, planning_models::KinematicModel::GroupConfig>& group_config_map = kmodel_->getJointModelGroupConfigMap();

    for(std::map<std::string, planning_models::KinematicModel::GroupConfig>::const_iterator it = group_config_map.begin();
        it != group_config_map.end();
        it++) {
      if(!it->second.base_link_.empty()) {
        TiXmlElement *node = new TiXmlElement("node");
        launch_root->LinkEndChild(node);
        node->SetAttribute("pkg","arm_kinematics_constraint_aware");
        node->SetAttribute("type","arm_kinematics_constraint_aware");
        node->SetAttribute("name", getRobotName()+"_"+it->first+"_kinematics");

        TiXmlElement *group_param = new TiXmlElement("param");
        node->LinkEndChild(group_param);
        group_param->SetAttribute("name", "group");
        group_param->SetAttribute("type", "string");
        group_param->SetAttribute("value", it->first);

        TiXmlElement *base_param = new TiXmlElement("param");
        node->LinkEndChild(base_param);
        base_param->SetAttribute("name", it->first+"/root_name");
        base_param->SetAttribute("type", "string");
        base_param->SetAttribute("value", it->second.base_link_);

        TiXmlElement *tip_param = new TiXmlElement("param");
        node->LinkEndChild(tip_param);
        tip_param->SetAttribute("name", it->first+"/tip_name");
        tip_param->SetAttribute("type", "string");
        tip_param->SetAttribute("value", it->second.tip_link_);

        TiXmlElement *solver_param = new TiXmlElement("param");
        node->LinkEndChild(solver_param);
        solver_param->SetAttribute("name", "kinematics_solver");
        solver_param->SetAttribute("type", "string");
        solver_param->SetAttribute("value", "arm_kinematics_constraint_aware/KDLArmKinematicsPlugin");
      }
    }


    doc.SaveFile(dir_name_+"/launch/constraint_aware_kinematics.launch");
  }

  void outputPlanningComponentVisualizerLaunchFile() {
    TiXmlDocument doc;
    TiXmlElement* launch_root = new TiXmlElement("launch");
    doc.LinkEndChild(launch_root);

    TiXmlElement *inc = new TiXmlElement("include");
    launch_root->LinkEndChild(inc);
    inc->SetAttribute("file","$(find "+dir_name_+")/launch/"+launch_outfile_name_);

    TiXmlElement *pre = new TiXmlElement("include");
    launch_root->LinkEndChild(pre);
    pre->SetAttribute("file","$(find planning_environment)/launch/planning_environment_visualization_prerequisites.launch");

    TiXmlElement *kin = new TiXmlElement("include");
    launch_root->LinkEndChild(kin);
    kin->SetAttribute("file", "$(find "+dir_name_+")/launch/constraint_aware_kinematics.launch");

    TiXmlElement *ompl = new TiXmlElement("include");
    launch_root->LinkEndChild(ompl);
    ompl->SetAttribute("file", "$(find "+dir_name_+")/launch/ompl_planning.launch");

    TiXmlElement *fil = new TiXmlElement("include");
    launch_root->LinkEndChild(fil);
    fil->SetAttribute("file", "$(find "+dir_name_+")/launch/trajectory_filter_server.launch");

    TiXmlElement *vis = new TiXmlElement("node");
    launch_root->LinkEndChild(vis);
    vis->SetAttribute("pkg","move_arm");
    vis->SetAttribute("type","planning_components_visualizer");
    vis->SetAttribute("name","planning_components_visualizer");
    vis->SetAttribute("output","screen");

    TiXmlElement *state_publisher = new TiXmlElement("node");
    launch_root->LinkEndChild(state_publisher);
    state_publisher->SetAttribute("pkg","robot_state_publisher" );
    state_publisher->SetAttribute("type", "state_publisher");
    state_publisher->SetAttribute("name", "rob_st_pub");

    doc.SaveFile(dir_name_+"/launch/planning_components_visualizer.launch");
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
    if(!current_show_group_.empty()) {
      visualization_msgs::MarkerArray arr;
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

      const planning_models::KinematicModel::JointModelGroup* jmg = kmodel_->getModelGroup(current_show_group_);

      std::vector<std::string> group_link_names = jmg->getGroupLinkNames();
      getRobotMeshResourceMarkersGivenState(*robot_state_,
                                            arr,
                                            default_color,
                                            current_show_group_,
                                            ros::Duration(.2),
                                            &group_link_names);

      std::vector<std::string> updated_link_model_names = jmg->getUpdatedLinkModelNames();
      std::map<std::string, bool> dont_include;
      for(unsigned int i = 0; i < group_link_names.size(); i++) {
        dont_include[group_link_names[i]] = true;
      }

      std::vector<std::string> ex_list;
      for(unsigned int i = 0; i < updated_link_model_names.size(); i++) {
        if(dont_include.find(updated_link_model_names[i]) == dont_include.end()) {
          ex_list.push_back(updated_link_model_names[i]);
        }
      }
      //first n will be actually in group
      getRobotMeshResourceMarkersGivenState(*robot_state_,
                                            arr,
                                            color,
                                            current_show_group_+"_updated_links",
                                            ros::Duration(.2),
                                            &ex_list);
      vis_marker_array_publisher_.publish(arr);
    }
    lock_.unlock();
  }

  void sendTransforms() {
    lock_.lock();
    ros::WallTime cur_time = ros::WallTime::now();
    rosgraph_msgs::Clock c;
    c.clock.nsec = cur_time.nsec;
    c.clock.sec = cur_time.sec;
    std::vector<geometry_msgs::TransformStamped> trans_vector;
    planning_environment::getAllKinematicStateStampedTransforms(*robot_state_, trans_vector, c.clock);
    transform_broadcaster_.sendTransform(trans_vector);
    lock_.unlock();
  };

  bool isInited() const {
    return inited_;
  }

  planning_environment::CollisionOperationsGenerator* getOperationsGenerator()
  {
    return ops_gen_;
  }

  std::string getRobotName() {
    return urdf_->getName();
  }

  void getRobotMeshResourceMarkersGivenState(const planning_models::KinematicState& state,
                                             visualization_msgs::MarkerArray& arr,
                                             const std_msgs::ColorRGBA& color,
                                             const std::string& name,
                                             const ros::Duration& lifetime,
                                             const std::vector<std::string>* names) const
  {
    boost::shared_ptr<urdf::Model> robot_model = urdf_;

    std::vector<std::string> link_names;
    if(names == NULL) {
      kmodel_->getLinkModelNames(link_names);
    } else {
      link_names = *names;
    }

    for(unsigned int i = 0; i < link_names.size(); i++) {
      boost::shared_ptr<const urdf::Link> urdf_link = robot_model->getLink(link_names[i]);
      if(!urdf_link) {
        ROS_INFO_STREAM("Invalid urdf name " << link_names[i]);
        continue;
      }
      if(!urdf_link->collision) {
        continue;
      }
      const urdf::Geometry *geom = urdf_link->collision->geometry.get();
      if(!geom) {
        continue;
      }
      const urdf::Mesh *mesh = dynamic_cast<const urdf::Mesh*>(geom);
      if(!mesh) {
        continue;
      }
      if(mesh->filename.empty()) {
        continue;
      }
      const planning_models::KinematicState::LinkState* ls = state.getLinkState(link_names[i]);
      if(ls == NULL) {
        ROS_WARN_STREAM("No link state for name " << names << " though there's a mesh");
        continue;
      }
      visualization_msgs::Marker mark;
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
      tf::poseTFToMsg(ls->getGlobalCollisionBodyTransform(),mark.pose);
      arr.markers.push_back(mark);
    }
  }

public slots:

void selectJointButtonClicked()
{
  QList<QTableWidgetItem*> selected =  joint_table_->selectedItems();

  for(int i = 0; i < selected.size(); i++)
  {
    std::string name = selected[i]->text().toStdString();
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

void deselectJointButtonClicked()
{
  QList<QTableWidgetItem*> deselected = selected_joint_table_->selectedItems();

  for(int i = 0; i < deselected.size(); i++)
  {
    selected_joint_table_->removeRow(deselected[i]->row());
  }
}

void deleteGroupButtonClicked()
{
  QList<QTableWidgetItem*> itemList = current_group_table_->selectedItems();

  for(int i = 0; i < itemList.size(); i++)
  {
    if(kmodel_->hasModelGroup(itemList[i]->text().toStdString()))
    {
      kmodel_->removeModelGroup(itemList[i]->text().toStdString());
    }
  }

  updateGroupTable();
}

void acceptChainClicked()
{
  addGroup(chain_name_field_->text().toStdString(),base_link_field_->text().toStdString(), tip_link_field_->text().toStdString());
  createDofPageTable();
}

void acceptGroupClicked()
{
  lock_.lock();
  std::string new_group_name = joint_group_name_field_->text().toStdString();
  std::vector<std::string> joints;
  for(int i = 0; i < selected_joint_table_->rowCount(); i++)
  {
    joints.push_back(selected_joint_table_->item(i, 0)->text().toStdString());
  }

  deleteKinematicStates();
   if(kmodel_->hasModelGroup(new_group_name))
   {
     kmodel_->removeModelGroup(new_group_name);
   }

   std::vector<std::string> emp;
   planning_models::KinematicModel::GroupConfig gc(new_group_name,
                                                   joints,
                                                   emp);
   bool group_ok = kmodel_->addModelGroup(gc);

   robot_state_ = new planning_models::KinematicState(kmodel_);
   robot_state_->setKinematicStateToDefault();
   if(!group_ok) {
     ROS_ERROR_STREAM("Joint collection group really should be ok");
     current_show_group_ = "";
     popupNotOkayWarning();
   }
   popupOkayWarning();
   updateGroupTable();
   lock_.unlock();

   createDofPageTable();
}

void baseLinkTreeClick()
{
  QTreeWidgetItem* item = link_tree_->currentItem();
  if(item != NULL)
  {
    base_link_field_->setText(item->text(0));
  }
}

void tipLinkTreeClick()
{
  QTreeWidgetItem* item = link_tree_->currentItem();
  if(item != NULL)
  {
    tip_link_field_->setText(item->text(0));
  }
}

void dofSelectionTableChanged()
{
  int xind = 0;
  std::map<std::string, bool> cdof_map;
  const std::vector<planning_models::KinematicModel::JointModel*>& jmv = cm_->getKinematicModel()->getJointModels();
  for(unsigned int i = 1; i < jmv.size(); i++)
  {
    const std::map<std::string, std::pair<double, double> >& joint_bounds = jmv[i]->getAllVariableBounds();
    for(std::map<std::string, std::pair<double, double> >::const_iterator it = joint_bounds.begin();
        it != joint_bounds.end();
        it++)
    {
      QCheckBox* checkBox = dynamic_cast<QCheckBox*>(dof_selection_table_->cellWidget(xind, 3));

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

void oftenCollisionTableChanged()
{
  std::vector<std::pair<std::string, std::string> >& disableVector =
      disable_map_[planning_environment::CollisionOperationsGenerator::OFTEN];
  for(int i = 0; i < often_collision_table_->rowCount(); i++)
  {
    QCheckBox* box = dynamic_cast<QCheckBox*> (often_collision_table_->cellWidget(i, 3));
    if(box != NULL)
    {
      bool alreadyDisabled = false;
      std::vector<std::pair<std::string, std::string> >::iterator pos;
      std::pair<std::string, std::string> linkPair;
      linkPair.first = often_collision_table_->item(i, 0)->text().toStdString();
      linkPair.second = often_collision_table_->item(i, 1)->text().toStdString();
      for(std::vector<std::pair<std::string, std::string> >::iterator it = disableVector.begin(); it
          != disableVector.end(); it++)
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

void occasionallyCollisionTableChanged()
{
  std::vector<std::pair<std::string, std::string> >& disableVector =
      disable_map_[planning_environment::CollisionOperationsGenerator::OCCASIONALLY];
  for(int i = 0; i < occasionally_collision_table_->rowCount(); i++)
  {
    QCheckBox* box = dynamic_cast<QCheckBox*> (occasionally_collision_table_->cellWidget(i, 3));
    if(box != NULL)
    {
      bool alreadyDisabled = false;
      std::vector<std::pair<std::string, std::string> >::iterator pos;
      std::pair<std::string, std::string> linkPair;
      linkPair.first = occasionally_collision_table_->item(i, 0)->text().toStdString();
      linkPair.second = occasionally_collision_table_->item(i, 1)->text().toStdString();
      for(std::vector<std::pair<std::string, std::string> >::iterator it = disableVector.begin(); it
          != disableVector.end(); it++)
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

void generateOccasionallyInCollisionTable()
{
  lock_.lock();
  std::vector<planning_environment::CollisionOperationsGenerator::StringPair> occasionally_in_collision;
  std::vector<planning_environment::CollisionOperationsGenerator::StringPair> not_in_collision;
  std::vector<double> percentages;
  std::vector<planning_environment::CollisionOperationsGenerator::CollidingJointValues> in_collision_joint_values;

  ops_gen_->generateOccasionallyAndNeverInCollisionPairs(occasionally_in_collision, not_in_collision, percentages,
                                                         in_collision_joint_values);

  std::vector<std::pair<std::string, std::string> >& disableVector =
      disable_map_[planning_environment::CollisionOperationsGenerator::ALWAYS];


  for(size_t i = 0; i < occasionally_in_collision.size(); i++)
  {
    bool alreadyDisabled = false;
    std::vector<std::pair<std::string, std::string> >::iterator pos;
    for(std::vector<std::pair<std::string, std::string> >::iterator it = disableVector.begin(); it
        != disableVector.end(); it++)
    {
      if((*it) == occasionally_in_collision[i])
      {
        alreadyDisabled = true;
        pos = it;
        break;
      }
    }

    if(alreadyDisabled)
    {
      occasionally_in_collision.erase(pos);
    }
  }


  for(size_t i = 0; i < not_in_collision.size(); i++)
  {
    bool alreadyDisabled = false;
    std::vector<std::pair<std::string, std::string> >::iterator pos;
    for(std::vector<std::pair<std::string, std::string> >::iterator it = disableVector.begin(); it
        != disableVector.end(); it++)
    {
      if((*it) == not_in_collision[i])
      {
        alreadyDisabled = true;
        pos = it;
        break;
      }
    }

    if(alreadyDisabled)
    {
      not_in_collision.erase(pos);
    }
  }


  occasionally_collision_table_->clear();
  occasionally_collision_table_->setRowCount((int)(occasionally_in_collision.size() + not_in_collision.size()));
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

  if(occasionally_in_collision.size() + not_in_collision.size() == 0)
  {
    occasionally_collision_table_->setRowCount(1);
    QTableWidgetItem* noCollide = new QTableWidgetItem("No Collisions");
    occasionally_collision_table_->setItem(0, 0, noCollide);
  }

  ROS_INFO("%lu links occasionally in collision.", occasionally_in_collision.size());

  for(size_t i = 0; i < occasionally_in_collision.size(); i++)
  {
    QTableWidgetItem* linkA = new QTableWidgetItem(occasionally_in_collision[i].first.c_str());
    linkA->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

    QTableWidgetItem* linkB = new QTableWidgetItem(occasionally_in_collision[i].second.c_str());
    linkB->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

    std::stringstream percentageStream;
    percentageStream << percentages[i];
    QTableWidgetItem* percentage = new QTableWidgetItem(percentageStream.str().c_str());
    percentage->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

    QCheckBox* enableBox = new QCheckBox(occasionally_collision_table_);
    enableBox->setChecked(true);
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

    std::stringstream percentageStream;
    percentageStream << 0.0;
    QTableWidgetItem* percentage = new QTableWidgetItem(percentageStream.str().c_str());
    percentage->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

    QCheckBox* enableBox = new QCheckBox(occasionally_collision_table_);
    enableBox->setChecked(false);
    connect(enableBox, SIGNAL(toggled(bool)), this, SLOT(occasionallyCollisionTableChanged()));

    occasionally_collision_table_->setItem((int)(i+occasionally_in_collision.size()), 0, linkA);
    occasionally_collision_table_->setItem((int)(i+occasionally_in_collision.size()), 1, linkB);
    occasionally_collision_table_->setItem((int)(i+occasionally_in_collision.size()), 2, percentage);
    occasionally_collision_table_->setCellWidget((int)(i+occasionally_in_collision.size()), 3, enableBox);
  }
  occasionallyCollisionTableChanged();

  lock_.unlock();
}

void generateOftenInCollisionTable()
{
  lock_.lock();
  std::vector<planning_environment::CollisionOperationsGenerator::StringPair> often_in_collision;
  std::vector<double> percentages;
  std::vector<planning_environment::CollisionOperationsGenerator::CollidingJointValues> in_collision_joint_values;

  ops_gen_->generateOftenInCollisionPairs(often_in_collision, percentages, in_collision_joint_values);

  often_collision_table_->clear();
  often_collision_table_->setRowCount((int)often_in_collision.size());
  often_collision_table_->setColumnCount(4);

  often_collision_table_->setColumnWidth(0, 300);
  often_collision_table_->setColumnWidth(1, 300);
  often_collision_table_->setColumnWidth(2, 300);
  often_collision_table_->setColumnWidth(3, 300);

  QStringList titleList;
  titleList.append("Link A");
  titleList.append("Link B");
  titleList.append("% Colliding");
  titleList.append("Enable?");

  often_collision_table_->setHorizontalHeaderLabels(titleList);

  if(often_in_collision.size() == 0)
  {
    often_collision_table_->setRowCount(1);
    QTableWidgetItem* noCollide = new QTableWidgetItem("No Collisions");
    often_collision_table_->setItem(0, 0, noCollide);
  }

  ROS_INFO("%lu links often in collision.", often_in_collision.size());

  for(size_t i = 0; i < often_in_collision.size(); i++)
  {
    QTableWidgetItem* linkA = new QTableWidgetItem(often_in_collision[i].first.c_str());
    linkA->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

    QTableWidgetItem* linkB = new QTableWidgetItem(often_in_collision[i].second.c_str());
    linkB->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

    std::stringstream percentageStream;
    percentageStream << percentages[i];
    QTableWidgetItem* percentage = new QTableWidgetItem(percentageStream.str().c_str());
    percentage->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

    QCheckBox* enableBox = new QCheckBox(often_collision_table_);
    enableBox->setChecked(true);
    connect(enableBox, SIGNAL(toggled(bool)), this, SLOT(oftenCollisionTableChanged()));

    often_collision_table_->setItem((int)i, 0, linkA);
    often_collision_table_->setItem((int)i, 1, linkB);
    often_collision_table_->setItem((int)i, 2, percentage);
    often_collision_table_->setCellWidget((int)i, 3, enableBox);
  }
  oftenCollisionTableChanged();

  lock_.unlock();
}

void generateAlwaysInCollisionTable()
 {
   std::vector<planning_environment::CollisionOperationsGenerator::StringPair> always_in_collision;
   std::vector<planning_environment::CollisionOperationsGenerator::CollidingJointValues> in_collision_joint_values;

   ops_gen_->generateAlwaysInCollisionPairs(always_in_collision, in_collision_joint_values);

   lock_.lock();
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
   disable_map_[planning_environment::CollisionOperationsGenerator::ALWAYS] = always_in_collision;
 }

void writeFiles()
{
  outputJointLimitsYAML();
  outputOMPLGroupYAML();
  outputPlanningDescriptionYAML();
  outputOMPLLaunchFile();
  outputKinematicsLaunchFiles();
  outputTrajectoryFilterLaunch();
  outputPlanningEnvironmentLaunch();
  outputPlanningComponentVisualizerLaunchFile();
}

protected:


  int nextId() const
  {
    switch(currentId())
    {
      case SetupGroupsPage:
        if(group_selection_done_box_->isChecked())
        {
          return SelectDOFPage;
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

  void setupQtPages()
  {
    initSetupGroupsPage();
    initKinematicChainsPage();
    initJointCollectionsPage();
    initSelectDofPage();
    initAlwaysInCollisionPage();
    initOftenInCollisionPage();
    initOccasionallyInCollisionPage();
    initOutputFilesPage();

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



  }

  void initSetupGroupsPage()
  {
    setup_groups_page_ = new QWizardPage(this);
    setup_groups_page_->setTitle("Planning Group Setup");

    QGridLayout* layout = new QGridLayout(setup_groups_page_);

    QLabel* setupGroupsLabel = new QLabel(setup_groups_page_);
    setupGroupsLabel->setText("Select planning groups for your robot based on kinematic chains, or joint collections."
        " When you are finished, please check the checkbox and you can move on by pressing Next. Otherwise, simply press Next"
        " to create a planning group.");
    layout->addWidget(setupGroupsLabel, 0, 0 , 1, 2);

    QGroupBox* selectGroupsBox = new QGroupBox(setup_groups_page_);
    selectGroupsBox->setTitle("Current Groups");
    current_group_table_= new QTableWidget(selectGroupsBox);
    QVBoxLayout* groupBoxLayout = new QVBoxLayout(selectGroupsBox);
    groupBoxLayout->addWidget(current_group_table_);
    selectGroupsBox->setLayout(groupBoxLayout);
    layout->addWidget(selectGroupsBox, 1, 0, 1, 1);
    QPushButton* deleteButton = new QPushButton(selectGroupsBox);
    deleteButton->setText("Delete");
    groupBoxLayout->addWidget(deleteButton);

    connect(deleteButton, SIGNAL(clicked()), this, SLOT(deleteGroupButtonClicked()));

    QGroupBox* modeBox = new QGroupBox(setup_groups_page_);
    modeBox->setTitle("Select Mode");
    QVBoxLayout* modeBoxLayout = new QVBoxLayout(modeBox);
    group_selection_mode_box_ = new QComboBox(modeBox);
    group_selection_done_box_ = new QCheckBox(modeBox);
    group_selection_done_box_->setText("Done Selecting Groups");
    QStringList texts;
    texts.append("Kinematic Chains");
    texts.append("Joint Collections");
    group_selection_mode_box_->addItems(texts);

    modeBoxLayout->addWidget(group_selection_mode_box_);
    modeBoxLayout->addWidget(group_selection_done_box_);
    modeBox->setLayout(modeBoxLayout);
    layout->addWidget(modeBox, 1, 2, 1, 1);

    addPage(setup_groups_page_);
    setPage(SetupGroupsPage, setup_groups_page_);
    setup_groups_page_->setLayout(layout);
  }

  void initKinematicChainsPage()
  {
    kinematic_chains_page_ = new QWizardPage(this);
    kinematic_chains_page_->setTitle("Select Kinematic Chain");

    QGridLayout* layout = new QGridLayout(kinematic_chains_page_);
    QLabel* kinematicChainsLabel = new QLabel(kinematic_chains_page_);
    kinematicChainsLabel->setText("Select a planning group based on a kinematic chain."
        " Select a base link (the first link in the chain) and a tip link."
        " They must be connected by a direct line of joints.");
    layout->addWidget(kinematicChainsLabel, 0, 0, 1, 2);

    QGroupBox* treeBox = new QGroupBox(kinematic_chains_page_);
    treeBox->setTitle("Links");
    layout->addWidget(treeBox, 1, 0, 1, 1);

    QVBoxLayout* treeLayout = new QVBoxLayout(treeBox);
    link_tree_ = new QTreeWidget(treeBox);
    treeLayout->addWidget(link_tree_);
    QPushButton* baseLinkButton = new QPushButton(treeBox);
    baseLinkButton->setText("Select Base Link");
    treeLayout->addWidget(baseLinkButton);
    QPushButton* tipLinkButton = new QPushButton(treeBox);
    tipLinkButton->setText("Select Tip Link");
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

    connect(acceptButton, SIGNAL(clicked()), this, SLOT(acceptChainClicked()));

    layout->addWidget(chainBox, 1, 1, 1, 1);

    addPage(kinematic_chains_page_);
    setPage(KinematicChainsPage, kinematic_chains_page_);
    kinematic_chains_page_->setLayout(layout);
  }


  void createJointCollectionTables()
  {
    const std::vector<planning_models::KinematicModel::JointModel*>& jmv = kmodel_->getJointModels();

    joint_table_->setRowCount((int)jmv.size());
    joint_table_->setColumnCount(1);
    joint_table_->setColumnWidth(0, 300);
    selected_joint_table_->setColumnCount(1);
    selected_joint_table_->setColumnWidth(0,300);
    QStringList headerLabels;
    headerLabels.append("Joint");
    joint_table_->setHorizontalHeaderLabels(headerLabels);
    selected_joint_table_->setHorizontalHeaderLabels(headerLabels);
    for(size_t i = 1; i < jmv.size(); i++)
    {
      QTableWidgetItem* item = new QTableWidgetItem(jmv[i]->getName().c_str());
      item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
      joint_table_->setItem((int)i - 1, 0, item);
    }
  }


  void initJointCollectionsPage()
  {
    joint_collections_page_ = new QWizardPage(this);
    joint_collections_page_->setTitle("Select Joint Collections");

    QGridLayout* layout = new QGridLayout(joint_collections_page_);

    QLabel* jointCollectionLabel = new QLabel(joint_collections_page_);
    jointCollectionLabel->setText("Select a group of joints to form a planning group.");
    layout->addWidget(jointCollectionLabel, 0, 0, 1, 2);

    QGroupBox* jointBox = new QGroupBox(joint_collections_page_);
    jointBox->setTitle("Joints");

    QVBoxLayout* jointLayout = new QVBoxLayout(jointBox);
    joint_table_ = new QTableWidget(jointBox);
    jointLayout->addWidget(joint_table_);
    jointBox->setLayout(jointLayout);
    layout->addWidget(jointBox, 1, 0, 1, 1);

    QPushButton* selectButton = new QPushButton(jointBox);
    selectButton->setText("Select");
    jointLayout->addWidget(selectButton);

    connect(selectButton, SIGNAL(clicked()), this, SLOT(selectJointButtonClicked()));

    QGroupBox* selectedBox = new QGroupBox(joint_collections_page_);
    selectedBox->setTitle("Selected Joints");

    QVBoxLayout* selectedLayout = new QVBoxLayout(selectedBox);
    selected_joint_table_ = new QTableWidget(selectedBox);
    selectedLayout->addWidget(selected_joint_table_);


    QPushButton* deselectButton = new QPushButton(selectedBox);
    deselectButton->setText("Deselect");
    selectedLayout->addWidget(deselectButton);

    connect(deselectButton, SIGNAL(clicked()), this, SLOT(deselectJointButtonClicked()));

    QLabel* jointGroupLabel = new QLabel(selectedBox);
    jointGroupLabel->setText("\nJoint Group Name: ");
    joint_group_name_field_ = new QLineEdit(selectedBox);
    selectedLayout->addWidget(jointGroupLabel);
    selectedLayout->addWidget(joint_group_name_field_);
    QPushButton* acceptButton = new QPushButton(selectedBox);
    acceptButton->setText("Accept Joint Group");
    selectedLayout->addWidget(acceptButton);
    selectedBox->setLayout(selectedLayout);
    layout->addWidget(selectedBox, 1, 1, 1, 1);

    connect(acceptButton, SIGNAL(clicked()), this, SLOT(acceptGroupClicked()));

    addPage(joint_collections_page_);
    setPage(JointCollectionsPage, joint_collections_page_);
    joint_collections_page_->setLayout(layout);

    createJointCollectionTables();
  }

  void initSelectDofPage()
  {
    select_dof_page_ = new QWizardPage(this);
    select_dof_page_->setTitle("DOF Sampling");
    QVBoxLayout* layout = new QVBoxLayout(select_dof_page_);
    QLabel* selectDofLabel = new QLabel(select_dof_page_);
    selectDofLabel->setText("Select degrees of freedom to sample for collisions. The wizard will run your robot through "
        "a set of subsamples of these joints and check each pair of links for collisions. Unselected joints will remain "
        "in their default positions during these tests.");
    layout->addWidget(selectDofLabel);

    QPushButton* applyButton = new QPushButton(select_dof_page_);
    applyButton->setText("Apply Changes");
    layout->addWidget(applyButton);

    connect(applyButton, SIGNAL(clicked()), SLOT(dofSelectionTableChanged()));

    dof_selection_table_ = new QTableWidget(select_dof_page_);
    layout->addWidget(dof_selection_table_);

    addPage(select_dof_page_);
    setPage(SelectDOFPage, select_dof_page_);
    select_dof_page_->setLayout(layout);

  }

  void createDofPageTable()
  {
      ode_collision_model_ = new collision_space::EnvironmentModelODE();

      const std::vector<planning_models::KinematicModel::LinkModel*>& coll_links =
          kmodel_->getLinkModelsWithCollisionGeometry();

      std::vector<std::string> coll_names;
      for(unsigned int i = 0; i < coll_links.size(); i++)
      {
        coll_names.push_back(coll_links[i]->getName());
      }
      collision_space::EnvironmentModel::AllowedCollisionMatrix default_collision_matrix(coll_names, false);
      std::map<std::string, double> default_link_padding_map;
      ode_collision_model_->setRobotModel(kmodel_, default_collision_matrix, default_link_padding_map, 0.0, 1.0);

      cm_ = new planning_environment::CollisionModels(urdf_, kmodel_, ode_collision_model_);
      ops_gen_ = new planning_environment::CollisionOperationsGenerator(cm_);

      const std::vector<planning_models::KinematicModel::JointModel*>& jmv = cm_->getKinematicModel()->getJointModels();
      std::vector<bool> consider_dof;

      int numDofs = 0;
      //assuming that 0th is world joint, which we don't want to include
      for(unsigned int i = 1; i < jmv.size(); i++)
      {
        const std::map<std::string, std::pair<double, double> >& joint_bounds = jmv[i]->getAllVariableBounds();
        for(std::map<std::string, std::pair<double, double> >::const_iterator it = joint_bounds.begin(); it
            != joint_bounds.end(); it++)
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
        const std::map<std::string, std::pair<double, double> >& joint_bounds = jmv[i]->getAllVariableBounds();

        for(std::map<std::string, std::pair<double, double> >::const_iterator it = joint_bounds.begin(); it
            != joint_bounds.end(); it++)
        {
          std::stringstream lowerStream;
          std::stringstream upperStream;
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

          dof_selection_table_->setItem(ind - 1, 0, nameItem);
          dof_selection_table_->setItem(ind - 1, 1, lowerItem);
          dof_selection_table_->setItem(ind - 1, 2, upperItem);
          dof_selection_table_->setCellWidget(ind - 1, 3, check);
          ind++;
        }
      }


  }

  void initAlwaysInCollisionPage()
  {
    always_in_collision_page_ = new QWizardPage(this);
    always_in_collision_page_->setTitle("Links Always In Collision");

    QVBoxLayout* layout = new QVBoxLayout(always_in_collision_page_);
    QLabel* alwaysLabel = new QLabel(always_in_collision_page_);
    alwaysLabel->setText("The following links are always in collision over the sample space."
        " By default, collisions will be disabled for them. Collisions are visualized as yellow spheres in rviz.");
    layout->addWidget(alwaysLabel);
    QPushButton* generateButton = new QPushButton(always_in_collision_page_);
    generateButton->setText("Generate List (May take a minute)");
    layout->addWidget(generateButton);
    connect(generateButton, SIGNAL(clicked()), this, SLOT(generateAlwaysInCollisionTable()));

    always_collision_table_ = new QTableWidget(always_in_collision_page_);
    layout->addWidget(always_collision_table_);

    addPage(always_in_collision_page_);
    setPage(AlwaysInCollisionPage, always_in_collision_page_);
    always_in_collision_page_->setLayout(layout);
  }

  void initOftenInCollisionPage()
  {
    often_in_collision_page_ = new QWizardPage(this);
    often_in_collision_page_->setTitle("Links Often In Collision");


    QVBoxLayout* layout = new QVBoxLayout(often_in_collision_page_);
    QLabel* oftenLabel = new QLabel(often_in_collision_page_);
    oftenLabel->setText("The following links are often in collision over the sample space."
        " By default, collisions will be disabled for them. Collisions are visualized in rviz.");
    layout->addWidget(oftenLabel);

    QPushButton* generateButton = new QPushButton(often_in_collision_page_);
    generateButton->setText("Generate List (May take a minute)");
    layout->addWidget(generateButton);

    often_collision_table_ = new QTableWidget(often_in_collision_page_);
    layout->addWidget(often_collision_table_);


    connect(generateButton, SIGNAL(clicked()), this, SLOT(generateOftenInCollisionTable()));

    addPage(often_in_collision_page_);
    setPage(OftenInCollisionPage, often_in_collision_page_);
    often_in_collision_page_->setLayout(layout);
  }

  void initOccasionallyInCollisionPage()
  {
    occasionally_in_collision_page_ = new QWizardPage(this);
    occasionally_in_collision_page_->setTitle("Links Occasionally In Collision");

    QVBoxLayout* layout = new QVBoxLayout(occasionally_in_collision_page_);
    QLabel* occasionallyLabel = new QLabel(occasionally_in_collision_page_);
    occasionallyLabel->setText("The following links are occasionally (or never) in collision over the sample space."
        " By default, collisions will be disabled for those which never collide,"
        " and enabled for those which only occasionally collide. Collisions are visualized in rviz.");
    layout->addWidget(occasionallyLabel);

    QPushButton* generateButton = new QPushButton(occasionally_in_collision_page_);
    generateButton->setText("Generate List (May take a minute)");
    layout->addWidget(generateButton);
    connect(generateButton, SIGNAL(clicked()), this, SLOT(generateOccasionallyInCollisionTable()));

    occasionally_collision_table_ = new QTableWidget(occasionally_in_collision_page_);
    layout->addWidget(occasionally_collision_table_);

    addPage(occasionally_in_collision_page_);
    setPage(OccasionallyInCollisionPage, occasionally_in_collision_page_);
    occasionally_in_collision_page_->setLayout(layout);
  }

  void initOutputFilesPage()
  {
    output_files_page_ = new QWizardPage(this);
    output_files_page_->setTitle("Output Files");
    QVBoxLayout* layout = new QVBoxLayout(output_files_page_);
    QLabel* outputLabel = new QLabel(output_files_page_);
    outputLabel->setText("Done! The wizard will auto-generate a stack called <your_robot_name>_arm_navigation "
        "in your ~/.ros folder when you click the button below.");
    layout->addWidget(outputLabel);
    QPushButton* generateButton = new QPushButton(output_files_page_);
    generateButton->setText("Generate Config Files...");

    layout->addWidget(generateButton);

    connect(generateButton, SIGNAL(clicked()), this, SLOT(writeFiles()));

    addPage(output_files_page_);
    setPage(OutputFilesPage, output_files_page_);
    output_files_page_->setLayout(layout);
  }

  void createLinkTree()
  {
    const planning_models::KinematicModel::JointModel* rootJoint = kmodel_->getRoot();
    addLinktoTreeRecursive(rootJoint->getChildLinkModel(), NULL);
    link_tree_->expandAll();
  }

  void addLinktoTreeRecursive(const planning_models::KinematicModel::LinkModel* link,
                              const planning_models::KinematicModel::LinkModel* parent)
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

  bool addLinkChildRecursive(QTreeWidgetItem* parent,const planning_models::KinematicModel::LinkModel* link,
                             const std::string& parentName)
  {
    if(parent->text(0).toStdString() == parentName)
    {
      QTreeWidgetItem* toAdd = new QTreeWidgetItem(parent);
      toAdd->setText(0, link->getName().c_str());
      parent->addChild(toAdd);
      ROS_INFO("Added %s to parent %s", link->getName().c_str(), parentName.c_str());
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

  bool inited_;

  ros::NodeHandle nh_;
  boost::shared_ptr<urdf::Model> urdf_;
  planning_models::KinematicModel* kmodel_;
  planning_environment::CollisionModels* cm_;
  planning_environment::CollisionOperationsGenerator* ops_gen_;
  planning_models::KinematicState* robot_state_;
  collision_space::EnvironmentModel* ode_collision_model_;
  visualization_msgs::MarkerArray collision_markers_;
  planning_models::KinematicModel::MultiDofConfig world_joint_config_;
  std::map<planning_environment::CollisionOperationsGenerator::DisableType, std::vector<planning_environment::CollisionOperationsGenerator::StringPair> > disable_map_;

  std::string current_show_group_;

  tf::TransformBroadcaster transform_broadcaster_;
  ros::Publisher vis_marker_publisher_;
  ros::Publisher vis_marker_array_publisher_;

  boost::recursive_mutex lock_;

  YAML::Emitter emitter_;

  std::string dir_name_;
  std::string yaml_outfile_name_, full_yaml_outfile_name_;
  std::string launch_outfile_name_, full_launch_outfile_name_;

  std::string urdf_package_, urdf_path_;

  QWizardPage* setup_groups_page_;
  QWizardPage* kinematic_chains_page_;
  QWizardPage* joint_collections_page_;
  QWizardPage* select_dof_page_;
  QWizardPage* always_in_collision_page_;
  QWizardPage* often_in_collision_page_;
  QWizardPage* occasionally_in_collision_page_;
  QWizardPage* output_files_page_;

  QTableWidget* current_group_table_;
  QTreeWidget* link_tree_;
  QTableWidget* joint_table_;
  QTableWidget* selected_joint_table_;
  QComboBox* group_selection_mode_box_;
  QLineEdit* chain_name_field_;
  QLineEdit* base_link_field_;
  QLineEdit* tip_link_field_;
  QLineEdit* joint_group_name_field_;
  QTableWidget* dof_selection_table_;
  QTableWidget* always_collision_table_;
  QTableWidget* often_collision_table_;
  QTableWidget* occasionally_collision_table_;
  QCheckBox* group_selection_done_box_;

  QDialog* ok_dialog_;
  QDialog* not_ok_dialog_;


};
#endif
