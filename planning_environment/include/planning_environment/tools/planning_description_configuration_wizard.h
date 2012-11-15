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

#ifndef Q_MOC_RUN
#include <planning_environment/models/collision_models.h>
#include <planning_environment/models/model_utils.h>
#include <tf/transform_broadcaster.h>
#include <collision_space/environmentODE.h>
#include <rosgraph_msgs/Clock.h>
#include <planning_environment/util/collision_operations_generator.h>
#endif
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
#include <qt4/QtGui/qfiledialog.h>
#include <tinyxml.h>
#include <qt4/Qt/qthread.h>
#include <qt4/QtGui/qprogressbar.h>

static const std::string VIS_TOPIC_NAME = "planning_description_configuration_wizard";
static const unsigned int CONTROL_SPEED = 10;
static const double DEFAULT_ACCELERATION = 1.0;

class OutputWizardPage;
class KinematicChainWizardPage;
class JointCollectionWizardPage;
class SetupGroupsWizardPage;
class CollisionsWizardPage;

inline std::vector<int> getSelectedRows(QTableWidget* table) {
  QList<QTableWidgetItem*> selected = table->selectedItems();
  
  std::vector<int> rows;
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


class PlanningDescriptionConfigurationWizard: public QWizard
{
  Q_OBJECT

  public:
  enum WizardMode
    {
      Easy, Advanced
    };

  enum WizardPage
    {
      StartPage,
      SetupGroupsPage,
      KinematicChainsPage,
      JointCollectionsPage,
      SelectDOFPage,
      AdjacentLinkPage,
      AlwaysInCollisionPage,
      OftenInCollisionPage,
      OccasionallyInCollisionPage,
      OutputFilesPage,
      DefaultInCollisionPage
    };

  enum GroupAddStatus {
    GroupAddSuccess,
    GroupAddCancel,
    GroupAddFailed
  };

  PlanningDescriptionConfigurationWizard(const std::string& urdf_package, const std::string& urdf_path,
                                         QWidget* parent = NULL);
  ~PlanningDescriptionConfigurationWizard();
  void deleteKinematicStates();
  bool setupWithWorldFixedFrame(const std::string& world_fixed_frame, const std::string& joint_type);
  void emitWorldJointYAML();
  void outputConfigAndLaunchRviz();
  GroupAddStatus addGroup(const planning_models::KinematicModel::GroupConfig& group_config);
  void removeGroup(const std::string& name);
  inline void popupWaitWarning() { popupGenericWarning("Please wait..."); }
  void popupGenericWarning(const char* text);
  void popupFileFailure(const char* reason);
  void emitGroupYAML();
  void outputJointLimitsYAML();
  visualization_msgs::Marker
  transformEnvironmentModelContactInfoMarker(const collision_space::EnvironmentModel::Contact& c);
  void outputPlanningDescriptionYAML();
  void outputOMPLGroupYAML();
  void outputOMPLLaunchFile();
  void outputTrajectoryFilterLaunch();
  void outputPlanningEnvironmentLaunch();
  void outputKinematicsLaunchFiles();
  void outputMoveGroupLaunchFiles();
  void outputPlanningComponentVisualizerLaunchFile();
  void outputArmNavigationLaunchFile();
  void updateCollisionsInCurrentState();
  void sendMarkers();
  void sendTransforms();
  bool isInited() const;
  planning_environment::CollisionOperationsGenerator* getOperationsGenerator();
  std::string getRobotName();
  const planning_models::KinematicModel* getKinematicModel() {
    return kmodel_;
  }

  void setCurrentShowGroup(const std::string& s) {
    lock_.lock();
    current_show_group_ = s;
    lock_.unlock();
  }

  void setCurrentShowLink(const std::string& s) {
    lock_.lock();
    current_show_link_ = s;
    lock_.unlock();
  }

  WizardMode getWizardMode() const {
    return wizard_mode_;
  }
  
  void setDisableMap(const planning_environment::CollisionOperationsGenerator::DisableType disable_type,
                     const std::vector<planning_environment::CollisionOperationsGenerator::StringPair>& pairs)
  {
    disable_map_[disable_type] = pairs;
  }

  void
  visualizeCollision(std::vector<planning_environment::CollisionOperationsGenerator::CollidingJointValues>& jointValues,
                     std::vector<planning_environment::CollisionOperationsGenerator::StringPair>& pairs,
                     int& index, std_msgs::ColorRGBA& color);


signals:
  void changeProgress(int progress);
  void changeLabel(const char* name);                                  
                                    
public slots:
  void easyButtonToggled(bool checkState);
  void hardButtonToggled(bool checkState);

  void labelChanged(const char* name);

  void verySafeButtonToggled(bool checkState);
  void safeButtonToggled(bool checkState);
  void normalButtonToggled(bool checkState);
  void fastButtonToggled(bool checkState);
  void veryFastButtonToggled(bool checkState);

  void dofSelectionTableChanged();
  void dofTogglePushed();
  void writeFiles();
  void autoConfigure();
  void update();

protected:
  
  virtual int nextId() const;
  void setupQtPages();
  void initStartPage();
  void initSetupGroupsPage();

  void initSelectDofPage();
  void createDofPageTable();
  void initOutputFilesPage();

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
  std::map<planning_environment::CollisionOperationsGenerator::DisableType, 
           std::vector<planning_environment::CollisionOperationsGenerator::StringPair> > disable_map_;

  std::string current_show_group_;
  std::string current_show_link_;

  tf::TransformBroadcaster transform_broadcaster_;
  ros::Publisher vis_marker_publisher_;
  ros::Publisher vis_marker_array_publisher_;

  boost::recursive_mutex lock_;

  YAML::Emitter* emitter_;

  std::string package_directory_;
  std::string dir_name_;
  std::string yaml_outfile_name_, full_yaml_outfile_name_;
  std::string launch_outfile_name_, full_launch_outfile_name_;

  std::string urdf_package_, urdf_path_;

  QWizardPage* start_page_;
  SetupGroupsWizardPage* setup_groups_page_;
  KinematicChainWizardPage* kinematic_chain_page_;
  JointCollectionWizardPage* joint_collections_page_;
  QWizardPage* select_dof_page_;
  CollisionsWizardPage* adjacent_link_page_;
  CollisionsWizardPage* always_in_collision_page_;
  CollisionsWizardPage* default_in_collision_page_;
  CollisionsWizardPage* often_in_collision_page_;
  CollisionsWizardPage* occasionally_in_collision_page_;

  OutputWizardPage* output_wizard_page_;

  QTableWidget* dof_selection_table_;
  QCheckBox* group_selection_done_box_;

  QDialog* file_failure_dialog_;
  QLabel* file_failure_reason_;
  QLineEdit* package_path_field_;
  QDialog* generic_dialog_;
  QLabel* generic_dialog_label_;

  QDialog* confirm_group_replace_dialog_;
  QLabel* confirm_group_text_;

  int progress_;

  WizardMode wizard_mode_;

};

class AutoConfigureThread : public QThread
{
public:
  PlanningDescriptionConfigurationWizard* wizard_;
  AutoConfigureThread(PlanningDescriptionConfigurationWizard* wizard) : QThread(wizard), wizard_(wizard)
  {
    connect(wizard_, SIGNAL(changeProgress(int)), wizard_, SLOT(update()));
    connect(wizard_, SIGNAL(changeLabel(const char*)), wizard_, SLOT(labelChanged(const char*)));
  }

  void run()
  {
    wizard_->autoConfigure();
  }
};

class SetupGroupsWizardPage : public QWizardPage
{
  
  Q_OBJECT
  
  public:
  
  SetupGroupsWizardPage(PlanningDescriptionConfigurationWizard *parent);

  void updateGroupTable();

  virtual int nextId() const;

public slots:

  void deleteGroupButtonClicked();
  void groupTableClicked();
  void addKinematicChainGroup();
  void addJointCollectionGroup();


protected:

  virtual bool validatePage() {
    parent_->setCurrentShowGroup("");
    return true;
  }

  PlanningDescriptionConfigurationWizard *parent_;

  int next_from_groups_id_;

  QLineEdit* first_group_field_;

  QTableWidget* current_group_table_;

};

class KinematicChainWizardPage : public QWizardPage
{
  Q_OBJECT
  
  public:
  
  KinematicChainWizardPage(PlanningDescriptionConfigurationWizard *parent);

  std::string getChainNameField() {
    return chain_name_field_->text().toStdString();
  }

  std::string getBaseLinkField() {
    return base_link_field_->text().toStdString();
  }

  std::string getTipLinkField() {
    return tip_link_field_->text().toStdString();
  }

  bool getReturnToGroups() const {
    return return_to_groups_;
  }

public slots:

  void baseLinkTreeClick();
  void tipLinkTreeClick();
  void showTreeLink();

  void resetPage() {
    good_group_dialog_->done(0);
  }

protected:

  virtual bool validatePage();

  void createLinkTree(const planning_models::KinematicModel* kmodel);
  void addLinktoTreeRecursive(const planning_models::KinematicModel::LinkModel* link,
                              const planning_models::KinematicModel::LinkModel* parent);

  bool addLinkChildRecursive(QTreeWidgetItem* parent, const planning_models::KinematicModel::LinkModel* link,
                             const std::string& parentName);

  PlanningDescriptionConfigurationWizard *parent_;

  bool return_to_groups_;

  QDialog* good_group_dialog_;  
  QDialog* not_ok_dialog_;

  QTreeWidget* link_tree_;

  QLineEdit* chain_name_field_;
  QLineEdit* base_link_field_;
  QLineEdit* tip_link_field_;

};

class JointCollectionWizardPage : public QWizardPage
{
  Q_OBJECT
  
  public:
  
  JointCollectionWizardPage(PlanningDescriptionConfigurationWizard *parent);

  bool getReturnToGroups() const {
    return return_to_groups_;
  }

public slots:

  void resetPage() {
    good_group_dialog_->done(0);
  }

  void selectJointButtonClicked();
  void deselectJointButtonClicked();

protected:

  void createJointCollectionTables();

  virtual bool validatePage();

  PlanningDescriptionConfigurationWizard *parent_;

  bool return_to_groups_;

  QLineEdit* first_joint_field_;

  QDialog* good_group_dialog_;  
  QDialog* not_ok_dialog_;

  QTableWidget* joint_table_;
  QTableWidget* selected_joint_table_;
  QLineEdit* joint_group_name_field_;

};

class CollisionsWizardPage : public QWizardPage
{
  Q_OBJECT

  public:
  
  CollisionsWizardPage(PlanningDescriptionConfigurationWizard *parent,
                       planning_environment::CollisionOperationsGenerator::DisableType);
                                                                                       
  
public slots:

  void tableClicked();
  void tableChanged();
  void generateCollisionTable();
  void toggleTable();

protected:

  virtual bool validatePage();
  PlanningDescriptionConfigurationWizard *parent_;
  QTableWidget* collision_table_;
  planning_environment::CollisionOperationsGenerator::DisableType disable_type_;

  std::vector<planning_environment::CollisionOperationsGenerator::CollidingJointValues>
  in_collision_joint_values_;
  std::vector<planning_environment::CollisionOperationsGenerator::StringPair> collision_pairs_;
  std::vector<planning_environment::CollisionOperationsGenerator::StringPair> enable_pairs_;
  std::vector<planning_environment::CollisionOperationsGenerator::StringPair> extra_disable_pairs_;
  std::vector<planning_environment::CollisionOperationsGenerator::StringPair> disable_pairs_;

  bool allow_enable_;
  bool show_percentages_;
  bool is_clickable_;
  bool coll_default_enabled_;
};
  
class OutputWizardPage : public QWizardPage 
{
  Q_OBJECT
  
  public:
  OutputWizardPage(PlanningDescriptionConfigurationWizard *parent = 0);
  
  std::string getPackagePathField() const {
    return package_path_field_->text().toStdString();
  }
  
  void setSuccessfulGeneration() {
    successful_generation_ = true;
    successful_creation_dialog_->show();
  }

  void updateProgressBar(unsigned int progress) {
    progress_bar_->setValue(progress);
  }

  void setProgressLabel(const char* label) {
    progress_label_->setText(label);
  }

public slots:
  
  void fileSelected(const QString& file)
  {
    package_path_field_->setText(file);
    ROS_INFO_STREAM("Setting file to " << package_path_field_->text().toStdString());
  }

protected:

  virtual bool validatePage() {
    if(!successful_generation_) {
      int val = really_exit_dialog_->exec();
      if(val == QDialog::Accepted) {
        return true;
      } else {
        return false;
      }
    }
    return true;
  }

private:

  QProgressBar* progress_bar_;
  QLabel* progress_label_;
  bool successful_generation_;

  QLineEdit* package_path_field_;
  QFileDialog* file_selector_;

  QDialog* really_exit_dialog_;
  QDialog* successful_creation_dialog_;
};

#endif
