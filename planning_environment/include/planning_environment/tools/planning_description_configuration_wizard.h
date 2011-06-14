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
#include <qt4/QtGui/qfiledialog.h>
#include <ncurses.h>
#include <tinyxml/tinyxml.h>

static const std::string VIS_TOPIC_NAME = "planning_description_configuration_wizard";
static const unsigned int CONTROL_SPEED = 10;
static const double DEFAULT_ACCELERATION = 1.0;

class PlanningDescriptionConfigurationWizard: public QWizard
{
  Q_OBJECT

  public:
    enum WizardPage
    {
      StartPage, SetupGroupsPage, KinematicChainsPage, JointCollectionsPage, SelectDOFPage, AlwaysInCollisionPage,
      OftenInCollisionPage, OccasionallyInCollisionPage, OutputFilesPage, DefaultInCollisionPage
    };

    PlanningDescriptionConfigurationWizard(const std::string& urdf_package, const std::string& urdf_path,
                                           QWidget* parent = NULL);
    ~PlanningDescriptionConfigurationWizard();
    void deleteKinematicStates();
    bool setupWithWorldFixedFrame(const std::string& world_fixed_frame, const std::string& joint_type);
    void emitWorldJointYAML();
    void outputConfigAndLaunchRviz();
    void setupGroups();
    void setupGroupKinematicChain(const std::string& new_group_name);
    bool addGroup(std::string new_group_name, std::string base, std::string tip);
    void popupNotOkayWarning();
    void popupOkayWarning();
    void popupFileFailure(const char* reason);
    void popupFileSuccess();
    void updateGroupTable();
    void setupGroupJointCollection(const std::string& new_group_name);
    void emitGroupYAML();
    void outputJointLimitsYAML();
    void setJointsForCollisionSampling();
    visualization_msgs::Marker
        transformEnvironmentModelContactInfoMarker(const collision_space::EnvironmentModel::Contact& c);
    void considerAlwaysAndDefaultInCollisionMarkers();
    void considerOftenInCollisionPairs();
    void considerOccasionallyInCollisionPairs();
    void
        considerInCollisionPairs(
                                 std::vector<planning_environment::CollisionOperationsGenerator::StringPair>& in_collision_pairs,
                                 std::vector<double>& percentages,
                                 std::vector<planning_environment::CollisionOperationsGenerator::CollidingJointValues>& in_collision_joint_values,
                                 const std_msgs::ColorRGBA& color);
    void outputPlanningDescriptionYAML();
    void outputOMPLGroupYAML();
    void outputOMPLLaunchFile();
    void outputTrajectoryFilterLaunch();
    void outputPlanningEnvironmentLaunch();
    void outputKinematicsLaunchFiles();
    void outputPlanningComponentVisualizerLaunchFile();
    void updateCollisionsInCurrentState();
    void sendMarkers();
    void sendTransforms();
    bool isInited() const;
    planning_environment::CollisionOperationsGenerator* getOperationsGenerator();
    std::string getRobotName();
    void getRobotMeshResourceMarkersGivenState(const planning_models::KinematicState& state,
                                               visualization_msgs::MarkerArray& arr, const std_msgs::ColorRGBA& color,
                                               const std::string& name, const ros::Duration& lifetime,
                                               const std::vector<std::string>* names) const;
  public slots:
    std::vector<int> getSelectedRows(QTableWidget* table);
    void toggleTable(QTableWidget* table, int column = 3);
    void defaultTogglePushed();
    void oftenTogglePushed();
    void occasionallyTogglePushed();
    void dofTogglePushed();
    void selectJointButtonClicked();
    void deselectJointButtonClicked();
    void deleteGroupButtonClicked();
    void acceptChainClicked();
    void acceptGroupClicked();
    void baseLinkTreeClick();
    void tipLinkTreeClick();
    void validateDoneBox();
    void groupTableClicked();
    void defaultTableClicked();
    void oftenTableClicked();
    void occasionallyTableClicked();
    void dofSelectionTableChanged();
    void oftenCollisionTableChanged();
    void defaultCollisionTableChanged();
    void occasionallyCollisionTableChanged();
    void generateOccasionallyInCollisionTable();
    void generateOftenInCollisionTable();
    void generateAlwaysInCollisionTable();
    void generateDefaultInCollisionTable();
    void fileSelected(const QString& file);
    void writeFiles();
    
  protected:
    int nextId() const;
    void
        visualizeCollision(
                           std::vector<planning_environment::CollisionOperationsGenerator::CollidingJointValues>& jointValues,
                           std::vector<planning_environment::CollisionOperationsGenerator::StringPair>& pairs,
                           int& index, std_msgs::ColorRGBA& color);
    void setupQtPages();
    void initStartPage();
    void initSetupGroupsPage();
    void initKinematicChainsPage();
    void createJointCollectionTables();
    void initJointCollectionsPage();
    void initSelectDofPage();
    void createDofPageTable();
    void initAlwaysInCollisionPage();
    void initDefaultInCollisionPage();
    void initOftenInCollisionPage();
    void initOccasionallyInCollisionPage();
    void initOutputFilesPage();
    void createLinkTree();
    void addLinktoTreeRecursive(const planning_models::KinematicModel::LinkModel* link,
                                const planning_models::KinematicModel::LinkModel* parent);
    bool addLinkChildRecursive(QTreeWidgetItem* parent, const planning_models::KinematicModel::LinkModel* link,
                               const std::string& parentName);

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
    std::map<planning_environment::CollisionOperationsGenerator::DisableType, std::vector<
        planning_environment::CollisionOperationsGenerator::StringPair> > disable_map_;

    std::string current_show_group_;

    tf::TransformBroadcaster transform_broadcaster_;
    ros::Publisher vis_marker_publisher_;
    ros::Publisher vis_marker_array_publisher_;

    boost::recursive_mutex lock_;

    YAML::Emitter emitter_;

    std::string package_directory_;
    std::string dir_name_;
    std::string yaml_outfile_name_, full_yaml_outfile_name_;
    std::string launch_outfile_name_, full_launch_outfile_name_;

    std::string urdf_package_, urdf_path_;

    QWizardPage* start_page_;
    QWizardPage* setup_groups_page_;
    QWizardPage* kinematic_chains_page_;
    QWizardPage* joint_collections_page_;
    QWizardPage* select_dof_page_;
    QWizardPage* always_in_collision_page_;
    QWizardPage* default_collision_page_;
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
    QTableWidget* default_collision_table_;
    QTableWidget* often_collision_table_;
    QTableWidget* occasionally_collision_table_;
    QCheckBox* group_selection_done_box_;

    QDialog* need_groups_dialog_;
    QDialog* ok_dialog_;
    QDialog* not_ok_dialog_;
    QDialog* file_failure_dialog_;
    QLabel* file_failure_reason_;
    QDialog* file_success_dialog_;
    QLineEdit* package_path_field_;
    QFileDialog* file_selector_;

    std::vector<planning_environment::CollisionOperationsGenerator::CollidingJointValues>
        default_in_collision_joint_values_;
    std::vector<planning_environment::CollisionOperationsGenerator::StringPair> default_collision_pairs_;
    std::vector<planning_environment::CollisionOperationsGenerator::CollidingJointValues>
        often_in_collision_joint_values_;
    std::vector<planning_environment::CollisionOperationsGenerator::StringPair> often_collision_pairs_;
    std::vector<planning_environment::CollisionOperationsGenerator::CollidingJointValues>
        occasionally_collision_joint_values_;
    std::vector<planning_environment::CollisionOperationsGenerator::StringPair> occasionally_collision_pairs_;

};
#endif
