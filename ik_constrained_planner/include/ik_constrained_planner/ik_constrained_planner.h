/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** \author Sachin Chitta, Ioan Sucan */


#include <ros/ros.h>
#include <ros/console.h>
#include <sstream>
#include <cstdlib>

#include <motion_planning_msgs/convert_messages.h>
#include <motion_planning_msgs/GetMotionPlan.h>
#include <planning_environment_msgs/ContactInformation.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

#include <planning_environment/monitors/planning_monitor.h>
#include <planning_models/kinematic_model.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <tf/tf.h>

#include <ik_constrained_planner/ik_constrained_goal.h>
#include <ik_constrained_planner/projection_evaluators.h>
#include <ik_constrained_planner/ik_state_validator.h>
//#include <ik_constrained_planner/ik_constrained_helpers.h>

#include <ompl/base/State.h>
#include <ompl/base/Planner.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/kinematic/SpaceInformationKinematic.h>

#include <ompl/kinematic/planners/sbl/SBL.h>
#include <ompl/kinematic/planners/kpiece/LBKPIECE1.h>

#include <pluginlib/class_loader.h>
#include <ompl_planning/PlannerConfig.h>
#include <kinematics_base/kinematics_base.h>


namespace ik_constrained_planner
{

static const unsigned int IK_CONSTRAINED_DIMENSION = 7;
static const double IK_CONSTRAINED_MIN_LINEAR_STATE = -5.0;
static const double IK_CONSTRAINED_MAX_LINEAR_STATE = 5.0;
static const double IK_CONSTRAINED_RESOLUTION_LINEAR_STATE = 0.01;
static const double IK_CONSTRAINED_RESOLUTION_WRAPPING_ANGLE = 0.01;

struct Solution
{
  ompl::base::Path *path;
  double            difference;
  bool              approximate;
};	

class IKConstrainedPlanner
{

public:

  IKConstrainedPlanner():kinematics_loader_("kinematics_base","kinematics::KinematicsBase"){};

  ~IKConstrainedPlanner()
  {
    for(std::map<std::string,ompl::base::Planner*>::iterator pm = planner_map_.begin(); pm != planner_map_.end(); ++pm)
      delete pm->second;

    for(std::map<std::string,kinematics::KinematicsBase*>::iterator kb = kinematics_solver_map_.begin(); kb != kinematics_solver_map_.end(); kb++)
      delete kb->second;
  }

  bool isRequestValid(motion_planning_msgs::GetMotionPlan::Request &req);
    
  void setWorkspaceBounds(motion_planning_msgs::WorkspaceParameters &workspace_parameters, 
                          ompl::base::SpaceInformation *space_information);

  bool configureOnRequest(motion_planning_msgs::GetMotionPlan::Request &req, 
                          ompl::base::SpaceInformation *space_information);

  void printSettings(ompl::base::SpaceInformation *si);

  bool computePlan(motion_planning_msgs::GetMotionPlan::Request &req, 
                   motion_planning_msgs::GetMotionPlan::Response &res);

  void fillResult(motion_planning_msgs::GetMotionPlan::Request &req, 
                  motion_planning_msgs::GetMotionPlan::Response &res, 
                  const Solution &sol);

  bool callPlanner(ompl::base::Planner *planner,
                   ompl::base::SpaceInformation *space_information,
                   motion_planning_msgs::GetMotionPlan::Request &req, 
                   Solution &sol);

  ompl::base::ProjectionEvaluator* getProjectionEvaluator(boost::shared_ptr<ompl_planning::PlannerConfig> &options, 
                                                          ompl::base::SpaceInformation * space_information) const;
  
  bool initializeSpaceInformation(planning_environment::PlanningMonitor *planning_monitor_,
                                  const std::string &param_server_prefix);

  bool getStateSpecs(const std::string &param_server_prefix, 
                     const std::string &state_name,
                     ompl::base::StateComponent &state_specification);

  bool initializeKinematics(const std::string &param_server_prefix,
                            const std::vector<std::string> &group_names);

  bool getGroupNames(const std::string &param_server_prefix,
                     std::vector<std::string> &group_names);

  bool addPlanner(const std::string &param_server_prefix,
                  const std::string &planner_config_name,
                  const std::string &model_name,
                  ompl::base::SpaceInformation* space_information);

  bool addPlanner(boost::shared_ptr<ompl_planning::PlannerConfig> &cfg, 
                  const std::string &model_name,
                  ompl::base::SpaceInformation* space_information);

  bool initializePlanners(planning_environment::PlanningMonitor *planning_monitor,
                          const std::string &param_server_prefix,
                          const std::vector<std::string> &group_names);

  bool initialize(planning_environment::PlanningMonitor *planning_monitor,
                  const std::string &param_server_prefix);

  void updateStateComponents(const motion_planning_msgs::Constraints &constraints);

  void resetStateComponents();

private:

  kinematics::KinematicsBase* kinematics_solver_;
  pluginlib::ClassLoader<kinematics::KinematicsBase> kinematics_loader_;
  std::vector<std::string> group_names_;
  planning_environment::PlanningMonitor *planning_monitor_;
  ompl::base::SpaceInformation *space_information_;
  std::vector<ompl::base::StateComponent> original_state_specification_;
  ros::NodeHandle node_handle_;

  std::map<std::string,kinematics::KinematicsBase*> kinematics_solver_map_;
  std::map<std::string,ompl::base::Planner*> planner_map_;
  std::map<std::string,ompl::base::StateComponent> redundancy_map_;  
  std::map<std::string,std::string> redundant_joint_map_;  

  std::string default_planner_id_;
  ompl::base::Goal* computeGoalFromConstraints(ompl::base::SpaceInformation *space_information, 
                                               const motion_planning_msgs::Constraints &constraints,
                                               const std::string &group_name);
  std::vector<std::string> state_names_;
  ompl::base::StateValidityChecker* validity_checker_;
  tf::TransformListener tf_listener_;

  bool solver_initialized_;
  geometry_msgs::PoseStamped kinematics_planner_frame_;
  void contactFound(collision_space::EnvironmentModel::Contact &contact);
  std::vector<planning_environment_msgs::ContactInformation> contact_information_;
  ros::Publisher vis_marker_publisher_;
  void ikDesiredPoseCallback(const geometry_msgs::Pose &ik_pose,
			     const std::vector<double> &ik_solution,
			     int &error_code);

  void ikSolutionCallback(const geometry_msgs::Pose &ik_pose,
			  const std::vector<double> &ik_solution,
			  int &error_code);

  bool computeRedundancyFromConstraints(const motion_planning_msgs::Constraints &goal_constraint,
					geometry_msgs::PoseStamped &kinematics_planner_frame,
					kinematics::KinematicsBase *kinematics_solver,
					const std::string &redundant_joint_name,
					double &redundancy);

};
}
