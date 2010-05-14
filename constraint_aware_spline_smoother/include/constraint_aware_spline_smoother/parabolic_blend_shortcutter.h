/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

/** \author Sachin Chitta */

#ifndef CUBIC_SPLINE_SHORT_CUTTER_H_
#define CUBIC_SPLINE_SHORT_CUTTER_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include <spline_smoother/spline_smoother.h>
#include <spline_smoother/cubic_trajectory.h>
#include <planning_environment/monitors/planning_monitor.h>
#include <motion_planning_msgs/RobotState.h>
#include <motion_planning_msgs/ArmNavigationErrorCodes.h>
#include <motion_planning_msgs/LinkPadding.h>

#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <constraint_aware_spline_smoother/Math.h>
#include <constraint_aware_spline_smoother/DynamicPath.h>

namespace constraint_aware_spline_smoother
{
static const double MIN_DELTA = 0.01;

class FeasibilityChecker : public FeasibilityCheckerBase
{
public: 
  FeasibilityChecker();
  virtual bool ConfigFeasible(const Vector& x);
  virtual bool SegmentFeasible(const Vector& a,const Vector& b);
  bool setInitial(const trajectory_msgs::JointTrajectory &trajectory,
                  const motion_planning_msgs::OrderedCollisionOperations &ordered_collision_operations,
                  const std::vector<motion_planning_msgs::AllowedContactSpecification> &allowed_contact_regions,
                  const std::vector<motion_planning_msgs::LinkPadding> &link_padding,
                  const motion_planning_msgs::Constraints &path_constraints);
  void resetRequest();
  bool isActive();
  void initialize();
private:
  std::vector<std::string> joint_names_;
  bool active_;
  double discretization_;
  tf::TransformListener tf_;
  ros::NodeHandle node_handle_;
  bool setupCollisionEnvironment();
  planning_environment::CollisionModels *collision_models_;
  planning_environment::PlanningMonitor *planning_monitor_;    
  void discretizeTrajectory(const trajectory_msgs::JointTrajectory &trajectory, 
                            trajectory_msgs::JointTrajectory &trajectory_out);
};

FeasibilityChecker::FeasibilityChecker() : FeasibilityCheckerBase(), node_handle_("~")
{
  initialize();
}

bool FeasibilityChecker::isActive()
{
  return active_;
}

void FeasibilityChecker::initialize()
{
  if(!setupCollisionEnvironment())
  {
    ROS_ERROR("Could not setup collision environment");
    active_ = false;
  }
  else
  {
    ROS_INFO("Setup collision environment");
    active_ = true;
  }
}

bool FeasibilityChecker::setInitial(const trajectory_msgs::JointTrajectory &trajectory,
                                    const motion_planning_msgs::OrderedCollisionOperations &ordered_collision_operations,
                                    const std::vector<motion_planning_msgs::AllowedContactSpecification> &allowed_contact_regions,
                                    const std::vector<motion_planning_msgs::LinkPadding> &link_padding,
                                    const motion_planning_msgs::Constraints &path_constraints)
{
  std::vector<std::string> child_links;
  motion_planning_msgs::RobotState robot_state;
  motion_planning_msgs::ArmNavigationErrorCodes error_code;
  motion_planning_msgs::OrderedCollisionOperations operations;

  joint_names_ = trajectory.joint_names;
  planning_monitor_->getRobotStateMsg(robot_state);
  planning_monitor_->getChildLinks(trajectory.joint_names, child_links);
  planning_monitor_->getOrderedCollisionOperationsForOnlyCollideLinks(child_links,ordered_collision_operations,operations);
  planning_monitor_->applyLinkPaddingToCollisionSpace(link_padding);
  planning_monitor_->applyOrderedCollisionOperationsToCollisionSpace(operations);
  planning_monitor_->setAllowedContacts(allowed_contact_regions);
  planning_monitor_->setPathConstraints(path_constraints,error_code);
  if(error_code.val != error_code.SUCCESS)
  {
    ROS_ERROR("Could not set path constraints");
    return false;
  }
  return true;
}

void FeasibilityChecker::resetRequest()
{
  planning_monitor_->revertAllowedCollisionToDefault();
  planning_monitor_->revertCollisionSpacePaddingToDefault();
  planning_monitor_->clearAllowedContacts();
  planning_monitor_->clearConstraints();
  planning_monitor_->getKinematicModel()->lock();
  planning_monitor_->getEnvironmentModel()->lock();
  planning_monitor_->revertAllowedCollisionToDefault();
  planning_monitor_->getKinematicModel()->unlock();
  planning_monitor_->getEnvironmentModel()->unlock();
}

bool FeasibilityChecker::setupCollisionEnvironment()
{
  bool use_collision_map;
  node_handle_.param<bool>("use_collision_map", use_collision_map, true);

  // monitor robot
  collision_models_ = new planning_environment::CollisionModels("robot_description");
  planning_monitor_ = new planning_environment::PlanningMonitor(collision_models_, &tf_);
  planning_monitor_->use_collision_map_ = use_collision_map;
  if(!collision_models_->loadedModels())
    return false;

  if (planning_monitor_->getExpectedJointStateUpdateInterval() > 1e-3)
    planning_monitor_->waitForState();
  if (planning_monitor_->getExpectedMapUpdateInterval() > 1e-3 && use_collision_map)
    planning_monitor_->waitForMap();
  return true;
}

void FeasibilityChecker::discretizeTrajectory(const trajectory_msgs::JointTrajectory &trajectory, 
                                              trajectory_msgs::JointTrajectory &trajectory_out)
{    
  trajectory_out.joint_names = trajectory.joint_names;
  for(unsigned int i=1; i < trajectory.points.size(); i++)
  {
    double diff = 0.0;      
    for(unsigned int j=0; j < trajectory.points[i].positions.size(); j++)
    {
      double start = trajectory.points[i-1].positions[j];
      double end   = trajectory.points[i].positions[j];
      if(fabs(end-start) > diff)
        diff = fabs(end-start);        
    }
    int num_intervals =(int) (diff/MIN_DELTA+1.0);
      
    for(unsigned int k=0; k < (unsigned int) num_intervals; k++)
    {
      trajectory_msgs::JointTrajectoryPoint point;
      for(unsigned int j=0; j < trajectory.points[i].positions.size(); j++)
      {
        double start = trajectory.points[i-1].positions[j];
        double end   = trajectory.points[i].positions[j];
        point.positions.push_back(start + (end-start)*k/num_intervals);
      }
      point.time_from_start = ros::Duration(trajectory.points[i].time_from_start.toSec() + k* (trajectory.points[i].time_from_start - trajectory.points[i-1].time_from_start).toSec()/num_intervals);
      trajectory_out.points.push_back(point);
    }
  }
  trajectory_out.points.push_back(trajectory.points.back());
}

bool FeasibilityChecker::ConfigFeasible(const Vector& x)
{
  motion_planning_msgs::ArmNavigationErrorCodes error_code;
  std::vector<motion_planning_msgs::ArmNavigationErrorCodes> trajectory_error_codes;
  motion_planning_msgs::RobotState robot_state;
  planning_monitor_->getRobotStateMsg(robot_state);

  trajectory_msgs::JointTrajectory joint_traj;
  joint_traj.joint_names = joint_names_;
  joint_traj.header.stamp = ros::Time::now();
  joint_traj.points.resize(1);
  joint_traj.points[0].positions = x;
  return planning_monitor_->isTrajectoryValid(joint_traj,robot_state,0,joint_traj.points.size(),
                                              planning_environment::PlanningMonitor::COLLISION_TEST | planning_environment::PlanningMonitor::PATH_CONSTRAINTS_TEST,
                                              false,error_code,trajectory_error_codes);    
}

bool FeasibilityChecker::SegmentFeasible(const Vector& a,const Vector& b)
{
  motion_planning_msgs::ArmNavigationErrorCodes error_code;
  std::vector<motion_planning_msgs::ArmNavigationErrorCodes> trajectory_error_codes;
  motion_planning_msgs::RobotState robot_state;
  planning_monitor_->getRobotStateMsg(robot_state);

  trajectory_msgs::JointTrajectory joint_traj_in, joint_traj;
  joint_traj_in.joint_names = joint_names_;
  joint_traj.header.stamp = ros::Time::now();
  joint_traj_in.points.resize(2);
  joint_traj_in.points[0].positions = a;
  joint_traj_in.points[1].positions = b;
  joint_traj.joint_names = joint_traj_in.joint_names;
  discretizeTrajectory(joint_traj_in,joint_traj);
  return planning_monitor_->isTrajectoryValid(joint_traj,robot_state,0,joint_traj.points.size(),
                                              planning_environment::PlanningMonitor::COLLISION_TEST | planning_environment::PlanningMonitor::PATH_CONSTRAINTS_TEST,
                                              false,error_code,trajectory_error_codes);
}

/**
 * \brief Scales the time intervals stretching them if necessary so that the trajectory conforms to velocity limits
 */
template <typename T>
class ParabolicBlendShortCutter: public spline_smoother::SplineSmoother<T>
{
public:
  /**
   * \brief Construct the smoother
   */
  ParabolicBlendShortCutter();
  virtual ~ParabolicBlendShortCutter();
  virtual bool smooth(const T& trajectory_in, 
                      T& trajectory_out) const;
  /** \brief Configure the filter with the discretization for returned trajectories
   */
  virtual bool configure();
private:
  int num_iterations_;
  double discretization_;
  bool active_;
  boost::shared_ptr<FeasibilityChecker> feasibility_checker_;
};

template <typename T>
bool ParabolicBlendShortCutter<T>::configure()
{
  if (!spline_smoother::SplineSmoother<T>::getParam("discretization", discretization_))
  {
    ROS_ERROR("Spline smoother, \"%s\", params has no attribute discretization.", spline_smoother::SplineSmoother<T>::getName().c_str());
    return false;
  }
  if (!spline_smoother::SplineSmoother<T>::getParam("num_iterations", num_iterations_))
  {
    ROS_ERROR("Spline smoother, \"%s\", params has no attribute num_iterations.", spline_smoother::SplineSmoother<T>::getName().c_str());
    return false;
  }
  ROS_INFO("Configuring parabolic blend short cutter");
  ROS_INFO("Using a discretization value of %f",discretization_);
  ROS_INFO("Using num_iterations value of %d",(int)num_iterations_);
  feasibility_checker_.reset(new constraint_aware_spline_smoother::FeasibilityChecker());
  return true;
};

template <typename T>
ParabolicBlendShortCutter<T>::ParabolicBlendShortCutter()
{
  ROS_INFO("Setting up parabolic blend short cutter");
}

template <typename T>
ParabolicBlendShortCutter<T>::~ParabolicBlendShortCutter()
{
}

template <typename T>
bool ParabolicBlendShortCutter<T>::smooth(const T& trajectory_in, 
                                          T& trajectory_out) const
{
  srand(time(NULL)); // initialize random seed: 
  if(!feasibility_checker_->isActive())
  {
    ROS_ERROR("Smoother is not active");
    return false;
  }
  
  feasibility_checker_->setInitial(trajectory_in.trajectory,
                                   trajectory_in.ordered_collision_operations,
                                   trajectory_in.allowed_contacts,
                                   trajectory_in.link_padding,
                                   trajectory_in.path_constraints);
  std::vector<Vector> path;        //the sequence of milestones
  Vector vmax,amax;           //velocity and acceleration bounds, respectively
  Real tol=1e-4;              //if a point is feasible, any point within tol is considered acceptable
  //TODO: compute milestones, velocity and acceleration bounds

  vmax.resize(trajectory_in.limits.size());
  amax.resize(trajectory_in.limits.size());

  for(unsigned int i=0; i < trajectory_in.limits.size(); i++)
  {
    vmax[i] = trajectory_in.limits[i].max_velocity;
    amax[i] = trajectory_in.limits[i].max_acceleration;
  }

  for(unsigned int i=0; i<trajectory_in.trajectory.points.size(); i++)
  {
    path.push_back(trajectory_in.trajectory.points[i].positions);
  }                   

  DynamicPath traj;
  traj.Init(vmax,amax);
  traj.SetMilestones(path);   //now the trajectory starts and stops at every milestone
  ROS_DEBUG("Initial path duration: %g\n",(double)traj.GetTotalTime());
  int res=traj.Shortcut(num_iterations_,feasibility_checker_.get(),tol);
  ROS_DEBUG("After shortcutting: duration %g\n",(double)traj.GetTotalTime());
  unsigned int num_points = (unsigned int)(traj.GetTotalTime()/discretization_+0.5) + 1;
  double totalTime = (double) traj.GetTotalTime();
  for(unsigned int i=0; i < num_points; i++)
  {
    Vector x, dx;
    double t = i*totalTime/(num_points-1);
    traj.Evaluate(t,x);
    traj.Derivative(t,dx);
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = x;
    point.velocities = dx;
    point.time_from_start = ros::Duration(t);
    trajectory_out.trajectory.points.push_back(point);
  }
  trajectory_out.trajectory.joint_names = trajectory_in.trajectory.joint_names;
  feasibility_checker_->resetRequest();
  return res;
}

}

#endif /* CUBIC_SPLINE_SMOOTHER_H_ */
