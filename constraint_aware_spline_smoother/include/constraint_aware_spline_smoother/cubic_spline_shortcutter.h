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
#include <planning_environment/models/collision_models_interface.h>
#include <planning_environment/models/model_utils.h>
#include <arm_navigation_msgs/RobotState.h>
#include <arm_navigation_msgs/ArmNavigationErrorCodes.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace constraint_aware_spline_smoother
{

/**
 * \brief Scales the time intervals stretching them if necessary so that the trajectory conforms to velocity limits
 */
template <typename T>
class CubicSplineShortCutter: public spline_smoother::SplineSmoother<T>
{
public:
 /**
   * \brief Construct the smoother
   */
  CubicSplineShortCutter();
  virtual ~CubicSplineShortCutter();
  virtual bool smooth(const T& trajectory_in, 
                      T& trajectory_out) const;
  /** \brief Configure the filter with the discretization for returned trajectories
   */
  virtual bool configure();
private:
  bool active_;
  double discretization_;
  bool setupCollisionEnvironment();
  planning_environment::CollisionModelsInterface *collision_models_interface_;
  //  ros::NodeHandle node_handle_;
  int getRandomInt(int min,int max) const;
  double getRandomTimeStamp(double min,double max) const;
  void discretizeTrajectory(const spline_smoother::SplineTrajectory &spline, 
                            const double &discretization,
                            trajectory_msgs::JointTrajectory &joint_trajectory) const;
  bool trimTrajectory(trajectory_msgs::JointTrajectory &trajectory_out, 
                      const double &segment_start_time, 
                      const double &segment_end_time) const;
  bool findTrajectoryPointsInInterval(const trajectory_msgs::JointTrajectory &trajectory,
                                      const double &segment_start_time, 
                                      const double &segment_end_time,
                                      int &index_1,
                                      int &index_2) const;
  bool getWaypoints(const spline_smoother::SplineTrajectory &spline, 
                    trajectory_msgs::JointTrajectory &joint_trajectory) const;
  bool addToTrajectory(trajectory_msgs::JointTrajectory &trajectory_out, 
                       const trajectory_msgs::JointTrajectoryPoint &trajectory_point,
                       const ros::Duration& delta_time) const;

  void printTrajectory(const trajectory_msgs::JointTrajectory &joint_trajectory) const;

  void discretizeAndAppendSegment(const spline_smoother::SplineTrajectorySegment &spline_segment,
                                  const double &discretization,
                                  trajectory_msgs::JointTrajectory &joint_trajectory,
                                  const ros::Duration &segment_start_time,
                                  const bool &include_segment_end) const;

  double maxLInfDistance(const trajectory_msgs::JointTrajectoryPoint &start, 
                         const trajectory_msgs::JointTrajectoryPoint &end) const;

  void refineTrajectory(T &trajectory) const;

};

template <typename T>
bool CubicSplineShortCutter<T>::configure()
{
  if (!spline_smoother::SplineSmoother<T>::getParam("discretization", discretization_))
  {
    ROS_ERROR("Spline smoother, \"%s\", params has no attribute discretization.", spline_smoother::SplineSmoother<T>::getName().c_str());
    return false;
  }///\todo check length
  else
  {
    ROS_DEBUG("Using a discretization value of %f",discretization_);
    return true;
  }
};

template <typename T>
CubicSplineShortCutter<T>::CubicSplineShortCutter()
{
  if(!setupCollisionEnvironment())
    active_ = false;
  else
    active_ = true;
}

template <typename T>
CubicSplineShortCutter<T>::~CubicSplineShortCutter()
{
}

template <typename T>
int CubicSplineShortCutter<T>::getRandomInt(int min_index,int max_index)const
{
  int rand_num = rand()%100+1;
  double result = min_index + (double)((max_index-min_index)*rand_num)/101.0;
  return (int) result;
}

template <typename T>
double CubicSplineShortCutter<T>::getRandomTimeStamp(double min,double max)const
{
  int rand_num = rand()%100+1;
  double result = min + (double)((max-min)*rand_num)/101.0;
  return result;
}

template <typename T>
bool CubicSplineShortCutter<T>::smooth(const T& trajectory_in, 
                                       T& trajectory_out) const
{
  double discretization = discretization_;
  srand(time(NULL)); // initialize random seed: 
  if(!active_)
  {
    ROS_ERROR("Smoother is not active");
    return false;
  }

  ROS_INFO("Got trajectory with %d points",(int)trajectory_in.request.trajectory.points.size());

  if(!collision_models_interface_->isPlanningSceneSet()) {
    ROS_INFO("Planning scene not set, can't do anything");
    return false;
  }

  arm_navigation_msgs::ArmNavigationErrorCodes error_code;
  std::vector<arm_navigation_msgs::ArmNavigationErrorCodes> trajectory_error_codes;
  arm_navigation_msgs::RobotState robot_state;
  spline_smoother::CubicTrajectory trajectory_solver;
  spline_smoother::SplineTrajectory spline, shortcut_spline;
  arm_navigation_msgs::JointTrajectoryWithLimits shortcut, discretized_trajectory;

  trajectory_out.request = trajectory_in.request;

  collision_models_interface_->disableCollisionsForNonUpdatedLinks(trajectory_in.request.group_name);

  planning_environment::setRobotStateAndComputeTransforms(trajectory_in.request.start_state,
                                                          *collision_models_interface_->getPlanningSceneState());

  if(!collision_models_interface_->isJointTrajectoryValid(*collision_models_interface_->getPlanningSceneState(),
                                                          trajectory_out.request.trajectory,
                                                          trajectory_in.request.goal_constraints,
                                                          trajectory_in.request.path_constraints,
                                                          trajectory_out.response.error_code,
                                                          trajectory_error_codes,
                                                          false)) {
    ROS_INFO_STREAM("Original trajectory invalid with error code " << trajectory_out.response.error_code.val);
    return false;
  }


  if (!spline_smoother::checkTrajectoryConsistency(trajectory_out))
    return false;

  shortcut.limits = trajectory_in.request.limits;
  shortcut.trajectory.joint_names = trajectory_in.request.trajectory.joint_names;

  discretized_trajectory.limits = trajectory_in.request.limits;
  discretized_trajectory.trajectory.joint_names = trajectory_in.request.trajectory.joint_names;

  ros::Time start_time = ros::Time::now();
  ros::Duration timeout = trajectory_in.request.allowed_time;

  bool success = trajectory_solver.parameterize(trajectory_out.request.trajectory,trajectory_in.request.limits,spline);      
  getWaypoints(spline,trajectory_out.request.trajectory);
  printTrajectory(trajectory_out.request.trajectory);
  
  for(unsigned int i = 0; i < trajectory_in.request.limits.size(); i++) {
    ROS_DEBUG_STREAM("Joint " << trajectory_in.request.limits[i].joint_name 
                     << " has " << (bool) trajectory_in.request.limits[i].has_position_limits
                     << " low " << trajectory_in.request.limits[i].min_position
                     << " high " << trajectory_in.request.limits[i].max_position);
  }

  if(!collision_models_interface_->isJointTrajectoryValid(*collision_models_interface_->getPlanningSceneState(),
                                                          trajectory_out.request.trajectory,
                                                          trajectory_in.request.goal_constraints,
                                                          trajectory_in.request.path_constraints,
                                                          trajectory_out.response.error_code,
                                                          trajectory_error_codes,
                                                          false)) {
    ROS_INFO_STREAM("Original sampled trajectory invalid with error code " << trajectory_out.response.error_code.val);
    return false;
  } else {
    ROS_DEBUG_STREAM("Originally sampled trajectory ok");
  }
  
  T test_traj = trajectory_out;

  discretizeTrajectory(spline,discretization,test_traj.request.trajectory);

  if(!collision_models_interface_->isJointTrajectoryValid(*collision_models_interface_->getPlanningSceneState(),
                                                          test_traj.request.trajectory,
                                                          trajectory_in.request.goal_constraints,
                                                          trajectory_in.request.path_constraints,
                                                          trajectory_out.response.error_code,
                                                          trajectory_error_codes,
                                                          false)) {
    ROS_WARN_STREAM("Original discretized trajectory invalid with error code " << trajectory_out.response.error_code.val);
    trajectory_out.request = test_traj.request;
    return false;
  } else {
    ROS_DEBUG_STREAM("Originally discretized trajectory ok");
  }
  

  std::vector<double> sample_times;
  sample_times.resize(2);
  bool first_try = true;
  while(ros::Time::now() - start_time < timeout)
  {
    double total_time = trajectory_out.request.trajectory.points.back().time_from_start.toSec();
    double segment_start_time = getRandomTimeStamp(0.0,total_time);
    double segment_end_time = getRandomTimeStamp(segment_start_time,total_time);
    if(segment_start_time == segment_end_time)
      continue;
    if(first_try)
    {
      segment_start_time = 0.0;
      segment_end_time = total_time;
      first_try = false;
    }
    sample_times[0] = segment_start_time;
    sample_times[1] = segment_end_time;

    spline_smoother::sampleSplineTrajectory(spline,sample_times,shortcut.trajectory);
    ROS_DEBUG("Start time: %f, %f",segment_start_time,shortcut.trajectory.points[0].positions[0]);
    ROS_DEBUG("End time  : %f, %f",segment_end_time,shortcut.trajectory.points[1].positions[0]);
    shortcut.trajectory.points[0].time_from_start = ros::Duration(0.0);
    shortcut.trajectory.points[1].time_from_start = ros::Duration(0.0);
    
    if(!trajectory_solver.parameterize(shortcut.trajectory,trajectory_in.request.limits,shortcut_spline))
      return false;
    discretizeTrajectory(shortcut_spline,discretization,discretized_trajectory.trajectory);
    ROS_DEBUG("Succeeded in sampling trajectory with size: %d",(int)discretized_trajectory.trajectory.points.size());

    arm_navigation_msgs::Constraints empty_goal_constraints;

    if(collision_models_interface_->isJointTrajectoryValid(*collision_models_interface_->getPlanningSceneState(),
                                                           discretized_trajectory.trajectory,
                                                           empty_goal_constraints,
                                                           trajectory_in.request.path_constraints,
                                                           error_code,
                                                           trajectory_error_codes,
                                                           false))
    {
      ros::Duration shortcut_duration = discretized_trajectory.trajectory.points.back().time_from_start - discretized_trajectory.trajectory.points.front().time_from_start;
      if(segment_end_time-segment_start_time <= shortcut_duration.toSec())
        continue;
      if(!trimTrajectory(trajectory_out.request.trajectory,segment_start_time,segment_end_time))
        continue;
      ROS_DEBUG_STREAM("Trimmed trajectory has " << trajectory_out.request.trajectory.points.size() << " points");

      ROS_DEBUG("Shortcut reduced duration from: %f to %f",
                segment_end_time-segment_start_time,
                shortcut_duration.toSec());
      shortcut.trajectory.points[0].time_from_start = ros::Duration(segment_start_time);
      shortcut.trajectory.points[1].time_from_start = ros::Duration(segment_start_time) + shortcut_duration;
      addToTrajectory(trajectory_out.request.trajectory,
                      shortcut.trajectory.points[0],
                      ros::Duration(0.0));
      addToTrajectory(trajectory_out.request.trajectory,
                      shortcut.trajectory.points[1],
                      shortcut_duration-ros::Duration(segment_end_time-segment_start_time));
      spline.segments.clear();
      if(!trajectory_solver.parameterize(trajectory_out.request.trajectory,trajectory_in.request.limits,spline)) {
        trajectory_out.response.error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_TRAJECTORY;
        return false;
      }
      if(!getWaypoints(spline,trajectory_out.request.trajectory)) {
        trajectory_out.response.error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_TRAJECTORY;
        return false;
      }
      printTrajectory(trajectory_out.request.trajectory);
      if(trajectory_out.request.trajectory.points.size() < 3)
        break;
    }
    else 
    {
      ROS_DEBUG("Traj segment rejected with error code: %d",error_code.val);
      continue;
    }
  }
  ROS_INFO("Trajectory filter took %f seconds",(ros::Time::now() - start_time).toSec());
  for(unsigned int i=0; i < trajectory_out.request.trajectory.points.size(); i++)
  {
    trajectory_out.request.trajectory.points[i].accelerations.clear();
  }
  if(!collision_models_interface_->isJointTrajectoryValid(*collision_models_interface_->getPlanningSceneState(),
                                                          trajectory_out.request.trajectory,
                                                          trajectory_in.request.goal_constraints,
                                                          trajectory_in.request.path_constraints,
                                                          trajectory_out.response.error_code,
                                                          trajectory_error_codes,
                                                          false)) {
    ROS_INFO_STREAM("Before refine trajectory invalid with error code " << trajectory_out.response.error_code.val);
  } else {
    ROS_DEBUG_STREAM("Before refine trajectory ok");
  }

  printTrajectory(trajectory_out.request.trajectory);
  refineTrajectory(trajectory_out);
  
  if(!collision_models_interface_->isJointTrajectoryValid(*collision_models_interface_->getPlanningSceneState(),
                                                          trajectory_out.request.trajectory,
                                                          trajectory_in.request.goal_constraints,
                                                          trajectory_in.request.path_constraints,
                                                          trajectory_out.response.error_code,
                                                          trajectory_error_codes,
                                                          false)) {
    ROS_INFO_STREAM("Before waypoints trajectory invalid with error code " << trajectory_out.response.error_code.val);
  } else {
    ROS_DEBUG_STREAM("Before waypoints trajectory ok");
  }

  if(!trajectory_solver.parameterize(trajectory_out.request.trajectory,trajectory_in.request.limits,spline))
    return false;
  if(!getWaypoints(spline,trajectory_out.request.trajectory))
    return false;
  if(!collision_models_interface_->isJointTrajectoryValid(*collision_models_interface_->getPlanningSceneState(),
                                                          trajectory_out.request.trajectory,
                                                          trajectory_in.request.goal_constraints,
                                                          trajectory_in.request.path_constraints,
                                                          error_code,
                                                          trajectory_error_codes,
                                                          false)) {
    ROS_INFO_STREAM("Before discretize trajectory invalid with error code " << error_code.val);
  } else {
    ROS_DEBUG_STREAM("Before discretize trajectory ok");
  }
  discretizeTrajectory(spline,discretization,trajectory_out.request.trajectory);

  trajectory_out.request.limits = trajectory_in.request.limits;

  printTrajectory(trajectory_out.request.trajectory);
	
  ROS_DEBUG("Final trajectory has %d points and %f total time",(int)trajectory_out.request.trajectory.points.size(),
            trajectory_out.request.trajectory.points.back().time_from_start.toSec());
  
  if(!collision_models_interface_->isJointTrajectoryValid(*collision_models_interface_->getPlanningSceneState(),
                                                          trajectory_out.request.trajectory,
                                                          trajectory_in.request.goal_constraints,
                                                          trajectory_in.request.path_constraints,
                                                          trajectory_out.response.error_code,
                                                          trajectory_error_codes,
                                                          false)) {
    ROS_INFO_STREAM("Final trajectory invalid with error code " << error_code.val);
    ROS_INFO_STREAM("Trajectory error codes size is " << trajectory_error_codes.size());
    return false;
  } else {
    ROS_DEBUG_STREAM("Final trajectory ok");
  }

  return success;
}

template <typename T>
void CubicSplineShortCutter<T>::refineTrajectory(T &trajectory) const
{
  if(trajectory.request.trajectory.points.size() < 3)
    return;

  for(unsigned int i=1; i < trajectory.request.trajectory.points.size()-1; i++)
  {
    for(unsigned int j=0; j < trajectory.request.trajectory.points[i].positions.size(); j++)
    {
      double dq_first = trajectory.request.trajectory.points[i].positions[j] - trajectory.request.trajectory.points[i-1].positions[j];
      double dq_second = trajectory.request.trajectory.points[i+1].positions[j] - trajectory.request.trajectory.points[i].positions[j];
      //double dq_dot = trajectory.request.trajectory.points[i].velocities[j];
      double dt_first = (trajectory.request.trajectory.points[i].time_from_start - trajectory.request.trajectory.points[i-1].time_from_start).toSec();
      double dt_second = (trajectory.request.trajectory.points[i+1].time_from_start - trajectory.request.trajectory.points[i].time_from_start).toSec();
      if( (dq_first > 0 && dq_second > 0) || (dq_first < 0 && dq_second < 0)) 
      {       
        if(trajectory.request.trajectory.points[i].velocities[j] == 0.0)
        {
          trajectory.request.trajectory.points[i].velocities[j] = 0.5*(dq_first/dt_first + dq_second/dt_second);
          trajectory.request.trajectory.points[i].velocities[j] = std::max(std::min(trajectory.request.trajectory.points[i].velocities[j],
                                                                                    trajectory.request.limits[j].max_velocity),
                                                                           -trajectory.request.limits[j].max_velocity);
        }
      }
    }
  }
}

template <typename T>
void CubicSplineShortCutter<T>::printTrajectory(const trajectory_msgs::JointTrajectory &trajectory) const
{
  for(unsigned int i = 0; i < trajectory.points.size(); i++)
  {
    ROS_DEBUG("%f: %f %f %f %f %f %f %f, %f %f %f %f %f %f %f, %f %f %f %f %f %f %f",
             trajectory.points[i].time_from_start.toSec(),
             trajectory.points[i].positions[0],
             trajectory.points[i].positions[1],
             trajectory.points[i].positions[2],
             trajectory.points[i].positions[3],
             trajectory.points[i].positions[4],
             trajectory.points[i].positions[5],
             trajectory.points[i].positions[6],
             trajectory.points[i].velocities[0],
             trajectory.points[i].velocities[1],
             trajectory.points[i].velocities[2],
             trajectory.points[i].velocities[3],
             trajectory.points[i].velocities[4],
             trajectory.points[i].velocities[5],
             trajectory.points[i].velocities[6],
             trajectory.points[i].accelerations[0],
             trajectory.points[i].accelerations[1],
             trajectory.points[i].accelerations[2],
             trajectory.points[i].accelerations[3],
             trajectory.points[i].accelerations[4],
             trajectory.points[i].accelerations[5],
             trajectory.points[i].accelerations[6]);
  }
  ROS_DEBUG(" ");
}

/*template <typename T>
void CubicSplineShortCutter<T>::discretizeTrajectory(const spline_smoother::SplineTrajectory &spline, 
                                                     const double &discretization,
                                                     trajectory_msgs::JointTrajectory &joint_trajectory) const
{
  double total_time;
  spline_smoother::getTotalTime(spline,total_time);
  std::vector<double> times;
  double time_index = 0.0;
  while(time_index < total_time)
  {
    times.push_back(time_index);
    time_index += discretization;
  }
  times.push_back(total_time);  
  spline_smoother::sampleSplineTrajectory(spline,times,joint_trajectory);
}
*/

template <typename T>
void CubicSplineShortCutter<T>::discretizeTrajectory(const spline_smoother::SplineTrajectory &spline, 
                                                     const double &discretization,
                                                     trajectory_msgs::JointTrajectory &joint_trajectory) const
{
  if(spline.segments.empty())
    return;
  joint_trajectory.points.clear();
  ros::Duration segment_start_time(0.0);
  for(unsigned int i=0; i < spline.segments.size(); i++)
  {
    if(i == spline.segments.size()-1)
      discretizeAndAppendSegment(spline.segments[i],discretization,joint_trajectory,segment_start_time,true);
    else
      discretizeAndAppendSegment(spline.segments[i],discretization,joint_trajectory,segment_start_time,false);
    segment_start_time += spline.segments[i].duration;
    ROS_DEBUG("Discretizing and appending segment %d",i);
  }
}

template <typename T>
void CubicSplineShortCutter<T>::discretizeAndAppendSegment(const spline_smoother::SplineTrajectorySegment &spline_segment,
                                                           const double &discretization,
                                                           trajectory_msgs::JointTrajectory &joint_trajectory,
                                                           const ros::Duration &segment_start_time,
                                                           const bool &include_segment_end) const
{
  ros::Duration time_from_start = segment_start_time;
  double total_time = spline_segment.duration.toSec();
  double sample_time = 0.0;
  trajectory_msgs::JointTrajectoryPoint start,end;
  spline_smoother::sampleSplineTrajectory(spline_segment,0.0,start);
  if(joint_trajectory.points.empty())
  {
    start.time_from_start = ros::Duration(0.0);
    joint_trajectory.points.push_back(start);
    sample_time += 0.01;
  }
  start = joint_trajectory.points.back();
  while(sample_time < total_time)
  {
     ROS_DEBUG("Sample time is %f",sample_time);
     spline_smoother::sampleSplineTrajectory(spline_segment,sample_time,end);
     double max_diff = maxLInfDistance(start,end);
     if(sample_time > 0 && max_diff < discretization)
     {
       ROS_DEBUG("Max diff is %f. Skipping",max_diff);
       sample_time += 0.01;
       continue;
     }
     end.time_from_start = time_from_start + ros::Duration(sample_time);
     joint_trajectory.points.push_back(end);
     ROS_DEBUG("Pushing back point with time: %f",end.time_from_start.toSec());
     sample_time += 0.01;
     start = end;
  }
  if(include_segment_end)
  {
    spline_smoother::sampleSplineTrajectory(spline_segment,total_time,end);
    end.time_from_start = time_from_start + ros::Duration(total_time);
    joint_trajectory.points.push_back(end);
  }    
}

template <typename T>
double CubicSplineShortCutter<T>::maxLInfDistance(const trajectory_msgs::JointTrajectoryPoint &start, 
                                                  const trajectory_msgs::JointTrajectoryPoint &end) const
{
  double max_diff = 0.0;
  for(unsigned int i=0; i< start.positions.size(); i++)
  {
    double diff = fabs(end.positions[i]-start.positions[i]);
    if(diff > max_diff)
      max_diff = diff;
  }
  return max_diff;
}
      


template <typename T>
bool CubicSplineShortCutter<T>::trimTrajectory(trajectory_msgs::JointTrajectory &trajectory_out, 
                                               const double &segment_start_time, 
                                               const double &segment_end_time) const
{
  int index1;
  int index2;
  if(findTrajectoryPointsInInterval(trajectory_out,segment_start_time,segment_end_time,index1,index2))
  {
    ROS_DEBUG("Trimming trajectory between segments: %d and %d",index1,index2);
    std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator remove_start = trajectory_out.points.begin() + index1;
    std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator remove_end;
    if((unsigned int) index2 >= trajectory_out.points.size())
      remove_end = trajectory_out.points.end();
    else
      remove_end = trajectory_out.points.begin()+index2;
      

    if(remove_start != remove_end)
      trajectory_out.points.erase(remove_start,remove_end);
    else
      trajectory_out.points.erase(remove_start);
  }
  else
    return false;
  return true;
}

template <typename T>
bool CubicSplineShortCutter<T>::findTrajectoryPointsInInterval(const trajectory_msgs::JointTrajectory &trajectory,
                                                               const double &segment_start_time, 
                                                               const double &segment_end_time,
                                                               int &index_1,
                                                               int &index_2) const
{
  index_1 = -1;
  index_2 = -1;
  if(segment_start_time > segment_end_time)
    return false;
  for(unsigned int i=0; i < trajectory.points.size(); i++)
    if(trajectory.points[i].time_from_start.toSec() >= segment_start_time)
    {
      index_1 = i;
      break;
    }
  ROS_DEBUG("First trim index: %d",index_1);
  if(index_1>=0)
    for(unsigned int i=index_1; i < trajectory.points.size(); i++)
    {
      if(trajectory.points[i].time_from_start.toSec() > segment_end_time)
      {
        index_2 = i;
        break;
      }
      if(trajectory.points[i].time_from_start.toSec() == segment_end_time)
      {
        index_2 = i+1;
        break;
      }
    }
  ROS_DEBUG("Second trim index: %d",index_2);
  if(index_1 >= index_2 || index_1 < 0 || index_2 < 0)
    return false;
  return true;
}

template <typename T>
bool CubicSplineShortCutter<T>::getWaypoints(const spline_smoother::SplineTrajectory &spline, 
                                             trajectory_msgs::JointTrajectory &joint_trajectory) const
{
  std::vector<double> waypoint_times_vector;
  double waypoint_time = 0.0;
  waypoint_times_vector.push_back(waypoint_time);
  for(unsigned int i=0; i < spline.segments.size(); i++)
  {
    waypoint_time = waypoint_time + spline.segments[i].duration.toSec();
    waypoint_times_vector.push_back(waypoint_time);
    ROS_DEBUG("Spline segment time: %f",spline.segments[i].duration.toSec());
  }
  if(!spline_smoother::sampleSplineTrajectory(spline,waypoint_times_vector,joint_trajectory))
    return false;
  return true;
}

template <typename T>
bool CubicSplineShortCutter<T>::addToTrajectory(trajectory_msgs::JointTrajectory &trajectory_out, 
                                                const trajectory_msgs::JointTrajectoryPoint &trajectory_point,
                                                const ros::Duration &delta_time) const
{

  ROS_DEBUG("Inserting point at time: %f",trajectory_point.time_from_start.toSec());
  ROS_DEBUG("Old trajectory has %u points",(unsigned int)trajectory_out.points.size());

  if(trajectory_out.points.empty())
  {
    trajectory_out.points.push_back(trajectory_point);
    return true;
  }

  unsigned int counter = 0;
  unsigned int old_size = trajectory_out.points.size();
  for(std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator iter = trajectory_out.points.begin(); 
      iter != trajectory_out.points.end() ; iter++)
  {   
    if(iter->time_from_start >= trajectory_point.time_from_start)
    {
      trajectory_out.points.insert(iter,trajectory_point);
      break;
    }
    counter++;
  }

  if(delta_time == ros::Duration(0.0))
    return true;

  if(counter == old_size)
    trajectory_out.points.push_back(trajectory_point);
  else
    if(counter+1 < trajectory_out.points.size())
      for(unsigned int i= counter+1; i < trajectory_out.points.size(); i++)
      {
        trajectory_out.points[i].time_from_start += delta_time;
      } 
  return true;
}

template <typename T>
bool CubicSplineShortCutter<T>::setupCollisionEnvironment()
{
  bool use_collision_map;
  ros::NodeHandle node_handle("~");
  node_handle.param<bool>("use_collision_map", use_collision_map, true);

  // monitor robot
  collision_models_interface_ = new planning_environment::CollisionModelsInterface("robot_description");
  if(!collision_models_interface_->loadedModels())
    return false;

  return true;
}
}

#endif /* CUBIC_SPLINE_SMOOTHER_H_ */
