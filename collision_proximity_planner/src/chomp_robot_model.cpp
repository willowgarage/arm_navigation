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

/** \author Mrinal Kalakrishnan */

#include <collision_proximity_planner/chomp_robot_model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <cstdio>
#include <iostream>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace mapping_msgs;

namespace chomp
{

ChompRobotModel::ChompRobotModel():
  node_handle_("~")
{
}

ChompRobotModel::~ChompRobotModel()
{
}

bool ChompRobotModel::init(const planning_environment::CollisionModels* models)
{
  reference_frame_ = models->getRobotFrameId();

  // get the urdf as a string:
  string urdf_string;
  if (!node_handle_.getParam(models->getDescription(), urdf_string))
  {
    return false;
  }

  // get some other params:
  double joint_update_limit;
  node_handle_.param("joint_update_limit", joint_update_limit, 0.05);

  // Construct the KDL tree
  if (!kdl_parser::treeFromString(urdf_string, kdl_tree_))
  {
    ROS_ERROR("Failed to construct KDL tree from URDF.");
    return false;
  }
  num_kdl_joints_ = kdl_tree_.getNrOfJoints();

  // create the joint_segment_mapping, which used to be created by the URDF -> KDL parser
  // but not any more, but the rest of the code depends on it, so we simply generate the mapping here:
  KDL::SegmentMap segment_map = kdl_tree_.getSegments();

  for (KDL::SegmentMap::const_iterator it = segment_map.begin(); it != segment_map.end(); ++it)
  {
    if (it->second.segment.getJoint().getType() != KDL::Joint::None)
    {
      std::string joint_name = it->second.segment.getJoint().getName();
      std::string segment_name = it->first;
      joint_segment_mapping_.insert(make_pair(joint_name, segment_name));
    }
  }

  // create the fk solver:
  fk_solver_ = new KDL::TreeFkSolverJointPosAxis(kdl_tree_, reference_frame_);

  kdl_number_to_urdf_name_.resize(num_kdl_joints_);
  // Create the inverse mapping - KDL segment to joint name
  // (at the same time) Create a mapping from KDL numbers to URDF joint names and vice versa
  for (map<string, string>::iterator it = joint_segment_mapping_.begin(); it!= joint_segment_mapping_.end(); ++it)
  {
    std::string joint_name = it->first;
    std::string segment_name = it->second;
  //  std::cout << joint_name << " -> " << segment_name << std::endl;
    segment_joint_mapping_.insert(make_pair(segment_name, joint_name));
    int kdl_number = kdl_tree_.getSegment(segment_name)->second.q_nr;
    if (kdl_tree_.getSegment(segment_name)->second.segment.getJoint().getType() != KDL::Joint::None)
    {
  //    std::cout << "Kdl number is " << kdl_number << std::endl;
      kdl_number_to_urdf_name_[kdl_number] = joint_name;
      urdf_name_to_kdl_number_.insert(make_pair(joint_name, kdl_number));
    }
  }

  // initialize the planning groups
  const std::map<std::string, planning_models::KinematicModel::JointModelGroup*> groups = models->getKinematicModel()->getJointModelGroupMap();  
  for(std::map<std::string, planning_models::KinematicModel::JointModelGroup*>::const_iterator it = groups.begin();
      it != groups.end();
      it++) {
    ChompPlanningGroup group;
    ROS_INFO_STREAM("Making group for " << it->first);
    group.name_ = it->first;
    int num_links = it->second->getGroupLinkModels().size();
    group.num_joints_ = 0;
    group.link_names_.resize(num_links);
    std::vector<bool> active_joints;
    active_joints.resize(num_kdl_joints_, false);
    for (unsigned int i=0; i < it->second->getJointModelNames().size(); i++)
    {
      std::string joint_name = it->second->getJointModelNames()[i];
      map<string, string>::iterator link_name_it = joint_segment_mapping_.find(joint_name);
      if (link_name_it == joint_segment_mapping_.end())
      {
        ROS_WARN("Joint name %s did not have containing KDL segment.", joint_name.c_str());
        continue;
      }
      std::string link_name = link_name_it->second;
      group.link_names_[i] = link_name;
      const KDL::Segment* segment = &(kdl_tree_.getSegment(link_name)->second.segment);
      KDL::Joint::JointType joint_type =  segment->getJoint().getType();
      if (joint_type != KDL::Joint::None)
      {
        ChompJoint joint;
        joint.chomp_joint_index_ = group.num_joints_;
        joint.kdl_joint_index_ = kdl_tree_.getSegment(link_name)->second.q_nr;
        joint.kdl_joint_ = &(segment->getJoint());
        joint.link_name_ = link_name;
        joint.joint_name_ = segment_joint_mapping_[link_name];
        joint.joint_update_limit_ = joint_update_limit;
        const planning_models::KinematicModel::JointModel* kin_model_joint = models->getKinematicModel()->getJointModel(joint.joint_name_);
        if (const planning_models::KinematicModel::RevoluteJointModel* revolute_joint = dynamic_cast<const planning_models::KinematicModel::RevoluteJointModel*>(kin_model_joint))
        {
          joint.wrap_around_ = revolute_joint->continuous_;
          joint.has_joint_limits_ = !(joint.wrap_around_);
          std::pair<double,double> bounds;
          revolute_joint->getVariableBounds(revolute_joint->getName(), bounds);
          joint.joint_limit_min_ = bounds.first;
          joint.joint_limit_max_ = bounds.second;
          ROS_DEBUG_STREAM("Setting bounds for joint " << revolute_joint->getName() << " to " << joint.joint_limit_min_ << " " << joint.joint_limit_max_);
        }
        else if (const planning_models::KinematicModel::PrismaticJointModel* prismatic_joint = dynamic_cast<const planning_models::KinematicModel::PrismaticJointModel*>(kin_model_joint))
        {
          joint.wrap_around_ = false;
          joint.has_joint_limits_ = true;
          std::pair<double,double> bounds;
          prismatic_joint->getVariableBounds(prismatic_joint->getName(), bounds);
          joint.joint_limit_min_ = bounds.first;
          joint.joint_limit_max_ = bounds.second;
          ROS_DEBUG_STREAM("Setting bounds for joint " << prismatic_joint->getName() << " to " << joint.joint_limit_min_ << " " << joint.joint_limit_max_);
        }
        else
        {
          ROS_WARN("CHOMP cannot handle floating or planar joints yet.");
        }

        group.num_joints_++;
        group.chomp_joints_.push_back(joint);
        active_joints[joint.kdl_joint_index_] = true;
      }

    }
    group.fk_solver_.reset(new KDL::TreeFkSolverJointPosAxisPartial(kdl_tree_, reference_frame_, active_joints));
    planning_groups_.insert(make_pair(it->first, group));
  }

  // test it:
/*  KDL::JntArray q_in(kdl_tree_.getNrOfJoints());
  std::vector<KDL::Frame> segment_frames;
  std::vector<KDL::Vector> joint_axis;
  std::vector<KDL::Vector> joint_pos;

  ros::WallTime start_time = ros::WallTime::now();

  boost::shared_ptr<KDL::TreeFkSolverJointPosAxisPartial> fks = planning_groups_["right_arm"].fk_solver_;
  double q=0.0;
  int n = kdl_tree_.getNrOfJoints();
  for (int i=0; i<100000; i++)
  {
    for (int j=0; j<n; j++)
    {
      q_in(j) += q;
      q+=0.1;
    }
    if (i==0)
      fks->JntToCartFull(q_in, joint_pos, joint_axis, segment_frames);
    else
      fks->JntToCartPartial(q_in, joint_pos, joint_axis, segment_frames);
  }
  ROS_INFO("100000 FK calls in %f wall-seconds.", (ros::WallTime::now() - start_time).toSec());
*/

  ROS_INFO("Initialized chomp robot model in %s reference frame.", reference_frame_.c_str());

  return true;
}

void ChompRobotModel::getLinkInformation(const std::string link_name, std::vector<int>& active_joints, int& segment_number)
{
  ROS_DEBUG_STREAM("Link info for " << link_name);

  // identify the joints that contribute to this link
  active_joints.clear();
  KDL::SegmentMap::const_iterator segment_iter = kdl_tree_.getSegment(link_name);

  // go up the tree until we find the root:
  while (segment_iter != kdl_tree_.getRootSegment())
  {
    KDL::Joint::JointType joint_type =  segment_iter->second.segment.getJoint().getType();
    if (joint_type != KDL::Joint::None)
    {
      active_joints.push_back(segment_iter->second.q_nr);
      ROS_DEBUG_STREAM("Adding parent " << segment_iter->second.segment.getJoint().getName());
    }
    segment_iter = segment_iter->second.parent;
  }
  ROS_DEBUG(" ");

  segment_number = fk_solver_->segmentNameToIndex(link_name);

}

void ChompRobotModel::getActiveJointInformation(const std::string &link_name, 
                                                std::vector<int>& active_joints, 
                                                int& segment_number)
{
  KDL::SegmentMap::const_iterator segment_iter = kdl_tree_.getSegment(link_name);
  // go up the tree until we find the root:
  while (segment_iter != kdl_tree_.getRootSegment())
  {
    KDL::Joint::JointType joint_type =  segment_iter->second.segment.getJoint().getType();
    if (joint_type != KDL::Joint::None)
    {
      active_joints.push_back(segment_iter->second.q_nr);
      ROS_DEBUG_STREAM("Adding parent " << segment_iter->second.segment.getJoint().getName());
    }
    segment_iter = segment_iter->second.parent;
  }
  ROS_DEBUG(" ");
  segment_number = fk_solver_->segmentNameToIndex(link_name);
}

} // namespace chomp
