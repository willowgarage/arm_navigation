/*********************************************************************
 *
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
 *
 *  \author E. Gil Jones
 *********************************************************************/

#include <ros/ros.h>

class CollisionOperationsGenerator {

  CollisionOperationsGenerator(const std::string& full_path_name) {
    
    urdf_ok_ = urdf_model_.initFile(full_path_name);

    if(!urdf_ok_) ROS_WARN_STREAM("Urdf file " << full_path_name << " not ok");

    std::vector<planning_models::KinematicModel::GroupConfig> gcs;
    std::vector<planning_models::KinematicModel::MultiDofConfig> multi_dof_configs;
    const urdf::Link *root = urdf_model_.getRoot().get();

    //now this should work with an non-identity transform
    planning_models::KinematicModel::MultiDofConfig config("world_joint");
    config.type = "Floating";
    config.parent_frame_id = root->name_;
    config.child_frame_id = root->name_;
    multi_dof_configs.push_back(config);
    
    kin_model_ = new planning_models::KinematicModel(urdf_model_,gcs, multi_dof_configs);    
    if(kin_model_->getRoot() == NULL) {
      ROS_INFO_STREAM("Kinematic root is NULL");
      return;
    }
  }

  void generateSamplingStructure() {
    const std::vector<planning_models::KinematicModel::JointModel*>& jmv = kin_model_->getJointModels();
    //assuming that 0th is world joint, which we don't want to include
    for(unsigned int i = 1; i < jmv.size(); i++) {
      const std::map<std::string, std::pair<double, double> >& joint_bounds = jmv[i]->getAllVariableBounds();
      for(std::map<std::string, std::pair<double, double> >::const_iterator it = joint_bounds.begin();
          it != joint_bounds.end();
          it++) {
        if(joint_bounds_map_.find(it->first) != joint_bounds_map_.end()) {
          ROS_WARN_STREAM("Have repeat DOF names for " << it->first);
          continue;
        }
        if(it->second.first > it->second.second) {
          ROS_WARN_STREAM("Lower bound for DOF " << it->first << " is greater than upper bound");
        } else if(it->second.first == -DBL_MAX) {
          ROS_WARN_STREAM("Some non-root DOF " << it->first << " has negative inf lower bound");
        } 
        if(it->second.second == DBL_MAX) {
          ROS_WARN_STREAM("Some non-root DOF " << it->first << " has inf upper[ bound");
        } 
        joint_bounds_map_[it->first] = it->second;        
      }
    }
    const std::vector<planning_models::KinematicModel::LinkModel*>& lmv = kin_model_->getLinkModels();
    //assuming that 0th is world joint, which we don't want to include
    std::map<std::string, unsigned int> all_link_zero;
    for(unsigned int i = 0; i < lmv.size(); i++) {
      all_link_zero[lmv[i]->getName()] = 0;
    }
    for(unsigned int i = 0; i < lmv.size(); i++) {
      collision_count_map_[lmv[i]->getName()] = all_link_zero;
    }
  }

protected:

  std::map<std::string, std::pair<double, double> > joint_bounds_map_;
  std::map<std::string, std::map<std::string, unsigned int> > collision_count_map_;

};
