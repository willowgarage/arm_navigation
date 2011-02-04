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

/** \author Ioan Sucan */

#include "collision_space/environment.h"
#include <ros/console.h>

bool collision_space::EnvironmentModel::getCollisionContacts(std::vector<Contact> &contacts, unsigned int max_count) const
{
  std::vector<AllowedContact> allowed;
  return getCollisionContacts(allowed, contacts, max_count);
}

bool collision_space::EnvironmentModel::getVerbose(void) const
{
  return verbose_;
}

void collision_space::EnvironmentModel::setVerbose(bool verbose)
{
  verbose_ = verbose;
}

const collision_space::EnvironmentObjects* collision_space::EnvironmentModel::getObjects(void) const
{
  return objects_;
}

const planning_models::KinematicModel* collision_space::EnvironmentModel::getRobotModel(void) const
{
  return robot_model_;
}

double collision_space::EnvironmentModel::getRobotScale(void) const
{
  return robot_scale_;
}

double collision_space::EnvironmentModel::getRobotPadding(void) const
{
  return default_robot_padding_;
}
	
void collision_space::EnvironmentModel::setRobotModel(const planning_models::KinematicModel* model, 
                                                      const std::vector<std::string> &links, 
                                                      const std::map<std::string, double>& link_padding_map,
                                                      double default_padding,
                                                      double scale) 
{
  robot_model_ = model;
  collision_links_ = links;
  self_collision_test_.clear();
  collision_link_index_.clear();
  self_collision_test_.resize(links.size());
  //default is that no collisions are allowed
  for (unsigned int i = 0 ; i < links.size() ; ++i)
  {
    self_collision_test_[i].resize(links.size(), false);
    collision_link_index_[links[i]] = i;
  }
  robot_scale_ = scale;
  default_robot_padding_ = default_padding;
}

void collision_space::EnvironmentModel::setAllCollisionPairs(const std::vector<std::string>& group1,
                                                             const std::vector<std::string>& group2,
                                                             bool disable) 
{
  for(std::vector<std::string>::const_iterator stit1 = group1.begin();
      stit1 != group1.end();
      stit1++) {
    if (collision_link_index_.find(*stit1) == collision_link_index_.end())
    {
      //ROS_WARN("Unknown link '%s'", (*stit1).c_str());
      continue;
    }
    for(std::vector<std::string>::const_iterator stit2 = group2.begin();
        stit2 != group2.end();
        stit2++) {
      if (collision_link_index_.find(*stit2) == collision_link_index_.end())
      {
        //ROS_WARN("Unknown link '%s'", (*stit2).c_str());
        continue;
      }
      self_collision_test_[collision_link_index_[*stit1]][collision_link_index_[*stit2]] = disable;
      self_collision_test_[collision_link_index_[*stit2]][collision_link_index_[*stit1]] = disable;
    }
  }
}

void collision_space::EnvironmentModel::lock(void)
{
  lock_.lock();
}

void collision_space::EnvironmentModel::unlock(void)
{
  lock_.unlock();
}

void collision_space::EnvironmentModel::setCheckSelfCollision(bool check_self_collision)
{
  check_self_collision = check_self_collision;
}

bool collision_space::EnvironmentModel::getCheckSelfCollision(void) const
{
  return check_self_collision_;
}

void collision_space::EnvironmentModel::setAllowedCollisionMatrix(const std::vector<std::vector<bool> > &matrix,
                                                                  const std::map<std::string, unsigned int> &ind){
  use_set_collision_matrix_ = true;
  set_collision_matrix_ = matrix;
  set_collision_ind_ = ind;
}

void collision_space::EnvironmentModel::revertAllowedCollisionMatrix() {
  use_set_collision_matrix_ = false;
  set_collision_matrix_.clear();
  set_collision_ind_.clear();
}

void collision_space::EnvironmentModel::getCurrentAllowedCollisionMatrix(std::vector<std::vector<bool> > &matrix,
                                                                         std::map<std::string, unsigned int> &ind) const {
  if(use_set_collision_matrix_) {
    matrix = set_collision_matrix_;
    ind = set_collision_ind_;
  } else {
    getDefaultAllowedCollisionMatrix(matrix, ind);
  }
}

void collision_space::EnvironmentModel::setRobotLinkPadding(const std::map<std::string, double>& new_link_padding) {
  current_link_padding_map_ = default_link_padding_map_;
  for(std::map<std::string, double>::const_iterator it = new_link_padding.begin();
      it != new_link_padding.end();
      it++) {
    if(current_link_padding_map_.find(it->first) == current_link_padding_map_.end()) {
      //don't have this currently
      continue;
    }
    //only putting in altered padding if it's different
    if(current_link_padding_map_.find(it->first)->second != it->second) {
      altered_link_padding_map_[it->first] = it->second;
      current_link_padding_map_[it->first] = it->second;
    }
  }
}

void collision_space::EnvironmentModel::revertRobotLinkPadding() {
  altered_link_padding_map_.clear();
  current_link_padding_map_ = default_link_padding_map_;
}

const std::map<std::string, double>& collision_space::EnvironmentModel::getCurrentLinkPaddingMap() const {
  return current_link_padding_map_;
}

double collision_space::EnvironmentModel::getCurrentLinkPadding(std::string name) const {
  if(current_link_padding_map_.find(name) != current_link_padding_map_.end()) {
    return current_link_padding_map_.find(name)->second;
  }
  return 0.0;
}
