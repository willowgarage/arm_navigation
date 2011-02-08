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

collision_space::EnvironmentModel::AllowedCollisionMatrix::AllowedCollisionMatrix(const std::vector<std::string>& names,
                                                                                  bool allowed) 
{
  unsigned int ns = names.size();
  allowed_entries_.resize(ns);
  for(unsigned int i = 0; i < ns; i++) {
    allowed_entries_[i].resize(ns,allowed);
    allowed_entries_bimap_.insert(entry_type::value_type(names[i], i));
  }
}

collision_space::EnvironmentModel::AllowedCollisionMatrix::AllowedCollisionMatrix(const std::vector<std::vector<bool> >& all_coll_vectors,
                                                                                  const std::map<std::string, unsigned int>& all_coll_indices)
{
  unsigned int num_outer = all_coll_vectors.size();
  valid_ = true;
  if(all_coll_indices.size() != all_coll_vectors.size()) {
    valid_ = false;
    ROS_WARN_STREAM("Indices size " << all_coll_indices.size() << " not equal to num vecs " << all_coll_vectors.size());
    return;
  }
  for(std::map<std::string, unsigned int>::const_iterator it = all_coll_indices.begin();
      it != all_coll_indices.end();
      it++) {
    allowed_entries_bimap_.insert(entry_type::value_type(it->first, it->second));
  }
  
  if(allowed_entries_bimap_.left.size() != all_coll_indices.size()) {
    valid_ = false;
    ROS_WARN_STREAM("Some strings or values in allowed collision matrix are repeated");
  }
  if(allowed_entries_bimap_.right.begin()->first != 0) {
    valid_ = false;
    ROS_WARN_STREAM("No entry with index 0 in map");
  }
  if(allowed_entries_bimap_.right.rbegin()->first != num_outer-1) {
    valid_ = false;
    ROS_WARN_STREAM("Last index should be " << num_outer << " but instead is " << allowed_entries_bimap_.right.rbegin()->first);
  }
  
  for(unsigned int i = 0; i < num_outer; i++) {
    if(num_outer != all_coll_vectors[i].size()) {
      valid_ = false;
      ROS_WARN_STREAM("Entries size for " << allowed_entries_bimap_.right.at(i) << " is " << all_coll_vectors[i].size() << " instead of " << num_outer);
    }
  }
}

collision_space::EnvironmentModel::AllowedCollisionMatrix::AllowedCollisionMatrix(const AllowedCollisionMatrix& acm)
{
  valid_ = acm.valid_;
  allowed_entries_ = acm.allowed_entries_;
  allowed_entries_bimap_ = acm.allowed_entries_bimap_;
}

bool collision_space::EnvironmentModel::AllowedCollisionMatrix::getAllowedCollision(const std::string& name1, 
                                                                                    const std::string& name2,
                                                                                    bool& allowed_collision) const
{
  entry_type::left_const_iterator it1 = allowed_entries_bimap_.left.find(name1);
  if(it1 == allowed_entries_bimap_.left.end()) {
    return false;
  }
  entry_type::left_const_iterator it2 = allowed_entries_bimap_.left.find(name2);
  if(it2 == allowed_entries_bimap_.left.end()) {
    return false;
  }
  allowed_collision = allowed_entries_[it1->second][it2->second];
  return true;
}

bool collision_space::EnvironmentModel::AllowedCollisionMatrix::getAllowedCollision(unsigned int i, unsigned int j,
                                                                                    bool& allowed_collision) const
{
  if(i > allowed_entries_.size() || j > allowed_entries_[i].size()) {
    return false;
  }
  allowed_collision = allowed_entries_[i][j];
  return true;
}

bool collision_space::EnvironmentModel::AllowedCollisionMatrix::hasEntry(const std::string& name) const
{
  return(allowed_entries_bimap_.left.find(name) != allowed_entries_bimap_.left.end());
}

bool collision_space::EnvironmentModel::AllowedCollisionMatrix::getEntryIndex(const std::string& name,
                                                                              unsigned int& index) const
{
  entry_type::left_const_iterator it1 = allowed_entries_bimap_.left.find(name);
  if(it1 == allowed_entries_bimap_.left.end()) {
    return false;
  }
  index = it1->second;
  return true;
}

bool collision_space::EnvironmentModel::AllowedCollisionMatrix::getEntryName(const unsigned int ind,
                                                                             std::string& name) const
{
  entry_type::right_const_iterator it1 = allowed_entries_bimap_.right.find(ind);
  if(it1 == allowed_entries_bimap_.right.end()) {
    return false;
  }
  name = it1->second;
  return true;
}

bool collision_space::EnvironmentModel::AllowedCollisionMatrix::removeEntry(const std::string& name) {
  if(allowed_entries_bimap_.left.find(name) == allowed_entries_bimap_.left.end()) {
    return false;
  }
  unsigned int ind = allowed_entries_bimap_.left.find(name)->second;
  allowed_entries_.erase(allowed_entries_.begin()+ind);
  for(unsigned int i = 0; i < allowed_entries_.size(); i++) {
    allowed_entries_[i].erase(allowed_entries_[i].begin()+ind);
  }
  allowed_entries_bimap_.left.erase(name);
  return true;
}

bool collision_space::EnvironmentModel::AllowedCollisionMatrix::addEntry(const std::string& name,
                                                                         bool allowed)
{
  if(allowed_entries_bimap_.left.find(name) != allowed_entries_bimap_.left.end()) {
    return false;
  }
  unsigned int ind = allowed_entries_.size();
  allowed_entries_bimap_.insert(entry_type::value_type(name,ind));
  std::vector<bool> new_entry(ind+1, allowed);
  allowed_entries_.resize(ind+1);
  allowed_entries_[ind] = new_entry;
  for(unsigned int i = 0; i < ind; i++) {
    allowed_entries_[i].resize(ind+1);
    allowed_entries_[i][ind] = allowed;
  }
  return true;
}

bool collision_space::EnvironmentModel::AllowedCollisionMatrix::changeEntry(const std::string& name, 
                                                                            bool allowed)
{
  if(allowed_entries_bimap_.left.find(name) == allowed_entries_bimap_.left.end()) {
    return false;
  }
  unsigned int ind = allowed_entries_bimap_.left.find(name)->second;
  for(unsigned int i = 0; i < allowed_entries_.size(); i++) {
    allowed_entries_[i][ind] = allowed;
    allowed_entries_[ind][i] = allowed;
  }
  return true;
}

bool collision_space::EnvironmentModel::AllowedCollisionMatrix::changeEntry(const std::string& name, 
                                                                            const std::vector<std::string>& change_names,
                                                                            bool allowed)
{
  if(allowed_entries_bimap_.left.find(name) == allowed_entries_bimap_.left.end()) {
    return false;
  }
  unsigned int ind_1 = allowed_entries_bimap_.left.find(name)->second;
  for(unsigned int i = 0; i < change_names.size(); i++) {
    if(allowed_entries_bimap_.left.find(change_names[i]) == allowed_entries_bimap_.left.end()) {
      return false;
    }
    unsigned int ind_2 = allowed_entries_bimap_.left.find(change_names[i])->second;
    allowed_entries_[ind_1][ind_2] = allowed;
    allowed_entries_[ind_2][ind_1] = allowed;
  }
  return true;
}

bool collision_space::EnvironmentModel::AllowedCollisionMatrix::changeEntry(const std::vector<std::string>& change_names_1,
                                                                            const std::vector<std::string>& change_names_2,
                                                                            bool allowed)
{
  for(unsigned int i = 0; i < change_names_1.size(); i++) {
    if(!changeEntry(change_names_1[i], change_names_2, allowed)) {
      return false;
    }
  }
  return true;
}

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
                                                      const AllowedCollisionMatrix& acm,
                                                      const std::map<std::string, double>& link_padding_map,
                                                      double default_padding,
                                                      double scale) 
{
  robot_model_ = model;
  default_collision_matrix_ = acm;
  robot_scale_ = scale;
  default_robot_padding_ = default_padding;
  default_link_padding_map_ = link_padding_map;
}

void collision_space::EnvironmentModel::lock(void)
{
  lock_.lock();
}

void collision_space::EnvironmentModel::unlock(void)
{
  lock_.unlock();
}

const collision_space::EnvironmentModel::AllowedCollisionMatrix& 
collision_space::EnvironmentModel::getDefaultAllowedCollisionMatrix() const
{
  return default_collision_matrix_;
}

const collision_space::EnvironmentModel::AllowedCollisionMatrix& 
collision_space::EnvironmentModel::getCurrentAllowedCollisionMatrix() const {
  if(use_altered_collision_matrix_) {
    return altered_collision_matrix_;
  }
  return default_collision_matrix_;
}

void collision_space::EnvironmentModel::setAlteredCollisionMatrix(const AllowedCollisionMatrix& acm) {
  use_altered_collision_matrix_ = true;
  altered_collision_matrix_ = acm;
}

void collision_space::EnvironmentModel::revertAlteredCollisionMatrix() {
  use_altered_collision_matrix_ = false;
}

void collision_space::EnvironmentModel::setAlteredLinkPadding(const std::map<std::string, double>& new_link_padding) {
  altered_link_padding_map_.clear();
  for(std::map<std::string, double>::const_iterator it = new_link_padding.begin();
      it != new_link_padding.end();
      it++) {
    if(default_link_padding_map_.find(it->first) == default_link_padding_map_.end()) {
      //don't have this currently
      continue;
    }
    //only putting in altered padding if it's different
    if(default_link_padding_map_.find(it->first)->second != it->second) {
      altered_link_padding_map_[it->first] = it->second;
    }
  }
  use_altered_link_padding_map_ = true;
}

void collision_space::EnvironmentModel::revertAlteredLinkPadding() {
  altered_link_padding_map_.clear();
  use_altered_link_padding_map_ = false;
}

std::map<std::string, double> collision_space::EnvironmentModel::getCurrentLinkPaddingMap() const {
  std::map<std::string, double> ret_map = default_link_padding_map_;
  for(std::map<std::string, double>::const_iterator it = altered_link_padding_map_.begin();
      it != altered_link_padding_map_.end();
      it++) {
    ret_map[it->first] = it->second;
  }
  return ret_map;
}

double collision_space::EnvironmentModel::getCurrentLinkPadding(std::string name) const {
  if(altered_link_padding_map_.find(name) != altered_link_padding_map_.end()) {
    return altered_link_padding_map_.find(name)->second;
  } else if(default_link_padding_map_.find(name) != default_link_padding_map_.end()) {
    return default_link_padding_map_.find(name)->second;
  }
  return 0.0;
}
