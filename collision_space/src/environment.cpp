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

bool collision_space::EnvironmentModel::getCollisionContacts(std::vector<Contact> &contacts, unsigned int max_count)
{
  std::vector<AllowedContact> allowed;
  return getCollisionContacts(allowed, contacts, max_count);
}

bool collision_space::EnvironmentModel::getVerbose(void) const
{
  return m_verbose;
}

void collision_space::EnvironmentModel::setVerbose(bool verbose)
{
  m_verbose = verbose;
}

const collision_space::EnvironmentObjects* collision_space::EnvironmentModel::getObjects(void) const
{
  return m_objects;
}

const boost::shared_ptr<const planning_models::KinematicModel>& collision_space::EnvironmentModel::getRobotModel(void) const
{
  return m_robotModel;
}

double collision_space::EnvironmentModel::getRobotScale(void) const
{
  return m_robotScale;
}

double collision_space::EnvironmentModel::getRobotPadding(void) const
{
  return m_robotPadd;
}
	
void collision_space::EnvironmentModel::setRobotModel(const boost::shared_ptr<const planning_models::KinematicModel> &model, 
                                                      const std::vector<std::string> &links, 
                                                      const std::map<std::string, double>& link_padding_map,
                                                      double default_padding,
                                                      double scale) 
{
  m_robotModel = model;
  m_collisionLinks = links;
  m_selfCollisionTest.resize(links.size());
  for (unsigned int i = 0 ; i < links.size() ; ++i)
  {
    m_selfCollisionTest[i].resize(links.size(), true);
    m_collisionLinkIndex[links[i]] = i;
  }
  m_robotScale = scale;
  m_robotPadd = default_padding;
}

void collision_space::EnvironmentModel::addSelfCollisionGroup(const std::vector<std::string>& group1,
                                                              const std::vector<std::string>& group2) 
{
  for(std::vector<std::string>::const_iterator stit1 = group1.begin();
      stit1 != group1.end();
      stit1++) {
    if (m_collisionLinkIndex.find(*stit1) == m_collisionLinkIndex.end())
    {
      ROS_WARN("Unknown link '%s'", (*stit1).c_str());
      continue;
    }
    for(std::vector<std::string>::const_iterator stit2 = group2.begin();
        stit2 != group2.end();
        stit2++) {
      if (m_collisionLinkIndex.find(*stit2) == m_collisionLinkIndex.end())
      {
        ROS_WARN("Unknown link '%s'", (*stit2).c_str());
        continue;
      }
      m_selfCollisionTest[m_collisionLinkIndex[*stit1]][m_collisionLinkIndex[*stit2]] = false;
      m_selfCollisionTest[m_collisionLinkIndex[*stit2]][m_collisionLinkIndex[*stit1]] = false;
    }
  }
}

void collision_space::EnvironmentModel::removeSelfCollisionGroup(const std::vector<std::string>& group1,
                                                                 const std::vector<std::string>& group2) 
{
  for(std::vector<std::string>::const_iterator stit1 = group1.begin();
      stit1 != group1.end();
      stit1++) {
    if (m_collisionLinkIndex.find(*stit1) == m_collisionLinkIndex.end())
    {
      ROS_WARN("Unknown link '%s'", (*stit1).c_str());
      continue;
    }
    for(std::vector<std::string>::const_iterator stit2 = group2.begin();
        stit2 != group2.end();
        stit2++) {
      if (m_collisionLinkIndex.find(*stit2) == m_collisionLinkIndex.end())
      {
        ROS_WARN("Unknown link '%s'", (*stit2).c_str());
        continue;
      }
      m_selfCollisionTest[m_collisionLinkIndex[*stit1]][m_collisionLinkIndex[*stit2]] = true;
      m_selfCollisionTest[m_collisionLinkIndex[*stit2]][m_collisionLinkIndex[*stit1]] = true;
    }
  }
}

void collision_space::EnvironmentModel::lock(void)
{
  m_lock.lock();
}

void collision_space::EnvironmentModel::unlock(void)
{
  m_lock.unlock();
}

void collision_space::EnvironmentModel::setSelfCollision(bool selfCollision)
{
  m_selfCollision = selfCollision;
}

bool collision_space::EnvironmentModel::getSelfCollision(void) const
{
  return m_selfCollision;
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
  for(std::map<std::string, double>::const_iterator it = new_link_padding.begin();
      it != new_link_padding.end();
      it++) {
    altered_link_padding_[it->first] = it->second;
  }
}

void collision_space::EnvironmentModel::revertRobotLinkPadding() {
  altered_link_padding_.clear();
}

double collision_space::EnvironmentModel::getCurrentLinkPadding(std::string name) const {
  if(altered_link_padding_.find(name) != altered_link_padding_.end()) {
    return altered_link_padding_.find(name)->second;
  }
  if(link_padding_map_.find(name) != link_padding_map_.end()) {
    return link_padding_map_.find(name)->second;
  }
  return 0.0;
}
