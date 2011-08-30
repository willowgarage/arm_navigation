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
#ifndef KINEMATICS_MSGS_HELPER_FUNCTIONS_
#define KINEMATICS_MSGS_HELPER_FUNCTIONS_

#include <set>
#include <vector>
#include <string>

namespace kinematics_msgs
{
/**
 *@brief Get a list of exclusive links for which collision checks are to be enabled. There are four cases:
 *(a) If both collision_enable_links and collision_disable_links are empty, result = default_links
 *(b) If collision_enable_links is non-empty and collision_disable_links is empty, result = collision_enable_links
 *(c) If collision_enable_links is empty and collision_disable_links is non-empty, result = default_links - collision_disable_links
 *(d) If collision_enable_links is non-empty and collision_disable_links is non-empty, result = Union(default_links, collision_enable_links) - disable_links
 *@param The default set of links
 *@param The set of links for which collisions are to be enabled
 *@param The set of links for which collisions are to be disabled
 *@param The result
*/
void getCollisionLinks(const std::vector<std::string> &default_links, const std::vector<std::string> &collision_enable_links, const std::vector<std::string> &collision_disable_links, std::vector<std::string> &result)
{
  if(collision_enable_links.empty() && collision_disable_links.empty())
    result = default_links;
  else if(!collision_enable_links.empty() && collision_disable_links.empty())
    result = collision_enable_links;
  else if(collision_enable_links.empty() && !collision_disable_links.empty())
  {
    std::set<std::string> tmp_set;
    for(unsigned int i=0; i < default_links.size(); i++)
      tmp_set.insert(default_links[i]);
    for(unsigned int i=0; i < collision_disable_links.size(); i++)
      tmp_set.erase(collision_disable_links[i]);

    for(std::set<std::string>::iterator set_iterator = tmp_set.begin(); set_iterator != tmp_set.end(); set_iterator++)
    {
       result.push_back(*set_iterator);
    }     
  }
  else 
  {
    std::set<std::string> tmp_set;
    for(unsigned int i=0; i < default_links.size(); i++)
      tmp_set.insert(default_links[i]);
    for(unsigned int i=0; i < collision_enable_links.size(); i++)
      tmp_set.insert(collision_enable_links[i]);
    for(unsigned int i=0; i < collision_disable_links.size(); i++)
      tmp_set.erase(collision_disable_links[i]);

    for(std::set<std::string>::iterator set_iterator = tmp_set.begin(); set_iterator != tmp_set.end(); set_iterator++)
    {
       result.push_back(*set_iterator);
    }
  }
}

}

#endif
