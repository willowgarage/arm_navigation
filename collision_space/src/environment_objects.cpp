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

#include "collision_space/environment_objects.h"
#include <geometric_shapes/shape_operations.h>

std::vector<std::string> collision_space::EnvironmentObjects::getNamespaces(void) const
{
    std::vector<std::string> ns;
    for (std::map<std::string, NamespaceObjects>::const_iterator it = m_objects.begin() ; it != m_objects.end() ; ++it)
	ns.push_back(it->first);
    return ns;
}

const collision_space::EnvironmentObjects::NamespaceObjects& collision_space::EnvironmentObjects::getObjects(const std::string &ns) const
{
    std::map<std::string, NamespaceObjects>::const_iterator it = m_objects.find(ns);
    if (it == m_objects.end())
	return m_empty;
    else
	return it->second;
}

collision_space::EnvironmentObjects::NamespaceObjects& collision_space::EnvironmentObjects::getObjects(const std::string &ns)
{
    return m_objects[ns];
}

void collision_space::EnvironmentObjects::addObject(const std::string &ns, shapes::StaticShape *shape)
{
    m_objects[ns].staticShape.push_back(shape);
}

void collision_space::EnvironmentObjects::addObject(const std::string &ns, shapes::Shape *shape, const btTransform &pose)
{
    m_objects[ns].shape.push_back(shape);
    m_objects[ns].shapePose.push_back(pose);
}

bool collision_space::EnvironmentObjects::removeObject(const std::string &ns, const shapes::Shape *shape)
{
    std::map<std::string, NamespaceObjects>::iterator it = m_objects.find(ns);
    if (it != m_objects.end())
    { 
	unsigned int n = it->second.shape.size();
	for (unsigned int i = 0 ; i < n ; ++i)
	    if (it->second.shape[i] == shape)
	    {
		it->second.shape.erase(it->second.shape.begin() + i);
		it->second.shapePose.erase(it->second.shapePose.begin() + i);
		return true;
	    }
    }
    return false;
}

bool collision_space::EnvironmentObjects::removeObject(const std::string &ns, const shapes::StaticShape *shape)
{
    std::map<std::string, NamespaceObjects>::iterator it = m_objects.find(ns);
    if (it != m_objects.end())
    { 
	unsigned int n = it->second.staticShape.size();
	for (unsigned int i = 0 ; i < n ; ++i)
	    if (it->second.staticShape[i] == shape)
	    {
		it->second.staticShape.erase(it->second.staticShape.begin() + i);
		return true;
	    }
    }
    return false;
}

void collision_space::EnvironmentObjects::clearObjects(const std::string &ns)
{
    std::map<std::string, NamespaceObjects>::iterator it = m_objects.find(ns);
    if (it != m_objects.end())
    {
	unsigned int n = it->second.staticShape.size();
	for (unsigned int i = 0 ; i < n ; ++i)
	    delete it->second.staticShape[i];
	n = it->second.shape.size();
	for (unsigned int i = 0 ; i < n ; ++i)
	    delete it->second.shape[i];
	m_objects.erase(it);
    }
}

void collision_space::EnvironmentObjects::clearObjects(void)
{
    std::vector<std::string> ns = getNamespaces();
    for (unsigned int i = 0 ; i < ns.size() ; ++i)
	clearObjects(ns[i]);
}

void collision_space::EnvironmentObjects::addObjectNamespace(const std::string ns)
{
  if(m_objects.find(ns) == m_objects.end()) {
    m_objects[ns] = NamespaceObjects();
  }
  //doesn't do anything if the object is already in m_objects
}

collision_space::EnvironmentObjects* collision_space::EnvironmentObjects::clone(void) const
{
    EnvironmentObjects *c = new EnvironmentObjects();
    for (std::map<std::string, NamespaceObjects>::const_iterator it = m_objects.begin() ; it != m_objects.end() ; ++it)
    {
	NamespaceObjects &ns = c->m_objects[it->first];
	unsigned int n = it->second.staticShape.size();
	for (unsigned int i = 0 ; i < n ; ++i)
	    ns.staticShape.push_back(shapes::cloneShape(it->second.staticShape[i]));
	n = it->second.shape.size();
	for (unsigned int i = 0 ; i < n ; ++i)
	    ns.shape.push_back(shapes::cloneShape(it->second.shape[i]));
	ns.shapePose = it->second.shapePose;
    }
    return c;
}
