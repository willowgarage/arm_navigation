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

#include "DisplayPlanner.h"

void ompl_planning::DisplayPlanner::enableDisplay(int idx1, int idx2, int idx3)
{
    px_ = idx1;
    py_ = idx2;
    pz_ = idx3;
    displayPublisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
}

void ompl_planning::DisplayPlanner::disableDisplay(void)
{
    displayPublisher_.shutdown();
}

void ompl_planning::DisplayPlanner::display(PlannerSetup *psetup)
{
    int dim = psetup->ompl_model->si->getStateDimension();
    if (px_ >= dim || py_ >= dim || pz_ >= dim)
    {
	ROS_WARN("Display projection out of bounds. Not publishing markers");
	return;
    }
    
    std::vector<const ompl::base::State*> states;
    psetup->mp->getStates(states);
    
    if (states.empty())
	return;
    
    visualization_msgs::Marker mk;        
    mk.header.stamp = psetup->ompl_model->planningMonitor->lastJointStateUpdate();
    mk.header.frame_id = psetup->ompl_model->planningMonitor->getWorldFrameId();
    mk.ns = ros::this_node::getName();
    mk.id = 1;    
    mk.type = visualization_msgs::Marker::POINTS;
    mk.action = visualization_msgs::Marker::ADD;
    mk.lifetime = ros::Duration(30);
    mk.color.a = 1.0;
    mk.color.r = 1.0;
    mk.color.g = 0.04;
    mk.color.b = 0.04;
    mk.pose.position.x = 0;
    mk.pose.position.y = 0;
    mk.pose.position.z = 0;
    mk.pose.orientation.x = 0;
    mk.pose.orientation.y = 0;
    mk.pose.orientation.z = 0;
    mk.pose.orientation.w = 1;
    mk.scale.x = 0.01;
    mk.scale.y = 0.01;
    mk.scale.z = 0.01;
    mk.points.resize(states.size());
    
    for (unsigned int i = 0 ; i < states.size() ; ++i)
    {
	mk.points[i].x = px_ >= 0 ? states[i]->values[px_] : 0.0;
	mk.points[i].y = py_ >= 0 ? states[i]->values[py_] : 0.0;
	mk.points[i].z = pz_ >= 0 ? states[i]->values[pz_] : 0.0;
    }
    
    displayPublisher_.publish(mk);
    ROS_INFO("Published %d points in the diffusion tree", (int)states.size());
}
