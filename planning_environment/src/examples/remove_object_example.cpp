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

/**

@b RemoveObjectExample is a node that forwards a collision map after it removes a box from it

note: remember to remap "collision_map" to the desired collision map to be used by the planning environment. To run this, launch "launch/remove_object_example.launch"

**/

#include "planning_environment/monitors/planning_monitor.h"
#include <arm_navigation_msgs/CollisionMap.h>

#include <iostream>
#include <sstream>
#include <string>
#include <map>

class RemoveObjectExample
{
public:

    RemoveObjectExample(void) 
    {
	collisionModels_ = new planning_environment::CollisionModels("robot_description");
	if (collisionModels_->loadedModels())
	{	
	    collisionMapPublisher_ = nh_.advertise<arm_navigation_msgs::CollisionMap>("collision_map_with_removed_box", 1);
	    planningMonitor_ = new planning_environment::PlanningMonitor(collisionModels_, &tf_);
	    planningMonitor_->setOnAfterMapUpdateCallback(boost::bind(&RemoveObjectExample::afterWorldUpdate, this, _1, _2));
	}
    }

    virtual ~RemoveObjectExample(void)
    {
        if (planningMonitor_)
	    delete planningMonitor_;
	if (collisionModels_)
	    delete collisionModels_;
    }

protected:

    void afterWorldUpdate(const arm_navigation_msgs::CollisionMapConstPtr &collisionMap, bool clear)
    {
	// we do not care about incremental updates, only re-writes of the map
	if (!clear)
	    return;
	
	// at this point, the environment model has the collision map inside it
	
	
	// get exclusive access
	planningMonitor_->getEnvironmentModel()->lock();
	
	// get a copy of our own, to play with :)
	collision_space::EnvironmentModel *env = planningMonitor_->getEnvironmentModel()->clone();
	
	// release our hold
	planningMonitor_->getEnvironmentModel()->unlock();

	
	// create a box
	shapes::Shape *box = new shapes::Box(2, 2, 2);
	tf::Transform pose;
	pose.setIdentity();
	
	// remove the objects colliding with the box
	env->removeCollidingObjects(box, pose);	
	
	// forward the updated map
	arm_navigation_msgs::CollisionMap cmap;
	planningMonitor_->recoverCollisionMap(env, cmap);
	collisionMapPublisher_.publish(cmap);

	// throw away our copy
	delete env;
	
	ROS_INFO("Received collision map with %d points and published one with %d points", 
		 (int)collisionMap->boxes.size(), (int)cmap.boxes.size());
	
    }

private:

    ros::NodeHandle                        nh_;
    tf::TransformListener                  tf_;
    
    ros::Publisher                         collisionMapPublisher_;
    planning_environment::CollisionModels *collisionModels_;
    planning_environment::PlanningMonitor *planningMonitor_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "remove_object_example");

    RemoveObjectExample example;
    ros::spin();
    
    return 0;
}
