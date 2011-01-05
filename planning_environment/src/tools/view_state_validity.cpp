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

@b ViewStateValidity is a node capable of verifying if the current
state of the robot is valid or not.

**/

#include "planning_environment/monitors/planning_monitor.h"
#include <std_msgs/Byte.h>

#include <iostream>
#include <sstream>
#include <string>
#include <map>

class ViewStateValidity
{
public:

  ViewStateValidity(void) : last_(-1)
  {
    collisionModels_ = new planning_environment::CollisionModels("robot_description");
    if (collisionModels_->loadedModels())
    {	
	    stateValidityPublisher_ = nh_.advertise<std_msgs::Byte>("state_validity", 1);
	    planningMonitor_ = new planning_environment::PlanningMonitor(collisionModels_, &tf_);
	    planningMonitor_->getEnvironmentModel()->setVerbose(true);
	    planningMonitor_->setOnAfterMapUpdateCallback(boost::bind(&ViewStateValidity::afterWorldUpdate, this, _1, _2));
	    planningMonitor_->setOnStateUpdateCallback(boost::bind(&ViewStateValidity::stateUpdate, this));
    }
  }

  virtual ~ViewStateValidity(void)
  {
    if (planningMonitor_)
	    delete planningMonitor_;
    if (collisionModels_)
	    delete collisionModels_;
  }

protected:

  void afterWorldUpdate(const mapping_msgs::CollisionMapConstPtr &collisionMap, bool clear)
  {
    last_ = -1;
  }
    
  void stateUpdate(void)
  {	
    motion_planning_msgs::ArmNavigationErrorCodes error_code;
    motion_planning_msgs::RobotState empty_state;
    bool valid = planningMonitor_->isStateValid(empty_state, planning_environment::PlanningMonitor::COLLISION_TEST,false,error_code);
    std_msgs::Byte msg;
    msg.data = valid ? 1 : 0;
    if (last_ != msg.data)
    {
	    last_ = msg.data;
	    stateValidityPublisher_.publish(msg);
	    if (valid && error_code.val == error_code.SUCCESS)
        ROS_INFO("State is valid");
	    else
        ROS_WARN("State is in collision");
    }
  }
    
private:

  int                                    last_;
  ros::NodeHandle                        nh_;
  ros::Publisher                         stateValidityPublisher_;
  tf::TransformListener                  tf_;
  planning_environment::CollisionModels *collisionModels_;
  planning_environment::PlanningMonitor *planningMonitor_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "view_state_validity");

  ViewStateValidity validator;
  ros::spin();
    
  return 0;
}
