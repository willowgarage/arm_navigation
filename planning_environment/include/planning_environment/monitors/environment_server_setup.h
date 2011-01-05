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
 * \author Sachin Chitta, Ioan Sucan
 *********************************************************************/

#ifndef ENVIRONMENT_SERVER_SETUP_
#define ENVIRONMENT_SERVER_SETUP_

#include <ros/ros.h>
#include <planning_environment/monitors/planning_monitor.h>

namespace planning_environment
{
/** \brief Configuration of actions that need to actuate parts of the robot */
class EnvironmentServerSetup
{
  friend class EnvironmentServer;

public:

	EnvironmentServerSetup(void): node_handle_("~")
	{
    collision_models_ = NULL;
    planning_monitor_ = NULL;
	}

	virtual ~EnvironmentServerSetup(void)
	{
    if (planning_monitor_)
      delete planning_monitor_;
    if (collision_models_)
      delete collision_models_;
	}
	bool configure(void);

protected:
	ros::NodeHandle                        node_handle_, root_handle_;
	tf::TransformListener                  tf_;
	planning_environment::CollisionModels *collision_models_;
	planning_environment::PlanningMonitor *planning_monitor_;
	std::string                            group_;

private:
  bool use_collision_map_;

};
}

#endif
