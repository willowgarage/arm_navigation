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

#include <ros/ros.h>
#include "SearchRequestHandler.h"
#include <ompl_planning/PlannerConfig.h>

using namespace ompl_search;

class OMPLSearching 
{
public:
    
  OMPLSearching(void):nodeHandle_("~")
  {	
    // register with ROS
    collisionModels_ = new planning_environment::CollisionModels("robot_description");
        nodeHandle_.param("distance_metric", distance_metric_, std::string("L2Square"));
    if (nodeHandle_.hasParam("environment_frame_id"))
      {
        std::string frame; nodeHandle_.param("environment_frame_id", frame, std::string(""));
        planningMonitor_ = new planning_environment::PlanningMonitor(collisionModels_, &tf_, frame);
      }
    else
	    planningMonitor_ = new planning_environment::PlanningMonitor(collisionModels_, &tf_);
	
    findValidStateService_ = nodeHandle_.advertiseService("find_valid_state", &OMPLSearching::findValidState, this);
  }
    
  /** Free the memory */
  ~OMPLSearching(void)
  {
    for (std::map<std::string, SearchModel*>::iterator it = models_.begin() ; it != models_.end() ; ++it)
	    delete it->second;
    delete planningMonitor_;
    delete collisionModels_;
  }
    
  void run(void)
  {
    bool execute = false;
    std::vector<std::string> mlist;    

    if (collisionModels_->loadedModels())
    {	    
      //create PlannerConfigMap
      //and load the planner configs
      ompl_planning::PlannerConfigMap plan_config_map(ros::this_node::getName());
      plan_config_map.loadPlannerConfigs();
      
      /* create a model for each group */
      for(std::vector<std::string>::iterator it = plan_config_map.planning_group_names_.begin();
          it != plan_config_map.planning_group_names_.end();
          it++) {
        std::vector< boost::shared_ptr<ompl_planning::PlannerConfig> > cfgs = plan_config_map.getGroupPlannersConfig(*it);
            for (unsigned int i = 0 ; i < cfgs.size() ; ++i)
              {
                if (cfgs[i]->getParamString("type") != "GAIK")
                  continue;
                models_[*it] = new SearchModel(planningMonitor_, *it);
                ompl::kinematic::GAIK *gaik = models_[*it]->gaik;
                if (cfgs[i]->hasParam("max_improve_steps"))
                  {
                    gaik->setMaxImproveSteps(cfgs[i]->getParamInt("max_improve_steps", gaik->getMaxImproveSteps()));
                    ROS_DEBUG("Max improve steps is set to %u", gaik->getMaxImproveSteps());
                  }
		    
                if (cfgs[i]->hasParam("range"))
                  {
                    gaik->setRange(cfgs[i]->getParamDouble("range", gaik->getRange()));
                    ROS_DEBUG("Range is set to %g", gaik->getRange());
                  }
		    
                if (cfgs[i]->hasParam("pool_size"))
                  {
                    gaik->setPoolSize(cfgs[i]->getParamInt("pool_size", gaik->getPoolSize()));
                    ROS_DEBUG("Pool size is set to %u", gaik->getPoolSize());
                  }
		    
                if (cfgs[i]->hasParam("pool_expansion_size"))
                  {
                    gaik->setPoolExpansionSize(cfgs[i]->getParamInt("pool_expansion_size", gaik->getPoolExpansionSize()));
                    ROS_DEBUG("Pool expansion size is set to %u", gaik->getPoolExpansionSize());
                  }
		    
                if (cfgs[i]->hasParam("max_improve_steps"))
                  {
                    gaik->setMaxImproveSteps(cfgs[i]->getParamInt("max_improve_steps", gaik->getMaxImproveSteps()));
                    ROS_DEBUG("Max improve steps is set to %u", gaik->getMaxImproveSteps());
                  }
                mlist.push_back(*it);
              }
          }
	    
        ROS_INFO("Known models:");    
        for (unsigned int i = 0 ; i < mlist.size() ; ++i)
          ROS_INFO("  * %s", mlist[i].c_str());    

        execute = !mlist.empty();
	    
        if (execute)
          ROS_INFO("State search running in frame '%s'", planningMonitor_->getFrameId().c_str());
      }
	
    if (execute)
      {
        bool verbose_collisions;	
        nodeHandle_.param("verbose_collisions", verbose_collisions, false);
        if (verbose_collisions)
          {
            planningMonitor_->getEnvironmentModel()->setVerbose(true);
            ROS_WARN("Verbose collisions is enabled");
          }
        else
          planningMonitor_->getEnvironmentModel()->setVerbose(false);
    
        ros::spin();
      }
    else
	    if (mlist.empty())
        ROS_ERROR("No robot model loaded. OMPL search node cannot start.");
  }

private:

  planning_models::KinematicState* fillStartState(const motion_planning_msgs::RobotState &robot_state)
  {
    motion_planning_msgs::ArmNavigationErrorCodes error_code;
    planning_models::KinematicState *s = new planning_models::KinematicState(planningMonitor_->getKinematicModel());
    if (!planningMonitor_->getTransformListener()->frameExists(robot_state.joint_state.header.frame_id))
      {
        ROS_ERROR("Frame '%s' in starting state is unknown.", robot_state.joint_state.header.frame_id.c_str());
      }
    std::vector<double> tmp;
    tmp.resize(1);

    for(unsigned int i=0; i < robot_state.joint_state.name.size(); i++)
      {
        double tmp_state = robot_state.joint_state.position[i];
        std::string tmp_name = robot_state.joint_state.name[i];
        roslib::Header tmp_header = robot_state.joint_state.header;
        std::string tmp_frame = planningMonitor_->getFrameId();
        if (planningMonitor_->transformJointToFrame(tmp_state, tmp_name, tmp_header, tmp_frame, error_code))
          {
            tmp[0] = tmp_state;
            s->setParamsJoint(tmp, robot_state.joint_state.name[i]);
          }
      }

    if (s->seenAll())
      return s;
    else
      {
        if (planningMonitor_->haveState())
          {
            ROS_INFO("Using the current state to fill in the starting state for the motion plan");
            std::vector<const planning_models::KinematicModel::Joint*> joints;
            planningMonitor_->getKinematicModel()->getJoints(joints);
            for (unsigned int i = 0 ; i < joints.size() ; ++i)
              if (!s->seenJoint(joints[i]->name))
                s->setParamsJoint(planningMonitor_->getRobotState()->getParamsJoint(joints[i]->name), joints[i]->name);
            return s;
          }
      }
    delete s;
    return NULL;
  }
    
  bool findValidState(motion_planning_msgs::ConvertToJointConstraint::Request &req, motion_planning_msgs::ConvertToJointConstraint::Response &res)
  {
    ROS_INFO("Received request for searching a valid state");
    bool st = false;

    planning_models::KinematicState *startState = fillStartState(req.start_state);
    if (startState)
      {
        std::stringstream ss;
        startState->print(ss);
        ROS_DEBUG("Complete starting state:\n%s", ss.str().c_str());
        st = requestHandler_.findState(models_, startState, req, res,distance_metric_);
        delete startState;
      }
    else
	    ROS_ERROR("Starting robot state is unknown. Cannot start search.");
	
    return st;
  }
    
  // ROS interface 
  ros::NodeHandle                        nodeHandle_;
  planning_environment::CollisionModels *collisionModels_;
  planning_environment::PlanningMonitor *planningMonitor_;
  tf::TransformListener                  tf_;
  ros::ServiceServer                     findValidStateService_;    

  // planning data
  std::map<std::string, SearchModel*>    models_;
  SearchRequestHandler                   requestHandler_;
  std::string distance_metric_;
};


int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "ompl_search");

  OMPLSearching search;
  search.run();
    
  return 0;
}
