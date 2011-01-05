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

#ifndef PLANNING_ENVIRONMENT_MODELS_COLLISION_MODELS_
#define PLANNING_ENVIRONMENT_MODELS_COLLISION_MODELS_

#include "planning_environment/models/robot_models.h"
#include <collision_space/environment.h>
#include <motion_planning_msgs/OrderedCollisionOperations.h>

namespace planning_environment
{

    /** \brief A class capable of loading a robot model from the parameter server */
    
    class CollisionModels : public RobotModels
    {
    public:
	
	CollisionModels(const std::string &description, double scale, double padd) : RobotModels(description), scale_(scale), padd_(padd)
	{
	    loadCollision(group_link_union_);
	}

	CollisionModels(const std::string &description, const std::vector<std::string> &links, double scale, double padd) : RobotModels(description), scale_(scale), padd_(padd)
	{
	    loadCollision(links);
	}
	
	CollisionModels(const std::string &description) : RobotModels(description)
	{
	    loadParams();
	    loadCollision(group_link_union_);
	}

	CollisionModels(const std::string &description, const std::vector<std::string> &links) : RobotModels(description)
	{
	    loadParams();
	    loadCollision(links);
	}
	
	virtual ~CollisionModels(void)
	{
	}

	/** \brief Reload the robot description and recreate the model */	
	virtual void reload(void)
	{
	    RobotModels::reload();
	    ode_collision_model_.reset();
	    //bullet_collision_model_.reset();
	    loadCollision(group_link_union_);
	}
	
	/** \brief Return the instance of the constructed ODE collision model */
	const boost::shared_ptr<collision_space::EnvironmentModel> &getODECollisionModel(void) const
	{
	    return ode_collision_model_;
	}

	/** \brief Return the instance of the constructed Bullet collision model */
	// const boost::shared_ptr<collision_space::EnvironmentModel> &getBulletCollisionModel(void) const
	// {
	//     return bullet_collision_model_;
	// }

	/** \brief Get the scaling to be used for the robot parts when inserted in the collision space */
	double getScale(void)
	{
	    return scale_;
	}
	
	/** \brief Get the padding to be used for the robot parts when inserted in the collision space */
	double getPadding(void)
	{
	    return padd_;
	}

	void getDefaultOrderedCollisionOperations(std::vector<motion_planning_msgs::CollisionOperation> &self_collision)
	{
          self_collision = default_collision_operations_;
	}
	
    protected:
	
	void loadParams();
	void loadCollision(const std::vector<std::string> &links);
	void setupModel(boost::shared_ptr<collision_space::EnvironmentModel> &model, const std::vector<std::string> &links);
	
	boost::shared_ptr<collision_space::EnvironmentModel> ode_collision_model_;
      //boost::shared_ptr<collision_space::EnvironmentModel> bullet_collision_model_;

	double                                               scale_;
	double                                               padd_;
	std::vector<double>                                  boundingPlanes_;
      std::vector<motion_planning_msgs::CollisionOperation> default_collision_operations_;
    };
    
	
}

#endif

