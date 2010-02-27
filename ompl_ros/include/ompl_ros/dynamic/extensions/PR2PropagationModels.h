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

#ifndef OMPL_ROS_DYNAMIC_EXTENSIONS_PR2_PROPAGATION_MODELS_
#define OMPL_ROS_DYNAMIC_EXTENSIONS_PR2_PROPAGATION_MODELS_

#include "ompl_ros/dynamic/ForwardPropagationModels.h"

namespace ompl_ros
{
    
    /** \brief Model for PR2 Base */
    class PR2BaseModel : public ForwardPropagationModel
    {
    public:

	PR2BaseModel(ompl::dynamic::SpaceInformationControlsIntegrator *si) : m_(boost::bind(&PR2BaseModel::modelODE, this, _1, _2, _3), 3), L_(0.75), si_(si)
	{
	    name = "base";
	}
	
	virtual void controlDefinition(std::vector<ompl::base::ControlComponent> &component, unsigned int *dimension,
				       unsigned int *minDuration, unsigned int *maxDuration, double *resolution)
	{
	    *dimension = 2;
	    component.resize(*dimension);
	    component[0].type = ompl::base::ControlComponent::LINEAR;
	    component[0].minValue = -1.0;
	    component[0].maxValue = 1.5;
	    component[1].type = ompl::base::ControlComponent::LINEAR;
	    component[1].minValue = -0.9;
	    component[1].maxValue =  0.9;
	    
	    *resolution = 0.05;
	    *minDuration = 5;
	    *maxDuration = 10;
	}
	
	virtual void operator()(const ompl::base::State *begin, const ompl::base::Control *ctrl, double resolution, ompl::base::State *end) const 
	{
	    m_.step(begin, ctrl, resolution, end);
	}
	
    private:
	
	// state = (x, y, theta) ; control = (fwd speed, turn speed)
	void modelODE(const ompl::base::State *begin, const ompl::base::Control *ctrl, double *diff) const
	{
	    // dx = fwd speed * cos (theta)
	    diff[0] = ctrl->values[0] * cos(begin->values[2]);

	    // dy = fwd speed * sin (theta)
	    diff[1] = ctrl->values[0] * sin(begin->values[2]);
	    
	    // dtheta = tan(turn speed) * fwd speed / L 
	    diff[2] = tan(ctrl->values[1]) * ctrl->values[0] / L_;
	}
	
	EulerMethod                                        m_;	
	double                                             L_;	
	ompl::dynamic::SpaceInformationControlsIntegrator *si_;
	
    };
}

#endif
