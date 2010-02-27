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

#ifndef OMPL_PLANNING_DEBUG_DISPLAY_
#define OMPL_PLANNING_DEBUG_DISPLAY_

#include "ompl_planning/Model.h"
#include <visualization_msgs/Marker.h>

namespace ompl_planning
{
    
    class DisplayPlanner
    {
    
    public:

	DisplayPlanner(void)
	{
	    px_ = py_ = pz_ = -1;
	}
	
	/** \brief Enable debug mode. Display markers consisting of a 1-D orthogonal projection of states */
	void enableDisplay(int idx)
	{
	    enableDisplay(idx, -1, -1);
	}

	/** \brief Enable debug mode. Display markers consisting of a 2-D orthogonal projection of states */
	void enableDisplay(int idx1, int idx2)
	{
	    enableDisplay(idx1, idx2, -1);
	}

	/** \brief Enable debug mode. Display markers consisting of a 3-D orthogonal projection of states */
	void enableDisplay(int idx1, int idx2, int idx3);
	
	/** \brief Disable debug mode */
	void disableDisplay(void);

	/** \brief Display information extracted from a planner setup */
	void display(PlannerSetup *psetup);
	

    protected:
	
	ros::Publisher  displayPublisher_;	
	ros::NodeHandle nh_;	
	int             px_, py_, pz_;
	
    };
    
}

#endif
