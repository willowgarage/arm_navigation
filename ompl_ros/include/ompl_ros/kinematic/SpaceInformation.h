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

#ifndef OMPL_ROS_KINEMATIC_SPACE_INFORMATION_
#define OMPL_ROS_KINEMATIC_SPACE_INFORMATION_

#include "ompl_ros/ModelBase.h"
#include <ompl/kinematic/SpaceInformationKinematic.h>
#include <motion_planning_msgs/Constraints.h>
#include <vector>

namespace ompl_ros
{
  /** \brief This class configures an instance of SpaceInformationKinematic with data from a kinematic model */
  class ROSSpaceInformationKinematic : public ompl::kinematic::SpaceInformationKinematic
    {
    public:
	
    ROSSpaceInformationKinematic(ModelBase *model) : ompl::kinematic::SpaceInformationKinematic()
        {
          configureOMPLSpace(model);
        }
	
      virtual ~ROSSpaceInformationKinematic(void)
        {
        }
	
      /** \brief For planar and floating joints, we have infinite
          dimensions. The bounds for these dimensions are set by the
          user. */
      void setPlanningVolume(double x0, double y0, double z0, double x1, double y1, double z1);
	
      /** \brief For planar and floating joints, we have infinite
          dimensions. The bounds for these dimensions are set by the
          user. */
      void setPlanningArea(double x0, double y0, double x1, double y1);
	
      /** \brief Reduce bounds on the state space to incorporate joint constraints if needed, and/or add
       * constraints to the state validity predicate */
      void setPathConstraints(const motion_planning_msgs::Constraints &kc);
	
      /** \brief Restore the constraints to the ones when the class was instantiated */
      void clearPathConstraints(void);

      /** \brief Print settings about the space  */
      virtual void printSettings(std::ostream &out = std::cout) const;

    protected:

      void configureOMPLSpace(ModelBase *model);
      void checkResolution(void);
      bool checkBounds(void);
	
      std::vector<ompl::base::StateComponent>  basicStateComponent_;
      planning_models::KinematicModel         *kmodel_;
      std::string                              groupName_;
      std::vector<int>                         floatingJoints_;
      std::vector<int>                         planarJoints_;	
      double                                   divisions_;
    };    
} // ompl_ros

#endif
    
