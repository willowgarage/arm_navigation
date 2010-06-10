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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
* Author: Sachin Chitta
*********************************************************************/
#ifndef KINEMATICS_BASE_
#define KINEMATICS_BASE_

#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetPositionFK.h>

namespace kinematics {
  /**
   * @class KinematicsBase
   * @brief Provides an interface for kinematics solvers.
   */
  class KinematicsBase{
    public:
      /**
       * @brief Given a desired pose of the end-effector, compute the joint angles to reach it
       * @param request  - the request contains the desired pose, seed angles and a timeout
       * @param response - the response contains the resulting joint angle positions
       * @return True if a valid solution was found, false otherwise
       */
      virtual bool getPositionIK(kinematics_msgs::GetPositionIK::Request &request, 
                                 kinematics_msgs::GetPositionIK::Response &response) = 0;      

      /**
       * @brief Given a set of joint angles and a set of links, compute their pose
       * @param request  - the request contains the joint angles, set of links for which poses are to be computed and a timeout
       * @param response - the response contains stamped pose information for all the requested links
       * @return True if a valid solution was found, false otherwise
       */
      virtual bool getPositionFK(kinematics_msgs::GetPositionFK::Request &request, 
                                 kinematics_msgs::GetPositionFK::Response &response) = 0;

      /**
       * @brief  Initialization function for the kinematics
       * @return True if initialization was successful, false otherwise
       */
      virtual bool initialize(std::string name) = 0;

      /**
       * @brief  Virtual destructor for the interface
       */
      virtual ~KinematicsBase(){}

    protected:
    KinematicsBase(){}
  };
};

#endif
