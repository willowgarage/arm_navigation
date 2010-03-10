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

/** \author Ioan Sucan, Sachin Chitta */

#ifndef PLANNING_LINK_H_
#define PLANNING_LINK_H_

#include <geometric_shapes/shapes.h>
#include <LinearMath/btTransform.h>
#include <urdf/model.h>

#include <planning_models/joint.h>
#include <planning_models/attached_body.h>

#include <vector>
#include <string>
#include <map>

namespace planning_models
{
  class PlanningModel;

	/** \brief A link from the robot. Contains the constant transform applied to the link and its geometry */
  class Link
	{
	public:

    Link(boost::shared_ptr<PlanningModel> model);	    
    ~Link(void);

    /** \brief The model that owns this link */
    boost::shared_ptr<PlanningModel> parent_model_;
	    
    /** \brief The constant transform applied to the link (local) */
    btTransform               local_transform_;
	    
    /** \brief The collision geometry of the link */
    boost::shared_ptr<shapes::Shape> shape_;
	    
    /** \brief Attached bodies */
    std::vector<boost::shared_ptr<AttachedBody> > attached_bodies_;	    
	    
    /** \brief The global transform this link forwards (computed by forward kinematics) */
    btTransform global_transform_fwd_;

    /** \brief The global transform for this link (computed by forward kinematics) */
    btTransform global_transform_;

    /** \brief Recompute global_transform_ and global_transform_fwd_ */
    void updateTransform(void);	    

    boost::shared_ptr<Joint> parent_joint_;

    std::vector<boost::shared_ptr<Joint> > child_joints_;
	};
}

#endif
