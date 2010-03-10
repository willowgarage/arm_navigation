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
 *   * Neither the name of Willow Garage nor the names of its
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

#ifndef PLANNING_JOINT_GROUP_H_
#define PLANNING_JOINT_GROUP_H_

#include <geometric_shapes/shapes.h>

#include <urdf/model.h>

#include <LinearMath/btTransform.h>
#include <boost/thread/mutex.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <map>

namespace planning_models
{
	class JointGroup
	{
	public:
	    
    JointGroup(boost::shared_ptr<PlanningModel> model, const std::string& group_name, const std::vector<boost::shared_ptr<Joint> > &group_joints);
    ~JointGroup(void);
	    
    /** \brief The kinematic model that owns the group */
    boost::shared_ptr<PlanningModel>    parent_model_;	    

    /** \brief Name of group */
    std::string                         name_;

    /** \brief Names of joints in the order they appear in the group state */
    std::vector<std::string>            joint_names_;

    /** \brief Joint instances in the order they appear in the group state */
    std::vector<boost::shared_ptr<Joint> >                 joints_;

    /** \brief Index where each joint starts within the group state */
    std::vector<unsigned int>           joint_index_;

    /** \brief Easy way of finding the position of a joint in the list of joints contained in the group */
    std::map<std::string, unsigned int> joint_index_map_;

    /** \brief The dimension of the group */
    unsigned int                        dimension_;

    /** \brief The bounds for the state corresponding to the group */
    std::vector<urdf::JointLimits>      joint_limits_;
	    
    /** \brief An array containing the index in the global state for each dimension of the state of the group */
    std::vector<unsigned int>           state_index_;
	    
    /** \brief The list of joints that are roots in this group */
    std::vector<boost::shared_ptr<Joint> >                 root_joints_;

    /** \brief The list of links that are updated when computeTransforms() is called, in the order they are updated */
    std::vector<boost::shared_ptr<Link> >                  updated_links_;
	    
    /** \brief Perform forward kinematics starting at the roots
        within a group. Links that are not in the group are also
        updated, but transforms for joints that are not in the
        group are not recomputed.  */
    void updateTransforms(void);	

    /** \brief Check if a joint is part of this group */
    bool hasJoint(const std::string &joint) const;
	    
    /** \brief Get the position of a joint inside this group */
    int  getJointPosition(const std::string &joint) const;
	    
	};
}

#endif
