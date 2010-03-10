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

#ifndef PLANNING_MODEL_H_
#define PLANNING_MODEL_H_

#include <urdf/model.h>
#include <planning_models/link.h>
#include <LinearMath/btTransform.h>
#include <boost/thread/mutex.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <map>


/** \brief Main namespace */
namespace planning_models
{
/** \brief Definition of a planning model. This class is not thread
    safe, however multiple instances can be created */
class PlanningModel
{	
public:

	/** \brief Construct a kinematic model from another one */
	PlanningModel(const PlanningModel &source);

	/** \brief Construct a kinematic model from a parsed description and a list of planning groups */
	PlanningModel(const urdf::Model &model, const std::map< std::string, std::vector<std::string> > &groups);
	
	/** \brief Destructor. Clear all memory. */
	~PlanningModel(void);

	/** \brief Bring the robot to a default state */
	void resetToDefaultState(void);
	
	/** \brief Get a group by its name */
  boost::shared_ptr<const JointGroup> getGroup(const std::string &name) const;

	/** \brief Get a group by its name */
	boost::shared_ptr<const JointGroup> getGroup(const std::string &name);
	
	/** \brief Check if a group exists */
	bool hasGroup(const std::string &name) const;
	
	/** \brief Get the array of planning groups */
	void getGroups(std::vector<boost::shared_ptr<const JointGroup> > &groups) const;
	
	/** \brief Get the group names, in no particular order */
	void getGroupNames(std::vector<std::string> &groups) const;

	/** \brief Get a link by its name */
	boost:shared_ptr<const Link> getLink(const std::string &link) const;

	/** \brief Get a link by its name */
	Link* getLink(const std::string &link);

	/** \brief Check if a link exists */
	bool hasLink(const std::string &name) const;

	/** \brief Get the array of links, in no particular order */
	void getLinks(std::vector<boost:shared_ptr<const Link> > &links) const;

	/** \brief Get the link names, in no particular order */
	void getLinkNames(std::vector<std::string> &links) const;

	/** \brief Get a joint by its name */
	boost::shared_ptr<const Joint> getJoint(const std::string &joint) const;

	/** \brief Get a joint by its name */
  boost::shared_ptr<Joint> getJoint(const std::string &joint);

	/** \brief Check if a joint exists */
	bool hasJoint(const std::string &name) const;
	
	/** \brief Get the array of joints, in the order they appear
	    in the robot state. */
	void getJoints(std::vector<boost::shared_ptr<const Joint> > &joints) const;
	
	/** \brief Get the array of joint names, in the order they
	    appear in the robot state. */
	void getJointNames(std::vector<std::string> &joints) const;
	
	/** \brief Perform forward kinematics for the entire robot */
	void updateTransforms();
	
	/** \brief Get the global transform applied to the entire tree of links */
	const btTransform& getRootTransform(void) const;
	
	/** \brief Set the global transform applied to the entire tree of links */
	void setRootTransform(const btTransform &transform);

	/** \brief Get the root joint */
	boost::shared_ptr<const Joint> getRootJoint(void) const;

	/** \brief Get the root joint */
  boost::shared_ptr<Joint> getRootJoint(void);
	
	/** \brief Get the dimension of the entire model */
	unsigned int getDimension(void) const;
		
	/** \brief Return a list of names of joints that are planar */
	const std::vector<std::string> &getSingleDOFJointNames(void) const;

	/** \brief Return a list of names of joints that are planar */
	const std::vector<std::string> &getPlanarJointNames(void) const;

	/** \brief Return a list of names of joints that are floating */
	const std::vector<std::string> &getFloatingJointNames(void) const;

	/** \brief Return a list of names of joints that are floating */
	const std::vector<std::string> &getFixedJointNames(void) const;
	
	/** \brief Provide interface to a lock. Use carefully! */
	void lock(void);
	
	/** \brief Provide interface to a lock. Use carefully! */
	void unlock(void);

	/** \brief Print information about the constructed model */
	void printModelInfo(std::ostream &out = std::cout) const;

	/** \brief Print the pose of every link */
	void printTransforms(std::ostream &out = std::cout) const;
	
private:
	
	/** \brief The name of the model */
	std::string                                       model_name_;	

	/** \brief A map from group names to their instances */
	std::map<std::string, boost::shared_ptr<JointGroup> >                group_map_;	

	/** \brief A map from link names to their instances */
	std::map<std::string, boost::shared_ptr<Link> >                      link_map_;

	/** \brief A map from single dof joint names to their instances */
	std::map<std::string, boost::shared_ptr<SingleDOFJoint> >            single_dof_joint_map_;

	/** \brief A map from planar joint names to their instances */
	std::map<std::string, boost::shared_ptr<PlanarJoint> >               planar_joint_map_;

	/** \brief A map from floating joint names to their instances */
	std::map<std::string, boost::shared_ptr<FloatingJoint> >             floating_joint_map_;

	/** \brief The list of all single dof joints in the model */
	std::vector<boost::shared_ptr<SingleDOFJoint> >                      single_dof_joints_;

	/** \brief The list of all planar joints in the model */
	std::vector<boost::shared_ptr<PlanarJoint> >                         planar_joints_;

	/** \brief The list of all floating joints in the model */
	std::vector<boost::shared_ptr<FloatingJoint> >                       floating_joints_;
	
	/** \brief The index at which a single dof joint starts reading values in the state vector corresponding to single dof joints*/
	std::vector<unsigned int>                         single_dof_joints_index_;

	/** \brief The index at which a planar joint reads values in the state vector corresponding to the planar joints*/
	std::vector<unsigned int>                         planar_joints_index_;

	/** \brief The index at which a floating joint reads values in the state vector corresponding to the floating joints*/
	std::vector<unsigned int>                         floating_joints_index_;
	
	/** \brief The list of links that are updated when computeTransforms() is called, in the order they are updated */
	std::vector<boost::shared_ptr<Link> >             updated_links_;	
	
	/** \brief The root joint */
  boost::shared_ptr<Joint>                          root_joint_;

	/** \brief List of single dof joints, maintained for convenience */
	std::vector<std::string>                          single_dof_joint_names_;
	
	/** \brief List of floating joints, maintained for convenience */
	std::vector<std::string>                          floating_joint_names_;

	/** \brief List of planar joints, maintained for convenience */
	std::vector<std::string>                          planar_joint_names_;

	/** \brief List of fixed joints, maintained for convenience */
  std::vector<std::string>                          fixed_joint_names_;

	/** \brief List of single dof joint limits*/
  std::vector<planning_models::SingleDOFJointLimits> single_dof_joint_limits_;

	/** \brief List of planar joint limits */
  std::vector<planning_models::PlanarJointLimits> planar_joint_limits_;

	/** \brief List of planar joint limits */
  std::vector<planning_models::FloatingJointLimits> floating_joint_limits_;
	
	/** \brief The dimension of the model */
	unsigned int                                      dimension_;
	
	/** \brief Additional transform to be applied to the tree of links */
	btTransform                                       root_transform_;
	
	boost::mutex                                      lock_;

	void buildConvenientDatastructures(void);	
	void buildGroups(const std::map< std::string, std::vector<std::string> > &groups);
  boost::shared_ptr<Joint> buildRecursive(boost::shared_ptr<Link> parent, boost::shared_ptr<const Link> link);
  boost::shared_ptr<Joint> constructJoint(boost::shared_ptr<const urdf::Joint> urdfJoint, boost::shared_ptr<const urdf::Joint::JointLimits> urdfLimits);
  boost::shared_ptr<Link> constructLink(boost::shared_ptr<const urdf::Link> urdfLink);
  boost::shared_ptr<shapes::Shape> constructShape(boost::shared_ptr<const urdf::Geometry> geom);

  boost::shared_ptr<Joint> copyJoint(boost::shared_ptr<const Joint> joint);
  boost::shared_ptr<Link> copyLink(boost::shared_ptr<const Link> link);
  boost::shared_ptr<Link> copyRecursive(boost::shared_ptr<Link> parent, boost::shared_ptr<const Link> link);

	void printTransform(const std::string &st, const btTransform &t, std::ostream &out = std::cout) const;
	
};

}

#endif
