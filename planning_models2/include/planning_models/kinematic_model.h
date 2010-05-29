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

#ifndef PLANNING_MODELS_KINEMATIC_MODEL_
#define PLANNING_MODELS_KINEMATIC_MODEL_

#include <geometric_shapes/shapes.h>

#include <urdf/model.h>
#include <LinearMath/btTransform.h>
#include <boost/thread/mutex.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <map>


/** \brief Main namespace */
namespace planning_models
{
 
    /** \brief Definition of a kinematic model. Const members of this
	class are thread safe. Note: if using instances of nested
	classes (such as JointGroup) make sure that operations
	involving such instances are in the context of the same
	kinematic model. */
    class KinematicModel
    {
    public:

	/** \brief The type of joint used to connect to the world */
	enum WorldJointType
	{
	    /** \brief No motion (0 DOF) */
	    CONNECT_FIXED,

	    /** \brief Planar motion (3 DOF represented as x, y, yaw : 3 values) */
	    CONNECT_XY_YAW,

	    /** \brief Free motion (6 DOF represented as x, y, z and a quaternion: 7 values)*/
	    CONNECT_XYZ_QUAT
	    
	};
	
	
	/** \brief Forward definition of a joint */
	class Joint;
	
	/** \brief Forward definition of a link */
	class Link;

	/** \brief Forward definition of an attached body */
	class AttachedBody;
	
	/** \brief Forward definition of a group of joints */
	class JointGroup;
	
	
	/** \brief A joint from the robot. Contains the transform applied by the joint type */
	class Joint
	{
	public:
	    Joint(KinematicModel *model);
	    virtual ~Joint(void);

	    /** \brief Name of the joint */
	    std::string       name;
	    
	    /** \brief The model that owns this joint */
	    KinematicModel   *owner;
	    
	    /** \brief The range of indices in the parameter vector that
		needed to access information about the position of this
		joint */
	    unsigned int      used_params;

	    /** \brief The index where this joint starts reading params in the global state vector */
	    unsigned int      state_index;
	    
	    /** \brief The link before this joint */
	    Link             *parent_link;

	    /** \brief The link after this joint */
	    Link             *child_link;

	    /** \brief the local transform (computed by forward kinematics) */
	    btTransform       variable_transform;

	    /** \brief Update the value of variable_transform using the information from params */
	    virtual void updateVariableTransform(const double *params) = 0;

	};

	/** \brief A fixed joint */
	class FixedJoint : public Joint
	{
	public:
	    
	    FixedJoint(KinematicModel *owner) : Joint(owner)
	    {
	    }
	    
	    /** \brief Update the value of variable_transform using the information from params */
	    virtual void updateVariableTransform(const double *params);
	};

	/** \brief A prismatic joint */
	class PrismaticJoint : public Joint
	{
	public:
	    
	    PrismaticJoint(KinematicModel *owner) : Joint(owner), axis(0.0, 0.0, 0.0), low_limit(0.0), hi_limit(0.0)
	    {
		used_params = 1;
	    }
	    
	    btVector3 axis;
	    double    low_limit;
	    double    hi_limit;
	    
	    /** \brief Update the value of variable_transform using the information from params */
	    virtual void updateVariableTransform(const double *params);
	    
	};
	
	/** \brief A revolute joint */
	class RevoluteJoint : public Joint
	{
	public:
	    
	    RevoluteJoint(KinematicModel *owner) : Joint(owner), axis(0.0, 0.0, 0.0),
						   low_limit(0.0), hi_limit(0.0), continuous(false)
	    {
		used_params = 1;
	    }
	    	    
	    btVector3 axis;
	    double    low_limit;
	    double    hi_limit;
	    bool      continuous;

	    /** \brief Update the value of variable_transform using the information from params */
	    virtual void updateVariableTransform(const double *params);

	};
	
	/** \brief Definition of a special joint that connects the model to the world */
	class WorldJoint : public Joint
	{
	public:

	    /** \brief The name used for world joints */
	    static const std::string NAME;
	    
	    WorldJoint(KinematicModel *owner) : Joint(owner)
	    {
		type = owner->getWorldJointType();
		name = NAME;
		
		switch (type)
		{
		case CONNECT_FIXED:
		    break;
		case CONNECT_XY_YAW:
		    used_params = 3;
		    break;
		case CONNECT_XYZ_QUAT:
		    used_params = 7;
		    break;
		default:
		    break;
		}
	    }
	    
	    /** \brief The type of connection this joint realizes */
	    WorldJointType type;
	    
	    /** \brief Update the value of variable_transform using the information from params */
	    virtual void updateVariableTransform(const double *params);
	};	
	
	/** \brief A link from the robot. Contains the constant transform applied to the link and its geometry */
	class Link
	{
	public:

	    Link(KinematicModel *model);	    
	    ~Link(void);

	    /** \brief Name of the link */
	    std::string                name;

	    /** \brief The model that owns this link */
	    KinematicModel            *owner;

	    /** \brief Joint that connects this link to the parent link */
	    Joint                     *parent_joint;
	    
	    /** \brief List of descending joints (each connects to a child link) */
	    std::vector<Joint*>        child_joint;
	    
	    /** \brief The constant transform applied to the link (local) */
	    btTransform                joint_origin_transform;
	    
	    /** \brief The constant transform applied to the collision geometry of the link (local) */
	    btTransform                collision_origin_transform;
	    
	    /** \brief The geometry of the link */
	    shapes::Shape             *shape;
	    
	    /** \brief Attached bodies */
	    std::vector<AttachedBody*> attached_bodies;	    
	    
	    /** \brief The global transform this link forwards (computed by forward kinematics) */
	    btTransform                global_link_transform;

	    /** \brief The global transform for this link (computed by forward kinematics) */
	    btTransform                global_collision_body_transform;

	    /** \brief Recompute global_collision_body_transform and global_link_transform */
	    void computeTransform(void);

	};
	
	/** \brief Class defining bodies that can be attached to robot
	    links. This is useful when handling objects picked up by
	    the robot. */
	class AttachedBody
	{
	public:
	    
	    AttachedBody(Link *link, const std::string& id);
	    ~AttachedBody(void);
	    
	    /** \brief The link that owns this attached body */
	    Link                                  *owner;
	    
	    /** \brief The geometries of the attached body */
	    std::vector<shapes::Shape*>            shapes;
	    
	    /** \brief The constant transforms applied to the link (need to be specified by user) */
	    std::vector<btTransform>               attach_trans;
	    
	    /** \brief The global transforms for these attached bodies (computed by forward kinematics) */
	    std::vector<btTransform>               global_collision_body_transform;
	    
	    /** \brief The set of links this body is allowed to touch */
	    std::vector<std::string>               touch_links;
	    
	    /** string id for reference */
	    std::string                            id;
	    
	    /** \brief Recompute global_collision_body_transform */
	    void computeTransform(void);
	};
	
	
	class JointGroup
	{
	public:
	    
	    JointGroup(KinematicModel *model, const std::string& groupName, const std::vector<Joint*> &groupJoints);
	    ~JointGroup(void);
	    
	    /** \brief The kinematic model that owns the group */
	    KinematicModel                     *owner;	    

	    /** \brief Name of group */
	    std::string                         name;

	    /** \brief Names of joints in the order they appear in the group state */
	    std::vector<std::string>            joint_names;

	    /** \brief Joint instances in the order they appear in the group state */
	    std::vector<Joint*>                 joints;

	    /** \brief Joint instances that have a single DOF */
	    std::vector<Joint*>                 joints_single_dof;

	    /** \brief Joint instances that have multiple DOF */
	    std::vector<Joint*>                 joints_multi_dof;

	    /** \brief Index where each joint starts within the group state */
	    std::vector<unsigned int>           joint_index;

	    /** \brief The dimension of the group */
	    unsigned int                        dimension;

	    /** \brief The bounds for the state corresponding to the group */
	    std::vector<double>                 state_bounds;
	    
	    /** \brief An array containing the index in the global state for each dimension of the state of the group */
	    std::vector<unsigned int>           state_index;
	    
	    /** \brief The list of joints that are roots in this group */
	    std::vector<Joint*>                 jointRoots;

	    /** \brief The list of links that are updated when computeTransforms() is called, in the order they are updated */
	    std::vector<Link*>                  updated_links;

	    /** \brief Bring the group to a default state. All joints are
		at 0. If 0 is not within the bounds of the joint, the
		middle of the bounds is used. */
	    void defaultState(void);
	    
	    /** \brief Perform forward kinematics starting at the roots
		within a group. Links that are not in the group are also
		updated, but transforms for joints that are not in the
		group are not recomputed.  */
	    void computeTransforms(const double *params);	

	    /** \brief Check if a joint is part of this group */
	    bool hasJoint(const std::string &joint) const;
	    
	    /** \brief Get the position of a joint inside this group */
	    int  getJointPosition(const std::string &joint) const;

	    /** \brief Check if this group contains the joints from another group */
	    bool containsGroup(const JointGroup *group) const;
	    
	    /** \brief Construct a group that consists of the union of joints of this group and the argument group */
	    JointGroup* addGroup(const JointGroup *group) const;

	    /** \brief Construct a group that consists of the joints of this group that are not joints in the argument group */
	    JointGroup* removeGroup(const JointGroup *group) const;
	    
	private:
	    
	    /** \brief Easy way of finding the position of a joint in the list of joints contained in the group */
	    std::map<std::string, unsigned int> joint_map;

	};
	
	/** \brief Construct a kinematic model from another one */
	KinematicModel(const KinematicModel &source);

	/** \brief Construct a kinematic model from a parsed description and a list of planning groups */
	KinematicModel(const urdf::Model &model, const std::map< std::string, std::vector<std::string> > &groups, WorldJointType wjt);
	
	/** \brief Destructor. Clear all memory. */
	~KinematicModel(void);
	
	/** \brief Assignment operator */
	KinematicModel& operator=(const KinematicModel &rhs);

	/** \brief Equality operator. Two models are considered equal
	    if they have the same name, the same connection to the
	    world and the same set of joint groups */
	bool operator==(const KinematicModel &rhs) const;
	
	/** \brief Bring the robot to a default state. All joints are
	    at 0. If 0 is not within the bounds of the joint, the
	    middle of the bounds is used. */
	void defaultState(void);
	
	/** \brief General the model name **/
	const std::string& getName(void) const;

	/** \brief Get the type of connection this model has to the environment */
	WorldJointType getWorldJointType(void) const;
	
	/** \brief Check if a group exists */
	bool hasGroup(const std::string &name) const;

	/** \brief Check if a link exists */
	bool hasLink(const std::string &name) const;

	/** \brief Check if a joint exisOBts */
	bool hasJoint(const std::string &name) const;

	/** \brief Get the group names, in no particular order */
	void getGroupNames(std::vector<std::string> &groups) const;

	/** \brief Get the link names, in no particular order */
	void getLinkNames(std::vector<std::string> &links) const;

	/** \brief Get the array of joint names, in the order they
	    appear in the robot state. */
	void getJointNames(std::vector<std::string> &joints) const;
       
	/** \brief Get the root joint */
	Joint* getRoot(void);

	/** \brief Get the root joint. If there is a WorldJoint in the model, this is the one */
	const Joint* getRoot(void) const;

	/** \brief Get a group by its name */
	JointGroup* getGroup(const std::string &name);

	/** \brief Get a group by its name */
	const JointGroup* getGroup(const std::string &name) const;

	/** \brief Get a link by its name */
	Link* getLink(const std::string &link);

	/** \brief Get a link by its name */
	const Link* getLink(const std::string &link) const;
	
	/** \brief Get a joint by its name */
	Joint* getJoint(const std::string &joint);

	/** \brief Get a joint by its name */
	const Joint* getJoint(const std::string &joint) const;

	/** \brief Get the array of planning groups */
	void getGroups(std::vector<JointGroup*> &groups);
	
	/** \brief Get the array of planning groups */
	void getGroups(std::vector<const JointGroup*> &groups) const;
	
	/** \brief Get the array of links, in no particular order */
	void getLinks(std::vector<Link*> &links);

	/** \brief Get the array of links, in no particular order */
	void getLinks(std::vector<const Link*> &links) const;

	/** \brief Get the set of links that follow a parent link in the kinematic chain */
	void getChildLinks(const Link* parent, std::vector<Link*> &links);

	/** \brief Get the set of links that follow a parent link in the kinematic chain */
	void getChildLinks(const Link* parent, std::vector<const Link*> &links) const;
		
	/** \brief Get the array of joints, in the order they appear
	    in the robot state. */
	void getJoints(std::vector<Joint*> &joints);	
		
	/** \brief Get the array of joints, in the order they appear
	    in the robot state. */
	void getJoints(std::vector<const Joint*> &joints) const;	

	/** \brief Get the joints that require a single parameter to describe. */
	void getSingleDOFJoints(std::vector<Joint*> &joints);
	
	/** \brief Get the joints that require a single parameter to describe. */
	void getSingleDOFJoints(std::vector<const Joint*> &joints) const;

	/** \brief Get the joints that require a multiple parameters to describe. */
	void getMultiDOFJoints(std::vector<Joint*> &joints);

	/** \brief Get the joints that require a multiple parameters to describe. */
	void getMultiDOFJoints(std::vector<const Joint*> &joints) const;

	/** \brief Get the set of joints that follow a parent joint in the kinematic chain */
	void getChildJoints(const Joint* parent, std::vector<Joint*> &joints);

	/** \brief Get the set of joints that follow a parent joint in the kinematic chain */
	void getChildJoints(const Joint* parent, std::vector<const Joint*> &joints) const;
	
	/** \brief Get the list of attached bodies */
	void getAttachedBodies(std::vector<AttachedBody*> &attached_bodies);

	/** \brief Get the list of attached bodies */
	void getAttachedBodies(std::vector<const AttachedBody*> &attached_bodies) const; 

	/** \brief Get the dimension of the entire model */
	unsigned int getDimension(void) const;
	
	/** \brief Get the state bounds constructed for this
	    model. Component i of the state space has bounds (min,
	    max) at index positions (2*i, 2*i+1)*/
	const std::vector<double> &getStateBounds(void) const;
	
	/** \brief Perform forward kinematics for the entire robot */
	void computeTransforms(const double *params);
	
	/** \brief Assuming a link is set at a specific transform, update the transforms for the necessary (descendant) links */
	void updateTransformsWithLinkAt(Link *link, const btTransform &transform);

	/** \brief Get the global transform applied to the entire tree of links */
	const btTransform& getRootTransform(void) const;
	
	/** \brief Set the global transform applied to the entire tree of links */
	void setRootTransform(const btTransform &transform);
	
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
	
	/** \brief The type of connection this model has with respect to the environment */
	WorldJointType                                    connect_type_;

	/** \brief A map from group names to their instances */
	std::map<std::string, JointGroup*>                group_map_;	

	/** \brief A map from link names to their instances */
	std::map<std::string, Link*>                      link_map_;

	/** \brief A map from joint names to their instances */
	std::map<std::string, Joint*>                     joint_map_;

	/** \brief The list of joints in the model, in the order they appear in the state vector */
	std::vector<Joint*>                               joint_list_;

	/** \brief The joints in the model that have a single DOF. This is a subset of joint_list_ */
	std::vector<Joint*>                               joint_single_dof_;
	
	/** \brief The joints in the model that have multiple DOF. This is a subset of joint_list_ */
	std::vector<Joint*>                               joint_multi_dof_;
	
	/** \brief The index at which a joint starts reading values in the state vector */
	std::vector<unsigned int>                         joint_index_;
	
	/** \brief The list of links that are updated when computeTransforms() is called, in the order they are updated */
	std::vector<Link*>                                updated_links_;	
	
	/** \brief The root joint */
	Joint                                            *root_;
	
	/** \brief The bounds in the form (min, max) for every component of the state */
	std::vector<double>                               state_bounds_;
	
	/** \brief The dimension of the model */
	unsigned int                                      dimension_;
	
	/** \brief Additional transform to be applied to the tree of links */
	btTransform                                       root_transform_;
	
	/** \brief A lock for the instance of this kinematic model */
	boost::mutex                                      lock_;
	
	/** \brief Copy the data of this model from another instance to this one */
	void copyFrom(const KinematicModel &source);
	
	/** \brief Compute additional information that is useful in the computation of this model */
	void buildConvenientDatastructures(void);	
	
	/** \brief Allocate the requested groups of joints */
	void buildGroups(const std::map< std::string, std::vector<std::string> > &groups);

	/** \brief Recursively build the set of joints and links that
	    make up the kinematic model, using parsed data */
	Joint* buildRecursive(Link *parent, const urdf::Link *link);

	/** \brief Instantiate a joint based on parsed model information */
	Joint* constructJoint(const urdf::Joint *urdfJoint, std::vector<double> &bounds);

	/** \brief Instantiate a link based on parsed model information */
	Link* constructLink(const urdf::Link *urdfLink);

	/** \brief Instantiate the correct shape from the geometry
	    specified in the parsed model. This will retrieve
	    resources if necessary. */
	shapes::Shape* constructShape(const urdf::Geometry *geom);

	/** \brief Clone a joint */
	Joint* copyJoint(const Joint *joint);

	/** \brief Clone a link */
	Link* copyLink(const Link *link);

	/** \brief Perform a copy of the links and joints starting at a specified link */
	Joint* copyRecursive(Link *parent, const Link *link);
    
    };

}

#endif
