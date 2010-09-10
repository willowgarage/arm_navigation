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
#include <boost/thread/recursive_mutex.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <boost/bimap.hpp>

/** \brief Main namespace */
namespace planning_models
{
 
/** \brief Definition of a kinematic model. This class is not thread
    safe, however multiple instances can be created */
class KinematicModel
{
public:	
  /** \brief Forward definition of a joint */
  class Joint;
	
  /** \brief Forward definition of a link */
  class Link;

  /** \brief Forward definition of an attached body */
  class AttachedBody;
	
  /** \brief Forward definition of a group of joints */
  class JointGroup;
	
  struct MultiDofConfig
  {
    MultiDofConfig(std::string n) : name(n) {
    }
    
    ~MultiDofConfig() {
    }
    
    std::string name;
    std::string type;
    std::string parent_frame_id;
    std::string child_frame_id;
    std::map<std::string, std::string> name_equivalents;
  };
  
  /** \brief A joint from the robot. Contains the transform applied by the joint type */
  class Joint
  {
  public:
    Joint(KinematicModel *model, 
          const std::string name);

    Joint(const Joint* joint);

    void initialize(const std::vector<std::string>& local_names,
                    const MultiDofConfig* multi_dof_config = NULL);

    virtual ~Joint(void);

    /** \brief Name of the joint */
    std::string       name;
    
    /** \brief The model that owns this joint */
    KinematicModel   *owner;
	 
    /** \brief The link before this joint */
    Link             *parent_link;
    
    /** \brief The link after this joint */
    Link             *child_link;
        
    /** \brief the local transform (computed by forward kinematics) */
    btTransform       variable_transform;

    std::string getEquiv(const std::string name);

    std::pair<double, double> getVariableBounds(std::string variable) const;
   
    void setVariableBounds(std::string variable, double low, double high);

    bool allJointStateEquivalentsAreDefined(const std::map<std::string, double>& joint_value_map) const;

    bool setStoredJointValues(const std::map<std::string, double>& joint_value_map);

    /** \brief returns the current transform as a map from variable names to values */
    const std::map<std::string, double>& getVariableTransformValues() const;
    
    /** \brief Update the value of varTrans using current joint_state_values */
    virtual bool updateVariableTransform(const std::map<std::string, double>& joint_value_map);
    
    /** \brief Updates the value of varTrans using a transform */
    virtual bool updateVariableTransform(const btTransform& trans) = 0;

    virtual void updateVariableTransformFromStoredJointValues() = 0;

    typedef boost::bimap< std::string, std::string > js_type;
    //local names on the left, config names on the right
    js_type joint_state_equivalents;

    //map for high and low bounds
    std::map<std::string, std::pair<double, double> > joint_state_bounds;
    
    //stored in local names
    std::map<std::string, double> stored_joint_values;
    
    std::string parent_frame_id;
    std::string child_frame_id;
  };

  /** \brief A fixed joint */
  class FixedJoint : public Joint
  {
  public:
	    
    FixedJoint(KinematicModel *owner, const std::string name, const MultiDofConfig* multi_dof_config) :
      Joint(owner, name)
    {
    }
	    
    FixedJoint(const FixedJoint* joint): Joint(joint)
    {
    }

    /** \brief Updates the value of varTrans using a transform */
    virtual bool updateVariableTransform(const btTransform& trans){
      return true;
    };

    virtual void updateVariableTransformFromStoredJointValues(){
    }

    static const std::string local_names[4];

  };

  /** \brief A planar joint */
  class PlanarJoint : public Joint
  {
  public:
	    
    PlanarJoint(KinematicModel *owner, const std::string name, const MultiDofConfig* multi_dof_config);
	    
    PlanarJoint(const PlanarJoint* joint): Joint(joint)
    {
    }

    /** \brief Updates the value of varTrans using a transform */
    virtual bool updateVariableTransform(const btTransform& trans);

    virtual void updateVariableTransformFromStoredJointValues();
  };

  /** \brief A floating joint */
  class FloatingJoint : public Joint
  {
  public:
	    
    FloatingJoint(KinematicModel *owner, const std::string name, const MultiDofConfig* multi_dof_config);

    FloatingJoint(const FloatingJoint* joint) : Joint(joint)
    {
    }

    /** \brief Updates the value of varTrans using a transform */
    virtual bool updateVariableTransform(const btTransform& trans);
    
    virtual void updateVariableTransformFromStoredJointValues();

  };

  /** \brief A prismatic joint */
  class PrismaticJoint : public Joint
  {
  public:
	    
    PrismaticJoint(KinematicModel *owner, const std::string name, const MultiDofConfig* multi_dof_config);
    
    PrismaticJoint(const PrismaticJoint* joint) : Joint(joint){
      axis = joint->axis;
    }
	    
    btVector3 axis;
    
    /** \brief Updates the value of varTrans using a transform */
    virtual bool updateVariableTransform(const btTransform& trans);
    
    virtual void updateVariableTransformFromStoredJointValues();     
  };
	
  /** \brief A revolute joint */
  class RevoluteJoint : public Joint
  {
  public:
	    
    RevoluteJoint(KinematicModel *owner, const std::string name, const MultiDofConfig* multi_dof_config);

    RevoluteJoint(const RevoluteJoint* joint) : Joint(joint){
      axis = joint->axis;
      continuous = joint->continuous;
    }
	    	    
    btVector3 axis;
    bool      continuous;

    /** \brief Update the value of varTrans using the information from params */
    virtual bool updateVariableTransform(const std::map<std::string, double>& joint_value_map);
    
    /** \brief Updates the value of varTrans using a transform */
    virtual bool updateVariableTransform(const btTransform& trans);

    virtual void updateVariableTransformFromStoredJointValues();  
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

    /** \brief A map from joint names to their instances */
    std::map<std::string, Joint*> joint_map;

     /** \brief The list of joints that are roots in this group */
    std::vector<Joint*>                 joint_roots;

    /** \brief The list of links that are updated when computeTransforms() is called, in the order they are updated */
    std::vector<Link*>                  updated_links;
	    
    /** \brief Perform forward kinematics starting at the roots
        within a group. Links that are not in the group are also
        updated, but transforms for joints that are not in the
        group are not recomputed.  */
    void computeTransforms(const std::map<std::string, double>& joint_value_map);

    /** compute transforms using current joint values */
    void computeTransforms();	

    /** \brief Check if a joint is part of this group */
    bool hasJoint(const std::string &joint) const;

    /** \brief Get a joint by its name */
    Joint* getJoint(const std::string &joint);
	    
    /** \brief Gets all the joint values */
    std::map<std::string, double> getAllJointsValues() const;
    
    /** \brief Gets all the joint values associated with a particular joint  */
    std::map<std::string, double> getJointValues(const std::string joint) const;

    /** \brief Returns all the joint values in alphabetical (map) order */
    std::vector<double> getAllJointsValuesVector() const;

    /** \brief Gets all the joint values in map order*/
    std::map<std::string, unsigned int> getMapOrderIndex() const;
    
    /** \brief Update all joint values assuming alphabetical (map) order */
    void setAllJointsValues(const std::vector<double>& joint_values);
    
    /** \brief Bring the group to a default state. All joints are
        at 0. If 0 is not within the bounds of the joint, the
        middle of the bounds is used. */
    void defaultState(void);
	    
    /** \brief Check if this group contains the joints from another group */
    bool containsGroup(const JointGroup *group) const;

    /** \brief Construct a group that consists of the union of joints of this group and the argument group */
    JointGroup* addGroup(const JointGroup *group) const;

    /** \brief Construct a group that consists of the joints of this group that are not joints in the argument group */
    JointGroup* removeGroup(const JointGroup *group) const;

  };
	
  /** \brief Construct a kinematic model from another one */
  KinematicModel(const KinematicModel &source, bool load_meshes=true);

  /** \brief Construct a kinematic model from a parsed description and a list of planning groups */
  KinematicModel(const urdf::Model &model, 
                 const std::map< std::string, std::vector<std::string> > &groups, 
                 const std::vector<MultiDofConfig>& multi_dof_configs,
                 bool load_meshes = true);
	
  /** \brief Destructor. Clear all memory. */
  ~KinematicModel(void);

  void copyFrom(const KinematicModel &source);

  /** \brief Bring the robot to a default state */
  void defaultState(void);
	
  /** \brief General the model name **/
  const std::string& getName(void) const;
	
  /** \brief Get a group by its name */
  const JointGroup* getGroup(const std::string &name) const;

  /** \brief Get a group by its name */
  JointGroup* getGroup(const std::string &name);
	
  /** \brief Check if a group exists */
  bool hasGroup(const std::string &name) const;
	
  /** \brief Get the array of planning groups */
  void getGroups(std::vector<const JointGroup*> &groups) const;
	
  /** \brief Get the group names, in no particular order */
  void getGroupNames(std::vector<std::string> &groups) const;

  /** \brief Get a link by its name */
  const Link* getLink(const std::string &link) const;

  /** \brief Get a link by its name */
  Link* getLink(const std::string &link);

  /** \brief Check if a link exists */
  bool hasLink(const std::string &name) const;

  /** \brief Get the array of links, in no particular order */
  void getLinks(std::vector<const Link*> &links) const;

  /** \brief Get the link names, in no particular order */
  void getLinkNames(std::vector<std::string> &links) const;

  /** \brief Get the set of links that follow a parent link in the kinematic chain */
  void getChildLinks(const Link* parent, std::vector<Link*> &links);
  
  /** \brief Get a joint by its name */
  const Joint* getJoint(const std::string &joint) const;

  /** \brief Get a joint by its name */
  Joint* getJoint(const std::string &joint);

  /** \brief Check if a joint exists */
  bool hasJoint(const std::string &name) const;
	
  /** \brief Get the array of joints, in the order they appear
      in the robot state. */
  void getJoints(std::vector<const Joint*> &joints) const;

  /** \brief Get the array of joints, in the order they appear
      in the robot state. */
  void getJoints(std::vector<Joint*> &joints) const;
	
  /** \brief Get the array of joint names, in the order they
      appear in the robot state. */
  void getJointNames(std::vector<std::string> &joints) const;

  /** \brief Gets all the joint values */
  std::map<std::string, double> getAllJointsValues() const;
  
  /** \brief Gets all the joint values in map order*/
  std::map<std::string, unsigned int> getMapOrderIndex() const;

  /** \brief Returns all the joint values in alphabetical (map) order */
  std::vector<double> getAllJointsValuesVector() const;
  
  /** \brief Update all joint values assuming alphabetical (map) order */
  void setAllJointsValues(const std::vector<double>& joint_values);

  /** \brief Gets all the joint values associated with a particular joint  */
  std::map<std::string, double> getJointValues(std::string joint) const;

  /** \brief Check if a single joint's values are in bounds */
  bool checkJointBounds(std::string joint) const;

  /** \brief Checks if a vector of joints values' are in bounds */
  bool checkJointsBounds(std::vector<std::string> joints) const;

  /** \brief Perform forward kinematics for the entire robot */
  void computeTransforms(std::map<std::string, double> joint_value_map);

  /** \brief Perform forward kinematics for the entire robot using all current joint transforms*/
  void computeTransforms();
  
  /** \brief Assuming a link is set at a specific transform, update the transforms for the necessary (descendant) links */
  void updateTransformsWithLinkAt(Link *link, const btTransform &transform);
	
  /** \brief Get the global transform applied to the entire tree of links */
  const btTransform& getRootTransform(void) const;
	
  /** \brief Set the global transform applied to the entire tree of links */
  void setRootTransform(const btTransform &transform);

  /** \brief Get the root joint */
  const Joint* getRoot(void) const;

  /** \brief Get the root joint */
  Joint* getRoot(void);
	
  /** \brief Provide interface to a lock. Use carefully! */
  void lock(void);
	
  /** \brief Provide interface to a lock. Use carefully! */
  void unlock(void);

  /** \brief Print information about the constructed model */
  void printModelInfo(std::ostream &out = std::cout) const;

  /** \brief Print the pose of every link */
  void printTransforms(std::ostream &out = std::cout) const;

  void printTransform(const std::string &st, const btTransform &t, std::ostream &out = std::cout) const;
	
private:
	
  /** \brief The name of the model */
  std::string                                       model_name_;	

  /** \brief A map from group names to their instances */
  std::map<std::string, JointGroup*>                group_map_;	

  /** \brief A map from link names to their instances */
  std::map<std::string, Link*>                      link_map_;

  /** \brief A map from joint names to their instances */
  std::map<std::string, Joint*>                     joint_map_;

  /** \brief The list of joints in the model, in the order they appear in the state vector */
  std::vector<Joint*>                               joint_list_;
	
  /** \brief The index at which a joint starts reading values in the state vector */
  std::vector<unsigned int>                         joint_index_;
	
  /** \brief The list of links that are updated when computeTransforms() is called, in the order they are updated */
  std::vector<Link*>                                updated_links_;	
	
  /** \brief The root joint */
  Joint                                            *root_;
	
  boost::recursive_mutex                                      lock_;

  bool load_meshes_;

  void buildConvenientDatastructures(void);	
  void buildGroups(const std::map< std::string, std::vector<std::string> > &groups);
  Joint* buildRecursive(Link *parent, const urdf::Link *link, const std::vector<MultiDofConfig>& multi_dof_configs);
  Joint* constructJoint(const urdf::Joint *urdfJoint,  const urdf::Link *child_link,
                        const std::vector<MultiDofConfig>& multi_dof_configs);
  Link* constructLink(const urdf::Link *urdfLink);
  shapes::Shape* constructShape(const urdf::Geometry *geom);

  Joint* copyJoint(const Joint *joint);
  Link* copyLink(const Link *link);
  Joint* copyRecursive(Link *parent, const Link *link);
	
};

}

#endif
