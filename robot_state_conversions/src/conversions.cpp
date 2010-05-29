#include "robot_state_conversions/conversions.h"

namespace robot_state_conversions
{
    
    void robotStateToKinematicState(const motion_planning_msgs::RobotState &rs, planning_models::KinematicState &kstate)
    {
	kstate.setParamsJoints(rs.joint_state.position, rs.joint_state.name);
	for (unsigned int i = 0 ; i < rs.multi_dof_joint_state.size() ; ++i)
	    kstate.setParamsJoint(rs.multi_dof_joint_state[i].values, rs.multi_dof_joint_state[i].joint_name);
    }   

    void kinematicStateToRobotState(const planning_models::KinematicModel &kmodel, const planning_models::KinematicState &kstate,
				    motion_planning_msgs::RobotState &rs)
    {
	// get the 1-DOF joints and copy the data that corresponds to them from the kinematic state to the robot state
	std::vector<const planning_models::KinematicModel::Joint*> joints;
	kmodel.getSingleDOFJoints(joints);
	rs.joint_state.name.resize(joints.size());
	for (unsigned int i = 0 ; i < joints.size() ; ++i)
	    rs.joint_state.name[i] = joints[i]->name;
	kstate.copyParamsJoints(rs.joint_state.position, rs.joint_state.name);

	// get the multi-DOF joints
	joints.clear();
	kmodel.getMultiDOFJoints(joints);

	// copy them to the robot state
	rs.multi_dof_joint_state.resize(joints.size());
	for (unsigned int i = 0 ; i < rs.multi_dof_joint_state.size() ; ++i)
	{
	    rs.multi_dof_joint_state[i].joint_name = joints[i]->name;
	    kstate.copyParamsJoint(rs.multi_dof_joint_state[i].values, rs.multi_dof_joint_state[i].joint_name);
	}
    }
    
    void kinematicStateToRobotState(const planning_models::KinematicModel::JointGroup &group, const planning_models::KinematicState &kstate,
				    motion_planning_msgs::RobotState &rs)
    {
	// get the 1-DOF joints in the group and copy the data that corresponds to them from the kinematic state to the robot state
	rs.joint_state.name.resize(group.joints_single_dof.size());
	for (unsigned int i = 0 ; i < group.joints_single_dof.size() ; ++i)
	    rs.joint_state.name[i] = group.joints_single_dof[i]->name;
	kstate.copyParamsJoints(rs.joint_state.position, rs.joint_state.name);

	// copy the multi-dof joints from the group to the robot state
	rs.multi_dof_joint_state.resize(group.joints_multi_dof.size());
	for (unsigned int i = 0 ; i < rs.multi_dof_joint_state.size() ; ++i)
	{
	    rs.multi_dof_joint_state[i].joint_name = group.joints_multi_dof[i]->name;
	    kstate.copyParamsJoint(rs.multi_dof_joint_state[i].values, rs.multi_dof_joint_state[i].joint_name);
	}
    }

    void robotTrajectoryPointToKinematicState(const motion_planning_msgs::RobotTrajectory &rt, const unsigned int pos, planning_models::KinematicState &kstate)
    {
	kstate.setParamsJoints(rt.joint_trajectory.points[pos].positions, rt.joint_trajectory.joint_names);
	for (unsigned int i = 0 ; i < rt.multi_dof_joint_trajectory.size() ; ++i)
	    kstate.setParamsJoint(rt.multi_dof_joint_trajectory[i].points[pos].values, rt.multi_dof_joint_trajectory[i].joint_name);
    }
    

    void kinematicStateToRobotTrajectoryPoint(const planning_models::KinematicModel &kmodel, const planning_models::KinematicState &kstate,
					      motion_planning_msgs::RobotTrajectory &rt, const unsigned int pos, const unsigned int length)
    {
	// make sure memory is allocated (this does nothing if we already have the memory)
	rt.joint_trajectory.points.resize(length);

	// if needed, get the joint trajectory names for 1DOF joints
	if (rt.joint_trajectory.joint_names.empty())
	{
	    std::vector<const planning_models::KinematicModel::Joint*> joints;
	    kmodel.getSingleDOFJoints(joints);
	    rt.joint_trajectory.joint_names.resize(joints.size());
	    for (unsigned int i = 0 ; i < joints.size() ; ++i)
		rt.joint_trajectory.joint_names[i] = joints[i]->name;
	}
	
	// copy the 1DOF joint data
	kstate.copyParamsJoints(rt.joint_trajectory.points[pos].positions, rt.joint_trajectory.joint_names);

	// if needed, get the joint names for multi-DOF joints
	if (rt.multi_dof_joint_trajectory.empty())
	{
	    std::vector<const planning_models::KinematicModel::Joint*> joints;
	    kmodel.getMultiDOFJoints(joints);
	    rt.multi_dof_joint_trajectory.resize(joints.size());
	    for (unsigned int i = 0 ; i < joints.size() ; ++i)
	    {
		rt.multi_dof_joint_trajectory[i].joint_name = joints[i]->name;
		rt.multi_dof_joint_trajectory[i].points.resize(length);
	    }
	}
	
	// copy multi-dof data
	for (unsigned int i = 0 ; i < rt.multi_dof_joint_trajectory.size() ; ++i)
	    kstate.copyParamsJoint(rt.multi_dof_joint_trajectory[i].points[pos].values, rt.multi_dof_joint_trajectory[i].joint_name);
    }

    void kinematicStateToRobotTrajectoryPoint(const planning_models::KinematicModel::JointGroup &group, const planning_models::KinematicState &kstate,
					      motion_planning_msgs::RobotTrajectory &rt, const unsigned int pos, const unsigned int length)
    {
	// make sure memory is allocated (this does nothing if we already have the memory)
	rt.joint_trajectory.points.resize(length);

	// if needed, get the joint trajectory names for 1DOF joints
	if (rt.joint_trajectory.joint_names.empty())
	{
	    rt.joint_trajectory.joint_names.resize(group.joints_single_dof.size());
	    for (unsigned int i = 0 ; i < group.joints_single_dof.size() ; ++i)
		rt.joint_trajectory.joint_names[i] = group.joints_single_dof[i]->name;
	}
	
	// copy the 1DOF joint data
	kstate.copyParamsJoints(rt.joint_trajectory.points[pos].positions, rt.joint_trajectory.joint_names);

	// if needed, get the joint names for multi-DOF joints
	if (rt.multi_dof_joint_trajectory.empty())
	{
	    rt.multi_dof_joint_trajectory.resize(group.joints_multi_dof.size());
	    for (unsigned int i = 0 ; i < group.joints_multi_dof.size() ; ++i)
	    {
		rt.multi_dof_joint_trajectory[i].joint_name = group.joints_multi_dof[i]->name;
		rt.multi_dof_joint_trajectory[i].points.resize(length);
	    }
	}
	
	// copy multi-dof data
	for (unsigned int i = 0 ; i < rt.multi_dof_joint_trajectory.size() ; ++i)
	    kstate.copyParamsJoint(rt.multi_dof_joint_trajectory[i].points[pos].values, rt.multi_dof_joint_trajectory[i].joint_name);	
    }
}
