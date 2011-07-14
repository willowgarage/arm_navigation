#! /usr/bin/env python

PKG = 'planning_environment'

import roslib; roslib.load_manifest(PKG)
import rospy
import arm_navigation_msgs.srv
import sys
import unittest
import math

import sensor_msgs.msg
import mapping_msgs.msg
from mapping_msgs.msg import CollisionObject
from mapping_msgs.msg import AttachedCollisionObject
from arm_navigation_msgs.msg import CollisionOperation, ArmNavigationErrorCodes, AllowedContactSpecification, RobotState
from arm_navigation_msgs.msg import Shape
from geometry_msgs.msg import Pose, PointStamped
from arm_navigation_msgs.srv import GetRobotState, GetStateValidity, GetRobotStateRequest, GetStateValidityRequest
from tf import TransformListener

default_prefix = "/environment_server"

class TestGetStateValidity(unittest.TestCase):
    
    def setUp(self):

        rospy.init_node('test_get_base_state_validity')

        self.tf = TransformListener()
        
        self.obj_pub = rospy.Publisher('collision_object',CollisionObject,latch=True)

        rospy.wait_for_service(default_prefix+'/get_robot_state')
        self.get_robot_state_server = rospy.ServiceProxy(default_prefix+'/get_robot_state', GetRobotState)
        
        rospy.wait_for_service(default_prefix+'/get_state_validity')
        self.get_state_validity_server = rospy.ServiceProxy(default_prefix+'/get_state_validity', GetStateValidity)
        
        obj1 = CollisionObject()

        obj1.header.stamp = rospy.Time.now()
        obj1.header.frame_id = 'odom_combined'
        obj1.id = 'table'
        obj1.operation.operation = mapping_msgs.msg.CollisionObjectOperation.ADD
        obj1.shapes = [Shape() for _ in range(1)]
        obj1.shapes[0].type = Shape.BOX
        obj1.shapes[0].dimensions = [float() for _ in range(3)]
        obj1.shapes[0].dimensions[0] = 1.0
        obj1.shapes[0].dimensions[1] = 1.0
        obj1.shapes[0].dimensions[2] = .1
        obj1.poses = [Pose() for _ in range(1)]
        obj1.poses[0].position.x = 4.25
        obj1.poses[0].position.y = 0
        obj1.poses[0].position.z = .75
        obj1.poses[0].orientation.x = 0
        obj1.poses[0].orientation.y = 0
        obj1.poses[0].orientation.z = 0
        obj1.poses[0].orientation.w = 1
    
        self.obj_pub.publish(obj1)
        
        rospy.sleep(5.)

    def test_get_base_state_validity(self):

        state_req = GetStateValidityRequest()
        state_req.check_collisions = True

        #empty state should be ok
        res = self.get_state_validity_server.call(state_req)
        self.failIf(res.error_code.val != res.error_code.SUCCESS)

        #should be in collision
        state_req.robot_state.multi_dof_joint_state.joint_names.append("base_joint")
        state_req.robot_state.multi_dof_joint_state.frame_ids.append("odom_combined")
        state_req.robot_state.multi_dof_joint_state.child_frame_ids.append("base_footprint")
        pose = Pose()
        pose.position.x = 4.0
        pose.orientation.w = 1.0
        state_req.robot_state.multi_dof_joint_state.poses.append(pose)
        res = self.get_state_validity_server.call(state_req)
        self.failIf(res.error_code.val == res.error_code.SUCCESS)
        self.assertEqual(res.error_code.val, res.error_code.COLLISION_CONSTRAINTS_VIOLATED) 
        
        #shouldn't be in collision as it doesn't have right parent_frame
        state_req.robot_state.multi_dof_joint_state.frame_ids[0] = ""
        res = self.get_state_validity_server.call(state_req)
        self.failIf(res.error_code.val != res.error_code.SUCCESS)

        #should be in collision now
        state_req.robot_state.joint_state.name.append("floating_trans_x");
        state_req.robot_state.joint_state.position.append(4.0);
        res = self.get_state_validity_server.call(state_req)
        self.failIf(res.error_code.val == res.error_code.SUCCESS)
        self.assertEqual(res.error_code.val, res.error_code.COLLISION_CONSTRAINTS_VIOLATED) 


        #should be in collision now
        #moving back, but should still be in collision
        state_req.robot_state.joint_state.position[0] = 3.3;
        res = self.get_state_validity_server.call(state_req)
        self.failIf(res.error_code.val == res.error_code.SUCCESS)
        self.assertEqual(res.error_code.val, res.error_code.COLLISION_CONSTRAINTS_VIOLATED) 

        #but at a different theta should be ok
        state_req.robot_state.joint_state.name.append("floating_rot_z");
        state_req.robot_state.joint_state.position.append(.7071);
        state_req.robot_state.joint_state.name.append("floating_rot_w");
        state_req.robot_state.joint_state.position.append(.7071);
        res = self.get_state_validity_server.call(state_req)
        self.failIf(res.error_code.val != res.error_code.SUCCESS)


if __name__ == '__main__':
    import rostest
    rostest.unitrun('test_get_state_validity', 'test_get_state_validity', TestGetStateValidity, sys.argv)

 
