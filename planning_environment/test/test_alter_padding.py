#! /usr/bin/env python

PKG = 'planning_environment'

import roslib; roslib.load_manifest(PKG)
import rospy
import arm_navigation_msgs.srv
import sys
import unittest

import sensor_msgs.msg
import mapping_msgs.msg
from mapping_msgs.msg import CollisionObject
from arm_navigation_msgs.msg import CollisionOperation, ArmNavigationErrorCodes, LinkPadding
from arm_navigation_msgs.msg import Shape
from geometry_msgs.msg import Pose, PointStamped
from arm_navigation_msgs.srv import GetRobotState, GetStateValidity, GetRobotStateRequest, GetStateValidityRequest
from tf import TransformListener

default_prefix = "/environment_server"

class TestAlterPadding(unittest.TestCase):
    
    def setUp(self):

        rospy.init_node('test_allowed_collision_near_start')

        self.tf = TransformListener()
        
        self.obj_pub = rospy.Publisher('collision_object',CollisionObject,latch=True)
      
        rospy.wait_for_service(default_prefix+'/get_robot_state')
        self.get_robot_state_server = rospy.ServiceProxy(default_prefix+'/get_robot_state', GetRobotState)

        rospy.wait_for_service(default_prefix+'/get_state_validity')
        self.get_state_validity_server = rospy.ServiceProxy(default_prefix+'/get_state_validity', GetStateValidity)
        
        obj1 = CollisionObject()

        obj1.header.stamp = rospy.Time.now()
        obj1.header.frame_id = "base_link"
        obj1.id = "obj1";
        obj1.operation.operation = mapping_msgs.msg.CollisionObjectOperation.ADD
        obj1.shapes = [Shape() for _ in range(1)]
        obj1.shapes[0].type = Shape.CYLINDER
        obj1.shapes[0].dimensions = [float() for _ in range(2)]
        obj1.shapes[0].dimensions[0] = .1
        obj1.shapes[0].dimensions[1] = 1.5
        obj1.poses = [Pose() for _ in range(1)]
        obj1.poses[0].position.x = .75
        obj1.poses[0].position.y = -.37
        obj1.poses[0].position.z = .75
        obj1.poses[0].orientation.x = 0
        obj1.poses[0].orientation.y = 0
        obj1.poses[0].orientation.z = 0
        obj1.poses[0].orientation.w = 1

        self.obj_pub.publish(obj1)
        
        rospy.sleep(5.)

    def test_alter_padding(self):

        req = GetRobotStateRequest()

        cur_state = self.get_robot_state_server.call(req)

        state_req = GetStateValidityRequest()
        state_req.robot_state = cur_state.robot_state
    
        state_req.check_collisions = True

        res = self.get_state_validity_server.call(state_req)

        #no big padding initially
        self.failIf(res.error_code.val != res.error_code.SUCCESS)

        padd = LinkPadding()

        padd.link_name = "r_end_effector"
        padd.padding = .03

        state_req.link_padding = []
        state_req.link_padding.append(padd)

        res = self.get_state_validity_server.call(state_req)

        #should be in collision
        self.failIf(res.error_code.val == res.error_code.SUCCESS)
        self.assertEqual(res.error_code.val, res.error_code.COLLISION_CONSTRAINTS_VIOLATED) 

        #should be some contacts
        self.failIf(len(res.contacts) == 0)

        #check that revert works
        state_req.link_padding = []

        res = self.get_state_validity_server.call(state_req)

        self.failIf(res.error_code.val != res.error_code.SUCCESS)

if __name__ == '__main__':
    import rostest
    rostest.unitrun('test_alter_padding', 'test_alter_padding', TestAlterPadding)
