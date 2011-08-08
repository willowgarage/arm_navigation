#! /usr/bin/env python

PKG = 'planning_environment'

import roslib; roslib.load_manifest(PKG)
import rospy
import arm_navigation_msgs.srv
import sys
import unittest

import sensor_msgs.msg
import mapping_msgs.msg
from arm_navigation_msgs import arm_navigation_msgs_utils
from mapping_msgs.msg import CollisionObject
from mapping_msgs.msg import AttachedCollisionObject
from arm_navigation_msgs.msg import CollisionOperation, ArmNavigationErrorCodes, AllowedContactSpecification
from arm_navigation_msgs.msg import Shape
from geometry_msgs.msg import Pose, PointStamped
from arm_navigation_msgs.srv import GetRobotState, GetStateValidity, GetRobotStateRequest, GetStateValidityRequest, GetGroupInfoRequest, GetGroupInfo
from tf import TransformListener

default_prefix = "/environment_server"

class TestGetStateValidity(unittest.TestCase):
    
    def setUp(self):

        rospy.init_node('test_allowed_collision_near_start')

        self.tf = TransformListener()
        
        self.obj_pub = rospy.Publisher('collision_object',CollisionObject,latch=True)

        rospy.wait_for_service(default_prefix+'/get_robot_state')
        self.get_robot_state_server = rospy.ServiceProxy(default_prefix+'/get_robot_state', GetRobotState)
        
        rospy.wait_for_service(default_prefix+'/get_state_validity')
        self.get_state_validity_server = rospy.ServiceProxy(default_prefix+'/get_state_validity', GetStateValidity)

        rospy.wait_for_service(default_prefix+'/get_group_info')
        self.get_group_info_server = rospy.ServiceProxy(default_prefix+'/get_group_info', GetGroupInfo)
        
        rospy.sleep(3.)

        obj1 = CollisionObject()

        obj1.header.stamp = rospy.Time.now()
        obj1.header.frame_id = "r_gripper_palm_link"
        obj1.id = "test_object"
        obj1.operation.operation = mapping_msgs.msg.CollisionObjectOperation.ADD
        obj1.shapes = [Shape() for _ in range(1)]
        obj1.shapes[0].type = Shape.BOX
        obj1.shapes[0].dimensions = [float() for _ in range(3)]
        obj1.shapes[0].dimensions[0] = .03
        obj1.shapes[0].dimensions[1] = .03
        obj1.shapes[0].dimensions[2] = .03
        obj1.poses = [Pose() for _ in range(1)]
        obj1.poses[0].position.x = 0
        obj1.poses[0].position.y = 0
        obj1.poses[0].position.z = 0
        obj1.poses[0].orientation.x = 0
        obj1.poses[0].orientation.y = 0
        obj1.poses[0].orientation.z = 0
        obj1.poses[0].orientation.w = 1
    
        self.obj_pub.publish(obj1)

        rospy.sleep(2.)

        # now we put another object in collision with the base
        obj2 = CollisionObject()

        obj2.header.stamp = rospy.Time.now()
        obj2.header.frame_id = "base_link"
        obj2.id = "base_object"
        obj2.operation.operation = mapping_msgs.msg.CollisionObjectOperation.ADD
        obj2.shapes = [Shape() for _ in range(1)]
        obj2.shapes[0].type = Shape.BOX
        obj2.shapes[0].dimensions = [float() for _ in range(3)]
        obj2.shapes[0].dimensions[0] = .1
        obj2.shapes[0].dimensions[1] = .1
        obj2.shapes[0].dimensions[2] = .1
        obj2.poses = [Pose() for _ in range(1)]
        obj2.poses[0].position.x = 0
        obj2.poses[0].position.y = 0
        obj2.poses[0].position.z = 0
        obj2.poses[0].orientation.x = 0
        obj2.poses[0].orientation.y = 0
        obj2.poses[0].orientation.z = 0
        obj2.poses[0].orientation.w = 1

        self.obj_pub.publish(obj2)
        
        rospy.sleep(5.)

    def test_get_state_validity(self):

        req = GetRobotStateRequest()

        cur_state = self.get_robot_state_server.call(req)
        
        #for i in range(len(cur_state.robot_state.joint_state.name)):
        #    print cur_state.robot_state.joint_state.name[i], cur_state.robot_state.joint_state.position[i] 

        state_req = GetStateValidityRequest()
        state_req.robot_state = cur_state.robot_state

        group_req = GetGroupInfoRequest()
        group_req.group_name = 'right_arm'

        group_res = self.get_group_info_server.call(group_req)

        self.failIf(len(group_res.link_names) == 0)

        right_arm_links = group_res.link_names

        group_req.group_name = ''
        
        group_res = self.get_group_info_server.call(group_req)
        
        self.failIf(len(group_res.link_names) == 0)

        state_req.ordered_collision_operations.collision_operations = arm_navigation_msgs_utils.make_disable_allowed_collisions_with_exclusions(group_res.link_names,
                                                                                                                                                      right_arm_links)

        for i in state_req.ordered_collision_operations.collision_operations:
            print 'Disabling ', i.object1
        
        state_req.check_collisions = True

        res = self.get_state_validity_server.call(state_req)

        #should be in collision
        self.failIf(res.error_code.val == res.error_code.SUCCESS)
        self.assertEqual(res.error_code.val, res.error_code.COLLISION_CONSTRAINTS_VIOLATED) 

        #should be some contacts
        self.failIf(len(res.contacts) == 0)

        for c in res.contacts:
        
            #getting everything in common frame of base_link
            contact_point = PointStamped()
            contact_point.header = c.header
            contact_point.point = c.position
            contact_point_base = self.tf.transformPoint("base_link",contact_point)

            gripper_point = PointStamped()
            gripper_point.header.stamp = c.header.stamp
            gripper_point.header.frame_id = "r_gripper_palm_link"
            gripper_point.point.x = 0
            gripper_point.point.y = 0
            gripper_point.point.z = 0
            gripper_point_base = self.tf.transformPoint("base_link", gripper_point)

            print 'x diff:', abs(gripper_point_base.point.x-contact_point_base.point.x)
            print 'y diff:', abs(gripper_point_base.point.y-contact_point_base.point.y)
            print 'z diff:', abs(gripper_point_base.point.z-contact_point_base.point.z)

            self.failIf(abs(gripper_point_base.point.x-contact_point_base.point.x) > .031)
            self.failIf(abs(gripper_point_base.point.y-contact_point_base.point.y) > .031)
            self.failIf(abs(gripper_point_base.point.z-contact_point_base.point.z) > .031)

        #now we delete obj1
        obj2 = CollisionObject()

        obj2.header.stamp = rospy.Time.now()
        obj2.header.frame_id = "base_link"
        obj2.id = "test_object"
        obj2.operation.operation = mapping_msgs.msg.CollisionObjectOperation.REMOVE

        self.obj_pub.publish(obj2)
        
        rospy.sleep(5.)

        cur_state = self.get_robot_state_server.call(req)
        state_req.robot_state = cur_state.robot_state

        res = self.get_state_validity_server.call(state_req)

        # base shouldn't collide due to child only links
        self.failIf(res.error_code.val != res.error_code.SUCCESS)

        # now it should collide
        state_req.ordered_collision_operations.collision_operations = []

        res = self.get_state_validity_server.call(state_req)

        # base shouldn't collide due to child only links
        self.failIf(res.error_code.val == res.error_code.SUCCESS)
        
if __name__ == '__main__':
    import rostest
    rostest.unitrun('test_get_state_validity', 'test_get_state_validity', TestGetStateValidity, sys.argv)

 
