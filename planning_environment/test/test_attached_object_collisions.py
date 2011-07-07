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
from mapping_msgs.msg import AttachedCollisionObject
from arm_navigation_msgs.msg import CollisionOperation
from arm_navigation_msgs.msg import Shape
from geometry_msgs.msg import Pose
from arm_navigation_msgs.srv import GetStateValidity,GetStateValidityRequest
from arm_navigation_msgs.msg import ContactInformation

default_prefix = "/environment_server"

class TestAttachedObjectCollisions(unittest.TestCase):

    def setUp(self):

        rospy.init_node('test_attached_object_collisions')

        self.obj_pub = rospy.Publisher('collision_object',CollisionObject,latch=True)
        self.att_pub = rospy.Publisher('attached_collision_object', AttachedCollisionObject,latch=True)

        rospy.wait_for_service(default_prefix+'/get_state_validity')
        self.get_state_validity_server = rospy.ServiceProxy(default_prefix+'/get_state_validity', GetStateValidity)
        
        self.state_req = GetStateValidityRequest()
        self.state_req.robot_state.joint_state.name = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint']
        self.state_req.robot_state.joint_state.position = [float(0.0) for _ in range(7)]
        self.state_req.check_collisions = True

        self.table = CollisionObject()

        self.table.header.stamp = rospy.Time.now()
        self.table.header.frame_id = "base_link"
        self.table.id = "tabletop"
        self.table.operation.operation = mapping_msgs.msg.CollisionObjectOperation.ADD
        self.table.shapes = [Shape() for _ in range(1)]
        self.table.shapes[0].type = Shape.BOX
        self.table.shapes[0].dimensions = [float() for _ in range(3)]
        self.table.shapes[0].dimensions[0] = 1.0
        self.table.shapes[0].dimensions[1] = 1.0
        self.table.shapes[0].dimensions[2] = .05
        self.table.poses = [Pose() for _ in range(1)]
        self.table.poses[0].position.x = 1.0
        self.table.poses[0].position.y = 0
        self.table.poses[0].position.z = .5
        self.table.poses[0].orientation.x = 0
        self.table.poses[0].orientation.y = 0
        self.table.poses[0].orientation.z = 0
        self.table.poses[0].orientation.w = 1

        #not publishing table right away

        self.att_object = AttachedCollisionObject();
        self.att_object.object.header.stamp = rospy.Time.now()
        self.att_object.object.header.frame_id = "r_gripper_r_finger_tip_link"
        self.att_object.link_name = "r_gripper_r_finger_tip_link"
        self.att_object.object.operation.operation = mapping_msgs.msg.CollisionObjectOperation.ADD
        self.att_object.object = CollisionObject();

        self.att_object.object.header.stamp = rospy.Time.now()
        self.att_object.object.header.frame_id = "r_gripper_r_finger_tip_link"
        self.att_object.object.id = "pole"
        self.att_object.object.shapes = [Shape() for _ in range(1)]
        self.att_object.object.shapes[0].type = Shape.CYLINDER
        self.att_object.object.shapes[0].dimensions = [float() for _ in range(2)]
        self.att_object.object.shapes[0].dimensions[0] = .02
        self.att_object.object.shapes[0].dimensions[1] = 1.2
        self.att_object.object.poses = [Pose() for _ in range(1)]
        self.att_object.object.poses[0].position.x = 0
        self.att_object.object.poses[0].position.y = 0
        self.att_object.object.poses[0].position.z = 0
        self.att_object.object.poses[0].orientation.x = 0
        self.att_object.object.poses[0].orientation.y = 0
        self.att_object.object.poses[0].orientation.z = 0
        self.att_object.object.poses[0].orientation.w = 1

        self.att_pub.publish(self.att_object)

        self.touch_links = ['r_gripper_palm_link', 'r_gripper_r_finger_link', 'r_gripper_l_finger_link',
                            'r_gripper_r_finger_tip_link', 'r_gripper_l_finger_tip_link']

        rospy.sleep(2.)
        
    def test_attached_collisions_touch_links(self):

        #original attached object had no touch links
        
        self.state_req.robot_state.joint_state.header.stamp = rospy.Time.now()

        res = self.get_state_validity_server.call(self.state_req)

        self.failIf(res.error_code.val == res.error_code.SUCCESS)
        self.assertEqual(res.error_code.val, res.error_code.COLLISION_CONSTRAINTS_VIOLATED) 

        #should be some contacts
        self.failIf(len(res.contacts) == 0)
        
        for c in res.contacts:

            is_first = False
            
            if(c.contact_body_1 == 'r_gripper_r_finger_tip_link'):
                is_first = True
            elif(c.contact_body_2 != 'r_gripper_r_finger_tip_link'):
                self.fail("Contacts but not with correct link")

            if c.contact_body_1 not in self.touch_links:
                self.fail("Contact body 1 ", c.contact_body_1, " not in touch links");

            if c.contact_body_2 not in self.touch_links:
                self.fail("Contact body 2 ", c.contact_body_2, " not in touch links");

            self.failIf(not(c.contact_body_1 in self.touch_links))
            self.failIf(not(c.contact_body_2 in self.touch_links))

            if(is_first):
                self.failIf(c.body_type_1 != ContactInformation.ATTACHED_BODY)
                self.failIf(c.attached_body_1 != "pole")
            else:
                self.failIf(c.body_type_2 != ContactInformation.ATTACHED_BODY)
                self.failIf(c.attached_body_2 != "pole")

            

        #adding in touch links
        self.att_object.touch_links = self.touch_links
        self.att_pub.publish(self.att_object)

        rospy.sleep(2.)

        res = self.get_state_validity_server.call(self.state_req)

        #should be ok now
        self.failIf(res.error_code.val != res.error_code.SUCCESS)

    def test_attached_collisions_with_static_objects(self):
        
        self.att_object.touch_links = self.touch_links
        self.att_pub.publish(self.att_object)

        rospy.sleep(2.)

        self.obj_pub.publish(self.table)
        
        rospy.sleep(2.)

        self.state_req.robot_state.joint_state.header.stamp = rospy.Time.now()

        res = self.get_state_validity_server.call(self.state_req)

        self.failIf(res.error_code.val == res.error_code.SUCCESS)
        self.assertEqual(res.error_code.val, res.error_code.COLLISION_CONSTRAINTS_VIOLATED) 
        
        #should be some contacts
        self.failIf(len(res.contacts) == 0)
        
        for c in res.contacts:
            
            self.failIf(c.contact_body_1 != 'r_gripper_r_finger_tip_link')
            self.failIf(c.body_type_1 != ContactInformation.ATTACHED_BODY)
            self.failIf(c.attached_body_1 != "pole")

            self.failIf(c.contact_body_2 != 'tabletop')
            self.failIf(c.body_type_2 != ContactInformation.OBJECT)

if __name__ == '__main__':
    import rostest
    rostest.unitrun('test_attached_object_collisions', 'test_attached_object_collisions', TestAttachedObjectCollisions, sys.argv)
