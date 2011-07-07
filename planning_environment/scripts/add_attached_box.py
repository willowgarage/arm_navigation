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

get_collision_objects_service_name = "get_collision_objects"

def test_add_convert_objects():

    att_pub = rospy.Publisher('attached_collision_object',AttachedCollisionObject)

    rospy.init_node('test_collision_objects')

    att_obj = AttachedCollisionObject()
    att_obj.link_name = "r_gripper_palm_link"
    att_obj.touch_links = ['r_gripper_palm_link', 'r_gripper_r_finger_link', 'r_gripper_l_finger_link',
                           'r_gripper_r_finger_tip_link', 'r_gripper_l_finger_tip_link', 'r_wrist_roll_link', 'r_wrist_flex_link', 'r_forearm_link', 'r_gripper_motor_accelerometer_link']
    obj2 = CollisionObject()
    
    obj2.header.stamp = rospy.Time.now()
    obj2.header.frame_id = "r_gripper_palm_link"
    obj2.id = "obj2";
    obj2.operation.operation = mapping_msgs.msg.CollisionObjectOperation.ADD
    obj2.shapes = [Shape() for _ in range(1)]
    obj2.shapes[0].type = Shape.BOX
    obj2.shapes[0].dimensions = [float() for _ in range(3)]
    obj2.shapes[0].dimensions[0] = .2
    obj2.shapes[0].dimensions[1] = .4
    obj2.shapes[0].dimensions[2] = .2
    obj2.poses = [Pose() for _ in range(1)]
    obj2.poses[0].position.x = .12
    obj2.poses[0].position.y = 0
    obj2.poses[0].position.z = 0
    obj2.poses[0].orientation.x = 0
    obj2.poses[0].orientation.y = 0
    obj2.poses[0].orientation.z = 0
    obj2.poses[0].orientation.w = 1
    att_obj.object = obj2
    r = rospy.Rate(6)

    sent_twice = 0

    while(True):
        sent_twice += 1
        if sent_twice >= 4 and sent_twice%2 == 0:
            att_obj.object.operation.operation = mapping_msgs.msg.CollisionObjectOperation.DETACH_AND_ADD_AS_OBJECT
        elif sent_twice > 4 and sent_twice%2 == 1:
            att_obj.object.operation.operation = mapping_msgs.msg.CollisionObjectOperation.ATTACH_AND_REMOVE_AS_OBJECT
        print 'sending'    
        att_obj.object.header.stamp = rospy.Time.now()
        att_pub.publish(att_obj)     
        
        r.sleep()

if __name__ == '__main__':

    test_add_convert_objects()
    

    
