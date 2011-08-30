#! /usr/bin/env python

PKG = 'planning_environment'

import roslib; roslib.load_manifest(PKG)
import rospy
import arm_navigation_msgs.srv
import sys
import unittest


import sensor_msgs.msg
import arm_navigation_msgs.msg
from arm_navigation_msgs.msg import CollisionObject
from arm_navigation_msgs.msg import AttachedCollisionObject
from arm_navigation_msgs.msg import CollisionOperation
from arm_navigation_msgs.msg import Shape
from geometry_msgs.msg import Pose

get_collision_objects_service_name = "get_collision_objects"

def test_add_convert_objects():

    obj_pub = rospy.Publisher('collision_object',CollisionObject)
    att_pub = rospy.Publisher('attached_collision_object',AttachedCollisionObject)

    rospy.init_node('test_collision_objects')

    #let everything settle down
    rospy.sleep(5.)
      
    obj1 = CollisionObject()
    
    obj1.header.stamp = rospy.Time.now()
    obj1.header.frame_id = "base_link"
    obj1.id = "obj1";
    obj1.operation.operation = arm_navigation_msgs.msg.CollisionObjectOperation.ADD
    obj1.shapes = [Shape() for _ in range(1)]
    obj1.shapes[0].type = Shape.BOX
    obj1.shapes[0].dimensions = [float() for _ in range(3)]
    obj1.shapes[0].dimensions[0] = .1
    obj1.shapes[0].dimensions[1] = .1
    obj1.shapes[0].dimensions[2] = .75
    obj1.poses = [Pose() for _ in range(1)]
    obj1.poses[0].position.x = .6
    obj1.poses[0].position.y = -.6
    obj1.poses[0].position.z = .375
    obj1.poses[0].orientation.x = 0
    obj1.poses[0].orientation.y = 0
    obj1.poses[0].orientation.z = 0
    obj1.poses[0].orientation.w = 1

    att_obj = AttachedCollisionObject()
    att_obj.link_name = "r_gripper_palm_link"
    att_obj.touch_links = ['r_gripper_palm_link', 'r_gripper_r_finger_link', 'r_gripper_l_finger_link',
                           'r_gripper_r_finger_tip_link', 'r_gripper_l_finger_tip_link', 'r_wrist_roll_link', 'r_wrist_flex_link', 'r_forearm_link', 'r_gripper_motor_accelerometer_link']
    obj2 = CollisionObject()
    
    obj2.header.stamp = rospy.Time.now()
    obj2.header.frame_id = "r_gripper_palm_link"
    obj2.id = "obj2";
    obj2.operation.operation = arm_navigation_msgs.msg.CollisionObjectOperation.ADD
    obj2.shapes = [Shape() for _ in range(1)]
    obj2.shapes[0].type = Shape.CYLINDER
    obj2.shapes[0].dimensions = [float() for _ in range(2)]
    obj2.shapes[0].dimensions[0] = .025
    obj2.shapes[0].dimensions[1] = .5
    obj2.poses = [Pose() for _ in range(1)]
    obj2.poses[0].position.x = .12
    obj2.poses[0].position.y = 0
    obj2.poses[0].position.z = 0
    obj2.poses[0].orientation.x = 0
    obj2.poses[0].orientation.y = 0
    obj2.poses[0].orientation.z = 0
    obj2.poses[0].orientation.w = 1
    att_obj.object = obj2
    r = rospy.Rate(.1)

    while(True):
        obj1.header.stamp = rospy.Time.now()
        obj_pub.publish(obj1)
        att_obj.object.header.stamp = rospy.Time.now()
        att_pub.publish(att_obj)
        r.sleep()

if __name__ == '__main__':

    test_add_convert_objects()
    

    
