#! /usr/bin/env python

PKG = 'planning_environment'

import roslib; roslib.load_manifest(PKG)
import rospy
import planning_environment_msgs.srv
import sys
import unittest


import sensor_msgs.msg
import mapping_msgs.msg
from mapping_msgs.msg import CollisionObject
from mapping_msgs.msg import AttachedCollisionObject
from motion_planning_msgs.msg import CollisionOperation
from geometric_shapes_msgs.msg import Shape
from geometry_msgs.msg import Pose

get_collision_objects_service_name = "get_collision_objects"

def test_add_convert_objects():

    obj_pub = rospy.Publisher('collision_object',CollisionObject)

    rospy.init_node('test_collision_objects')

    #let everything settle down
    rospy.sleep(5.)
      
    obj1 = CollisionObject()
    
    obj1.header.stamp = rospy.Time.now()
    obj1.header.frame_id = "torso_lift_link"
    obj1.id = "obj1";
    obj1.operation.operation = mapping_msgs.msg.CollisionObjectOperation.ADD
    obj1.shapes = [Shape() for _ in range(1)]
    obj1.shapes[0].type = Shape.CYLINDER
    obj1.shapes[0].dimensions = [float() for _ in range(2)]
    obj1.shapes[0].dimensions[0] = .1
    obj1.shapes[0].dimensions[1] = 1.5
    obj1.poses = [Pose() for _ in range(1)]
    obj1.poses[0].position.x = .75
    obj1.poses[0].position.y = -.188
    obj1.poses[0].position.z = .0
    obj1.poses[0].orientation.x = 0
    obj1.poses[0].orientation.y = 0
    obj1.poses[0].orientation.z = 0
    obj1.poses[0].orientation.w = 1

    obj_pub.publish(obj1)

if __name__ == '__main__':

    test_add_convert_objects()
    

    
