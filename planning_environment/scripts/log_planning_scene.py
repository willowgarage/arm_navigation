#! /usr/bin/env python

PKG = 'planning_environment'

import roslib; roslib.load_manifest(PKG)
import rospy
import planning_environment_msgs.srv
import sys

from planning_environment_msgs.srv import LogPlanningScene, LogPlanningSceneRequest
from planning_environment_msgs.msg import PlanningScene

import mapping_msgs.msg
from mapping_msgs.msg import CollisionObject
from mapping_msgs.msg import AttachedCollisionObject
from geometric_shapes_msgs.msg import Shape
from geometry_msgs.msg import Pose

default_prefix = "/environment_server"

if __name__ == '__main__':
    rospy.init_node('log_planning_scene')

    print 'trying to connect to service ', default_prefix+'/log_planning_scene'

    rospy.wait_for_service(default_prefix+'/log_planning_scene')
    log_planning_scene_server = rospy.ServiceProxy(default_prefix+'/log_planning_scene', LogPlanningScene)
    
    print 'made service'

    table = CollisionObject()
    
    table.header.stamp = rospy.Time.now()
    table.header.frame_id = 'base_link'
    table.id = 'tabletop'
    table.operation.operation = mapping_msgs.msg.CollisionObjectOperation.ADD
    table.shapes = [Shape() for _ in range(1)]
    table.shapes[0].type = Shape.BOX
    table.shapes[0].dimensions = [float() for _ in range(3)]
    table.shapes[0].dimensions[0] = .7
    table.shapes[0].dimensions[1] = .7
    table.shapes[0].dimensions[2] = .10
    table.poses = [Pose() for _ in range(1)]
    table.poses[0].position.x = 1.0
    table.poses[0].position.y = 0
    table.poses[0].position.z = .55
    table.poses[0].orientation.x = 0
    table.poses[0].orientation.y = 0
    table.poses[0].orientation.z = 0
    table.poses[0].orientation.w = 1

    lpsr = LogPlanningSceneRequest()
    lpsr.package_name = 'move_arm'
    lpsr.filename = 'pole_scene.bag'
    #lpsr.planning_scene_diff.collision_objects.append(table)

    log_planning_scene_server.call(lpsr)

    exit()

