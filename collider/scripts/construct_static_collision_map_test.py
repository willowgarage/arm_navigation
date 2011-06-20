#! /usr/bin/env python

import roslib; roslib.load_manifest('collider')
import rospy

import actionlib
from collider.msg import ConstructStaticCollisionMapAction, ConstructStaticCollisionMapGoal

if __name__ == '__main__':

    rospy.init_node('testing_static_collision_map_construction')

    construct_map_client = actionlib.SimpleActionClient("construct_static_collision_map_action", ConstructStaticCollisionMapAction)

    construct_map_client.wait_for_server()

    goal = ConstructStaticCollisionMapGoal()

    construct_map_client.send_goal(goal)
