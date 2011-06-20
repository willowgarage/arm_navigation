#! /usr/bin/env python

import roslib; roslib.load_manifest('collider')
roslib.load_manifest('actionlib')
roslib.load_manifest('pr2_controllers_msgs')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('sensor_msgs')

import rospy
from actionlib import SimpleActionClient, GoalStatus, SimpleActionServer
from pr2_controllers_msgs.msg import \
    PointHeadAction, PointHeadGoal, \
    SingleJointPositionAction, SingleJointPositionGoal
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2
from collision_environment_msgs.msg import MakeStaticCollisionMapAction, MakeStaticCollisionMapGoal
from collider.msg import ConstructStaticCollisionMapAction

import numpy as np

class ConstructStaticCollisionMapServer:

    def __init__(self,
                 node_name = 'construct_static_collision_map_server',
                 action_name = 'construct_static_collision_map_action',
                 head_topic = '/head_traj_controller/point_head_action',
                 static_collision_map_topic = '/make_static_collision_map'
                 
                 ):
                 
        rospy.init_node(node_name)
        rospy.loginfo ( 'waiting for SimpleActionClient: %s'%( head_topic )  )
        self.head_client = SimpleActionClient(head_topic,PointHeadAction )
        self.head_client.wait_for_server( )

        self.take_static_collision_map_client = SimpleActionClient(static_collision_map_topic, MakeStaticCollisionMapAction)

        self._action_server = SimpleActionServer(action_name, ConstructStaticCollisionMapAction, execute_cb=self.execute_cb)
        rospy.loginfo('ready')

    def execute_cb(self, goal):

        self.scan_table()
        goal = MakeStaticCollisionMapGoal()
        self.take_static_collision_map_client.send_goal(goal)

        self._action_server.set_succeeded()

    def move_head(self, x, y, z,
                  min_dur = 0.0,
                  max_velocity = 1.0,
                  frame_id = 'base_link',
                  timeout = 5.0):
        point = PointStamped()
        point.header.frame_id = frame_id
        point.header.stamp = rospy.Time.now()
        point.point.x, point.point.y, point.point.z = x, y, z
            
        goal = PointHeadGoal()
        goal.pointing_frame = 'head_plate_frame'
        goal.max_velocity = max_velocity
        goal.min_duration = rospy.Duration.from_sec( min_dur )
        goal.target = point
        
        self.head_client.send_goal( goal )
        self.head_client.wait_for_result( timeout = 
                                          rospy.Duration.from_sec( timeout ) )
        if not self.head_client.get_state() == GoalStatus.SUCCEEDED:
            rospy.logerr( 'can not move head to:\n %s'%( goal ) )
            return False
            
        return True

    def scan_table(self,
                   x = 0.5, 
                   y_min = -1.0, y_max = 1.0, 
                   z = 0.7, 
                   steps = 10, 
                   min_dur = 1.0, 
                   max_wait_time = 10.0
                   ):
        rospy.loginfo( 'scanning table with x = %5.3f, y_min = %5.3f, y_max = %5.3f, z = %5.3f, steps = %d'%( x, y_min, y_max, z, steps ) )
        Y = np.linspace( y_min, y_max, steps )
        
        for y in Y:
        
            if not self.move_head( x, y, z, min_dur ):
                return

        rospy.loginfo( 'done scanning table!' )

if __name__ == '__main__':
    ome = ConstructStaticCollisionMapServer()
    rospy.spin()
