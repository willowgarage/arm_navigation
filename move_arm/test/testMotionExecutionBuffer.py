#! /usr/bin/env python

PKG = 'move_arm'

import roslib; roslib.load_manifest(PKG)
import rospy
import planning_environment_msgs.srv
import sys
import unittest
import actionlib
import actionlib_msgs
import math

import sensor_msgs.msg
import mapping_msgs.msg
from mapping_msgs.msg import CollisionObject
from motion_planning_msgs.msg import CollisionOperation
from geometric_shapes_msgs.msg import Shape
from geometry_msgs.msg import Pose, PointStamped
from move_arm_msgs.msg import MoveArmGoal, MoveArmAction
from tf import TransformListener
from motion_planning_msgs.msg import JointConstraint

padd_name = "ompl_planning/robot_padd"
extra_buffer = .1

class TestMotionExecutionBuffer(unittest.TestCase):

    def setUp(self):

        self.tf = TransformListener()
        
        self.move_arm_action_client = actionlib.SimpleActionClient("move_right_arm", MoveArmAction)

        obj_pub = rospy.Publisher('collision_object',CollisionObject)
        
        rospy.init_node('test_motion_execution_buffer')
        
        #let everything settle down
        rospy.sleep(20.)
      
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
        obj1.poses[0].position.x = .6
        obj1.poses[0].position.y = -.6
        obj1.poses[0].position.z = .75
        obj1.poses[0].orientation.x = 0
        obj1.poses[0].orientation.y = 0
        obj1.poses[0].orientation.z = 0
        obj1.poses[0].orientation.w = 1
        
        obj_pub.publish(obj1)

        rospy.sleep(1.0)

    def testMotionExecutionBuffer(self):
        
        global padd_name
        global extra_buffer
        
        allow_padd = rospy.get_param(padd_name)
        

        joint_names = ['%s_%s' % ('r', j) for j in ['shoulder_pan_joint', 'shoulder_lift_joint', 'upper_arm_roll_joint', 'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']]
        goal = MoveArmGoal()

        goal.motion_plan_request.goal_constraints.joint_constraints = [JointConstraint() for i in range(len(joint_names))]

        goal.motion_plan_request.group_name = "right_arm"
        goal.motion_plan_request.num_planning_attempts = 1
        goal.motion_plan_request.allowed_planning_time = rospy.Duration(5.)
        goal.motion_plan_request.planner_id = ""
        goal.planner_service_name = "ompl_planning/plan_kinematic_path"

        goal.motion_plan_request.goal_constraints.joint_constraints = [JointConstraint() for i in range(len(joint_names))]
        for i in range(len(joint_names)):
            goal.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = joint_names[i]
            goal.motion_plan_request.goal_constraints.joint_constraints[i].position = 0.0
            goal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.08
            goal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.08

        min_dist = 1000

        for z in range(10):

            if(z%2 == 0):
                goal.motion_plan_request.goal_constraints.joint_constraints[0].position = -2.0
                goal.motion_plan_request.goal_constraints.joint_constraints[3].position = -0.2
                goal.motion_plan_request.goal_constraints.joint_constraints[5].position = -0.2
            else:
                goal.motion_plan_request.goal_constraints.joint_constraints[0].position = 0.0
                goal.motion_plan_request.goal_constraints.joint_constraints[3].position = -0.2
                goal.motion_plan_request.goal_constraints.joint_constraints[5].position = -0.2

            for x in range(3):

                self.move_arm_action_client.send_goal(goal)

                r = rospy.Rate(.01)

                while True:
                    cur_state = self.move_arm_action_client.get_state()
                    if(cur_state != actionlib_msgs.msg.GoalStatus.ACTIVE and
                       cur_state != actionlib_msgs.msg.GoalStatus.PENDING):
                        break

                    #getting right finger tip link in base_link frame
                    t = self.tf.getLatestCommonTime("/base_link", "/r_gripper_r_finger_tip_link") 
                    finger_point = PointStamped()
                    finger_point.header.frame_id = "/r_gripper_r_finger_tip_link"
                    finger_point.header.stamp = t
                    finger_point.point.x = 0
                    finger_point.point.y = 0
                    finger_point.point.z = 0
                    finger_point_base = self.tf.transformPoint("base_link",finger_point)

                    distance = math.sqrt(math.pow(finger_point_base.point.x-.6,2)+math.pow(finger_point_base.point.y-.6,2))

                    # pole is .1 in diameter
                    distance -= .1

                    if distance < min_dist:
                        min_dist = distance

                end_state = self.move_arm_action_client.get_state()

                if(end_state == actionlib_msgs.msg.GoalStatus.SUCCEEDED): break

            rospy.loginfo("Min dist %d is %g",z,min_dist)

            #should be a .5 buffer, allowing .1 buffer
            self.failIf(min_dist < (allow_padd-extra_buffer))

            final_state = self.move_arm_action_client.get_state()

            self.assertEqual(final_state,  actionlib_msgs.msg.GoalStatus.SUCCEEDED)

if __name__ == '__main__':

    import rostest
    rostest.unitrun('test_motion_execution_buffer', 'test_motion_execution_buffer', TestMotionExecutionBuffer)


    
