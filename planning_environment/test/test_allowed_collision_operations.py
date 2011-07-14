#! /usr/bin/env python

import roslib; roslib.load_manifest('planning_environment')
import rospy
import arm_navigation_msgs.srv
import sys
import std_srvs.srv
import unittest
import mapping_msgs.msg
from mapping_msgs.msg import CollisionObject
from mapping_msgs.msg import AttachedCollisionObject
from arm_navigation_msgs.msg import CollisionOperation
from arm_navigation_msgs.msg import Shape
from geometry_msgs.msg import Pose
from std_msgs.msg import String

set_allowed_collision_service_name = "set_allowed_collisions"
revert_allowed_collision_service_name = "revert_allowed_collisions"
get_allowed_collision_service_name = "get_current_allowed_collision_matrix"
default_prefix = "/environment_server"

monk = False

class TestAllowedCollisionOperations(unittest.TestCase):

    def setUp(self):

        rospy.init_node('test_allowed_collisions')

        obj_pub = rospy.Publisher('collision_object',CollisionObject,latch=True)
        att_pub = rospy.Publisher('attached_collision_object', AttachedCollisionObject,latch=True)

        full_name = default_prefix+'/'+revert_allowed_collision_service_name
        rospy.wait_for_service(full_name)
        revert_allowed_collision_service_proxy = rospy.ServiceProxy(full_name, std_srvs.srv.Empty)   

        clear_all_att = AttachedCollisionObject()
        clear_all_att.object.header.stamp = rospy.Time.now()
        clear_all_att.object.header.frame_id = "base_link"
        clear_all_att.link_name = "all"
        clear_all_att.object.operation.operation = mapping_msgs.msg.CollisionObjectOperation.REMOVE

        clear_all_obj = CollisionObject()
        clear_all_obj.header.stamp = rospy.Time.now()
        clear_all_obj.header.frame_id = "base_link"
        clear_all_obj.id = "all"
        clear_all_obj.operation.operation = mapping_msgs.msg.CollisionObjectOperation.REMOVE
        
        empty_req = std_srvs.srv.EmptyRequest()

        revert_allowed_collision_service_proxy(empty_req)

    def tearDown(self):

        full_name = default_prefix+'/'+revert_allowed_collision_service_name
        rospy.wait_for_service(full_name)
        revert_allowed_collision_service_proxy = rospy.ServiceProxy(full_name, std_srvs.srv.Empty)   
        
        empty_req = std_srvs.srv.EmptyRequest()

        revert_allowed_collision_service_proxy(empty_req)

    def test_self_collisions(self):

        global set_allowed_collision_service_name
        global get_allowed_collision_service_name
        global revert_allowed_collision_service_name
        full_name = default_prefix+'/'+set_allowed_collision_service_name
        rospy.wait_for_service(full_name)
        set_allowed_collision_service_proxy = rospy.ServiceProxy(full_name, arm_navigation_msgs.srv.SetAllowedCollisions)  

        full_name = default_prefix+'/'+get_allowed_collision_service_name
        rospy.wait_for_service(full_name)
        get_allowed_collision_service_proxy = rospy.ServiceProxy(full_name, arm_navigation_msgs.srv.GetAllowedCollisionMatrix)

        full_name = default_prefix+'/'+revert_allowed_collision_service_name
        rospy.wait_for_service(full_name)
        revert_allowed_collision_service_proxy = rospy.ServiceProxy(full_name, std_srvs.srv.Empty)   

        #first we get the default
        
        get_mat = arm_navigation_msgs.srv.GetAllowedCollisionMatrixRequest()
    
        get_res = get_allowed_collision_service_proxy(get_mat)

        #entry for left gripper and right gripper should be false - just an example
        try:
            r_gripper_palm_link_index = get_res.matrix.link_names.index("r_gripper_palm_link")
        except ValueError:
            self.fail("No r_gripper_palm_link in AllowedCollisionMatrix link names")

        try:
            l_gripper_palm_link_index = get_res.matrix.link_names.index("l_gripper_palm_link")
        except ValueError:
            self.fail("No l_gripper_palm_link in AllowedCollisionMatrix link names")
         
        self.failIf(len(get_res.matrix.entries) <= r_gripper_palm_link_index)
        self.failIf(len(get_res.matrix.entries) <= l_gripper_palm_link_index)

        self.failIf(len(get_res.matrix.entries[l_gripper_palm_link_index].enabled) <= r_gripper_palm_link_index)
        self.failIf(len(get_res.matrix.entries[r_gripper_palm_link_index].enabled) <= l_gripper_palm_link_index)
         
        self.assertEqual(False, get_res.matrix.entries[l_gripper_palm_link_index].enabled[r_gripper_palm_link_index])
        self.assertEqual(False, get_res.matrix.entries[r_gripper_palm_link_index].enabled[l_gripper_palm_link_index])
         
        #if enable just this link, it should be true
        set_allowed_request = arm_navigation_msgs.srv.SetAllowedCollisionsRequest()   
        set_allowed_request.ord.collision_operations = [CollisionOperation() for _ in range(1)]
        set_allowed_request.ord.collision_operations[0].object1 = "r_gripper_palm_link"
        set_allowed_request.ord.collision_operations[0].object2 = "l_gripper_palm_link"
        set_allowed_request.ord.collision_operations[0].operation = CollisionOperation.DISABLE

        set_allowed_collision_service_proxy(set_allowed_request)

        rospy.sleep(1.0)

        get_res = get_allowed_collision_service_proxy(get_mat)

        self.assertEqual(True, get_res.matrix.entries[l_gripper_palm_link_index].enabled[r_gripper_palm_link_index])
        self.assertEqual(True, get_res.matrix.entries[r_gripper_palm_link_index].enabled[l_gripper_palm_link_index])
        
        #now we enable r_end_effector versus l_end_effector, and it should be false again
        set_allowed_request.ord.collision_operations[0].object1 = "r_end_effector"
        set_allowed_request.ord.collision_operations[0].object2 = "l_end_effector"
        set_allowed_request.ord.collision_operations[0].operation = CollisionOperation.ENABLE

        set_allowed_collision_service_proxy(set_allowed_request)

        rospy.sleep(1.0)

        get_res = get_allowed_collision_service_proxy(get_mat)

        self.assertEqual(False, get_res.matrix.entries[l_gripper_palm_link_index].enabled[r_gripper_palm_link_index])
        self.assertEqual(False, get_res.matrix.entries[r_gripper_palm_link_index].enabled[l_gripper_palm_link_index])

    def test_object_collisions(self):
        global set_allowed_collision_service_name
        global get_allowed_collision_service_name
        global revert_allowed_collision_service_name
        full_name = default_prefix+'/'+set_allowed_collision_service_name
        rospy.wait_for_service(full_name)
        set_allowed_collision_service_proxy = rospy.ServiceProxy(full_name, arm_navigation_msgs.srv.SetAllowedCollisions)  

        full_name = default_prefix+'/'+get_allowed_collision_service_name
        rospy.wait_for_service(full_name)
        get_allowed_collision_service_proxy = rospy.ServiceProxy(full_name, arm_navigation_msgs.srv.GetAllowedCollisionMatrix)

        full_name = default_prefix+'/'+revert_allowed_collision_service_name
        rospy.wait_for_service(full_name)
        revert_allowed_collision_service_proxy = rospy.ServiceProxy(full_name, std_srvs.srv.Empty)   

        obj_pub = rospy.Publisher('collision_object',CollisionObject)
        att_pub = rospy.Publisher('attached_collision_object', AttachedCollisionObject)

        rospy.init_node('test_allowed_collisions')

        obj1 = CollisionObject()

        obj1.header.stamp = rospy.Time.now()
        obj1.header.frame_id = "base_link"
        obj1.id = "obj1";
        obj1.operation.operation = mapping_msgs.msg.CollisionObjectOperation.ADD
        obj1.shapes = [Shape() for _ in range(2)]
        obj1.shapes[0].type = Shape.BOX
        obj1.shapes[0].dimensions = [float() for _ in range(3)]
        obj1.shapes[0].dimensions[0] = 1.0
        obj1.shapes[0].dimensions[1] = 1.0
        obj1.shapes[0].dimensions[2] = .05
        obj1.shapes[1].type = Shape.BOX
        obj1.shapes[1].dimensions = [float() for _ in range(3)]
        obj1.shapes[1].dimensions[0] = 1.0
        obj1.shapes[1].dimensions[1] = 1.0
        obj1.shapes[1].dimensions[2] = .05
        obj1.poses = [Pose() for _ in range(2)]
        obj1.poses[0].position.x = 1.0
        obj1.poses[0].position.y = 0
        obj1.poses[0].position.z = .5
        obj1.poses[0].orientation.x = 0
        obj1.poses[0].orientation.y = 0
        obj1.poses[0].orientation.z = 0
        obj1.poses[0].orientation.w = 1
        obj1.poses[1].position.x = 1.0
        obj1.poses[1].position.y = 0
        obj1.poses[1].position.z = .75
        obj1.poses[1].orientation.x = 0
        obj1.poses[1].orientation.y = 0
        obj1.poses[1].orientation.z = 0
        obj1.poses[1].orientation.w = 1
        
        obj_pub.publish(obj1)
        
        obj2 = CollisionObject();
        
        obj2.header.stamp = rospy.Time.now()
        obj2.header.frame_id = "base_link"
        obj2.id = "obj2";
        obj2.operation.operation = mapping_msgs.msg.CollisionObjectOperation.ADD
        obj2.shapes = [Shape() for _ in range(1)]
        obj2.shapes[0].type = Shape.BOX
        obj2.shapes[0].dimensions = [float() for _ in range(3)]
        obj2.shapes[0].dimensions[0] = 1.0
        obj2.shapes[0].dimensions[1] = 1.0
        obj2.shapes[0].dimensions[2] = .05
        obj2.poses = [Pose() for _ in range(1)]
        obj2.poses[0].position.x = .15
        obj2.poses[0].position.y = 0
        obj2.poses[0].position.z = .5
        obj2.poses[0].orientation.x = 0
        obj2.poses[0].orientation.y = 0
        obj2.poses[0].orientation.z = 0
        obj2.poses[0].orientation.w = 1
        
        obj_pub.publish(obj2)

        rospy.sleep(2.0)

        get_mat = arm_navigation_msgs.srv.GetAllowedCollisionMatrixRequest()
    
        get_res = get_allowed_collision_service_proxy(get_mat)

        try:
            r_gripper_palm_link_index = get_res.matrix.link_names.index("r_gripper_palm_link")
        except ValueError:
            self.fail("No r_gripper_palm_link in AllowedCollisionMatrix link names")

        try:
            obj1_index = get_res.matrix.link_names.index("obj1")
        except ValueError:
            self.fail("No obj1 in AllowedCollisionMatrix link names")
         
        self.failIf(len(get_res.matrix.entries) <= r_gripper_palm_link_index)
        self.failIf(len(get_res.matrix.entries) <= obj1_index)

        self.failIf(len(get_res.matrix.entries[obj1_index].enabled) <= r_gripper_palm_link_index)
        self.failIf(len(get_res.matrix.entries[r_gripper_palm_link_index].enabled) <= obj1_index)
         
        self.assertEqual(False, get_res.matrix.entries[obj1_index].enabled[r_gripper_palm_link_index])
        self.assertEqual(False, get_res.matrix.entries[r_gripper_palm_link_index].enabled[obj1_index])
        
        #now disable between all objects

        #if enable just this link, it should be true
        set_allowed_request = arm_navigation_msgs.srv.SetAllowedCollisionsRequest()   
        set_allowed_request.ord.collision_operations = [CollisionOperation() for _ in range(1)]
        set_allowed_request.ord.collision_operations[0].object1 = "r_gripper_palm_link"
        set_allowed_request.ord.collision_operations[0].object2 = CollisionOperation.COLLISION_SET_OBJECTS
        set_allowed_request.ord.collision_operations[0].operation = CollisionOperation.DISABLE

        set_allowed_collision_service_proxy(set_allowed_request)

        rospy.sleep(1.)

        get_res = get_allowed_collision_service_proxy(get_mat)

        self.assertEqual(True, get_res.matrix.entries[obj1_index].enabled[r_gripper_palm_link_index])
        self.assertEqual(True, get_res.matrix.entries[r_gripper_palm_link_index].enabled[obj1_index])

    def test_attached_object_collision_operations(self):

        global set_allowed_collision_service_name
        global get_allowed_collision_service_name
        global revert_allowed_collision_service_name
        full_name = default_prefix+'/'+set_allowed_collision_service_name
        rospy.wait_for_service(full_name)
        set_allowed_collision_service_proxy = rospy.ServiceProxy(full_name, arm_navigation_msgs.srv.SetAllowedCollisions)  

        full_name = default_prefix+'/'+get_allowed_collision_service_name
        rospy.wait_for_service(full_name)
        get_allowed_collision_service_proxy = rospy.ServiceProxy(full_name, arm_navigation_msgs.srv.GetAllowedCollisionMatrix)

        full_name = default_prefix+'/'+revert_allowed_collision_service_name
        rospy.wait_for_service(full_name)
        revert_allowed_collision_service_proxy = rospy.ServiceProxy(full_name, std_srvs.srv.Empty)   

        att_pub = rospy.Publisher('attached_collision_object', AttachedCollisionObject)

        rospy.init_node('test_allowed_collisions')

        att_object = AttachedCollisionObject();
        att_object.object.header.stamp = rospy.Time.now()
        att_object.object.header.frame_id = "base_link"
        att_object.link_name = "r_gripper_r_finger_tip_link"
        att_object.touch_links = [str() for _ in range(1)]
        att_object.touch_links[0] = "r_gripper_palm_link"
        att_object.object.operation.operation = mapping_msgs.msg.CollisionObjectOperation.ADD
        att_object.object = CollisionObject();
        
        att_object.object.header.stamp = rospy.Time.now()
        att_object.object.header.frame_id = "base_link"
        att_object.object.id = "obj3"
        att_object.object.shapes = [Shape() for _ in range(1)]
        att_object.object.shapes[0].type = Shape.BOX
        att_object.object.shapes[0].dimensions = [float() for _ in range(3)]
        att_object.object.shapes[0].dimensions[0] = .2
        att_object.object.shapes[0].dimensions[1] = 0.3
        att_object.object.shapes[0].dimensions[2] = .05
        att_object.object.poses = [Pose() for _ in range(1)]
        att_object.object.poses[0].position.x = .15
        att_object.object.poses[0].position.y = 0
        att_object.object.poses[0].position.z = 1.0
        att_object.object.poses[0].orientation.x = 0
        att_object.object.poses[0].orientation.y = 0
        att_object.object.poses[0].orientation.z = 0
        att_object.object.poses[0].orientation.w = 1
        att_pub.publish(att_object)

        rospy.sleep(5.0)

        get_mat = arm_navigation_msgs.srv.GetAllowedCollisionMatrixRequest()
    
        get_res = get_allowed_collision_service_proxy(get_mat)

        try:
            r_gripper_palm_link_index = get_res.matrix.link_names.index("r_gripper_palm_link")
        except ValueError:
            self.fail("No r_gripper_palm_link in AllowedCollisionMatrix link names")

        try:
            r_gripper_r_finger_tip_link_index = get_res.matrix.link_names.index("r_gripper_r_finger_tip_link")
        except ValueError:
            self.fail("No r_gripper_r_finger_tip_link in AllowedCollisionMatrix link names")

        try:
            obj3_index = get_res.matrix.link_names.index("obj3")
        except ValueError:
            self.fail("No obj3 in AllowedCollisionMatrix link names") 

        self.assertEqual(True, get_res.matrix.entries[obj3_index].enabled[r_gripper_r_finger_tip_link_index])
        self.assertEqual(True, get_res.matrix.entries[r_gripper_r_finger_tip_link_index].enabled[obj3_index])

        self.assertEqual(True, get_res.matrix.entries[obj3_index].enabled[r_gripper_palm_link_index])
        self.assertEqual(True, get_res.matrix.entries[r_gripper_palm_link_index].enabled[obj3_index])
        
        #this should overwrite touch links
        set_allowed_request = arm_navigation_msgs.srv.SetAllowedCollisionsRequest()   
        set_allowed_request.ord.collision_operations = [CollisionOperation() for _ in range(1)]
        set_allowed_request.ord.collision_operations[0].object1 = "obj3"
        set_allowed_request.ord.collision_operations[0].object2 = "r_gripper_palm_link"
        set_allowed_request.ord.collision_operations[0].operation = CollisionOperation.ENABLE

        set_allowed_collision_service_proxy(set_allowed_request)

        rospy.sleep(1.)

        get_res = get_allowed_collision_service_proxy(get_mat)

        self.assertEqual(False, get_res.matrix.entries[obj3_index].enabled[r_gripper_palm_link_index])
        self.assertEqual(False, get_res.matrix.entries[r_gripper_palm_link_index].enabled[obj3_index])

        set_allowed_request.ord.collision_operations[0].object1 = "r_gripper_palm_link"
        set_allowed_request.ord.collision_operations[0].object2 = CollisionOperation.COLLISION_SET_ATTACHED_OBJECTS
        set_allowed_request.ord.collision_operations[0].operation = CollisionOperation.DISABLE

        set_allowed_collision_service_proxy(set_allowed_request)

        rospy.sleep(1.)

        get_res = get_allowed_collision_service_proxy(get_mat)

        self.assertEqual(True, get_res.matrix.entries[obj3_index].enabled[r_gripper_palm_link_index])
        self.assertEqual(True, get_res.matrix.entries[r_gripper_palm_link_index].enabled[obj3_index])

if __name__ == '__main__':
    import rostest
    rostest.unitrun('test_allowed_collisions', 'test_allowed_collisions', TestAllowedCollisionOperations)
