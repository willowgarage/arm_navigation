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
default_prefix = "/environment_server"

class TestCollisionObjects(unittest.TestCase):

    def test_add_convert_objects(self):

        rospy.init_node('test_collision_objects')

        obj_pub = rospy.Publisher('collision_object',CollisionObject,latch=True)
        att_pub = rospy.Publisher('attached_collision_object', AttachedCollisionObject,latch=True)

        global get_collision_objects_service_name 
        full_name = default_prefix+'/'+get_collision_objects_service_name
        rospy.wait_for_service(full_name)
        get_collision_objects_service = rospy.ServiceProxy(full_name, arm_navigation_msgs.srv.GetCollisionObjects)  

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

        #tripling for publish wonkitude

        att_pub.publish(clear_all_att)

        obj_pub.publish(clear_all_obj)

        rospy.sleep(2.)
      
        get_collision_objects_req = arm_navigation_msgs.srv.GetCollisionObjectsRequest()
        try:
            get_collision_objects_res = get_collision_objects_service(get_collision_objects_req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        #should have no objects
        self.assertEquals(0, len(get_collision_objects_res.collision_objects))
        self.assertEquals(0, len(get_collision_objects_res.attached_collision_objects))
        
        # first add a couple objects to the environment
        
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
        
        rospy.sleep(2.)

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

        rospy.loginfo("WTF")
        
        rospy.sleep(2.)

        get_collision_objects_req = arm_navigation_msgs.srv.GetCollisionObjectsRequest()
        
        get_collision_objects_res = get_collision_objects_service(get_collision_objects_req)
        
        #should have two objects and no attached objects
        self.assertEquals(2, len(get_collision_objects_res.collision_objects))
        self.assertEquals(0, len(get_collision_objects_res.attached_collision_objects))

        conv_object = AttachedCollisionObject();
        conv_object.object.header.stamp = rospy.Time.now()
        conv_object.object.header.frame_id = "base_link"
        conv_object.link_name = "base_link"
        conv_object.object.id = "obj2"
        conv_object.object.operation.operation = mapping_msgs.msg.CollisionObjectOperation.ATTACH_AND_REMOVE_AS_OBJECT
        att_pub.publish(conv_object)

        rospy.sleep(2.)

        get_collision_objects_req = arm_navigation_msgs.srv.GetCollisionObjectsRequest()
        
        get_collision_objects_res = get_collision_objects_service(get_collision_objects_req)
        
        #should have one object non-attached and one attached
        self.assertEquals(1, len(get_collision_objects_res.collision_objects))
        self.assertEquals(1, len(get_collision_objects_res.attached_collision_objects))
        self.assertEquals("obj1", get_collision_objects_res.collision_objects[0].id)
        self.assertEquals("obj2", get_collision_objects_res.attached_collision_objects[0].object.id)

        #now we add attach another object directly to a different link
        att_object = AttachedCollisionObject();
        att_object.object.header.stamp = rospy.Time.now()
        att_object.object.header.frame_id = "base_link"
        att_object.link_name = "r_gripper_r_finger_tip_link"
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
    
        rospy.sleep(2.)

        get_collision_objects_req = arm_navigation_msgs.srv.GetCollisionObjectsRequest()
        
        get_collision_objects_res = get_collision_objects_service(get_collision_objects_req)
        
        #should have one object and two attached
        self.assertEquals(1, len(get_collision_objects_res.collision_objects))
        self.assertEquals(2, len(get_collision_objects_res.attached_collision_objects))
        self.assertEquals("obj1", get_collision_objects_res.collision_objects[0].id)

        #deleting those attached to base link
        del_object = AttachedCollisionObject();
        del_object.object.header.stamp = rospy.Time.now()
        del_object.object.header.frame_id = "base_link"
        del_object.link_name = "base_link"
        del_object.object.id = "all"
        del_object.object.operation.operation = mapping_msgs.msg.CollisionObjectOperation.REMOVE
        att_pub.publish(del_object)

        rospy.sleep(2.)

        get_collision_objects_req = arm_navigation_msgs.srv.GetCollisionObjectsRequest()
        
        get_collision_objects_res = get_collision_objects_service(get_collision_objects_req)
        
        #should have one object and two attached
        self.assertEquals(1, len(get_collision_objects_res.collision_objects))
        self.assertEquals(1, len(get_collision_objects_res.attached_collision_objects))
        self.assertEquals("obj1", get_collision_objects_res.collision_objects[0].id)
        self.assertEquals("obj3", get_collision_objects_res.attached_collision_objects[0].object.id)

        #converting back to object
        back_object = AttachedCollisionObject();
        back_object.object.header.stamp = rospy.Time.now()
        back_object.object.header.frame_id = "base_link"
        back_object.link_name = "r_gripper_r_finger_tip_link"
        back_object.object.id = "obj3"
        back_object.object.operation.operation = mapping_msgs.msg.CollisionObjectOperation.DETACH_AND_ADD_AS_OBJECT
        att_pub.publish(back_object)

        rospy.sleep(2.)

        get_collision_objects_req = arm_navigation_msgs.srv.GetCollisionObjectsRequest()
        
        get_collision_objects_res = get_collision_objects_service(get_collision_objects_req)
        
        #should have one object and two attached
        self.assertEquals(2, len(get_collision_objects_res.collision_objects))
        self.assertEquals(0, len(get_collision_objects_res.attached_collision_objects))

if __name__ == '__main__':
    import rostest
    rostest.unitrun('test_collision_objects', 'test_collision_objects', TestCollisionObjects)

    
