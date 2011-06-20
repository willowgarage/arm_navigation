/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** \author Ioan Sucan */

/**

@b DisplayPlannerCollisionModel is a node that displays the state of
the robot's collision space, as seen by the planner

**/

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <visualization_msgs/Marker.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes_msgs/Shape.h>
#include <mapping_msgs/CollisionMap.h>
#include <planning_environment_msgs/GetCollisionObjects.h>

#include <iostream>
#include <sstream>
#include <string>
#include <map>

static const std::string GET_OBJECTS_SERVICE_NAME = "get_collision_objects";
static const std::string COLLISION_MARKER_TOPIC = "collision_model_markers";

static const unsigned int FAILURE_MAX = 5;

class DisplayPlannerCollisionModel
{
public:

  DisplayPlannerCollisionModel(void): nh_("~")
  {
    ros::NodeHandle root_node;
                                                  
    nh_.param<std::string>("prefix", prefix_, "environment_server");
    nh_.param<bool>("skip_collision_map", skip_collision_map_, false);

    visualizationMarkerPublisher_ = root_node.advertise<visualization_msgs::Marker>(COLLISION_MARKER_TOPIC+prefix_, 10240);

    if(!skip_collision_map_) {
      collision_map_publisher_ = root_node.advertise<mapping_msgs::CollisionMap>("collision_rebroadcast_"+prefix_, 10240);
    }

    std::string service_name = prefix_+"/"+GET_OBJECTS_SERVICE_NAME;

    ROS_INFO_STREAM("Going to wait for service " << service_name);
    
    ros::service::waitForService(service_name);

    ROS_INFO_STREAM("Connected to " << service_name);

    get_objects_service_client_ = root_node.serviceClient<planning_environment_msgs::GetCollisionObjects>(service_name);

    object_color_.a = 0.5;
    object_color_.r = 0.1;
    object_color_.g = 0.8;
    object_color_.b = 0.3;

    attached_color_.a = 0.5;
    attached_color_.r = 0.6;
    attached_color_.g = 0.4;
    attached_color_.b = 0.3;

    failure_count_= 0;

  }

  virtual ~DisplayPlannerCollisionModel(void)
  {
  }
    
  void run(void)
  {
    unsigned int update = 0;

    ros::Rate r(4);

    while(ros::NodeHandle().ok()) {
      ros::spinOnce();
      if(++update%4 == 0) {
        publishMapObjects();
      }
      r.sleep();
    }
  }
    
protected:

  void publishMapObjects() {

    planning_environment_msgs::GetCollisionObjects::Request req;
    planning_environment_msgs::GetCollisionObjects::Response res;

    req.include_points = !skip_collision_map_;

    bool serviceOk = get_objects_service_client_.call(req,res);

    if(!serviceOk) {
      failure_count_++;
      if(failure_count_ < FAILURE_MAX) {
        ROS_WARN("Display failed to call get objects service");
      }
    } else {
      failure_count_ = 0;
    }
    if(failure_count_ > 0 && failure_count_ < FAILURE_MAX) {
      ROS_WARN("Display failed to call get objects service");
    }

    if(res.points.boxes.size() != 0) {
      ROS_DEBUG_STREAM("Publishing " << res.points.boxes.size() << " points."); 
      collision_map_publisher_.publish(res.points);
    }

    ROS_DEBUG_STREAM("Got " << res.collision_objects.size() << " objects from collision space");
    ROS_DEBUG_STREAM("Got " << res.attached_collision_objects.size() << " attached objects from collision space");

    std::map<std::string, unsigned int> current_num;

    //resets the current list to not found in the new message
    for(std::map<std::string, unsigned int>::iterator it = cur_collision_objects_.begin();
        it != cur_collision_objects_.end();
        it++) {
      current_num[it->first] = 0;
    }

    for(std::vector<mapping_msgs::CollisionObject>::iterator it = res.collision_objects.begin();
        it != res.collision_objects.end();
        it++) {
      current_num[(*it).id] = (*it).shapes.size();
    }

    std::vector<std::string> er_ids;
    std::map<std::string, unsigned int>::iterator it = cur_collision_objects_.begin();
    while(it != cur_collision_objects_.end()) {
      //if fewer (including zero, need to delete)
      if(current_num[it->first] < it->second) {
        mapping_msgs::CollisionObject obj;
        obj.header.frame_id = "base_link";
        obj.header.stamp = ros::Time::now();
        std::string id = prefix_+"---"+it->first;
        obj.operation.operation = mapping_msgs::CollisionObjectOperation::REMOVE;
        publishObjects(obj, id, object_color_, it->second);
        //entirely gone
        if(current_num[it->first] == 0) {
          er_ids.push_back(it->first);
        }
      }
      it++;
    }
    for(std::vector<std::string>::iterator ir = er_ids.begin();
        ir != er_ids.end();
        ir++) {
      cur_collision_objects_.erase(*ir);
    }

    for(std::vector<mapping_msgs::CollisionObject>::iterator it = res.collision_objects.begin();
        it != res.collision_objects.end();
        it++) {
      cur_collision_objects_[(*it).id] = (*it).shapes.size();
      std::string id = prefix_+"---"+(*it).id;
      publishObjects(*it, id, object_color_);
    }

    //now dealing with attached bodies
    current_num.clear();

    //resets the current list to not found in the new message
    for(std::map<std::string, unsigned int>::iterator it = cur_attached_objects_.begin();
        it != cur_attached_objects_.end();
        it++) {
      current_num[it->first] = 0;
    }

    for(std::vector<mapping_msgs::AttachedCollisionObject>::iterator it = res.attached_collision_objects.begin();
        it != res.attached_collision_objects.end();
        it++) {
      std::string id = (*it).link_name+"+"+(*it).object.id;
      current_num[id] = (*it).object.shapes.size();
    }

    er_ids.clear();
    it = cur_attached_objects_.begin();
    while(it != cur_attached_objects_.end()) {
      //if fewer (including zero, need to delete)
      if(current_num[it->first] < it->second) {
        mapping_msgs::CollisionObject obj;
        obj.header.frame_id = "base_link";
        obj.header.stamp = ros::Time::now();
        std::string id = prefix_+"---"+it->first;
        obj.operation.operation = mapping_msgs::CollisionObjectOperation::REMOVE;
        publishObjects(obj, id, attached_color_, it->second);
        //entirely gone
        if(current_num[it->first] == 0) {
          er_ids.push_back(it->first);
        }
      }
      it++;
    }
    for(std::vector<std::string>::iterator ir = er_ids.begin();
        ir != er_ids.end();
        ir++) {
      cur_attached_objects_.erase(*ir);
    }

    for(std::vector<mapping_msgs::AttachedCollisionObject>::iterator it = res.attached_collision_objects.begin();
        it != res.attached_collision_objects.end();
        it++) {
      std::string id1 = (*it).link_name+"+"+(*it).object.id;
      cur_attached_objects_[id1] = (*it).object.shapes.size();
      std::string id2 = prefix_+"---"+id1;
      publishObjects((*it).object, id2, attached_color_);
    }
  } 
 
  void publishObjects(const mapping_msgs::CollisionObject& collisionObject, const std::string id, const std_msgs::ColorRGBA color, unsigned int num = 0) {
    visualization_msgs::Marker mk;

    if (collisionObject.operation.operation == mapping_msgs::CollisionObjectOperation::REMOVE) {
      for(unsigned int i = 0; i < num; i++) {
        mk.ns = id;
        mk.id = i;
        mk.header = collisionObject.header;
        mk.header.stamp = ros::Time::now();
        mk.action =  visualization_msgs::Marker::DELETE;
        mk.color = color;
        visualizationMarkerPublisher_.publish(mk);
        ROS_DEBUG_STREAM("Sending delete for " << mk.ns << " id " << i);
      }
      return;
    }
    
    if (collisionObject.operation.operation == mapping_msgs::CollisionObjectOperation::ADD) {
      for(unsigned int i = 0; i < collisionObject.shapes.size(); i++) {
        mk.ns = id;
        mk.id = i;
        mk.header = collisionObject.header;
        mk.header.stamp = ros::Time::now();
        mk.action = visualization_msgs::Marker::ADD;
        setObject(collisionObject.shapes[i], mk);
        mk.pose = collisionObject.poses[i];
        mk.color = color;
        visualizationMarkerPublisher_.publish(mk);
      }
    }
  }
    
private:

  void setObject(const geometric_shapes_msgs::Shape &obj, visualization_msgs::Marker &mk)
  {
    switch (obj.type)
    {
    case geometric_shapes_msgs::Shape::SPHERE:
      mk.type = visualization_msgs::Marker::SPHERE;
      mk.scale.x = mk.scale.y = mk.scale.z = obj.dimensions[0] * 2.0;
      break;
	    
    case geometric_shapes_msgs::Shape::BOX:
      mk.type = visualization_msgs::Marker::CUBE;
      mk.scale.x = obj.dimensions[0];
      mk.scale.y = obj.dimensions[1];
      mk.scale.z = obj.dimensions[2];
      break;

    case geometric_shapes_msgs::Shape::CYLINDER:
      mk.type = visualization_msgs::Marker::CYLINDER;
      mk.scale.x = obj.dimensions[0] * 2.0;
      mk.scale.y = obj.dimensions[0] * 2.0;
      mk.scale.z = obj.dimensions[1];
      break;

    case geometric_shapes_msgs::Shape::MESH:
      mk.type = visualization_msgs::Marker::LINE_LIST;
      mk.scale.x = mk.scale.y = mk.scale.z = 0.001;
      {
        unsigned int nt = obj.triangles.size() / 3;
        for (unsigned int i = 0 ; i < nt ; ++i)
        {
          mk.points.push_back(obj.vertices[obj.triangles[3*i]]);
          mk.points.push_back(obj.vertices[obj.triangles[3*i+ 1]]);
          mk.points.push_back(obj.vertices[obj.triangles[3*i]]);
          mk.points.push_back(obj.vertices[obj.triangles[3*i+2]]);
          mk.points.push_back(obj.vertices[obj.triangles[3*i+1]]);
          mk.points.push_back(obj.vertices[obj.triangles[3*i+2]]);
        }
      }
	    
      break;
	    
    default:
      ROS_ERROR("Unknown object type: %d", (int)obj.type);
    }
  }

  void setObject(const shapes::Shape *obj, visualization_msgs::Marker &mk)
  {
    switch (obj->type)
    {
    case shapes::SPHERE:
      mk.type = visualization_msgs::Marker::SPHERE;
      mk.scale.x = mk.scale.y = mk.scale.z = static_cast<const shapes::Sphere*>(obj)->radius * 2.0;
      break;
	    
    case shapes::BOX:
      mk.type = visualization_msgs::Marker::CUBE;
      {
        const double *size = static_cast<const shapes::Box*>(obj)->size;
        mk.scale.x = size[0];
        mk.scale.y = size[1];
        mk.scale.z = size[2];
      }
      break;

    case shapes::CYLINDER:
      mk.type = visualization_msgs::Marker::CYLINDER;
      mk.scale.x = static_cast<const shapes::Cylinder*>(obj)->radius * 2.0;
      mk.scale.y = mk.scale.x;
      mk.scale.z = static_cast<const shapes::Cylinder*>(obj)->length;
      break;

    case shapes::MESH:
      mk.type = visualization_msgs::Marker::LINE_LIST;
      mk.scale.x = mk.scale.y = mk.scale.z = 0.001;
      {	   
        const shapes::Mesh *mesh = static_cast<const shapes::Mesh*>(obj);
        unsigned int nt = mesh->triangleCount / 3;
        for (unsigned int i = 0 ; i < nt ; ++i)
        {
          unsigned int v = mesh->triangles[3*i];
          geometry_msgs::Point pt1;
          pt1.x = mesh->vertices[v];
          pt1.y = mesh->vertices[v+1];
          pt1.z = mesh->vertices[v+2];
          mk.points.push_back(pt1);

          v = mesh->triangles[3*i + 1];
          geometry_msgs::Point pt2;
          pt2.x = mesh->vertices[v];
          pt2.y = mesh->vertices[v+1];
          pt2.z = mesh->vertices[v+2];
          mk.points.push_back(pt2);

          mk.points.push_back(pt1);

          v = mesh->triangles[3*i + 2];
          geometry_msgs::Point pt3;
          pt3.x = mesh->vertices[v];
          pt3.y = mesh->vertices[v+1];
          pt3.z = mesh->vertices[v+2];
          mk.points.push_back(pt3);

          mk.points.push_back(pt2);
          mk.points.push_back(pt3);
        }
      }
	    
      break;
	    
    default:
      ROS_ERROR("Unknown object type: %d", (int)obj->type);
    }
  }
    
  void sendPoint(int id, const std::string &ns, double x, double y, double z, double radius, const std::string& frame_id, const ros::Time &stamp)
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = frame_id;
    mk.header.stamp = stamp;

    mk.id = id;
    mk.type = visualization_msgs::Marker::SPHERE;
    mk.action = visualization_msgs::Marker::ADD;

    mk.pose.position.x = x;
    mk.pose.position.y = y;
    mk.pose.position.z = z;
    mk.pose.orientation.w = 1.0;

    mk.scale.x = mk.scale.y = mk.scale.z = radius * 2.0;

    mk.color.a = 1.0;
    mk.color.r = 0.9;
    mk.color.g = 0.1;
    mk.color.b = 0.1;
    mk.lifetime = ros::Duration(10.0);
	
    visualizationMarkerPublisher_.publish(mk);
  }

  ros::NodeHandle                              nh_;
  tf::TransformListener                        tf_;
  ros::Publisher                               visualizationMarkerPublisher_;
  ros::ServiceClient get_objects_service_client_;
  bool                                         skip_collision_map_;
  std::map<std::string, unsigned int> cur_collision_objects_;
  std::map<std::string, unsigned int> cur_attached_objects_;
  ros::Publisher collision_map_publisher_;
  std::string prefix_;
  std_msgs::ColorRGBA object_color_;
  std_msgs::ColorRGBA attached_color_;
  unsigned int failure_count_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "display_planner_collision_model");

  DisplayPlannerCollisionModel disp;
  disp.run();
    
  return 0;
}
