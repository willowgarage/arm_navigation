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

#include "planning_environment/models/collision_models.h"
#include <collision_space/environmentODE.h>
#include <collision_space/environmentBullet.h>
#include <sstream>
#include <vector>

void planning_environment::CollisionModels::setupModel(boost::shared_ptr<collision_space::EnvironmentModel> &model, const std::vector<std::string>& links)
{
  XmlRpc::XmlRpcValue coll_ops;

  //first we do default collision operations
  if(!nh_.hasParam(description_ + "_collision/default_collision_operations")) {
    ROS_WARN("No default collision operations specified");
  } else {
  
    nh_.getParam(description_ + "_collision/default_collision_operations", coll_ops);
    
    if(coll_ops.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_WARN("default_collision_operations is not an array");
      return;
    }
    
    if(coll_ops.size() == 0) {
      ROS_WARN("No collision operations in default collision operations");
      return;
    }
    
    for(int i = 0; i < coll_ops.size(); i++) {
      if(!coll_ops[i].hasMember("object1") || !coll_ops[i].hasMember("object2") || !coll_ops[i].hasMember("operation")) {
        ROS_WARN("All collision operations must have two objects and an operation");
        continue;
      }
      motion_planning_msgs::CollisionOperation collision_operation;
      collision_operation.object1 = std::string(coll_ops[i]["object1"]);
      collision_operation.object2 = std::string(coll_ops[i]["object2"]);
      std::string operation = std::string(coll_ops[i]["operation"]);
      if(operation == "enable") {
        collision_operation.operation =  motion_planning_msgs::CollisionOperation::ENABLE;
      } else if(operation == "disable") {
        collision_operation.operation =  motion_planning_msgs::CollisionOperation::DISABLE;
      } else {
        ROS_WARN_STREAM("Unrecognized collision operation " << operation << ". Must be enable or disable");
        continue;
      }
      default_collision_operations_.push_back(collision_operation);
    }
  }

  //ROS_INFO_STREAM("Padd is " << padd_);

  //now we do paddings in the private namespace
  ros::NodeHandle priv("~");

  priv.param("default_robot_padding", padd_, 0.01);
  priv.param("robot_scale", scale_, 1.0);

  //ROS_INFO_STREAM("Padd is " << padd_);

  for(std::vector<std::string>::const_iterator it = links.begin();
      it != links.end();
      it++) {
    link_padding_map_[*it] = padd_;
  }

  if(priv.hasParam("link_padding")) {
    XmlRpc::XmlRpcValue link_padding_xml;
    priv.getParam("link_padding", link_padding_xml);
    if(link_padding_xml.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_WARN("link_padding is not an array");
    } else if(link_padding_xml.size() == 0) {
      ROS_WARN("No links specified in link_padding");
    } else {
      for(int i = 0; i < link_padding_xml.size(); i++) {
        if(!link_padding_xml[i].hasMember("link") || !link_padding_xml[i].hasMember("padding")) {
          ROS_WARN("Each entry in link padding must specify a link and a padding");
          continue;
        }
        std::string link = std::string(link_padding_xml[i]["link"]);
        double padding = link_padding_xml[i]["padding"];
        std::vector<std::string> svec1;
        if(planning_group_links_.find(link) != planning_group_links_.end()) {
          svec1 = planning_group_links_[link];
        } else {
          svec1.push_back(link);
        }
        for(std::vector<std::string>::iterator it = svec1.begin();
            it != svec1.end();
            it++) {
          link_padding_map_[*it] = padding;
        }
      }
    }
  }
  
  model->lock();
  std::vector<std::string> links_with_collision = links;
  std::vector<std::string>::iterator lit = links_with_collision.begin();
  while(lit != links_with_collision.end()) {
    if(kmodel_->getLinkModel(*lit)->getLinkShape() == NULL) {
      lit = links_with_collision.erase(lit);
    } else {
      lit++;
    }
  }
  model->setRobotModel(kmodel_, links_with_collision, link_padding_map_, padd_,scale_);

  for(std::vector<motion_planning_msgs::CollisionOperation>::iterator it = default_collision_operations_.begin();
      it != default_collision_operations_.end();
      it++) {
    std::vector<std::string> svec1;
    std::vector<std::string> svec2;
    if(planning_group_links_.find((*it).object1) != planning_group_links_.end()) {
      svec1 = planning_group_links_[(*it).object1];
    } else {
      svec1.push_back((*it).object1);
    }
    if(planning_group_links_.find((*it).object2) != planning_group_links_.end()) {
      svec2 = planning_group_links_[(*it).object2];
    } else {
      svec2.push_back((*it).object2);
    }
    if((*it).operation == motion_planning_msgs::CollisionOperation::ENABLE) {
      model->addSelfCollisionGroup(svec1,svec2);
    } else {
      model->removeSelfCollisionGroup(svec1,svec2);
    }
  }
  for (unsigned int i = 0 ; i < boundingPlanes_.size() / 4 ; ++i)
  {
    shapes::Plane *plane = new shapes::Plane(boundingPlanes_[i * 4], boundingPlanes_[i * 4 + 1], boundingPlanes_[i * 4 + 2], boundingPlanes_[i * 4 + 3]);
    model->addObject("bounds", plane);
    ROS_INFO("Added static plane %fx + %fy + %fz + %f = 0 for model %p", boundingPlanes_[i * 4], boundingPlanes_[i * 4 + 1], boundingPlanes_[i * 4 + 2], boundingPlanes_[i * 4 + 3], model.get());
  }
  
  model->unlock();    
}

void planning_environment::CollisionModels::loadParams(void)
{

}

void planning_environment::CollisionModels::loadCollision(const std::vector<std::string> &links)
{
    // a list of static planes bounding the environment
    boundingPlanes_.clear();
    
    std::string planes;
    nh_.param<std::string>("bounding_planes", planes, std::string());
    
    std::stringstream ss(planes);
    if (!planes.empty())
	while (ss.good() && !ss.eof())
	{
	    double value;
	    ss >> value;
	    boundingPlanes_.push_back(value);
	}
    if (boundingPlanes_.size() % 4 != 0)
    {
	ROS_WARN("~bounding_planes must be a list of 4-tuples (a b c d) that define planes ax+by+cz+d=0");
	boundingPlanes_.resize(boundingPlanes_.size() - (boundingPlanes_.size() % 4));
    }
    
    if (loadedModels())
    {
	ode_collision_model_ = boost::shared_ptr<collision_space::EnvironmentModel>(new collision_space::EnvironmentModelODE());
	setupModel(ode_collision_model_,links);
	
	//	bullet_collision_model_ = boost::shared_ptr<collision_space::EnvironmentModel>(new collision_space::EnvironmentModelBullet());
	//	setupModel(bullet_collision_model_, links);
    } else {
      ROS_WARN("Models not loaded");
    }
}
