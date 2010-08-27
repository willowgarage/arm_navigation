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

/** \author Sachin Chitta, Ioan Sucan */

#ifndef IK_CONSTRAINT_EVALUATORS_UTILS_
#define IK_CONSTRAINT_EVALUATORS_UTILS_

#include <motion_planning_msgs/Constraints.h>
#include <geometric_shapes_msgs/Shape.h>
#include <geometric_shapes/bodies.h>
#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/Pose.h>

#include <boost/scoped_ptr.hpp>

#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <angles/angles.h>

namespace constraint_evaluators
{    
  bool createConstraintRegionFromMsg(const geometric_shapes_msgs::Shape &constraint_region_shape, 
                                     const geometry_msgs::Pose &constraint_region_pose, 
                                     boost::scoped_ptr<bodies::Body> &body)
  {
    if(constraint_region_shape.type == geometric_shapes_msgs::Shape::SPHERE)
    {
      if(constraint_region_shape.dimensions.empty())
        return false;
      shapes::Sphere shape(constraint_region_shape.dimensions[0]);
      body.reset(new bodies::Sphere(&shape));
    }
    else if(constraint_region_shape.type == geometric_shapes_msgs::Shape::BOX)
    {
      if((int) constraint_region_shape.dimensions.size() < 3)
        return false;
      shapes::Box shape(constraint_region_shape.dimensions[0],constraint_region_shape.dimensions[1],constraint_region_shape.dimensions[2]);
      body.reset(new bodies::Box(&shape));
    }
    else if(constraint_region_shape.type == geometric_shapes_msgs::Shape::CYLINDER)
    {
      if((int) constraint_region_shape.dimensions.size() < 2)
        return false;
      shapes::Cylinder shape(constraint_region_shape.dimensions[0],constraint_region_shape.dimensions[1]);
      body.reset(new bodies::Cylinder(&shape));
    }
    else if(constraint_region_shape.type == geometric_shapes_msgs::Shape::MESH)
    {
      std::vector<btVector3> vertices;
      std::vector<unsigned int> triangles; 
      for(unsigned int i=0; i < constraint_region_shape.triangles.size(); i++)
      {
        triangles.push_back((unsigned int) constraint_region_shape.triangles[i]);
      }
      for(unsigned int i=0; i < constraint_region_shape.triangles.size(); i++)
      {
        btVector3 tmp;
        tf::pointMsgToTF(constraint_region_shape.vertices[i],tmp);
        vertices.push_back(tmp);
      }
      shapes::Mesh *shape = shapes::createMeshFromVertices(vertices,triangles);
      body.reset(new bodies::ConvexMesh(shape));    
    }
    else
    {
      ROS_ERROR("Could not recognize constraint type");
      return false;
    }
    btTransform pose_tf;
    tf::poseMsgToTF(constraint_region_pose,pose_tf);
    body->setPose(pose_tf);
    return true;
  }
}

#endif
