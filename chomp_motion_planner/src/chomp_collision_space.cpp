/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

/** \author Mrinal Kalakrishnan */

#include <chomp_motion_planner/chomp_collision_space.h>
#include <sstream>

namespace chomp
{

ChompCollisionSpace::ChompCollisionSpace():
  node_handle_("~"),distance_field_(NULL),collision_map_subscriber_(root_handle_,"collision_map_occ",1)
{
}

ChompCollisionSpace::~ChompCollisionSpace()
{
  delete distance_field_;
  delete collision_map_filter_;
  delete collision_object_filter_;
  delete collision_object_subscriber_;
  delete monitor_;
}

bool ChompCollisionSpace::init(planning_environment::CollisionModels* collision_models, double max_radius_clearance)
{
  double size_x, size_y, size_z;
  double origin_x, origin_y, origin_z;
  double resolution;

  node_handle_.param("reference_frame", reference_frame_, std::string("base_link"));
  node_handle_.param("collision_space/size_x", size_x, 2.0);
  node_handle_.param("collision_space/size_y", size_y, 3.0);
  node_handle_.param("collision_space/size_z", size_z, 4.0);
  node_handle_.param("collision_space/origin_x", origin_x, 0.1);
  node_handle_.param("collision_space/origin_y", origin_y, -1.5);
  node_handle_.param("collision_space/origin_z", origin_z, -2.0);
  node_handle_.param("collision_space/resolution", resolution, 0.02);
  node_handle_.param("collision_space/field_bias_x", field_bias_x_, 0.0);
  node_handle_.param("collision_space/field_bias_y", field_bias_y_, 0.0);
  node_handle_.param("collision_space/field_bias_z", field_bias_z_, 0.0);
  resolution_ = resolution;
  max_expansion_ = max_radius_clearance;

  collision_models_ = collision_models;

  //initCollisionCuboids();

  distance_field_ = new distance_field::PropagationDistanceField(size_x, size_y, size_z, resolution, origin_x, origin_y, origin_z, max_expansion_);

  monitor_ = new planning_environment::KinematicModelStateMonitor(collision_models, &tf_, reference_frame_);
  //now setting up robot bodies for potential inclusion in the distance field
  loadRobotBodies();

  updateBodiesPoses();

  addBodiesInGroupToDistanceField("right_arm");

  collision_map_filter_ = new tf::MessageFilter<mapping_msgs::CollisionMap>(collision_map_subscriber_,tf_,reference_frame_,1);
  collision_map_filter_->registerCallback(boost::bind(&ChompCollisionSpace::collisionMapCallback, this, _1));

  collision_object_subscriber_ = new message_filters::Subscriber<mapping_msgs::CollisionObject>(root_handle_, "collision_object", 1024);
  collision_object_filter_ = new tf::MessageFilter<mapping_msgs::CollisionObject>(*collision_object_subscriber_, tf_, reference_frame_, 1024);
  collision_object_filter_->registerCallback(boost::bind(&ChompCollisionSpace::collisionObjectCallback, this, _1));


  ROS_INFO("Initialized chomp collision space in %s reference frame with %f expansion radius.", reference_frame_.c_str(), max_expansion_);
  return true;
}

void ChompCollisionSpace::collisionMapCallback(const mapping_msgs::CollisionMapConstPtr& collision_map)
{
  return;

  // @TODO transform the collision map to the required frame!!
  if (mutex_.try_lock())
  {
    ros::WallTime start = ros::WallTime::now();
    distance_field_->reset();
    ROS_INFO("Reset prop distance_field in %f sec", (ros::WallTime::now() - start).toSec());
    start = ros::WallTime::now();
    distance_field_->addPointsToField(cuboid_points_);
    distance_field_->addCollisionMapToField(*collision_map);
    mutex_.unlock();
    ROS_INFO("Updated prop distance_field in %f sec", (ros::WallTime::now() - start).toSec());

    distance_field_->visualize(0.895*max_expansion_, 0.9*max_expansion_, collision_map->header.frame_id, collision_map->header.stamp);

  }
  else
  {
    ROS_INFO("Skipped collision map update because planning is in progress.");
  }
}

void ChompCollisionSpace::collisionObjectCallback(const mapping_msgs::CollisionObjectConstPtr &collisionObject) {
  if (collisionObject->operation.operation == mapping_msgs::CollisionObjectOperation::ADD)
  {
    if (mutex_.try_lock())
    {
      std::vector<btVector3> points;
      for (size_t i=0; i<collisionObject->shapes.size(); ++i)
      {
        const geometric_shapes_msgs::Shape& object = collisionObject->shapes[i];
        const geometry_msgs::Pose& pose = collisionObject->poses[i];
        if (object.type == geometric_shapes_msgs::Shape::CYLINDER)
        {
          ROS_INFO("Got cylinder");

          if (object.dimensions.size() != 2) {
            ROS_INFO_STREAM("Cylinder must have exactly 2 dimensions, not " 
                            << object.dimensions.size()); 
            continue;
          }
          KDL::Rotation rotation = KDL::Rotation::Quaternion(pose.orientation.x,
                                                             pose.orientation.y,
                                                             pose.orientation.z,
                                                             pose.orientation.w);
          KDL::Vector position(pose.position.x, pose.position.y, pose.position.z);
          KDL::Frame f(rotation, position);
          // generate points:
          double radius = object.dimensions[0];
          double length = object.dimensions[1];
          KDL::Vector p(0,0,0);
          KDL::Vector p2;
          double xdiv = object.dimensions[0]/resolution_;
          double ydiv = object.dimensions[0]/resolution_;
          double zdiv = object.dimensions[1]/resolution_;

          //ROS_INFO_STREAM("Divs " << xdiv << " " << ydiv << " " << zdiv);

          double xlow = pose.position.x - object.dimensions[0]/2.0;
          double ylow = pose.position.y - object.dimensions[0]/2.0;
          double zlow = pose.position.z - object.dimensions[1]/2.0;

          for(double x = xlow; x <= xlow+object.dimensions[0]+resolution_; x += resolution_) {
            for(double y = ylow; y <= ylow+object.dimensions[0]+resolution_; y += resolution_) {
              for(double z = zlow; z <= zlow+object.dimensions[1]+resolution_; z += resolution_) {
                double xdist = fabs(pose.position.x-x);
                double ydist = fabs(pose.position.y-y);
                if(sqrt(xdist*xdist+ydist*ydist) < radius) {
                  points.push_back(btVector3(x,y,z));
                }
              }
            }
          }
          //double spacing = resolution_;//radius/2.0;
          //int num_points = ceil(length/spacing)+1;
          //spacing = length/(num_points-1.0);
          //for (int i=0; i<num_points; ++i)
          //{
          //  p(2) = -length/2.0 + i*spacing;
          //  p2 = f*p;
          //  points.push_back(btVector3(p2(0), p2(1), p2(2)));
          //}
        } else if (object.type == geometric_shapes_msgs::Shape::BOX) {
          if(object.dimensions.size() != 3) {
            ROS_INFO_STREAM("Box must have exactly 3 dimensions, not " 
                            << object.dimensions.size());
            continue;
          }
          
          double xdiv = object.dimensions[0]/resolution_;
          double ydiv = object.dimensions[1]/resolution_;
          double zdiv = object.dimensions[2]/resolution_;

          ROS_INFO_STREAM("Divs " << xdiv << " " << ydiv << " " << zdiv);

          double xlow = pose.position.x - object.dimensions[0]/2.0;
          double ylow = pose.position.y - object.dimensions[1]/2.0;
          double zlow = pose.position.z - object.dimensions[2]/2.0;

          for(double x = xlow; x <= xlow+object.dimensions[0]+resolution_; x += resolution_) {
            for(double y = ylow; y <= ylow+object.dimensions[1]+resolution_; y += resolution_) {
              for(double z = zlow; z <= zlow+object.dimensions[2]+resolution_; z += resolution_) {
                points.push_back(btVector3(x,y,z));
              }
            }
          }
        } else if (object.type == geometric_shapes_msgs::Shape::MESH) {
          // //adding the vertices themselves
          // for(std::vector<geometry_msgs::Point>::const_iterator it = object.vertices.begin();
          //     it != object.vertices.end();
          //     it++) {
          //   points.push_back(btVector3(it->x, it->y, it->z));
          // }
          // //now interpolating for big triangles
          // for (size_t i=0; i<object.triangles.size(); i+=3)
          // {
          //   btVector3 v0( object.vertices.at( object.triangles.at(i+0) ).x,
          //                 object.vertices.at( object.triangles.at(i+0) ).y,
          //                 object.vertices.at( object.triangles.at(i+0) ).z);
          //   btVector3 v1( object.vertices.at( object.triangles.at(i+1) ).x,
          //                 object.vertices.at( object.triangles.at(i+1) ).y,
          //                 object.vertices.at( object.triangles.at(i+1) ).z);
          //   btVector3 v2( object.vertices.at( object.triangles.at(i+2) ).x,
          //                 object.vertices.at( object.triangles.at(i+2) ).y,
          //                 object.vertices.at( object.triangles.at(i+2) ).z);
          //   std::vector<btVector3> triangleVectors = interpolateTriangle(v0, v1, v2, resolution_);
          //   points.insert(points.end(), triangleVectors.begin(), triangleVectors.end());
          // }
        } else {
          ROS_WARN("Attaching objects of non-cylinder types is not supported yet!");
        }
      }
      ROS_INFO_STREAM("Trying to add " << points.size() << " to distance field");  
      distance_field_->reset();
      distance_field_->addPointsToField(cuboid_points_);
      distance_field_->addPointsToField(points);
      mutex_.unlock();
      
      distance_field_->visualize(0.895*max_expansion_, 0.9*max_expansion_, collisionObject->header.frame_id, collisionObject->header.stamp);
    }
  }
}

// std::vector<btVector3> ChompCollisionSpace::interpolateTriangle(btVector3 v0, 
//                                                                   btVector3 v1, 
//                                                                   btVector3 v2, double min_res)
// {
//   std::vector<btVector3> vectors;

//   //find out the interpolation resolution for the first coordinate
//   //based on the size of the 0-1 and 0-2 edges
//   double d01 = dist(v0, v1);
//   double d02 = dist(v0, v2);
//   double res_0 = min_res / std::max(d01, d02);

//   //perform the first interpolation
//   //we do not want the vertices themselves, so we don't start at 0 
//   double t0=res_0;
//   bool done = false;
//   while (!done)
//   {
//     if (t0 >= 1.0)
//     {
//       t0 = 1.0;
//       done = true;
//     }
//     //compute the resolution for the second interpolation
//     btVector3 p1 = t0*v0 + (1-t0) * v1;
//     btVector3 p2 = t0*v0 + (1-t0) * v2;
//     double d12 = dist(p1, p2);
//     double res_12 = min_res / d12;

//     //perform the second interpolation
//     double t12 = 0;
//     bool done12 = false;
//     while (!done12)
//     {
//       if (t12 >= 1.0)
//       {
// 	t12 = 1.0;
// 	done12 = true;
//       }
//       //actual point insertion
//       //do not insert the vertices themselves
//       if (t0!=1.0 || (t12!=0.0 && t12!=1.0))
//       {
// 	vectors.push_back( t12*p1 + (1.0 - t12)*p2 );
//       }
//       t12 += res_12;
//     }
    
//     t0 += res_0;
//   }
//   return vectors;
// }

static std::string intToString(int i)
{
  std::ostringstream oss;
  oss << i;
  return oss.str();
}

void ChompCollisionSpace::initCollisionCuboids()
{
  int index=1;
  while (node_handle_.hasParam(std::string("collision_space/cuboids/cuboid")+intToString(index)+"/size_x"))
  {
    addCollisionCuboid(std::string("collision_space/cuboids/cuboid")+intToString(index));
    index++;
  }

}

void ChompCollisionSpace::addCollisionCuboid(const std::string param_name)
{
  double size_x, size_y, size_z;
  double origin_x, origin_y, origin_z;
  if (!node_handle_.getParam(param_name+"/size_x", size_x))
    return;
  if (!node_handle_.getParam(param_name+"/size_y", size_y))
    return;
  if (!node_handle_.getParam(param_name+"/size_z", size_z))
    return;
  if (!node_handle_.getParam(param_name+"/origin_x", origin_x))
    return;
  if (!node_handle_.getParam(param_name+"/origin_y", origin_y))
    return;
  if (!node_handle_.getParam(param_name+"/origin_z", origin_z))
    return;

  if (size_x<0 || size_y<0 || size_z<0)
    return;

  // add points:
  int num_points=0;
  for (double x=origin_x; x<=origin_x+size_x; x+=resolution_)
    for (double y=origin_y; y<=origin_y+size_y; y+=resolution_)
      for (double z=origin_z; z<=origin_z+size_z; z+=resolution_)
      {
        cuboid_points_.push_back(btVector3(x,y,z));
        ++num_points;
      }
  ROS_INFO("Added %d points for collision cuboid %s", num_points, param_name.c_str());
}

void ChompCollisionSpace::loadRobotBodies() {
  planning_group_link_names_.clear();

  planning_group_link_names_ = collision_models_->getPlanningGroupLinks();

  ROS_INFO_STREAM("Planning group links size " << planning_group_link_names_.size());

  for(std::map<std::string, std::vector<std::string> >::iterator it1 = planning_group_link_names_.begin();
      it1 != planning_group_link_names_.end();
      it1++)
  {
    ROS_INFO_STREAM("Chomp loading group " << it1->first);

    for(std::vector<std::string>::iterator it2 = it1->second.begin();
        it2 != it1->second.end();
        it2++) {
      const planning_models::KinematicModel::Link* link = collision_models_->getKinematicModel()->getLink(*it2);
      if(link != NULL) {
        planning_group_bodies_[it1->first].push_back(bodies::createBodyFromShape(link->shape));
      } else {
        ROS_WARN_STREAM("Error - no link for name " << *it2);
      }
      //planning_group_bodies_[it1->first].back()->setPadding(padding_);
    }
  }
}

void ChompCollisionSpace::updateBodiesPoses() {
  monitor_->getKinematicModel()->lock();
  monitor_->getKinematicModel()->computeTransforms(monitor_->getRobotState()->getParams());

  for(std::map<std::string, std::vector<bodies::Body *> >::iterator it1 = planning_group_bodies_.begin();
      it1 != planning_group_bodies_.end();
      it1++) {
    std::vector<std::string>& v = planning_group_link_names_[it1->first];
    for(unsigned int i = 0; i < it1->second.size(); i++) {
      const planning_models::KinematicModel::Link* link = monitor_->getKinematicModel()->getLink(v[i]);
      (it1->second)[i]->setPose(link->globalTrans);
    }
  }
}

void ChompCollisionSpace::addBodiesInGroupToDistanceField(const std::string& group) {
  
  std::vector<btVector3> body_points;
  
  if(group == std::string("all")) {
    for(std::map<std::string, std::vector<bodies::Body *> >::iterator it1 = planning_group_bodies_.begin();
        it1 != planning_group_bodies_.end();
        it1++) {
      for(unsigned int i = 0; i < it1->second.size(); i++) {
        std::vector<btVector3> single_body_points;
        getVoxelsInBody((*it1->second[i]), single_body_points);
        ROS_INFO_STREAM("Group " << it1->first << " link num " << i << " points " << single_body_points.size());
        body_points.insert(body_points.end(), single_body_points.begin(), single_body_points.end());
      }
    }
  } else {
    if(planning_group_bodies_.find(group) != planning_group_bodies_.end()) {
      std::vector<bodies::Body *>& bodies = planning_group_bodies_[group];
      for(unsigned int i = 0; i < bodies.size(); i++) {
        std::vector<btVector3> single_body_points;
        getVoxelsInBody(*(bodies[i]), single_body_points);
        ROS_INFO_STREAM("Group " << group << " link num " << i << " points " << single_body_points.size());
        body_points.insert(body_points.end(), single_body_points.begin(), single_body_points.end());
      }
    } else {
      ROS_WARN_STREAM("Group " << group << " not found in planning groups");
    }
  }
  
  ROS_INFO_STREAM("Adding points for robot's body that number " << body_points.size());
  distance_field_->reset();
  distance_field_->addPointsToField(body_points);
  distance_field_->visualize(0.895*max_expansion_, 0.9*max_expansion_, "base_footprint", ros::Time::now());

}

void ChompCollisionSpace::getVoxelsInBody(const bodies::Body &body, std::vector<btVector3> &voxels)
{
  bodies::BoundingSphere bounding_sphere;

  body.computeBoundingSphere(bounding_sphere);
  int x,y,z,x_min,x_max,y_min,y_max,z_min,z_max;
  double xw,yw,zw;
  btVector3 v;
	
  worldToGrid(bounding_sphere.center,bounding_sphere.center.x()-bounding_sphere.radius,bounding_sphere.center.y()-bounding_sphere.radius,	
              bounding_sphere.center.z()-bounding_sphere.radius, x_min,y_min,z_min);
  worldToGrid(bounding_sphere.center,bounding_sphere.center.x()+bounding_sphere.radius,bounding_sphere.center.y()+bounding_sphere.radius,	
              bounding_sphere.center.z()+bounding_sphere.radius, x_max,y_max,z_max);
	
  // 	ROS_INFO("xyz_min: %i %i %i xyz_max: %i %i %i",x_min,y_min,z_min,x_max,y_max,z_max);
	
  voxels.reserve(30000);
  for(x = x_min; x <= x_max; ++x)
  {
    for(y = y_min; y <= y_max; ++y)
    {
      for(z = z_min; z <= z_max; ++z)
      {
        gridToWorld(bounding_sphere.center,x,y,z,xw,yw,zw);
        if(body.containsPoint(xw,yw,zw))
        {	
          v.setX(xw);
          v.setY(yw);
          v.setZ(zw);
          //ROS_INFO_STREAM(xw << " " << yw << " " << zw);
          voxels.push_back(v);
        }
      }
    }
  }
	
  // 	ROS_INFO("number of occupied voxels in bounding sphere: %i", voxels.size());
}

}
