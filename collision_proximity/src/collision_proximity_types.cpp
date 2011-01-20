/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

/** \author E. Gil Jones */

#include <collision_proximity/collision_proximity_types.h>

std::vector<collision_proximity::CollisionSphere> collision_proximity::determineCollisionSpheres(const bodies::Body* body)
{
  std::vector<collision_proximity::CollisionSphere> css;
  
  bodies::BoundingCylinder cyl;
  body->computeBoundingCylinder(cyl);
  unsigned int num_points = ceil(cyl.length/(cyl.radius/2.0));
  double spacing = cyl.length/((num_points*1.0)-1.0);
  btVector3 vec(0.0,0.0,0.0);
  for(unsigned int i = 0; i < num_points; i++) {
    vec.setZ((-cyl.length/2.0)+i*spacing);
    btVector3 p = cyl.pose*vec;
    collision_proximity::CollisionSphere cs(vec,cyl.radius);
    css.push_back(cs);
  }
  return css; 
}

std::vector<btVector3> collision_proximity::determineCollisionPoints(const bodies::Body* body, double resolution)
{
  std::vector<btVector3> ret_vec;
  bodies::BoundingSphere sphere;
  body->computeBoundingSphere(sphere);
  //ROS_INFO_STREAM("Radius is " << sphere.radius);
  //ROS_INFO_STREAM("Center is " << sphere.center.z() << " " << sphere.center.y() << " " << sphere.center.z());
  for(double xval = sphere.center.x()-sphere.radius-resolution; xval < sphere.center.x()+sphere.radius+resolution; xval += resolution) {
    for(double yval = sphere.center.y()-sphere.radius-resolution; yval < sphere.center.y()+sphere.radius+resolution; yval += resolution) {
      for(double zval = sphere.center.z()-sphere.radius-resolution; zval < sphere.center.z()+sphere.radius+resolution; zval += resolution) {
        btVector3 rel_vec(xval, yval, zval);
        if(body->containsPoint(body->getPose()*rel_vec)) {
          ret_vec.push_back(rel_vec);
        }
      }
    }
  }
  return ret_vec;
}

bool collision_proximity::getCollisionSphereGradients(const distance_field::PropagationDistanceField* distance_field, 
                                                      const std::vector<CollisionSphere>& sphere_list, 
                                                      double& closest_distance, 
                                                      std::vector<double>& distances, 
                                                      std::vector<btVector3>& gradients, 
                                                      double tolerance, 
                                                      bool subtract_radii, 
                                                      bool stop_at_first_collision) {
  gradients.clear();
  distances.clear();
  closest_distance = DBL_MAX;
  bool in_collision = false;
  for(unsigned int i = 0; i < sphere_list.size(); i++) {
    btVector3 p = sphere_list[i].center_;
    double gx, gy, gz;
    double dist = distance_field->getDistanceGradient(p.x(), p.y(), p.z(), gx, gy, gz);
    if(subtract_radii) {
      dist -= sphere_list[i].radius_;
      if(dist <= tolerance) {
        if(stop_at_first_collision) {
          return true;
        } 
        in_collision = true;
      } 
    }
    if(dist < closest_distance) {
      closest_distance = dist;
    }
    distances.push_back(dist);
    gradients.push_back(btVector3(gx,gy,gz));
  }
  return in_collision;

}

bool collision_proximity::getCollisionSphereCollision(const distance_field::PropagationDistanceField* distance_field, 
                                                      const std::vector<CollisionSphere>& sphere_list,
                                                      double tolerance)
{
  for(unsigned int i = 0; i < sphere_list.size(); i++) {
    btVector3 p = sphere_list[i].center_;
    double gx, gy, gz;
    double dist = distance_field->getDistanceGradient(p.x(), p.y(), p.z(), gx, gy, gz);
    if(dist - sphere_list[i].radius_ < tolerance) {
      return true;
    }
  }
  return false;

} 

///
/// BodyDecomposition
///

collision_proximity::BodyDecomposition::BodyDecomposition(const std::string& object_name, const shapes::Shape* shape, double resolution) :
  object_name_(object_name)
{
  body_ = bodies::createBodyFromShape(shape); //unpadded
  btTransform ident;
  ident.setIdentity();
  body_->setPose(ident);
  body_->setPadding(.01);
  collision_spheres_ = determineCollisionSpheres(body_);
  relative_collision_points_ = determineCollisionPoints(body_, resolution);
  posed_collision_points_ = relative_collision_points_;
  ROS_DEBUG_STREAM("Object " << object_name << " has " << relative_collision_points_.size() << " collision points");
}

collision_proximity::BodyDecomposition::~BodyDecomposition()
{
  delete body_;
}

void collision_proximity::BodyDecomposition::updateSpheresPose(const btTransform& trans) 
{
  body_->setPose(trans);
  bodies::BoundingCylinder cyl;
  body_->computeBoundingCylinder(cyl);
  for(unsigned int i = 0; i < collision_spheres_.size(); i++) {
    collision_spheres_[i].center_ = cyl.pose*collision_spheres_[i].relative_vec_;
  }
}

void collision_proximity::BodyDecomposition::updatePointsPose(const btTransform& trans) {
  body_->setPose(trans);
  posed_collision_points_.clear();
  posed_collision_points_.resize(relative_collision_points_.size());
  for(unsigned int i = 0; i < relative_collision_points_.size(); i++) {
    posed_collision_points_[i] = body_->getPose()*relative_collision_points_[i];
  }
}


void collision_proximity::BodyDecomposition::updatePose(const btTransform& trans)
{
  updateSpheresPose(trans);
  updatePointsPose(trans);
}
