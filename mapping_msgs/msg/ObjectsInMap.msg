# a header, used for interpreting the poses
Header header

# the id of the object
string id

#This contains what is to be done with the object
ObjectAction action

#the shape associated with the object
geometric_shapes_msgs/Shape[] objects

#the pose associated with the object - will be transformed using the header
geometry_msgs/Pose[] poses
