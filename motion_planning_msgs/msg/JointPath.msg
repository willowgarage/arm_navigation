# This message defines a joint path 

# The standard ROS header
Header header

# Vector of joint names for which positions are specified. 
# The size of this vector must match the size of each waypoint below
string[] joint_names

# A vector of waypoints. 
# Each waypoint is a vector of joint positions. 
# Each waypoint must have the same size as the vector of joint names
JointPathPoint[] points