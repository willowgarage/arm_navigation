# This message contains a set of parameters useful in
# setting up the workspace for planning
geometric_shapes_msgs/Shape  workspace_region_shape
geometry_msgs/PoseStamped    workspace_region_pose

# A specification for regions where contact is 
# allowed upto a certain depth
# Any collision within this set of regions with a link 
# specified in the message will be allowed if
# it is less than the penetration depth specified in the message
AllowedContactSpecification[] allowed_contacts

# A set of ordered collision operations, 
# these are applied to all links, objects, 
# namespaces in the collision space
OrderedCollisionOperations ordered_collision_operations