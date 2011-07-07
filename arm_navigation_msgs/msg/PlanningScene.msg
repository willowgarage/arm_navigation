#full robot state
motion_planning_msgs/RobotState robot_state

#additional frames for duplicating tf
geometry_msgs/TransformStamped[] fixed_frame_transforms

#full allowed collision matrix
AllowedCollisionMatrix allowed_collision_matrix

#allowed contacts
motion_planning_msgs/AllowedContactSpecification[] allowed_contacts

#all link paddings
motion_planning_msgs/LinkPadding[] link_padding

#collision objects
mapping_msgs/CollisionObject[] collision_objects
mapping_msgs/AttachedCollisionObject[] attached_collision_objects

#the collision map
mapping_msgs/CollisionMap collision_map
