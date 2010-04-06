# This service contains the definition for a request to the motion
# planner and the output it provides

# Parameters for the workspace that the planner should work inside
motion_planning_msgs/WorkspaceParameters workspace_parameters

# Starting state updates. If certain joints should be considered
# at positions other than the current ones, these positions should
# be set here
motion_planning_msgs/RobotState start_state

# The goal state for the model to plan for. The goal is achieved
# if all constraints are satisfied
motion_planning_msgs/Constraints goal_constraints

# No state at any point along the path in the produced motion plan will violate these constraints
motion_planning_msgs/Constraints path_constraints

# A specification for regions where contact is 
# allowed up to a certain depth
# Any collision within this set of regions with a link 
# specified in the message will be allowed if
# it is less than the penetration depth specified in the message
AllowedContactSpecification[] allowed_contacts

# A set of ordered collision operations, 
# these are applied to all links, objects, 
# namespaces in the collision space
OrderedCollisionOperations ordered_collision_operations

# Specifies a set of links and paddings to change from the default
# specified in the yaml file
motion_planning_msgs/LinkPadding[] link_padding

# The name of the motion planner to use. If no name is specified,
# a default motion planner will be used
string planner_id

# The name of the group of joints on which this planner is operating
string group_name

# The number of times this plan is to be computed. Shortest solution
# will be reported.
int32 num_planning_attempts

# The maximum amount of time the motion planner is allowed to plan for
duration allowed_planning_time

# An expected path duration (in seconds) along with an expected discretization of the path allows the planner to determine the discretization of the trajectory that it returns
duration expected_path_duration
duration expected_path_dt
