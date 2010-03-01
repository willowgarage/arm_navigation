# service name to call for planning
string planner_service_name

# Planner id to be used for planning 
string planner_id

# group to plan for
string group_name

# num planning attempts
int32 num_planning_attempts

# allowed planning time
float64 allowed_planning_time

# The goal state for the model to plan for. The state is not necessarily explicit.
# The goal is considered achieved if all the constraints are satisfied.
# An explicit state can be specified by setting joint constraints to a specific value.
motion_planning_msgs/Constraints goal_constraints

# The set of constraints that need to be satisfied by the entire solution path.
motion_planning_msgs/Constraints path_constraints

# A set of named regions, ordered collision operations are applied after 
# these regions are specified internally inside the collision space
motion_planning_msgs/AllowedContactSpecification[] allowed_contacts

# Collision specification
motion_planning_msgs/OrderedCollisionOperations ordered_collision_operations

# An expected path duration (in seconds) along with an expected discretization of the path allows the planner to determine the discretization of the trajectory that it returns
duration expected_path_duration
duration expected_path_dt

# Encodes options that you can pass into move_arm
# e.g. ACCEPT_PARTIAL_PLANS will allow move_arm to accept plans that do not go all the way to the goal
# e.g. ACCEPT_INVALID_GOALS will allow move_arm to accept goals that are invalid and try to plan to them
# This is useful if you are using a planner like CHOMP along with a noisy rapidly changing collision map
# and you would like to plan to a goal near an object.
int32 options
int32 ACCEPT_PARTIAL_PLANS=1
int32 ACCEPT_INVALID_GOALS=2