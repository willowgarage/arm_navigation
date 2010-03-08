# Service name to call for getting a motion plan
# Move arm will call a service on this service name 
# using the MotionPlanRequest specified here
string planner_service_name

# A motion planning request
motion_planning_msgs/MotionPlanRequest motion_plan_request

# OPTIONAL flag
# Setting this flag to true will allow move_arm to accept plans that do not go all the way to the goal
bool accept_partial_plans

# OPTIONAL flag
# Setting this flag to true will allow move_arm to accept invalid goals
# This is useful if you are using a planner like CHOMP along with a noisy rapidly changing collision map
# and you would like to plan to a goal near an object.
bool accept_invalid_goals

# OPTIONAL flag
# Setting this flag to true will disable the call to IK for a pose goal
bool disable_ik

# OPTIONAL flag
# Setting this flag to true will disable collision monitoring during execution of a trajectory
bool disable_collision_monitoring