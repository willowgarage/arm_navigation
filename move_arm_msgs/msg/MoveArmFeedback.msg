# The internal state that the move arm action currently is in
string state

# Time to completion - this is a combination of requested planning time and trajectory completion time
duration time_to_completion

# error code reflecting what went wrong
motion_planning_msgs/ArmNavigationErrorCodes error_code