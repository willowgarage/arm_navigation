# The trajectory to filter
trajectory_msgs/JointTrajectory trajectory
# A vector of JointLimit messages.
# Each message contains the limits for a specific joint
motion_planning_msgs/JointLimits[] limits
# Total allowed time for filtering
duration allowed_time
