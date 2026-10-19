# Geometric joint paths only, untimed and NEVER executable trajectories.
# First path approaches contact carrying=false for pick, true for place.
# Second path leaves contact carrying=true for pick, false for place.
uint32 seed_index
trajectory_msgs/JointTrajectory incoming
trajectory_msgs/JointTrajectory outgoing
