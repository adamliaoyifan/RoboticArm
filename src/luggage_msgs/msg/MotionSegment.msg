# One leg of a pick/place motion sequence (input to motion_planner).
string name
string type
geometry_msgs/Pose target_pose
geometry_msgs/Pose[] waypoints
# Enforce tool-Z-axis-down OrientationConstraint during planning (heavy payload transit).
bool keep_tool_down
# Enforce camera optical +Z toward world -Z (keep camera looking straight down).
bool keep_camera_down
# Lock J4/J5/J6 near current angles via JointConstraints to prevent wrist spin.
bool lock_wrist
# Permit Cartesian planning to fall back to OMPL pose planning.
# Keep false for constrained insertion/probe segments.
bool allow_ompl_fallback
# Minimum Cartesian path fraction required before this segment may
# execute (0.0 = executor default). Suction approach/attach/retry_reverse
# carry 1.0 so a partial straight-line path is refused, not executed
# (dynamic-suction plan, gate C1).
float64 required_cartesian_fraction
