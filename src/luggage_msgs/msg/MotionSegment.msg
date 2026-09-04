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
