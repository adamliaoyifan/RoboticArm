# F15 typed motion leg. No geometry_hash and no map_revision.
# Joint paths carry no Cartesian frame on this envelope.
uint32 contract_version
string request_id
string scene_revision
bool stamp_valid
uint64 stamp_ns
string time_basis
string name
string segment_type
string target_link
# policy_valid=false rejects a real ROS message whose bools would otherwise
# silently default to false.
bool policy_valid
bool keep_tool_down
bool keep_camera_down
bool lock_wrist
bool allow_ompl_fallback
float64 required_cartesian_fraction
luggage_msgs/PoseContractV2 target_pose
luggage_msgs/PoseContractV2[] waypoints
luggage_msgs/JointTrajectoryContractV2 joint_trajectory
