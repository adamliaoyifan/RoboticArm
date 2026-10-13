# Joint space: deliberately no Cartesian frame/reference or length unit.
uint32 contract_version
string request_id
string scene_revision
bool stamp_valid
uint64 stamp_ns
string time_basis
string angle_unit
# header.frame_id and header.stamp must be empty/zero; identity is above.
trajectory_msgs/JointTrajectory trajectory
