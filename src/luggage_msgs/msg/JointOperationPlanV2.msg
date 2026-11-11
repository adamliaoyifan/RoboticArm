uint32 contract_version
string request_id
string scene_revision
string snapshot_id
string geometry_hash
string model_sha256
uint64 map_revision
string observation_id
bool observation_stamp_valid
uint64 observation_stamp_ns
bool robot_state_stamp_valid
uint64 robot_state_stamp_ns
string box_id
uint64 generation
string contact_model_hash
string candidate_id
# Issued by whole-chain collision/kinematic certification, not by serialization.
string certificate_id
# These four directed poses are all world-frame, metres, xyzw, full SO(3).
# Box axes retain the measured directed axes; no yaw modulo-pi reduction.
# Pick poses belong to observation_stamp_ns; place poses to the bound snapshot.
# Tools reference suction_contact_frame, box poses reference nominal box centre.
geometry_msgs/Pose world_from_pick_box
geometry_msgs/Pose world_from_pick_tool
geometry_msgs/Pose world_from_place_box
geometry_msgs/Pose world_from_place_tool
luggage_msgs/AttachmentGeometryV2 predicted_attachment
float64[6] start_joints
luggage_msgs/JointOperationLegV2[] legs
luggage_msgs/JointOperationBarrierV2[] barriers
string plan_digest
