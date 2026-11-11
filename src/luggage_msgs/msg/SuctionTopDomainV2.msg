# Read-only observed support for a single measured box top. This is not an
# execution approval. No missing-depth interpolation or rectangle filling.
std_msgs/Header header
string box_id
string instance_id
uint64 generation
uint32 contact_model_version
string contact_model_hash
bool valid
string reason
# SHA256 of the canonical semantic record, including support and observation.
string domain_sha256
# Nominal measured box geometry binds this domain to DetectedLuggage.
geometry_msgs/Pose box_pose_world
float64[3] box_wdh
# Orthonormal right-handed basis and plane equation n_world . p_world + d = 0.
float64[3] u_axis_world
float64[3] v_axis_world
float64[3] normal_world
float64 plane_offset_world_m
# Grid coordinates in that measured top-plane basis, lengths in metres.
float64[2] origin_uv_top_m
float64 cell_size_m
uint32 size_u
uint32 size_v
# Row major (u,v). 0 = unknown/outside, 1 = whole cell directly observed
# inside the winning connected same-box depth/plane patch. Other values refuse.
uint8[<=1000000] support
