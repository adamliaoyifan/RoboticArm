# An undecidable geometric hypothesis. NOT a contact query, grasp or motion permit.
string query_id
string domain_sha256
string contact_model_hash
float64[2] center_uv_top_m
float64 yaw_rad
float64[8] footprint_uv_top_m
uint32 unproven_cell_count
uint32 checked_centres
string placement_slot_id
# Exact already-admitted target that motivated this probe, never a new target.
luggage_msgs/PoseContractV2 placement
