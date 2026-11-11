string query_id
float64[2] center_uv_top_m
float64 yaw_rad
bool accepted
string reason
# Active only when accepted. Row-major proper 4x4 rigid transform in metres.
# Source: suction_contact_frame; target: world; tool +Z points into the face.
float64[16] world_from_contact_frame
string quality_metrics_json
uint32 refinement_count
string evidence_sha256
