# One sealable suction-patch candidate (DYNAMIC-SUCTION plan section C).
# Acquisition identity: matches the DetectedLuggage observation exactly;
# any mismatch is SUCTION_CANDIDATE_IDENTITY_MISMATCH before planning.
std_msgs/Header header
string instance_id
uint32 generation
# World-frame contact pose; +Z is the outward local surface normal the
# tool must oppose.
geometry_msgs/Pose contact_pose
# Deterministic grid id (C%03d_%03d of the height-map cell) and rank
# within the accepted set (1 = best).
string candidate_id
uint32 rank
# Ranking score (reporting only; ordering uses the plan's tuple).
float64 score
float64 valid_coverage
float64 mask_coverage
float64 plane_coverage
# Plane-fit residual statistics over the footprint cells.
float64 rms_residual
float64 p95_residual
float64 peak_to_valley
float64 normal_deviation_p95
float64 max_adjacent_step
# Free margin between the footprint edge and the instance boundary.
float64 boundary_clearance
# Contact-model provenance: candidates carry the same version/hash the
# hardware launch logged at startup.
uint32 model_version
string model_hash
