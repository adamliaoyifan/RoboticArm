# ST09-I: one certified-input leg of a ValidateRouteV2 request.
#
# The leg carries the ORIGINAL knot trajectory plus the states sampled
# from it at the frozen certification spacing (the interpolation model of
# the actual controller, built by the caller). The service verifies the
# sampling discipline and validates the given states — it never
# re-plans, never runs IK, and never modifies the input.
string edge_id
# Reporting only: TRANSIT/TRAVERSE/INSERT/DESCEND/RETREAT/RETURN/...
string edge_class
# Informational: payload attached during this leg.
bool carrying
# Index into the request's load_state_scenes: the frozen scene this
# leg is validated against.
uint32 load_state_index
# Canonical joint order; every leg of one request shares it (verified).
string[] joint_names
# Sampled states: monotonic times within the leg, row-major positions.
uint32 n_samples
float64[] sample_times_ns
float64[] sample_positions_flat
# The original knots with velocities/accelerations; the service verifies
# knots are included in the samples, per-knot velocity/acceleration
# bounds and time monotonicity.
trajectory_msgs/JointTrajectory knots
# Designated support landing for this leg: contacts between the attached
# body support_payload_id and the world object support_object_id are
# reported separately (support_contacts) and are exempt from this leg's
# collision verdict and min-distance; the CERTIFIER judges their
# legality (pairing, box bottom, region, normal, penetration). Empty
# support_payload_id disables the exemption.
string support_payload_id
string support_object_id
# Optional per-sample TCP pose constraint (e.g. carry-level orientation):
# empty tcp_link disables it.
string tcp_link
geometry_msgs/Pose tcp_pose
float64 tcp_position_tol_m
float64 tcp_orientation_tol_rad
# Optional spatial sampling verification link (empty = time-only): the
# TCP translation between consecutive samples must stay within
# max_sample_step_m.
string spatial_check_link
