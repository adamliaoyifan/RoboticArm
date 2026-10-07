# ST09-I: per-leg verdict from the batched route validator.
string edge_id
bool valid
# First failure name: SAMPLING_DISCIPLINE / JOINT_LIMIT /
# VELOCITY_LIMIT / ACCELERATION_LIMIT / TIME_NOT_MONOTONIC /
# COLLISION / TCP_CONSTRAINT / MODEL_MISMATCH / BUDGET_EXCEEDED,
# empty when valid.
string reason
string detail
# Index of the first invalid sample; n_samples when none.
uint32 first_invalid_sample
uint32 n_samples
# Measured minimum distance per sample (ACM-exempt and support-exempt
# pairs excluded); negative values report penetration depth.
float64[] min_distance_m
# Contacts at the first invalid sample.
luggage_msgs/RouteContact[] contacts
# All contacts on the declared support pair (for certifier legality
# judgement), across samples where the pair touches.
luggage_msgs/RouteContact[] support_contacts
float64 leg_time_ns
# Time spent in checkCollision for this leg, nanoseconds.
float64 collision_time_ns
# Time spent in distanceRobot for this leg, nanoseconds.
float64 distance_time_ns
# distanceRobot calls made for this leg.
uint32 distance_queries
