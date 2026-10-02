# ST09-I: one collision contact reported by the batched route validator.
# Body names are collision-body names exactly as MoveIt reports them:
# robot link names, attached object ids, or world object ids.
string body_1
string body_2
# 0 = unknown, 1 = robot link, 2 = robot-attached object, 3 = world object
uint8 body_type_1
uint8 body_type_2
# Index of the sample the contact belongs to (leg-local).
uint32 sample_index
# Penetration depth in meters (0.0 for touching-only contacts).
float64 depth
# Contact position and normal (body_1 -> body_2), planning frame.
geometry_msgs/Point position
geometry_msgs/Vector3 normal
