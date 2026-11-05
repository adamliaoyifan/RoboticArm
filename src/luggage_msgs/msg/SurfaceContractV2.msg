luggage_msgs/FrameIdentityV2 identity
string geometry_hash
uint64 map_revision
string height_datum
float64[2] origin_xy_container
float64 resolution_m
float64 floor_z_container
float64 ceiling_z_container
uint32 rows
uint32 columns
# Row-major arrays, exactly rows*columns. Invalid z is canonical 0, never support.
bool[] valid
float64[] surface_z_container
string[] state
string[] confidence
# Optional derived height_above_floor uses the same validity array.
bool has_height_above_floor
float64[] height_above_floor
# [B1 snapshot identity] The publishing map's epoch, content digest over
# planning-relevant content + confirmation state, and the confirmation
# headline fields. Empty epoch/digest = not declared (pre-B1 records);
# occupancy_confirmed False means consumers must treat the map as pending.
string epoch
string content_digest
bool occupancy_confirmed
string change_event
# [Reset validity] The observer validity generation the published map
# crossed (plan OCTOMAP_RESET_VALIDITY_NEXT_20261103 §3): the planner
# admission binds map -> epoch, and to generation when declared;
# 0 = not declared (pre-reset-contract publishers).
uint64 validity_generation
