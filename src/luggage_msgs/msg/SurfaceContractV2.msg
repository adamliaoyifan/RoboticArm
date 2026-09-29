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
