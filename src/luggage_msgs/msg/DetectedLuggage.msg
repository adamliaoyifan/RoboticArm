# One suitcase detected on the ground (or in scene).
string id
geometry_msgs/Pose pose
float64 width
float64 height
float64 depth
# False when the top-face PCA heading is not identifiable (near-square).
bool yaw_valid
# max(width, depth) / min(width, depth) of the reported size.
float64 aspect_ratio
# --- Platform-free geometry contract (E0, docs/plans/platform_free_height_eng_todo.md) ---
# Acquisition identity of the sensor observation that produced this result.
std_msgs/Header header
# Measured pickup contact surface. Its Z is the sole Z input for pick
# waypoint generation; valid only when top_surface_valid is true.
geometry_msgs/Pose top_surface_pose
bool top_surface_valid
float64 top_surface_confidence
# Height/center-Z are geometric measurements only when height_valid is true.
# A catalog/configured prior may populate the numeric height with
# height_valid=false; consumers must not build collision geometry from it.
bool height_valid
float64 height_confidence
uint8 HEIGHT_SOURCE_UNAVAILABLE=0
uint8 HEIGHT_SOURCE_MEASURED_SUPPORT=1
uint8 HEIGHT_SOURCE_CONFIGURED_SUPPORT=2
uint8 HEIGHT_SOURCE_CATALOG_PRIOR=3
uint8 height_source
