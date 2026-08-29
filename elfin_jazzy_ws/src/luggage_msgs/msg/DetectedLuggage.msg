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
