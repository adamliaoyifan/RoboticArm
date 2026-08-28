# Runtime container-opening geometry derived from real sensor observations.
std_msgs/Header header
geometry_msgs/PoseWithCovariance opening_pose
# Aperture vertices expressed in the opening-pose frame.
geometry_msgs/Polygon aperture
# Estimated inner length/width/height in metres.
float64[3] inner_size
float64 confidence
string source
bool valid
uint64 geometry_version
string rejection_reason
