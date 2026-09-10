# Runtime container-opening geometry derived from real sensor observations.
std_msgs/Header header
geometry_msgs/PoseWithCovariance opening_pose
# Aperture vertices expressed in the opening-pose frame.
geometry_msgs/Polygon aperture
# Estimated inner length/width/height in metres.
# Convenience only: maintained consumers validate the canonical descriptor
# below and never treat inner_size as a complete geometry contract.
float64[3] inner_size
uint32 geometry_schema_version
string geometry_encoding
string geometry_descriptor_json
string geometry_hash
float64 confidence
string source
bool valid
uint64 geometry_version
string rejection_reason
