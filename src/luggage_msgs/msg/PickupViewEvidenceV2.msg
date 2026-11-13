# Measured acquisition inputs, not inferred support or motion permission.
# Header and both images share the exact optical frame and acquisition stamp.
std_msgs/Header header
string box_id
string instance_id
uint64 generation
string domain_sha256
string evidence_sha256
# Effective rectified pinhole model used by the estimator; no guessed CameraInfo.
float64[4] intrinsics_fx_fy_cx_cy
float64[16] world_from_optical
# Encoding preserves uint16/float32/float64 numerical samples, in MILLIMETRES.
sensor_msgs/Image depth_mm
# mono8 values0/1; region is NOT automatically a verified box-face contour.
sensor_msgs/Image instance_region
# instance_mask or detection_bbox
string region_source
# Optional exact inputs for later cross-view association; no identity permission.
# Complete means the entire received reported list, not all physical boxes.
bool association_inputs_available
string association_inputs_reason
sensor_msgs/Image rgb
YoloDetections candidate_pool
int32 selected_candidate_index
