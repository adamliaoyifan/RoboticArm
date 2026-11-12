# Measurement only. Labels are frame-local, never physical cargo ownership.
# This exact cloud (including order/Float32 bytes) is the later O source input.
std_msgs/Header header
string request_id
string sensor_id
string observation_id
uint32 sampling_stride
string binding_sha256
# Complete source order: RGB, depth, mask, camera calibration, acquisition TF.
string[5] source_sha256
sensor_msgs/Image rgb
sensor_msgs/Image depth
sensor_msgs/Image instance_mask
sensor_msgs/CameraInfo camera_info
geometry_msgs/TransformStamped container_from_optical
sensor_msgs/PointCloud2 cloud
uint32[] source_pixel_indices
uint32[] frame_instance_labels
