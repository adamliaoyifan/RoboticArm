# Fused YOLO + PCA for one RGB/cargo stamp.
# header.stamp is the preprocessor primary stamp (join key).
# header.frame_id is the TF frame of the 3D PCA pose/centroid (world).
std_msgs/Header header
string yolo_optical_frame
uint32 frame_seq
uint32 generation
string instance_id
YoloBox[] yolo
bool pca_valid
string pca_reason
string pca_source
float32 pca_confidence
uint32 n_cargo_points
geometry_msgs/Point centroid
DetectedLuggage box
