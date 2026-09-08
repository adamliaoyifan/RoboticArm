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
# --- Support-plane diagnostics (E0, platform-free height) ---
# Raw-cloud support fit status for this acquisition stamp; sufficient to
# debug why a result is TOP_ONLY vs FULL_3D.
bool support_valid
string support_reason
float64 support_z
float64 support_confidence
float64 support_residual
float64 support_side_coverage
uint32 support_inliers
uint8 GEOMETRY_TOP_ONLY=0
uint8 GEOMETRY_FULL_3D=1
uint8 geometry_level
