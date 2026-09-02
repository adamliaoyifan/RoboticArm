# Per-RGB-frame YOLO boxes. header.stamp is the preprocessor primary stamp.
# header.frame_id is the optical frame; bbox values are pixels in that image.
std_msgs/Header header
uint32 frame_seq
uint32 generation
string instance_id
uint32 image_width
uint32 image_height
string backend
YoloBox[] detections
