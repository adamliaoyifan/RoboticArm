string status
string sensor_id
string observation_id
int64 input_stamp_ns
# Subscription callback entry in CLOCK_MONOTONIC ns (steady_clock). 0 = not
# recorded (service-mode integration). Distinct from input_stamp_ns, which is
# the cloud header stamp in the ROS clock domain.
int64 callback_enter_ns
uld_occ_observer/SnapshotIdentity identity
uint64 raw_points
uint64 accepted_points
uint64 free_keys
uint64 hit_keys
uint64 changed_keys
uint64 occupancy_changed_keys
uint64 rebuilt_blocks
float64 preprocessing_ms
float64 preparation_ms
float64 fusion_ms
float64 total_ms
float64 queue_ms
