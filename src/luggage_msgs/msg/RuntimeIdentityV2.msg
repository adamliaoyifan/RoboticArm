# Heartbeat at 4 Hz on /frame_contracts/v2/runtime_identity.
# Receiver uses its own monotonic clock for the two-second lease.
uint32 contract_version
string peer
string incarnation
string contract_digest
string scene_revision
string calibration_revision
string urdf_sha256
string srdf_sha256
string tool_link
string[] joint_names
string geometry_hash
string atlas_sha256
string atlas_certification_sha256
luggage_msgs/RuntimeEndpointV2[] endpoints
# Canonical JSON of locally resolved settings, independently compared to intent.
string resolved_config_json
luggage_msgs/RuntimeArtifactV2[] artifacts
luggage_msgs/RuntimePackageV2[] package_prefixes
# Mutable heartbeat/diagnostics are excluded from the identity fingerprint.
uint64 heartbeat_sequence
bool ready
string readiness_code
