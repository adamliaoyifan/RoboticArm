# Container coordinates only. use_current and token are mutually exclusive.
string request_id
string frame_id
bool use_current
luggage_msgs/CargoMapQueryIdentityV2 expected_identity
string snapshot_token
# Closing does not renew a token; every token expires after five wall seconds.
bool release_after_query
bool has_region
geometry_msgs/Point region_min
geometry_msgs/Point region_max
# 0 selects default 1024; maximum 4096.
uint32 limit
string cursor
