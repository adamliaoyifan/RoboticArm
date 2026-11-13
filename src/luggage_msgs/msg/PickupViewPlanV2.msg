# Plan-only empty-arm view, independently retained by its preparation owner.
# Not a six-leg operation or controller permission. Execution requires an owner
# lookup, current source/start/empty-attachment admission and fresh native check.
string request_id
string task_request_id
string domain_sha256
string view_sha256
string query_id
string model_sha256
luggage_msgs/OperationMapSourceV2 map_source
luggage_msgs/JointOperationLegV2 leg
string route_digest
string certificate_id
