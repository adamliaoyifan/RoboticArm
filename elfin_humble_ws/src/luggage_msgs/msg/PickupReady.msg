# Versioned operator confirmation for the current pickup prompt.
#
# QoS contract: RELIABLE + VOLATILE. This message intentionally carries no
# box geometry, pose, identity, dimensions, or simulation truth.
uint32 SCHEMA_VERSION=1
uint32 schema_version
std_msgs/Header header
string request_id
string operator_id
