# Versioned operator prompt published by the production orchestrator.
#
# QoS contract: RELIABLE + TRANSIENT_LOCAL. The prompt is the only online
# source of request_id values accepted by PickupReady.
uint32 SCHEMA_VERSION=1
uint32 schema_version
std_msgs/Header header
string request_id
string kind
string state
string display_text
