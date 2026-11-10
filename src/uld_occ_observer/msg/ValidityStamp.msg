# Validity-board view stamped at this response's commit check (the second
# epoch/state check of the reset contract). A response with state=VALID and
# generation G was admitted while the board was exactly at G; any other state
# proves the request lost (or has not yet rejoined) the validity race.
string state
uint64 generation
uint64 epoch
string reason
