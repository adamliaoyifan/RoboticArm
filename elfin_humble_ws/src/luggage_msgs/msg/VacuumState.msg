# Vacuum gripper state (sim = attached flag; hardware = suction sensed).
# Same topic shape for both backends; transient-local so late consumers
# (eval driver, GUI) see the last state immediately.
bool attached
bool vacuum_on
string fail_reason
float64 contact_distance
float64 retention_margin
float64 tilt_deg
