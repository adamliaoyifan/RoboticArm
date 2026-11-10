# Frozen at confirmed attachment. All transforms map child into tool coordinates.
# Nominal dimensions remain measured; conservative dimensions are never re-expanded.
float64[3] measured_dimensions_wdh
float64[3] tool_box_translation
float64[4] tool_box_quaternion_xyzw
float64[3] conservative_dimensions_wdh
float64[3] tool_conservative_translation
float64[4] tool_conservative_quaternion_xyzw
# Diagnostic policy provenance, not a statistical coverage claim.
float64 yaw_half_range_deg
float64[3] size_growth_fraction_wdh
float64[2] position_half_range_tool_xy_m
float64 away_from_contact_position_error_m
string nominal_digest
string geometry_digest
