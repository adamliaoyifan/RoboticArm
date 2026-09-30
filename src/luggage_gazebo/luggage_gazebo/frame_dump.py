"""Compatibility import; the ROS-free dump contract belongs to description."""
from luggage_description.frame_dump import (
    SCHEMA, adapt_legacy_surface, fail, read_dump, validate_dump,
    world_marker_payload, write_dump,
)

__all__ = ["SCHEMA", "adapt_legacy_surface", "fail", "read_dump", "validate_dump",
           "world_marker_payload", "write_dump"]
