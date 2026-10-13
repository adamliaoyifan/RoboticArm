"""The matrix atlas query uses container tool yaw, the builder's frame."""
import math
import os
import sys

_ROOT = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
_SCRIPTS = os.path.join(_ROOT, "scripts", "st09_i")
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from atlas_yaw_probe import fixed_pose_report  # noqa: E402


def test_matrix_yaw_matches_builder_and_not_world_yaw():
    report = fixed_pose_report(math.pi)
    assert report["convention_mismatch"] is False
    assert report["query_frame"] == "container_link tool yaw"
    for row in report["rows"]:
        assert row["matrix_matches_production"]
        assert row["matrix_matches_builder"]
        assert row["world_yaw_is_other_bin"]
        assert row["heading_parameter_is_other_bin"]
