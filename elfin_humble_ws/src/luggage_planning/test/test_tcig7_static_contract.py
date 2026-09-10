#!/usr/bin/env python3
"""Static packaging and reference-code checks for TCIG-7."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
PLANNING = ROOT / "luggage_planning"
MSGS = ROOT / "luggage_msgs"


def test_ros1_builder_is_reference_only_and_unchanged():
    reference = PLANNING / "scripts/ros1_reference/reachability_atlas_builder.py"
    installed = (PLANNING / "CMakeLists.txt").read_text(encoding="utf-8")
    assert reference.exists()
    assert "import rospy" in reference.read_text(encoding="utf-8")
    assert "scripts/ros1_reference" not in installed
    assert "scripts/reachability_atlas_builder_node.py" in installed


def test_runtime_message_carries_complete_geometry_identity():
    message = (MSGS / "msg/ContainerOpeningEstimate.msg").read_text(
        encoding="utf-8")
    for declaration in (
        "uint32 geometry_schema_version",
        "string geometry_encoding",
        "string geometry_descriptor_json",
        "string geometry_hash",
        "uint64 geometry_version",
    ):
        assert declaration in message


def test_migration_tool_and_ros2_adapter_are_installed():
    cmake = (PLANNING / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "scripts/migrate_reachability_atlas.py" in cmake
    assert "scripts/reachability_atlas_builder_node.py" in cmake
