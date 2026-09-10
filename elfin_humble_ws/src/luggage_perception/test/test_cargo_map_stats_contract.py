#!/usr/bin/env python3
"""Static adapter contract for typed GetCargoMapStats geometry fields."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SERVICE = ROOT / "luggage_msgs" / "srv" / "GetCargoMapStats.srv"
NODE = ROOT / "luggage_perception" / "scripts" / "cargo_volume_mapper_node.py"


def test_service_declares_typed_geometry_identity_and_physical_volumes():
    fields = {
        line.split("#", 1)[0].strip()
        for line in SERVICE.read_text(encoding="utf-8").splitlines()
    }
    assert {
        "int32 geometry_schema_version",
        "string geometry_hash",
        "float64 usable_volume",
        "float64 unknown_volume",
        "float64 free_volume",
        "float64 occupied_volume",
    }.issubset(fields)


def test_ros2_adapter_populates_each_typed_field_directly():
    source = NODE.read_text(encoding="utf-8")
    for field in (
            "geometry_schema_version", "geometry_hash", "usable_volume",
            "unknown_volume", "free_volume", "occupied_volume"):
        assert "response.%s =" % field in source
        assert "stats[\"%s\"]" % field in source
