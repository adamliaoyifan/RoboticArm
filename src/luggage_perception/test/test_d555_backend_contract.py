"""Static composition checks that do not require D555 hardware."""

from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def test_live_profile_enables_compressed_edge_and_motion_gate():
    profile = yaml.safe_load(
        (PACKAGE / "config" / "preprocessor_d555_live.yaml").read_text())
    params = profile["sensor_preprocessor"]["ros__parameters"]
    assert params["input.use_compressed"] is True
    assert params["motion_gate.enabled"] is True
    assert params["enable_lidar_output"] is False
    assert params["camera_pair_tolerance_sec"] <= 0.005
    assert params["input.depth_image"].endswith("/compressed")


def test_driver_is_raw_only_and_pointcloud_is_disabled():
    launch_source = (
        PACKAGE / "launch" / "d555_canonical_pipeline.launch.py").read_text()
    assert '"image_transport/raw"' in launch_source
    assert '"pointcloud.enable": False' in launch_source
    assert '"enable_sync": True' in launch_source
    assert "d555_transport_adapter_node.py" in launch_source
    assert "sensor_preprocessor_node.py" in launch_source


def test_transport_adapter_does_not_own_rgbd_pairing():
    source = (
        PACKAGE / "scripts" / "d555_transport_adapter_node.py").read_text()
    assert "ApproximateTimeSynchronizer" not in source
    assert "TimeSynchronizer" not in source
    assert "DeviceClockMapper" in source
