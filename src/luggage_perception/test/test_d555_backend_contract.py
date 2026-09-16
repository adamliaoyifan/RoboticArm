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
    assert params["camera_pair_tolerance_sec"] == 0.0
    assert params["camera_info_shared"] is False
    assert params["input.depth_image"].endswith("/compressed")
    assert params["use_sim_time"] is False


def test_site_profile_b_is_humble_thresholds_on_wall_clock():
    profile = yaml.safe_load(
        (PACKAGE / "config" / "preprocessor_d555_site.yaml").read_text())
    params = profile["sensor_preprocessor"]["ros__parameters"]
    assert params["use_sim_time"] is False
    assert params["motion_gate.enabled"] is False
    assert params["camera_pair_tolerance_sec"] == 0.050
    assert params["camera_wait_deadline_sec"] == 0.150
    assert params["camera_info_shared"] is False
    # Dead override: nothing reads it, so it must not look like a gate.
    assert "camera_emit_rgb_only" not in params
    assert params["input.use_compressed"] is True
    assert params["enable_lidar_output"] is False
    assert params["input.depth_image"].endswith("/compressed")
    assert params["output_cloud_frame"] == "d555_color_optical_frame"


def test_sim_profile_declares_its_single_shared_camera_info():
    profile = yaml.safe_load(
        (PACKAGE / "config" / "sensor_preprocessor.yaml").read_text())
    params = profile["sensor_preprocessor"]["ros__parameters"]
    # gz rgbd_camera publishes one CameraInfo; the alias is declared, not
    # invented at runtime when a slot happens to be empty.
    assert params["camera_info_shared"] is True
    assert params["input.color_camera_info"] == ""
    assert params["camera_pair_tolerance_sec"] == 0.0


def test_driver_is_raw_only_and_pointcloud_is_disabled():
    launch_source = (
        PACKAGE / "launch" / "d555_canonical_pipeline.launch.py").read_text()
    assert '"image_transport/raw"' in launch_source
    assert '"pointcloud.enable": False' in launch_source
    assert '"enable_sync": True' in launch_source
    assert "d555_transport_adapter_node.py" in launch_source
    assert "sensor_preprocessor_node.py" in launch_source


def test_layered_site_profiles_stay_self_consistent():
    """hardware_pick layers the base yaml under a D555 overlay."""
    base = yaml.safe_load(
        (PACKAGE / "config" / "sensor_preprocessor.yaml").read_text()
    )["sensor_preprocessor"]["ros__parameters"]
    for name in ("preprocessor_d555_live.yaml", "preprocessor_d555_site.yaml"):
        overlay = yaml.safe_load(
            (PACKAGE / "config" / name).read_text()
        )["sensor_preprocessor"]["ros__parameters"]
        merged = dict(base)
        merged.update(overlay)
        # A shared-info declaration must not survive next to a real colour
        # info topic: the node refuses that combination at startup.
        assert merged["camera_info_shared"] is False
        assert merged["input.color_camera_info"]


def test_preprocessor_node_publishes_each_product_with_its_own_stamp():
    source = (
        PACKAGE / "scripts" / "sensor_preprocessor_node.py").read_text()
    # A blanket rewrite to primary_stamp would make a tolerance pair look
    # like one exposure to every exact-stamp consumer.
    assert "color_stamp = adapters.sec_to_stamp(obs.rgb_stamp)" in source
    assert "depth_stamp = adapters.sec_to_stamp(obs.depth_stamp)" in source
    assert "adapters.sec_to_stamp(obs.primary_stamp)" not in source


def test_preprocessor_node_declares_camera_info_provenance():
    source = (
        PACKAGE / "scripts" / "sensor_preprocessor_node.py").read_text()
    assert '"camera_info_shared": False' in source
    # The depth info topic owns one slot; aliasing is the core's decision.
    assert 'update_camera_info(frame, slots=("depth",))' in source
    assert 'slots=("depth", "color")' not in source
    assert "camera_info_shared is true but input.color_camera_info" in source


def test_transport_adapter_does_not_own_rgbd_pairing():
    source = (
        PACKAGE / "scripts" / "d555_transport_adapter_node.py").read_text()
    assert "ApproximateTimeSynchronizer" not in source
    assert "TimeSynchronizer" not in source
    assert "DeviceClockMapper" in source
