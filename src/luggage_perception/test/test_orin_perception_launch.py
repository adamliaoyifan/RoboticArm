"""Static checks for the Orin Humble perception launch (no ROS graph)."""

from pathlib import Path

LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "orin_perception.launch.py"


def test_orin_launch_uses_site_preprocessor_not_replay():
    src = LAUNCH.read_text(encoding="utf-8")
    assert "preprocessor_d555_orin.yaml" in src
    assert "preprocessor_d555_site.yaml" in src
    assert "preprocessor_d555_replay.yaml" in src
    assert "Do not pass" in src
    assert 'default_value=orin_pp' in src


def test_orin_launch_wires_d555_adapter_yolo_livox():
    src = LAUNCH.read_text(encoding="utf-8")
    assert "realsense2_camera_node" in src
    assert "d555_transport_adapter_node.py" in src
    assert '"image_transport/raw"' in src
    assert '"pointcloud.enable": False' in src
    assert '"publish_compressed": False' in src
    assert '"publish_raw": True' in src
    assert "image_raw/compressed" in src
    assert "image_hw/compressed" in src
    assert '"publish_tf": True' in src
    assert '"tf_publish_rate": 0.0' in src
    assert '"initial_reset": True' in src
    assert '"max_rate_hz": 0.0' in src
    assert '"max_rate_hz": 15.0' not in src
    overlay = src.index('"publish_overlay"')
    assert 'default_value="true"' in src[overlay:overlay + 220]
    assert "publish_overlay:=false" in src
    assert "librealsense_dds/lib" in src
    assert "sensor_preprocessor_node.py" in src
    assert "semantic_segmenter_node.py" in src
    assert "semantic_point_filter_node.py" in src
    assert "luggage_detector_node.py" in src
    assert "require_backend" in src
    assert "yolo_world" in src
    assert 'default_value="cuda"' in src
    assert "CUDA-only" in src
    assert "mid360.launch.py" in src
    assert "MID360s_config.json" in src
    assert "hardware_pick.launch.py" not in src
    assert "d555_rgbd.launch.py" not in src
    assert "/lib/x86_64-linux-gnu" not in src


def test_orin_launch_does_not_start_planning_or_cps():
    src = LAUNCH.read_text(encoding="utf-8")
    assert 'executable="move_group"' not in src
    assert 'name="scene_manager"' not in src
    assert "jazzy_real.launch.py" not in src
    assert 'executable="trajectory_executor"' not in src
    assert "d555_rgbd.launch.py" not in src
