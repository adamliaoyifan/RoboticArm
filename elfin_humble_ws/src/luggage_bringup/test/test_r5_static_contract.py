#!/usr/bin/env python3
"""Static production-boundary checks for SIM-R1 Gate R5."""

from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
PRODUCTION = [
    PACKAGE / "luggage_bringup" / "production_adapter.py",
    PACKAGE / "luggage_bringup" / "production_orchestrator.py",
    PACKAGE / "scripts" / "orchestrator_node.py",
    PACKAGE / "scripts" / "operator_control.py",
    PACKAGE / "launch" / "production_orchestrator.launch.py",
]


def test_only_ros2_entry_points_are_installed():
    cmake = (PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "ament_python_install_package" in cmake
    assert "scripts/orchestrator_node.py" in cmake
    assert "scripts/operator_control.py" in cmake
    assert "scripts/ros1_reference/orchestrator_node.py" not in cmake
    assert "install(DIRECTORY launch" not in cmake
    assert "catkin" not in cmake.lower()
    assert not (PACKAGE / "COLCON_IGNORE").exists()

    wrapper = (PACKAGE / "scripts" / "orchestrator_node.py").read_text(
        encoding="utf-8"
    )
    assert "production_orchestrator" in wrapper
    assert "rospy" not in wrapper
    legacy = PACKAGE / "scripts" / "ros1_reference" / "orchestrator_node.py"
    assert legacy.is_file()
    assert "import rospy" in legacy.read_text(encoding="utf-8")


def test_production_surface_has_no_test_or_environment_shortcut():
    forbidden = (
        "spawn_next_box",
        "clear_current_box",
        "get_model_state",
        "gazebo_msgs",
        "simulation_pose_oracle",
        "sim_mode",
        "eval_truth",
        "fake_backend",
    )
    combined = "\n".join(path.read_text(encoding="utf-8") for path in PRODUCTION)
    lowered = combined.lower()
    for token in forbidden:
        assert token not in lowered
    launch = PRODUCTION[-1].read_text(encoding="utf-8")
    assert "operator_control" not in launch
    assert "start" not in launch.lower()


def test_operator_start_requires_visible_exact_confirmation():
    source = (PACKAGE / "scripts" / "operator_control.py").read_text(
        encoding="utf-8"
    )
    assert 'value.strip() == "START"' in source
    assert "start_confirmation_required" in source
    assert "Type START to confirm" in source
