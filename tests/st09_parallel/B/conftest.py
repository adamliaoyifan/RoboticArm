"""Shared pytest fixtures for the ST09-B route test battery.

Keeps the unified manifest entry ``python3 -m pytest tests/st09_parallel/B``
working with or without an install overlay: the repo root (for
``st09_contracts``) and the src tree (for ``luggage_planning``) are both
put on ``sys.path`` idempotently. ROS-dependent tests skip loudly instead
of failing collection; the mock-graph session fixture launches a minimal
mock-controller + move_group stack on a free domain (never the reserved
87/42/7) with 180 s startup / 90 s teardown budgets.
"""

from __future__ import division

import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[3]
for _path in (str(_REPO_ROOT), str(_REPO_ROOT / "src" / "luggage_planning")):
    if _path not in sys.path:
        sys.path.insert(0, _path)

#: Reserved domains: 87 final graph campaign, 42 replay, 7 the one Gazebo.
RESERVED_DOMAINS = {87, 42, 7}
DEFAULT_DOMAIN_CANDIDATES = tuple(
    d for d in range(55, 71) if d not in RESERVED_DOMAINS)
STARTUP_BUDGET_S = 180.0
TEARDOWN_BUDGET_S = 90.0

_LAUNCH_TEMPLATE = '''\
"""Generated minimal mock graph for ST09-B tests (mock controllers + move_group)."""
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

URDF_PATH = os.environ["ST09_B_URDF"]
SRDF_PATH = os.environ["ST09_B_SRDF"]
CONTROLLERS = os.environ["ST09_B_CONTROLLERS"]


def _text(path):
    with open(path, "r") as handle:
        return handle.read()


def generate_launch_description():
    description = {"robot_description": _text(URDF_PATH)}
    semantic = {"robot_description_semantic": _text(SRDF_PATH)}
    moveit_config = (
        MoveItConfigsBuilder("S20", package_name="elfin_moveit_config")
        .trajectory_execution(file_path="config/moveit_controllers.yaml",
                              moveit_manage_controllers=False)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs())
    return LaunchDescription([
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="robot_state_publisher", output="screen",
            parameters=[description]),
        Node(
            package="controller_manager", executable="ros2_control_node",
            name="controller_manager", output="screen",
            parameters=[description, CONTROLLERS],
            remappings=[("~/robot_description", "/robot_description")]),
        Node(
            package="controller_manager", executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager",
                       "/controller_manager", "--controller-manager-timeout",
                       "60"], output="screen"),
        Node(
            package="moveit_ros_move_group", executable="move_group",
            name="move_group", output="screen",
            parameters=[moveit_config.to_dict(), description, semantic,
                        {"publish_robot_description_semantic": True,
                         "allow_trajectory_execution": True,
                         "use_sim_time": False}]),
    ])
'''


def _ros_available():
    try:
        from ament_index_python.packages import get_package_share_directory  # noqa: F401
        import rclpy  # noqa: F401
        return True
    except Exception:
        return False


def _pick_domain():
    override = os.environ.get("ST09_B_DOMAIN_ID")
    if override is not None:
        return int(override)
    for domain in DEFAULT_DOMAIN_CANDIDATES:
        probe = subprocess.run(
            ["timeout", "4", "ros2", "node", "list", "--no-daemon"],
            env=dict(os.environ, ROS_DOMAIN_ID=str(domain),
                     ROS_LOCALHOST_ONLY="1"),
            capture_output=True, text=True)
        nodes = [line for line in (probe.stdout or "").splitlines()
                 if line.strip()]
        if not nodes:
            return domain
    return None


class MockGraphSession(object):
    """A running minimal mock graph plus an in-process client node."""

    def __init__(self, process, node, spin_thread, domain, workdir):
        self.process = process
        self.node = node
        self.spin_thread = spin_thread
        self.domain = domain
        self.workdir = workdir


@pytest.fixture(scope="session")
def mock_graph(request, tmp_path_factory):
    """Minimal mock graph: mock ros2_control + move_group, no Gazebo.

    Skips loudly (never fails collection) when ROS or the install overlay
    is missing or no free domain exists. Never uses domains 87/42/7.
    """
    if not _ros_available():
        pytest.skip("ROS/moveit not sourced; mock-graph tests skipped")
    pytest.importorskip("moveit_msgs.msg")
    from ament_index_python.packages import get_package_share_directory
    from luggage_description.xacro_robot_with_scene_base import expand_and_patch

    domain = _pick_domain()
    if domain is None:
        pytest.skip("no free ROS domain in %s (set ST09_B_DOMAIN_ID)"
                    % (DEFAULT_DOMAIN_CANDIDATES,))
    workdir = tmp_path_factory.mktemp("b-mock-graph")
    xacro_path = os.path.join(
        get_package_share_directory("luggage_description"),
        "urdf", "elfin_s20_with_camera.urdf.xacro")
    srdf_path = os.path.join(
        get_package_share_directory("luggage_description"),
        "config", "S20_with_camera.srdf")
    urdf_text, _pose = expand_and_patch(
        xacro_path,
        xacro_args=["hardware_plugin:=mock_components/GenericSystem",
                    "use_gz_sim:=false"])
    srdf_text = open(srdf_path, encoding="utf-8").read().replace(
        'name="elfin_s20_with_camera"', 'name="S20"', 1)
    urdf_file = workdir / "robot.urdf"
    srdf_file = workdir / "robot.srdf"
    urdf_file.write_text(urdf_text)
    srdf_file.write_text(srdf_text)
    launch_file = workdir / "b_mock_graph_launch.py"
    launch_file.write_text(_LAUNCH_TEMPLATE)

    controllers = os.path.join(
        get_package_share_directory("elfin_control"),
        "config", "elfin_controllers.yaml")
    env = dict(
        os.environ,
        ROS_DOMAIN_ID=str(domain),
        ROS_LOCALHOST_ONLY="1",
        ST09_B_URDF=str(urdf_file),
        ST09_B_SRDF=str(srdf_file),
        ST09_B_CONTROLLERS=controllers,
        RCUTILS_LOGGING_BUFFERED_STREAM="1")
    log = (workdir / "launch.log").open("w")
    process = subprocess.Popen(
        ["ros2", "launch", str(launch_file)],
        env=env, stdout=log, stderr=subprocess.STDOUT, cwd=str(workdir))

    import rclpy

    # Same discovery scope as the launched stack: without this the
    # in-process client never sees the mock graph's services.
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    rclpy.init()
    node = rclpy.create_node("st09_b_edge_test")
    stop = threading.Event()
    spin_thread = threading.Thread(
        target=_spin_until, args=(node, stop), daemon=True)
    spin_thread.start()
    session = MockGraphSession(process, node, spin_thread, domain, workdir)
    request.addfinalizer(lambda: _teardown(session))

    ready = _wait_ready(node, STARTUP_BUDGET_S)
    if not ready:
        _teardown(session)
        pytest.skip("mock graph did not become ready in %.0fs (log: %s)"
                    % (STARTUP_BUDGET_S, workdir / "launch.log"))
    return session


def _spin_until(node, stop):
    import rclpy

    while not stop.is_set() and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)


def _wait_ready(node, budget_s):
    from moveit_msgs.srv import GetCartesianPath

    deadline = time.monotonic() + float(budget_s)
    cartesian = node.create_client(GetCartesianPath, "/compute_cartesian_path")
    scene = node.create_client(
        __import__("moveit_msgs.srv", fromlist=["ApplyPlanningScene"])
        .ApplyPlanningScene, "/apply_planning_scene")
    while time.monotonic() < deadline:
        if cartesian.service_is_ready() and scene.service_is_ready():
            return True
        time.sleep(0.25)
    return False


def _teardown(session):
    if session.process.poll() is None:
        session.process.send_signal(signal.SIGINT)
        try:
            session.process.wait(timeout=TEARDOWN_BUDGET_S / 3.0)
        except subprocess.TimeoutExpired:
            session.process.terminate()
            try:
                session.process.wait(timeout=TEARDOWN_BUDGET_S / 3.0)
            except subprocess.TimeoutExpired:
                session.process.kill()
                session.process.wait(timeout=10.0)
    import rclpy

    if rclpy.ok():
        rclpy.shutdown()
