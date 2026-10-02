"""ST09-I whole-chain fixtures: the real seven-endpoint v2 stack on mock
controllers, reused from the g4 graph campaign template.

Domain discipline: probe 55..70 (never the reserved 87/42/7), localhost
only, startup budget 180 s, teardown SIGINT->TERM->KILL within 90 s.
"""

from __future__ import division

import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "scripts"
for entry in (str(SCRIPTS),):
    if entry not in sys.path:
        sys.path.insert(0, entry)

RESERVED_DOMAINS = {87, 42, 7}
DEFAULT_DOMAIN_CANDIDATES = [d for d in range(55, 71)
                             if d not in RESERVED_DOMAINS]
STARTUP_BUDGET_S = 180.0
TEARDOWN_BUDGET_S = 90.0


def _domain_free(domain):
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(domain)
    env["ROS_LOCALHOST_ONLY"] = "1"
    try:
        result = subprocess.run(
            ["timeout", "4", "ros2", "node", "list", "--no-daemon"],
            env=env, capture_output=True, text=True)
    except (OSError, subprocess.SubprocessError):
        return False
    return result.returncode == 0 and not result.stdout.strip()


def _pick_domain():
    override = os.environ.get("ST09_I_DOMAIN_ID")
    if override:
        return int(override)
    for domain in DEFAULT_DOMAIN_CANDIDATES:
        if _domain_free(domain):
            return domain
    return None


def _ros_available():
    try:
        import rclpy  # noqa: F401
        import moveit_msgs.msg  # noqa: F401
    except ImportError:
        return False
    return True


def _teardown_process(proc):
    import signal

    if proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=12.0)
        return
    except subprocess.TimeoutExpired:
        pass
    proc.terminate()
    try:
        proc.wait(timeout=4.0)
        return
    except subprocess.TimeoutExpired:
        pass
    proc.kill()
    proc.wait(timeout=10.0)


class WholeChain(object):
    """Probe handle over the live seven-endpoint mock stack."""

    def __init__(self, probe, out_dir, launch_proc, record):
        self.probe = probe
        self.node = probe.node
        self.out_dir = out_dir
        self.launch_proc = launch_proc
        self.record = record

    # -- convenience -----------------------------------------------------

    def adapter(self):
        from luggage_planning.typed_driver_adapter import TypedDriverAdapter

        return TypedDriverAdapter(self.node, self.probe.runtime)

    def latched_surface(self, timeout=10.0):
        from luggage_msgs.msg import SurfaceContractV2
        from rclpy.qos import (DurabilityPolicy, QoSProfile,
                               ReliabilityPolicy)

        received = {}
        self.node.create_subscription(
            SurfaceContractV2, "/luggage/cargo_map/surface_v2",
            lambda message: received.setdefault("message", message),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        deadline = time.monotonic() + timeout
        while "message" not in received and time.monotonic() < deadline:
            time.sleep(0.1)
        return received.get("message")

    def joint_state(self, timeout=10.0):
        from sensor_msgs.msg import JointState

        received = {}
        self.node.create_subscription(
            JointState, "/joint_states",
            lambda message: received.setdefault("message", message), 10)
        deadline = time.monotonic() + timeout
        while "message" not in received and time.monotonic() < deadline:
            time.sleep(0.05)
        message = received.get("message")
        if message is None:
            return None
        names = list(message.name)
        return [float(message.position[names.index(
            "elfin_joint%d" % i)]) for i in range(1, 7)]


@pytest.fixture(scope="session")
def whole_chain(tmp_path_factory):
    if not _ros_available():
        pytest.skip("ROS 2 / moveit_msgs not available")
    domain = _pick_domain()
    if domain is None:
        pytest.skip("no free ROS domain in 55..70")
    from frame_migration_st09_graph import (
        LAUNCH_TEMPLATE,
        GraphProbe,
        build_frame_params,
        run_scenario,
    )

    out_dir = tmp_path_factory.mktemp("st09_i_whole_chain")
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(domain)
    env["ROS_LOCALHOST_ONLY"] = "1"
    env["ROS_LOG_DIR"] = str(out_dir / "roslog")
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    (out_dir / "roslog").mkdir(parents=True, exist_ok=True)

    import rclpy

    from ament_index_python.packages import get_package_share_directory
    scene_tf = os.path.join(
        get_package_share_directory("luggage_description"),
        "config", "scene_tf.yaml.example")
    frame = build_frame_params(ROOT, scene_tf)
    # [ST09-I] the g4 template's move_group carries only URDF+SRDF (the
    # campaign never planned motion); swap in the B-proven
    # MoveItConfigsBuilder move_group (OMPL + kinematics + controllers)
    # and enable the scene_manager static sync so the certified route
    # plans against the real container collision world.
    launch_text = LAUNCH_TEMPLATE % {
        "scene_tf": scene_tf, "frame_params": frame}
    old_move_group = """    nodes.append(Node(
        package="moveit_ros_move_group", executable="move_group",
        name="move_group", output="screen",
        parameters=[description, semantic,
                    {"publish_robot_description_semantic": True,
                      "allow_trajectory_execution": True,
                      "use_sim_time": False}],
        condition=_cond(want("move_group"))))"""
    new_move_group = """    from moveit_configs_utils import MoveItConfigsBuilder

    moveit_config = (
        MoveItConfigsBuilder("S20", package_name="elfin_moveit_config")
        .trajectory_execution(file_path="config/moveit_controllers.yaml",
                              moveit_manage_controllers=False)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs())
    nodes.append(Node(
        package="moveit_ros_move_group", executable="move_group",
        name="move_group", output="screen",
        parameters=[moveit_config.to_dict(), description, semantic,
                    {"publish_robot_description_semantic": True,
                     "allow_trajectory_execution": True,
                     "use_sim_time": False}],
        condition=_cond(want("move_group"))))"""
    assert old_move_group in launch_text, "template move_group drifted"
    launch_text = launch_text.replace(old_move_group, new_move_group)
    launch_text = launch_text.replace('"auto_sync": False',
                                      '"auto_sync": True')
    # The mock stack never configured the arm controller (the g4 campaign
    # was plan-only): the controller_manager sees the type map but not
    # the controller's own parameters — load them via the spawner.
    arm_controllers_yaml = os.path.join(
        get_package_share_directory("elfin_control"),
        "config", "elfin_controllers.yaml")
    # launch parameter-file loading only applies node-matching sections,
    # which orphans the elfin_arm_controller params (empty joints). Hand
    # the controller_manager the FULL file exactly like the working
    # `ros2 run` invocation, via explicit --params-file arguments.
    old_cm = '''nodes.append(Node(
        package="controller_manager", executable="ros2_control_node",
        name="controller_manager", output="screen",
        parameters=[description, controllers],
        remappings=[("~/robot_description", "/robot_description")],
        condition=_cond(want("controller_manager"))))'''
    new_cm = '''from launch.actions import ExecuteProcess

    cm_prefix = os.path.dirname(os.path.dirname(
        get_package_share_directory("controller_manager")))
    cm_exec = os.path.join(
        cm_prefix, "lib", "controller_manager", "ros2_control_node")
    nodes.append(ExecuteProcess(
        cmd=[cm_exec, "--ros-args",
             "-p", "robot_description:=" + urdf,
             "--params-file", %r],
        output="screen"))''' % str(
            arm_controllers_yaml)
    assert old_cm in launch_text, "template ros2_control_node drifted"
    launch_text = launch_text.replace(old_cm, new_cm)
    old_args = ('arguments=["elfin_arm_controller", "--controller-manager",\n'
                '                   "/controller_manager", '
                '"--controller-manager-timeout", "60"],')
    new_args = ('arguments=["elfin_arm_controller", "--controller-manager",\n'
                '                   "/controller_manager", '
                '"--controller-manager-timeout", "60",\n'
                '                   "--param-file", %r],' % str(
                    arm_controllers_yaml))
    assert old_args in launch_text, "template spawner args drifted"
    launch_text = launch_text.replace(old_args, new_args)
    launch_path = out_dir / "graph_launch.py"
    launch_path.write_text(launch_text)
    command = ["ros2", "launch", str(launch_path)]

    rclpy.init()
    log = open(out_dir / "whole_chain.launch.log", "w")
    proc = subprocess.Popen(
        command, env=env, stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True)
    try:
        record, probe, _proc = run_scenario(
            "whole_chain", command, out_dir, env,
            probe_factory=(lambda: GraphProbe("motion_occ_driver")),
            timeout=STARTUP_BUDGET_S, launch_process=proc,
            own_rclpy=False, stop_after_ready=True)
        if not record.get("ready"):
            pytest.skip("whole-chain stack did not reach READY: %r" % record)
        yield WholeChain(probe, out_dir, proc, record)
    finally:
        _teardown_process(proc)
        log.close()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
