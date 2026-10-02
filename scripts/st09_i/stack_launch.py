"""ST09-I Phase 1: regenerate and launch the mock whole-chain stack.

Mirrors the launch generation of tests/st09_parallel/I/conftest.py (same
assert-guarded template patches, so template drift fails loudly here too)
so the Phase 1 failure-freeze and control probes run against exactly the
stack the whole-chain tests use.

Usage:
    python3 scripts/st09_i/stack_launch.py --out /tmp/st09_i_stack [--wait 180]

Writes <out>/graph_launch.py, starts `ros2 launch` under setsid, waits for
runtime READY, and leaves the stack running (pid in <out>/launch.pid).
"""

from __future__ import division

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / "scripts"
for entry in (str(SCRIPTS),):
    if entry not in sys.path:
        sys.path.insert(0, entry)

RESERVED_DOMAINS = {87, 42, 7}


def generate_launch_text(scene_tf=None):
    """The whole-chain launch text (template + the I-proven patches)."""
    from frame_migration_st09_graph import LAUNCH_TEMPLATE, build_frame_params

    if scene_tf is None:
        from ament_index_python.packages import get_package_share_directory

        scene_tf = os.path.join(
            get_package_share_directory("luggage_description"),
            "config", "scene_tf.yaml.example")
    frame = build_frame_params(ROOT, scene_tf)
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
    from ament_index_python.packages import get_package_share_directory

    arm_controllers_yaml = os.path.join(
        get_package_share_directory("elfin_control"),
        "config", "elfin_controllers.yaml")
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
    # [ST09-I] the batched route validation service: the SAME robot
    # description/semantic move_group plans with, so certification
    # judges the identical model.
    old_anchor = "    # Controllers chain after the controller manager is up."
    new_anchor = """    nodes.append(Node(
        package="luggage_planning", executable="validate_route_node",
        name="validate_route_v2", output="screen",
        parameters=[description, semantic]))
    # Controllers chain after the controller manager is up."""
    assert old_anchor in launch_text, "template controllers anchor drifted"
    launch_text = launch_text.replace(old_anchor, new_anchor)
    return launch_text, scene_tf


def pick_domain():
    override = os.environ.get("ST09_I_DOMAIN_ID")
    if override:
        return int(override)
    for domain in range(55, 71):
        if domain in RESERVED_DOMAINS:
            continue
        env = dict(os.environ)
        env["ROS_DOMAIN_ID"] = str(domain)
        env["ROS_LOCALHOST_ONLY"] = "1"
        result = subprocess.run(
            ["timeout", "4", "ros2", "node", "list", "--no-daemon"],
            env=env, capture_output=True, text=True)
        if result.returncode == 0 and not result.stdout.strip():
            return domain
    return None


def wait_ready(out_dir, budget_s):
    """Wait for the seven endpoint peers to publish identity.

    The runtime graph only reports READY once the DRIVER peer joins
    (motion_occ_driver is a client of most endpoints), which happens in
    the freeze/control probes — so stack readiness here means: all
    seven endpoint peers have published identity at least once.
    """
    deadline = time.monotonic() + budget_s
    while time.monotonic() < deadline:
        time.sleep(3.0)
        try:
            result = subprocess.run(
                ["timeout", "12", "ros2", "topic", "echo",
                 "/frame_contracts/v2/runtime_identity", "--no-daemon"],
                capture_output=True, text=True, env=dict(os.environ))
        except OSError:
            result = None
        if result is None:
            continue
        peers = set(line.split(":", 1)[1].strip() for line in
                    (result.stdout or "").splitlines()
                    if line.startswith("peer:"))
        expected = {"placement_planner", "waypoint_generator",
                    "trajectory_selector", "motion_planner",
                    "vacuum_controller", "scene_manager",
                    "cargo_volume_mapper"}
        if expected <= peers:
            return True
    return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    parser.add_argument("--wait", type=float, default=180.0)
    args = parser.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    domain = pick_domain()
    if domain is None:
        print("no free domain in 55..70")
        return 1
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    (out_dir / "domain").write_text(str(domain))

    launch_text, _scene_tf = generate_launch_text()
    launch_path = out_dir / "graph_launch.py"
    launch_path.write_text(launch_text)
    log = open(out_dir / "stack.log", "w")
    proc = subprocess.Popen(
        ["ros2", "launch", str(launch_path)], stdout=log,
        stderr=subprocess.STDOUT, start_new_session=True)
    (out_dir / "launch.pid").write_text(str(proc.pid))
    try:
        ready = wait_ready(out_dir, args.wait)
    finally:
        log.close()
    print("domain=%d pid=%d ready=%s launch=%s" % (
        domain, proc.pid, ready, launch_path))
    return 0 if ready else 2


if __name__ == "__main__":
    sys.exit(main())
