#!/usr/bin/env python3
"""ST09 F21/F01 runtime graph campaign: production nodes on mock controllers.

No Gazebo, no hardware. Composes the production v2 chain (scene TF, the
seven endpoint nodes, MoveIt on the with-camera description with mock
controllers) plus a driver-peer probe that adopts the measured identity
from the wire. Per profile: three healthy startups within the 180 s budget
(each exercising one no-effect v2 call), the fault set once per fault, and
teardown within 90 s. Evidence: graph snapshot, per-peer readiness codes
from the wire, the probe's gate transitions, exercised-call results and
each fault's refusal with zero effects.
"""
from __future__ import division

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

STARTUP_BUDGET_S = 180.0
TEARDOWN_BUDGET_S = 90.0

LAUNCH_TEMPLATE = '''\
"""ST09 graph campaign launch (generated). Production v2 chain, mock controllers."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from luggage_description.xacro_robot_with_scene_base import expand_and_patch


def _srdf():
    path = os.path.join(
        get_package_share_directory("luggage_description"),
        "config", "S20_with_camera.srdf")
    with open(path, "r", encoding="utf-8") as handle:
        xml = handle.read()
    return xml.replace('name="elfin_s20_with_camera"', 'name="S20"', 1)


def _urdf():
    xacro_path = os.path.join(
        get_package_share_directory("luggage_description"),
        "urdf", "elfin_s20_with_camera.urdf.xacro")
    urdf, _pose = expand_and_patch(
        xacro_path, config_path=%(scene_tf)r,
        xacro_args=["hardware_plugin:=mock_components/GenericSystem",
                    "use_gz_sim:=false"])
    return urdf


def _nodes(context):
    urdf = _urdf()
    frame = %(frame_params)r
    scene_tf = %(scene_tf)r
    description = {"robot_description": ParameterValue(urdf, value_type=str)}
    semantic = {"robot_description_semantic": _srdf()}
    controllers = os.path.join(
        get_package_share_directory("elfin_control"),
        "config", "elfin_controllers.yaml")
    only = LaunchConfiguration("only").perform(context)
    wanted = [n.strip() for n in only.split(",") if n.strip()]

    def want(name):
        return not wanted or name in wanted

    nodes = []
    nodes.append(Node(
        package="luggage_description", executable="container_tf_publisher",
        name="container_tf_publisher", output="screen",
        parameters=[{"scene_tf_config": scene_tf, "republish_period": 0.0,
                     "frame_scene_revision": frame.get("frame_scene_revision", "")}]))
    nodes.append(Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="robot_state_publisher", output="screen",
        parameters=[description],
        condition=_cond(want("robot_state_publisher"))))
    nodes.append(Node(
        package="controller_manager", executable="ros2_control_node",
        name="controller_manager", output="screen",
        parameters=[description, controllers],
        remappings=[("~/robot_description", "/robot_description")],
        condition=_cond(want("controller_manager"))))
    nodes.append(Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager",
                   "/controller_manager", "--controller-manager-timeout", "60"],
        output="screen",
        condition=_cond(want("controllers"))))
    nodes.append(Node(
        package="moveit_ros_move_group", executable="move_group",
        name="move_group", output="screen",
        parameters=[description, semantic,
                    {"publish_robot_description_semantic": True,
                      "allow_trajectory_execution": True,
                      "use_sim_time": False}],
        condition=_cond(want("move_group"))))
    nodes.append(Node(
        package="luggage_planning", executable="scene_manager_node.py",
        name="scene_manager", output="screen",
        parameters=[{"scene_tf_config": scene_tf, "world_frame": "world",
                     "pickup_object_id": "pickup_box", "auto_sync": False,
                     "use_sim_time": False, **frame}]))
    nodes.append(Node(
        package="luggage_planning", executable="waypoint_generator_node.py",
        name="waypoint_generator", output="screen",
        parameters=[{"scene_tf_config": scene_tf, "world_frame": "world",
                     "place_slot_frame": "elfin_base_link",
                     "use_sim_time": False, **frame}]))
    nodes.append(Node(
        package="luggage_planning", executable="trajectory_selector_node.py",
        name="trajectory_selector", output="screen",
        parameters=[{"scene_tf_config": scene_tf,
                     "use_sim_time": False, **frame}]))
    nodes.append(Node(
        package="luggage_planning", executable="motion_planner_node.py",
        name="motion_planner", output="screen",
        parameters=[{"scene_tf_config": scene_tf, "use_sim_time": False,
                     **frame}]))
    nodes.append(Node(
        package="luggage_perception", executable="cargo_volume_mapper_node.py",
        name="cargo_volume_mapper", output="screen",
        parameters=[{"cargo_cloud_topic": "/luggage/semantic/cargo_points_untracked",
                     "base_frame": "container_link", "cloud_buffer_size": 12,
                     "freshness_window_sec": 0.20, "use_sim_time": False,
                     **frame}]))
    nodes.append(Node(
        package="luggage_packing", executable="placement_planner_node.py",
        name="placement_planner", output="screen",
        parameters=[{"scene_tf_config": scene_tf,
                     "use_sim_time": False, **frame}]))
    nodes.append(Node(
        package="luggage_planning", executable="vacuum_controller_node.py",
        name="vacuum_controller", output="screen",
        parameters=[{"use_sim_time": False, **frame}]))
    # Controllers chain after the controller manager is up.
    from launch.actions import RegisterEventHandler
    from launch.event_handlers import OnProcessExit
    spawners = []
    arm = Node(
        package="controller_manager", executable="spawner",
        arguments=["elfin_arm_controller", "--controller-manager",
                   "/controller_manager", "--controller-manager-timeout", "60"],
        output="screen",
        condition=_cond(want("controllers")))
    return [n for n in nodes if n is not None], arm


def _cond(enabled):
    from launch.conditions import IfCondition
    return IfCondition("true" if enabled else "false")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("only", default_value=""),
        OpaqueFunction(function=_nodes_opaque),
    ])


def _nodes_opaque(context):
    nodes, arm = _nodes(context)
    return nodes + [arm]
'''


def build_frame_params(ws, scene_tf):
    """Measured identity for the graph (same sources the launches use)."""
    sys.path.insert(0, str(Path(ws) / "src" / "luggage_description"))
    from luggage_planning.atlas.runtime import (
        load_atlas_runtime_for_scene, resolve_atlas_files)
    from luggage_description.runtime_graph import (
        endpoint_registry, identity_endpoints, resolve_peers)
    from luggage_description.runtime_identity import build_identity
    from luggage_description.xacro_robot_with_scene_base import expand_and_patch
    from ament_index_python.packages import get_package_share_directory
    peers = resolve_peers("motion_occ_driver")
    xacro_path = os.path.join(
        get_package_share_directory("luggage_description"),
        "urdf", "elfin_s20_with_camera.urdf.xacro")
    urdf, _pose = expand_and_patch(
        xacro_path, config_path=scene_tf,
        xacro_args=["hardware_plugin:=mock_components/GenericSystem",
                    "use_gz_sim:=false"])
    srdf_path = os.path.join(
        get_package_share_directory("luggage_description"),
        "config", "S20_with_camera.srdf")
    srdf = open(srdf_path, encoding="utf-8").read().replace(
        'name="elfin_s20_with_camera"', 'name="S20"', 1)
    identity = build_identity(
        scene_tf_config=scene_tf, calibration_revision="graph-campaign",
        urdf_text=urdf, srdf_text=srdf, tool_link="suction_contact_frame",
        atlas_files=resolve_atlas_files(
            load_atlas_runtime_for_scene(scene_tf, None)),
        endpoints=identity_endpoints(endpoint_registry(peers)))
    return {
        "frame_runtime": True,
        "frame_identity_json": json.dumps(identity, sort_keys=True),
        "frame_peers": ",".join(peers),
        "frame_scene_revision": identity["scene_revision"],
    }


class GraphProbe:
    """Driver peer: adopts identity, observes readiness, exercises v2."""

    def __init__(self, peer="motion_occ_driver"):
        import rclpy
        from luggage_msgs.msg import RuntimeIdentityV2
        from luggage_description.ros_frame_bindings.runtime import TOPIC
        from rclpy.qos import (DurabilityPolicy, QoSProfile,
                               ReliabilityPolicy)
        self.node = rclpy.create_node(peer)
        self.peer = peer
        self.peer_readiness = {}
        self._sub = self.node.create_subscription(
            RuntimeIdentityV2, TOPIC, self._on_message,
            QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE))
        self.runtime = None
        self._lock = threading.Lock()

    def _on_message(self, message):
        with self._lock:
            self.peer_readiness[str(message.peer)] = str(
                message.readiness_code or "")

    def adopt(self, timeout=STARTUP_BUDGET_S):
        from luggage_planning.typed_driver_adapter import adopt_runtime_identity
        self.runtime, _registry = adopt_runtime_identity(
            self.node, peer=self.peer, timeout_sec=timeout)
        from rclpy.executors import SingleThreadedExecutor
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin, daemon=True)
        self._spin_thread.start()
        return self.runtime

    def wait_ready(self, timeout=60.0):
        """Converged readiness: our gate AND every server's advertised code."""
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            last = self.runtime.readiness()
            view = self.readiness_view()
            peers_converged = bool(view) and all(
                code == "READY" for peer, code in view.items()
                if peer != self.peer)
            if last == "READY" and peers_converged:
                return True, last
            time.sleep(0.2)
        return False, last

    def close(self):
        """Stop the spin thread and shut the executor down cleanly."""
        self._executor.shutdown()
        self._spin_thread.join(timeout=5.0)
        self.node.destroy_node()

    def readiness_view(self):
        with self._lock:
            return dict(self.peer_readiness)

    def readiness(self):
        return self.runtime.readiness()

    def exercise_compute(self):
        """One no-effect v2 call: ComputePlacementV2 must answer.

        The expected map revision comes from the mapper's latched surface
        contract, exactly as the real drivers consume it — INCLUDING the
        catch-up wait: the latch can lag the last integrate's republish,
        and every CAS consumer must read the caught-up state or be
        refused STALE/REVISION_MISMATCH (the loop's _fresh_surface
        discipline; without it the probe flaked STALE_MAP_REVISION when
        the preceding test's commit was still in flight).
        """
        import threading

        from luggage_msgs.msg import SurfaceContractV2
        from luggage_msgs.srv import GetCargoMapStats
        from rclpy.qos import (DurabilityPolicy, QoSProfile,
                               ReliabilityPolicy)
        received = {}
        self.node.create_subscription(
            SurfaceContractV2, "/luggage/cargo_map/surface_v2",
            lambda message: received.__setitem__("message", message),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        deadline = time.monotonic() + 10.0
        while "message" not in received and time.monotonic() < deadline:
            time.sleep(0.1)
        surface = received.get("message")
        if surface is None:
            return {"ok": False, "code": "SURFACE_V2_MISSING"}
        stats_client = self.node.create_client(
            GetCargoMapStats, "/cargo_map/get_stats")
        if stats_client.wait_for_service(timeout_sec=5.0):
            event = threading.Event()
            future = stats_client.call_async(GetCargoMapStats.Request())
            future.add_done_callback(lambda _f: event.set())
            if event.wait(5.0) and future.done():
                stats = future.result()
                if stats is not None and stats.success:
                    deadline = time.monotonic() + 10.0
                    while (int(surface.map_revision) < int(
                                stats.map_revision)
                            and time.monotonic() < deadline):
                        time.sleep(0.1)
                        newer = received.get("message")
                        if newer is not None and int(
                                newer.map_revision) > int(
                                    surface.map_revision):
                            surface = newer
        from luggage_planning.typed_driver_adapter import TypedDriverAdapter
        from luggage_description.frame_contracts import ContractError
        adapter = TypedDriverAdapter(self.node, self.runtime)
        revision = self.runtime.identity["scene_revision"]
        try:
            response = adapter.compute_placement(
                request_id="graph-probe-1", box_wdh=[0.55, 0.40, 0.26],
                height_measured=True, scene_revision=revision,
                geometry_hash=str(surface.geometry_hash or ""),
                expected_map_revision=int(surface.map_revision),
                max_candidates=2)
        except ContractError as error:
            return {"ok": False, "code": error.code, "field": error.field}
        return {"ok": bool(response.success),
                "reason_code": str(response.reason_code or ""),
                "resulting_map_revision": int(response.resulting_map_revision)}


def run_scenario(name, command, out_dir, env, probe_factory, timeout,
                 launch_process=None, own_rclpy=True, stop_after_ready=False):
    """Launch a graph, run probe checks, teardown (unless given a process).

    The probe adopts BEFORE any executor claims its node; the runner then
    spins its own executor. Returns (record, probe, process).
    """
    import rclpy
    log_path = out_dir / ("%s.launch.log" % name)
    record = {"scenario": name, "command": command, "started_at": time.time()}
    if launch_process is None:
        launch_process = subprocess.Popen(
            command, env=env, stdout=open(log_path, "w"),
            stderr=subprocess.STDOUT, start_new_session=True)
    probe = None
    try:
        if own_rclpy:
            rclpy.init()
        probe = probe_factory()
        t0 = time.monotonic()
        try:
            probe.adopt(timeout=timeout)
            record["adopt_seconds"] = round(time.monotonic() - t0, 2)
        except Exception as error:  # noqa: BLE001 - evidence, not control flow
            record["adopt_error"] = repr(error)
        if probe.runtime is not None:
            ready, reason = probe.wait_ready(timeout=max(
                5.0, timeout - (time.monotonic() - t0)))
            record["ready"] = ready
            record["readiness_final"] = reason
            record["peer_readiness"] = probe.readiness_view()
            record["startup_seconds"] = round(time.monotonic() - t0, 2)
            if ready and not stop_after_ready:
                record["exercise"] = probe.exercise_compute()
                record["graph_nodes"] = str(graph_snapshot(out_dir, env, name))
    except Exception as error:  # noqa: BLE001 - record, never hide
        record["runner_error"] = repr(error)
    if stop_after_ready:
        return record, probe, launch_process
    try:
        _teardown(launch_process, record, log_path)
    finally:
        try:
            if probe is not None:
                probe.close()
        except Exception:  # noqa: BLE001
            pass
        if own_rclpy:
            try:
                rclpy.shutdown()
            except Exception:  # noqa: BLE001
                pass
    _write_json(out_dir / ("%s.json" % name), record)
    return record, probe, launch_process



def _teardown(proc, record, log_path):
    t0 = time.monotonic()
    if proc.poll() is None:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        try:
            proc.wait(timeout=TEARDOWN_BUDGET_S)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            try:
                proc.wait(timeout=15.0)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait(timeout=10.0)
    record["teardown_seconds"] = round(time.monotonic() - t0, 2)
    record["teardown_within_budget"] = (
        record["teardown_seconds"] <= TEARDOWN_BUDGET_S)
    record["launch_log"] = str(log_path)


def _write_json(path, payload):
    with open(path, "x", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=1, sort_keys=True, default=str,
                  allow_nan=False)
        stream.write("\n")


def graph_snapshot(out_dir, env, name):
    """ros2 node list snapshot for the F24 graph-side isolation."""
    result = subprocess.run(
        ["ros2", "node", "list"], env=env, capture_output=True, text=True,
        timeout=30)
    path = out_dir / ("%s.nodes.txt" % name)
    path.write_text(result.stdout, encoding="utf-8")
    return path


def run_fault(name, command, out_dir, env, timeout=STARTUP_BUDGET_S):
    """Healthy graph -> inject one fault -> observe refusal -> teardown."""
    import rclpy
    record = {"scenario": "fault-" + name}
    rclpy.init()
    holder = {}

    def factory():
        holder["probe"] = GraphProbe("motion_occ_driver")
        return holder["probe"]

    try:
        healthy, probe, launcher = run_scenario(
            "fault-%s-healthy" % name, command, out_dir, env,
            probe_factory=factory, timeout=timeout, own_rclpy=False,
            stop_after_ready=True)
        record["healthy"] = {k: healthy.get(k) for k in (
            "ready", "startup_seconds")}
        if probe is None or not healthy.get("ready"):
            record["fault_error"] = "graph never reached converged READY"
            return record
        resume = None
        if name == "lease_expiry":
            pid = find_node_process("waypoint_generator_node.py")
            record["target_pid"] = pid
            if pid:
                os.kill(pid, signal.SIGSTOP)
                resume = (pid, signal.SIGCONT)
        elif name == "restart":
            pid = find_node_process("trajectory_selector_node.py")
            record["target_pid"] = pid
            if pid:
                os.kill(pid, signal.SIGTERM)
        elif name == "v1_bypass":
            pid = find_node_process("waypoint_generator_node.py")
            record["target_pid"] = pid
            if pid:
                os.kill(pid, signal.SIGSTOP)
                resume = (pid, signal.SIGCONT)
        elif name in ("duplicate_provider", "wrong_overlay", "stale_atlas"):
            identity_path = out_dir / "identity.json"
            record["rogue"] = subprocess.call(
                [sys.executable, str(Path(__file__).resolve()),
                 "--rogue-mode", name, "--identity", str(identity_path),
                 "--ros-domain-id", str(env.get("ROS_DOMAIN_ID", "0"))],
                env=env, stdout=subprocess.DEVNULL)
        else:
            record["fault_error"] = "unknown fault %r" % name
            return record

        deadline = time.monotonic() + 20.0
        degraded = None
        while time.monotonic() < deadline:
            degraded = probe.readiness()
            if degraded != "READY":
                break
            time.sleep(0.2)
        record["probe_readiness_after_fault"] = degraded
        record["peer_readiness_after_fault"] = probe.readiness_view()
        snapshot = probe.runtime.snapshot()
        record["gate_transitions"] = snapshot["transitions"]
        record["counters"] = snapshot["counters"]

        if name == "v1_bypass":
            record["v1_call"] = _v1_compute_refusal(probe)
        else:
            from luggage_description.frame_contracts import ContractError
            from luggage_planning.typed_driver_adapter import TypedDriverAdapter
            try:
                response = TypedDriverAdapter(
                    probe.node, probe.runtime).compute_placement(
                    request_id="fault-%s" % name,
                    box_wdh=[0.55, 0.40, 0.26], height_measured=True,
                    scene_revision=probe.runtime.identity["scene_revision"],
                    geometry_hash=probe.runtime.identity["geometry_hash"],
                    expected_map_revision=1, max_candidates=1)
                record["v2_call"] = {
                    "ok": bool(response.success),
                    "reason_code": str(response.reason_code)}
            except ContractError as error:
                record["v2_call"] = {"ok": False, "code": error.code}
        if resume is not None:
            try:
                os.kill(resume[0], resume[1])
            except OSError:
                pass
        return record
    finally:
        launcher = locals().get("launcher")
        if launcher is not None:
            _teardown(launcher, record,
                      out_dir / ("fault-%s.launch.log" % name))
        try:
            if holder.get("probe") is not None:
                holder["probe"].close()
        except Exception:  # noqa: BLE001
            pass
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
        _write_json(out_dir / ("fault-%s.json" % name), record)


def _v1_compute_refusal(probe):
    """v1 ComputePlacement while the graph is not ready: refused, no effect."""
    from luggage_msgs.srv import ComputePlacement
    client = probe.node.create_client(
        ComputePlacement, "/placement_planner/compute_placement")
    if not client.wait_for_service(timeout_sec=5.0):
        return {"ok": False, "code": "V1_SERVICE_MISSING"}
    request = ComputePlacement.Request()
    request.box.width = 0.55
    request.box.depth = 0.40
    request.box.height = 0.26
    request.box.height_valid = True
    request.box.height_source = 1
    future = client.call_async(request)
    deadline = time.monotonic() + 15.0
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.1)
    if not future.done():
        return {"ok": False, "code": "V1_TIMEOUT"}
    response = future.result()
    return {"ok": bool(response.success),
            "reason_code": str(response.reason_code or ""),
            "message": str(response.message or "")[:120]}


def find_node_process(match):
    """One live pid whose command contains ``match`` (this user)."""
    result = subprocess.run(
        ["pgrep", "-f", match], capture_output=True, text=True)
    for pid in result.stdout.split():
        return int(pid)
    return None


def rogue_main(kind, identity_path, domain):
    """Publish malformed identity reports (fault injection, once per fault).

    duplicate_provider: a registry claiming an endpoint the graph assigns
    elsewhere. wrong_overlay: an artifact outside the install root.
    stale_atlas: a foreign atlas hash. Every receiver must reject.
    """
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    import rclpy
    from copy import deepcopy
    from luggage_description.ros_frame_bindings.runtime import (
        NodeRuntime, report_to_message, TOPIC)
    from luggage_msgs.msg import RuntimeIdentityV2
    from rclpy.qos import (DurabilityPolicy, QoSProfile,
                           ReliabilityPolicy)
    payload = json.loads(Path(identity_path).read_text())
    identity = payload["identity"]
    peers = [p for p in str(payload["frame_peers"]).split(",") if p]
    rclpy.init()
    node = rclpy.create_node("rogue_publisher")
    import luggage_planning.typed_driver_adapter as adapter_module
    runtime = NodeRuntime(
        node, peer="vacuum_controller", identity=deepcopy(identity),
        peers=peers, resolved_config={"rogue": kind},
        modules=[(adapter_module, "module", "application")])
    report = deepcopy(runtime.report)
    registry = deepcopy(runtime.registry)
    if kind == "duplicate_provider":
        registry["/placement_planner/compute_placement_v2"] = dict(
            registry["/placement_planner/compute_placement_v2"],
            provider=["vacuum_controller"])
    elif kind == "wrong_overlay":
        report["artifacts"] = [dict(item) for item in report["artifacts"]]
        report["artifacts"][0]["path"] = "/other/module.py"
    elif kind == "stale_atlas":
        report["identity"] = dict(report["identity"])
        report["identity"]["atlas_sha256"] = "f" * 64
        from luggage_description.runtime_profile import canonical_hash
        report["contract_digest"] = canonical_hash(report["identity"])
    else:
        raise SystemExit("unknown rogue kind %r" % kind)
    publisher = node.create_publisher(
        RuntimeIdentityV2, TOPIC,
        QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.VOLATILE))
    for index in range(8):
        publisher.publish(report_to_message(
            report, registry, sequence=index, readiness_code="READY"))
        time.sleep(0.3)
    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rogue-mode", default="",
                        choices=("", "duplicate_provider", "wrong_overlay",
                                 "stale_atlas"))
    parser.add_argument("--identity", default="")
    parser.add_argument("--ws", type=Path)
    parser.add_argument("--install", type=Path)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--ros-domain-id", type=int)
    parser.add_argument("--scene-tf", default="")
    parser.add_argument("--profile", choices=("sim", "hardware_plan_only"),
                        default="sim")
    parser.add_argument("--runs", default="healthy,healthy,healthy",
                        help="comma list: healthy or fault:<name>")
    args = parser.parse_args()

    if args.rogue_mode:
        rogue_main(args.rogue_mode, args.identity, args.ros_domain_id)
        return 0

    from ament_index_python.packages import get_package_share_directory
    scene_tf = args.scene_tf or os.path.join(
        get_package_share_directory("luggage_description"),
        "config", "scene_tf.yaml.example")
    args.out.mkdir(parents=True, exist_ok=False)
    frame = build_frame_params(args.ws, scene_tf)
    _write_json(args.out / "identity.json", {
        "frame_peers": frame["frame_peers"],
        "frame_scene_revision": frame["frame_scene_revision"],
        "identity": json.loads(frame["frame_identity_json"])})

    launch_path = args.out / "graph_launch.py"
    launch_path.write_text(LAUNCH_TEMPLATE % {
        "scene_tf": scene_tf, "frame_params": frame})
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    env["ROS_LOG_DIR"] = str(args.out / "roslog")
    (args.out / "roslog").mkdir(parents=True, exist_ok=True)
    command = ["ros2", "launch", str(launch_path)]

    summary = []
    for run in [r.strip() for r in args.runs.split(",") if r.strip()]:
        if run.startswith("fault:"):
            name = run.split(":", 1)[1]
            record = run_fault(name, command, args.out, env)
            entry = {"scenario": record["scenario"],
                     "ready": record.get("healthy", {}).get("ready"),
                     "degraded": record.get("probe_readiness_after_fault"),
                     "v2_call": record.get("v2_call"),
                     "v1_call": record.get("v1_call"),
                     "teardown_within_budget": record.get(
                         "teardown_within_budget")}
        else:
            record, _probe, _proc = run_scenario(
                run, command, args.out, env,
                probe_factory=(lambda: GraphProbe("motion_occ_driver")),
                timeout=STARTUP_BUDGET_S)
            entry = {"scenario": run, "ready": record.get("ready"),
                     "startup_seconds": record.get("startup_seconds"),
                     "teardown_within_budget": record.get(
                         "teardown_within_budget"),
                     "exercise": record.get("exercise")}
        summary.append(entry)
    _write_json(args.out / "campaign.json", {
        "profile": args.profile, "runs": summary})
    print(json.dumps({"profile": args.profile, "runs": summary}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
