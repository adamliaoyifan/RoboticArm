"""Score slots 639 and 656 with batched ValidateRouteV2 collision checks.

Run18 and the three-box layout are not scored here. Isolated MoveIt is
domain 42, plan-only, and this module stops it before returning.
"""

from __future__ import division

import json
import os
import signal
import subprocess
import sys
import threading
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
for _path in (
        ROOT,
        os.path.join(ROOT, "src", "luggage_planning"),
        os.path.join(ROOT, "src", "luggage_description"),
        os.path.join(ROOT, "src", "luggage_perception"),
):
    if _path not in sys.path:
        sys.path.insert(0, _path)

JOINTS = tuple("elfin_joint%d" % i for i in range(1, 7))
DOMAIN = 42


def _residuals():
    pattern = (
        "ros2 launch luggage_gazebo|ign gazebo|gz sim|"
        "isolated_moveit.launch|ros_gz_bridge|parameter_bridge|"
        "clock_bridge|camera_bridge|validate_route_node")
    try:
        text = subprocess.check_output(
            ["pgrep", "-af", pattern], text=True, stderr=subprocess.DEVNULL)
    except subprocess.CalledProcessError:
        return []
    rows = []
    for line in text.splitlines():
        if "pgrep" in line or "corridor_lattice" in line:
            continue
        rows.append(line.strip())
    return rows


def _start_moveit(env, scene_path, log_path, launch_args=None):
    command = [
        "ros2", "launch", "luggage_planning", "isolated_moveit.launch.py",
        "use_rviz:=false",
        "scene_tf_config:=" + scene_path,
    ]
    for key, value in sorted((launch_args or {}).items()):
        command.append("%s:=%s" % (key, value))
    log = open(log_path, "w", encoding="utf-8")
    proc = subprocess.Popen(
        command, env=env, stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True)
    proc._log = log  # noqa: SLF001
    return proc


def _stop_moveit(proc):
    if proc is None:
        return
    if proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except (ProcessLookupError, PermissionError, OSError):
            proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except (ProcessLookupError, PermissionError, OSError):
                proc.kill()
            proc.wait(timeout=8)
    log = getattr(proc, "_log", None)
    if log is not None and not log.closed:
        log.close()


def _spin(node):
    import rclpy
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    stop = threading.Event()

    def loop():
        while not stop.is_set():
            executor.spin_once(timeout_sec=0.05)

    thread = threading.Thread(target=loop, daemon=True)
    thread.start()
    return stop, thread


def _sync_world(node, scene_config):
    from luggage_description.scene_mesh_utils import (
        container_collision_mesh_path,
        container_model_name,
        require_existing_mesh,
    )
    from luggage_description.scene_tf_config_utils import (
        origin_in_world,
        pedestal_collision_pose_in_world,
        pedestal_dimensions,
        pedestal_enabled,
    )
    from luggage_description.scene_tf_publisher import rpy_to_quaternion
    from luggage_planning.motion.planning_scene_client import (
        PlanningSceneClient,
        load_stl_mesh_msg,
    )

    client = PlanningSceneClient(node)
    if not client.wait_ready(timeout_sec=60.0):
        raise RuntimeError("apply_planning_scene unavailable")
    mesh_path = require_existing_mesh(
        container_collision_mesh_path(scene_config))
    mesh = load_stl_mesh_msg(mesh_path, max_faces=8000)
    obj_id = container_model_name(scene_config)
    xyz, rpy = origin_in_world(scene_config)
    client.remove_object(obj_id)
    ok, message = client.add_collision_mesh(
        obj_id, xyz, rpy_to_quaternion(rpy), mesh, frame_id="world")
    if not ok:
        raise RuntimeError("container mesh: %s" % message)
    client.remove_object("robot_pedestal")
    if pedestal_enabled(scene_config):
        center, ped_rpy = pedestal_collision_pose_in_world(scene_config)
        length, width, height = pedestal_dimensions(scene_config)
        ok, message = client.add_collision_box(
            "robot_pedestal", center, rpy_to_quaternion(ped_rpy),
            (length, width, height), frame_id="world")
        if not ok:
            raise RuntimeError("pedestal: %s" % message)
    return client, obj_id, len(mesh.triangles)


def _scenes(client, start_q, grasp, box_world, box_quat, box_wdh):
    from luggage_planning.motion.moveit_state_validity import FrozenSceneComposer
    from luggage_planning.motion.planning_scene_client import (
        build_attach_scene,
        build_collision_object,
    )
    from moveit_msgs.msg import CollisionObject

    composer = FrozenSceneComposer(client)
    live = composer.read_live()
    world_ids = [str(obj.id) for obj in live.world.collision_objects]
    base = composer._base_scene(live, start_q)  # noqa: SLF001
    carrying = composer._copy_scene(base)  # noqa: SLF001
    attach = build_attach_scene(
        "cert-payload",
        list(grasp["translation"]),
        list(grasp["quaternion_xyzw"]),
        list(box_wdh),
        "suction_contact_frame",
        touch_links=("suction_panel", "suction_contact_frame"))
    carrying.robot_state.attached_collision_objects = \
        attach.robot_state.attached_collision_objects
    carrying.robot_state.is_diff = False
    carrying.is_diff = False
    unloaded = composer._copy_scene(base)  # noqa: SLF001
    unloaded.world.collision_objects.append(build_collision_object(
        "placed-slot", list(box_world), list(box_quat), list(box_wdh),
        "world", CollisionObject.ADD))
    unloaded.robot_state.is_diff = False
    unloaded.is_diff = False
    return (carrying, unloaded), world_ids


def _leg(states, load_index, carrying):
    from trajectory_msgs.msg import JointTrajectoryPoint

    from luggage_msgs.msg import RouteLegV2

    leg = RouteLegV2()
    leg.edge_id = "lattice-%s-%d" % (
        "carry" if carrying else "retreat", len(states))
    leg.edge_class = "TRAVERSE" if carrying else "RETREAT"
    leg.carrying = bool(carrying)
    leg.load_state_index = int(load_index)
    leg.joint_names = list(JOINTS)
    leg.n_samples = len(states)
    leg.sample_times_ns = [float(i) * 1.0e8 for i in range(len(states))]
    flat = []
    for joints in states:
        flat.extend(float(v) for v in joints)
    leg.sample_positions_flat = flat
    point = JointTrajectoryPoint()
    point.positions = [float(v) for v in states[0]]
    leg.knots.joint_names = list(JOINTS)
    leg.knots.points = [point]
    return leg


def _call(client, scenes, legs):
    from luggage_msgs.srv import ValidateRouteV2

    request = ValidateRouteV2.Request()
    request.request_id = "corridor-lattice"
    request.route_digest = "corridor-lattice"
    request.scene_digest = "corridor-lattice"
    request.load_state_scenes = list(scenes)
    request.max_sample_dt_ns = 0.0
    request.max_sample_step_m = 0.0
    request.distance_stride = 10 ** 9
    request.budget_s = 60.0
    request.legs = legs
    event = threading.Event()
    future = client.call_async(request)
    future.add_done_callback(lambda _future: event.set())
    if not event.wait(90.0):
        return None
    return future.result()


def _classify(client, scenes, states, load_index, timing=None):
    """Collision verdict per state. One global distance query per call.

    ``timing`` (a dict) accumulates ``service_time_s`` from the
    service's own ``service_time_ns``.
    """
    verdicts = {}
    pending = [list(states)]
    calls = 0
    while pending:
        group = pending.pop()
        response = _call(
            client, scenes, [_leg(group, load_index, load_index == 0)])
        calls += 1
        if timing is not None and response is not None:
            timing["service_time_s"] = timing.get("service_time_s", 0.0) \
                + float(response.service_time_ns) / 1e9
        if response is None or int(response.result) != 0:
            detail = "" if response is None else response.detail
            return None, calls, "service:%s" % (detail or "timeout")
        leg = response.leg_results[0]
        if leg.valid:
            for joints in group:
                verdicts[joints] = True
            continue
        index = int(leg.first_invalid_sample)
        for joints in group[:index]:
            verdicts[joints] = True
        if index >= len(group):
            return None, calls, "leg:%s" % leg.reason
        verdicts[group[index]] = False
        if index + 1 < len(group):
            pending.append(group[index + 1:])
    return verdicts, calls, ""


def _mask(client, scenes, queries, timing=None):
    by_load = {"carrying": [], "unloaded": []}
    for joints, load in queries:
        by_load[load].append(joints)
    mask = {}
    calls = 0
    for load, index in (("carrying", 0), ("unloaded", 1)):
        states = by_load[load]
        if not states:
            continue
        verdicts, used, error = _classify(
            client, scenes, states, index, timing)
        calls += used
        if error:
            return None, calls, error
        for joints, clear in verdicts.items():
            mask[(joints, load)] = bool(clear)
    return mask, calls, ""


def _request_for(document, candidate, limits):
    from luggage_planning.planning_contract.attachment import compose_transform

    from luggage_description.scene_tf_config_utils import (
        load_scene_tf_config,
    )
    from luggage_planning.routing.corridor_lattice import CorridorRequest
    import corridor_lattice_gate as gate

    scene = load_scene_tf_config(gate.SCENE)
    anchors = gate._anchors_for(candidate, document, scene)
    slot = candidate["slot_pose"]
    transform = document["container_transform"]
    center, quat = compose_transform(
        tuple(float(v) for v in transform["translation"]),
        tuple(float(v) for v in transform["quaternion_xyzw"]),
        tuple(float(v) for v in slot["position"]),
        tuple(float(v) for v in slot["quaternion_xyzw"]))
    request = CorridorRequest(
        anchors=anchors,
        orientation=tuple(float(v) for v in candidate["targets"][0][1]),
        start_q=tuple(float(v) for v in document["start_joints"]),
        joint_limits=limits,
        candidate_id=str(candidate["slot_index"]))
    return request, center, quat


def _one_run(document, candidate, limits, refine, client, scenes):
    from luggage_planning.routing.corridor_lattice import (
        kinematic_queries,
        prepare_corridor,
        search_prepared,
    )
    from luggage_planning.routing.grid_search import ROUTE_TOTAL_WALL_S

    started = time.perf_counter()
    request, _center, _quat = _request_for(document, candidate, limits)
    prepared = prepare_corridor(request, refine)
    if prepared.reason:
        return search_prepared(
            prepared, lambda _q, _load: False,
            ROUTE_TOTAL_WALL_S, started), 0, ""
    queries = kinematic_queries(prepared)
    mask, calls, error = _mask(client, scenes, queries)
    if error:
        return None, calls, error

    def collision(joints, load):
        return bool(mask.get((joints, load), False))

    result = search_prepared(
        prepared, collision, ROUTE_TOTAL_WALL_S, started)
    return result, calls, ""


def _row(result, service_calls):
    if result is None:
        return None
    return {
        "ok": bool(result.ok),
        "reason": result.reason,
        "within_budget": bool(result.within_budget),
        "wall_time_s": round(result.wall_time_s, 3),
        "n_vertices": result.n_vertices,
        "n_edges": result.n_edges,
        "n_collision_calls": result.n_collision_calls,
        "service_calls": int(service_calls),
        "max_step_rad": round(result.max_step_rad, 6),
        "n_layers": len(result.layer_names),
        "edge_loads": list(result.edge_loads),
        "chain": [[round(float(v), 6) for v in joints]
                  for joints in result.chain],
    }


def _same_chain(left, right):
    return left.get("chain") == right.get("chain") and left.get("ok")


def score_slots(evidence_dir, request_path, scene_path):
    import rclpy
    from luggage_perception.eval.isolated_domain import isolated_replay_env
    from luggage_planning.motion.offline_ik import load_urdf, refine_to_pose

    residuals = _residuals()
    if residuals:
        path = os.path.join(evidence_dir, "summary.json")
        with open(path, "w", encoding="utf-8") as handle:
            json.dump({
                "outcome": "not_evaluated",
                "detail": "live sim or MoveIt already running",
                "residuals": residuals,
            }, handle, indent=2)
            handle.write("\n")
        print(json.dumps({"outcome": "not_evaluated", "residuals": residuals}))
        return 2

    env = isolated_replay_env(DOMAIN)
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    urdf = load_urdf(scene_tf_config=scene_path)
    tcp = urdf.path_to("suction_contact_frame")
    limits = urdf.joint_limits()

    def refine(position, quaternion, seed):
        return refine_to_pose(
            urdf, tcp, position, quaternion, seed,
            pos_tol=2e-3, rot_tol=2e-2)

    document = json.load(open(request_path, encoding="utf-8"))
    log_path = os.path.join(evidence_dir, "moveit.log")
    proc = _start_moveit(env, scene_path, log_path)
    rclpy.init()
    node = rclpy.create_node("corridor_lattice_score")
    stop, thread = _spin(node)
    summary = {"outcome": "incomplete", "slots": {}}
    try:
        from luggage_msgs.srv import ValidateRouteV2

        client = node.create_client(ValidateRouteV2, "/validate_route_v2")
        if not client.wait_for_service(timeout_sec=90.0):
            summary["detail"] = "validate_route_v2 did not come up"
            return 2
        from luggage_description.scene_tf_config_utils import (
            load_scene_tf_config,
        )
        scene_config = load_scene_tf_config(scene_path)
        _scene_client, container_id, triangles = _sync_world(
            node, scene_config)
        grasp = document["attachment_transform"]
        summary["container_id"] = container_id
        summary["mesh_triangles"] = triangles
        failed = False
        for index in (639, 656):
            candidate = next(
                row for row in document["candidates"]
                if int(row["slot_index"]) == index)
            _request, center, quat = _request_for(
                document, candidate, limits)
            scenes, world_ids = _scenes(
                _scene_client, document["start_joints"], grasp,
                center, quat, document["box_wdh"])
            if container_id not in world_ids:
                summary["detail"] = "container missing from planning scene"
                summary["world_ids"] = world_ids
                return 2
            runs = []
            chains = []
            for _repeat in range(2):
                result, calls, error = _one_run(
                    document, candidate, limits, refine, client, scenes)
                if error or result is None:
                    summary["slots"][str(index)] = {
                        "outcome": "incomplete",
                        "detail": error,
                    }
                    failed = True
                    break
                row = _row(result, calls)
                runs.append(row)
                chains.append(result.chain)
            else:
                deterministic = chains[0] == chains[1]
                passed = (
                    runs[0]["ok"] and runs[1]["ok"]
                    and deterministic
                    and runs[0]["within_budget"]
                    and runs[1]["within_budget"]
                    and runs[0]["n_collision_calls"] > 0
                    and "carrying" in runs[0]["edge_loads"]
                    and "unloaded" in runs[0]["edge_loads"])
                summary["slots"][str(index)] = {
                    "outcome": "pass" if passed else "fail",
                    "deterministic": deterministic,
                    "runs": runs,
                }
                if not passed:
                    failed = True
            if failed and str(index) not in summary["slots"]:
                break
        summary["outcome"] = "fail" if failed else "pass"
        if any(
                row.get("outcome") == "incomplete"
                for row in summary["slots"].values()):
            summary["outcome"] = "incomplete"
        return 0 if summary["outcome"] == "pass" else 1
    finally:
        path = os.path.join(evidence_dir, "summary.json")
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2)
            handle.write("\n")
        print(json.dumps({
            "outcome": summary.get("outcome"),
            "slots": {
                key: {
                    "outcome": value.get("outcome"),
                    "reason": (value.get("runs") or [{}])[0].get("reason"),
                    "wall_time_s": (value.get("runs") or [{}])[0].get(
                        "wall_time_s"),
                    "deterministic": value.get("deterministic"),
                } for key, value in summary.get("slots", {}).items()
            },
            "path": path,
        }, indent=2))
        stop.set()
        thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        _stop_moveit(proc)
