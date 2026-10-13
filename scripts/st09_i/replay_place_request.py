#!/usr/bin/env python3
"""Replay one captured place request with atlas on and atlas off.

A fixture that lacks the request, start joints, attachment, map identity
or atlas identity exits ``not_replayable`` and names the missing fields.
It does not launch a simulation.
"""

from __future__ import division

import argparse
import json
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
GAZEBO_SRC = os.path.join(ROOT, "src", "luggage_gazebo")
if GAZEBO_SRC not in sys.path:
    sys.path.append(GAZEBO_SRC)

from luggage_gazebo.planning_request_capture import (  # noqa: E402
    audit_replay_bundle,
)
from luggage_planning.atlas_prior import (  # noqa: E402
    replay_captured_candidates,
)


def _search(case_id):
    sys.path.insert(0, os.path.join(ROOT, "scripts", "st09_i"))
    from route_matrix_eval import (  # noqa: WPS433
        SCENE,
        Scene,
        build_cases,
        load_retired_held_out,
        load_urdf,
        run_case,
    )
    from route_matrix_eval import _starts  # noqa: WPS433

    scene = Scene(SCENE)
    urdf = load_urdf(scene_tf_config=SCENE)
    tcp = urdf.path_to("suction_contact_frame")
    starts = _starts(urdf, tcp)
    retired = load_retired_held_out(os.path.join(
        ROOT, "docs/status/evidence/frame_migration/ST09-parallel-I-s5/"
        "route_matrix_20261004/split.json"))
    cases, occupancy = build_cases(scene, starts, retired)
    case = next(item for item in cases if item["id"] == case_id)
    ctx = {
        "scene": scene, "urdf": urdf, "tcp": tcp,
        "fk": None, "starts": starts, "occupancy": occupancy,
        "atlases": None,
    }
    from route_matrix_eval import Atlases, make_offline_fk  # noqa: WPS433
    ctx["fk"] = make_offline_fk(urdf, tcp)
    ctx["atlases"] = Atlases()
    rows = []
    for atlas_on in (False, True):
        row = run_case(case, ctx, "state", atlas_on)
        rows.append({
            "atlas": atlas_on,
            "success": bool(row.get("success")),
            "reason": row.get("reason"),
            "search_outcome": row.get("search_outcome"),
            "goal_nodes": row.get("goal_nodes"),
            "goal_seed_sources": row.get("goal_seed_sources"),
            "wall_s": row.get("wall_s"),
        })
    return rows


def _load_atlas_query(_document):
    """Production atlas query. Atlas-off never calls it."""
    from luggage_planning.reachability_atlas import ReachabilityAtlas

    directory = os.path.join(
        ROOT, "src", "luggage_planning", "data",
        "reachability_atlas_deploy")
    stem = "s20_container_collision_aware"
    atlas = ReachabilityAtlas.load(
        os.path.join(directory, stem + ".npz"),
        os.path.join(directory, stem + ".yaml"))
    return atlas.query


def _install_share(*parts):
    return os.path.join(ROOT, "install-st09", *parts)


def _default_scene():
    return _install_share(
        "luggage_description", "share", "luggage_description",
        "config", "scene_tf_deploy.yaml")


def _default_atlas_config():
    return _install_share(
        "luggage_planning", "share", "luggage_planning",
        "config", "atlas_runtime_deploy_sim.yaml")


def _sibling(fixture, *parts):
    return os.path.normpath(os.path.join(os.path.dirname(fixture), *parts))


def _read_bag_messages(bag_dir):
    """Captured surface_v2 and surface_2d. Does not join a live graph."""
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rosidl_runtime_py.utilities import get_message

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_dir, storage_id="sqlite3"),
        ConverterOptions(input_serialization_format="cdr",
                         output_serialization_format="cdr"))
    surface_type = get_message("luggage_msgs/msg/SurfaceContractV2")
    string_type = get_message("std_msgs/msg/String")
    found = {}
    while reader.has_next():
        topic, data, _stamp = reader.read_next()
        if topic == "/luggage/cargo_map/surface_v2" and "v2" not in found:
            found["v2"] = deserialize_message(data, surface_type)
        elif topic == "/luggage/cargo_map/surface_2d" and "v1" not in found:
            found["v1"] = json.loads(
                deserialize_message(data, string_type).data)
        if "v2" in found and "v1" in found:
            break
    return found


def _surface_record(message):
    columns = int(message.columns)
    count = int(message.rows) * columns

    def grid(values, blank_invalid=False):
        rows = []
        for start in range(0, count, columns):
            row = []
            for offset in range(columns):
                index = start + offset
                if blank_invalid and not message.valid[index]:
                    row.append(None)
                else:
                    row.append(values[index])
            rows.append(row)
        return rows

    return {
        "floor_z_container": float(message.floor_z_container),
        "resolution_m": float(message.resolution_m),
        "origin_xy_container": [
            float(v) for v in message.origin_xy_container],
        "surface_z_container": grid(message.surface_z_container, True),
        "state": grid(list(message.state)),
        "confidence": grid(list(message.confidence)),
        "geometry_hash": str(message.geometry_hash),
        "map_revision": int(message.map_revision),
    }


def _load_planner():
    import importlib.util

    path = os.path.join(
        ROOT, "src", "luggage_packing", "scripts",
        "placement_planner_node.py")
    spec = importlib.util.spec_from_file_location(
        "placement_planner_node", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _recover_slots(document, surface_v2, urdf_hash):
    """Rebuild the placement list and require every captured pose to match.

    The request file stores only rows the selector closed, so the untried
    reachable slots are absent. Re-solving is used only after that check.
    """
    import math

    module = _load_planner()
    node = module.PlacementPlannerNode()
    try:
        if str(surface_v2.geometry_hash) != node._geometry_hash:
            return None, {
                "outcome": "not_replayable",
                "missing": ["surface_geometry_hash"],
                "detail": "bag surface hash differs from the planner hull",
            }
        node._urdf_identity.hash = urdf_hash
        node._urdf_identity.ready = True
        node._surface = node._surface_view_from_record(
            _surface_record(surface_v2))
        size = document["box_wdh"]
        _atlas, diag = node._atlas_for(size)
        if not (diag or {}).get("loaded"):
            return None, {
                "outcome": "not_replayable",
                "missing": ["atlas"],
                "detail": diag,
            }
        result = module.solve_placement(
            node._active_surface(), size,
            allowed_yaws=node._allowed_yaws,
            params=node._params,
            candidate_validator=lambda candidate: node._constraint_verdict(
                candidate, [], size),
            atlas_annotate=node._atlas_hook(size))
        ranked = module.ranked_feasible_candidates(result, 2000)
        mismatches = []
        for item in document["candidates"]:
            index = int(item["slot_index"])
            if index >= len(ranked):
                mismatches.append({"slot_index": index, "error": "past_end"})
                continue
            got = ranked[index]["center_container"]
            want = item["slot_pose"]["position"]
            err = math.sqrt(sum(
                (float(a) - float(b)) ** 2 for a, b in zip(got, want)))
            yaw = float(ranked[index]["box_yaw"])
            quat = item["slot_pose"]["quaternion_xyzw"]
            qerr = (
                abs(math.sin(yaw * 0.5) - float(quat[2]))
                + abs(math.cos(yaw * 0.5) - float(quat[3])))
            if err > 1e-6 or qerr > 1e-6:
                mismatches.append({
                    "slot_index": index,
                    "position_err_m": err,
                    "quaternion_err": qerr,
                })
        if mismatches:
            return None, {
                "outcome": "not_replayable",
                "missing": ["slots"],
                "detail": "reconstructed poses diverge from the capture",
                "n_mismatch": len(mismatches),
                "mismatches": mismatches[:8],
            }
        slots, scores = [], []
        for candidate in ranked:
            yaw = float(candidate["box_yaw"])
            slots.append({
                "position": [
                    float(v) for v in candidate["center_container"]],
                "quaternion_xyzw": [
                    0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)],
                "request_id": document["request_id"],
            })
            scores.append(float(candidate.get("score") or 0.0))
        return (slots, scores, {
            "n_ranked": len(ranked),
            "n_captured": len(document["candidates"]),
            "pose_mismatches": 0,
            "atlas_hash_message": (diag or {}).get("hash_message"),
        }), None
    finally:
        node.destroy_node()


def _contract_surface(message, document):
    record = _surface_record(message)
    return {
        "state": record["state"],
        "resolution_m": record["resolution_m"],
        "origin_xy_container": record["origin_xy_container"],
        "frame_id": "container_link",
        "stamp_ns": 0,
        "map_source": str(document.get("map_source") or ""),
        "placed_count": 0,
        "cargo_observed": False,
        "commit_revision": int(document["map_revision"]),
    }


def _route_request(document, slots, scores, surface, inner_size,
                   scene_revision, stamp_ns):
    grasp = document["attachment_transform"]
    return {
        "request_id": document["request_id"],
        "scene_revision": scene_revision,
        "box_wdh": [float(v) for v in document["box_wdh"]],
        "slots": slots,
        "slot_scores": scores,
        "surface": surface,
        "inner_size": [float(v) for v in inner_size],
        "geometry_hash": document["geometry_hash"],
        "map_revision": int(document["map_revision"]),
        "start_joints": [float(v) for v in document["start_joints"]],
        "grasp": {
            "position": list(grasp["translation"]),
            "quaternion_xyzw": list(grasp["quaternion_xyzw"]),
            "stamp_ns": int(stamp_ns),
        },
        "container_transform": document["container_transform"],
        "excluded_ids": [],
        "box_id": "",
        "generation": 0,
    }


def _reference_field(reference_path, name):
    if not reference_path or not os.path.isfile(reference_path):
        return ""
    with open(reference_path, encoding="utf-8") as handle:
        document = json.load(handle)
    for row in document.get("rows") or []:
        if row.get(name):
            return str(row[name])
    return ""


def _live_residuals():
    import subprocess

    pattern = (
        "ros2 launch luggage_gazebo|ign gazebo|gz sim|"
        "isolated_moveit.launch|ros_gz_bridge|parameter_bridge|"
        "clock_bridge|camera_bridge")
    try:
        text = subprocess.check_output(
            ["pgrep", "-af", pattern], text=True,
            stderr=subprocess.DEVNULL)
    except subprocess.CalledProcessError:
        return []
    rows = []
    for line in text.splitlines():
        if "pgrep" in line or "replay_place_request" in line:
            continue
        rows.append(line.strip())
    return rows


def _start_moveit(env, scene_path, log_path):
    import subprocess

    command = [
        "ros2", "launch", "luggage_planning", "isolated_moveit.launch.py",
        "use_rviz:=false",
        "scene_tf_config:=" + scene_path,
    ]
    log = open(log_path, "w", encoding="utf-8")
    proc = subprocess.Popen(
        command, env=env, stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True)
    proc._log = log  # noqa: SLF001 - closed in _stop_moveit
    return proc


def _stop_moveit(proc):
    import signal
    import subprocess

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


def _wait_moveit(node, timeout_sec):
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.srv import GetCartesianPath
    from rclpy.action import ActionClient

    from luggage_msgs.srv import ValidateRouteV2

    cartesian = node.create_client(
        GetCartesianPath, "/compute_cartesian_path")
    action = ActionClient(node, MoveGroup, "/move_action")
    validator = node.create_client(ValidateRouteV2, "/validate_route_v2")
    if not action.wait_for_server(timeout_sec=timeout_sec):
        return "move_action"
    if not cartesian.wait_for_service(timeout_sec=10.0):
        return "compute_cartesian_path"
    if not validator.wait_for_service(timeout_sec=10.0):
        return "validate_route_v2"
    return ""


def _serve_captured_stats(node, document):
    from luggage_msgs.srv import GetCargoMapStats

    def handle(_request, response):
        response.success = True
        response.map_revision = int(document["map_revision"])
        response.geometry_hash = str(document["geometry_hash"])
        response.map_source = str(document.get("map_source") or "")
        response.placed_count = 0
        response.cargo_observed = False
        response.last_acquisition_stamp_ns = 0
        return response

    return node.create_service(
        GetCargoMapStats, "/cargo_map/get_stats", handle)


def _compact_row(row, rank):
    import math

    pose = row.get("slot_pose") or {}
    quat = pose.get("quaternion_xyzw") or [0.0, 0.0, 0.0, 1.0]
    yaw = math.atan2(
        2.0 * (float(quat[3]) * float(quat[2])
               + float(quat[0]) * float(quat[1])),
        1.0 - 2.0 * (float(quat[1]) ** 2 + float(quat[2]) ** 2))
    detail = row.get("detail") if isinstance(row.get("detail"), dict) else {}
    coverage = detail.get("coverage") or {}
    budget = coverage.get("budget") or {}
    return {
        "rank": rank,
        "slot_index": row.get("slot_index"),
        "tier": row.get("reachability"),
        "reason": row.get("reason"),
        "route_build_count": row.get("route_build_count"),
        "depth_m": row.get("depth_m"),
        "placement_score": row.get("placement_score"),
        "position": pose.get("position"),
        "yaw": yaw,
        "seed_sources": row.get("seed_sources"),
        "seed_count": row.get("seed_count"),
        "elapsed_ms": row.get("elapsed_ms"),
        "cartesian": row.get("cartesian"),
        "retreat_target_missing": bool(detail.get("retreat_target_missing")),
        "frontier": detail.get("frontier"),
        "edge_attempts": sorted(budget.get("attempts") or {}),
        "edge_durations_s": budget.get("durations_s") or {},
        "edges_built": coverage.get("edges_built"),
        "start_joints": row.get("start_joints"),
        "atlas_seed_q": row.get("atlas_seed_q"),
        "first_edge": row.get("first_edge"),
        "planner_parameters": row.get("planner_parameters"),
        "scene": row.get("scene"),
        "anchors": row.get("anchors"),
        "ik_refine": row.get("ik_refine"),
        "atlas_queries": row.get("atlas_queries"),
        "targets": row.get("targets"),
        "n_ordered": row.get("n_ordered"),
        "n_seeded": row.get("n_seeded"),
        "n_tried": row.get("n_tried"),
        "n_untried": row.get("n_untried"),
    }


def _built_rows(rows):
    return [
        row for row in rows or []
        if int(row.get("route_build_count") or 0) > 0]


def _q_delta(left, right):
    if not left or not right or len(left) != len(right):
        return None
    return max(abs(float(a) - float(b)) for a, b in zip(left, right))


def _layer_missing(rows, names):
    missing = []
    built = _built_rows(rows)
    if not built:
        return ["built_rows"]
    sample = built[0]
    for name in names:
        if name == "refine_q":
            attempts = []
            for record in sample.get("ik_refine") or []:
                attempts.extend(record.get("attempts") or [])
            if not attempts or all(item.get("q") is None for item in attempts):
                missing.append(name)
        elif not sample.get(name):
            missing.append(name)
    return missing


def _q_close(left, right, tol=1e-3):
    delta = _q_delta(left, right)
    return delta is not None and delta <= tol


def _fidelity(result_rows, reason_code, reference_path):
    if not reference_path or not os.path.isfile(reference_path):
        return {
            "outcome": "not_replayable",
            "missing": ["reference_candidates"],
        }
    with open(reference_path, encoding="utf-8") as handle:
        reference = json.load(handle)
    ref_built = _built_rows(reference.get("rows"))
    got_built = _built_rows(result_rows)
    ref_ids = [int(row["slot_index"]) for row in ref_built]
    got_ids = [int(row["slot_index"]) for row in got_built]
    ref_reasons = [str(row.get("reason")) for row in ref_built]
    got_reasons = [str(row.get("reason")) for row in got_built]
    shared = min(len(ref_ids), len(got_ids))
    prefix_ok = (
        ref_ids[:shared] == got_ids[:shared]
        and ref_reasons[:shared] == got_reasons[:shared])
    exact = ref_ids == got_ids and ref_reasons == got_reasons
    cutoff = abs(len(ref_ids) - len(got_ids)) <= 1 and prefix_ok and shared > 0
    q_mismatch = []
    for ref, got in zip(ref_built, got_built):
        if int(ref["slot_index"]) != int(got["slot_index"]):
            break
        left = (ref.get("cartesian") or [None])[0] or {}
        right = (got.get("cartesian") or [None])[0] or {}
        if not left.get("from_q") or not right.get("from_q"):
            q_mismatch.append({
                "slot_index": int(ref["slot_index"]),
                "missing_q": True,
            })
            continue
        deltas = {
            "from_q": _q_delta(left.get("from_q"), right.get("from_q")),
            "to_q": _q_delta(left.get("to_q"), right.get("to_q")),
        }
        same_edge = (
            left.get("edge_class") == right.get("edge_class")
            and left.get("failed_waypoint_index")
            == right.get("failed_waypoint_index")
            and deltas["from_q"] is not None and deltas["from_q"] <= 1e-3
            and deltas["to_q"] is not None and deltas["to_q"] <= 1e-3)
        if not same_edge:
            q_mismatch.append({
                "slot_index": int(ref["slot_index"]),
                "deltas": deltas,
                "ref_edge": left.get("edge_class"),
                "got_edge": right.get("edge_class"),
                "ref_waypoint": left.get("failed_waypoint_index"),
                "got_waypoint": right.get("failed_waypoint_index"),
            })
    ref_summary = next((
        row for row in reference.get("rows") or []
        if row.get("reason") == "ATTEMPT_SUMMARY"), {})
    got_summary = next((
        row for row in result_rows
        if row.get("reason") == "ATTEMPT_SUMMARY"), {})
    ref_unknown = sum(
        1 for row in reference.get("rows") or []
        if row.get("reason") == "ATLAS_UNKNOWN_STRICT")
    got_unknown = sum(
        1 for row in result_rows
        if row.get("reason") == "ATLAS_UNKNOWN_STRICT")
    same_counts = (
        int(got_summary.get("n_ordered") or -1)
        == int(ref_summary.get("n_ordered") or -2)
        and int(got_summary.get("n_seeded") or -1)
        == int(ref_summary.get("n_seeded") or -2)
        and got_unknown == ref_unknown)
    geometry = "pass" if same_counts else "fail"
    topology_missing = _layer_missing(
        reference.get("rows"), ("anchors", "first_edge", "refine_q"))
    if topology_missing:
        topology = "not_captured"
    else:
        ref0, got0 = ref_built[0], got_built[0]
        def _first_portal(row):
            for anchor in row.get("anchors") or []:
                if anchor.get("region") == "portal":
                    return anchor.get("q")
            return None

        ref_anchor = _first_portal(ref0)
        got_anchor = _first_portal(got0)
        ref_edge = ref0.get("first_edge") or {}
        got_edge = got0.get("first_edge") or {}
        same_anchor = _q_close(ref_anchor, got_anchor)
        same_edge = (
            ref_edge.get("edge_class") == got_edge.get("edge_class")
            and _q_close(ref_edge.get("from_q"), got_edge.get("from_q"))
            and _q_close(ref_edge.get("to_q"), got_edge.get("to_q"))
            and ref_edge.get("fraction") == got_edge.get("fraction"))
        topology = "pass" if same_anchor and same_edge else "fail"
    if geometry == "pass" and topology == "pass":
        budget = (
            "pass" if reason_code == "PLANNING_BUDGET_EXHAUSTED"
            and exact else "fail")
        outcome = "fidelity_pass" if budget == "pass" else "replay_divergent"
    else:
        budget = "not_evaluated"
        outcome = "replay_divergent"
    return {
        "outcome": outcome,
        "geometry": geometry,
        "topology": topology,
        "topology_missing": topology_missing,
        "budget": budget,
        "timing_cutoff": bool(cutoff and not exact),
        "reason_code": reason_code,
        "ref_slots": ref_ids,
        "got_slots": got_ids,
        "ref_reasons": ref_reasons,
        "got_reasons": got_reasons,
        "legacy_cartesian_q_mismatch": q_mismatch,
        "capture_n_ordered": ref_summary.get("n_ordered"),
        "capture_n_seeded": ref_summary.get("n_seeded"),
        "capture_unknown": ref_unknown,
        "replay_n_ordered": got_summary.get("n_ordered"),
        "replay_n_seeded": got_summary.get("n_seeded"),
        "replay_unknown": got_unknown,
        "same_counts": same_counts,
    }


def _write_json(path, payload):
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=1, sort_keys=True, default=str)
        handle.write("\n")


def _route(args, document):
    import hashlib
    import subprocess
    import threading
    import time

    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    from luggage_description.runtime_identity import scene_revision_from_path
    from luggage_description.scene_tf_config_utils import (
        container_usable_dimensions,
        load_scene_tf_config,
    )
    from luggage_perception.eval.isolated_domain import (
        IsolatedDomainError,
        isolated_replay_env,
    )
    from luggage_planning.route_producer import register_attach_from_request

    bag = args.bag or _sibling(args.fixture, "..", "..", "bag")
    reference = args.reference or _sibling(args.fixture, "candidates.json")
    scene_path = args.scene or _default_scene()
    atlas_config = args.atlas_config or _default_atlas_config()
    missing = []
    if not os.path.isdir(bag):
        missing.append("bag")
    urdf_hash = _reference_field(reference, "urdf_hash")
    scene_hash = _reference_field(reference, "scene_tf_hash")
    if not urdf_hash:
        missing.append("urdf_hash")
    if not scene_hash:
        missing.append("scene_tf_hash")
    if not os.path.isfile(scene_path):
        missing.append("scene_tf_deploy")
    if not os.path.isfile(atlas_config):
        missing.append("atlas_runtime_config")
    if not args.out:
        missing.append("out")
    if missing:
        return {"outcome": "not_replayable", "missing": missing, "bag": bag}, 2
    if scene_revision_from_path(scene_path) != scene_hash:
        return {
            "outcome": "not_replayable",
            "missing": ["scene_revision"],
            "detail": "scene file hash differs from the capture",
            "scene_path": scene_path,
            "capture": scene_hash,
            "file": scene_revision_from_path(scene_path),
        }, 2
    residuals = _live_residuals()
    if residuals:
        return {
            "outcome": "not_replayable",
            "missing": ["clean_machine"],
            "residuals": residuals,
        }, 2
    try:
        env = isolated_replay_env(args.domain)
    except IsolatedDomainError as exc:
        return {"outcome": "not_replayable", "missing": [str(exc)]}, 2
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.makedirs(args.out, exist_ok=False)
    rclpy.init(args=[
        "--ros-args",
        "-p", "scene_tf_config:=" + scene_path,
        "-p", "atlas_runtime_config:=" + atlas_config,
        "-p", "frame_scene_revision:=" + scene_hash,
        "-p", "atlas_enabled:=true",
        "-p", "route_planning:=true",
    ])
    moveit = None
    selector = None
    executor = None
    try:
        bag_msgs = _read_bag_messages(bag)
        absent_bag = [
            name for name, key in (
                ("surface_v2", "v2"), ("surface_2d", "v1"))
            if key not in bag_msgs]
        if absent_bag:
            report = {"outcome": "not_replayable", "missing": absent_bag}
            _write_json(os.path.join(args.out, "summary.json"), report)
            return report, 2
        recovered, error = _recover_slots(
            document, bag_msgs["v2"], urdf_hash)
        if error:
            _write_json(os.path.join(args.out, "summary.json"), error)
            return error, 2
        slots, scores, recover_info = recovered
        moveit = _start_moveit(
            env, scene_path, os.path.join(args.out, "moveit.log"))
        sys.path.insert(0, os.path.join(
            ROOT, "src", "luggage_planning", "scripts"))
        from trajectory_selector_node import TrajectorySelectorNode

        selector = TrajectorySelectorNode()
        selector._urdf_identity.hash = urdf_hash
        selector._urdf_identity.ready = True
        selector._atlas_key = None
        selector._surface = bag_msgs["v1"]
        _serve_captured_stats(selector, document)
        absent = _wait_moveit(selector, 90.0)
        if absent:
            report = {
                "outcome": "not_replayable",
                "missing": [absent],
                "detail": "isolated_moveit service did not come up",
            }
            _write_json(os.path.join(args.out, "summary.json"), report)
            return report, 2
        _atlas, diag = selector._runtime_atlas()
        if not (diag or {}).get("loaded"):
            report = {
                "outcome": "not_replayable",
                "missing": ["selector_atlas"],
                "detail": diag,
            }
            _write_json(os.path.join(args.out, "summary.json"), report)
            return report, 2
        executor = MultiThreadedExecutor()
        executor.add_node(selector)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        producer = selector._route_producer_or_none()
        if producer is None:
            report = {
                "outcome": "not_replayable",
                "missing": ["route_producer"],
            }
            _write_json(os.path.join(args.out, "summary.json"), report)
            return report, 2
        producer.budget_ns = int(float(args.budget_s) * 1e9)
        stamp_ns = int(selector.get_clock().now().nanoseconds)
        inner = list(container_usable_dimensions(
            load_scene_tf_config(scene_path)))
        request = _route_request(
            document, slots, scores,
            _contract_surface(bag_msgs["v2"], document),
            inner, scene_hash, stamp_ns)
        register_attach_from_request(
            producer.attachment_provider, request,
            box_wdh=request["box_wdh"])
        started = time.time()
        result = producer.plan_place_route(request)
        elapsed = time.time() - started
        rows = list(result.get("rows") or [])
        fidelity = _fidelity(
            rows, str(result.get("reason_code") or ""), reference)
        head = subprocess.check_output(
            ["git", "-C", ROOT, "rev-parse", "HEAD"], text=True).strip()
        dirty = subprocess.check_output(
            ["git", "-C", ROOT, "status", "--porcelain"], text=True)
        manifest = {
            "commit": head,
            "dirty_count": len([
                line for line in dirty.splitlines() if line.strip()]),
            "inputs_sha256": hashlib.sha256(
                open(args.fixture, "rb").read()).hexdigest(),
            "scene_path": scene_path,
            "scene_revision": scene_hash,
            "geometry_hash": document["geometry_hash"],
            "domain": int(env["ROS_DOMAIN_ID"]),
            "budget_s": float(args.budget_s),
            "capture_complete": True,
            "replay_possible": True,
            "recovered": {
                "slots": "placement solver plus bag surface_v2, pose-checked",
                "surface_v2": "bag",
                "surface_2d": "bag",
                "grasp_stamp_ns": (
                    "replay clock; the capture has no stamp and 0 is "
                    "STAMP_INVALID"),
                "urdf_hash": urdf_hash,
                "placed_count": 0,
            },
            "recover_info": recover_info,
            "atlas_diag": diag,
            "elapsed_s": round(elapsed, 3),
        }
        _write_json(os.path.join(args.out, "manifest.json"), manifest)
        with open(os.path.join(args.out, "census.jsonl"), "w",
                  encoding="utf-8") as handle:
            for index, row in enumerate(rows):
                handle.write(json.dumps(
                    _compact_row(row, index), sort_keys=True, default=str))
                handle.write("\n")
        summary = {
            "outcome": fidelity["outcome"],
            "fidelity": fidelity,
            "reason_code": result.get("reason_code"),
            "success": bool(result.get("success")),
            "slot_index": result.get("slot_index"),
            "elapsed_s": round(elapsed, 3),
            "n_rows": len(rows),
            "n_ranked_slots": len(slots),
        }
        _write_json(os.path.join(args.out, "summary.json"), summary)
        return summary, 0 if fidelity["outcome"] == "fidelity_pass" else 1
    finally:
        if executor is not None:
            executor.shutdown()
        if selector is not None:
            selector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        _stop_moveit(moveit)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--fixture", required=True)
    parser.add_argument("--search", action="store_true",
                        help="also run the offline state search, atlas off then on")
    parser.add_argument(
        "--route", action="store_true",
        help="replay the captured request through the production RouteProducer")
    parser.add_argument("--budget-s", type=float, default=10.0)
    parser.add_argument("--domain", default="42")
    parser.add_argument("--bag", default="")
    parser.add_argument("--reference", default="")
    parser.add_argument("--scene", default="")
    parser.add_argument("--atlas-config", default="")
    parser.add_argument("--out", default="")
    args = parser.parse_args()
    with open(args.fixture, encoding="utf-8") as handle:
        document = json.load(handle)
    audit = audit_replay_bundle(document)
    report = {"audit": audit, "fixture": args.fixture}
    if not audit["replay_possible"]:
        print(json.dumps(report, indent=1))
        return 2
    if args.route:
        summary, code = _route(args, document)
        print(json.dumps(summary, indent=1, default=str))
        return code
    if args.search:
        if document.get("candidates"):
            report["atlas"] = replay_captured_candidates(
                document, _load_atlas_query(document))
        else:
            case_id = document.get("matrix_case_id")
            if not case_id:
                report["search"] = {
                    "status": "not_replayable",
                    "missing": ["matrix_case_id"],
                }
                print(json.dumps(report, indent=1))
                return 2
            report["search"] = _search(case_id)
    print(json.dumps(report, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
