#!/usr/bin/env python3
"""Corrected tool-down 48-point baseline, then collision of its chains.

The synthetic grasp is Rx(pi) with translation ``height/2``. Old scans
that used the identity attachment stay in their original directories and
are listed here as superseded diagnostics. This script does not rewrite
them.
"""

from __future__ import division

import os

for _var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS",
             "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(_var, "1")

import argparse
import json
import sys
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
HERE = os.path.dirname(os.path.abspath(__file__))
for _path in (
        HERE,
        os.path.join(ROOT, "src", "luggage_packing"),
        os.path.join(ROOT, "src", "luggage_packing", "scripts"),
        os.path.join(ROOT, "src", "luggage_planning"),
        os.path.join(ROOT, "src", "luggage_description"),
        os.path.join(ROOT, "src", "luggage_perception"),
        os.path.join(ROOT, "st09_contracts"),
):
    if _path not in sys.path:
        sys.path.insert(0, _path)

import corridor_ablation as ablation  # noqa: E402
import placement_robustness as scan  # noqa: E402
from luggage_planning.routing.corridor_lattice import (  # noqa: E402
    CorridorRequest,
    has_kinematic_chain,
    prepare_corridor,
)
from st09_contracts.attachment import rotate_vector  # noqa: E402

OUT = os.path.join(
    ROOT, "docs/status/evidence/frame_migration/ST09-parallel-I-s5/m3",
    "g3_n3_tooldown_baseline_20261006")
SCAN = os.path.join(
    ROOT, "docs/status/evidence/frame_migration/ST09-parallel-I-s5/m3",
    "g3_n3_baseline_freeze_20261005", "scan_list.json")
SUPERSEDED = (
    "g3_placement_robustness_20261005",
    "g3_corridor_ablation_20261005",
    "g3_n3_failure_attribute_20261005",
)
ATTACHMENT_SOURCE = "synthetic_rx_pi_half_height"


def _prefer_source_contracts():
    """The eval interpreter may already have imported the installed contract."""
    import importlib
    source = os.path.join(ROOT, "st09_contracts")
    sys.path.insert(0, source)
    for key in list(sys.modules):
        if key == "st09_contracts" or key.startswith("st09_contracts."):
            del sys.modules[key]
    importlib.invalidate_caches()
    planning = importlib.import_module("st09_contracts.planning")
    if "occupancy_domain_complete" not in planning.PlanningSnapshot.__dataclass_fields__:
        raise SystemExit("PlanningSnapshot missing the domain flag: %s" % planning.__file__)
    return planning.__file__


def _round3(values):
    return [round(float(v), 6) for v in values]


def _ik_row(point, prepared, tcp, attachment):
    chain = (not prepared.reason) and has_kinematic_chain(prepared)
    layers = []
    for record in prepared.attempts:
        layers.append({
            "layer": record.get("layer"),
            "family": record.get("layer_family"),
            "seeds": record.get("seeds"),
            "raw_ik_ok": record.get("raw_ik_ok"),
            "joint_limit_reject": record.get("joint_limit_reject"),
            "branch_reject": record.get("branch_reject"),
            "kept": record.get("kept"),
        })
    empty = next((row for row in layers if int(row["kept"] or 0) == 0), None)
    tool_z = rotate_vector(tcp.quaternion_xyzw, (0.0, 0.0, 1.0))
    return {
        "scan_id": point["scan_id"],
        "box": point["box"],
        "yaw_deg": point["yaw_deg"],
        "depth_band": point["depth_band"],
        "lateral_band": point["lateral_band"],
        "size_m": [float(v) for v in point["size"]],
        "attachment_source": ATTACHMENT_SOURCE,
        "t_tcp_box_translation": _round3(attachment.t_tcp_box_translation),
        "t_tcp_box_quaternion_xyzw": _round3(
            attachment.t_tcp_box_quaternion_xyzw),
        "tcp_position": _round3(tcp.position),
        "tcp_quaternion_xyzw": _round3(tcp.quaternion_xyzw),
        "tcp_plus_z_world": _round3(tool_z),
        "kinematic_chain": bool(chain),
        "reason": prepared.reason or (
            "KINEMATIC_CHAIN" if chain else "KINEMATIC_NO_CHAIN"),
        "solved_layers": len(prepared.solutions),
        "empty_layer": None if empty is None else empty["layer"],
        "empty_raw_ik_ok": None if empty is None else empty["raw_ik_ok"],
        "empty_seeds": None if empty is None else empty["seeds"],
        "ik_layers": layers,
        "complete_route": False,
        "collision_checked": False,
    }


def freeze():
    scan.assert_safety_pins()
    os.makedirs(OUT, exist_ok=True)
    points = json.load(open(SCAN, encoding="utf-8"))["points"]
    _config, transform, opening, transit = ablation._scene()
    ctx = scan.SceneContext(scan.SCENE, scan.ATLAS_CONFIG)
    urdf, _fk, refine = ablation._models()
    rows = []
    path = os.path.join(OUT, "kinematic.jsonl")
    with open(path, "w", encoding="utf-8") as handle:
        for point in points:
            request = scan._request(
                ctx, transform, [point], point["scan_id"],
                time.monotonic_ns(), None)
            attachment, tcp = ablation._targets(request)
            anchors, _plan = ablation._anchors_for(
                point, transform, opening, transit, tcp, request["box_wdh"][2])
            prepared = prepare_corridor(CorridorRequest(
                anchors=anchors, orientation=tcp.quaternion_xyzw,
                start_q=request["start_joints"],
                joint_limits=urdf.joint_limits(), atlas_seeds=(),
                candidate_id=point["scan_id"]), refine, cache_ik=True)
            row = _ik_row(point, prepared, tcp, attachment)
            rows.append(row)
            handle.write(json.dumps(row, sort_keys=True) + "\n")
            handle.flush()
            print("%s %s" % (row["scan_id"], row["reason"]), flush=True)
    chains = [row["scan_id"] for row in rows if row["kinematic_chain"]]
    manifest = {
        "attachment_source": ATTACHMENT_SOURCE,
        "n": len(rows),
        "kinematic_chains": chains,
        "n_kinematic_chains": len(chains),
        "by_box": {
            box: {
                "n": sum(1 for row in rows if row["box"] == box),
                "chains": sum(1 for row in rows
                              if row["box"] == box and row["kinematic_chain"]),
            }
            for box in ("20in", "standard")},
        "superseded": [
            {
                "directory": name,
                "status": "superseded_for_attachment_convention",
                "use": "diagnostic_only",
            }
            for name in SUPERSEDED],
        "note": (
            "Chain counts in this file are the corrected denominator. "
            "Do not add them to the identity-attachment scan."),
    }
    _dump("baseline_manifest.json", manifest)
    _heatmap(rows, "kinematic_chain",
             "Tool-down attachment: kinematic chains by depth and lateral")
    print("chains %d / %d" % (len(chains), len(rows)))
    return 0


def _dump(name, document):
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, name)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(document, handle, indent=2, sort_keys=True)
        handle.write("\n")
    return path


def _load():
    rows = []
    with open(os.path.join(OUT, "kinematic.jsonl"), encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                rows.append(json.loads(line))
    return rows


def collide():
    import threading

    import corridor_lattice_score as score
    import slot64_classify_replay as classify

    scan.assert_safety_pins()
    print("contract", _prefer_source_contracts(), flush=True)
    rows = _load()
    by_id = {row["scan_id"]: row for row in rows}
    wanted = [row["scan_id"] for row in rows if row["kinematic_chain"]]
    points = [
        point for point in json.load(open(SCAN, encoding="utf-8"))["points"]
        if point["scan_id"] in set(wanted)]
    points.sort(key=lambda point: point["scan_id"])
    if not points:
        raise SystemExit("no kinematic chains")
    residuals = score._residuals()  # noqa: SLF001
    if residuals:
        raise SystemExit("residual sim processes: %s" % residuals)
    from luggage_description.scene_tf_config_utils import load_scene_tf_config
    from luggage_perception.eval.isolated_domain import isolated_replay_env
    from luggage_planning.routing.grid_search import ROUTE_TOTAL_WALL_S
    from luggage_planning.routing.lattice import LatticeStateChecker
    from luggage_planning.planning_scene_client import PlanningSceneClient
    from luggage_planning.typed_driver_adapter import container_transform_fields
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    ctx = scan.SceneContext(scan.SCENE, scan.ATLAS_CONFIG)
    scene_config = load_scene_tf_config(scan.SCENE)
    transform = container_transform_fields(scene_config)
    env = isolated_replay_env(score.DOMAIN)
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    proc = score._start_moveit(  # noqa: SLF001
        env, scan.SCENE, os.path.join(OUT, "moveit_collision.log"))
    rclpy.init()
    node = Node("tooldown_baseline")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        from luggage_msgs.srv import ValidateRouteV2
        probe = node.create_client(ValidateRouteV2, "/validate_route_v2")
        if not probe.wait_for_service(timeout_sec=90.0):
            raise SystemExit("validate_route_v2 did not come up")
        classify._sync_world(  # noqa: SLF001
            node, scene_config, {"world_objects": []})
        scene_client = PlanningSceneClient(node)
        producer, urdf, _tcp = classify._producer(  # noqa: SLF001
            node, scene_config, scan.SCENE, None, time.monotonic_ns(),
            budget_s=float(ROUTE_TOTAL_WALL_S))
        producer._clock_ns = time.monotonic_ns
        producer.corridor_planner = "lattice"
        producer.lattice_checker_factory = (
            lambda scene_states, start_q: LatticeStateChecker(
                node, scene_client, scene_states, start_q,
                distance_stride=1))
        _config, _transform, opening, transit = ablation._scene()
        parent_added = False
        for point in points:
            occupied = None
            if point["support"] == "stack":
                scan._add_support(scene_client, transform, point)  # noqa: SLF001
                parent_added = True
                surface = scan.floor_prior_surface(ctx)
                parent = {
                    "center_local": None,
                    "center_container": point["support_parent_center"],
                    "footprint": point["support_parent_footprint"],
                    "size": [
                        point["support_parent_footprint"][0],
                        point["support_parent_footprint"][1],
                        point["support_parent_height"]],
                }
                center = point["support_parent_center"]
                parent["center_local"] = [
                    float(center[0]), float(center[1]),
                    float(center[2]) - float(surface["center_base"][2])]
                scan.apply_placement(surface, parent, 0.0)
                occupied = surface
            elif parent_added:
                scene_client.remove_object("robustness_support")
                parent_added = False
            request = scan._request(
                ctx, transform, [point], point["scan_id"],
                time.monotonic_ns(),
                occupied if point["support"] == "stack" else None)
            outcome, detail = ablation._plan_recorded(producer, request)
            block = detail.get("first_block") or {}
            layer = _layer_of(
                point, request, opening, transit, urdf, block.get("joints"))
            contacts = list(block.get("contacts") or [])
            depths = [float(item["depth_m"]) for item in contacts]
            rejected = int(detail.get("collision_rejected") or 0)
            row = by_id[point["scan_id"]]
            row.update({
                "collision_checked": True,
                "complete_route": bool(outcome.get("complete_route")),
                "reason_code": outcome.get("reason_code"),
                "wall_s": outcome.get("wall_s"),
                "edge_classes": [
                    edge["edge_class"] for edge in outcome.get("edges") or []],
                "first_collision_layer": layer,
                "first_collision_load": block.get("load"),
                "contact_pairs": contacts,
                "max_penetration_m": max(depths) if depths else None,
                "min_collision_distance_m": detail.get(
                    "min_collision_distance_m"),
                "collision_rejected_states": rejected,
                "collision_free_chain": bool(outcome.get("complete_route"))
                or (outcome.get("reason_code") == "OK"),
                "alternate_clear_chain": bool(
                    rejected > 0 and (
                        outcome.get("complete_route")
                        or outcome.get("reason_code") == "OK")),
            })
            if not row["collision_free_chain"]:
                row["collision_free_chain"] = False
                row["alternate_clear_chain"] = False
            print("%s complete=%s layer=%s contacts=%s" % (
                point["scan_id"], row["complete_route"],
                layer, len(contacts)), flush=True)
    finally:
        score._stop_moveit(proc)  # noqa: SLF001
        if rclpy.ok():
            rclpy.shutdown()
    with open(os.path.join(OUT, "kinematic.jsonl"), "w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, sort_keys=True) + "\n")
    _heatmap(rows, "complete_route",
             "Tool-down attachment: certified portal-to-portal routes")
    passed = [row for row in rows if row.get("complete_route")]
    _dump("collision_summary.json", {
        "n_chains_checked": len(points),
        "complete_routes": [row["scan_id"] for row in passed],
        "complete_20in": [row["scan_id"] for row in passed
                          if row["box"] == "20in"],
        "complete_standard": [row["scan_id"] for row in passed
                              if row["box"] == "standard"],
        "touch_links": ["suction_panel", "suction_contact_frame"],
    })
    return 0


def _layer_of(point, request, opening, transit, urdf, joints):
    if not joints:
        return None
    from luggage_planning.offline_ik import refine_to_pose

    def refine(position, quaternion, seed):
        return refine_to_pose(
            urdf, urdf.path_to("suction_contact_frame"),
            position, quaternion, seed, pos_tol=2e-3, rot_tol=2e-2)

    attachment, tcp = ablation._targets(request)
    anchors, _plan = ablation._anchors_for(
        point, (
            None, None,
            request["container_transform"]["translation"],
            request["container_transform"]["quaternion_xyzw"]),
        opening, transit, tcp, request["box_wdh"][2])
    prepared = prepare_corridor(CorridorRequest(
        anchors=anchors, orientation=tcp.quaternion_xyzw,
        start_q=request["start_joints"], joint_limits=urdf.joint_limits(),
        atlas_seeds=(), candidate_id=point["scan_id"]), refine, cache_ik=True)
    del attachment
    key = tuple(round(float(v), 3) for v in joints)
    for (name, _xyz), solutions in zip(prepared.layers, prepared.solutions):
        for solution in solutions:
            if tuple(round(float(v), 3) for v in solution) == key:
                return name
    return None


def _heatmap(rows, field, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ink = "#0b0b0b"
    ink2 = "#52514e"
    ramp = ("#cde2fb", "#86b6ef", "#3987e5", "#1c5cab", "#104281")
    depths = ("near", "mid", "deep")
    laterals = ("left", "center", "right")
    fig, axes = plt.subplots(1, 2, figsize=(9.6, 4.8), dpi=140)
    fig.suptitle(title, color=ink, fontsize=11)
    image = None
    for axis, box_name in zip(axes, ("20in", "standard")):
        subset = [row for row in rows if row["box"] == box_name]
        grid, labels = [], []
        for depth in depths:
            rate_row, label_row = [], []
            for lateral in laterals:
                cell = [row for row in subset
                        if row["depth_band"] == depth
                        and row["lateral_band"] == lateral]
                if not cell:
                    rate_row.append(float("nan"))
                    label_row.append("n=0")
                else:
                    hits = sum(1 for row in cell if row.get(field))
                    rate_row.append(hits / float(len(cell)))
                    label_row.append("%d/%d" % (hits, len(cell)))
            grid.append(rate_row)
            labels.append(label_row)
        image = axis.imshow(
            grid, vmin=0.0, vmax=1.0, cmap=_ramp(ramp), aspect="equal")
        axis.set_xticks(range(3), laterals)
        axis.set_yticks(range(3), depths)
        axis.set_title(box_name, color=ink, fontsize=11)
        axis.tick_params(colors=ink, length=0)
        for spine in axis.spines.values():
            spine.set_color(ink2)
        for i in range(3):
            for j in range(3):
                value = grid[i][j]
                color = ink2 if value != value or value < 0.55 else "white"
                axis.text(j, i, labels[i][j], ha="center", va="center",
                          color=color, fontsize=9)
    fig.colorbar(image, ax=list(axes), fraction=0.046, pad=0.04)
    fig.tight_layout(rect=(0, 0.02, 1, 0.90))
    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()
    bad = _audit(fig, renderer)
    name = "kinematic_chain_rate.png" if field == "kinematic_chain" \
        else "complete_route_rate.png"
    fig.savefig(os.path.join(OUT, name))
    plt.close(fig)
    print("AUDIT %s: %s" % (name, "CLEAN" if bad == 0 else bad))


def _ramp(colors):
    from matplotlib.colors import LinearSegmentedColormap
    return LinearSegmentedColormap.from_list("route", list(colors))


def _audit(fig, renderer):
    bad = 0
    boxes = []
    for axis in fig.axes:
        for text in axis.texts:
            if not text.get_text().strip():
                continue
            boxes.append((text.get_text().split("\n")[0],
                          text.get_window_extent(renderer)))
    if fig._suptitle is not None:
        boxes.append(("title", fig._suptitle.get_window_extent(renderer)))
    for label, box in boxes:
        if (box.x0 < 1 or box.x1 > fig.bbox.x1 - 1
                or box.y0 < 1 or box.y1 > fig.bbox.y1 - 1):
            print("CLIP", label)
            bad += 1
    for i in range(len(boxes)):
        for j in range(i + 1, len(boxes)):
            left, right = boxes[i][1], boxes[j][1]
            ox = min(left.x1, right.x1) - max(left.x0, right.x0)
            oy = min(left.y1, right.y1) - max(left.y0, right.y0)
            if (ox > 2 and oy > 2 and ox * oy > 0.10 * min(
                    left.width * left.height, right.width * right.height)):
                print("OVERLAP", boxes[i][0], boxes[j][0])
                bad += 1
    return bad


def fallback():
    """One request: an empty-IK slot, then a certified slot of the same box."""
    import threading

    import corridor_lattice_score as score
    import slot64_classify_replay as classify

    scan.assert_safety_pins()
    print("contract", _prefer_source_contracts(), flush=True)
    points = {
        point["scan_id"]: point
        for point in json.load(open(SCAN, encoding="utf-8"))["points"]}
    ordered = [points["20in-02"], points["20in-03"]]
    residuals = score._residuals()  # noqa: SLF001
    if residuals:
        raise SystemExit("residual sim processes: %s" % residuals)
    from luggage_description.scene_tf_config_utils import load_scene_tf_config
    from luggage_perception.eval.isolated_domain import isolated_replay_env
    from luggage_planning.routing.grid_search import ROUTE_TOTAL_WALL_S
    from luggage_planning.routing.lattice import LatticeStateChecker
    from luggage_planning.planning_scene_client import PlanningSceneClient
    from luggage_planning.typed_driver_adapter import container_transform_fields
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    ctx = scan.SceneContext(scan.SCENE, scan.ATLAS_CONFIG)
    scene_config = load_scene_tf_config(scan.SCENE)
    transform = container_transform_fields(scene_config)
    env = isolated_replay_env(score.DOMAIN)
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.makedirs(OUT, exist_ok=True)
    proc = score._start_moveit(  # noqa: SLF001
        env, scan.SCENE, os.path.join(OUT, "moveit_fallback.log"))
    rclpy.init()
    node = Node("tooldown_fallback")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    record = {}
    try:
        from luggage_msgs.srv import ValidateRouteV2
        probe = node.create_client(ValidateRouteV2, "/validate_route_v2")
        if not probe.wait_for_service(timeout_sec=90.0):
            raise SystemExit("validate_route_v2 did not come up")
        classify._sync_world(  # noqa: SLF001
            node, scene_config, {"world_objects": []})
        scene_client = PlanningSceneClient(node)
        producer, _urdf, _tcp = classify._producer(  # noqa: SLF001
            node, scene_config, scan.SCENE, None, time.monotonic_ns(),
            budget_s=float(ROUTE_TOTAL_WALL_S))
        producer._clock_ns = time.monotonic_ns
        producer.corridor_planner = "lattice"
        producer.lattice_checker_factory = (
            lambda scene_states, start_q: LatticeStateChecker(
                node, scene_client, scene_states, start_q, distance_stride=1))
        request = scan._request(
            ctx, transform, ordered, "fallback-20in-02-then-03",
            time.monotonic_ns(), None)
        outcome, detail = ablation._plan_recorded(producer, request)
        classes = [edge["edge_class"] for edge in outcome.get("edges") or []]
        record = {
            "order": ["20in-02", "20in-03"],
            "success": bool(outcome.get("success")),
            "complete_route": bool(outcome.get("complete_route")),
            "reason_code": outcome.get("reason_code"),
            "row_reasons": outcome.get("row_reasons"),
            "edge_classes": classes,
            "retreat": classes.count("RETREAT"),
            "return": classes.count("RETURN"),
            "wall_s": outcome.get("wall_s"),
            "lattice_reason": detail.get("lattice_reason"),
        }
        print(json.dumps(record, indent=2), flush=True)
    finally:
        score._stop_moveit(proc)  # noqa: SLF001
        if rclpy.ok():
            rclpy.shutdown()
    _dump("fallback.json", record)
    return 0 if record.get("complete_route") and record.get("retreat", 0) >= 3 \
        and record.get("return", 0) >= 1 else 1


def main():
    global OUT
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("freeze", "collide", "fallback"))
    parser.add_argument("--out", default="")
    args = parser.parse_args()
    if args.out:
        OUT = args.out
    if args.command == "collide":
        return collide()
    if args.command == "fallback":
        return fallback()
    return freeze()


if __name__ == "__main__":
    raise SystemExit(main())
