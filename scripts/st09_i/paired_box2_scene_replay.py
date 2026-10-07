#!/usr/bin/env python3
"""Replay one certified post-656 standard route and the -10 cm witness.

Both go through ``build_production_route_producer`` on one Gazebo URDF.
The atlas loader uses the live URDF hash only. Each case resets the live
planning scene and checks that reset before planning. Isolated MoveIt is
domain 42. Gazebo is not started.
"""

from __future__ import division

import hashlib
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
for path in (
        HERE, ROOT,
        os.path.join(ROOT, "src", "luggage_planning"),
        os.path.join(ROOT, "src", "luggage_packing"),
        os.path.join(ROOT, "src", "luggage_description"),
        os.path.join(ROOT, "src", "luggage_perception"),
):
    if path not in sys.path:
        sys.path.insert(0, path)

import atlas_admission_scan as scan  # noqa: E402
import corridor_lattice_score as score  # noqa: E402
import probe_opening_certify as probes  # noqa: E402
import slot64_classify_replay as classify  # noqa: E402

OUT = os.environ.get("SCENE_REPLAY_OUT") or os.path.join(
    ROOT, "docs/status/evidence/frame_migration/ST09-parallel-I-s5/m3",
    "g3_paired_scene_replay_20261007")
SCENE_TF = scan.SCENE_TF
ATLAS_CFG = os.path.join(
    ROOT, "src/luggage_planning/config/atlas_runtime_deploy_sim.yaml")
ONLINE_MODEL_SHA256 = (
    "759bfab2cd007a520d3857b6a7b3382e29703ae1bff5e71ac47406ec3bf3cbeb")
CONTROL_XY = (-0.52, -0.535)
GZ_LAUNCH = {
    "use_gz_sim": "true",
    "hardware_plugin": "gz_ros2_control/GazeboSimSystem",
    "xacro_extra": (
        "initial_joint1:=-1.250000,initial_joint2:=-1.746600,"
        "initial_joint3:=-0.997400,initial_joint4:=-1.915300,"
        "initial_joint5:=1.771400,initial_joint6:=1.861000"),
}
KEEP_IDS = ("airport_container_real", "robot_pedestal")
GOAL_STATE_INVALID = -27


def _dump(name, document):
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, name), "w", encoding="utf-8") as handle:
        json.dump(document, handle, indent=1, sort_keys=True, default=str)
        handle.write("\n")


def _load(path):
    with open(path, encoding="utf-8") as handle:
        return json.load(handle)


def _round(values, digits=4):
    return [round(float(v), digits) for v in values or []]


def _sha256_text(text):
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def _witness():
    found = [row for row in probes._probes() if row["name"] == "standard_m10"]
    if len(found) != 1:
        raise SystemExit("standard_m10 probe missing")
    return found[0]


def _slot656(document):
    return next(
        row for row in document["candidates"]
        if int(row.get("slot_index")) == 656)


def _control_candidate(document, surface):
    """The standard pose certified as box2-standard-after-656-0."""
    placed = _slot656(document)
    position = placed["slot_pose"]["position"]
    quaternion = placed["slot_pose"]["quaternion_xyzw"]
    ranked, skipped, _floor = scan._second_slots(
        surface, position, quaternion, document["box_wdh"],
        scan.STANDARD_WDH, None, 0.0)
    best = None
    for slot in ranked:
        center = slot["slot_pose"]["position"]
        dx = abs(float(center[0]) - CONTROL_XY[0])
        dy = abs(float(center[1]) - CONTROL_XY[1])
        if best is None or (dx + dy) < best[0]:
            best = (dx + dy, slot)
    if best is None or best[0] > 0.002:
        raise SystemExit(
            "control pose missing (nearest %s, deep_skipped %s)"
            % (None if best is None else best[1]["slot_pose"]["position"],
               skipped))
    slot = best[1]
    slot["slot_index"] = 0
    return slot


def _empty_surface(surface, document):
    return scan._static_empty_surface(surface, document)


def _occupied(surface):
    return sum(
        1 for row in surface.get("state") or [] for cell in row
        if cell == "occupied")


def _static_diff(control_slot, witness):
    box0 = _load(scan.REQUEST)
    surface0 = _load(scan.SURFACE)
    empty = _empty_surface(surface0, box0)
    placed = _slot656(box0)
    world, world_q = scan._world_box(
        box0["container_transform"],
        placed["slot_pose"]["position"],
        placed["slot_pose"]["quaternion_xyzw"])
    scene = _load(witness["scene_path"])
    placed_body = next(
        obj for obj in scene["world_objects"] if obj.get("id") == "placed_0_0_0")
    measured = _load(witness["surface_path"])
    trial = _load(os.path.join(
        os.path.dirname(os.path.dirname(witness["scene_path"])),
        "trial_00_ok/trial.json"))
    verify = (trial.get("extras") or {}).get("verify") or {}
    return {
        "control_request": {
            "label": "box2-standard-after-656-0",
            "source": "twostep_limit8",
            "center": control_slot["slot_pose"]["position"],
            "quaternion_xyzw": control_slot["slot_pose"]["quaternion_xyzw"],
            "box_wdh": list(scan.STANDARD_WDH),
            "start_joints": box0["start_joints"],
            "attachment_translation": box0["attachment_transform"]["translation"],
            "surface": {
                "map_revision": empty["map_revision"],
                "map_source": empty["map_source"],
                "occupied_cells": _occupied(empty),
                "placed_count": empty.get("placed_count"),
            },
            "first_box_collision": {
                "id": "scan_placed_0",
                "xyz": [float(v) for v in world],
                "quat": [float(v) for v in world_q],
                "size": [float(v) for v in box0["box_wdh"]],
                "note": (
                    "Added by the scan client before planning. The request "
                    "surface itself has zero occupied cells."),
            },
        },
        "witness_request": {
            "label": "standard_m10",
            "center": witness["center"],
            "quaternion_xyzw": witness["quaternion_xyzw"],
            "box_wdh": witness["box_wdh"],
            "request_path": witness["request_path"],
            "surface": {
                "map_revision": measured.get("map_revision"),
                "map_source": measured.get("map_source"),
                "occupied_cells": _occupied(measured),
                "placed_count": measured.get("placed_count"),
                "cargo_observed": measured.get("cargo_observed"),
            },
            "online_scene_ids": [
                obj.get("id") for obj in scene.get("world_objects") or []],
            "placed_0_0_0": {
                "xyz": placed_body.get("xyz"),
                "quat": placed_body.get("quat"),
                "size": placed_body.get("size"),
            },
        },
        "expression": {
            "placed_count_0": (
                "surface placed_count is len(commit_ledger). This dump is "
                "map_source=measured and cargo_observed=true, so the 66 "
                "cells are the sensor view. The commit ledger is empty."),
            "occupancy_vs_live_scene": (
                "scene.json for the online second-box failure has the "
                "container, the pedestal, and placed_0_0_0. It does not "
                "contain 66 occupancy boxes. Production adds snapshot "
                "cells only on unloaded edges."),
            "fall_m": verify.get("fall_m"),
            "support_rest": verify.get("support_rest"),
            "box_pose_at_release": verify.get("box_pose_at_release"),
            "err_z_vs_support": verify.get("err_z"),
            "gt_note": (
                "The 7.9 cm gap is fall_m: the release-GT sample is the "
                "pose at vacuum-off, and placed_0_0_0 is the support-rest "
                "center. Settled z matches that center to about 1 mm."),
        },
        "harness_not_replayed": (
            "The earlier witness select pre-inserted 66 world-frame "
            "snapshot cells and substituted the atlas URDF hash. This "
            "replay does neither."),
    }


def _stats_for(surface, clock_ns):
    def stats():
        return {
            "success": True,
            "last_acquisition_stamp_ns": int(clock_ns()) - 1_000_000_000,
            "map_source": str(surface.get("map_source") or "measured"),
            "placed_count": int(surface.get("placed_count") or 0),
            "cargo_observed": bool(surface.get("cargo_observed")),
            "map_revision": int(surface.get("map_revision") or 0),
            "geometry_hash": str(surface.get("geometry_hash") or ""),
        }
    return stats


def _host(node, scene_config, clock_ns):
    from luggage_planning.atlas_runtime import (
        compute_reachability_scene_hash,
        compute_scene_tf_hash,
        load_atlas_runtime_for_scene,
        load_runtime_atlas,
    )
    from luggage_planning.urdf_identity import UrdfIdentity

    node._scene = scene_config
    node._scene_tf_hash = compute_scene_tf_hash(SCENE_TF)
    node._reachability_scene_hash = compute_reachability_scene_hash(SCENE_TF)
    node._atlas_cfg = load_atlas_runtime_for_scene(SCENE_TF, ATLAS_CFG)
    node._urdf_identity = UrdfIdentity(node)
    node._urdf_identity.start()
    node._atlas = None
    node._atlas_diag = {"loaded": False, "reason": "pending"}
    node._atlas_key = None
    node._clock_ns = clock_ns

    def runtime_atlas():
        node._urdf_identity.poll()
        urdf_hash = (
            node._urdf_identity.hash if node._urdf_identity.ready else "")
        key = (node._reachability_scene_hash, node._scene_tf_hash, urdf_hash)
        if key != node._atlas_key:
            atlas, diag = load_runtime_atlas(
                node._atlas_cfg, scene_tf_hash=node._scene_tf_hash,
                urdf_hash=urdf_hash,
                reachability_scene_hash=node._reachability_scene_hash)
            diag = dict(diag)
            diag["live_urdf_hash"] = urdf_hash
            diag["urdf_hash_override"] = ""
            node._atlas, node._atlas_diag = atlas, diag
            node._atlas_key = key
        return node._atlas, node._atlas_diag

    node._runtime_atlas = runtime_atlas
    return node


def _wait_urdf(node, timeout_s=30.0):
    deadline = time.perf_counter() + timeout_s
    while time.perf_counter() < deadline:
        node._urdf_identity.poll()
        if node._urdf_identity.ready:
            return True
        time.sleep(0.2)
    return False


def _robot_model(node):
    from rcl_interfaces.srv import GetParameters

    client = node.create_client(
        GetParameters, "/robot_state_publisher/get_parameters")
    if not client.wait_for_service(timeout_sec=15.0):
        return {"ready": False, "reason": "robot_state_publisher_unavailable"}
    request = GetParameters.Request()
    request.names = ["robot_description"]
    event_done = __import__("threading").Event()
    future = client.call_async(request)
    future.add_done_callback(lambda _f: event_done.set())
    if not event_done.wait(15.0):
        return {"ready": False, "reason": "robot_description_timeout"}
    result = future.result()
    text = ""
    if result is not None and result.values:
        text = str(result.values[0].string_value or "")
    from luggage_planning.atlas_runtime import compute_urdf_hash
    from luggage_planning.offline_ik import default_urdf_xacro, load_urdf
    from luggage_description.xacro_robot_with_scene_base import (
        expand_and_patch,
    )

    xacro_path = default_urdf_xacro()
    offline_xml, _patched = expand_and_patch(
        xacro_path, SCENE_TF, ["fixed_world:=true"])
    return {
        "ready": bool(text),
        "live_sha256": _sha256_text(text) if text else "",
        "live_urdf_hash": compute_urdf_hash(text) if text else "",
        "matches_online_sha256": (
            _sha256_text(text) == ONLINE_MODEL_SHA256 if text else False),
        "offline_ik_sha256": _sha256_text(offline_xml),
        "offline_ik_urdf_hash": compute_urdf_hash(offline_xml),
        "offline_ik_equals_live": offline_xml == text,
        "gz_plugin": "gz_ros2_control/GazeboSimSystem" in text,
        "urdf_hash_override": "",
        "load_urdf_cached": bool(load_urdf(scene_tf_config=SCENE_TF)),
    }


def _scene_client(node):
    from luggage_planning.planning_scene_client import PlanningSceneClient

    client = PlanningSceneClient(node)
    if not client.wait_ready(timeout_sec=30.0):
        raise RuntimeError("apply_planning_scene unavailable")
    return client


def _components(names_only):
    from moveit_msgs.msg import PlanningSceneComponents

    parts = (
        PlanningSceneComponents.WORLD_OBJECT_NAMES
        | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
    if not names_only:
        parts |= (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
    return parts


def _acm_pairs(scene):
    acm = getattr(scene, "allowed_collision_matrix", None)
    if acm is None:
        return None, []
    names = list(acm.entry_names)
    pairs = []
    for index, name in enumerate(names):
        values = acm.entry_values[index].enabled if index < len(
            acm.entry_values) else []
        for other_index, flag in enumerate(values):
            if flag and other_index < len(names):
                left, right = sorted((str(name), str(names[other_index])))
                pairs.append([left, right])
    unique = [list(item) for item in sorted(set(tuple(row) for row in pairs))]
    default_names = [
        str(item) for item in (getattr(acm, "default_entry_names", None) or [])]
    default_values = [
        bool(item) for item in (getattr(acm, "default_entry_values", None) or [])]
    default_entry = getattr(acm, "default_entry", None)
    return {
        "default_entry": None if default_entry is None else bool(default_entry),
        "default_names": default_names,
        "default_values": default_values,
    }, unique


def _inventory(client, names_only=False):
    from luggage_planning.planning_scene_client import summarize_planning_scene

    scene = client.get_scene(_components(names_only), timeout=8.0)
    if scene is None:
        return {"error": "no planning scene"}
    summary = summarize_planning_scene(scene)
    objects = []
    for row in summary.get("world_objects") or []:
        objects.append({
            "id": row.get("id"),
            "type": row.get("type"),
            "xyz": _round(row.get("xyz") or [], 4),
            "quat": _round(row.get("quat") or [], 6),
            "size": _round(row.get("size") or [], 4),
            "n_triangles": row.get("n_triangles"),
        })
    objects.sort(key=lambda item: str(item["id"]))
    default_entry, pairs = (None, [])
    if not names_only:
        default_entry, pairs = _acm_pairs(scene)
    attached = sorted(summary.get("attached") or [])
    digest_body = {
        "objects": objects,
        "attached": attached,
        "acm": default_entry,
        "acm_pairs": pairs,
    }
    return {
        "world_object_ids": [item["id"] for item in objects],
        "objects": objects,
        "attached": attached,
        "acm_default": default_entry,
        "acm_pair_count": len(pairs),
        "digest": hashlib.sha256(
            json.dumps(digest_body, sort_keys=True).encode("utf-8")
        ).hexdigest(),
    }


def _purge(client):
    current = _inventory(client, names_only=True)
    removed = []
    for object_id in current.get("world_object_ids") or []:
        if object_id in KEEP_IDS:
            continue
        client.remove_object(object_id)
        removed.append(object_id)
    for box_id in ("b-payload", "pickup_box", "cert-payload"):
        client.detach_and_remove(box_id=box_id)
    return removed


def _apply_box(client, object_id, xyz, quat, size):
    ok = client.add_collision_box(
        object_id, list(xyz), list(quat), tuple(size), frame_id="world")
    if not classify._applied(ok):  # noqa: SLF001
        raise RuntimeError("add %s failed: %r" % (object_id, ok))


def _reset_case(client, kind, box0, witness):
    removed = _purge(client)
    added = []
    if kind == "scan":
        placed = _slot656(box0)
        world, world_q = scan._world_box(
            box0["container_transform"],
            placed["slot_pose"]["position"],
            placed["slot_pose"]["quaternion_xyzw"])
        _apply_box(
            client, "scan_placed_0", world, world_q, box0["box_wdh"])
        added.append("scan_placed_0")
    elif kind == "production":
        scene = _load(witness["scene_path"])
        body = next(
            obj for obj in scene["world_objects"]
            if obj.get("id") == "placed_0_0_0")
        _apply_box(
            client, "placed_0_0_0", body["xyz"], body["quat"], body["size"])
        added.append("placed_0_0_0")
    else:
        raise RuntimeError("unknown scene kind %s" % kind)
    time.sleep(0.3)
    inventory = _inventory(client, names_only=False)
    return {
        "removed": removed,
        "added": added,
        "inventory": inventory,
    }


def _instrument(builder, node, client, sink):
    original = builder.build_edge

    def wrapped(from_node, to_node, edge_class, candidate=None, scene=None):
        before = len(builder._request_dumps)
        started = time.perf_counter()
        error = None
        try:
            return original(
                from_node, to_node, edge_class,
                candidate=candidate, scene=scene)
        except Exception as exc:  # noqa: BLE001 - recorded, then re-raised
            error = exc
            raise
        finally:
            dumps = list(builder._request_dumps[before:])
            last = dumps[-1] if dumps else {}
            code = last.get("error_code")
            row = {
                "edge_class": str(getattr(edge_class, "value", edge_class)),
                "load_state": getattr(scene, "load_state", None),
                "from_q": last.get("from_q"),
                "to_q": last.get("to_q"),
                "error_code": code,
                "wall_s": round(time.perf_counter() - started, 3),
                "scene_dump": (
                    builder._scene_dumps[-1] if builder._scene_dumps else None),
            }
            try:
                live = _inventory(client, names_only=error is None)
                row["live_ids"] = live.get("world_object_ids")
                row["attached"] = live.get("attached")
                if error is not None:
                    row["live_scene"] = live
            except Exception as exc:  # noqa: BLE001 - diagnostics must not hide the edge error
                row["live_error"] = str(exc)
            if error is not None and last.get("to_q") is not None:
                try:
                    row["goal_validity"] = _goal_validity(node, last.get("to_q"))
                except Exception as exc:  # noqa: BLE001
                    row["goal_validity"] = {
                        "status": "not_evaluated", "reason": str(exc)}
            sink.append(row)

    builder.build_edge = wrapped


def _goal_validity(node, joints):
    from moveit_msgs.srv import GetStateValidity
    from sensor_msgs.msg import JointState

    if not hasattr(node, "_validity_client"):
        node._validity_client = node.create_client(
            GetStateValidity, "/check_state_validity")
    client = node._validity_client
    if not client.wait_for_service(timeout_sec=5.0):
        return {"status": "not_evaluated", "reason": "service_unavailable"}
    request = GetStateValidity.Request()
    request.group_name = "elfin_arm"
    request.robot_state.is_diff = True
    request.robot_state.joint_state = JointState()
    request.robot_state.joint_state.name = list(score.JOINTS)
    request.robot_state.joint_state.position = [float(v) for v in joints]
    done = __import__("threading").Event()
    future = client.call_async(request)
    future.add_done_callback(lambda _f: done.set())
    if not done.wait(8.0):
        return {"status": "not_evaluated", "reason": "timeout"}
    response = future.result()
    if response is None:
        return {"status": "not_evaluated", "reason": "empty_response"}
    contacts = []
    for contact in list(response.contacts or [])[:12]:
        contacts.append({
            "body_1": str(contact.contact_body_1),
            "body_2": str(contact.contact_body_2),
            "depth": round(float(contact.depth), 6),
        })
    return {
        "status": "queried",
        "valid": bool(response.valid),
        "n_contacts": len(list(response.contacts or [])),
        "contacts": contacts,
        "error_code_was": GOAL_STATE_INVALID,
    }


def _cert_legs(producer):
    adapter = getattr(producer, "_last_adapter", None)
    legs = []
    for item in (getattr(adapter, "requests", None) or []):
        if item.get("kind") != "validate_route":
            continue
        for leg in item.get("legs") or []:
            legs.append({
                "edge": leg.get("edge"),
                "reason": leg.get("reason"),
                "n_samples": leg.get("n_samples"),
            "leg_time_ms": round(
                int(leg.get("leg_time_ns") or 0) / 1e6, 1),
            "collision_ms": round(
                int(leg.get("collision_time_ns") or 0) / 1e6, 1),
            "distance_ms": round(
                int(leg.get("distance_time_ns") or 0) / 1e6, 1),
            "distance_queries": int(leg.get("distance_queries") or 0),
            })
    return legs


def _run_case(node, client, case, clock_holder):
    from luggage_description.scene_tf_config_utils import load_scene_tf_config
    from luggage_planning.route_producer import register_attach_from_request
    from luggage_planning.route_producer_factory import (
        build_production_route_producer,
    )

    document = case["document"]
    surface = case["surface"]
    stamp_ns = int(surface.get("acquisition_stamp_ns") or 1)
    scene_config = load_scene_tf_config(SCENE_TF)
    started = time.perf_counter()

    def clock():
        return stamp_ns + 1_000_000_000 + int(
            (time.perf_counter() - started) * 1e9)

    clock_holder[0] = clock
    _host(node, scene_config, clock)
    producer = build_production_route_producer(
        node, SCENE_TF, lambda: surface, _stats_for(surface, clock),
        scene=scene_config, clock_ns=clock, box_count=1,
        corridor_planner="lattice",
        scene_revision=case["scene_revision"],
        vacuum_state_fn=lambda: {
            "attached": True, "vacuum_on": True, "received_ns": clock(),
        },
        site_poses_fn=lambda: None)
    if producer is None:
        return {"outcome": "not_evaluated", "reason": "producer_unavailable"}
    if not producer.edge_builder.wait_ready(timeout_sec=60.0):
        return {"outcome": "not_evaluated", "reason": "move_group_unavailable"}
    _wait_urdf(node)
    factory = producer.validity_adapter_factory

    def capturing(**kwargs):
        adapter = factory(**kwargs)
        producer._last_adapter = adapter
        return adapter

    producer.validity_adapter_factory = capturing
    edges = []
    _instrument(producer.edge_builder, node, client, edges)
    pre = _reset_case(client, case["scene_kind"], case["box0"], case["witness"])
    request = classify._build_request(  # noqa: SLF001
        document, surface, case["slot"], case["scene_revision"], stamp_ns)
    for key in (
            "floor_z_container", "height_above_floor",
            "origin_xy_container", "resolution_m"):
        if key in surface:
            request["surface"][key] = surface[key]
    request["box_wdh"] = [float(v) for v in case["box_wdh"]]
    request["request_id"] = case["name"]
    request["box_id"] = case["name"]
    request["start_joints"] = [float(v) for v in case["start_joints"]]
    register_attach_from_request(
        producer.attachment_provider, request,
        box_wdh=request["box_wdh"], box_id=request["box_id"], generation=1)
    wall_started = time.perf_counter()
    result = producer.plan_place_route(request)
    wall_s = time.perf_counter() - wall_started
    reset = getattr(producer.edge_builder, "reset_scene", None)
    if callable(reset):
        reset()
    post = _inventory(client, names_only=False)
    rows = list(result.get("rows") or [])
    row = next((
        item for item in reversed(rows)
        if item.get("reason") not in ("ATTEMPT_SUMMARY", "", None)),
        rows[-1] if rows else {})
    failed = [
        item for item in edges
        if item.get("error_code") not in (None, 1)]
    return {
        "name": case["name"],
        "success": bool(result.get("success")),
        "reason_code": result.get("reason_code"),
        "row_reason": row.get("reason"),
        "detail": row.get("detail"),
        "wall_s": round(wall_s, 3),
        "pre_digest": pre["inventory"].get("digest"),
        "pre_ids": pre["inventory"].get("world_object_ids"),
        "pre_attached": pre["inventory"].get("attached"),
        "pre_acm_pairs": pre["inventory"].get("acm_pair_count"),
        "post_reset_digest": post.get("digest"),
        "post_reset_ids": post.get("world_object_ids"),
        "edges": edges,
        "failed_edges": failed,
        "cert_legs": _cert_legs(producer),
        "atlas_diag": getattr(producer, "last_atlas_diag", None),
        "goal_joints": [
            {
                "edge_class": item.get("edge_class"),
                "error_code": item.get("error_code"),
                "to_q": item.get("to_q"),
                "goal_validity": item.get("goal_validity"),
            }
            for item in failed
        ],
    }


def _cases(control_slot, witness, box0, empty, measured):
    witness_document = _load(witness["request_path"])
    return [
        {
            "name": "native_control",
            "scene_kind": "scan",
            "scene_revision": "paired-scan-scene",
            "document": box0,
            "surface": empty,
            "slot": control_slot,
            "box_wdh": list(scan.STANDARD_WDH),
            "start_joints": box0["start_joints"],
            "box0": box0,
            "witness": witness,
        },
        {
            "name": "control_pose_on_production_scene",
            "scene_kind": "production",
            "scene_revision": "paired-production-scene",
            "document": witness_document,
            "surface": measured,
            "slot": control_slot,
            "box_wdh": [float(v) for v in witness["box_wdh"]],
            "start_joints": witness_document["start_joints"],
            "box0": box0,
            "witness": witness,
        },
        {
            "name": "witness_repeat_0",
            "scene_kind": "production",
            "scene_revision": "paired-production-scene",
            "document": witness_document,
            "surface": measured,
            "slot": {
                "slot_index": 0,
                "slot_pose": {
                    "position": [float(v) for v in witness["center"]],
                    "quaternion_xyzw": [
                        float(v) for v in witness["quaternion_xyzw"]],
                },
            },
            "box_wdh": [float(v) for v in witness["box_wdh"]],
            "start_joints": witness_document["start_joints"],
            "box0": box0,
            "witness": witness,
        },
        {
            "name": "witness_repeat_1",
            "scene_kind": "production",
            "scene_revision": "paired-production-scene",
            "document": witness_document,
            "surface": measured,
            "slot": {
                "slot_index": 0,
                "slot_pose": {
                    "position": [float(v) for v in witness["center"]],
                    "quaternion_xyzw": [
                        float(v) for v in witness["quaternion_xyzw"]],
                },
            },
            "box_wdh": [float(v) for v in witness["box_wdh"]],
            "start_joints": witness_document["start_joints"],
            "box0": box0,
            "witness": witness,
        },
    ]


def _decide(rows):
    by_name = {row["name"]: row for row in rows}
    production = [
        by_name.get("control_pose_on_production_scene"),
        by_name.get("witness_repeat_0"),
        by_name.get("witness_repeat_1"),
    ]
    digests = [row.get("pre_digest") for row in production if row]
    same_initial = len(set(digests)) == 1 and digests[0]
    witness_reasons = [
        (by_name.get(name) or {}).get("row_reason")
        for name in ("witness_repeat_0", "witness_repeat_1")]
    return {
        "native_control_success": (by_name.get("native_control") or {}).get(
            "success"),
        "control_on_production_success": (
            by_name.get("control_pose_on_production_scene") or {}).get(
                "success"),
        "witness_reasons": witness_reasons,
        "production_pre_digests_equal": bool(same_initial),
        "production_pre_digest": digests[0] if same_initial else digests,
    }


def main():
    import rclpy
    from rclpy.node import Node
    from luggage_description.scene_tf_config_utils import load_scene_tf_config
    from luggage_perception.eval.isolated_domain import isolated_replay_env

    os.makedirs(OUT, exist_ok=True)
    residuals = score._residuals()  # noqa: SLF001
    if residuals:
        _dump("manifest.json", {
            "outcome": "not_evaluated", "residuals": residuals})
        print(json.dumps({"outcome": "not_evaluated", "residuals": residuals}))
        return 2
    witness = _witness()
    box0 = _load(scan.REQUEST)
    empty = _empty_surface(_load(scan.SURFACE), box0)
    measured = _load(witness["surface_path"])
    from luggage_description.scene_tf_config_utils import (
        container_inner_floor_z,
        load_scene_tf_config,
    )
    measured["floor_z_container"] = float(container_inner_floor_z(
        load_scene_tf_config(SCENE_TF)))
    measured["height_above_floor"] = measured.get("height")
    measured["origin_xy_container"] = measured.get("origin_local")
    measured["resolution_m"] = measured.get("resolution")
    control_slot = _control_candidate(box0, empty)
    static = _static_diff(control_slot, witness)
    _dump("static_diff.json", static)
    env = isolated_replay_env(score.DOMAIN)
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    proc = score._start_moveit(  # noqa: SLF001
        env, SCENE_TF, os.path.join(OUT, "moveit.log"), GZ_LAUNCH)
    rclpy.init()
    node = Node("paired_box2_scene_replay")
    stop, thread = score._spin(node)  # noqa: SLF001
    reports = []
    model = {}
    try:
        from luggage_msgs.srv import ValidateRouteV2

        client_srv = node.create_client(ValidateRouteV2, "/validate_route_v2")
        if not client_srv.wait_for_service(timeout_sec=90.0):
            _dump("manifest.json", {
                "outcome": "not_evaluated",
                "reason": "validate_route_unavailable"})
            return 2
        scene_client = _scene_client(node)
        classify._sync_world(  # noqa: SLF001
            node, load_scene_tf_config(SCENE_TF), {"world_objects": []})
        model = _robot_model(node)
        _dump("robot_model.json", model)
        for case in _cases(control_slot, witness, box0, empty, measured):
            print(json.dumps({"case": case["name"], "event": "start"}),
                  flush=True)
            report = _run_case(node, scene_client, case, [None])
            reports.append(report)
            _dump("cases.json", reports)
            print(json.dumps({
                "case": case["name"],
                "success": report.get("success"),
                "reason": report.get("row_reason"),
                "wall_s": report.get("wall_s"),
                "pre_digest": report.get("pre_digest"),
            }), flush=True)
        decision = _decide(reports)
        decision["collision_urdf_md5"] = (
            (reports[-1].get("atlas_diag") or {}).get("live_urdf_hash")
            if reports else "")
        decision["urdf_hash_override"] = ""
        decision["online_model_sha256_is_urdf_plus_srdf"] = ONLINE_MODEL_SHA256
        _dump("decision.json", decision)
        return 0
    finally:
        stop.set()
        thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        score._stop_moveit(proc)  # noqa: SLF001
        manifest = {
            "outcome": "complete" if reports else "not_evaluated",
            "n_cases": len(reports),
            "robot_model": {
                "live_sha256": model.get("live_sha256"),
                "matches_online_sha256": model.get("matches_online_sha256"),
                "urdf_hash_override": "",
                "offline_ik_equals_live": model.get("offline_ik_equals_live"),
            },
            "residuals_after": score._residuals(),  # noqa: SLF001
            "gazebo": False,
            "replay_command": "python3 scripts/st09_i/paired_box2_scene_replay.py",
        }
        _dump("manifest.json", manifest)


if __name__ == "__main__":
    sys.exit(main())
