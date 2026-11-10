#!/usr/bin/env python3
"""Geometry reference chain audit (G3 wave 3, plan P1).

Two subcommands:

  collect <session_dir>   inside a running observation sim (ROS_DOMAIN_ID=7
                          stack from scripts/g3/run.sh observe): spawn one
                          inch20, read the GT record, call the production
                          DetectLuggage service exactly like the driver,
                          record one synchronized RGBD frame bundle.
  analyze <session_dir> <run_dir_with_sdf_dumps> <out_json>
                          offline: hop-by-hop comparison (assets -> SDF ->
                          camera -> measurement -> planning input -> release
                          body -> side table) and the FIRST divergence.

Naming: records use footprint_x / footprint_y / vertical_extent for the
placement semantics and catalog_W/D/H for the catalog semantics
(docs/architecture/size_semantics.md).
"""

import json
import math
import os
import struct
import sys

WS = "/tmp/elfin-master-pack-baseline-g3"


# ---------------------------------------------------------------- collect

def collect(session_dir):
    import time
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from luggage_msgs.srv import SpawnNextBox, DetectLuggage

    os.makedirs(session_dir, exist_ok=True)
    rclpy.init()
    node = Node("g3_chain_audit")
    gt = {"received": False}
    yolo = {"n": 0, "labels": []}

    node.create_subscription(
        __import__("std_msgs.msg", fromlist=["String"]).String,
        "/luggage/perception/size_eval/spawned",
        lambda msg: gt.update(received=True, record=msg.data),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=ReliabilityPolicy.RELIABLE))

    from luggage_msgs.msg import YoloDetections
    node.create_subscription(
        YoloDetections, "/luggage/semantic/yolo_detections",
        lambda msg: (yolo.__setitem__("n", yolo["n"] + 1),
                     yolo["labels"].extend(
                         str(d.label) for d in msg.detections)),
        QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    spawn = node.create_client(SpawnNextBox, "/pickup_box_spawner/spawn_next_box")
    detect = node.create_client(DetectLuggage, "/luggage_detector/detect_luggage")
    for client in (spawn, detect):
        if not client.wait_for_service(timeout_sec=30.0):
            raise SystemExit("service unavailable: %s" % client.srv_name)

    def call(client, request, timeout=15.0):
        future = client.call_async(request)
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        return future.result()

    session = {"started": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
    spawn_resp = call(spawn, SpawnNextBox.Request(), timeout=20.0)
    session["spawn"] = {
        "success": bool(spawn_resp.success),
        "message": str(spawn_resp.message),
        "gt_box_wdh": [spawn_resp.box.width, spawn_resp.box.height,
                       spawn_resp.box.depth],
        "gt_box_pose_z": float(spawn_resp.box.pose.position.z),
    }
    # let detections warm up on the spawned box
    end = time.time() + 8.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    session["yolo_in_8s"] = dict(yolo)

    attempts = []
    measured = None
    for _ in range(8):
        resp = call(detect, DetectLuggage.Request(), timeout=10.0)
        if resp is None:
            attempts.append({"timeout": True})
            break
        entry = {
            "success": bool(resp.success),
            "message": str(resp.message or ""),
            "n_luggage": len(resp.luggage),
        }
        if resp.luggage:
            first = resp.luggage[0]
            entry.update({
                "id": str(first.id),
                "footprint_x_width": float(first.width),
                "footprint_y_depth": float(first.depth),
                "vertical_extent_height": float(first.height),
                "height_valid": bool(first.height_valid),
                "height_source": int(first.height_source),
                "pose_z": float(first.pose.position.z),
                "top_surface_pose_z": float(
                    first.top_surface_pose.position.z),
                "top_surface_valid": bool(first.top_surface_valid),
            })
        attempts.append(entry)
        if entry.get("height_valid") and entry.get("vertical_extent_height", 0) > 0:
            measured = entry
            break
        time.sleep(0.5)
    session["detect_attempts"] = attempts
    session["measured"] = measured
    session["gt_record"] = json.loads(gt["record"]) if gt["received"] else None

    # one synchronized frame bundle for the record (reuses b1_capture)
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    try:
        from b1_capture import BundleRecorder
        recorder = BundleRecorder(os.path.join(session_dir, "frame"))
        deadline = time.time() + 20.0
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if recorder.complete(require_detection=True):
                recorder.save()
                session["frame_bundle"] = "frame/"
                break
    except Exception as exc:  # noqa: BLE001 - bundle is supplementary
        session["frame_bundle_error"] = str(exc)[:200]

    node.destroy_node()
    rclpy.shutdown()
    with open(os.path.join(session_dir, "session.json"), "w") as handle:
        json.dump(session, handle, indent=1)
        handle.write("\n")
    print(json.dumps({"measured": measured is not None,
                      "gt": session["gt_record"] is not None,
                      "yolo_n": yolo["n"]}))
    return 0 if (measured and session["gt_record"]) else 1


# ---------------------------------------------------------------- parsing

def stl_aabb(path):
    with open(path, "rb") as handle:
        header = handle.read(80)
        count = struct.unpack("<I", handle.read(4))[0]
        payload = handle.read(50 * count)
    if len(payload) < 50 * count:
        raise ValueError("truncated STL: %s" % path)
    mins = [1e30] * 3
    maxs = [-1e30] * 3
    for i in range(count):
        nums = struct.unpack("<12f", payload[i * 50:i * 50 + 48])
        for v in range(1, 4):  # 0..2 is the normal; vertices are 3..11
            for k in range(3):
                value = nums[3 * v + k]
                mins[k] = min(mins[k], value)
                maxs[k] = max(maxs[k], value)
    return {"mins": mins, "maxs": maxs,
            "extents": [maxs[k] - mins[k] for k in range(3)]}


def dae_extents(path):
    import xml.etree.ElementTree as ET
    root = ET.parse(path).getroot()
    meter = 1.0
    for el in root.iter():
        if el.tag.endswith("unit") and el.get("meter"):
            meter = float(el.get("meter"))
            break
    vals = []
    for fa in root.iter():
        if not fa.tag.endswith("float_array"):
            continue
        source_id = str(fa.get("id") or "")
        if not (source_id.lower().startswith("verts")
                or "position" in source_id.lower()):
            continue
        vals.extend(float(v) for v in fa.text.split())
    if not vals:
        return None
    points = [vals[i:i + 3] for i in range(0, len(vals) - 2, 3)]
    mins = [min(p[k] for p in points) for k in range(3)]
    maxs = [max(p[k] for p in points) for k in range(3)]
    return {"unit_meter": meter,
            "extents": [(maxs[k] - mins[k]) * meter for k in range(3)]}


def parse_sdf(path):
    import xml.etree.ElementTree as ET
    if not os.path.exists(path):
        return None
    root = ET.parse(path).getroot()
    model = root.find("model")
    link = model.find("link")
    out = {
        "file": path,
        "model": model.get("name"),
        "static": model.findtext("static") or "false",
        "visual": None, "collision": None,
    }
    visual_mesh = link.find("visual/geometry/mesh")
    if visual_mesh is not None:
        out["visual"] = {"uri": visual_mesh.findtext("uri"),
                         "scale": visual_mesh.findtext("scale")}
    collision_box = link.find("collision/geometry/box")
    collision_mesh = link.find("collision/geometry/mesh")
    if collision_box is not None:
        out["collision"] = {"kind": "box",
                            "size": collision_box.findtext("size")}
    elif collision_mesh is not None:
        out["collision"] = {"kind": "mesh",
                            "uri": collision_mesh.findtext("uri"),
                            "scale": collision_mesh.findtext("scale")}
    return out


# ---------------------------------------------------------------- analyze

def analyze(session_dir, run_dir, out_json):
    sys.path.insert(0, os.path.join(WS, "src", "luggage_description"))
    sys.path.insert(0, os.path.join(WS, "src", "luggage_gazebo"))
    from luggage_description.suitcase_visual import iter_size_tiers
    from luggage_description.scene_tf_config_utils import (
        load_scene_tf_config, pickup_platform_top_in_world)

    session = json.load(open(os.path.join(session_dir, "session.json")))
    scene = load_scene_tf_config(os.path.join(
        WS, "src/luggage_description/config/scene_tf_deploy_update.yaml"))
    platform_top = float(pickup_platform_top_in_world(scene)[0][2])

    # hop 1: runtime assets ------------------------------------------------
    models_roots = [os.environ.get("GZ_SIM_RESOURCE_PATH", ""), os.path.join(
        WS, "install/luggage_gazebo/share/luggage_gazebo/models"), os.path.join(
        WS, "src/luggage_gazebo/models")]
    roots = [r for entry in models_roots if entry
             for r in entry.split(":") if r]
    hop1 = {"tiers": {}}
    for tier, size, mass, catalog_id in iter_size_tiers():
        for visual in ("suitcase_loafbrr", "suitcase_vintage"):
            name = "%s_%s" % (visual, tier)
            stl = dae = root_used = None
            for root in roots:
                candidate = os.path.join(root, name, "meshes", "suitcase.stl")
                if os.path.exists(candidate):
                    root_used = root
                    stl = stl_aabb(candidate)
                    dae_path = os.path.join(root, name, "meshes", "suitcase.dae")
                    dae = dae_extents(dae_path) if os.path.exists(dae_path) else None
                    break
            hop1["tiers"][name] = {
                "registered_size": list(size),
                "resolved_root": root_used,
                "stl": stl, "dae": dae,
                "stl_matches_registered": (
                    stl is not None and all(
                        abs(stl["extents"][k] - size[k]) <= 1e-3
                        for k in range(3))),
            }
    manifest_path = os.path.join(
        WS, "src/luggage_gazebo/models/sized_suitcases.json")
    manifest = json.load(open(manifest_path)) if os.path.exists(manifest_path) else {}
    hop1["manifest_keys"] = sorted(manifest.keys())

    # hop 2: exact spawned SDF --------------------------------------------
    hop2 = {
        "spawn_sdf": parse_sdf(os.path.join(run_dir, "sdf", "spawn.sdf")),
        "release_sdf": parse_sdf(os.path.join(run_dir, "sdf", "release.sdf")),
    }

    # hop 3: what the camera measured ---------------------------------------
    measured = session.get("measured") or {}
    hop3 = {
        "platform_top_world_z": platform_top,
        "measured": measured,
        "gt_box_pose_z": session.get("spawn", {}).get("gt_box_pose_z"),
        "reconstructed_support_z": (
            measured.get("top_surface_pose_z", float("nan"))
            - measured.get("vertical_extent_height", float("nan"))
            if measured else None),
    }
    if measured and session.get("gt_record"):
        gt = session["gt_record"]
        hop3["gt_command_size"] = gt.get("command_size")
        hop3["support_vs_platform_top"] = (
            hop3["reconstructed_support_z"] - platform_top
            if hop3["reconstructed_support_z"] is not None else None)

    # hop 4: planning input --------------------------------------------------
    hop4 = {
        "box_wdh_from_measurement": ([
            measured.get("footprint_x_width"),
            measured.get("footprint_y_depth"),
            measured.get("vertical_extent_height")] if measured else None),
        "r09_archived_request_box_wdh": [0.4757, 0.3234, 0.4174],
    }

    # hop 5: release collision body ------------------------------------------
    hop5 = {
        "release_sdf_collision": (hop2["release_sdf"] or {}).get("collision"),
        "note": ("observation session has no release; code path uses "
                 "command_size (catalog) until P2 switches it to the "
                 "manifest primitive_collision_size"),
    }

    # hop 6: side table -------------------------------------------------------
    gt_record = session.get("gt_record") or {}
    hop6 = {
        "catalog_command_size": gt_record.get("command_size"),
        "gt_reference_wdh": [gt_record.get("width"), gt_record.get("height"),
                             gt_record.get("depth")],
        "atlas_payload": {"enabled": False, "size": None,
                          "note": "s20 atlas payload disabled; box-size free"},
    }

    # divergence verdict ------------------------------------------------------
    checks = []
    inch = "suitcase_loafbrr_inch20"
    tier = hop1["tiers"].get(inch)
    checks.append(("hop1_assets_self_consistent",
                    bool(tier and tier["stl_matches_registered"])))
    manifest_has = inch in manifest
    checks.append(("hop1_manifest_has_tier", manifest_has))
    if measured:
        reg = tier["registered_size"] if tier else None
        dae = (tier or {}).get("dae") or {}
        dae_ext = dae.get("extents")
        m = [measured["footprint_x_width"], measured["footprint_y_depth"],
             measured["vertical_extent_height"]]
        checks.append(("hop3_measurement_matches_registered_size",
                        reg is not None and all(
                            abs(m[k] - reg[k]) <= 0.02 for k in range(3))))
        checks.append(("hop3_measurement_matches_visual_dae",
                        dae_ext is not None and all(
                            abs(sorted(m)[k] - sorted(dae_ext)[k]) <= 0.02
                            for k in range(3))))
        checks.append(("hop3_support_is_platform_top",
                        hop3["support_vs_platform_top"] is not None
                        and abs(hop3["support_vs_platform_top"]) <= 0.02))
    first = next((name for name, ok in checks if not ok), None)
    report = {
        "session": session_dir, "run": run_dir,
        "hops": {"1_assets": hop1, "2_spawned_sdf": hop2,
                 "3_camera_measurement": hop3, "4_planning_input": hop4,
                 "5_release_body": hop5, "6_side_table": hop6},
        "checks": dict(checks), "first_divergence": first,
    }
    with open(out_json, "w") as handle:
        json.dump(report, handle, indent=1, default=str)
        handle.write("\n")
    print(json.dumps({"first_divergence": first,
                      "checks": dict(checks)}, indent=1))
    return 0


def main():
    if len(sys.argv) < 3:
        raise SystemExit(__doc__)
    if sys.argv[1] == "collect":
        return collect(sys.argv[2])
    if sys.argv[1] == "analyze":
        return analyze(sys.argv[2], sys.argv[3], sys.argv[4])
    raise SystemExit("unknown subcommand: %s" % sys.argv[1])


if __name__ == "__main__":
    sys.exit(main())
