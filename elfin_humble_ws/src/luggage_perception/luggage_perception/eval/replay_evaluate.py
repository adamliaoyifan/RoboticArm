"""Eval-only pendant bag replay orchestrator. Not imported by online nodes.

Two streaming passes over the mcap (see bag_mcap_source): pass A indexes
header stamps and tiny aux payloads, computes the pure colour↔depth join
plan (bag_frame_join) and writes the join report; pass B re-streams only
the image topics, runs the ROS-free YOLO segmenter on each planned frame
and writes the timestamp-keyed evidence tree. camera_info is static and
written once per bag, not per frame.

The segmenter is driven through ``segment()`` directly — never
``update()``, which needs online TF for self-body painting, workspace
acceptance and the temporal gate. A stub backend failing silently is
refused (the workspace rule that an eval report can never be faked).
"""
from __future__ import division

import json
import math
import os
import time
from dataclasses import dataclass, field

import numpy as np

from luggage_perception.eval.bag_frame_join import (
    dedupe_stamped_entries,
    frame_dir_name,
    nearest_stamp,
    plan_frame_join,
)
from luggage_perception.eval.bag_mcap_source import (
    TOPIC_TYPES,
    decode_color_rgb,
    decode_depth_mm,
    decode_cloud_xyz,
    find_mcap_file,
    iter_bag_messages,
    scan_bag,
)
from luggage_perception.eval.detection_gate_sampling import (
    colorize_mask_rgb,
    depth_vis_uint8,
    write_png,
)
from luggage_perception.eval.gate4_dump import write_ply_xyz
from luggage_perception.ros_message_adapters import (
    camera_info_frame_from_msg,
    joint_sample_from_msg,
)
from luggage_perception.semantic_segmenter import draw_detections_overlay

COLOR_TOPIC = "/camera/d555/color/image_raw"
DEPTH_TOPIC = "/camera/d555/aligned_depth_to_color/image_raw"
COLOR_INFO_TOPIC = "/camera/d555/color/camera_info"
DEPTH_INFO_TOPIC = "/camera/d555/aligned_depth_to_color/camera_info"
JOINT_TOPIC = "/joint_states"
TCP_TOPIC = "/elfin/tcp_pose"
LIDAR_TOPIC = "/livox/lidar"
TF_STATIC_TOPIC = "/tf_static"

# Mirrors config/semantic_segmenter.yaml (the sim-tuned prompt set already
# fires on the real pendant footage: conf 0.956 on the first record_site
# frame). The yaml stays the source of truth when it is loadable.
DEFAULT_PROMPTS = [
    "luggage on a platform viewed from directly above",
    "vintage leather suitcase with handles viewed from directly above",
    "suitcase", "luggage", "box", "container", "floor", "robot arm",
]
DEFAULT_CLASS_MAPPING_LABELS = [2, 2, 2, 2, 2, 1, 0, 3]

# Real-site replay prompt set (2026-09-10 ablation on the pendant bags).
# The scene-descriptive prompt carries the detection (bare "suitcase" alone
# collapses to conf ~0.1); "box"/"floor"/"container"/"robot arm" produced
# the low-confidence false-positive boxes (4.7k 'box' boxes in vaccum3) and
# are dropped. Everything maps to cargo.
REAL_SITE_PROMPTS = [
    "luggage on a platform viewed from directly above",
    "suitcase",
    "luggage",
]
REAL_SITE_CLASS_MAPPING_LABELS = [2, 2, 2]

NS_PER_MS = 1_000_000

LABEL_CARGO = 2


def select_cargo_detection(detections, image_size, min_conf=0.3,
                           center_radius_frac=0.35):
    """Pick THE luggage detection for one frame (one-cargo-per-frame site).

    Rule, from the 2026-09-10 pendant-bag measurements: the true box is the
    max-confidence cargo det in 97-100% of frames AND lies near the image
    centre (the wrist camera keeps the suitcase centred), while strong
    false positives sit at the border. So: among cargo detections with
    ``confidence >= min_conf``, prefer those whose bbox centre lies within
    ``center_radius_frac * width`` px of the image centre and keep the
    max-confidence one; with no central candidate keep the max-confidence
    det overall and flag it ``off_center``; with no candidate above the
    floor keep nothing (an honest miss, never a hallucinated box).

    Returns ``(kept_detection_or_None, info_dict)`` where info records the
    selection mode, the raw cargo count and the dropped detections.
    """
    width, height = int(image_size[0]), int(image_size[1])
    cx, cy = width / 2.0, height / 2.0
    radius = float(center_radius_frac) * width

    def _center_dist(det):
        x1, y1, x2, y2 = det["bbox"]
        return math.hypot((x1 + x2) / 2.0 - cx, (y1 + y2) / 2.0 - cy)

    cargo = [det for det in detections
             if int(det.get("label", -1)) == LABEL_CARGO]
    above = [det for det in cargo
             if float(det.get("confidence", 0.0)) >= float(min_conf)]
    central = [det for det in above if _center_dist(det) <= radius]

    kept, mode = None, "none"
    if central:
        kept = max(central, key=lambda det: float(det["confidence"]))
        mode = "central"
    elif above:
        kept = max(above, key=lambda det: float(det["confidence"]))
        mode = "off_center"
    dropped = [det for det in cargo if det is not kept]
    info = {
        "selection": mode,
        "n_cargo_raw": len(cargo),
        "n_dropped": len(dropped),
        "kept_confidence": (float(kept["confidence"])
                            if kept is not None else None),
        "kept_center_dist_px": (round(_center_dist(kept), 1)
                                if kept is not None else None),
    }
    return kept, info


def repaint_label_map(shape, detections):
    """Rebuild a label map from ONLY the given detections (bbox fill).

    The bbox_fill backend paints every returned box — including the
    low-confidence false positives that made the earlier replay masks
    unusable. Repainting from the selected detection gives a mask that is
    exactly the accepted box.
    """
    label_map = np.zeros((int(shape[0]), int(shape[1])), dtype=np.uint8)
    height, width = label_map.shape
    for det in detections:
        x1, y1, x2, y2 = (int(v) for v in det["bbox"])
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(width, x2), min(height, y2)
        if x2 > x1 and y2 > y1:
            label_map[y1:y2, x1:x2] = int(det["label"])
    return label_map


@dataclass
class ReplayEvalConfig(object):
    backend: str = "yolo_world"
    device: str = "cuda"
    model_path: str = ""
    config_yaml: str = ""
    prompts: object = None            # list[str] override
    class_mapping_labels: object = None
    confidence: object = None
    join_tolerance_ms: float = 30.0
    aux_tolerance_ms: float = 50.0
    lidar_tolerance_ms: float = 100.0
    stride: int = 1
    max_frames: int = 0               # 0 = unlimited
    with_points: bool = True
    pixel_stride: int = 2
    with_lidar: bool = False
    cargo_select: str = "center_conf"   # "center_conf" | "none"
    center_radius_frac: float = 0.35
    save_depth_npy: bool = True
    depth_vis: bool = True
    make_video: bool = False
    require_backend: bool = True
    dry_run: bool = False


def _package_root():
    import luggage_perception
    return os.path.dirname(os.path.abspath(luggage_perception.__file__))


def resolve_model_path(model_path=""):
    """Bare filenames resolve against the package root and the share
    models dir (same contract as the node's _resolve_model)."""
    model_path = str(model_path or "yolov8s-world.pt")
    if os.path.isfile(model_path):
        return model_path
    candidates = [
        os.path.join(os.path.dirname(_package_root()), model_path),
        os.path.join(_package_root(), model_path),
    ]
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(
            get_package_share_directory("luggage_perception"),
            "models", os.path.basename(model_path)))
    except Exception:  # noqa: BLE001 share dir optional for offline runs
        pass
    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate
    return model_path


def load_segmenter_config(cfg):
    """Config dict for build_segmenter: yaml section first, CLI overrides
    last. The real-site .real.yaml override is deliberately not loaded
    (its device:cpu belongs to the robot host, not this GPU workstation).
    """
    section = {}
    if cfg.config_yaml and os.path.isfile(str(cfg.config_yaml)):
        import yaml
        with open(cfg.config_yaml, encoding="utf-8") as handle:
            doc = yaml.safe_load(handle) or {}
        section = (doc.get("semantic_segmenter", {})
                   .get("ros__parameters", {}))
    prompts = list(cfg.prompts) if cfg.prompts else list(
        section.get("prompts", DEFAULT_PROMPTS))
    labels = list(cfg.class_mapping_labels) if (
        cfg.class_mapping_labels) else list(
        section.get("class_mapping_labels", DEFAULT_CLASS_MAPPING_LABELS))
    if len(prompts) != len(labels):
        raise ValueError(
            "prompts (%d) and class_mapping_labels (%d) must be equal "
            "length" % (len(prompts), len(labels)))
    config = {
        "backend": str(cfg.backend),
        "prompts": prompts,
        "class_mapping": dict(zip(prompts, [int(v) for v in labels])),
        "confidence_threshold": float(
            cfg.confidence if cfg.confidence is not None else
            section.get("confidence_threshold", 0.005)),
        "model_name": resolve_model_path(cfg.model_path),
        "device": str(cfg.device),
    }
    sam2_cfg = section.get("sam2")
    if isinstance(sam2_cfg, dict) and sam2_cfg:
        config["sam2"] = dict(sam2_cfg)
    return config


def build_replay_segmenter(cfg):
    """build_segmenter + the stub-fallback guard. Returns
    (segmenter, backend_string, config_dict)."""
    from luggage_perception.semantic_segmenter import build_segmenter
    config = load_segmenter_config(cfg)
    segmenter = build_segmenter(config)
    backend = str(getattr(segmenter, "last_stats", {}).get("backend", ""))
    if not backend:
        backend = str(getattr(segmenter, "_last_stats", {}).get(
            "backend", ""))
    if cfg.require_backend and (not backend or backend.startswith("stub")):
        raise RuntimeError(
            "segmenter backend is %r (wanted %r); a stub fallback may "
            "never produce an eval report" % (backend, cfg.backend))
    return segmenter, backend, config


# ---------------------------------------------------------------------------
# Writers
# ---------------------------------------------------------------------------

def _write_json(path, payload):
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True, default=str)
        handle.write("\n")


def _camera_info_payload(frame):
    return {
        "frame_id": frame.frame_id,
        "width": frame.width, "height": frame.height,
        "k": [float(v) for v in (frame.fx, 0.0, frame.cx,
                                 0.0, frame.fy, frame.cy, 0.0, 0.0, 1.0)],
        "distortion_model": frame.distortion_model,
        "distortion_coeffs": [float(v) for v in frame.distortion_coeffs],
        "projection": [float(v) for v in frame.projection],
    }


def write_join_report(bag_out, scan, plan, duplicates, aux_stats,
                      known_limitations=None):
    report = {
        "bag_path": scan.bag_path,
        "topics": scan.topics,
        "skipped_topics": scan.skipped_topics,
        "join": plan.stats,
        "color_orphans": plan.color_orphans,
        "depth_orphans": plan.depth_orphans,
        "duplicates": {str(k): v for k, v in (duplicates or {}).items()},
        "aux_join": aux_stats,
        "known_limitations": list(known_limitations or [
            "cargo points stay in the optical frame: d555_color_optical_frame"
            " is not reachable from the recorded TF tree (driver TFs were"
            " not recorded)",
        ]),
    }
    _write_json(os.path.join(bag_out, "join_report.json"), report)
    return report


def _percentile(values, q):
    if not values:
        return None
    ordered = sorted(values)
    idx = min(len(ordered) - 1, max(0, int(
        round(q / 100.0 * (len(ordered) - 1)))))
    return float(ordered[idx])


def write_index_md(bag_out, rows, bag_name):
    lines = [
        "# Pendant bag replay — %s" % bag_name,
        "",
        "| frame | stamp | join | dets | cargo | conf max | infer ms"
        " | joint dt ms |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for row in rows:
        lines.append(
            "| [%s](frames/%s) | %.9f | %s | %d | %d | %s | %.1f | %s |" % (
                row["dir"], row["dir"], row["stamp_sec"], row["join"],
                row["n_detections"], row["n_cargo_detections"],
                ("%.3f" % row["conf_max"]) if row["conf_max"] is not None
                else "-",
                row["inference_ms"],
                ("%.1f" % row["joint_dt_ms"]) if row.get("joint_dt_ms")
                is not None else "miss",
            ))
    with open(os.path.join(bag_out, "INDEX.md"), "w",
              encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")


# ---------------------------------------------------------------------------
# Aux payload extraction
# ---------------------------------------------------------------------------

def _joint_payload(msg):
    sample = joint_sample_from_msg(msg)
    if sample is None:
        return None
    return {
        "stamp": float(sample.stamp),
        "joint_names": list(sample.joint_names),
        "position": [float(v) for v in sample.positions],
        "velocity": ([float(v) for v in sample.velocities]
                     if sample.velocities is not None else None),
    }


def _tcp_payload(msg):
    pose = msg.pose
    return {
        "stamp": float(msg.header.stamp.sec)
        + 1e-9 * float(msg.header.stamp.nanosec),
        "frame_id": msg.header.frame_id,
        "position": [float(pose.position.x), float(pose.position.y),
                     float(pose.position.z)],
        "orientation_xyzw": [
            float(pose.orientation.x), float(pose.orientation.y),
            float(pose.orientation.z), float(pose.orientation.w)],
    }


def _planned_pairs(plan, cfg):
    pairs = plan.pairs
    stride = max(1, int(cfg.stride))
    pairs = pairs[::stride]
    if cfg.max_frames and cfg.max_frames > 0:
        pairs = pairs[:int(cfg.max_frames)]
    return pairs


# ---------------------------------------------------------------------------
# Main entry
# ---------------------------------------------------------------------------

def evaluate_bag(bag_path, out_root, cfg, segmenter=None,
                 source_iter=None):
    """Replay one bag end to end. Returns the summary dict (also written
    to summary.json). ``source_iter`` injects a replacement message
    stream for tests."""
    from luggage_perception.semantic_point_filter import (
        CameraIntrinsics, DepthToColorExtrinsics, SemanticPointFilter)

    if os.path.isdir(bag_path):
        bag_name = os.path.basename(os.path.normpath(bag_path))
    else:
        bag_name = os.path.basename(str(bag_path))
        if bag_name.endswith(".mcap"):
            bag_name = bag_name[:-len(".mcap")]
    bag_out = os.path.join(out_root, bag_name)
    frames_root = os.path.join(bag_out, "frames")
    os.makedirs(frames_root, exist_ok=True)

    scan = scan_bag(bag_path)
    mcap_path = find_mcap_file(bag_path)
    stream = source_iter or (lambda topics: iter_bag_messages(
        mcap_path, topics=topics))

    index_topics = [COLOR_TOPIC, DEPTH_TOPIC, COLOR_INFO_TOPIC,
                    DEPTH_INFO_TOPIC, JOINT_TOPIC, TCP_TOPIC,
                    TF_STATIC_TOPIC]
    if cfg.with_lidar:
        index_topics.append(LIDAR_TOPIC)

    # ---- Pass A: stamps + tiny aux payloads -----------------------------
    color_entries, depth_entries = [], []
    joint_by_stamp, tcp_by_stamp = {}, {}
    lidar_by_stamp = {}
    info_first, info_k_seen = {}, {}
    tf_static_msgs = 0
    for rec in stream(index_topics):
        topic = rec.topic
        stamp = rec.header_stamp_ns
        if topic == COLOR_TOPIC:
            color_entries.append((stamp, rec.log_time_ns, None))
        elif topic == DEPTH_TOPIC:
            depth_entries.append((stamp, rec.log_time_ns, None))
        elif topic == JOINT_TOPIC:
            payload = _joint_payload(rec.message)
            if payload is not None:
                joint_by_stamp[stamp] = payload
        elif topic == TCP_TOPIC:
            tcp_by_stamp[stamp] = _tcp_payload(rec.message)
        elif topic in (COLOR_INFO_TOPIC, DEPTH_INFO_TOPIC):
            frame = camera_info_frame_from_msg(rec.message)
            if topic not in info_first:
                info_first[topic] = frame
            info_k_seen.setdefault(topic, {})[
                (frame.fx, frame.fy, frame.cx, frame.cy)] = \
                info_k_seen.setdefault(topic, {}).get(
                    (frame.fx, frame.fy, frame.cx, frame.cy), 0) + 1
        elif topic == TF_STATIC_TOPIC:
            tf_static_msgs += 1
        elif topic == LIDAR_TOPIC:
            lidar_by_stamp[stamp] = rec.message

    color_sorted, color_dup = dedupe_stamped_entries(color_entries)
    depth_sorted, depth_dup = dedupe_stamped_entries(depth_entries)
    duplicates = {"color": color_dup, "depth": depth_dup}
    plan = plan_frame_join(
        [s for s, _log, _payload in color_sorted],
        [s for s, _log, _payload in depth_sorted],
        tolerance_ns=int(cfg.join_tolerance_ms * NS_PER_MS))
    pairs = _planned_pairs(plan, cfg)
    planned_by_color = {p.stamp_ns: p for p in pairs}
    planned_depths = {p.depth_stamp_ns for p in pairs}

    joint_stamps = sorted(joint_by_stamp)
    tcp_stamps = sorted(tcp_by_stamp)
    lidar_stamps = sorted(lidar_by_stamp)
    aux_stats = {
        "n_joint_states": len(joint_stamps),
        "n_tcp_pose": len(tcp_stamps),
        "n_lidar": len(lidar_stamps),
        "aux_tolerance_ms": float(cfg.aux_tolerance_ms),
        "lidar_tolerance_ms": float(cfg.lidar_tolerance_ms),
        "tf_static_messages": tf_static_msgs,
    }

    # camera_info: stored ONCE per bag (static); variant Ks are kept
    # alongside if the intrinsics ever drift mid-recording.
    camera_info_payload = {}
    for topic in (COLOR_INFO_TOPIC, DEPTH_INFO_TOPIC):
        frame = info_first.get(topic)
        if frame is None:
            continue
        key = "color" if topic == COLOR_INFO_TOPIC else "depth"
        camera_info_payload[key] = _camera_info_payload(frame)
        variants = info_k_seen.get(topic, {})
        camera_info_payload["%s_distinct_k" % key] = len(variants)
    _write_json(os.path.join(bag_out, "camera_info.json"),
                camera_info_payload)

    report = write_join_report(
        bag_out, scan, plan,
        {"color": len(color_dup), "depth": len(depth_dup)}, aux_stats)

    summary = {
        "bag": bag_name, "bag_path": scan.bag_path,
        "frames_planned": len(pairs),
        "join": plan.stats,
        "backend": None, "device": cfg.device,
        "frames_processed": 0,
        "inference_ms": {}, "detection_conf": {},
        "per_prompt_counts": {}, "frames_with_cargo": 0,
        "skipped_topics": sorted(scan.skipped_topics),
    }
    if cfg.dry_run:
        summary["dry_run"] = True
        _write_json(os.path.join(bag_out, "summary.json"), summary)
        return summary

    segmenter, backend, config_dict = (
        segmenter if segmenter is not None
        else build_replay_segmenter(cfg))
    summary["backend"] = backend
    summary["segmenter_config"] = {
        k: v for k, v in config_dict.items() if k != "model_name"}

    color_frame = info_first.get(COLOR_INFO_TOPIC)
    if (cfg.with_points and color_frame is not None
            and hasattr(color_frame, "fx")):
        intrinsics = CameraIntrinsics(
            color_frame.fx, color_frame.fy, color_frame.cx, color_frame.cy,
            color_frame.width, color_frame.height,
            distortion_coeffs=list(color_frame.distortion_coeffs),
            distortion_model=color_frame.distortion_model)
        point_filter = SemanticPointFilter(
            intrinsics, intrinsics,
            DepthToColorExtrinsics.identity(), [2], [])
    else:
        point_filter = None

    detections_jsonl = open(os.path.join(bag_out, "detections.jsonl"),
                            "w", encoding="utf-8")
    frames_jsonl = open(os.path.join(bag_out, "frames.jsonl"),
                        "w", encoding="utf-8")
    t_start = time.time()
    pending = {"color": {}, "depth": {}}
    frame_rows, all_confs, all_infer_ms = [], [], []
    per_prompt_counts = {}
    select_stats = []

    def _flush_pair(color_stamp):
        pair = planned_by_color.pop(color_stamp)
        rgb = pending["color"].pop(color_stamp)
        depth = pending["depth"].pop(pair.depth_stamp_ns)
        _process_frame(
            bag_out, frames_root, pair, rgb, depth,
            segmenter, point_filter, cfg,
            joint_by_stamp, joint_stamps, tcp_by_stamp, tcp_stamps,
            lidar_by_stamp, lidar_stamps,
            detections_jsonl, frames_jsonl, frame_rows,
            all_confs, all_infer_ms, per_prompt_counts, select_stats)

    def _flush_ready_pairs():
        # A tolerance pair's depth stamp differs from the colour stamp, so
        # completeness must be re-checked after EVERY arrival, not only on
        # colour messages.
        while True:
            for color_stamp in sorted(pending["color"]):
                pair = planned_by_color.get(color_stamp)
                if pair is None:
                    pending["color"].pop(color_stamp, None)
                    continue
                if pair.depth_stamp_ns in pending["depth"]:
                    _flush_pair(color_stamp)
                    break
            else:
                return

    for rec in stream([COLOR_TOPIC, DEPTH_TOPIC]):
        stamp = rec.header_stamp_ns
        if rec.topic == COLOR_TOPIC:
            if stamp in planned_by_color and stamp not in pending["color"]:
                pending["color"][stamp] = decode_color_rgb(rec.message)
        else:
            if stamp in planned_depths and stamp not in pending["depth"]:
                pending["depth"][stamp] = decode_depth_mm(rec.message)
        _flush_ready_pairs()
        # Bound the pending window (planned entries only ever enter; the
        # colour stream arrives first, so >64 pending means a dropped side).
        for side in ("color", "depth"):
            if len(pending[side]) > 64:
                for key in sorted(pending[side])[:len(pending[side]) - 64]:
                    pending[side].pop(key, None)
                    if side == "color":
                        planned_by_color.pop(key, None)

    detections_jsonl.close()
    frames_jsonl.close()

    confs = [c for c in all_confs if c is not None]
    summary["frames_processed"] = len(frame_rows)
    summary["frames_with_cargo"] = sum(
        1 for row in frame_rows if row["n_cargo_detections"] > 0)
    summary["inference_ms"] = {
        "mean": (float(np.mean(all_infer_ms)) if all_infer_ms else None),
        "p95": _percentile(all_infer_ms, 95),
        "max": (float(np.max(all_infer_ms)) if all_infer_ms else None),
    }
    summary["detection_conf"] = {
        "n": len(confs),
        "p50": _percentile(confs, 50),
        "p90": _percentile(confs, 90),
        "max": _percentile(confs, 100),
    }
    summary["per_prompt_counts"] = per_prompt_counts
    summary["cargo_selection"] = {
        "mode": cfg.cargo_select,
        "min_conf": (float(cfg.confidence) if cfg.confidence is not None
                     else None),
        "center_radius_frac": float(cfg.center_radius_frac),
        "frames_central": select_stats.count("central"),
        "frames_off_center": select_stats.count("off_center"),
        "frames_no_cargo": select_stats.count("none"),
    }
    summary["wall_sec"] = round(time.time() - t_start, 2)
    _write_json(os.path.join(bag_out, "summary.json"), summary)
    write_index_md(bag_out, frame_rows, bag_name)
    with open(os.path.join(bag_out, "index.json"), "w",
              encoding="utf-8") as handle:
        json.dump(frame_rows, handle, indent=1)
        handle.write("\n")

    if cfg.make_video and frame_rows:
        _make_replay_video(bag_out, frames_root, frame_rows)
    return summary


def _process_frame(bag_out, frames_root, pair, rgb, depth,
                   segmenter, point_filter, cfg,
                   joint_by_stamp, joint_stamps,
                   tcp_by_stamp, tcp_stamps,
                   lidar_by_stamp, lidar_stamps,
                   detections_jsonl, frames_jsonl, frame_rows,
                   all_confs, all_infer_ms, per_prompt_counts,
                   select_stats):
    import cv2  # noqa: WPS433 dump path only (matches repo convention)

    stamp_ns = pair.stamp_ns
    stamp_sec = stamp_ns / 1e9
    name = frame_dir_name(stamp_ns)
    frame_dir = os.path.join(frames_root, name)
    os.makedirs(frame_dir, exist_ok=True)

    label_map, detections = segmenter.segment(rgb)
    inference_ms = float(getattr(
        segmenter, "last_stats", {}).get("inference_ms", 0.0) or 0.0)
    stats = getattr(segmenter, "last_stats", {})
    backend = str(stats.get("backend", ""))

    # One-luggage-per-frame site rule: keep THE cargo box, repaint the
    # mask from it alone (raw bbox_fill paints every false positive too).
    selection_info = {"selection": "disabled", "n_cargo_raw": sum(
        1 for det in detections if int(det.get("label", -1)) == LABEL_CARGO),
        "n_dropped": 0, "kept_confidence": None, "kept_center_dist_px": None}
    if cfg.cargo_select == "center_conf":
        min_conf = float(cfg.confidence if cfg.confidence is not None
                         else 0.3)
        kept, selection_info = select_cargo_detection(
            detections, (rgb.shape[1], rgb.shape[0]),
            min_conf=min_conf, center_radius_frac=cfg.center_radius_frac)
        detections = ([dict(kept)] if kept is not None else [])
        label_map = repaint_label_map(rgb.shape[:2], detections)
    select_stats.append(selection_info.get("selection", "none"))

    write_png(os.path.join(frame_dir, "color.png"), rgb)
    np.save(os.path.join(frame_dir, "mask.npy"), label_map)
    write_png(os.path.join(frame_dir, "mask.png"),
              colorize_mask_rgb(label_map))
    overlay_bgr = draw_detections_overlay(rgb, detections)
    cv2.imwrite(os.path.join(frame_dir, "overlay.png"), overlay_bgr)

    if depth is not None:
        if cfg.save_depth_npy:
            np.save(os.path.join(frame_dir, "depth.npy"),
                    np.asarray(depth))
        if cfg.depth_vis:
            depth_m = np.asarray(depth, dtype=np.float32) * 0.001
            write_png(os.path.join(frame_dir, "depth_vis.png"),
                      depth_vis_uint8(depth_m))

    instance_map = getattr(segmenter, "_instance_map", None)
    if instance_map is not None:
        np.save(os.path.join(frame_dir, "instance_mask.npy"), instance_map)

    det_rows = []
    for det in detections:
        row = {
            "label": int(det["label"]),
            "prompt": str(det.get("prompt", "")),
            "confidence": float(det.get("confidence", 0.0)),
            "bbox": [int(v) for v in det["bbox"]],
        }
        if "instance_id" in det:
            row["instance_id"] = int(det["instance_id"])
        det_rows.append(row)
        per_prompt_counts[row["prompt"]] = \
            per_prompt_counts.get(row["prompt"], 0) + 1
        all_confs.append(row["confidence"])
    _write_json(os.path.join(frame_dir, "detections.json"),
                {"stamp_sec": stamp_sec, "detections": det_rows})

    # Aux nearest-stamp joins (miss is reported, never silently widened).
    joint_row, joint_dt_ms = None, None
    hit = nearest_stamp(joint_stamps, stamp_ns,
                        int(cfg.aux_tolerance_ms * NS_PER_MS))
    if hit is not None:
        idx, dt = hit
        joint_row = dict(joint_by_stamp[joint_stamps[idx]])
        joint_dt_ms = dt / 1e6
        _write_json(os.path.join(frame_dir, "joint_state.json"),
                    dict(joint_row, dt_sec=dt / 1e9))
    tcp_row, tcp_dt_ms = None, None
    hit = nearest_stamp(tcp_stamps, stamp_ns,
                        int(cfg.aux_tolerance_ms * NS_PER_MS))
    if hit is not None:
        idx, dt = hit
        tcp_row = dict(tcp_by_stamp[tcp_stamps[idx]])
        tcp_dt_ms = dt / 1e6
        _write_json(os.path.join(frame_dir, "tcp_pose.json"),
                    dict(tcp_row, dt_sec=dt / 1e9))

    cargo_meta = None
    if point_filter is not None and depth is not None:
        cargo_pts, _obstacle_pts = point_filter.filter_depth(
            depth, label_map, pixel_stride=int(cfg.pixel_stride))
        n_ply = write_ply_xyz(
            os.path.join(frame_dir, "cargo_points.ply"),
            np.asarray(cargo_pts, dtype=np.float64))
        cargo_meta = {"n_points": int(len(cargo_pts)),
                      "ply_vertices": int(n_ply),
                      "frame": "optical(d555_color_optical_frame)"}

    lidar_meta = None
    if cfg.with_lidar:
        hit = nearest_stamp(lidar_stamps, stamp_ns,
                            int(cfg.lidar_tolerance_ms * NS_PER_MS))
        if hit is not None:
            idx, dt = hit
            points = decode_cloud_xyz(lidar_by_stamp[lidar_stamps[idx]])
            if points is not None and len(points):
                np.save(os.path.join(frame_dir, "lidar.npy"),
                        np.asarray(points, dtype=np.float32))
                write_ply_xyz(os.path.join(frame_dir, "lidar.ply"),
                              np.asarray(points, dtype=np.float64))
                lidar_meta = {"n_points": int(len(points)),
                              "dt_sec": dt / 1e9}

    depth_stats = {"zero_px": None, "min_mm_nonzero": None, "max_mm": None}
    if depth is not None:
        raw = np.asarray(depth)
        nonzero = raw[raw > 0]
        depth_stats = {
            "zero_px": int((raw == 0).sum()),
            "min_mm_nonzero": int(nonzero.min()) if nonzero.size else None,
            "max_mm": int(nonzero.max()) if nonzero.size else None,
        }
    meta = {
        "stamp_sec": stamp_sec,
        "color_header_stamp_ns": int(stamp_ns),
        "depth_header_stamp_ns": int(pair.depth_stamp_ns),
        "join_source": pair.source,
        "join_dt_ns": int(pair.dt_ns),
        "backend": backend,
        "inference_ms": inference_ms,
        "detection_count": len(det_rows),
        "cargo_selection": selection_info,
        "depth_stats": depth_stats,
        "joint_dt_ms": joint_dt_ms,
        "tcp_dt_ms": tcp_dt_ms,
        "cargo_points": cargo_meta,
        "lidar": lidar_meta,
    }
    _write_json(os.path.join(frame_dir, "meta.json"), meta)

    row = {
        "dir": name,
        "stamp_sec": stamp_sec,
        "join": pair.source,
        "n_detections": len(det_rows),
        "n_cargo_detections": sum(
            1 for det in det_rows if det["label"] == 2),
        "conf_max": max((det["confidence"] for det in det_rows),
                        default=None),
        "inference_ms": inference_ms,
        "joint_dt_ms": joint_dt_ms,
        "detections": det_rows,
    }
    frame_rows.append(row)
    frames_jsonl.write(json.dumps(
        {k: v for k, v in row.items() if k != "detections"}) + "\n")
    detections_jsonl.write(json.dumps(row) + "\n")
    all_infer_ms.append(inference_ms)


def _make_replay_video(bag_out, frames_root, frame_rows):
    """Overlay quick-look MP4 via ffmpeg concat (demuxer, no re-decode)."""
    list_path = os.path.join(bag_out, "frames.txt")
    rate_probe = 10.0
    if len(frame_rows) >= 2:
        span = frame_rows[-1]["stamp_sec"] - frame_rows[0]["stamp_sec"]
        if span > 0:
            rate_probe = max(1.0, min(30.0, len(frame_rows) / span))
    frame_dur = 1.0 / rate_probe
    with open(list_path, "w", encoding="utf-8") as handle:
        # Absolute paths (concat resolves relatives against the list file,
        # not the cwd) plus explicit per-image durations — without them the
        # demuxer collapses the whole sequence into a couple of frames.
        for row in frame_rows:
            handle.write("file '%s'\n" % os.path.abspath(os.path.join(
                frames_root, row["dir"], "overlay.png")))
            handle.write("duration %.6f\n" % frame_dur)
        if frame_rows:
            handle.write("file '%s'\n" % os.path.abspath(os.path.join(
                frames_root, frame_rows[-1]["dir"], "overlay.png")))
    out_path = os.path.join(bag_out, "replay.mp4")
    rate = 10.0
    if len(frame_rows) >= 2:
        span = frame_rows[-1]["stamp_sec"] - frame_rows[0]["stamp_sec"]
        if span > 0:
            rate = max(1.0, min(30.0, len(frame_rows) / span))
    duration = 1.0 / rate
    import subprocess  # noqa: WPS433 eval dump path only
    subprocess.run(
        ["ffmpeg", "-y", "-loglevel", "error",
         "-f", "concat", "-safe", "0", "-i", list_path,
         "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2,format=yuv420p",
         "-r", "%.3f" % rate, out_path],
        check=False)
    if os.path.isfile(list_path):
        os.remove(list_path)
    return out_path
