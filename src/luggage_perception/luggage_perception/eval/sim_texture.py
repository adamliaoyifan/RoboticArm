#!/usr/bin/env python3
"""Eval-only PF-R10 generation-6 trial classification.

Not imported by online nodes. Consumes GT, dump artifacts, and YOLO
proposals after a trial to decide:

- ``SIM_TEXTURE_LOW_CONFIDENCE`` (waivable sim-texture miss);
- ``infrastructure_invalid`` (spawn flip; replace, do not score);
- ordinary pass/fail.

Production confidence floors stay at 0.20. This module never publishes
and never feeds GT into the live detector.
"""

from __future__ import division

import json
import math
import os
import re

from luggage_perception.detection_temporal_gate import bbox_iou
from luggage_perception.eval import gate4_scoring as scoring
from luggage_perception.eval.gate4_scoring import GATE4_LIMITS
from luggage_perception.semantic_segmenter import (
    LABEL_CARGO,
    bbox_is_edge_strip,
)

CLASS_NORMAL_PASS = "normal_pass"
CLASS_TEXTURE = "SIM_TEXTURE_LOW_CONFIDENCE"
CLASS_FAIL = "fail"
CLASS_INFRA = "infrastructure_invalid"

CARGO_MIN_CONFIDENCE = 0.20
TEXTURE_IOU_MIN = 0.50
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
CATALOG_SIZES = ("carryon", "standard", "large")
MAX_TEXTURE_WAIVERS = 2
MIN_NORMAL_PASS = 4

_SPAWN_FLIP_RE = re.compile(r"PLACE_VERIFY_FAILED:.*\btilt=")
_MODEL_RE = re.compile(r"model=(\S+)")

REQUIRED_SNAPSHOT_FILES = (
    "color.png",
    "overlay.png",
    "depth.npy",
    "mask.png",
    "meta.json",
)
REQUIRED_SNAPSHOT_CLOUDS = (
    "cargo_camera.ply",
    "depth_all.ply",
    "mask_cargo.ply",
)
REQUIRED_TRIAL_FILES = (
    "trial.json",
    "scores.jsonl",
    "gz_pose.json",
)


def catalog_size_from_box_id(box_id):
    text = str(box_id or "").lower()
    for name in CATALOG_SIZES:
        if text == name or text.endswith("_" + name):
            return name
    return ""


def spawn_is_flip(message):
    """True when closed-loop placement failed on roll/pitch tilt."""
    return bool(_SPAWN_FLIP_RE.search(str(message or "")))


def model_name_from_spawn_message(message):
    match = _MODEL_RE.search(str(message or ""))
    return match.group(1) if match else ""


def aabb_wholly_inside(bbox, width=IMAGE_WIDTH, height=IMAGE_HEIGHT):
    if bbox is None or len(bbox) < 4:
        return False
    x1, y1, x2, y2 = (float(v) for v in bbox[:4])
    return (
        x1 >= 0.0 and y1 >= 0.0
        and x2 <= float(width) and y2 <= float(height)
        and x2 > x1 and y2 > y1
    )


def aabb_from_span(span):
    if not span:
        return None
    try:
        return [
            float(span["u_min"]), float(span["v_min"]),
            float(span["u_max"]), float(span["v_max"]),
        ]
    except (KeyError, TypeError, ValueError):
        return None


def cargo_detections(detections, cargo_label=LABEL_CARGO):
    out = []
    for det in detections or ():
        try:
            label = int(det.get("label", -1))
        except (TypeError, ValueError):
            continue
        if label == int(cargo_label):
            out.append(det)
    return out


def accepted_cargo(detections, cargo_label=LABEL_CARGO):
    return [d for d in cargo_detections(detections, cargo_label)
            if d.get("accepted")]


def edge_strip_accepted(detections, image_shape=(IMAGE_HEIGHT, IMAGE_WIDTH),
                        cargo_label=LABEL_CARGO):
    h, w = int(image_shape[0]), int(image_shape[1])
    hits = []
    for det in accepted_cargo(detections, cargo_label):
        if bbox_is_edge_strip(det.get("bbox"), w, h):
            hits.append(det)
    return hits


def best_gt_iou_proposal(detections, gt_bbox, cargo_label=LABEL_CARGO,
                         min_iou=TEXTURE_IOU_MIN):
    """Highest-IoU cargo proposal vs the projected GT AABB."""
    best = None
    best_iou = 0.0
    for det in cargo_detections(detections, cargo_label):
        iou = float(bbox_iou(det.get("bbox"), gt_bbox))
        if best is None or iou > best_iou:
            best = det
            best_iou = iou
    if best is None:
        return None
    return {
        "detection": best,
        "iou": best_iou,
        "matches": bool(best_iou >= float(min_iou)),
        "confidence": float(best.get("confidence") or 0.0),
        "below_floor": float(best.get("confidence") or 0.0)
        < float(CARGO_MIN_CONFIDENCE),
    }


def _pair_width_depth(est_w, est_d, gt_w, gt_d):
    est = sorted([abs(float(est_w)), abs(float(est_d))])
    gt = sorted([abs(float(gt_w)), abs(float(gt_d))])
    return abs(est[0] - gt[0]), abs(est[1] - gt[1])


def raw_depth_geometry_ok(ransac, gt, limits=None):
    """Single-snapshot unsegmented-depth replay vs GT top/size limits."""
    limits = limits or GATE4_LIMITS
    reasons = []
    if not ransac or not ransac.get("ok"):
        return False, ["raw_depth_ransac_failed"], {}
    try:
        top_err = abs(float(ransac["plane_z"]) - float(gt["gt_top_z"]))
        xy_err = math.hypot(
            float(ransac["center_xy"][0]) - float(gt["gt_xy"][0]),
            float(ransac["center_xy"][1]) - float(gt["gt_xy"][1]))
        w_err, d_err = _pair_width_depth(
            ransac.get("width"), ransac.get("depth"),
            gt["gt_width"], gt["gt_depth"])
    except (KeyError, TypeError, ValueError, IndexError):
        return False, ["raw_depth_gt_incomplete"], {}
    errors = {
        "err_top_m": top_err,
        "err_xy_m": xy_err,
        "err_width_m": w_err,
        "err_depth_m": d_err,
    }
    if top_err > float(limits["top_z_max_m"]):
        reasons.append("raw_depth_top_z %.4f > %.4f" % (
            top_err, float(limits["top_z_max_m"])))
    if xy_err > float(limits["xy_p95_m"]):
        reasons.append("raw_depth_xy %.4f > %.4f" % (
            xy_err, float(limits["xy_p95_m"])))
    cap = float(limits["width_depth_p95_m"])
    if w_err > cap:
        reasons.append("raw_depth_width %.4f > %.4f" % (w_err, cap))
    if d_err > cap:
        reasons.append("raw_depth_depth %.4f > %.4f" % (d_err, cap))
    return (not reasons), reasons, errors


def dump_capture_health(trial_dir, snapshots=("early", "mid", "late")):
    """File presence for the generation-6 dump matrix. Eval-only."""
    missing = []
    root = str(trial_dir or "")
    if not root or not os.path.isdir(root):
        return {
            "capture_complete": False,
            "replay_possible": False,
            "missing": ["trial_dir"],
        }
    for name in REQUIRED_TRIAL_FILES:
        path = os.path.join(root, name)
        if not os.path.isfile(path) or os.path.getsize(path) <= 0:
            missing.append(name)
    present_snaps = []
    for label in snapshots:
        snap = os.path.join(root, label)
        if not os.path.isdir(snap):
            continue
        present_snaps.append(label)
        for name in REQUIRED_SNAPSHOT_FILES:
            path = os.path.join(snap, name)
            if not os.path.isfile(path) or os.path.getsize(path) <= 0:
                missing.append("%s/%s" % (label, name))
        for name in REQUIRED_SNAPSHOT_CLOUDS:
            path = os.path.join(snap, name)
            if not os.path.isfile(path):
                missing.append("%s/%s" % (label, name))
        replay = os.path.join(snap, "pca_replay", "pca_replay.json")
        if not os.path.isfile(replay) or os.path.getsize(replay) <= 0:
            missing.append("%s/pca_replay/pca_replay.json" % label)
    if "late" not in present_snaps:
        missing.append("late")
    replay_possible = (
        "late" in present_snaps
        and not any(item.endswith("depth.npy") for item in missing)
        and not any(item.endswith("pca_replay.json") for item in missing)
        and not any(item.endswith("meta.json") for item in missing)
    )
    return {
        "capture_complete": not missing,
        "replay_possible": bool(replay_possible and not missing),
        "missing": missing,
        "snapshots": present_snaps,
    }


def write_dump_manifest(trial_dir, payload):
    os.makedirs(trial_dir, exist_ok=True)
    path = os.path.join(trial_dir, "manifest.json")
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True, default=str)
        handle.write("\n")
    return path


def classify_texture_waiver(record):
    """Return (ok, proof) for the six conjunctive waiver conditions."""
    proof = {
        "visual_kind_mesh": False,
        "gt_bbox_in_frame": False,
        "yolo_gt_iou": None,
        "yolo_matches": False,
        "confidence_below_floor": False,
        "raw_depth_ok": False,
        "accepted_cargo_zero": False,
        "cargo_cloud_zero": False,
        "no_edge_fp": False,
        "dump_complete": False,
        "missing": [],
    }
    missing = proof["missing"]
    if str(record.get("visual_kind") or "") != "mesh":
        missing.append("visual_kind_not_mesh")
    else:
        proof["visual_kind_mesh"] = True

    gt_bbox = record.get("gt_bbox")
    image_wh = record.get("image_wh") or (IMAGE_WIDTH, IMAGE_HEIGHT)
    width, height = int(image_wh[0]), int(image_wh[1])
    if aabb_wholly_inside(gt_bbox, width, height):
        proof["gt_bbox_in_frame"] = True
    else:
        missing.append("gt_bbox_not_in_frame")

    match = best_gt_iou_proposal(
        record.get("detections"), gt_bbox,
        min_iou=record.get("min_iou") or TEXTURE_IOU_MIN)
    if match is None:
        missing.append("no_cargo_proposal")
    else:
        proof["yolo_gt_iou"] = match["iou"]
        proof["yolo_matches"] = bool(match["matches"])
        proof["confidence_below_floor"] = bool(match["below_floor"])
        proof["matched_confidence"] = match["confidence"]
        if not match["matches"]:
            missing.append("yolo_gt_iou_below_0.50")
        if not match["below_floor"]:
            missing.append("proposal_not_below_cargo_min_confidence")

    depth_ok, depth_reasons, depth_err = raw_depth_geometry_ok(
        record.get("raw_depth_ransac"), record.get("gt") or {},
        limits=record.get("limits"))
    proof["raw_depth_ok"] = bool(depth_ok)
    proof["raw_depth_errors"] = depth_err
    proof["raw_depth_reasons"] = depth_reasons
    if not depth_ok:
        missing.extend(depth_reasons or ["raw_depth_geometry_fail"])

    n_accepted = int(record.get("n_accepted_cargo")
                     if record.get("n_accepted_cargo") is not None
                     else len(accepted_cargo(record.get("detections"))))
    n_cloud = int(record.get("n_cargo_points") or 0)
    proof["accepted_cargo_zero"] = n_accepted == 0
    proof["cargo_cloud_zero"] = n_cloud == 0
    if n_accepted != 0:
        missing.append("accepted_cargo_not_zero")
    if n_cloud != 0:
        missing.append("cargo_cloud_not_zero")

    strips = edge_strip_accepted(
        record.get("detections"),
        image_shape=(height, width))
    # Also honor an explicit flag from the harness (predicate fail-open).
    if record.get("edge_fp_accepted") or strips:
        missing.append("edge_strip_or_false_cargo_accepted")
    else:
        proof["no_edge_fp"] = True

    health = record.get("dump_health") or {}
    if health.get("capture_complete") and health.get("replay_possible"):
        proof["dump_complete"] = True
    else:
        missing.append("dump_incomplete")
        proof["dump_missing"] = health.get("missing") or ["dump_health_absent"]

    proof["ok"] = not missing
    return proof["ok"], proof


def _image_shape(record):
    image_wh = record.get("image_wh") or (IMAGE_WIDTH, IMAGE_HEIGHT)
    width, height = int(image_wh[0]), int(image_wh[1])
    return height, width


def trial_normal_pass_reasons(record):
    """Reasons this trial is not a normal C1 pass. Empty means pass."""
    reasons = []
    recovery = record.get("recovery") or {}
    if not recovery.get("spawn_ok", True):
        reasons.append("spawn_not_ok")
    t_valid = recovery.get("t_first_valid_sec")
    t_full = recovery.get("t_first_full3d_sec")
    if t_valid is None or float(t_valid) > 1.4:
        reasons.append("t_first_valid %s > 1.4" % t_valid)
    if t_full is None or float(t_full) > 1.4:
        reasons.append("t_first_full3d %s > 1.4" % t_full)
    n_settled = int(recovery.get("n_settled")
                    if recovery.get("n_settled") is not None
                    else len(record.get("settled") or ()))
    if n_settled < 30:
        reasons.append("settled %d < 30" % n_settled)
    settled = list(record.get("settled") or [])
    failed = sum(1 for row in settled if not row.get("top_surface_valid"))
    if failed:
        reasons.append("failed_frames %d" % failed)
    false_h = sum(1 for row in settled if row.get("false_measured_height"))
    if false_h:
        reasons.append("false_measured_height %d" % false_h)
    if record.get("edge_fp_accepted") or edge_strip_accepted(
            record.get("detections"), image_shape=_image_shape(record)):
        reasons.append("edge_fp_accepted")
    if int(record.get("n_cargo_points") or 0) <= 0 and failed:
        # No cloud and failed frames: not a normal pass (maybe texture).
        reasons.append("no_cargo_cloud")
    return reasons


def classify_trial(record):
    """Classify one scored-or-invalid trial. Never looks at production nodes."""
    message = record.get("spawn_message") or ""
    if record.get("infrastructure_invalid") or spawn_is_flip(message):
        return {
            "trial_class": CLASS_INFRA,
            "reasons": ["spawn_flip"],
            "spawn_message": message,
            "model_name": (
                record.get("box_id")
                or model_name_from_spawn_message(message)),
        }
    if record.get("edge_fp_accepted") or edge_strip_accepted(
            record.get("detections"), image_shape=_image_shape(record)):
        return {
            "trial_class": CLASS_FAIL,
            "reasons": ["edge_fp_accepted"],
        }
    health = record.get("dump_health") or {}
    if not health.get("capture_complete"):
        return {
            "trial_class": CLASS_FAIL,
            "reasons": ["missing_dump"] + list(health.get("missing") or []),
        }
    normal_reasons = trial_normal_pass_reasons(record)
    if not normal_reasons:
        return {"trial_class": CLASS_NORMAL_PASS, "reasons": []}
    waived, proof = classify_texture_waiver(record)
    if waived:
        return {
            "trial_class": CLASS_TEXTURE,
            "reasons": [],
            "texture_proof": proof,
        }
    return {
        "trial_class": CLASS_FAIL,
        "reasons": normal_reasons + list(proof.get("missing") or []),
        "texture_proof": proof,
    }


def c1_g6_gate(trials, active_output_hz=None, diagnostic_summary=None):
    """Trial-level C1-G6 gate. Aggregate geometry uses normal-pass trials only.

    ``top_surface_rate`` over all six scored trials is diagnostic when any
    texture waiver is present; it is not a pass/fail input in that case.
    """
    failures = []
    scored = [t for t in trials
              if (t.get("trial_class") or t.get("class")) != CLASS_INFRA]
    invalid = [t for t in trials
               if (t.get("trial_class") or t.get("class")) == CLASS_INFRA]
    if len(scored) != 6:
        failures.append("scored_trials %d != 6" % len(scored))
    normal = [t for t in scored
              if (t.get("trial_class") or t.get("class")) == CLASS_NORMAL_PASS]
    waived = [t for t in scored
              if (t.get("trial_class") or t.get("class")) == CLASS_TEXTURE]
    failed = [t for t in scored
              if (t.get("trial_class") or t.get("class")) == CLASS_FAIL]
    if failed:
        failures.append("failed_trials %s" % [
            t.get("trial") for t in failed])
        for trial in failed:
            failures.extend(
                "trial %s: %s" % (trial.get("trial"), reason)
                for reason in (trial.get("reasons") or ["fail"])[:4])
    if len(waived) > MAX_TEXTURE_WAIVERS:
        failures.append("texture_waivers %d > %d" % (
            len(waived), MAX_TEXTURE_WAIVERS))
    if len(normal) < MIN_NORMAL_PASS:
        failures.append("normal_pass %d < %d" % (
            len(normal), MIN_NORMAL_PASS))
    sizes = {
        catalog_size_from_box_id(t.get("size") or t.get("box_id"))
        for t in normal}
    sizes.discard("")
    missing_sizes = [name for name in CATALOG_SIZES if name not in sizes]
    if missing_sizes:
        failures.append("normal_pass missing sizes %s" % missing_sizes)
    hz = active_output_hz
    if hz is None:
        stamps = []
        for trial in normal:
            stamps.extend(
                row.get("stamp_sec") for row in (trial.get("settled") or [])
                if row.get("stamp_sec") is not None)
        hz = scoring.active_window_hz(stamps)
    if hz is None or float(hz) < 4.0:
        failures.append("active_output_hz %s < 4.0" % hz)
    geometry = None
    if normal:
        geometry = scoring.gate_pass(scoring.aggregate([
            {
                "warmup": t.get("warmup") or [],
                "settled": t.get("settled") or [],
                "instance_id": t.get("instance_id") or t.get("box_id"),
            }
            for t in normal
        ]))
        # Recovery already checked per trial in classify_trial.
        extra = scoring.placement_recovery_gate(
            0,
            [t.get("recovery") or {} for t in normal],
            failed_count=geometry["categories"]["failed"])
        if extra:
            geometry["gate4_pass"] = False
            geometry["gate4_failures"] = (
                list(geometry.get("gate4_failures") or []) + extra)
        if not geometry.get("gate4_pass"):
            failures.extend(geometry.get("gate4_failures") or ["geometry"])
    summary = {
        "c1_g6_pass": not failures,
        "c1_g6_failures": failures,
        "n_scored": len(scored),
        "n_normal_pass": len(normal),
        "n_texture_waiver": len(waived),
        "n_fail": len(failed),
        "n_infrastructure_invalid": len(invalid),
        "normal_sizes": sorted(sizes),
        "active_output_hz": hz,
        "normal_pass_geometry": geometry,
        "diagnostic_top_surface_rate": (
            None if not diagnostic_summary
            else diagnostic_summary.get("top_surface_rate")),
        "trial_classes": [
            {
                "trial": t.get("trial"),
                "box_id": t.get("box_id"),
                "size": catalog_size_from_box_id(
                    t.get("size") or t.get("box_id")),
                "trial_class": t.get("trial_class") or t.get("class"),
                "reasons": t.get("reasons") or [],
            }
            for t in trials
        ],
    }
    return summary
