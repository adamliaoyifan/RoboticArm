#!/usr/bin/env python3
"""Distill bounded ACTIVE-VIEW-1 replay windows from PF-R7 G4 dump evidence.

Reads the immutable G4 bounded-acceptance evidence root (per-trial dump
directories with snapshot ``meta.json`` and per-frame ``scores.jsonl``)
and produces a small committed JSON fixture consumed by
``test_active_view_recovery.py``. No images or point clouds are copied;
only distilled scalars keep the fixture bounded.

Measured-data rules (recorded verbatim in the fixture provenance):

- Window frames: three consecutive ``scores.jsonl`` rows nearest the mid
  snapshot stamp, strictly increasing, spanning <= 0.50 s (real measured
  acquisition stamps).
- Per-frame content (detections, hints, foreground counts, health flags):
  the mid snapshot ``meta.json`` measured values. The three G4 snapshots
  (early/mid/late) carry identical detection content, so associating the
  mid values with the nearby settled window stamps is a declared,
  provenance-recorded mapping.
- Robot settled: ``not prep_status.flags.stale and
  not prep_status.flags.motion_too_large`` (the G4 motion gate ran
  disabled; these flags are the measured motion statement).
- Exact-stamp TF: ``tf_errors`` all null and the depth-join
  ``raw_lookup_status == "hit"``.
- Hint world landing: median valid ``depth_all.xyz`` point (raster,
  stride 2) inside the hint bbox, converted from the dump world frame to
  the base frame with the mean pass-trial rig offset (measured as
  mask_cargo cloud centroid minus filter_stats.centroid on the three
  proposal-available trials; spread recorded).
- Geometry identity token: sha256 of the static rig descriptor (workspace
  centre/half extents + camera model id), constant across trials; map
  revision is the measured perception generation counter.

Harness-only labels (attempt class, best confidence, IoU reference) live
in each session's ``expected`` block and must never enter the policy
input (the policy rejects them fail-closed).
"""

from __future__ import division

import argparse
import hashlib
import json
import math
import os
import sys

DEFAULT_EVIDENCE_ROOT = os.path.join(
    "docs", "status", "evidence", "platform_free_height",
    "2026-09-15_pfr7_g4", "rev_c086a396b974311ce40a6f8a3ab26dc14e9341f2",
    "live")

DEFAULT_OUTPUT = os.path.join(
    "src", "luggage_planning", "test", "data",
    "active_view_pfr7_g4_windows.json")

STANDARD_SEEDS = ("standard_00", "standard_01", "standard_02",
                  "standard_03", "standard_04", "standard_05")

WINDOW_SPAN_SEC = 0.50
XYZ_STRIDE = 2  # deproject stride used by the G4 dump writer


def load_json(path):
    with open(path, "r") as handle:
        return json.load(handle)


def load_jsonl(path):
    rows = []
    with open(path, "r") as handle:
        for line in handle:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def load_xyz_xy(path):
    """Load (x, y) pairs from a whitespace .xyz cloud (pure python)."""
    points = []
    with open(path, "r") as handle:
        for line in handle:
            parts = line.split()
            if len(parts) >= 2:
                try:
                    points.append((float(parts[0]), float(parts[1])))
                except ValueError:
                    continue
    return points


def seed_to_trial_map(evidence_root):
    """Map seed_id -> (attempt row, trial dump dir) from attempts.jsonl."""
    mapping = {}
    for row in load_jsonl(os.path.join(evidence_root, "attempts.jsonl")):
        seed = row.get("seed_id", "")
        if seed not in STANDARD_SEEDS or seed in mapping:
            continue
        slot_dir = row["dump"]
        dump_hint = os.path.join(slot_dir, "dump_dir.txt")
        trial_dir = None
        if os.path.isfile(dump_hint):
            with open(dump_hint, "r") as handle:
                trial_dir = handle.read().strip()
        if not trial_dir or not os.path.isdir(trial_dir):
            # Fall back to the shared dumps/ directory by trial number.
            dumps = os.path.join(evidence_root, "dumps")
            for name in sorted(os.listdir(dumps)):
                if name.startswith("trial_%02d_" % row["attempt"]):
                    trial_dir = os.path.join(dumps, name)
                    break
        if trial_dir and os.path.isdir(trial_dir):
            mapping[seed] = (row, trial_dir)
    return mapping


def select_window_rows(scores_rows, target_stamp):
    """Three consecutive strictly increasing rows nearest ``target_stamp``."""
    best = None
    for idx in range(len(scores_rows) - 2):
        trio = scores_rows[idx:idx + 3]
        stamps = [float(r["stamp_sec"]) for r in trio]
        if not all(stamps[i] < stamps[i + 1] for i in range(2)):
            continue
        if stamps[-1] - stamps[0] > WINDOW_SPAN_SEC:
            continue
        distance = abs(sum(stamps) / 3.0 - target_stamp)
        if best is None or distance < best[0]:
            best = (distance, trio)
    if best is None:
        raise RuntimeError("no 3-row window within %.2fs near %s"
                           % (WINDOW_SPAN_SEC, target_stamp))
    return best[1]


def bbox_landing_base_xy(xyz_path, bbox, image_wh, offset_xy):
    """Median dump-world (x, y) of strided cloud points inside ``bbox``,
    converted to the base frame with the measured rig offset."""
    width, height = image_wh
    cols = width // XYZ_STRIDE
    rows = height // XYZ_STRIDE
    points = load_xyz_xy(xyz_path)
    if len(points) != cols * rows:
        raise RuntimeError(
            "unexpected xyz layout: %d points != %dx%d"
            % (len(points), rows, cols))
    xs, ys = [], []
    for py in range(int(bbox[1]), int(bbox[3]), 4):
        for px in range(int(bbox[0]), int(bbox[2]), 4):
            r, c = py // XYZ_STRIDE, px // XYZ_STRIDE
            if r >= rows or c >= cols:
                continue
            x, y = points[r * cols + c]
            if math.isfinite(x) and math.isfinite(y):
                xs.append(x)
                ys.append(y)
    if not xs:
        return None
    xs.sort()
    ys.sort()
    return [round(xs[len(xs) // 2] - offset_xy[0], 4),
            round(ys[len(ys) // 2] - offset_xy[1], 4)]


def measure_rig_offsets(pass_trials):
    """offset_xy = mask_cargo cloud centroid - filter_stats.centroid."""
    offsets = {}
    for seed, (row, trial_dir) in sorted(pass_trials.items()):
        meta = load_json(os.path.join(trial_dir, "mid", "meta.json"))
        centroid = meta.get("filter_stats", {}).get("centroid")
        xyz_path = os.path.join(trial_dir, "mid", "mask_cargo.xyz")
        if not centroid or not os.path.isfile(xyz_path):
            continue
        points = load_xyz_xy(xyz_path)
        if not points:
            continue
        cx = sum(p[0] for p in points) / len(points)
        cy = sum(p[1] for p in points) / len(points)
        offsets[seed] = [round(cx - centroid[0], 4), round(cy - centroid[1], 4)]
    return offsets


def border_margin_px(bbox, width, height):
    return int(min(bbox[0], bbox[1],
                   (width - 1) - bbox[2], (height - 1) - bbox[3]))


def rig_geometry_hash(workspace_center_xy, workspace_half_xy, camera_model_id):
    descriptor = json.dumps(
        {"workspace_center_xy": list(workspace_center_xy),
         "workspace_half_xy": list(workspace_half_xy),
         "camera_model_id": camera_model_id},
        sort_keys=True)
    return hashlib.sha256(descriptor.encode("utf-8")).hexdigest()[:12]


def distill_session(seed, attempt_row, trial_dir, offset_xy, provenance):
    mid = load_json(os.path.join(trial_dir, "mid", "meta.json"))
    scores = load_jsonl(os.path.join(trial_dir, "scores.jsonl"))
    mid_stamp = float(mid["camera_info"]["stamp"])
    window_rows = select_window_rows(scores, mid_stamp)

    width = int(mid["camera_info"]["width"])
    height = int(mid["camera_info"]["height"])
    frame_id = str(mid.get("cargo_frame_id")
                   or mid["camera_info"].get("frame_id"))
    camera_model_id = "D555-%dx%d" % (width, height)
    geometry_hash = rig_geometry_hash(
        mid["workspace_center_xy"], mid["workspace_half_extents"],
        camera_model_id)

    flags = mid["prep_status"]["flags"]
    stale = bool(flags.get("stale", False))
    motion_too_large = bool(flags.get("motion_too_large", False))
    tf_errors = mid.get("tf_errors") or {}
    lookup = (mid.get("stream_stats", {}).get("raw_lookup") or {})
    exact_tf_ok = (all(value is None for value in tf_errors.values())
                   and lookup.get("raw_lookup_status") == "hit")

    accepted = []
    for det in mid["seg_stats"].get("detections", []) or []:
        centroid = (mid.get("filter_stats") or {}).get("centroid")
        accepted.append({
            "confidence": det["confidence"],
            "bbox": det["bbox"],
            "label": int(det.get("label", 2)),
            "border_margin_px": border_margin_px(det["bbox"], width, height),
            "valid_depth_ratio": None,
            "centre_world_xy": ([round(centroid[0], 4), round(centroid[1], 4)]
                                if centroid else None),
        })

    hints = []
    xyz_path = os.path.join(trial_dir, "mid", "depth_all.xyz")
    for hint in mid.get("eval_low_conf_detections", []) or []:
        landing = (bbox_landing_base_xy(xyz_path, hint["bbox"],
                                        (width, height), offset_xy)
                   if os.path.isfile(xyz_path) else None)
        hints.append({
            "confidence": hint["confidence"],
            "bbox": hint["bbox"],
            "label": int(hint.get("label", 2)),
            "centre_world_xy": landing,
        })

    filter_stats = mid.get("filter_stats") or {}
    depth_count = int(filter_stats.get("depth_count") or 0)
    components = []
    if depth_count > 0:
        components.append({
            "pixel_count": depth_count,
            "area_fraction": round(
                depth_count / float(width * height), 6),
            "height_above_support_m": None,
            "in_workspace": True,
            "bbox": None,
        })

    tail = mid.get("scoring_tail") or {}
    stream = mid.get("stream_stats") or {}
    geometry_level = int(tail.get("geometry_level")
                         or stream.get("geometry_level") or 0)
    support_mode = ("FULL_3D" if geometry_level == 1
                    else str(stream.get("pca_reason") or "NO_TOP"))
    cargo_geometry = {
        "point_count": int(stream.get("n_cargo_points") or 0),
        "top_surface_valid": bool(tail.get("top_surface_valid", False)),
        "support_mode": support_mode,
    }

    frames = []
    for row in window_rows:
        stamp = float(row["stamp_sec"])
        sec = int(stamp)
        frames.append({
            "stamp": {"sec": sec,
                      "nanosec": int(round((stamp - sec) * 1e9))},
            "frame_id": frame_id,
            "camera_model_id": camera_model_id,
            "geometry_hash": geometry_hash,
            "map_revision": int(stream.get("generation") or 0),
            "rgb_ok": bool(flags.get("rgb_ok", False)),
            "depth_ok": bool(flags.get("depth_ok", False)),
            "aligned_depth_ok": bool(flags.get("depth_ok", False)),
            "exact_tf_ok": exact_tf_ok,
            "robot_settled": (not stale) and (not motion_too_large),
            "accepted_detections": accepted,
            "diagnostic_hints": hints,
            "foreground_components": components,
            "cargo_geometry": cargo_geometry,
        })

    attempt_class = attempt_row.get("attempt_class", "")
    expected_reason = ("RECOVERY_SUCCEEDED"
                       if attempt_class == "eligible_pass" else "")
    expected = {
        "harness_only": True,
        "attempt_class": attempt_class,
        "trial": attempt_row.get("attempt"),
        "expected_reason": expected_reason,
        "expected_proposes_view": expected_reason == "",
    }

    return {
        "window": {
            "state": "pre_pick_detect",
            "payload_attached": False,
            "vacuum_commanded": False,
            "cancel_requested": False,
            "image_width": width,
            "image_height": height,
            "frames": frames,
        },
        "expected": expected,
        "session_provenance": {
            "trial_dir": os.path.relpath(trial_dir, provenance["source_root"]),
            "window_stamps_sec": [round(float(r["stamp_sec"]), 6)
                                  for r in window_rows],
            "window_stamp_span_sec": round(
                float(window_rows[-1]["stamp_sec"])
                - float(window_rows[0]["stamp_sec"]), 6),
            "mid_snapshot_stamp_sec": mid_stamp,
        },
    }


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0])
    parser.add_argument("--evidence-root", default=DEFAULT_EVIDENCE_ROOT)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--seeds", nargs="*", default=list(STANDARD_SEEDS))
    args = parser.parse_args(argv)

    evidence_root = os.path.abspath(args.evidence_root)
    if not os.path.isdir(evidence_root):
        sys.exit("evidence root not found: %s" % evidence_root)

    seed_map = seed_to_trial_map(evidence_root)
    missing = [seed for seed in args.seeds if seed not in seed_map]
    if missing:
        sys.exit("seeds missing from attempts.jsonl: %s" % missing)

    pass_trials = {seed: seed_map[seed] for seed in args.seeds
                   if seed_map[seed][0].get("attempt_class") == "eligible_pass"}
    rig_offsets = measure_rig_offsets(pass_trials)
    if not rig_offsets:
        sys.exit("no pass trials available to measure the rig offset")
    mean_offset = [
        round(sum(off[i] for off in rig_offsets.values())
              / len(rig_offsets), 4)
        for i in range(2)
    ]
    spread = [
        round(max(off[i] for off in rig_offsets.values())
              - min(off[i] for off in rig_offsets.values()), 4)
        for i in range(2)
    ]

    provenance = {
        "source_root": evidence_root,
        "source_rule": ("immutable PF-R7 G4 bounded-acceptance evidence "
                        "(rev c086a396b974311ce40a6f8a3ab26dc14e9341f2)"),
        "window_rule": ("three consecutive scores.jsonl rows nearest the "
                        "mid snapshot stamp, strictly increasing, "
                        "span <= 0.50 s (measured stamps)"),
        "content_rule": ("per-frame measured content associated from the "
                         "mid snapshot meta.json; early/mid/late snapshots "
                         "carry identical detection content"),
        "settled_rule": ("not prep_status.flags.stale and not "
                         "prep_status.flags.motion_too_large"),
        "tf_rule": "tf_errors all null and raw_lookup_status == hit",
        "hint_landing_rule": ("median valid depth_all.xyz point (raster, "
                              "stride 2) inside the hint bbox, converted "
                              "dump-world -> base with the mean rig offset"),
        "state_rule": ("campaign detect phase: pre-pick, vacuum off, "
                       "payload not attached"),
        "rig_offset_mean_dump_to_base_xy": mean_offset,
        "rig_offset_per_trial_xy": rig_offsets,
        "rig_offset_spread_xy": spread,
    }

    sessions = {}
    for seed in args.seeds:
        row, trial_dir = seed_map[seed]
        sessions[seed] = distill_session(
            seed, row, trial_dir, mean_offset, provenance)

    fixture = {
        "schema_id": "active-view-pfr7-g4-window-v1",
        "policy_id": "active-view-recovery-v1",
        "provenance": provenance,
        "sessions": sessions,
    }
    out_dir = os.path.dirname(os.path.abspath(args.output))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.output, "w") as handle:
        json.dump(fixture, handle, indent=1, sort_keys=True)
        handle.write("\n")

    size_kb = os.path.getsize(args.output) / 1024.0
    print("wrote %s (%.1f KiB, %d sessions)" % (args.output, size_kb,
                                                len(sessions)))
    for seed in args.seeds:
        session = sessions[seed]
        window = session["window"]
        hint_count = [len(f["diagnostic_hints"])
                      for f in window["frames"]]
        det_count = [len(f["accepted_detections"])
                     for f in window["frames"]]
        print("  %s: class=%s hints=%s dets=%s geom=%s span=%.3fs"
              % (seed, session["expected"]["attempt_class"], hint_count,
                 det_count,
                 window["frames"][0]["cargo_geometry"]["support_mode"],
                 session["session_provenance"]["window_stamp_span_sec"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
