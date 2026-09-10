#!/usr/bin/env python3
"""Run deterministic strict-RGBD active-loading Gazebo matrices.

Acceptance is utilization-driven: each seed draws a random box sequence and
runs until the planner reports NO_CANDIDATE (or the box budget is exhausted),
rather than replaying a fixed ``large -> standard -> carryon`` script. The fixed
script forced a three-layer stack inside a hand-tuned ROI, which is what made
E16R fail on the third box; how many layers a run ends up using is now an
outcome, not an input.

All runs are headless (``gui:=false start_camera_view:=false``); visualization
is left to interactive runs.
"""
from __future__ import division

import argparse
import json
import os
import re
import subprocess
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)
_SRC = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)
from active_loading_bag_harness import (  # noqa: E402
    evaluate_records,
    load_jsonl,
)
from luggage_packing.geometry_metrics import (  # noqa: E402
    GeometryMetricsError,
    capacity_report,
    denominators_from_scene_config,
)


def _cleanup_ros():
    for process in ("roslaunch", "gzserver", "gzclient", "rosmaster"):
        subprocess.run(
            ["pkill", "-TERM", process],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            check=False)
    time.sleep(5.0)


def _run_one(seed, boxes, output_dir, timeout_sec, post_verify,
             log_level="warn"):
    _cleanup_ros()
    os.makedirs(output_dir, exist_ok=True)
    log_path = os.path.join(
        output_dir, "seed_%03d_boxes_%02d.log" % (seed, boxes))
    # Structured record stream. Scraping the console log can only recover the
    # placement records; the harness gates on detections, releases and map
    # commits too, so the run writes them directly.
    events_path = os.path.join(
        output_dir, "seed_%03d_boxes_%02d_events.jsonl" % (seed, boxes))
    command = [
        "roslaunch", "luggage_bringup", "active_loading.launch",
        "orchestrator_required:=true",
        "scene_tf_config:=/catkin_ws/src/luggage_description/config/scene_tf.yaml",
        "max_placed:=%d" % boxes,
        "pickup_random_seed:=%d" % seed,
        "xy_jitter_range:=[0.05,0.05]",
        "strict_perception:=true",
        "allow_gt_fallback:=false",
        "enable_semantic:=false",
        "inspect_mode:=fused",
        "run_initial_explore:=false",
        "exploration_mode:=none",
        "use_placement_planner:=true",
        "use_motion_filter:=true",
        "post_place_verify:=%s" % ("true" if post_verify else "false"),
        "near_roi_enabled:=false",
        "gui:=false",
        "start_camera_view:=false",
        "show_image_views:=false",
        "enable_detect_viz:=false",
        "log_level:=%s" % log_level,
        "run_mode:=auto",
        "events_path:=%s" % events_path,
        "enable_dynamic_scene:=false",
        "enable_octomap:=false",
    ]
    started = time.time()
    timed_out = False
    return_code = None
    with open(log_path, "w") as stream:
        try:
            completed = subprocess.run(
                command, stdout=stream, stderr=subprocess.STDOUT,
                timeout=timeout_sec, check=False)
            return_code = completed.returncode
        except subprocess.TimeoutExpired:
            timed_out = True
            _cleanup_ros()
    _cleanup_ros()
    return _summarize_run(
        seed, boxes, log_path, started, timed_out, return_code,
        events_path=events_path)


def _parse_placements(text):
    """Structured commit records emitted by the orchestrator.

    Uses the ``PLACEMENT_COMMIT {json}`` line rather than scraping free text,
    so the matrix and the bag harness read the same schema.
    """
    placements = []
    for match in re.finditer(r"PLACEMENT_COMMIT (\{.*?\})\s*$", text, re.M):
        try:
            placements.append(json.loads(match.group(1)))
        except ValueError:
            continue
    return placements


def _capacity_from_placements(placements, scene_config=None):
    packed_volume = sum(
        p["size"][0] * p["size"][1] * p["size"][2] for p in placements
        if p.get("size") and len(p["size"]) >= 3)
    boxes = []
    for placement in placements:
        size = placement.get("size")
        if not size or len(size) < 3:
            continue
        box = {
            "size": size,
            "peak": placement.get("peak"),
            "yaw": placement.get("yaw", 0.0),
        }
        if placement.get("container_x") is not None and placement.get(
                "container_y") is not None:
            box["container_x"] = placement["container_x"]
            box["container_y"] = placement["container_y"]
        boxes.append(box)
    if scene_config is None:
        from luggage_description.scene_tf_config_utils import (
            load_scene_tf_config,
            resolve_scene_tf_config_path,
        )
        scene_config = load_scene_tf_config(resolve_scene_tf_config_path())
    hull = denominators_from_scene_config(scene_config)
    report = capacity_report(packed_volume, boxes, hull["geometry"])
    report["packed_volume_m3"] = packed_volume
    return report


def _events_summary(events_path, expected_boxes):
    """Bag-harness verdict for this run, or why it could not be produced."""
    if not events_path or not os.path.isfile(events_path):
        return {"available": False, "reason": "no events file"}
    try:
        result = evaluate_records(
            load_jsonl(events_path), expected_boxes=expected_boxes)
    except (ValueError, IOError) as exc:
        return {"available": False, "reason": "unreadable: %s" % exc}
    return {
        "available": True,
        "passed": result["passed"],
        "rejection_reasons": result["rejection_reasons"],
        "final_placed_count": result["metrics"]["final_placed_count"],
        "floor_items": result["metrics"]["floor_items"],
        "premature_stack_count": result["metrics"]["premature_stack_count"],
        "volume_utilization": (
            None if result["metrics"]["volume_utilization"] is None
            else round(result["metrics"]["volume_utilization"], 4)),
        "failure_classes": result["metrics"]["failure_classes"],
        "path": events_path,
    }


def _summarize_run(seed, boxes, log_path, started, timed_out, return_code,
                   events_path=""):
    with open(log_path, "r", errors="ignore") as stream:
        text = stream.read()
    placed_values = [
        int(value) for value in re.findall(r"placed=(\d+)", text)]
    placed = max(placed_values) if placed_values else 0
    fallback_count = text.count("gt fallback") + text.count(
        "using spawner GT")
    failures = re.findall(
        r"\[Idle\]\s+([^\r\n]+?)\s+\(placed=\d+\)", text)
    placements = _parse_placements(text)
    floor_items = 0
    premature_stacks = 0
    for placement in placements:
        if float(placement.get("peak", 0.0)) <= 1e-3:
            floor_items += 1
        elif int(placement.get("floor_candidates_available", 0)) > 0:
            premature_stacks += 1
    try:
        capacity = _capacity_from_placements(placements)
        volume_utilization = round(capacity["volume_fraction"], 4)
        floor_coverage_ratio = round(capacity["floor_coverage"], 4)
        placed_volume = capacity["packed_volume_m3"]
        geometry_hash = capacity["geometry_hash"]
        schema_version = capacity["schema_version"]
        usable_volume_m3 = capacity["usable_volume_m3"]
        floor_area_m2 = capacity["floor_area_m2"]
        geometry_reason = None
    except GeometryMetricsError as exc:
        volume_utilization = None
        floor_coverage_ratio = None
        placed_volume = sum(
            p["size"][0] * p["size"][1] * p["size"][2]
            for p in placements if p.get("size") and len(p["size"]) >= 3)
        geometry_hash = None
        schema_version = None
        usable_volume_m3 = None
        floor_area_m2 = None
        geometry_reason = exc.reason
    # A run that stops on NO_CANDIDATE has filled what it could reach; that is
    # a valid terminal state, not a failure.
    exhausted = "NO_CANDIDATE" in text
    # Self-check against silent evidence loss. A box cannot be committed
    # without having been detected first, so this combination means a record
    # stopped reaching the log -- e.g. after a console-verbosity change --
    # rather than anything about the run itself.
    perception_estimates = text.count("perception estimate")
    evidence_consistent = not (
        len(placements) > 0 and perception_estimates == 0)
    events = _events_summary(events_path, expected_boxes=boxes)
    # The console log and the events file are two independent renderings of the
    # same run. If they disagree on how many boxes were committed, one of them
    # lost records and neither can be trusted.
    if events.get("available"):
        evidence_consistent = evidence_consistent and (
            events["final_placed_count"] == placed)
    success = (
        fallback_count == 0
        and not timed_out
        and ("max placed reached" in text or exhausted)
    )
    return {
        "seed": seed,
        "box_budget": boxes,
        "placed_count": placed,
        "success": success,
        "capacity_exhausted": exhausted,
        "timed_out": timed_out,
        "return_code": return_code,
        "elapsed_sec": round(time.time() - started, 3),
        "gt_fallback_count": fallback_count,
        "perception_estimate_count": perception_estimates,
        "evidence_consistent": evidence_consistent,
        "commit_count": len(placements),
        "floor_items": floor_items,
        "premature_stack_count": premature_stacks,
        "floor_coverage_ratio": floor_coverage_ratio,
        "volume_utilization": volume_utilization,
        "placed_volume_m3": (
            None if placed_volume is None else round(placed_volume, 4)),
        "usable_volume_m3": usable_volume_m3,
        "floor_area_m2": floor_area_m2,
        "schema_version": schema_version,
        "geometry_hash": geometry_hash,
        "geometry_metrics_reason": geometry_reason,
        "failure_messages": failures[-5:],
        "log_path": log_path,
        "events": events,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seeds", default="0,1,2,3,4,5,6,7,8,9")
    parser.add_argument("--boxes", type=int, default=8,
                        help="box budget per run; the run may stop earlier on "
                             "NO_CANDIDATE, which is a valid terminal state")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--timeout-sec", type=float, default=900.0)
    parser.add_argument("--post-verify", action="store_true")
    parser.add_argument("--min-floor-items", type=int, default=2,
                        help="anti-regression gate: boxes that must land on "
                             "the container floor before stacking")
    parser.add_argument("--min-volume-utilization", type=float, default=0.10)
    parser.add_argument("--log-level", default="warn",
                        choices=["info", "warn"],
                        help="console verbosity; warn keeps every "
                             "acceptance record and drops routine chatter")
    args = parser.parse_args()
    seeds = [int(value) for value in args.seeds.split(",") if value.strip()]
    rows = []
    for seed in seeds:
        row = _run_one(
            seed, args.boxes, args.output_dir,
            args.timeout_sec, args.post_verify, log_level=args.log_level)
        rows.append(row)
        print(json.dumps(row, sort_keys=True), flush=True)
    run_count = max(1, len(rows))
    mean_floor_items = sum(row["floor_items"] for row in rows) / run_count
    util_rows = [row["volume_utilization"] for row in rows
                 if row["volume_utilization"] is not None]
    cov_rows = [row["floor_coverage_ratio"] for row in rows
                if row["floor_coverage_ratio"] is not None]
    mean_utilization = (
        sum(util_rows) / len(util_rows) if util_rows else None)
    mean_floor_coverage = (
        sum(cov_rows) / len(cov_rows) if cov_rows else None)
    result = {
        "schema_version": 2,
        "box_budget": args.boxes,
        "post_verify": args.post_verify,
        "log_level": args.log_level,
        "run_count": len(rows),
        "successful_runs": sum(row["success"] for row in rows),
        "clean_run_rate": (
            float(sum(row["success"] for row in rows)) / run_count),
        "mean_placed_count": sum(
            row["placed_count"] for row in rows) / run_count,
        "mean_floor_items": mean_floor_items,
        "mean_floor_coverage_ratio": mean_floor_coverage,
        "mean_volume_utilization": mean_utilization,
        "gt_fallback_count": sum(row["gt_fallback_count"] for row in rows),
        "runs": rows,
    }
    result["premature_stack_count"] = sum(
        row["premature_stack_count"] for row in rows)
    result["evidence_consistent"] = all(
        row["evidence_consistent"] for row in rows)
    result["events_recorded"] = all(
        row["events"].get("available") for row in rows)
    result["passed"] = (
        result["evidence_consistent"]
        and result["clean_run_rate"] >= 0.80
        and result["gt_fallback_count"] == 0
        and result["premature_stack_count"] == 0
        and mean_floor_items >= float(args.min_floor_items)
        and mean_utilization is not None
        and mean_utilization >= float(args.min_volume_utilization)
    )
    output = os.path.join(args.output_dir, "summary.json")
    with open(output, "w") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
