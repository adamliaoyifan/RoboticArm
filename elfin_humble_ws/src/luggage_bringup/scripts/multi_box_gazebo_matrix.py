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
from active_loading_bag_harness import (  # noqa: E402
    evaluate_records,
    load_jsonl,
)

# Usable cargo volume / floor from scene_tf (1.49 x 1.97 x 1.48 m).
USABLE_VOLUME_M3 = 1.49 * 1.97 * 1.48
USABLE_FLOOR_M2 = 1.49 * 1.97


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
        "volume_utilization": round(
            result["metrics"]["volume_utilization"], 4),
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
    placed_volume = sum(
        p["size"][0] * p["size"][1] * p["size"][2] for p in placements)
    floor_items = 0
    floor_area = 0.0
    premature_stacks = 0
    for placement in placements:
        footprint = placement.get("footprint") or placement["size"][:2]
        if float(placement.get("peak", 0.0)) <= 1e-3:
            floor_items += 1
            floor_area += float(footprint[0]) * float(footprint[1])
        elif int(placement.get("floor_candidates_available", 0)) > 0:
            premature_stacks += 1
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
        "floor_coverage_ratio": round(floor_area / USABLE_FLOOR_M2, 4),
        "volume_utilization": round(placed_volume / USABLE_VOLUME_M3, 4),
        "placed_volume_m3": round(placed_volume, 4),
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
    mean_utilization = sum(
        row["volume_utilization"] for row in rows) / run_count
    mean_floor_coverage = sum(
        row["floor_coverage_ratio"] for row in rows) / run_count
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
