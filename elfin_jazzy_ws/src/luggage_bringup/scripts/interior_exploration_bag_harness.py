#!/usr/bin/env python3
"""Evaluate recorded interior-exploration contracts and safety metrics.

PR1 of docs/plans/archive/2026-08/urdf_self_filter_task_roi_execution_plan.md: extends the bag
harness with (a) Phase 0 / Phase 1 selection separation, (b) optional
task-cloud-filter record gates, and (c) optional interior-explore loop event
gates. Filter/loop gates are only evaluated when the corresponding records are
supplied, so legacy bags and old unit tests keep passing unchanged.
"""

import argparse
import json
import os
import sys
from collections import Counter

_SCRIPTS = os.path.join(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from interior_explore_loop import LEGAL_TRANSITIONS  # noqa: E402


def _interior_selections(selection_records):
    """Phase 1 interior selections: non-empty lane and positive depth.

    Phase 0 (opening) selections have no lane / zero depth and are excluded
    from corridor-confidence, minimum-depth and same-lane checks.
    """
    return [
        record for record in selection_records
        if record.get("candidate_id")
        and record.get("lane_id")
        and float(record.get("insertion_depth", 0.0)) > 0.0
    ]


def _evaluate_filter_gates(
        filter_records, min_ready_ratio=0.95, max_cloud_age=0.50,
        require=False):
    """Gates over task_cloud_filter stats records.

    Returns ``{}`` (skip, all-pass) when no records are supplied and
    ``require`` is False; returns ``{"task_filter_available": False}`` when
    records are required but absent.
    """
    if not filter_records:
        return ({"task_filter_available": False} if require else {})
    ready = sum(1 for r in filter_records if r.get("ready"))
    ratio = float(ready) / float(len(filter_records)) if filter_records else 0.0
    revisions = [
        r.get("mask_revision") for r in filter_records
        if r.get("mask_revision") is not None]
    models = {
        r.get("robot_model") for r in filter_records
        if r.get("robot_model")}
    versions = {
        r.get("geometry_version") for r in filter_records
        if r.get("geometry_version") is not None}
    epochs = [
        r.get("octomap_epoch") for r in filter_records
        if r.get("octomap_epoch") is not None]
    return {
        "task_filter_available": True,
        "task_filter_ready_ratio": ratio >= min_ready_ratio,
        "robot_model_consistent": len(models) <= 1,
        "geometry_version_consistent": len(versions) <= 1,
        "task_cloud_fresh": all(
            float(r.get("cloud_age", 0.0)) <= max_cloud_age
            for r in filter_records),
        "exact_stamp_tf_complete": all(
            not r.get("tf_missing_links") for r in filter_records),
        "no_unsafe_passthrough": all(
            int(r.get("unsafe_passthrough_count", 0)) == 0
            for r in filter_records),
        "no_robot_overlap_after_filter": all(
            int(r.get("post_filter_robot_overlap_count", 0)) == 0
            for r in filter_records),
        "all_task_points_inside_roi": all(
            int(r.get("roi_outside_count", 0)) == 0 for r in filter_records),
        "filter_revision_monotonic": all(
            c >= p for p, c in zip(revisions, revisions[1:])),
        "explore_uses_container_mode": all(
            r.get("planning_mode") == "EXPLORE_CONTAINER"
            for r in filter_records),
        "octomap_epoch_consistent": all(
            c >= p for p, c in zip(epochs, epochs[1:])),
    }


def _evaluate_loop_gates(loop_records, require=False):
    """Gates over interior-explore loop event records."""
    if not loop_records:
        return ({"loop_events_available": False} if require else {})
    gates = {"loop_events_available": True}

    seqs = [r.get("sequence") for r in loop_records
            if r.get("sequence") is not None]
    gates["loop_sequence_monotonic"] = all(
        c > p for p, c in zip(seqs, seqs[1:]))

    legal = True
    for r in loop_records:
        before, after = r.get("state_before"), r.get("state_after")
        if before and after and before != after:
            if after not in LEGAL_TRANSITIONS.get(before, ()):
                legal = False
    gates["loop_transitions_legal"] = legal

    by_event = {}
    for r in loop_records:
        by_event.setdefault(r.get("event"), []).append(r)

    # enter_started must be preceded by a sequence_validated for the same
    # candidate; entered must have retreat_feasible=true on that validation.
    enter_validated = True
    enter_feasible = True
    for idx, r in enumerate(loop_records):
        if r.get("event") == "enter_started":
            cand = r.get("candidate_id")
            prior_validated = [
                p for p in loop_records[:idx]
                if p.get("event") == "sequence_validated"
                and p.get("candidate_id") == cand]
            if not prior_validated:
                enter_validated = False
            elif not prior_validated[-1].get("retreat_feasible", False):
                enter_feasible = False
    gates["enter_requires_validated_sequence"] = enter_validated
    gates["enter_requires_feasible_retreat"] = enter_feasible

    # inserted failure must request retreat; each retreat_requested must
    # eventually be followed by a retreated.
    inserted_failure_retreats = True
    retreat_complete = True
    for idx, r in enumerate(loop_records):
        if r.get("inserted") and r.get("event") in (
                "motion_failed",) and not any(
                    p.get("event") == "retreat_requested"
                    for p in loop_records[idx + 1:]):
            inserted_failure_retreats = False
        if r.get("event") == "retreat_requested":
            if not any(p.get("event") == "retreated"
                       for p in loop_records[idx + 1:]):
                retreat_complete = False
    gates["inserted_failure_requests_retreat"] = inserted_failure_retreats
    gates["retreat_attempts_complete"] = retreat_complete

    terminals = [r for r in loop_records if r.get("event") == "terminal"]
    gates["terminal_not_inserted"] = all(
        not r.get("inserted") for r in terminals) if terminals else True

    commits = [
        r.get("map_revision") for r in by_event.get("observation_committed", [])
        if r.get("map_revision") is not None]
    gates["loop_map_revision_strict"] = all(
        c > p for p, c in zip(commits, commits[1:]))

    lane_depths = {}
    lane_mono = True
    for r in loop_records:
        # Only candidate_selected proposes a new depth; entered repeats the
        # same candidate's depth and must not be treated as a regression.
        if r.get("event") == "candidate_selected":
            lane = r.get("lane_id")
            depth = float(r.get("insertion_depth", 0.0))
            if lane and depth > 0.0:
                prev = lane_depths.get(lane)
                if prev is not None and depth <= prev:
                    lane_mono = False
                lane_depths[lane] = depth
    gates["loop_same_lane_depth_monotonic"] = lane_mono

    resets = by_event.get("reset", [])
    budgets_respected = True
    if resets:
        budget = resets[-1]
        for r in loop_records:
            if r.get("views", 0) > budget.get("max_views", 1 << 30):
                budgets_respected = False
            if r.get("depth_steps", 0) > budget.get("max_depth_steps", 1 << 30):
                budgets_respected = False
    gates["loop_budgets_respected"] = budgets_respected

    return gates


def evaluate_records(
        opening_records, map_records, selection_records,
        max_geometry_age=0.75, min_corridor_confidence=0.95,
        min_depth=0.20, require_sensed_geometry=True,
        filter_records=None, loop_records=None,
        max_cloud_age=0.50, min_filter_ready_ratio=0.95,
        require_task_filter=False, require_loop_events=False):
    valid_openings = [
        record for record in opening_records if record["valid"]]
    valid_ratio = (
        float(len(valid_openings)) / float(len(opening_records))
        if opening_records else 0.0)
    max_age = max(
        [record["age"] for record in valid_openings] or [float("inf")])
    revisions = [record["map_revision"] for record in map_records]
    revision_monotonic = all(
        current >= previous
        for previous, current in zip(revisions, revisions[1:]))
    all_selections = [
        record for record in selection_records
        if record.get("candidate_id")]
    interior = _interior_selections(selection_records)
    safe_interior = [
        record for record in interior
        if record.get("corridor_free_confidence", 0.0)
        >= min_corridor_confidence]
    max_interior_depth = max(
        [float(r.get("insertion_depth", 0.0)) for r in interior] or [0.0])
    lane_depths = {}
    monotonic_depth = True
    for record in interior:
        lane = record.get("lane_id", "")
        depth = float(record.get("insertion_depth", 0.0))
        previous = lane_depths.get(lane)
        if previous is not None and depth <= previous:
            monotonic_depth = False
        lane_depths[lane] = depth
    rejection_reasons = Counter(
        record.get("diagnostics", "unspecified")
        for record in selection_records
        if not record.get("success", True))
    gates = {
        "opening_available": (
            bool(opening_records) or not require_sensed_geometry),
        "opening_valid_ratio": (
            valid_ratio >= 0.90 or not require_sensed_geometry),
        "geometry_fresh": (
            max_age <= max_geometry_age or not require_sensed_geometry),
        "map_revision_monotonic": revision_monotonic,
        "all_selected_corridors_safe": (
            len(safe_interior) == len(interior)
            or not require_sensed_geometry),
        "minimum_insertion_depth": (
            (not interior) or max_interior_depth >= min_depth),
        "same_lane_depth_monotonic": (
            monotonic_depth or not require_sensed_geometry),
    }
    gates.update(_evaluate_filter_gates(
        filter_records, min_filter_ready_ratio, max_cloud_age,
        require_task_filter))
    gates.update(_evaluate_loop_gates(loop_records, require_loop_events))
    metrics = {
        "opening_samples": len(opening_records),
        "opening_valid_ratio": valid_ratio,
        "max_geometry_age_sec": max_age,
        "map_samples": len(map_records),
        "selection_count": len(all_selections),
        "interior_selection_count": len(interior),
        "safe_selection_count": len(safe_interior),
        "max_insertion_depth_m": max_interior_depth,
        "lanes_used": sorted(lane_depths),
    }
    metrics.update(_floor_coverage_metrics(interior))
    return {
        "passed": all(gates.values()),
        "gates": gates,
        "metrics": metrics,
        "rejection_reasons": dict(sorted(rejection_reasons.items())),
    }


def _floor_coverage_metrics(interior):
    """Summarize observation-only FOV coverage of the container inner floor.

    Deliberately reported outside ``gates``: these numbers describe how much
    of the floor the chosen views actually saw, but no acceptance decision
    depends on them.
    """
    summary = {}
    for field, label in (
            ("floor_xy_coverage", "floor_coverage"),
            ("inside_container_fov_ratio", "inside_container_fov")):
        values = [
            float(record[field]) for record in interior if field in record]
        summary["%s_samples" % label] = len(values)
        summary["mean_%s" % label] = (
            sum(values) / float(len(values)) if values else 0.0)
        summary["min_%s" % label] = min(values) if values else 0.0
        summary["max_%s" % label] = max(values) if values else 0.0
    return summary


def read_bag(path, opening_topic, map_topic, selection_topic,
             filter_topic="/task_cloud_filter/stats_json",
             loop_topic="/luggage/interior_explore_loop/events_json"):
    try:
        import rosbag
    except ImportError as exc:
        raise RuntimeError(
            "rosbag Python bindings are required; run inside the ROS image"
        ) from exc
    opening_records = []
    map_records = []
    selection_records = []
    filter_records = []
    loop_records = []
    topics = [opening_topic, map_topic, selection_topic]
    if filter_topic:
        topics.append(filter_topic)
    if loop_topic:
        topics.append(loop_topic)
    with rosbag.Bag(path, "r") as bag:
        for topic, message, bag_stamp in bag.read_messages(topics=topics):
            if topic == opening_topic:
                sensor_stamp = message.header.stamp.to_sec()
                age = max(0.0, bag_stamp.to_sec() - sensor_stamp)
                opening_records.append({
                    "valid": bool(message.valid),
                    "age": age,
                    "confidence": float(message.confidence),
                    "source": str(message.source),
                    "rejection_reason": str(message.rejection_reason),
                })
            elif topic == map_topic:
                map_records.append(json.loads(message.data))
            elif topic == selection_topic:
                selection_records.append(json.loads(message.data))
            elif topic == filter_topic:
                filter_records.append(json.loads(message.data))
            elif topic == loop_topic:
                loop_records.append(json.loads(message.data))
    return (opening_records, map_records, selection_records,
            filter_records, loop_records)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument(
        "--opening-topic",
        default="/container_opening_estimator/opening_estimate")
    parser.add_argument(
        "--map-topic", default="/cargo_volume_mapper/stats_json")
    parser.add_argument(
        "--selection-topic",
        default="/cargo_exploration_planner/selection_diagnostics")
    parser.add_argument(
        "--filter-topic", default="/task_cloud_filter/stats_json")
    parser.add_argument(
        "--loop-topic",
        default="/luggage/interior_explore_loop/events_json")
    parser.add_argument("--max-geometry-age", type=float, default=0.75)
    parser.add_argument(
        "--min-corridor-confidence", type=float, default=0.95)
    parser.add_argument("--min-depth", type=float, default=0.20)
    parser.add_argument("--max-cloud-age", type=float, default=0.50)
    parser.add_argument(
        "--simulation-fallback",
        action="store_true",
        help="skip sensed-geometry gates for Gazebo config-fallback smoke")
    parser.add_argument(
        "--require-task-filter", action="store_true",
        help="require task-cloud-filter records (hardware acceptance)")
    parser.add_argument(
        "--require-loop-events", action="store_true",
        help="require interior-explore loop event records (hardware)")
    parser.add_argument(
        "--legacy-no-task-filter", action="store_true",
        help="explicitly skip filter gates for legacy bags")
    parser.add_argument(
        "--legacy-no-loop-events", action="store_true",
        help="explicitly skip loop gates for legacy bags")
    args = parser.parse_args()
    opening, maps, selections, filters, loops = read_bag(
        args.bag, args.opening_topic, args.map_topic, args.selection_topic,
        args.filter_topic, args.loop_topic)
    result = evaluate_records(
        opening, maps, selections,
        max_geometry_age=args.max_geometry_age,
        min_corridor_confidence=args.min_corridor_confidence,
        min_depth=args.min_depth,
        max_cloud_age=args.max_cloud_age,
        require_sensed_geometry=not args.simulation_fallback,
        filter_records=(None if args.legacy_no_task_filter else filters),
        loop_records=(None if args.legacy_no_loop_events else loops),
        require_task_filter=args.require_task_filter,
        require_loop_events=args.require_loop_events)
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
