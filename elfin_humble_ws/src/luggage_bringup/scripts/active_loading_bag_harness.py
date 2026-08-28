#!/usr/bin/env python3
"""Evaluate structured active-loading records for multi-box hard gates.

Acceptance is utilization-based rather than a fixed stacking script. The
previous form baked ``container_x <= 0.10`` into the gates, which is the same
hand-tuned near-ROI constant that limited usable floor area to 38% of the
container and forced stacking from the second box; reachability is now decided
by the atlas plus the MoveIt filter, so the gate here checks that no committed
placement was atlas-unreachable instead of clamping depth.
"""
from __future__ import division

import argparse
import json
from collections import Counter

# Usable cargo volume from scene_tf (1.49 x 1.97 x 1.48 m).
DEFAULT_USABLE_VOLUME_M3 = 4.344
DEFAULT_USABLE_FLOOR_M2 = 1.49 * 1.97
# Union of atlas REACHABLE/MARGINAL cells; see reachability_atlas.stats().
DEFAULT_REACHABLE_VOLUME_M3 = 0.804
_FLOOR_PEAK_TOL = 1e-3
ATLAS_UNREACHABLE = 1


def _placement_volume(record):
    size = record.get("size")
    if not size or len(size) < 3:
        return 0.0
    return float(size[0]) * float(size[1]) * float(size[2])


def _placement_footprint_area(record):
    footprint = record.get("footprint") or (record.get("size") or [])[:2]
    if not footprint or len(footprint) < 2:
        return 0.0
    return float(footprint[0]) * float(footprint[1])


def evaluate_records(records, expected_boxes=3, max_cycle_sec=120.0,
                     min_floor_items=2,
                     usable_volume_m3=DEFAULT_USABLE_VOLUME_M3,
                     usable_floor_m2=DEFAULT_USABLE_FLOOR_M2,
                     reachable_volume_m3=DEFAULT_REACHABLE_VOLUME_M3):
    by_kind = {}
    for record in records:
        by_kind.setdefault(record.get("kind", "unknown"), []).append(record)
    statuses = by_kind.get("status", [])
    detections = by_kind.get("detection", [])
    maps = by_kind.get("map", [])
    placements = by_kind.get("placement", [])
    releases = by_kind.get("release", [])
    failures = by_kind.get("failure", [])
    # A session that took out-of-band service calls (the GUI probe page) is
    # no longer a record of what the pipeline does on its own, so it cannot be
    # presented as acceptance evidence no matter how good its numbers look.
    sessions = by_kind.get("session", [])
    taints = by_kind.get("taint", [])
    tainted = bool(taints) or any(s.get("probe_touched") for s in sessions)

    placed = [int(r.get("placed_count", 0)) for r in statuses]
    final_placed = max(placed) if placed else 0
    revisions = [
        int(r["map_revision"]) for r in maps
        if r.get("map_revision") is not None]
    committed_revisions = [
        int(r["map_revision"]) for r in maps
        if r.get("event") == "commit" and r.get("map_revision") is not None]
    atlas_rejected = [
        r for r in placements
        if int(r.get("atlas_status", -1)) == ATLAS_UNREACHABLE]
    stale = [r for r in placements if r.get("stale_revision")]
    floor_items = [
        r for r in placements
        if float(r.get("peak", 0.0)) <= _FLOOR_PEAK_TOL]
    # Premature stacking: a box went on top while the planner still reported a
    # usable floor candidate. Only checked when the run logged that field.
    premature_stacks = [
        r for r in placements
        if float(r.get("peak", 0.0)) > _FLOOR_PEAK_TOL
        and int(r.get("floor_candidates_available", 0)) > 0]
    stacking_observable = any(
        "floor_candidates_available" in r for r in placements)
    pose_verified = [r for r in placements if r.get("pose_gate_passed")]
    placed_volume = sum(_placement_volume(r) for r in placements)
    floor_area_used = sum(_placement_footprint_area(r) for r in floor_items)
    detect_success = [r for r in detections if r.get("success")]
    fallbacks = [
        r for r in detections if r.get("source") == "gt_fallback"]
    released_before_retreat = all(
        r.get("released_at_contact") and r.get("retreat_after_release")
        for r in releases)
    cycles = [
        float(r["cycle_sec"]) for r in placements
        if r.get("cycle_sec") is not None]

    gates = {
        "expected_placed_count": final_placed >= expected_boxes,
        "placed_count_monotonic": all(
            cur >= prev for prev, cur in zip(placed, placed[1:])),
        "strict_detection_available": len(detect_success) >= expected_boxes,
        "zero_gt_fallback": not fallbacks,
        "map_revision_monotonic": all(
            cur >= prev for prev, cur in zip(revisions, revisions[1:])),
        "commit_revision_strict": all(
            cur > prev
            for prev, cur in zip(
                committed_revisions, committed_revisions[1:])),
        "release_before_retreat": (
            len(releases) >= expected_boxes and released_before_retreat),
        "zero_atlas_unreachable_commits": not atlas_rejected,
        "zero_stale_candidates": not stale,
        "zero_failures": not failures,
        "cycle_budget": (
            not cycles or max(cycles) <= float(max_cycle_sec)),
        "floor_layer_used": len(floor_items) >= int(min_floor_items),
        "no_premature_stacking": (
            not stacking_observable or not premature_stacks),
        "untainted_session": not tainted,
    }
    failed = sorted(name for name, passed in gates.items() if not passed)
    metrics = {
        "expected_boxes": expected_boxes,
        "final_placed_count": final_placed,
        "detection_count": len(detections),
        "detection_success_count": len(detect_success),
        "gt_fallback_count": len(fallbacks),
        "placement_count": len(placements),
        "release_count": len(releases),
        "map_revision_count": len(revisions),
        "atlas_unreachable_commit_count": len(atlas_rejected),
        "floor_items": len(floor_items),
        "premature_stack_count": len(premature_stacks),
        "floor_coverage_ratio": (
            floor_area_used / usable_floor_m2 if usable_floor_m2 > 0 else 0.0),
        "volume_utilization": (
            placed_volume / usable_volume_m3 if usable_volume_m3 > 0 else 0.0),
        "reachable_fill_rate": (
            placed_volume / reachable_volume_m3
            if reachable_volume_m3 > 0 else 0.0),
        "placed_volume_m3": placed_volume,
        "per_box_physical_pass_rate": (
            len(pose_verified) / len(placements) if placements else 0.0),
        "failure_classes": dict(Counter(
            r.get("failure_class", "UNKNOWN") for r in failures)),
        "max_cycle_sec": max(cycles) if cycles else None,
        "taint_count": len(taints),
        "taint_reasons": [str(r.get("reason", "")) for r in taints],
    }
    return {
        "passed": not failed,
        "gates": gates,
        "metrics": metrics,
        "rejection_reasons": failed,
    }


def load_jsonl(path):
    records = []
    with open(path, "r") as stream:
        for line in stream:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("records_jsonl")
    parser.add_argument("--expected-boxes", type=int, default=3)
    parser.add_argument("--max-cycle-sec", type=float, default=120.0)
    parser.add_argument("--min-floor-items", type=int, default=2,
                        help="anti-regression gate: boxes that must land on "
                             "the container floor before stacking is allowed")
    parser.add_argument("--usable-volume-m3", type=float,
                        default=DEFAULT_USABLE_VOLUME_M3)
    parser.add_argument("--usable-floor-m2", type=float,
                        default=DEFAULT_USABLE_FLOOR_M2)
    parser.add_argument("--reachable-volume-m3", type=float,
                        default=DEFAULT_REACHABLE_VOLUME_M3)
    parser.add_argument("--output", default="")
    args = parser.parse_args()
    result = evaluate_records(
        load_jsonl(args.records_jsonl),
        expected_boxes=args.expected_boxes,
        max_cycle_sec=args.max_cycle_sec,
        min_floor_items=args.min_floor_items,
        usable_volume_m3=args.usable_volume_m3,
        usable_floor_m2=args.usable_floor_m2,
        reachable_volume_m3=args.reachable_volume_m3,
    )
    text = json.dumps(result, indent=2, sort_keys=True)
    if args.output:
        with open(args.output, "w") as stream:
            stream.write(text + "\n")
    print(text)
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
