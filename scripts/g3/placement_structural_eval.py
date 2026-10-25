#!/usr/bin/env python3
"""STRUCTURAL-CORE offline evaluation: same-input old/new comparison.

``compare`` runs, on identical maps (real 0/5/20-box loads at
0.05/0.02/0.01 m), the OLD exhaustive chain (``solve_placement``) and
the NEW structural-first chain (structural proposals + cheap prefilter
+ exact gates + snug ranking + lazy grid fallback) and reports:

- correctness: structural feasible set has zero false-legals vs the
  exhaustive chain; a FULL fallback traversal reproduces the exhaustive
  feasible ID set exactly;
- computation: per-stage timings, candidate counts, prefilter savings,
  structural hit rate (answer without touching the grid), first-answer
  time vs the exhaustive time, and the missed-by-structural count
  (structural search is an ORDER change, not completeness -- missed
  legal positions stay reachable only via the fallback).

No simulation, no ROS; single process, one cold run per combination.
"""

import argparse
import json
import math
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_WS_SRC = os.path.normpath(os.path.join(_HERE, "..", ".."))
sys.path.insert(0, os.path.join(_WS_SRC, "src", "luggage_packing"))
sys.path.insert(0, os.path.join(_WS_SRC, "src", "luggage_planning"))
sys.path.insert(0, os.path.join(_WS_SRC, "src", "luggage_description"))
sys.path.insert(0, _WS_SRC)

from luggage_description.container_geometry import descriptor_from_scene_config  # noqa: E402
from luggage_packing.placement_solver import (  # noqa: E402
    _default_params,
    _depth_datum,
    _surface_hull,
    generate_candidates,
    solve_placement)
from luggage_packing.placement_structural_candidates import (  # noqa: E402
    evaluate_window,
    fallback_batch,
    generate_proposals,
    structural_search)
from luggage_packing.placement_support_index import SupportIndex  # noqa: E402

import yaml  # noqa: E402

BOX = (0.44, 0.33, 0.60)
YAWS = [0.0, math.pi / 2]

_SCENE = os.path.join(_WS_SRC, "src", "luggage_description", "config",
                      "scene_tf.yaml")
with open(_SCENE, "r", encoding="utf-8") as _handle:
    SCENE = yaml.safe_load(_handle)
DEPLOY_DESCRIPTOR = descriptor_from_scene_config(SCENE).descriptor()


def load_map(res, boxes, box_h=0.30):
    inner = SCENE["container"]["inner"]
    inner_size = (float(inner["length"]), float(inner["width"]),
                  float(inner["ceiling_z"]) - float(inner["floor_z"]))
    floor_z = float(inner["floor_z"])
    nx = max(1, int(round(inner_size[0] / res)))
    ny = max(1, int(round(inner_size[1] / res)))
    surface = {
        "resolution": res, "nx": nx, "ny": ny,
        "inner_size": list(inner_size), "floor_z": 0.0,
        "center_base": [0.0, 0.0, floor_z + 0.5 * inner_size[2]],
        "yaw": 0.0,
        "height": [[0.0] * ny for _ in range(nx)],
        "state": [["sensor"] * ny for _ in range(nx)],
        "confidence": [["none"] * ny for _ in range(nx)],
        "geometry_descriptor": dict(DEPLOY_DESCRIPTOR),
        "geometry_hash": str(DEPLOY_DESCRIPTOR.get("geometry_hash") or ""),
    }
    placed = 0
    layer = 0
    while placed < boxes and layer < 2:
        quota = min(15, boxes - placed) if layer == 0 else boxes - placed
        for row in range(5):
            for col in range(3):
                if placed >= boxes or quota <= 0:
                    break
                cx = -0.5 + col * 0.45
                cy = -0.7 + row * 0.35
                ix0 = int((cx + inner_size[0] * 0.5) / res)
                iy0 = int((cy + inner_size[1] * 0.5) / res)
                sx = max(1, int(0.40 / res))
                sy = max(1, int(0.30 / res))
                z = layer * box_h + box_h
                for ix in range(ix0, min(ix0 + sx, nx)):
                    for iy in range(iy0, min(iy0 + sy, ny)):
                        surface["height"][ix][iy] = z
                placed += 1
                quota -= 1
        layer += 1
    return surface, placed


def full_traversal_ids(index, cfg, hull, datum):
    seen = set()
    feasible = set()
    cursor = None
    for _ in range(100000):
        batch = fallback_batch(
            index, cfg, hull, datum, seen, cursor=cursor,
            batch_size=10 ** 9, budget_s=3600.0)
        cursor = batch["cursor"]
        feasible.update(c["candidate_id"] for c in batch["candidates"]
                        if c["feasible"])
        if batch["coverage_complete"]:
            return feasible, batch
    raise RuntimeError("fallback never completed")


def compare(resolutions, boxes_list, out_dir, extra_cases=True):
    rows = []
    cases = [(res, boxes, 0.30)
             for res in resolutions for boxes in boxes_list]
    if extra_cases:
        # Equal-height neighbours: the only load where the candidate's
        # predicted top can match a measured neighbour top, so the snug
        # term A is demonstrably nonzero (0.30-high neighbours never
        # match a 0.60-high candidate by construction).
        cases.append((0.05, 5, 0.60))
    for res, boxes, box_h in cases:
            surface, placed = load_map(res, boxes, box_h=box_h)
            assert placed == boxes, (placed, boxes)

            # OLD: exhaustive chain (enumeration + gates + frozen sort).
            t_old = time.perf_counter()
            old = generate_candidates(
                surface, list(BOX), allowed_yaws=YAWS,
                params={"top_n": 10 ** 9, "keep_rejected": 10 ** 9})
            old_time = time.perf_counter() - t_old
            old_feasible = {c["candidate_id"] for c in old if c["feasible"]}

            # NEW: structural-first chain (to first ranked answer).
            t_new = time.perf_counter()
            new = structural_search(surface, list(BOX), YAWS)
            new_time = time.perf_counter() - t_new
            diag = new["diagnostics"]
            new_structural_feasible = sorted(
                c["candidate_id"] for c in new["structural_candidates"]
                if c["feasible"])
            false_legal = sorted(
                set(new_structural_feasible) - old_feasible)
            missed = sorted(old_feasible - set(new_structural_feasible))

            # Oracle: full fallback traversal == exhaustive feasible set.
            cfg = _default_params()
            index = SupportIndex(surface, BOX, YAWS, cfg)
            hull = _surface_hull(surface)
            datum = _depth_datum(surface, hull)
            t_trav = time.perf_counter()
            got, _batch = full_traversal_ids(index, cfg, hull, datum)
            traversal_time = time.perf_counter() - t_trav
            set_equal = (got == old_feasible)

            row = {
                "res": res,
                "boxes": boxes,
                "placed_box_h": box_h,
                "old": {
                    "n_candidates": len(old),
                    "n_feasible": len(old_feasible),
                    "time_s": round(old_time, 3),
                },
                "new": {
                    "time_s": round(new_time, 3),
                    "stages": diag["stages"],
                    "proposals": diag["proposals"],
                    "structural_evaluated": diag["structural_evaluated"],
                    "prefilter_rejected": diag["prefilter_rejected"],
                    "structural_feasible": len(new_structural_feasible),
                    "fallback_used": new["fallback"] is not None,
                    "fallback_budget_exhausted": (
                        new["fallback"] or {}).get("budget_exhausted"),
                    "chosen": (new["selected"] or {}).get("candidate_id"),
                    "chosen_adjacency_a": (new["selected"] or {}).get(
                        "adjacency_a"),
                },
                "correctness": {
                    "false_legal": len(false_legal),
                    "full_traversal_set_equal": set_equal,
                    "missed_by_structural": len(missed),
                },
                "full_traversal_time_s": round(traversal_time, 3),
            }
            rows.append(row)
            print(json.dumps(row))
            if false_legal:
                print("FALSE-LEGAL detail:", false_legal[:5])
            if not set_equal:
                only_new = sorted(got - old_feasible)[:5]
                only_old = sorted(old_feasible - got)[:5]
                print("SET MISMATCH only-new:", only_new,
                      "only-old:", only_old)
    ok = (all(r["correctness"]["false_legal"] == 0 for r in rows)
          and all(r["correctness"]["full_traversal_set_equal"]
                  for r in rows))
    summary = {
        "rows": rows,
        "all_oracles_pass": ok,
        "interpretation": (
            "structural-first returns a ranked legal answer after "
            "evaluating only the structural subset; the exhaustive "
            "grid remains reachable via the lazy fallback (its full "
            "traversal cost is the old chain cost by construction)"),
    }
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "compare.json"), "w") as handle:
        json.dump(summary, handle, indent=1)
    print("all_oracles_pass:", ok)
    return 0 if ok else 3


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("subcommand", choices=["compare"])
    parser.add_argument("--resolutions", default="0.05,0.02,0.01")
    parser.add_argument("--boxes", default="0,5,20")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    resolutions = [float(v) for v in args.resolutions.split(",")]
    boxes_list = [int(v) for v in args.boxes.split(",")]
    if args.subcommand == "compare":
        return compare(resolutions, boxes_list, args.out)
    return 2


if __name__ == "__main__":
    sys.exit(main())
