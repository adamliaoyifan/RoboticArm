#!/usr/bin/env python3
"""Offline packing replay evaluator (P0-a) -- B0 / B2 / B3 comparison.

Generates fixed box sequences (seeds 0..N-1, length L), replays each through
three strategies, and prints the §9.2 metrics side by side:

  B0 = bin_packer DBLF first-feasible + atlas gate (current system).
  B2 = FreeSpaceModel (floor-prior + LBCP + vectorized) + atlas gate + proxy.
  B3 = B2 + rollout V̂ on top-K reachable candidates (§5.7).

Runs the real atlas and, optionally, a legacy floor-unlocked what-if. E12
corrected the container floor to slab-top Z~0.53; the what-if is retained only
for historical comparison.

Pure Python, no ROS / no Gazebo. See design §9.1/§9.3.

Usage:
    python3 packing_replay_eval.py [--seeds 20] [--length 60] [--b3] [--quick]
"""

from __future__ import division

import argparse
import os
import sys
import time
import yaml

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(PKG_ROOT, "scripts")
HARNESS = os.path.join(PKG_ROOT, "test", "harness")
SRC_ROOT = os.path.dirname(PKG_ROOT)
for _p in (SCRIPTS, HARNESS, SRC_ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from packing_replay import (  # noqa: E402
    BinPackerStrategy,
    ReachabilityAtlas,
    ReplaySimulator,
    FreeSpaceModelStrategy,
    StrategySimulator,
    summarize,
    unlock_floor_atlas,
)
from packing_sequences import generate_sequences, catalog_entries, load_catalog  # noqa: E402
from luggage_description.container_geometry import (  # noqa: E402
    descriptor_from_scene_config,
)
from luggage_description.scene_tf_config_utils import (  # noqa: E402
    load_scene_tf_config,
    resolve_scene_tf_config_path,
)
from luggage_packing.geometry_metrics import (  # noqa: E402
    annotate_replay_runs,
    denominators,
)

_DEFAULT_ATLAS = os.path.normpath(os.path.join(
    PKG_ROOT, "..", "luggage_planning", "data", "reachability_atlas",
    "s20_container_collision_aware.npz"))


def _run_b0(atlas, sequences):
    strat = BinPackerStrategy(atlas.inner_size)
    sim = ReplaySimulator(strat, atlas=atlas, retry=False)
    return [sim.run(seq) for _, seq in sorted(sequences.items())]


def _run_fsm(atlas, entries, sequences, mode, **kw):
    strat = FreeSpaceModelStrategy(
        atlas.inner_size, atlas.center_base, atlas.yaw, resolution=0.05,
        atlas=atlas, entries=entries, mode=mode, **kw)
    sim = StrategySimulator(strat, atlas=atlas)
    return [sim.run(seq) for _, seq in sorted(sequences.items())]


def _fmt(s):
    g5 = s.get("volume_fraction") or {}
    return (
        "rfr mean=%.4f (legacy_non_authoritative) | items=%.1f "
        "| floor_items=%.1f floor_cov=%.3f (legacy_non_authoritative) "
        "| G5 volume_fraction=%.4f" % (
            s["reachable_fill_rate_mean"],
            s["items_placed_mean"], s.get("floor_items_mean", 0.0),
            s.get("floor_coverage_mean", 0.0),
            g5.get("volume_fraction", 0.0)))


def _annotate(summary, runs, descriptor):
    g5 = annotate_replay_runs(runs, descriptor)
    out = dict(summary)
    out["volume_fraction"] = g5["volume_fraction"]
    out["legacy_non_authoritative"] = g5["legacy_non_authoritative"]
    out["schema_version"] = g5["schema_version"]
    out["geometry_hash"] = g5["geometry_hash"]
    out["usable_volume_m3"] = g5["usable_volume_m3"]
    return out


def _run_all(atlas, entries, sequences, run_b3, label, descriptor,
             w_floor_first=None):
    print("=" * 96)
    print(label)
    print("-" * 96)
    t0 = time.time()
    b0_runs = _run_b0(atlas, sequences)
    s0 = _annotate(summarize(b0_runs), b0_runs, descriptor)
    print("B0 (bin_packer DBLF):       %s  [%.1fs]" % (_fmt(s0), time.time() - t0))
    t0 = time.time()
    b2_runs = _run_fsm(atlas, entries, sequences, mode="b2")
    s2 = _annotate(summarize(b2_runs), b2_runs, descriptor)
    print("B2 (FreeSpaceModel+proxy):  %s  [%.1fs]" % (_fmt(s2), time.time() - t0))
    t0 = time.time()
    b4_runs = _run_fsm(atlas, entries, sequences, mode="b4",
                       w_floor_first=w_floor_first)
    s4 = _annotate(summarize(b4_runs), b4_runs, descriptor)
    print("B4 (production scorer):     %s  [%.1fs]" % (_fmt(s4), time.time() - t0))
    s3 = None
    if run_b3:
        t0 = time.time()
        b3_runs = _run_fsm(atlas, entries, sequences, mode="b3",
                           top_k_rollout=5, rollout_K=3, rollout_M=8)
        s3 = _annotate(summarize(b3_runs), b3_runs, descriptor)
        print("B3 (B2 + rollout Vhat):     %s  [%.1fs]" % (_fmt(s3), time.time() - t0))
        print("  B3 - B2 (rollout lift):   %+.4f" % (
            s3["reachable_fill_rate_mean"] - s2["reachable_fill_rate_mean"]))
    print("  B2 - B0 (algorithm lift): %+.4f" % (
        s2["reachable_fill_rate_mean"] - s0["reachable_fill_rate_mean"]))
    print("  B4 - B2 (floor-first):    %+.4f rfr, %+.1f floor items" % (
        s4["reachable_fill_rate_mean"] - s2["reachable_fill_rate_mean"],
        s4.get("floor_items_mean", 0.0) - s2.get("floor_items_mean", 0.0)))
    return {"label": label, "b0": s0, "b2": s2, "b3": s3, "b4": s4}


def main():
    p = argparse.ArgumentParser(description="Offline packing replay eval (B0/B2/B3).")
    p.add_argument("--atlas", default=_DEFAULT_ATLAS)
    p.add_argument("--seeds", type=int, default=20)
    p.add_argument("--length", type=int, default=60)
    p.add_argument("--b3", action="store_true", help="also run B3 (rollout, slower)")
    p.add_argument("--quick", action="store_true", help="fewer seeds/length for a fast check")
    p.add_argument("--no-unlock", action="store_true",
                   help="skip the floor-unlocked what-if (only run the real atlas)")
    p.add_argument("--unlock-ix", type=int, default=5,
                   help="front-floor X depth (cells) marked reachable in the what-if")
    p.add_argument("--unlock-iz", type=int, default=1,
                   help="floor Z layers marked reachable in the what-if")
    p.add_argument("--output", default="",
                   help="optional YAML path for summary output")
    p.add_argument("--w-floor-first", type=float, default=None,
                   help="override the B4 floor-first weight")
    args = p.parse_args()

    if args.quick:
        args.seeds = min(args.seeds, 5)
        args.length = min(args.length, 20)

    atlas = ReachabilityAtlas(args.atlas)
    entries = catalog_entries(load_catalog())
    sequences = generate_sequences(range(args.seeds), args.length)
    descriptor = descriptor_from_scene_config(
        load_scene_tf_config(resolve_scene_tf_config_path()))
    hull = denominators(descriptor)

    print("atlas: %s" % args.atlas)
    print("container exact hull: V=%.9f m^3  floor=%.9f m^2  hash=%s" % (
        hull["usable_volume_m3"], hull["floor_area_m2"],
        descriptor.geometry_hash[:12]))
    print("sequences: %d (length %d)" % (args.seeds, args.length))

    scenarios = [_run_all(
        atlas, entries, sequences, args.b3,
        "REAL atlas (configured slab-top floor)",
        descriptor,
        w_floor_first=args.w_floor_first)]

    if args.no_unlock:
        if args.output:
            with open(args.output, "w") as stream:
                yaml.safe_dump({
                    "schema_version": 1,
                    "atlas": args.atlas,
                    "seeds": args.seeds,
                    "length": args.length,
                    "geometry_hash": descriptor.geometry_hash,
                    "usable_volume_m3": hull["usable_volume_m3"],
                    "scenarios": scenarios,
                }, stream, default_flow_style=False, sort_keys=False)
        return

    print("")
    unlocked = unlock_floor_atlas(atlas, max_iz=args.unlock_iz, max_ix=args.unlock_ix)
    scenarios.append(_run_all(
        unlocked, entries, sequences, args.b3,
        "FLOOR-UNLOCKED historical what-if (ix<=%d, iz<=%d reachable)"
        % (args.unlock_ix, args.unlock_iz),
        descriptor,
        w_floor_first=args.w_floor_first))
    if args.output:
        with open(args.output, "w") as stream:
            yaml.safe_dump({
                "schema_version": 1,
                "atlas": args.atlas,
                "seeds": args.seeds,
                "length": args.length,
                "geometry_hash": descriptor.geometry_hash,
                "usable_volume_m3": hull["usable_volume_m3"],
                "scenarios": scenarios,
            }, stream, default_flow_style=False, sort_keys=False)


if __name__ == "__main__":
    main()
