#!/usr/bin/env python3
"""Reproducible ablations for the online placement scorer.

The floor-first weight and the atlas prior mapping were both chosen from sweeps
that originally lived only in a shell heredoc. Any number that ends up in a plan
or as a code default has to be re-derivable, so those sweeps live here and emit
``ablations.yaml`` for the phase evidence bundle.

Three studies:

  ``weight``    -- w_floor_first sweep. Establishes where the curve plateaus.
  ``prior``     -- atlas prior mapping x weight grid. Shows that grading
                   MARGINAL below REACHABLE costs placed items, which is why
                   the shipped mapping flattens the top two statuses.
  ``isolation`` -- forces a constant prior to separate the prior's effect from
                   the deterministic tie-break, so a regression is attributed to
                   the right change.

Pure Python + numpy, no ROS. Usage:

    python3 packing_score_ablation.py --output ablations.yaml
    python3 packing_score_ablation.py --study weight --seeds 5 --length 20
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
for _path in (SCRIPTS, HARNESS):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from packing_replay import (  # noqa: E402
    FreeSpaceModelStrategy,
    ReachabilityAtlas,
    StrategySimulator,
    summarize,
)
from packing_sequences import (  # noqa: E402
    catalog_entries,
    generate_sequences,
    load_catalog,
)

DEFAULT_ATLAS = os.path.normpath(os.path.join(
    PKG_ROOT, "..", "luggage_planning", "data", "reachability_atlas",
    "s20_container_collision_aware.npz"))

# Atlas status codes: 0 UNKNOWN, 1 UNREACHABLE, 2 MARGINAL, 3 REACHABLE.
PRIOR_MAPPINGS = {
    "graded_marginal_0.50": {0: 0.0, 1: -1.0, 2: 0.50, 3: 1.0},
    "graded_marginal_0.85": {0: 0.0, 1: -1.0, 2: 0.85, 3: 1.0},
    "graded_marginal_0.95": {0: 0.0, 1: -1.0, 2: 0.95, 3: 1.0},
    "flat_top_shipped": {0: 0.0, 1: -1.0, 2: 1.00, 3: 1.0},
}

DEFAULT_WEIGHTS = [0.0, 0.1, 0.2, 0.3, 0.45, 0.6, 0.9, 1.2, 2.0]


def _row(summary):
    return {
        "reachable_fill_rate": round(summary["reachable_fill_rate_mean"], 4),
        "items_placed": round(summary["items_placed_mean"], 2),
        "floor_items": round(summary["floor_items_mean"], 2),
        "floor_coverage": round(summary["floor_coverage_mean"], 4),
    }


class _Runner(object):
    def __init__(self, atlas, entries, sequences):
        self.atlas = atlas
        self.entries = entries
        self.sequences = sequences

    def run(self, mode, prior_mapping=None, **kwargs):
        original = FreeSpaceModelStrategy._annotate_prior
        if prior_mapping is not None:
            def patched(strategy, cand, _mapping=prior_mapping):
                status = strategy.atlas.status_at(
                    strategy._atlas_contact(cand), cand["box_yaw"])
                cand["reachability_prior"] = _mapping.get(status, 0.0)
                return cand
            FreeSpaceModelStrategy._annotate_prior = patched
        try:
            strategy = FreeSpaceModelStrategy(
                self.atlas.inner_size, self.atlas.center_base, self.atlas.yaw,
                resolution=0.05, atlas=self.atlas, entries=self.entries,
                mode=mode, **kwargs)
            simulator = StrategySimulator(strategy, atlas=self.atlas)
            return summarize([
                simulator.run(sequence)
                for _, sequence in sorted(self.sequences.items())])
        finally:
            FreeSpaceModelStrategy._annotate_prior = original


def study_weight(runner, weights):
    """Where does the floor-first term stop changing the outcome?"""
    rows = {}
    for weight in weights:
        rows["w=%.2f" % weight] = _row(
            runner.run("b4", w_floor_first=weight))
    return {
        "question": (
            "How does w_floor_first trade reachable fill rate against how "
            "many boxes reach the container floor?"),
        "baselines": {
            "mvp_three_term_old_online": _row(runner.run("mvp")),
            "b2_proxy_score": _row(runner.run("b2")),
        },
        "rows": rows,
    }


def study_prior(runner, weights):
    """Does grading MARGINAL below REACHABLE help or hurt packing?"""
    rows = {}
    for name, mapping in sorted(PRIOR_MAPPINGS.items()):
        for weight in weights:
            rows["%s|w=%.2f" % (name, weight)] = _row(
                runner.run("b4", prior_mapping=mapping, w_floor_first=weight))
    return {
        "question": (
            "Both REACHABLE and MARGINAL mean IK exists. Does ranking "
            "MARGINAL lower cost placed volume?"),
        "rows": rows,
    }


def study_isolation(runner):
    """Attribute a regression to the prior rather than to the tie-break."""
    constant = {code: 0.5 for code in range(4)}
    return {
        "question": (
            "b4 at w=0 differs from b2. Is that the per-candidate atlas prior "
            "or the deterministic tie-break?"),
        "rows": {
            "b2_reference": _row(runner.run("b2")),
            "b4_w0_real_prior_and_tiebreak": _row(
                runner.run("b4", w_floor_first=0.0)),
            "b4_w0_constant_prior_0.5": _row(
                runner.run(
                    "b4", prior_mapping=constant, w_floor_first=0.0)),
        },
        "reading": (
            "If the constant-prior row reproduces b2 exactly, the tie-break is "
            "neutral and the difference is entirely the prior."),
    }


def main():
    parser = argparse.ArgumentParser(
        description="Reproducible placement-scorer ablations.")
    parser.add_argument("--atlas", default=DEFAULT_ATLAS)
    parser.add_argument("--seeds", type=int, default=20)
    parser.add_argument("--length", type=int, default=60)
    parser.add_argument(
        "--study", default="all",
        choices=["all", "weight", "prior", "isolation"])
    parser.add_argument(
        "--weights", default=",".join(str(w) for w in DEFAULT_WEIGHTS))
    parser.add_argument("--output", default="")
    args = parser.parse_args()

    weights = [float(v) for v in args.weights.split(",") if v.strip()]
    atlas = ReachabilityAtlas(args.atlas)
    runner = _Runner(
        atlas, catalog_entries(load_catalog()),
        generate_sequences(range(args.seeds), args.length))

    started = time.time()
    studies = {}
    if args.study in ("all", "weight"):
        studies["w_floor_first_sweep"] = study_weight(runner, weights)
    if args.study in ("all", "prior"):
        studies["atlas_prior_mapping"] = study_prior(
            runner, [w for w in (0.0, 0.6, 1.2) if w in weights] or [0.0, 0.6])
    if args.study in ("all", "isolation"):
        studies["prior_vs_tiebreak_isolation"] = study_isolation(runner)

    report = {
        "schema_version": 1,
        "generated_by": "packing_score_ablation.py",
        "atlas": os.path.basename(args.atlas),
        "sequences": args.seeds,
        "sequence_length": args.length,
        "elapsed_sec": round(time.time() - started, 1),
        "studies": studies,
    }
    text = yaml.safe_dump(report, default_flow_style=False, sort_keys=False,
                          allow_unicode=True)
    if args.output:
        with open(args.output, "w") as stream:
            stream.write(text)
    print(text)


if __name__ == "__main__":
    main()
