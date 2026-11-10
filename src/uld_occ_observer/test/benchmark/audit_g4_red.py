#!/usr/bin/env python3
"""Replay the 1938 review's SEVEN bad cases against this tree's judge.

red   -- run BEFORE any judge edit: records which mutations the gen3 judge
         wrongly accepts (the review measured 7/7 accepted). The recorded
         RESULTS.json is the red baseline; nothing is modified.
after -- run after the gen4 repair: every mutation must be REJECTED with
         named reasons and both contract-grade healthy controls must stay
         green; exit 0 only then (the tests entry runs this on every call).

The mutations are the review's own, applied to the SAME full-grade fixtures
the offline suite uses -- never a short fixture.
"""
import copy
import json
import sys
from pathlib import Path

TEST = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TEST))
sys.path.insert(0, str(TEST / "benchmark"))

import test_freespace_g3_offline as fixtures  # noqa: E402
import freespace_judge as judge  # noqa: E402

MIB = 1024 * 1024

# (name, factory, judge, mutation) -- the 1938 review table, verbatim edits.
MUTATIONS = [
    ("p_all_no_data", fixtures.full_p_window, judge.judge_p_window,
     lambda a: [q.update(status="ACQUIRE_NO_DATA", release_status=None)
                for q in a["queries"]]),
    ("p_missing_materialized_candidates", fixtures.full_p_window,
     judge.judge_p_window,
     lambda a: [q.update(candidates=[], content_sha256=judge.candidate_digest([]),
                         feasible_total=100000) for q in a["queries"]]),
    ("p_wrong_epoch_all_queries", fixtures.full_p_window, judge.judge_p_window,
     lambda a: [q["lease_identity"].update(epoch=-999) for q in a["queries"]]),
    ("l_all_queries_age_30_seconds", fixtures.full_l_artifact,
     judge.judge_l_stage,
     lambda a: [q.update(age_ns=30_000_000_000) for q in a["query_records"]]),
    # Review case 5 ("L has no health gate"): the gen4 judge consumes ONLY
    # the raw health series, so the scenario translates to the raw evidence
    # being absent while the old self-reports still claim 100000 s -- proving
    # a self-reported maximum alone can never pass again.
    ("l_no_health", fixtures.full_l_artifact, judge.judge_l_stage,
     lambda a: (a.update(health_samples=0, health_gap_max_s=100000),
                a.pop("health_series", None))[0]),
    # Review case 6 (100 MiB cache residue with the directory count back at
    # baseline): a cache leak detached from any snapshot lands in the
    # EXTERNAL surface-buffer owner class (the deleter tracker), the class
    # the directory-only check that passed this in gen3 could not see.
    ("l_round_cache_residual_100MiB", fixtures.full_l_artifact,
     judge.judge_l_stage,
     lambda a: [sample.update(
         external_surface_buffer_bytes=sample.get("external_surface_buffer_bytes", 0) + 100 * MIB)
         for r in a["read_rounds_results"]
         for sample in r["ownership_after"]["resource_samples"]]),
    ("l_all_read_rounds_no_lease", fixtures.full_l_artifact, judge.judge_l_stage,
     lambda a: [r.update(status="ACQUIRE_INTERNAL_ERROR", release_status=None)
                for r in a["read_rounds_results"]]),
]

CONTROLS = [
    ("healthy_p", fixtures.full_p_window, judge.judge_p_window),
    ("healthy_l", fixtures.full_l_artifact, judge.judge_l_stage),
]


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "after"
    out_dir = Path(sys.argv[2]) if len(sys.argv) > 2 else Path(__file__).parent
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    for name, factory, judge_fn in CONTROLS:
        verdict = judge_fn(copy.deepcopy(factory()))
        rows.append({"name": name, "control": True,
                     "reasons": verdict["reasons"][:20],
                     "accepted": not verdict["reasons"]})
    for name, factory, judge_fn, mutate in MUTATIONS:
        artifact = copy.deepcopy(factory())
        mutate(artifact)
        verdict = judge_fn(artifact)
        rows.append({"name": name, "control": False,
                     "reasons": verdict["reasons"][:20],
                     "accepted": not verdict["reasons"]})
    (out_dir / ("RESULTS_%s.json" % mode)).write_text(json.dumps(rows, indent=1))
    controls_ok = all(row["accepted"] for row in rows if row["control"])
    mutants_rejected = all(not row["accepted"] for row in rows
                           if not row["control"])
    print(json.dumps({row["name"]: ("accepted" if row["accepted"]
                                    else "rejected") for row in rows}))
    if mode == "red":
        # The baseline documents the escapes; anything already rejected here
        # would mean the review's reproduction drifted.
        escaped = sum(1 for row in rows if not row["control"] and row["accepted"])
        print("red baseline: %d/7 wrongly accepted, controls green=%s"
              % (escaped, controls_ok))
        return 0
    if not controls_ok or not mutants_rejected:
        print("FAILED: controls_ok=%s mutants_rejected=%s"
              % (controls_ok, mutants_rejected))
        return 1
    print("PASS: controls green, 7/7 mutations rejected")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
