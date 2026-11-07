"""REUSE_MAP builder for OCC-GRID-CONSUMER-COMPAT G2 (P0).

One authoritative source per required case ID, with the actual
dependencies that decide reuse (G2 plan §7). Collection-time tool
identities are distinct from judge identities; run02/run03 outcomes are
read-only facts and are never rewritten here.
"""
import argparse
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from occ_compat.judge import (  # noqa: E402
    V1_REQUIRED, V2_REQUIRED, judge_events, _terminals)
from occ_compat.ledger import load_events  # noqa: E402

# Which T1 modules participate in *collecting* each family. Judge/freeze
# modules (judge.py, freeze.py, a0_verify.py) never collect evidence and
# therefore cannot invalidate a historical run.
DEPS = {
    "V1": ["chains.py", "client.py", "v1_states.py", "runner.py",
           "group_main.py"],
    "V2": ["chains.py", "client.py", "v2_races.py", "runner.py",
           "group_main.py"],
}

# G2 re-collection mapping (reviews 2026-11-06: old-run reuse cannot
# carry acceptance — every required ID must come from a freeze-v2 run
# with its own payload-closed A0). run02 = full matrix; run03 =
# targeted re-collection of the two driver-defect legs (R03 both
# backends: unreleased hook park; V1-O-cuboid: E4 joint-sample aging).
G2_TARGETS = {
    "V2-G-cuboid/R04": "g2/run05", "V2-O-cuboid/R04": "g2/run05",
    "V2-G-cuboid/R03": "g2/run06", "V2-O-cuboid/R03": "g2/run06",
    "V2-G-cuboid/R05": "g2/run06", "V2-O-cuboid/R05": "g2/run06",
    "V2-G-cuboid/R12": "g2/run06", "V2-O-cuboid/R12": "g2/run06",
}
# K01 (all four composites) in every V1 group re-collected with the
# externally-bound oracle payload under G3C
for _g in ("V1-G-cuboid", "V1-G-chamfer", "V1-O-cuboid", "V1-O-chamfer"):
    for _cid in ("K01", "K01/E2", "K01/E3", "K01/E4"):
        G2_TARGETS["%s/%s" % (_g, _cid)] = "g2/run06"
G2_GROUPS = {}
G2_DEFAULT = "g2/run04"

PINS = {
    "product_grid": "3b614c270ae4b727562c499360911c0b8564622e",
    "product_consumer_c1": "4ed90d71ad172310acad34bed77745c444ca36a8",
    "product_c1_test_expectation_fix":
        "e9d0a5c241caf3608b2972f2fce3978f527d56c9",
    "product_octomap": "cbdd5364206dca70c89c251b43c681c708e0c5cf",
    "tool_collect_run02": "d8fde2d96265499a95a04c36c2a7de13e19ba146",
    "tool_collect_run03": "2b0bfb7b137a93c26e9a1e0b5f82b28652e6b05a",
    "tool_judge_run03": "26f59db3897d384b4dce9f1ad6e9cad40c7c46b2",
    "tool_collect_g2": "4d004f2835e1013df636c5aaa23ec48db2b1e80d",
    "tool_judge_g2": "4d004f2835e1013df636c5aaa23ec48db2b1e80d",
}

#: Dependency diff V7 (run03 collection) -> G2 (4d004f28): the shared
#: collection modules changed ONLY additively — per-process sessions
#: (behavior-neutral for non-restart cases; full-chain health re-proven
#: by the G2 rehearsals on both backends), EXEC/MODULE probe file
#: writes (no chain interaction), the --cases filter (absent = full
#: matrix), killpg teardown (post-results), and new R03/R05/R12 driver
#: bodies (selected cases only). Product sources G/C1/O and inputs/
#: configs are byte-identical (FREEZE_G2 vs FREEZE_V7 source maps).
#: G2 §7 row 3 argument for the 160 V1 reuse: behaviorally unchanged
#: collection path, verified by diff review + rehearsal, not by rerun.
DEPENDENCY_DIFF = {
    "chains.py": "session-per-process + tree-kill teardown + exec probes",
    "runner.py": "--cases filter (default full matrix)",
    "group_main.py": "module probes at group end",
    "client.py": "lazily created ad-hoc service clients",
    "v2_races.py": "R03/R05/R12 driver bodies + forensics",
    "v1_states.py": "unchanged (V1 path byte-identical)",
}


def _group_states(evidence_root, run, group):
    """Per-required-ID state exactly as the judge computes it: terminals
    re-derived from the run's events.jsonl (judge input copy preferred),
    aggregated per _scope_gate's rule (any FAIL dominates, any PASS hit)."""
    ledger = evidence_root / ("%s_judge_input" % run) / group / "events.jsonl"
    if not ledger.exists():
        ledger = evidence_root / run / group / "events.jsonl"
    if not ledger.exists():
        return {}
    events = load_events(ledger)
    by_id = {}
    for terminal in _terminals(events):
        by_id.setdefault(terminal["case_id"], []).append(terminal["state"])
    states = {}
    for case_id in set(V1_REQUIRED) | set(V2_REQUIRED):
        rows = by_id.get(case_id) or []
        if any(s == "FAIL" for s in rows):
            states[case_id] = "FAIL"
        elif any(s == "PASS" for s in rows):
            states[case_id] = "PASS"
        else:
            states[case_id] = rows[0] if rows else "absent"
    # composites (K0x/E2..E4) share the parent case's terminal state —
    # the sweep records one terminal per composite id
    for composite in list(by_id):
        if composite not in states:
            rows = by_id[composite]
            states[composite] = ("FAIL" if any(s == "FAIL" for s in rows)
                                 else "PASS" if any(s == "PASS" for s in rows)
                                 else rows[0])
    return states


def _lookup(states, case_id):
    return states.get(case_id, "absent")


def build(evidence_root):
    groups = {
        "V1-G-cuboid": ("V1", V1_REQUIRED),
        "V1-G-chamfer": ("V1", V1_REQUIRED),
        "V1-O-cuboid": ("V1", V1_REQUIRED),
        "V1-O-chamfer": ("V1", V1_REQUIRED),
        "V2-G-cuboid": ("V2", V2_REQUIRED),
        "V2-O-cuboid": ("V2", V2_REQUIRED),
    }
    out = {
        "schema": "reuse-map-v1",
        "campaign": "occ_grid_consumer_compat_20261105/g2",
        "policy": (
            "Reuse requires identical product sources, inputs, configs and "
            "an unchanged collection path for the case family; judge/freeze "
            "changes alone never invalidate collection. Failed or "
            "not-evaluated history is never deleted or rewritten."),
        "pins": dict(PINS),
        "groups": {},
    }
    for group, (family, required) in groups.items():
        run03 = _group_states(evidence_root, "formal_run03", group)
        run02 = _group_states(evidence_root, "formal_run02", group)
        entries = {}
        for case_id in required:
            key = "%s/%s" % (group, case_id)
            state03 = _lookup(run03, case_id)
            state02 = _lookup(run02, case_id)
            source = G2_TARGETS.get(key) or G2_GROUPS.get(group) \
                or G2_DEFAULT
            if source != G2_DEFAULT:
                reason = ("targeted re-collection after the R04 budget-"
                          "field fix; run04's own FAIL retained as "
                          "history")
            else:
                reason = ("collected under FREEZE_G3 (freeze-v3: input "
                          "classes + runtime probes + deadline triple + "
                          "K01 oracle payload) in the full-matrix run")
            entries[case_id] = {
                "state_run03": state03,
                "state_run02": state02,
                "source": source,
                "reason": reason,
                "collection_deps": list(DEPS[family]),
                "dependency_diff": DEPENDENCY_DIFF,
                "reuse_verified": str(source).startswith("g2/"),
            }
        out["groups"][group] = entries
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence-root", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    root = Path(args.evidence_root)
    result = build(root)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=1, sort_keys=True))
    counts = {}
    for group, entries in result["groups"].items():
        for case_id, entry in entries.items():
            key = entry["source"]
            counts[key] = counts.get(key, 0) + 1
            if entry["source"] == "run03" and entry["state_run03"] != "PASS":
                print("WARNING: %s/%s reuse-planned but run03=%s"
                      % (group, case_id, entry["state_run03"]))
    print("wrote %s: %s" % (out, counts))


if __name__ == "__main__":
    main()
