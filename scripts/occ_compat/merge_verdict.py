"""merge_verdict — the G2 generation verdict (G2 plan §7/§9).

Combines per-required-ID authoritative sources from REUSE_MAP (g2 run01
for the four targets, run03 for the reused 180) into one A0–A5 verdict
per the ORIGINAL contract semantics. Fail-closed: any required ID
without an exact-PASS authoritative source keeps its gate
NOT_EVALUATED with the ID named; any real FAIL dominates. The old
run02/run03 verdicts are inputs, never modified.
"""
import argparse
import json
import sys
import os
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from occ_compat.judge import (  # noqa: E402
    V1_REQUIRED, V2_REQUIRED, EXPECTED_V1_GROUPS, EXPECTED_V1_O_GROUPS,
    EXPECTED_V2G_GROUPS, EXPECTED_V2O_GROUPS)


def _per_id_state(evidence_root, run, group, case_id):
    from occ_compat.ledger import load_events
    from occ_compat.judge import _terminals
    for base in ("%s_judge_input" % run, run):
        ledger = Path(evidence_root) / base / group / "events.jsonl"
        if not ledger.exists():
            continue
        rows = [t["state"] for t in _terminals(load_events(ledger))
                if t["case_id"] == case_id]
        if any(s == "FAIL" for s in rows):
            return "FAIL"
        if any(s == "PASS" for s in rows):
            return "PASS"
        return rows[0] if rows else "absent"
    return "absent"


def _scope(reuse, evidence_root, groups, required):
    """Per-ID rule over the REUSE_MAP's authoritative sources."""
    failed, not_hit = [], []
    for group in groups:
        entries = reuse["groups"].get(group, {})
        for case_id in required:
            entry = entries.get(case_id)
            if entry is None:
                not_hit.append("%s/%s(no-map-entry)" % (group, case_id))
                continue
            source = entry["source"]
            state = (entry["state_run03"] if source == "run03"
                     else _per_id_state(evidence_root, "g2/run01",
                                        group, case_id))
            if state == "FAIL":
                failed.append("%s/%s" % (group, case_id))
            elif state != "PASS":
                not_hit.append("%s/%s(%s@%s)" % (group, case_id, state,
                                                 source))
    return failed, not_hit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reuse-map", required=True)
    parser.add_argument("--g2-verdict", required=True,
                        help="the g2 run01 VERDICT.json")
    parser.add_argument("--run03-verdict", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    evidence_root = Path(args.reuse_map).parents[1]
    reuse = json.loads(Path(args.reuse_map).read_text())
    g2 = json.loads(Path(args.g2_verdict).read_text())
    run03 = json.loads(Path(args.run03-verdict if False else
                            args.run03_verdict).read_text())

    gates = {}
    # A0/A1: the g2 generation's freeze binding + judge wiring (the
    # reused cases' A1 evidence carries over via the dependency diff in
    # the reuse map; A0 payload closure is bound to g2's own freeze).
    gates["A0"] = g2["gates"]["A0"]
    gates["A1"] = g2["gates"]["A1"]

    for gate, groups, required in (
            ("A2", EXPECTED_V2G_GROUPS, V2_REQUIRED),
            ("A3", EXPECTED_V1_GROUPS, V1_REQUIRED),
            ("A4", list(EXPECTED_V1_O_GROUPS) + list(EXPECTED_V2O_GROUPS),
             None)):
        if gate == "A4":
            failed, not_hit = [], []
            f1, n1 = _scope(reuse, evidence_root, EXPECTED_V1_O_GROUPS,
                            V1_REQUIRED)
            f2, n2 = _scope(reuse, evidence_root, EXPECTED_V2O_GROUPS,
                            V2_REQUIRED)
            failed, not_hit = f1 + f2, n1 + n2
        else:
            failed, not_hit = _scope(reuse, evidence_root, groups, required)
        if failed:
            gates[gate] = "FAIL: %s" % failed[:4]
        elif not_hit:
            gates[gate] = "NOT_EVALUATED: %s" % not_hit[:6]
        else:
            gates[gate] = "PASS"

    # A5: the g2 run's own ledger/regression/replay gates PASS and the
    # reused slice's A5 stands on unchanged product+inputs (dependency
    # diff in the reuse map). Any FAIL in either dominates.
    a5_g2, a5_03 = g2["gates"].get("A5", ""), run03["gates"].get("A5", "")
    if a5_g2.startswith("FAIL") or a5_03.startswith("FAIL") or \
            a5_g2.startswith("INCOMPLETE") or a5_03.startswith("INCOMPLETE"):
        gates["A5"] = "FAIL: g2=%s run03=%s" % (a5_g2, a5_03)
    elif a5_g2 == "PASS" and a5_03 == "PASS":
        gates["A5"] = "PASS"
    else:
        gates["A5"] = "NOT_EVALUATED: g2=%s run03=%s" % (a5_g2, a5_03)

    verdict = {
        "schema": "g2-merged-verdict-v1",
        "policy": ("new-generation combination: g2/run01 for the four "
                   "targets, run03 for the reused 180 (REUSE_MAP "
                   "authoritative sources); run02/run03 verdicts "
                   "unchanged; any real FAIL dominates"),
        "gates": gates,
        "all_pass": all(v == "PASS" for v in gates.values()),
    }
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(verdict, indent=1))
    print("merged: " + " ".join("%s=%s" % (k, v[:60])
                                for k, v in sorted(gates.items())))
    print("ALL_PASS" if verdict["all_pass"] else "NOT-ALL-PASS")
    return 0 if verdict["all_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
