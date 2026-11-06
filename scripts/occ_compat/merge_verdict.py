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
    """Per-ID rule over the REUSE_MAP's authoritative sources — but the
    STATE is always recomputed from the source run's original ledger
    (reviews 2026-11-06: a merger that trusts the map's recorded
    state_run03 is not an independent verdict). A map entry only picks
    WHICH ledger is authoritative; if that ledger disagrees with the
    map, the ledger wins and the disagreement is named."""
    failed, not_hit = [], []
    for group in groups:
        entries = reuse["groups"].get(group, {})
        for case_id in required:
            entry = entries.get(case_id)
            if entry is None:
                not_hit.append("%s/%s(no-map-entry)" % (group, case_id))
                continue
            source = entry["source"]
            if not str(source).startswith("g2/"):
                not_hit.append("%s/%s(non-g2 source %s — old-run reuse "
                               "cannot carry acceptance)"
                               % (group, case_id, source))
                continue
            state = _per_id_state(evidence_root, source, group, case_id)
            recorded = entry.get("state_run03")
            if recorded == "PASS" and state != "PASS":
                not_hit.append("%s/%s(history PASS but %s@%s)"
                               % (group, case_id, state, source))
                continue
            if state == "FAIL":
                failed.append("%s/%s" % (group, case_id))
            elif state != "PASS":
                not_hit.append("%s/%s(%s@%s)" % (group, case_id, state,
                                                 source))
    return failed, not_hit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reuse-map", required=True)
    parser.add_argument("--g2-verdict", action="append", required=True,
                        help="VERDICT.json of a contributing g2 run "
                             "(repeatable; every one must exist)")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    evidence_root = Path(args.reuse_map).parents[1]
    reuse = json.loads(Path(args.reuse_map).read_text())
    g2_verdicts = [json.loads(Path(v).read_text()) for v in
                   args.g2_verdict]

    gates = {}
    # A0/A1: EVERY contributing run carries its own freeze-v2 payload
    # closure and the same judge wiring — all must PASS; the worst
    # string wins (a REJECTED A0 or failing A1 in any run fails all).
    gates["A0"] = min((v["gates"].get("A0", "") for v in g2_verdicts),
                      key=lambda g: (g == "PASS", g.startswith("FAIL"),
                                     g))
    gates["A1"] = min((v["gates"].get("A1", "") for v in g2_verdicts),
                      key=lambda g: (g == "PASS", g.startswith("FAIL"),
                                     g))

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

    # A5: every contributing run's own ledger/regression/replay gates
    # must PASS; the worst string wins.
    a5s = [v["gates"].get("A5", "") for v in g2_verdicts]
    bad = [a for a in a5s if a != "PASS"]
    if any(a.startswith(("FAIL", "INCOMPLETE")) for a in bad):
        gates["A5"] = "FAIL: %s" % bad[:2]
    elif bad:
        gates["A5"] = "NOT_EVALUATED: %s" % bad[:2]
    else:
        gates["A5"] = "PASS"

    verdict = {
        "schema": "g2-merged-verdict-v2",
        "policy": ("same-generation combination over freeze-v2 runs only: "
                   "each contributing run carries its own payload-closed "
                   "A0 and is judged against its own freeze; per-ID states "
                   "are recomputed from the authoritative run's original "
                   "ledger (never the map's self-report); any real FAIL "
                   "dominates; G1 history verdicts unchanged"),
        "contributing_runs": list(args.g2_verdict),
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
