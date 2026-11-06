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
import re
import subprocess
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


#: A5 problem strings name their case as GROUP/CASE — nothing else in
#: the verdict is scanned (reviews 2026-11-06_2307 point 3: A2/A4 text
#: must never be laundered into A5 defects).
_CASE_IN_A5 = re.compile(
    r"(V[12]-[GO]-(?:cuboid|chamfer))/(K\d+(?:/E\d)?|R\d+)")


def _run_of(verdict):
    """The g2/<run> directory a verdict judged, or None."""
    archive = str(verdict.get("archive", ""))
    match = re.search(r"g2/run\d+", archive)
    return match.group(0) if match else None


def _a5_gate(reuse, g2_verdicts):
    """A5 over the contributing runs, fail-closed.

    A non-PASS A5 may only be excused when EVERY case it names is (a)
    still authoritatively supplied by THIS run's ledger — no — by
    ANOTHER provided run whose own verdict carries A0/A1/A5 all PASS
    (the superseding evidence must be valid on its own). A defect that
    names no case ID (regression missing, replay missing, ledger
    problems…) is GLOBAL and can never be excused; an unrecognized run
    binding likewise binds. Only the A5 string itself is read."""
    verdicts_by_run = {}
    for verdict in g2_verdicts:
        run_dir = _run_of(verdict)
        if run_dir is None or run_dir in verdicts_by_run:
            return ("FAIL: verdict archive not a unique g2/<run> "
                    "directory: %s" % str(verdict.get("archive"))[:60]), []
        verdicts_by_run[run_dir] = verdict

    problems, excuses = [], []
    for run_dir, verdict in sorted(verdicts_by_run.items()):
        a5 = str(verdict["gates"].get("A5", ""))
        if a5 == "PASS":
            continue
        named = sorted(set(_CASE_IN_A5.findall(a5)))
        if not named:
            problems.append(
                "%s A5 global/unnamed defect not excusable: %s"
                % (run_dir, a5[:80]))
            continue
        unresolved = []
        for group, case_id in named:
            entry = (reuse.get("groups", {}).get(group, {})
                     .get(case_id))
            source = (entry or {}).get("source")
            if source == run_dir:
                unresolved.append("%s/%s (still authoritative here)"
                                  % (group, case_id))
                continue
            superseder = verdicts_by_run.get(source)
            if superseder is None:
                unresolved.append("%s/%s (source %s has no provided "
                                  "verdict)" % (group, case_id, source))
                continue
            gates_ok = all(
                superseder["gates"].get(g) == "PASS"
                for g in ("A0", "A1", "A5"))
            if not gates_ok:
                unresolved.append(
                    "%s/%s (superseder %s lacks A0/A1/A5 PASS)"
                    % (group, case_id, source))
        if unresolved:
            problems.append("%s A5 %s — not excusable: %s"
                            % (run_dir, a5[:60], unresolved[:3]))
        else:
            excuses.append({
                "run": run_dir,
                "a5_defect": a5[:120],
                "superseded_ids": ["%s/%s" % pair for pair in named],
                "carried_by": sorted({
                    (reuse["groups"][g][c]["source"])
                    for g, c in named}),
            })
    if problems:
        return "FAIL: %s" % problems[:2], excuses
    return "PASS", excuses


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reuse-map", required=False)
    parser.add_argument("--g2-verdict", action="append", default=[],
                        help="VERDICT.json of a contributing g2 run "
                             "(repeatable; every one must exist)")
    parser.add_argument("--out", required=False)
    parser.add_argument("--selftest", action="store_true",
                        help="run the summary-entry controls offline")
    args = parser.parse_args()
    if args.selftest:
        return _selftest()
    evidence_root = Path(args.reuse_map).parents[1]
    reuse = json.loads(Path(args.reuse_map).read_text())
    g2_verdicts = [json.loads(Path(v).read_text()) for v in
                   args.g2_verdict]
    binding = _check_run_binding(reuse, g2_verdicts)
    if binding:
        print(binding)
        return 2

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

    gates["A5"], excused = _a5_gate(reuse, g2_verdicts)

    verdict = {
        "schema": "g2-merged-verdict-v3",
        "policy": ("same-generation combination over freeze-v2 runs only: "
                   "each contributing run carries its own payload-closed "
                   "A0 and is judged against its own freeze; per-ID states "
                   "are recomputed from the authoritative run's original "
                   "ledger (never the map's self-report); A5 defects are "
                   "excusable only when every case they name is "
                   "superseded by a provided run whose own A0/A1/A5 all "
                   "PASS — global or unnamed defects always bind; G1 "
                   "history verdicts unchanged"),
        "contributing_runs": list(args.g2_verdict),
        "tool_commit": _tree_commit(),
        "a5_superseded_excuses": excused,
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


def _tree_commit():
    try:
        root = Path(__file__).resolve().parents[2]
        return subprocess.run(
            ["git", "-C", str(root), "rev-parse", "HEAD"],
            capture_output=True, text=True, timeout=30).stdout.strip()
    except Exception:  # noqa: BLE001 - stamping is required
        return ""


def _check_run_binding(reuse, g2_verdicts):
    """Every map source must have exactly one provided verdict and vice
    versa (reviews 2026-11-06_2307: reuse validity never follows from a
    label — only from the bound verdict)."""
    sources = {entry["source"]
               for group in reuse.get("groups", {}).values()
               for entry in group.values()}
    provided = {_run_of(v) for v in g2_verdicts}
    missing = sorted(sources - provided)
    extra = sorted(provided - sources)
    if missing or extra or None in provided:
        return ("FAIL: run binding mismatch — map sources without "
                "verdicts: %s; verdicts without map sources: %s"
                % (missing, extra))
    return None


def _selftest():
    """The three summary-entry controls (reviews 2026-11-06_2307
    Acceptance): (a) an ID-less global A5 failure must NOT become PASS;
    (b) an A5 gap for a case still authoritatively sourced by that run
    must NOT be excused; (c) the explicitly superseded K03/E4 gap passes
    only when the superseding run's A0/A1/A5 are all PASS."""
    base_map = {"groups": {
        "V1-O-cuboid": {
            "K03": {"source": "g2/run03"},
            "K03/E4": {"source": "g2/run03"},
            "K05": {"source": "g2/run02"}},
        "V2-G-cuboid": {"R03": {"source": "g2/run03"}}}}
    def _verdict(run, a5, a0="PASS", a1="PASS"):
        return {"archive": "/evidence/%s" % run,
                "gates": {"A0": a0, "A1": a1, "A5": a5}}
    strong = _verdict("g2/run03", "PASS")

    # (a) global regression failure names no case id — must FAIL
    gate, _ = _a5_gate(base_map, [
        _verdict("g2/run02",
                 "FAIL: G-BRIDGE-LIVE: 3 failure/error/skipped"), strong])
    control_a = gate.startswith("FAIL")
    # (b) still-authoritative gap — must FAIL
    gate, _ = _a5_gate(base_map, [
        _verdict("g2/run02",
                 "INCOMPLETE_EVIDENCE: V1-O-cuboid/K05 FAIL lacks "
                 "consumer-path events"), strong])
    control_b = gate.startswith("FAIL")
    # (c1) superseded gap with a valid superseder — must PASS + excuse
    gate, excuses = _a5_gate(base_map, [
        _verdict("g2/run02",
                 "INCOMPLETE_EVIDENCE: V1-O-cuboid/K03/E4 FAIL lacks "
                 "consumer-path events"), strong])
    control_c = gate == "PASS" and excuses and \
        excuses[0]["superseded_ids"] == ["V1-O-cuboid/K03/E4"]
    # (c2) same gap but the superseder's A0 is not PASS — must FAIL
    gate, _ = _a5_gate(base_map, [
        _verdict("g2/run02",
                 "INCOMPLETE_EVIDENCE: V1-O-cuboid/K03/E4 FAIL lacks "
                 "consumer-path events"),
        _verdict("g2/run03", "PASS", a0="NOT_EVALUATED")])
    control_d = gate.startswith("FAIL")
    # binding: a map source with no provided verdict must hard-fail
    binding = _check_run_binding(base_map, [strong]) is not None
    results = {"a_global_not_excused": control_a,
               "b_still_ours_not_excused": control_b,
               "c_superseded_excused": control_c,
               "c2_invalid_superseder_binds": control_d,
               "binding_missing_verdict": binding}
    ok = all(results.values())
    print("merge_verdict selftest: %s" % json.dumps(results))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
