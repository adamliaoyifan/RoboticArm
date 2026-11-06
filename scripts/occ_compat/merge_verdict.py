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


#: Sub-checks that are case-LOCAL (each issue carries group/case and
#: may be superseded by a newer collection of that case). Everything
#: else in the structured A5 detail is GLOBAL and always binds.
_LOCAL_SUBCHECKS = {"fail_evidence"}


def _a5_gate(reuse, g2_verdicts):
    """A5 over the contributing runs, fail-closed (reviews
    2026-11-06_2332).

    Reads the STRUCTURED A5 sub-check detail only — never a single
    problem string, which the upstream judge used to truncate to the
    first defect and thereby hide a simultaneous global failure behind
    an excusable local one. Rules: (1) a verdict without complete
    structured detail cannot be verified and binds as NOT_EVALUATED —
    no single-problem guessing; (2) every GLOBAL sub-check
    (ledger/regression/replay/unknown names) must be PASS or it binds;
    (3) a case-LOCAL issue may be excused only when its ID is
    authoritatively superseded by a provided run whose own A0/A1/A5 all
    PASS."""
    verdicts_by_run = {}
    for verdict in g2_verdicts:
        run_dir = _run_of(verdict)
        if run_dir is None or run_dir in verdicts_by_run:
            return ("FAIL: verdict archive not a unique g2/<run> "
                    "directory: %s" % str(verdict.get("archive"))[:60]), []
        verdicts_by_run[run_dir] = verdict

    problems, excuses = [], []
    for run_dir, verdict in sorted(verdicts_by_run.items()):
        subchecks = ((verdict.get("gates_detail") or {})
                     .get("A5"))
        if not isinstance(subchecks, dict) or not subchecks:
            problems.append(
                "%s A5 NOT_EVALUATED: verdict lacks the structured "
                "sub-check detail required to verify completeness — "
                "no single-problem guessing" % run_dir)
            continue
        local_pairs, globals_bad = [], []
        for name, check in sorted(subchecks.items()):
            state = str((check or {}).get("state", ""))
            if state == "PASS":
                continue
            if name not in _LOCAL_SUBCHECKS:
                globals_bad.append("%s=%s" % (name, state[:70]))
                continue
            for issue in (check or {}).get("issues", []):
                group, case_id = issue.get("group"), issue.get("case")
                if group and case_id:
                    local_pairs.append((group, case_id, str(
                        issue.get("detail", ""))[:60]))
                else:
                    # a "local" rule that names no group/case is unknown
                    globals_bad.append(
                        "%s issue without group/case: %s"
                        % (name, str(issue)[:70]))
        unresolved = []
        for group, case_id, detail in local_pairs:
            entry = (reuse.get("groups", {}).get(group, {})
                     .get(case_id))
            source = (entry or {}).get("source")
            if source == run_dir:
                unresolved.append("%s/%s still authoritative here (%s)"
                                  % (group, case_id, detail))
                continue
            superseder = verdicts_by_run.get(source)
            if superseder is None:
                unresolved.append("%s/%s source %s has no provided "
                                  "verdict" % (group, case_id, source))
                continue
            if not all(superseder["gates"].get(g) == "PASS"
                       for g in ("A0", "A1", "A5")):
                unresolved.append("%s/%s superseder %s lacks A0/A1/A5 "
                                  "PASS" % (group, case_id, source))
        if globals_bad:
            problems.append("%s A5 global sub-check failure(s) not "
                            "excusable: %s" % (run_dir, globals_bad[:3]))
        if unresolved:
            problems.append("%s A5 local issue(s) not excusable: %s"
                            % (run_dir, unresolved[:3]))
        if not globals_bad and not unresolved and local_pairs:
            excuses.append({
                "run": run_dir,
                "superseded_ids": sorted("%s/%s" % (g, c)
                                         for g, c, _ in local_pairs),
                "carried_by": sorted({
                    reuse["groups"][g][c]["source"]
                    for g, c, _ in local_pairs}),
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
        "schema": "g2-merged-verdict-v4",
        "policy": ("v4: reads the judge's COMPLETE structured A5 "
                   "sub-checks (scope/rule/group/case per issue); global "
                   "or unknown sub-check failures always bind; local "
                   "issues excusable only via explicit supersession by a "
                   "run whose own A0/A1/A5 all PASS; verdicts without "
                   "structured detail bind NOT_EVALUATED. "
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
    def _verdict(run, a5, a0="PASS", a1="PASS", subchecks=None):
        doc = {"archive": "/evidence/%s" % run,
               "gates": {"A0": a0, "A1": a1, "A5": a5}}
        if subchecks is not None:
            doc["gates_detail"] = {"A5": subchecks}
        return doc

    def _subs(fail_ids=(), regression="PASS", ledger="PASS",
              replay="PASS"):
        return {
            "fail_evidence": {
                "scope": "case-local" if fail_ids else "clean",
                "state": ("INCOMPLETE_EVIDENCE" if fail_ids else "PASS"),
                "issues": [{"scope": "case-local", "rule": "fail_evidence",
                            "group": g, "case": c,
                            "detail": "lacks consumer-path events"}
                           for g, c in fail_ids]},
            "ledger": {"scope": "global", "state": ledger, "issues": []},
            "regression": {"scope": "global", "state": regression,
                           "issues": []},
            "replay": {"scope": "global", "state": replay, "issues": []},
        }
    strong = _verdict("g2/run03", "PASS", subchecks=_subs())

    k03e4 = (("V1-O-cuboid", "K03/E4"),)

    # (a) global regression failure — must FAIL
    gate, _ = _a5_gate(base_map, [
        _verdict("g2/run02",
                 "regression=FAIL: G-BRIDGE-LIVE: 3 failure/error/skipped",
                 subchecks=_subs(regression="FAIL: G-BRIDGE-LIVE")), strong])
    control_a = gate.startswith("FAIL")
    # (b) still-authoritative local gap — must FAIL
    gate, _ = _a5_gate(base_map, [
        _verdict("g2/run02",
                 "fail_evidence=INCOMPLETE_EVIDENCE",
                 subchecks=_subs(fail_ids=(("V1-O-cuboid", "K05"),))),
        strong])
    control_b = gate.startswith("FAIL")
    # (c1) superseded local gap with a valid superseder — must PASS
    gate, excuses = _a5_gate(base_map, [
        _verdict("g2/run02", "fail_evidence=INCOMPLETE_EVIDENCE",
                 subchecks=_subs(fail_ids=k03e4)), strong])
    control_c = gate == "PASS" and excuses and \
        excuses[0]["superseded_ids"] == ["V1-O-cuboid/K03/E4"]
    # (c2) same gap but the superseder's A0 is not PASS — must FAIL
    gate, _ = _a5_gate(base_map, [
        _verdict("g2/run02", "fail_evidence=INCOMPLETE_EVIDENCE",
                 subchecks=_subs(fail_ids=k03e4)),
        _verdict("g2/run03", "PASS", a0="NOT_EVALUATED",
                 subchecks=_subs())])
    control_d = gate.startswith("FAIL")
    # (e) THE 2332 COUNTEREXAMPLE: local superseded gap AND a global
    # regression failure in the SAME run — the merged A5 must REJECT
    # (the old single-string path excused the local one and the global
    # hid behind the truncated first-defect output)
    gate, _ = _a5_gate(base_map, [
        _verdict("g2/run02",
                 "fail_evidence=INCOMPLETE_EVIDENCE; regression=FAIL",
                 subchecks=_subs(fail_ids=k03e4,
                                 regression="FAIL: missing TEST_SELECTION")),
        strong])
    control_e = gate.startswith("FAIL")
    # (f) a verdict WITHOUT structured detail binds as NOT_EVALUATED
    gate, _ = _a5_gate(base_map, [
        _verdict("g2/run02",
                 "INCOMPLETE_EVIDENCE: V1-O-cuboid/K03/E4 FAIL lacks "
                 "consumer-path events"), strong])
    control_f = "NOT_EVALUATED" in gate
    # binding: a map source with no provided verdict must hard-fail
    strong_detail = _verdict("g2/run03", "PASS", subchecks=_subs())
    binding = _check_run_binding(base_map, [strong_detail]) is not None
    results = {"a_global_not_excused": control_a,
               "b_still_ours_not_excused": control_b,
               "c_superseded_excused": control_c,
               "c2_invalid_superseder_binds": control_d,
               "e_local_plus_global_rejected": control_e,
               "f_missing_detail_binds_ne": control_f,
               "binding_missing_verdict": binding}
    ok = all(results.values())
    print("merge_verdict selftest: %s" % json.dumps(results))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
