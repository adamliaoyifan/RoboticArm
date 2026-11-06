"""merge_verdict — the G2 generation verdict (G2 plan §7/§9, reviews
2026-11-06_2307/2332/2350).

Combines per-required-ID authoritative sources from REUSE_MAP into one
A0–A5 verdict under the ORIGINAL contract semantics. Fail-closed by
construction:

* CONTENT BINDING (F02) — every contributing verdict carries the SHA256
  of each ledger it judged; the merger reads ledgers ONLY through the
  verdict's own archive binding and re-hashes them. A verdict pasted
  onto a different ledger cannot merge (canonical g2/<run> path match,
  never a substring; no implicit shadow *_judge_input directories; no
  duplicate sources; contributing runs must bind identical product
  commits).
* PER-LEG EVALUATION (F03) — an ID hits only when every leg of its
  FROZEN subprobe set carries exactly one PASS terminal (the judge's
  own _evaluate_case). Ledger FAIL always dominates; a map self-report
  never downgrades it.
* STRUCTURED A5 (F01/2332) — the complete structured sub-check detail
  is validated (required set, state enums, scope/rule per issue,
  PASS⇔empty issues); global/unknown failures always bind; only
  explicit case-local issues with proven supersession (by a provided
  run whose own A0/A1/A5 all PASS) are excusable; verdicts without
  detail bind NOT_EVALUATED.

A few single-purpose probe entry points (_per_id_state, the legacy
_scope form) exist for the adversarial controls in
review_full_v4/controls.py; the CLI never uses them.
"""
import argparse
import hashlib
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
    EXPECTED_V2G_GROUPS, EXPECTED_V2O_GROUPS, _evaluate_case,
    _terminals)

#: F01: the A5 detail must carry exactly this sub-check set with these
#: classifications; only fail_evidence is case-local.
_A5_REQUIRED_SUBCHECKS = {
    "fail_evidence": "local", "ledger": "global",
    "regression": "global", "replay": "global",
}
_A5_STATES = {"PASS", "FAIL", "INCOMPLETE_EVIDENCE", "NOT_EVALUATED"}
_SEVERITY = {"FAIL": 4, "INCOMPLETE_EVIDENCE": 3, "NOT_EVALUATED": 2}


def _no_dupes(pairs):
    out = {}
    for key, value in pairs:
        if key in out:
            raise ValueError("duplicate JSON key: %s" % key)
        out[key] = value
    return out


def _load(path):
    return json.loads(Path(path).read_text(),
                      object_pairs_hook=_no_dupes)


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _severity(text):
    head = str(text).split(":", 1)[0].split("=", 1)[0].strip()
    return _SEVERITY.get(head, 1 if str(text) != "PASS" else 0)


def _worst(strings):
    worst_text = "PASS"
    for s in strings:
        if _severity(s) > _severity(worst_text):
            worst_text = s
    return worst_text


class InvalidInput(Exception):
    """Named, non-crashing rejection of malformed inputs (F10)."""


def _run_key(verdict):
    """Canonical run id of a verdict — its archive path must END with
    g2/<run> (anchored suffix, never a bare substring; F02)."""
    match = re.search(r"(g2/run\d+)/*$", str(verdict.get("archive", "")))
    return match.group(1) if match else None


def _as_map(verdicts):
    """Accept a list of verdicts or the {run: verdict} map (controls
    pass a list); every verdict must bind a canonical run."""
    if not isinstance(verdicts, dict):
        out = {}
        for verdict in verdicts:
            key = _run_key(verdict)
            if key is None or key in out:
                raise InvalidInput(
                    "verdict archive not a unique canonical g2/<run> "
                    "path: %s" % str(verdict.get("archive"))[:60])
            out[key] = verdict
        return out
    return verdicts


def _bind_verdict(evidence_root, verdict, index):
    """Validate one contributing verdict and return
    (run_dir, {group: (ledger_path, events)}) bound to EXACTLY the
    ledgers it judged (F02)."""
    archive = verdict.get("archive")
    inputs = verdict.get("inputs") or {}
    declared = inputs.get("archive")
    if not archive or not declared:
        raise InvalidInput(
            "verdict %d lacks its archive binding (inputs.archive + "
            "per-group ledger digests are required)" % index)
    archive_dir = Path(archive).resolve()
    if archive_dir != Path(declared).resolve():
        raise InvalidInput(
            "verdict %d archive %s != declared inputs.archive %s"
            % (index, archive, declared))
    if not archive_dir.is_dir():
        raise InvalidInput(
            "verdict %d archive %s does not exist" % (index, archive))
    try:
        rel = archive_dir.relative_to(Path(evidence_root).resolve())
    except ValueError:
        raise InvalidInput(  # noqa: B904 - named rejection
            "verdict %d archive %s is outside the evidence root"
            % (index, archive))
    run_dir = str(rel)
    if not re.fullmatch(r"g2/run\d+", run_dir):
        raise InvalidInput(
            "verdict %d archive is not a canonical g2/<run> directory: %s"
            % (index, run_dir))
    group_digests = inputs.get("groups") or {}
    if not group_digests:
        raise InvalidInput(
            "verdict %d records no group ledger digests" % index)
    from occ_compat.ledger import load_events
    bound = {}
    for group, meta in sorted(group_digests.items()):
        ledger = archive_dir / group / "events.jsonl"
        if not ledger.is_file():
            raise InvalidInput(
                "verdict %d group ledger missing: %s" % (index, ledger))
        want = (meta or {}).get("ledger_sha256")
        if not want or want != _sha(ledger):
            raise InvalidInput(
                "verdict %d ledger digest mismatch for %s — the verdict "
                "was not judged against these bytes" % (index, group))
        bound[group] = (ledger, load_events(ledger))
    return run_dir, bound


def _evaluate_rows(events, case_id):
    rows = [t for t in _terminals(events)
            if t["case_id"] == case_id]
    ok, detail = _evaluate_case(case_id, rows)
    return detail


def _per_id(bound_run, group, case_id):
    """Per-leg state from the BOUND ledger only (F03)."""
    if group not in bound_run:
        return "missing-group"
    return _evaluate_rows(bound_run[group][1], case_id)


def _per_id_state(evidence_root, run, group, case_id):
    """Probe entry for the adversarial controls: reads
    <root>/<run>/<group>/events.jsonl ONLY — never an implicit shadow
    (*_judge_input) directory (F02)."""
    from occ_compat.ledger import load_events
    ledger = Path(evidence_root) / run / group / "events.jsonl"
    if not ledger.is_file():
        return "absent"
    return _evaluate_rows(load_events(ledger), case_id)


def _bind_legacy(root, reuse, groups):
    """Legacy-probe binding: each map entry's source ledger under the
    root, original paths only (no shadow directories)."""
    from occ_compat.ledger import load_events
    bound, run_dirs = {}, set()
    for group in groups:
        for case_id, entry in reuse["groups"].get(group, {}).items():
            source = str(entry.get("source", ""))
            ledger = root / source / group / "events.jsonl"
            run_dirs.add(source)
            if ledger.is_file():
                bound.setdefault(source, {})[group] = (
                    ledger, load_events(ledger))
            else:
                bound.setdefault(source, {})
    return bound, run_dirs


def _scope(reuse, bound, run_dirs, groups=None, required=None):
    """Formal entry: (reuse, bound, run_dirs, groups, required).

    The 4-arg legacy probe form (map, root, groups, required) reads
    each entry's OWN source ledger under the root — no shadow
    directories — and exists only for the adversarial controls; the CLI
    never uses it."""
    if (required is None and groups is not None
            and isinstance(bound, (str, Path))):
        root, groups, required = Path(bound), run_dirs, groups
        bound, run_dirs = _bind_legacy(root, reuse, groups)
    failed, not_hit = [], []
    for group in groups:
        entries = reuse["groups"].get(group, {})
        for case_id in required:
            entry = entries.get(case_id)
            if entry is None:
                not_hit.append("%s/%s(no-map-entry)" % (group, case_id))
                continue
            source = str(entry.get("source", ""))
            if source not in run_dirs:
                not_hit.append("%s/%s(source %s not provided)"
                               % (group, case_id, source))
                continue
            detail = _per_id(bound.get(source, {}), group, case_id)
            if detail == "FAIL":
                # The ledger's own FAIL always dominates; a map
                # self-report never downgrades it (F03).
                failed.append("%s/%s" % (group, case_id))
            elif detail != "PASS":
                not_hit.append("%s/%s(%s@%s)"
                               % (group, case_id, detail, source))
    return failed, not_hit


def _validate_a5_detail(detail):
    """(local_pairs, globals_bad, structure_errors) for one structured
    A5 detail (F01)."""
    errors = []
    if not isinstance(detail, dict) or not detail:
        return [], [], ["A5 detail absent/not a dict"]
    unknown = [k for k in detail if k not in _A5_REQUIRED_SUBCHECKS]
    absent = [k for k in _A5_REQUIRED_SUBCHECKS if k not in detail]
    if unknown:
        errors.append("unknown sub-checks: %s" % unknown[:3])
    if absent:
        errors.append("missing sub-checks: %s" % absent[:3])
    local_pairs, globals_bad = [], []
    for name, kind in _A5_REQUIRED_SUBCHECKS.items():
        if name not in detail:
            continue
        check = detail[name]
        if not isinstance(check, dict):
            errors.append("%s not an object" % name)
            continue
        state = str(check.get("state", ""))
        if state not in _A5_STATES:
            errors.append("%s unknown state %r" % (name, state))
            continue
        issues = check.get("issues", [])
        if state == "PASS" and issues:
            errors.append("%s PASS with non-empty issues" % name)
        if state != "PASS" and kind == "global":
            globals_bad.append("%s=%s" % (name, state[:70]))
        if state != "PASS" and kind == "local":
            if not issues:
                errors.append("%s non-PASS with EMPTY issues" % name)
                continue
            for issue in issues:
                if not isinstance(issue, dict):
                    errors.append("%s malformed issue" % name)
                    continue
                if issue.get("scope") != "case-local" or \
                        issue.get("rule") != name:
                    errors.append("%s issue scope/rule invalid: %s/%s"
                                  % (name, issue.get("scope"),
                                     issue.get("rule")))
                    continue
                group, case = issue.get("group"), issue.get("case")
                if group and case:
                    local_pairs.append((group, case))
                else:
                    errors.append("%s issue lacks group/case" % name)
    return local_pairs, globals_bad, errors


def _a5_gate(reuse, verdicts):
    """A5 over the contributing runs (F01/2332): reads ONLY the
    validated structured detail; global/unknown bind; local issues need
    proven supersession."""
    verdicts_by_run = _as_map(verdicts)
    problems, excuses = [], []
    for run_dir, verdict in sorted(verdicts_by_run.items()):
        detail = ((verdict.get("gates_detail") or {}).get("A5"))
        if detail is None:
            problems.append("%s A5 NOT_EVALUATED: verdict lacks the "
                            "structured sub-check detail required to "
                            "verify completeness" % run_dir)
            continue
        local_pairs, globals_bad, errors = _validate_a5_detail(detail)
        if errors:
            problems.append("%s A5 detail invalid: %s"
                            % (run_dir, errors[:3]))
        # headline<->detail consistency (F01): the gate string and the
        # structured detail must name the SAME failing sub-checks in
        # both directions — a headline claiming a global failure the
        # detail does not carry (or hiding one it does) binds.
        headline = str(verdict.get("gates", {}).get("A5", ""))
        detail_bad = sorted(
            n for n in _A5_REQUIRED_SUBCHECKS
            if n in detail
            and str((detail[n] or {}).get("state")) != "PASS")
        headline_bad = sorted(
            n for n in _A5_REQUIRED_SUBCHECKS if n in headline)
        if headline == "PASS" and detail_bad:
            problems.append("%s A5 headline PASS but detail fails %s"
                            % (run_dir, detail_bad[:3]))
        elif headline != "PASS" and headline_bad != detail_bad:
            problems.append(
                "%s A5 headline/detail mismatch: headline %s vs detail %s"
                % (run_dir, headline_bad or "clean", detail_bad or
                   "clean"))
        unresolved = []
        for group, case_id in local_pairs:
            entry = (reuse.get("groups", {}).get(group, {})
                     .get(case_id))
            source = (entry or {}).get("source")
            if source == run_dir:
                unresolved.append("%s/%s still authoritative here"
                                  % (group, case_id))
                continue
            superseder = verdicts_by_run.get(source)
            if superseder is None:
                unresolved.append("%s/%s source %s not provided"
                                  % (group, case_id, source))
                continue
            if not all(superseder["gates"].get(g) == "PASS"
                       for g in ("A0", "A1", "A5")):
                unresolved.append("%s/%s superseder %s not A0/A1/A5 PASS"
                                  % (group, case_id, source))
        if globals_bad:
            problems.append("%s A5 global failure(s) bind: %s"
                            % (run_dir, globals_bad[:3]))
        if unresolved:
            problems.append("%s A5 local issue(s) not excusable: %s"
                            % (run_dir, unresolved[:3]))
        if not globals_bad and not unresolved and local_pairs:
            excuses.append({
                "run": run_dir,
                "superseded_ids": sorted("%s/%s" % p for p in local_pairs),
                "carried_by": sorted({reuse["groups"][g][c]["source"]
                                      for g, c in local_pairs}),
            })
    if problems:
        return "FAIL: %s" % problems[:2], excuses
    return "PASS", excuses


def _check_run_binding(reuse, verdicts_by_run):
    sources = {entry["source"]
               for group in reuse.get("groups", {}).values()
               for entry in group.values()}
    provided = ({_run_key(v) for v in verdicts_by_run}
                if isinstance(verdicts_by_run, list)
                else set(verdicts_by_run))
    missing = sorted(sources - provided)
    extra = sorted(provided - sources)
    if missing or extra or None in provided:
        return ("FAIL: run binding mismatch — map sources without "
                "verdicts: %s; verdicts without map sources: %s"
                % (missing, extra))
    return None


def _tree_commit():
    try:
        root = Path(__file__).resolve().parents[2]
        return subprocess.run(
            ["git", "-C", str(root), "rev-parse", "HEAD"],
            capture_output=True, text=True, timeout=30).stdout.strip()
    except Exception:  # noqa: BLE001 - stamping is required
        return ""


#: excerpt of R03's frozen legs for the disk-level binding controls
REQUIRED_LEG_EXCERPT = ("handle-acquired", "old-handle-refused",
                        "old-identity-refused",
                        "old-payload-unauthorized",
                        "old-revision-request-refused",
                        "new-epoch-healthy", "")


def _selftest():
    """Offline entry controls (reviews 2307/2332/2350)."""
    base_map = {"groups": {
        "V1-O-cuboid": {"K03": {"source": "g2/run03"},
                        "K03/E4": {"source": "g2/run03"},
                        "K05": {"source": "g2/run02"}},
        "V2-G-cuboid": {"R03": {"source": "g2/run03"}}}}
    k03e4 = (("V1-O-cuboid", "K03/E4"),)

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

    def _v(run, a5_subs, a0="PASS"):
        # headline mirrors the real judge: names every failing sub-check
        if a5_subs is None:
            a5_gate = "NOT_EVALUATED"
        else:
            failing = ["%s=%s" % (n, sc["state"])
                       for n, sc in sorted(a5_subs.items())
                       if sc["state"] != "PASS"]
            a5_gate = "; ".join(failing) if failing else "PASS"
        return {"archive": "/e/%s" % run,
                "gates": {"A0": a0, "A1": "PASS", "A5": a5_gate},
                "gates_detail": ({} if a5_subs is None
                                 else {"A5": a5_subs})}

    def _gate(run02_subs, run03_subs=_subs(), run03_a0="PASS"):
        vs = {"g2/run02": _v("g2/run02", run02_subs),
              "g2/run03": _v("g2/run03", run03_subs, a0=run03_a0)}
        return _a5_gate(base_map, vs)

    results = {
        "a_global_binds": _gate(_subs(regression="FAIL: x"))[0] != "PASS",
        "b_still_ours_binds": _gate(
            _subs(fail_ids=(("V1-O-cuboid", "K05"),)))[0] != "PASS",
        "c_superseded_passes": _gate(_subs(fail_ids=k03e4))[0] == "PASS",
        "c2_invalid_superseder_binds": _gate(
            _subs(fail_ids=k03e4), run03_a0="NOT_EVALUATED")[0] != "PASS",
        "e_local_plus_global_rejected": _gate(
            _subs(fail_ids=k03e4,
                  regression="FAIL: missing TEST_SELECTION"))[0] != "PASS",
        "f_missing_detail_binds": _gate(None)[0] != "PASS",
        "s1_missing_subcheck_binds": _gate(
            {k: v for k, v in _subs(fail_ids=k03e4).items()
             if k != "regression"})[0] != "PASS",
        "s2_pass_with_issues_binds": _gate(
            {**_subs(), "ledger": {"scope": "global", "state": "PASS",
                                   "issues": [{"x": 1}]}})[0] != "PASS",
        "s3_mislabelled_scope_binds": _gate(
            {**_subs(fail_ids=k03e4), "fail_evidence": {
                "scope": "case-local", "state": "INCOMPLETE_EVIDENCE",
                "issues": [{"scope": "global", "rule": "regression",
                            "group": "V1-O-cuboid", "case": "K03/E4",
                            "detail": "x"}]}})[0] != "PASS",
    }
    # Content-binding controls (F02, disk-level): a verdict pasted onto
    # different ledger bytes must hard-fail at bind time.
    import tempfile
    from occ_compat.ledger import load_events
    with tempfile.TemporaryDirectory(prefix="mv-selftest-") as tmp:
        root = Path(tmp)
        grp = root / "g2/run01/V2-G-cuboid"
        grp.mkdir(parents=True)
        term = {"event_kind": "case_terminal", "case_id": "R03",
                "subprobe": "", "state": "PASS"}
        for leg in REQUIRED_LEG_EXCERPT:
            row = dict(term, subprobe=leg)
            (grp / "events.jsonl").open("a").write(
                json.dumps(row) + "\n")
        digest = _sha(grp / "events.jsonl")
        good = {"archive": str(root / "g2/run01"),
                "inputs": {"archive": str(root / "g2/run01"),
                           "groups": {"V2-G-cuboid":
                                      {"ledger_sha256": digest}}}}
        ok_bind = isinstance(_bind_verdict(root, good, 0), tuple)
        (grp / "events.jsonl").open("a").write(json.dumps(
            dict(term, subprobe="tampered")) + "\n")
        tampered = False
        try:
            _bind_verdict(root, good, 0)
        except InvalidInput:
            tampered = True
        results["b1_content_binding_passes"] = ok_bind
        results["b2_tampered_ledger_binds"] = tampered
    ok = all(results.values())
    print("merge_verdict selftest: %s" % json.dumps(results))
    return 0 if ok else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reuse-map")
    parser.add_argument("--g2-verdict", action="append", default=[])
    parser.add_argument("--out")
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()
    if args.selftest:
        return _selftest()
    if not args.reuse_map or not args.out or not args.g2_verdict:
        print("merge_verdict: --reuse-map, --out and at least one "
              "--g2-verdict are required (exit 2)")
        return 2
    evidence_root = Path(args.reuse_map).parents[1]
    try:
        reuse = _load(args.reuse_map)
        raw = [_load(v) for v in args.g2_verdict]
    except (OSError, ValueError) as error:
        print("merge_verdict: invalid input: %s (exit 2)" % error)
        return 2
    verdicts_by_run, bound = {}, {}
    try:
        for index, verdict in enumerate(raw):
            run_dir, groups = _bind_verdict(evidence_root, verdict, index)
            if run_dir in verdicts_by_run:
                raise InvalidInput("duplicate run %s" % run_dir)
            verdicts_by_run[run_dir] = verdict
            bound[run_dir] = groups
    except InvalidInput as error:
        print("merge_verdict: %s (exit 2)" % error)
        return 2
    binding = _check_run_binding(reuse, verdicts_by_run)
    if binding:
        print(binding)
        return 2
    # Product/interface consistency (F02 static gap): every contributing
    # run must have judged the SAME PRODUCT commits (G/C1/O_CONTROL).
    # T1 is the tool and may legitimately advance WITHIN the generation
    # — its per-run identity is already bound by each verdict's own
    # freeze digest and judge identity.
    digests = {v.get("inputs", {}).get("product_source_sha256")
               for v in verdicts_by_run.values()}
    if None in digests or len(digests) > 1:
        print("merge_verdict: contributing runs bind different (or "
              "unbound) product source digests (exit 2)")
        return 2

    gates = {}
    gates["A0"] = _worst([v["gates"].get("A0", "NOT_EVALUATED: absent")
                          for v in verdicts_by_run.values()])
    gates["A1"] = _worst([v["gates"].get("A1", "NOT_EVALUATED: absent")
                          for v in verdicts_by_run.values()])
    for gate, groups, required in (
            ("A2", EXPECTED_V2G_GROUPS, V2_REQUIRED),
            ("A3", EXPECTED_V1_GROUPS, V1_REQUIRED)):
        failed, not_hit = _scope(reuse, bound, set(verdicts_by_run),
                                 groups, required)
        if failed:
            gates[gate] = "FAIL: %s" % failed[:4]
        elif not_hit:
            gates[gate] = "NOT_EVALUATED: %s" % not_hit[:6]
        else:
            gates[gate] = "PASS"
    failed, not_hit = [], []
    for groups, required in ((EXPECTED_V1_O_GROUPS, V1_REQUIRED),
                             (EXPECTED_V2O_GROUPS, V2_REQUIRED)):
        f, n = _scope(reuse, bound, set(verdicts_by_run), groups, required)
        failed += f
        not_hit += n
    gates["A4"] = ("FAIL: %s" % failed[:4] if failed else
                   "NOT_EVALUATED: %s" % not_hit[:6] if not_hit
                   else "PASS")
    gates["A5"], excused = _a5_gate(reuse, verdicts_by_run)

    verdict = {
        "schema": "g2-merged-verdict-v5",
        "policy": ("content-bound per-verdict ledgers (SHA256), per-leg "
                   "frozen subprobe evaluation with FAIL dominance, "
                   "validated structured A5 with explicit local "
                   "supersession only"),
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


if __name__ == "__main__":
    sys.exit(main())
