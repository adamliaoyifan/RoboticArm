#!/usr/bin/env python3
"""The FIXED acceptance verdict: every row of the frozen table, recomputed
from the artifact files under an evidence root -- never from a stored
summary or a self-reported flag.

Row status:
  pass           the row's judge found no problems and its sources exist
  fail           the judge named problems
  not_evaluated  the sources are absent (no formal scoring directory, a
                 budget-exhausted stage) -- NEVER a pass; the dispatch keeps
                 the formal long windows closed until S3 opens

A stored verdict.json is cross-checked row by row on re-run; a mismatch is
itself a finding (verdict_row_mismatch_<row>). The MAIN conclusion covers
F/C/P/H/R/L/O8/O9 + Evidence; the legacy O5/O6 matrices stay separate rows
and never merge into it.
"""
from __future__ import annotations

import hashlib
import json
import subprocess
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))

import freespace_contract as contract  # noqa: E402
import freespace_judge as judge  # noqa: E402

BOUNDED_MARKER = "short_sample_not_formal"


def _bounded_clean(reasons):
    return reasons == [] or reasons == [BOUNDED_MARKER]


def _load(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None


def _min_over(snapshot, field):
    samples = snapshot.get("resource_samples")
    if isinstance(samples, list) and samples:
        values = [sample.get(field) for sample in samples
                  if isinstance(sample, dict)]
        if values and None not in values:
            return min(values)
    return snapshot.get(field)


# --- row evaluators ----------------------------------------------------------

def _row_f_matrix(root):
    summary = _load(root / "f_matrix" / "summary.json")
    cases_dir = root / "f_matrix" / "cases"
    # each case is a DIRECTORY holding responses.yaml (the disk judge's shape)
    cases = sorted(path for path in cases_dir.iterdir()
                   if path.is_dir()) if cases_dir.is_dir() else []
    if summary is None:
        return "not_evaluated", ["f_matrix_summary_missing"], None, []
    problems = judge.judge_f_summary(summary)
    if len(cases) != contract.F_CASES:
        problems.append("f_matrix_case_files_%d" % len(cases))
    return ("pass" if not problems else "fail"), problems, {
        "total": summary.get("total"), "passed": summary.get("passed"),
        "case_files": len(cases)}, ["f_matrix/summary.json", "f_matrix/cases/"]


def _row_behavior(root):
    results = _load(root / "behavior_results.json")
    if results is None:
        return "not_evaluated", ["behavior_results_missing"], None, []
    problems = []
    for name in ("observer_freespace_behavior", "observer_repair_s1_behavior"):
        if results.get(name) != 0:
            problems.append("%s_exit_%s" % (name, results.get(name)))
    return ("pass" if not problems else "fail"), problems, results, \
        ["behavior_results.json"]


def _row_seven_badcases(root):
    tool = Path(__file__).resolve().parent / "audit_g4_red.py"
    out = root / "red_baseline"
    if not tool.is_file():
        return "fail", ["audit_tool_missing"], None, []
    proc = subprocess.run([sys.executable, str(tool), "after", str(out)],
                          capture_output=True, text=True)
    results = _load(out / "RESULTS_after.json")
    if proc.returncode != 0 or results is None:
        problems = ["audit_exit_%d" % proc.returncode]
        if results is None:
            problems.append("audit_results_missing")
        return "fail", problems, None, ["red_baseline/RESULTS_after.json"]
    accepted = [row["name"] for row in results
                if not row.get("control") and row.get("accepted")]
    controls = [row["name"] for row in results
                if row.get("control") and not row.get("accepted")]
    problems = []
    if accepted:
        problems.append("mutants_accepted:%s" % accepted[:3])
    if controls:
        problems.append("controls_rejected:%s" % controls[:2])
    return ("pass" if not problems else "fail"), problems, {
        "mutants_rejected": sum(1 for row in results
                                if not row.get("control")
                                and not row.get("accepted")),
        "controls_green": sum(1 for row in results
                              if row.get("control") and row.get("accepted"))}, \
        ["red_baseline/RESULTS_after.json"]


def counterexample_problems(document):
    """Re-judge the controlled leak: a deliberately retained lease must be
    REPORTED as residue in the lease owner class (block bytes AND directory
    entries above the baseline) while every other class stays at the
    baseline, and releasing it must return every class to the baseline.
    This is the checker-discrimination proof -- one controlled
    counterexample, not an ever-growing set."""
    if not isinstance(document, dict):
        return ["counterexample_not_a_document"]
    problems = []
    baseline = document.get("baseline") or {}
    residue = document.get("residue_with_lease") or {}
    recovered = document.get("after_release") or {}
    for name, snapshot in (("baseline", baseline), ("residue", residue),
                           ("recovered", recovered)):
        if not snapshot.get("resource_samples"):
            problems.append("counterexample_%s_samples_missing" % name)
    for field in ("lease_view_block_bytes", "lease_view_directory_entries"):
        base = _min_over(baseline, field)
        value = _min_over(residue, field)
        if not isinstance(base, int) or not isinstance(value, int) \
                or value <= base:
            problems.append("counterexample_residue_not_reported_%s"
                            % field)
    # While the lease is deliberately retained, every OTHER class must sit
    # at the baseline (the residue is attributed, not smeared), and after
    # the release everything -- lease class included -- returns.
    for field in judge.OWNERSHIP_RECONCILE_FIELDS:
        if field.startswith("lease_view"):
            continue
        if _min_over(residue, field) != _min_over(baseline, field):
            problems.append("counterexample_residue_smear_%s" % field)
        if _min_over(recovered, field) != _min_over(baseline, field):
            problems.append("counterexample_not_recovered_%s" % field)
    for field in ("lease_view_block_bytes", "lease_view_directory_entries"):
        if _min_over(recovered, field) != _min_over(baseline, field):
            problems.append("counterexample_lease_not_recovered_%s" % field)
    for name, snapshot in (("baseline", baseline), ("residue", residue),
                           ("recovered", recovered)):
        if any(sample.get("uncovered_allocations")
               for sample in snapshot.get("resource_samples") or []
               if isinstance(sample, dict)):
            problems.append("counterexample_%s_uncovered" % name)
    return problems


def _row_counterexample(root):
    document = _load(root / "counterexample" / "counterexample.json")
    if document is None:
        return "not_evaluated", ["counterexample_missing"], None, []
    problems = counterexample_problems(document)
    return ("pass" if not problems else "fail"), problems, {
        "phases": [name for name in document
                   if isinstance(document.get(name), dict)]}, \
        ["counterexample/counterexample.json"]


def _rehearsal_p(root):
    """Returns (missing, problems, measured, sources)."""
    directory = root / "rehearsal" / "campaign"
    if not directory.is_dir():
        return True, ["rehearsal_missing"], None, []
    paths = sorted(directory.glob("p_b*_w*.json"))
    windows = [(_load(path), path.name) for path in paths]
    if not windows:
        return True, ["rehearsal_p_windows_missing"], None, []
    problems, boxes, measured = [], set(), []
    for window, name in windows:
        if window is None:
            problems.append("window_unreadable_%s" % name)
            continue
        boxes.add(window.get("box_count"))
        verdict = judge.judge_p_window(window)
        measured.append(verdict.get("measured"))
        if not _bounded_clean(verdict.get("reasons")):
            problems.append("%s:%s" % (name, verdict["reasons"][:2]))
    if boxes != set(contract.P_BOX_COUNTS):
        problems.append("rehearsal_p_box_coverage_%s" % sorted(boxes, key=str))
    return False, problems, {
        "windows": len(windows), "boxes": sorted(boxes, key=str),
        "window_measured": measured}, ["rehearsal/campaign/p_*.json"]


def _row_p_contract(root):
    missing, problems, measured, sources = _rehearsal_p(root)
    if missing:
        return "not_evaluated", problems, None, []
    return ("pass" if not problems else "fail"), problems, measured, sources


def _row_h_scheduling(root):
    missing, problems, measured, sources = _rehearsal_p(root)
    if missing:
        return "not_evaluated", problems, None, []
    # H reads the same records (planned vs started/published); a scheduling
    # breach lands in the shared reasons -- keep the rows separate so the
    # verdict names the right gate.
    sched = [problem for problem in problems
             if "sched" in problem or "send_late" in problem]
    others = [problem for problem in problems if problem not in sched]
    status = "pass" if not problems else "fail"
    return status, (sched + others)[:5], measured, sources


_L_ASPECTS = {
    "read": ("read_round", "l_real_read_rounds", "read_failed"),
    "reset": ("reset_round",),
    "ownership": ("class_", "_residual", "split_inconsistent",
                  "samples_missing", "uncovered"),
    "rss": ("rss_", "cache_peak", "health_"),
}


def _row_l_aspect(root, aspect):
    artifact = _load(root / "rehearsal" / "campaign" / "l.json")
    if artifact is None:
        return "not_evaluated", ["rehearsal_l_missing"], None, []
    verdict = judge.judge_l_stage(artifact)
    reasons = verdict.get("reasons") or []
    if not _bounded_clean(reasons):
        mine = [reason for reason in reasons
                if any(token in reason for token in _L_ASPECTS[aspect])]
        others = [reason for reason in reasons
                  if reason not in mine and reason != BOUNDED_MARKER]
        problems = mine[:5] + (["other_l_findings:%s" % others[:2]]
                               if others else [])
        return "fail", problems, verdict.get("measured"), \
            ["rehearsal/campaign/l.json"]
    return "pass", [], verdict.get("measured"), ["rehearsal/campaign/l.json"]


def _row_r(root):
    artifact = _load(root / "rehearsal" / "campaign" / "r.json")
    if artifact is None:
        return "not_evaluated", ["rehearsal_r_missing"], None, []
    verdict = judge.judge_r_stage(artifact)
    reasons = verdict.get("reasons") or []
    if _bounded_clean(reasons):
        return "pass", [], verdict.get("measured"), ["rehearsal/campaign/r.json"]
    return "fail", reasons[:5], verdict.get("measured"), \
        ["rehearsal/campaign/r.json"]


def load_o9_pairs(root):
    """The raw per-pair samples.yaml loader shared by the disk judge and
    this verdict (single source for the O9 artifact shape)."""
    directory = root / "o9"
    pairs = []
    index = 0
    while True:
        sides = {}
        for side in ("on", "off"):
            path = directory / ("pair_%02d_%s" % (index, side)) / "samples.yaml"
            if not path.is_file():
                return pairs
            document = next(yaml.safe_load_all(path.read_text()))
            first = document["surface_first_call"]
            sides[side] = {
                "roi": float(document["roi_ms"]),
                "integrate_30k": float(document["integrate_external_ms"]),
                "surface_first": float(first["external_ms"]),
                "freespace_full": float(document["freespace_full_external_ms"]),
            }
            sides["freespace_" + side] = document["freespace_content_sha256"]
            sides["content_" + side] = document["roi_surface_content_sha256"]
        pairs.append(sides)
        index += 1


def _row_o9(root):
    result = _load(root / "o9" / "o9_result.json")
    pairs = load_o9_pairs(root)
    if result is None or not pairs:
        return "not_evaluated", ["o9_result_missing"], None, []
    problems = judge.judge_o9_samples(pairs, result)
    return ("pass" if not problems else "fail"), problems[:5], {
        "pairs": len(pairs), "ratios": (result.get("ratios") or {})}, \
        ["o9/o9_result.json", "o9/pair_*/samples.yaml"]


def verify_freeze_files(root):
    """Byte-verify every FREEZE group entry against the tree (the COMMANDS
    entry additionally re-measures HEAD and tree cleanliness -- those need
    git and stay there)."""
    freeze = _load(root / "FREEZE.json")
    if freeze is None:
        return ["freeze_missing"], 0, 0
    problems = []
    checked = 0
    matched = 0
    for group in ("binaries", "tools", "fixtures", "inputs", "commands"):
        entries = freeze.get(group)
        if not entries:
            problems.append("freeze_group_empty_%s" % group)
            continue
        if isinstance(entries, dict) and "path" in entries:
            entries = {"self": entries}
        for name, entry in entries.items():
            path = Path(entry["path"])
            checked += 1
            if not path.is_file() \
                    or hashlib.sha256(path.read_bytes()).hexdigest() \
                    != entry["sha256"]:
                problems.append("freeze_mismatch_%s_%s" % (group, name))
            else:
                matched += 1
    return problems, matched, checked


def _row_evidence(root):
    problems, matched, checked = verify_freeze_files(root)
    if problems and problems[0] == "freeze_missing":
        return "not_evaluated", problems, None, []
    return ("pass" if not problems else "fail"), problems[:5], {
        "freeze_matched": matched, "freeze_checked": checked}, \
        ["FREEZE.json", "verdict.json"]


def _row_formal(root, stage):
    campaign_dir = root / "scoring" / "campaign"
    directory = root / "scoring" / stage
    if stage in ("p", "r", "l"):
        if not campaign_dir.is_dir() or not any(campaign_dir.iterdir()):
            return "not_evaluated", ["formal_not_run"], None, []
        if (campaign_dir / "failure.json").is_file():
            return "fail", ["formal_failure_prefix_present"], None, []
    if stage in ("o8", "o5", "o6") and not directory.is_dir():
        return "not_evaluated", ["formal_%s_not_run" % stage], None, []
    if stage == "p":
        windows = [(_load(path), path.name)
                   for path in sorted(campaign_dir.glob("p_b*_w*.json"))]
        windows = [item for item in windows if item[0] is not None]
        if not windows:
            return "not_evaluated", ["formal_p_windows_missing"], None, []
        problems = judge.judge_p_matrix([window for window, _ in windows])
        return ("pass" if not problems else "fail"), problems[:5], {
            "windows": len(windows)}, ["scoring/campaign/p_*.json"]
    if stage == "r":
        artifact = _load(campaign_dir / "r.json")
        if artifact is None:
            return "not_evaluated", ["formal_r_missing"], None, []
        reasons = judge.judge_r_stage(artifact).get("reasons") or []
        return ("pass" if not reasons else "fail"), reasons[:5], None, \
            ["scoring/campaign/r.json"]
    if stage == "l":
        artifact = _load(campaign_dir / "l.json")
        if artifact is None:
            return "not_evaluated", ["formal_l_missing"], None, []
        reasons = judge.judge_l_stage(artifact).get("reasons") or []
        return ("pass" if not reasons else "fail"), reasons[:5], None, \
            ["scoring/campaign/l.json"]
    summary = _load(directory / "summary.json")
    if summary is None:
        return "not_evaluated", ["formal_%s_summary_missing" % stage], None, []
    passed = summary.get("passed")
    total = summary.get("total")
    if not isinstance(passed, int) or not isinstance(total, int) \
            or passed != total:
        return "fail", ["formal_%s_failures_%s" % (stage, passed)], summary, \
            ["scoring/%s/summary.json" % stage]
    return "pass", [], summary, ["scoring/%s/summary.json" % stage]


# The FIXED table: id -> (group, main-conclusion, evaluator, coverage note).
ROW_TABLE = (
    ("F.matrix_360", "qualification", True,
     lambda root: _row_f_matrix(root),
     "sources f_matrix/summary.json+f_matrix/cases/ (freespace_matrix entry); "
     "judge judge_f_summary: 360/360 vs the independent oracle"),
    ("C.behavior_9x30", "qualification", True,
     lambda root: _row_behavior(root),
     "sources behavior_results.json (tests entry runs both C++ binaries); "
     "judge: exit 0 for observer_freespace_behavior (9 classes x 30 reps, "
     "owner-class split included) and observer_repair_s1_behavior"),
    ("C.owner_class_split", "qualification", True,
     lambda root: _row_behavior(root),
     "sources behavior_results.json + counterexample/: the C++ "
     "test_owner_class_split proves lease->client-held->released byte moves; "
     "the live counterexample proves the external checker reports residue"),
    ("G4.seven_review_badcases", "qualification", True,
     lambda root: _row_seven_badcases(root),
     "sources red_baseline/RESULTS_after.json (audit_g4_red.py re-run); "
     "judge: 1938 review mutations all rejected, healthy controls green"),
    ("G4.counterexample_controlled", "qualification", True,
     lambda root: _row_counterexample(root),
     "sources counterexample/counterexample.json (freespace_counterexample "
     "entry); judge counterexample_problems: retained lease reported in its "
     "owner class, release returns every class to the baseline"),
    ("P.record_contract", "qualification", True,
     lambda root: _row_p_contract(root),
     "sources rehearsal/campaign/p_*.json (stage_p); judge judge_p_window "
     "bounded-clean on every window: statuses/identity binding/result "
     "completeness/age/health recomputed per record"),
    ("H.scheduling", "qualification", True,
     lambda root: _row_h_scheduling(root),
     "sources rehearsal/campaign/p_*.json (same records); judge "
     "_ph_boundaries+_latency_gate_problems: sched late P95<=20ms/max<=50ms"),
    ("R.overload_drain_recovery", "qualification", True,
     lambda root: _row_r(root),
     "sources rehearsal/campaign/r.json (stage_r); judge judge_r_stage "
     "bounded-clean: 600 terminal closure, drain deadline from arrivals, "
     "recovery first-Wait chain on its own lease"),
    ("L.read_release_lifecycle_real_1000", "qualification", True,
     lambda root: _row_l_aspect(root, "read"),
     "sources rehearsal/campaign/l.json read_rounds_results (stage_l reader); "
     "judge _ownership_round_problems: every round a REAL acquire/release, "
     "failures listed in read_failed_attempts and never counted"),
    ("L.reset_lifecycle_100", "qualification", True,
     lambda root: _row_l_aspect(root, "reset"),
     "sources rehearsal/campaign/l.json reset_rounds_results (stage_l "
     "resetter); judge: real cross-reset lease, retained/client old blocks "
     "zero right after the reset, delta reconciliation to the pre-round state"),
    ("L.ownership_per_class_reconciliation", "qualification", True,
     lambda root: _row_l_aspect(root, "ownership"),
     "sources read/reset rounds' ownership_after.resource_samples + baseline "
     "(_ownership/_resource_snapshot); judge _class_residual_problems: "
     "per-class min-of-samples == post-warmup baseline, splits sum"),
    ("L.rss_stability", "qualification", True,
     lambda root: _row_l_aspect(root, "rss"),
     "sources rehearsal/campaign/l.json rss_series + health_series; judge: "
     "real-time monotonic samples, buckets dense, RSS<=2GiB, growth<=5%, "
     "composite peak<=512MiB (formal limits certify only in the formal run)"),
    ("O9.four_op_pairs", "qualification", True,
     lambda root: _row_o9(root),
     "sources o9/o9_result.json + pair_*/samples.yaml (s2_o9 entry); judge "
     "judge_o9_samples: 30 pairs, ratios recomputed <=1.10, digests equal"),
    ("Evidence.validity", "qualification", True,
     lambda root: _row_evidence(root),
     "sources FREEZE.json + verdict.json; judge verify_freeze_files: every "
     "group byte-verified, verdict cross-checked row by row"),
    ("Formal.P", "formal", True, lambda root: _row_formal(root, "p"),
     "sources scoring/campaign/p_*.json; judge judge_p_matrix formal"),
    ("Formal.R", "formal", True, lambda root: _row_formal(root, "r"),
     "sources scoring/campaign/r.json; judge judge_r_stage formal"),
    ("Formal.L", "formal", True, lambda root: _row_formal(root, "l"),
     "sources scoring/campaign/l.json; judge judge_l_stage formal"),
    ("Formal.O8", "formal", True, lambda root: _row_formal(root, "o8"),
     "sources scoring/o8/summary.json (campaign_executor --mode stage o8)"),
    ("Formal.O5", "formal", False, lambda root: _row_formal(root, "o5"),
     "sources scoring/o5/summary.json -- legacy matrix, listed SEPARATELY, "
     "never merged into the main conclusion"),
    ("Formal.O6", "formal", False, lambda root: _row_formal(root, "o6"),
     "sources scoring/o6/summary.json -- legacy matrix, listed SEPARATELY, "
     "never merged into the main conclusion"),
)


def judge_evidence_root(root):
    root = Path(root)
    rows = []
    for row_id, group, main, evaluator, note in ROW_TABLE:
        status, reasons, measured, sources = evaluator(root)
        rows.append({"row": row_id, "group": group, "main": main,
                     "status": status, "reasons": reasons or [],
                     "measured": measured, "sources": sources,
                     "coverage": note})
    main_rows = [row for row in rows if row["main"]]
    qualification = [row for row in main_rows
                     if row["group"] == "qualification"]
    formal = [row for row in main_rows if row["group"] == "formal"]
    not_evaluated = [row["row"] for row in rows
                     if row["status"] == "not_evaluated"]
    if formal and any(row["status"] != "not_evaluated" for row in formal):
        formal_status = "pass" if all(row["status"] == "pass"
                                      for row in formal) else "fail"
    else:
        formal_status = "not_evaluated"
    document = {
        "rows": rows,
        "overall": {
            "qualification": "pass" if all(
                row["status"] == "pass" for row in qualification) else "fail",
            "formal": formal_status,
            "not_evaluated_rows": not_evaluated,
            "main_conclusion_rows": [row["row"] for row in main_rows],
        },
        "generated_mono_ns": __import__("time").monotonic_ns(),
        "root": str(root),
    }
    return document


def main():
    if len(sys.argv) < 2:
        print("usage: freespace_verdict.py <evidence_root> [write|check]")
        return 2
    root = Path(sys.argv[1])
    mode = sys.argv[2] if len(sys.argv) > 2 else "check"
    document = judge_evidence_root(root)
    verdict_path = root / "verdict.json"
    if mode == "write":
        root.mkdir(parents=True, exist_ok=True)
        verdict_path.write_text(json.dumps(document, indent=1))
        print(json.dumps(document["overall"], indent=1))
        return 0 if document["overall"]["qualification"] == "pass" else 1
    # check mode: recompute, compare against a stored verdict if present
    problems = []
    if verdict_path.is_file():
        stored = _load(verdict_path)
        for row in document["rows"]:
            stored_row = next((item for item in (stored or {}).get("rows", [])
                               if item.get("row") == row["row"]), None)
            if stored_row is None:
                problems.append("verdict_row_missing_%s" % row["row"])
            elif stored_row.get("status") != row["status"]:
                problems.append("verdict_row_mismatch_%s:%s!=%s"
                                % (row["row"], stored_row.get("status"),
                                   row["status"]))
    else:
        problems.append("verdict_not_written")
    print(json.dumps({"problems": problems[:10], "total": len(problems),
                      "overall": document["overall"]}, indent=1))
    return 0 if not problems else 1


if __name__ == "__main__":
    raise SystemExit(main())
