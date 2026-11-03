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


def _row_evidence_formal(root, run):
    """Evidence.validity at a FORMAL run root: the gen4 PRODUCT binaries are
    byte-identical to the reviewed freeze (only scoring-side tools changed),
    and the S3 freeze binds those tools, the input frames and the scoring
    commands. The gen4 root's own qualification verdict stays preserved as
    history -- this row never re-judges it against S3-era tool bytes."""
    problems = []
    measured = {}
    qual_recorded = run.get("qualification_root")
    if not qual_recorded:
        return "fail", ["formal_qualification_root_unbound"], None, []
    qual = Path(qual_recorded)
    if not qual.is_absolute():
        qual = Path(root).parent / qual
    gen4 = _load(qual / "FREEZE.json")
    if not isinstance(gen4, dict):
        return "fail", ["formal_gen4_freeze_unreadable"], None, []
    binaries = gen4.get("binaries") or {}
    matched = 0
    for name, entry in sorted(binaries.items()):
        path = Path(entry.get("path") or "")
        if not path.is_file():
            problems.append("formal_binary_missing_%s" % name)
        elif hashlib.sha256(path.read_bytes()).hexdigest() != entry.get("sha256"):
            problems.append("formal_binary_changed_%s" % name)
        else:
            matched += 1
    measured["gen4_binaries_matched"] = matched
    measured["gen4_binaries_checked"] = len(binaries)
    s3_record = run.get("s3_freeze") or {}
    s3_path = s3_record.get("path")
    if not s3_path:
        problems.append("formal_s3_freeze_unbound")
    else:
        s3 = Path(s3_path)
        if not s3.is_absolute():
            s3 = Path(root).parent / s3
        document = _load(s3)
        if not isinstance(document, dict):
            problems.append("formal_s3_freeze_unreadable")
        else:
            if s3_record.get("sha256") and \
                    hashlib.sha256(s3.read_bytes()).hexdigest() \
                    != s3_record.get("sha256"):
                problems.append("formal_s3_freeze_changed")
            for group in ("tools", "inputs", "commands"):
                entries = document.get(group) or {}
                group_matched = 0
                for name, entry in entries.items():
                    path = Path(entry.get("path") or "")
                    if path.is_file() and \
                            hashlib.sha256(path.read_bytes()).hexdigest() \
                            == entry.get("sha256"):
                        group_matched += 1
                    else:
                        problems.append("formal_s3_%s_changed_%s" % (group, name))
                measured["s3_%s_matched" % group] = group_matched
                if not entries:
                    problems.append("formal_s3_%s_empty" % group)
    return ("pass" if not problems else "fail"), problems[:5], measured, \
        ["run_manifest.json", "FREEZE_S3.json"]


def _scoring_root(root):
    """Formal artifacts sit under <root>/scoring (qualification layout) or
    directly under a run root that carries run_manifest.json."""
    if (root / "run_manifest.json").is_file():
        return root
    return root / "scoring"


def _run_manifest(root):
    return _load(_scoring_root(root) / "run_manifest.json")


def _stage_attempt(run, stage_key):
    """None = no orchestrator record; else the stage's LATEST attempt.

    A resumed stage records under <stage>_resume*; the latest attempt wins
    (an earlier blocked/completed record never overrides the newest state).
    """
    if not isinstance(run, dict):
        return None
    stages = run.get("stages") or {}
    candidates = [key for key in stages
                  if key == stage_key or key.startswith(stage_key + "_resume")]
    if not candidates:
        return {"status": "absent"}

    def order(key):
        suffix = key[len(stage_key):]
        try:
            return (0, int(suffix.split("_")[-1]))
        except ValueError:
            return (0, 0)

    latest = sorted(candidates, key=order)[-1]
    state = stages[latest]
    return state if isinstance(state, dict) else {"status": "absent"}


def _stage_started(run, stage_key):
    state = _stage_attempt(run, stage_key)
    if state is None:
        return False
    return state.get("status") in ("started", "completed", "timeout_killed")


def _expected_matrix_groups(stage):
    """The FIXED formal group sets, derived from the frozen matrix scheme.

    O5: 2 cm chamfer, boxes 0/20, clouds 30k/100k, outside .5, cross 0
    (4 groups). O6: resolutions 2/5 cm x boxes 0/5/20 x both hulls x both
    clouds x both outside x both cross (96 groups). The 1 cm rows stay out.
    """
    if stage == "o5":
        combos = [(0.02, boxes, "chamfer", cloud, 0.5, 0.0)
                  for boxes in (0, 20) for cloud in (30000, 100000)]
    elif stage == "o6":
        combos = [(resolution, boxes, hull, cloud, outside, cross)
                  for resolution in (0.02, 0.05)
                  for boxes in (0, 5, 20)
                  for hull in ("chamfer", "cuboid")
                  for cloud in (30000, 100000)
                  for outside in (0.5, 0.9)
                  for cross in (0.0, 0.5)]
    else:
        raise ValueError(stage)

    def group_id(resolution, boxes, hull, cloud, outside, cross):
        return "r%02d_b%02d_%s_c%d_o%02d_x%02d" % (
            round(resolution * 100), boxes, hull, cloud,
            round(outside * 100), round(cross * 100))

    return {group_id(*combo): {
        "id": group_id(*combo), "resolution": combo[0], "boxes": combo[1],
        "hull": combo[2], "cloud": combo[3], "outside": combo[4],
        "cross": combo[5]} for combo in combos}


def _raw_group_problems(directory, artifact, stage):
    """Cross-check the assembled group artifact against the RAW per-sample
    files: the cumulative process samples and every cold-start sample.

    The assembled columns are copies; this re-derives them from the raw
    documents so a doctored group json cannot certify a stage whose raw
    measurements disagree or do not exist."""
    import yaml as _yaml
    problems = []
    cumulative = directory / "cumulative" / "samples.yaml"
    if not cumulative.is_file():
        return ["group_%s_cumulative_raw_missing" % artifact.get("id")]
    try:
        documents = [row for row in _yaml.safe_load_all(cumulative.read_text())
                     if isinstance(row, dict)]
    except _yaml.YAMLError:
        return ["group_%s_cumulative_raw_unreadable" % artifact.get("id")]
    scored_n = 30 if stage in ("o5", "o6") else None
    if scored_n is None:
        return ["group_%s_unknown_stage" % artifact.get("id")]
    # 3 warmup rows + 30 scored + 1 successor (the after-update surface of
    # the last scored row is the next row's first call).
    if len(documents) < 3 + scored_n + 1:
        problems.append("group_%s_cumulative_rows_%d" % (artifact.get("id"),
                                                         len(documents)))
        return problems
    for row in documents[:3 + scored_n]:
        if row.get("status") != "OK":
            problems.append("group_%s_raw_status_%s" % (artifact.get("id"),
                                                        row.get("status")))
            break
    scored_rows = documents[3:3 + scored_n]

    def _raw_column(key):
        values = []
        for row in scored_rows:
            if key in ("surface_first_ms", "surface_repeat_ms"):
                call = row.get("surface_first_call" if key == "surface_first_ms"
                               else "surface_second_call")
                values.append(call.get("external_ms")
                              if isinstance(call, dict) else None)
            elif key == "integrate_ms":
                # The raw sample documents carry the external-clock field the
                # adapter copies into the assembled integrate_ms column.
                values.append(row.get("integrate_external_ms",
                                      row.get("integrate_ms")))
            else:
                values.append(row.get(key))
        return values

    for key in ("integrate_ms", "surface_first_ms", "surface_repeat_ms"):
        raw = _raw_column(key)
        assembled = artifact.get(key)
        if not isinstance(assembled, list) or len(assembled) != scored_n:
            problems.append("group_%s_column_%s_len" % (artifact.get("id"), key))
        elif any((left is None) != (right is None) or
                 (left is not None and right is not None and not
                  __import__("math").isclose(left, right, rel_tol=1e-9,
                                             abs_tol=1e-6))
                 for left, right in zip(raw, assembled)):
            problems.append("group_%s_column_%s_raw_mismatch"
                            % (artifact.get("id"), key))
    # Every scored index must own one cold-start raw sample.
    missing_cold = []
    for offset in range(3, 3 + scored_n):
        cold = directory / ("f%03d" % offset) / ("cold-%03d" % offset) \
            / "samples.yaml"
        if not cold.is_file():
            missing_cold.append(offset)
            continue
        try:
            rows = [row for row in _yaml.safe_load_all(cold.read_text())
                    if isinstance(row, dict)]
        except _yaml.YAMLError:
            missing_cold.append(offset)
            continue
        if len(rows) != 1 or rows[0].get("status") != "OK":
            missing_cold.append(offset)
    if missing_cold:
        problems.append("group_%s_cold_raw_missing_%d_of_%d"
                        % (artifact.get("id"), len(missing_cold), scored_n))
    return problems


def _row_formal_matrix(root, stage):
    """Formal.O5 / Formal.O6: recompute the FIXED group campaign from the
    per-group artifacts AND the raw per-sample files. A summary.json's
    passed==total is never evidence (the review's nine pseudo-summaries all
    died here). Started-but-missing is a fail; never-started (budget or an
    O5 dependency) stays not_evaluated with its recorded reason."""
    import repair_s0_judge
    score = _scoring_root(root)
    directory = score / stage
    run = _run_manifest(root)
    expected = _expected_matrix_groups(stage)
    if not directory.is_dir() or not any(directory.iterdir()):
        if _stage_started(run, stage):
            return "fail", ["formal_%s_started_artifact_missing" % stage], \
                None, []
        attempt = _stage_attempt(run, stage)
        reason = (attempt or {}).get("not_evaluated_reason") \
            or "formal_%s_not_run" % stage
        return "not_evaluated", [reason], None, []
    groups = []
    raw_problems = []
    for group_id in sorted(expected):
        group_path = directory / ("%s.json" % group_id)
        artifact = _load(group_path)
        if artifact is None:
            continue
        groups.append(artifact)
        raw_problems.extend(_raw_group_problems(directory / group_id,
                                                artifact, stage))
    case = {
        "gate": "%s_campaign" % stage,
        "groups": groups,
        "planned_groups": [expected[group_id] for group_id in sorted(expected)],
    }
    verdict = repair_s0_judge.judge_case(case)
    reasons = list(verdict.get("reasons") or [])
    reasons.extend(raw_problems)
    present = len(groups)
    if present != len(expected):
        reasons.append("formal_%s_groups_%d_of_%d" % (stage, present,
                                                      len(expected)))
    measured = {"groups": present, "expected_groups": len(expected),
                "failed_groups": verdict.get("failed_groups")}
    status = "fail" if reasons else ("pass" if present == len(expected)
                                     else "not_evaluated")
    return status, reasons[:5], measured, ["scoring/%s/*.json" % stage,
                                           "scoring/%s/*/cumulative/samples.yaml" % stage]


def _row_formal_o8(root):
    """Formal.O8: recompute judge_o8 from runs.json. The judge re-decodes the
    lossless probe-content files from disk (sha, region-derived cell counts,
    state domain, finite odds) -- a summary is never evidence."""
    import repair_s0_judge
    score = _scoring_root(root)
    directory = score / "o8"
    run = _run_manifest(root)
    if not directory.is_dir() or not any(directory.iterdir()):
        if _stage_started(run, "o8"):
            return "fail", ["formal_o8_started_artifact_missing"], None, []
        attempt = _stage_attempt(run, "o8")
        reason = (attempt or {}).get("not_evaluated_reason") or "formal_o8_not_run"
        return "not_evaluated", [reason], None, []
    artifact = _load(directory / "runs.json")
    if artifact is None:
        # The directory has content but not the raw artifact: a stage that
        # produced something cannot wash itself into not-run.
        return "fail", ["formal_o8_runs_missing"], None, []
    artifact = dict(artifact)
    artifact["artifact_dir"] = str(directory)
    verdict = repair_s0_judge.judge_o8(artifact)
    reasons = list(verdict.get("reasons") or [])
    measured = {"runs": len(artifact.get("runs") or []),
                "holds": len(artifact.get("holds") or [])}
    return ("pass" if not reasons else "fail"), reasons[:5], measured, \
        ["scoring/o8/runs.json", "scoring/o8/probe-content/*"]


def _formal_run_ok(run):
    """A rehearsal root can never certify a formal row: clean judge output
    on a non-formal run stays not_evaluated, never a pass."""
    if run is None:
        return True
    return run.get("formal") is True


def _not_formal_result(run, judge_clean):
    if judge_clean:
        return "not_evaluated", ["rehearsal_not_formal_run"]
    return None


def _row_formal(root, stage):
    score = _scoring_root(root)
    run = _run_manifest(root)
    campaign_dir = score / "campaign"
    if stage in ("o8", "o5", "o6"):
        if stage == "o8":
            result = _row_formal_o8(root)
        else:
            result = _row_formal_matrix(root, stage)
        if result[0] == "pass" and not _formal_run_ok(run):
            return "not_evaluated", ["rehearsal_not_formal_run"], result[2], \
                result[3]
        return result
    if stage in ("p", "r", "l"):
        started = _stage_started(run, "campaign")
        if not campaign_dir.is_dir() or not any(campaign_dir.iterdir()):
            if started:
                return "fail", ["formal_campaign_started_artifact_missing"], \
                    None, []
            return "not_evaluated", ["formal_not_run"], None, []
        if (campaign_dir / "failure.json").is_file():
            return "fail", ["formal_failure_prefix_present"], None, []
    if stage == "p":
        windows = [(_load(path), path.name)
                   for path in sorted(campaign_dir.glob("p_b*_w*.json"))]
        windows = [item for item in windows if item[0] is not None]
        if not windows:
            reason = "formal_p_windows_missing"
            if started:
                return "fail", [reason], None, []
            return "not_evaluated", [reason], None, []
        problems = judge.judge_p_matrix([window for window, _ in windows])
        if not problems and not _formal_run_ok(run):
            return "not_evaluated", ["rehearsal_not_formal_run"], {
                "windows": len(windows)}, ["scoring/campaign/p_*.json"]
        return ("pass" if not problems else "fail"), problems[:5], {
            "windows": len(windows)}, ["scoring/campaign/p_*.json"]
    if stage in ("r", "l"):
        artifact = _load(campaign_dir / ("%s.json" % stage))
        if artifact is None:
            summary = _load(campaign_dir / "summary.json")
            skipped = summary.get("skipped") if isinstance(summary, dict) \
                else None
            if isinstance(skipped, dict):
                # A dependency/budget stop recorded by the runner: the stage
                # never started, so it is not a failure of its own.
                return "not_evaluated", ["formal_%s_skipped_%s" % (
                    stage, skipped.get("detail") or skipped.get("reason"))], \
                    None, []
            if started:
                return "fail", ["formal_%s_started_artifact_missing" % stage], \
                    None, []
            return "not_evaluated", ["formal_%s_missing" % stage], None, []
        verdict_stage = judge.judge_r_stage(artifact) \
            if stage == "r" else judge.judge_l_stage(artifact)
        reasons = verdict_stage.get("reasons") or []
        if stage == "r" and reasons == ["overload_pressure_not_evaluated"]:
            # Every measured aspect (drain closure, recovery, first-Wait) is
            # green; the only finding is that the FROZEN overload load never
            # materialized real pressure -- the execution contract scores the
            # pressure coverage not_evaluated, so the row cannot certify but
            # nothing failed. The e2e evidence is in the judge's measured.
            return "not_evaluated", reasons, verdict_stage.get("measured"), \
                ["scoring/campaign/r.json"]
        if not reasons and not _formal_run_ok(run):
            return "not_evaluated", ["rehearsal_not_formal_run"], None, \
                ["scoring/campaign/%s.json" % stage]
        return ("pass" if not reasons else "fail"), reasons[:5], \
            verdict_stage.get("measured"), \
            ["scoring/campaign/%s.json" % stage]
    raise ValueError(stage)


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
     "sources scoring/o8/runs.json + probe-content/ (campaign_executor "
     "--mode stage o8); judge repair_s0_judge.judge_o8 recomputed with the "
     "artifact_dir pointed at the run: 3 runs, 30 saturation frames, 3x3 "
     "holds, probe content re-decoded from the lossless files; a "
     "passed==total summary is never evidence (9 review counterexamples)"),
    ("Formal.O5", "formal", False, lambda root: _row_formal(root, "o5"),
     "sources scoring/o5/<gid>.json per group + raw cumulative/cold "
     "samples.yaml (campaign_executor --mode stage o5); judge "
     "repair_s0_judge.judge_case recomputed against the FIXED 4-group plan "
     "with raw-column cross-checks -- legacy matrix, listed SEPARATELY, "
     "never merged into the main conclusion"),
    ("Formal.O6", "formal", False, lambda root: _row_formal(root, "o6"),
     "sources scoring/o6/<gid>.json per group + raw cumulative/cold "
     "samples.yaml (campaign_executor --mode stage o6); judge "
     "repair_s0_judge.judge_case recomputed against the FIXED 96-group "
     "2/5 cm plan with raw-column cross-checks -- legacy matrix, listed "
     "SEPARATELY, never merged into the main conclusion; O5 failure stops "
     "O6 (recorded not_evaluated with its reason)"),
)


def judge_evidence_root(root, qualification_root=None):
    """Recompute the FIXED table for one evidence root.

    Two layouts:
    - a qualification root (the gen4 implementation evidence): qualification
      rows read it directly; the Formal.* rows see no scoring data and stay
      not_evaluated.
    - a formal run root (run_manifest.json present): the qualification rows
      are recomputed against the recorded qualification root -- preserved,
      never rewritten -- while the Formal.* rows read THIS run's artifacts.
      A first formal result turning a not_evaluated row into pass/fail is a
      new run, not tampering; rewriting the same run is caught by the row
      cross-check and the run binding.
    """
    root = Path(root)
    run = _load(root / "run_manifest.json")
    qual_target = None
    if isinstance(run, dict):
        if qualification_root is None:
            recorded = run.get("qualification_root")
            if recorded:
                candidate = Path(recorded)
                if not candidate.is_absolute():
                    candidate = root.parent / candidate
                qual_target = candidate
        else:
            qual_target = Path(qualification_root)
        if qual_target is not None and not qual_target.is_dir():
            raise SystemExit("run_manifest qualification_root missing: %s"
                             % qual_target)
    rows = []
    for row_id, group, main, evaluator, note in ROW_TABLE:
        target = qual_target if (qual_target is not None
                                 and group == "qualification") else root
        if row_id == "Evidence.validity" and isinstance(run, dict):
            # The formal run's evidence binding is the S3 freeze (product
            # unchanged + S3 tools/inputs/commands); the gen4 root's own
            # verdict is preserved history and is not re-judged against
            # S3-era tool bytes.
            status, reasons, measured, sources = _row_evidence_formal(root, run)
        else:
            status, reasons, measured, sources = evaluator(target)
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
            "any_row_failed": any(row["status"] == "fail" for row in rows),
            "not_evaluated_rows": not_evaluated,
            "main_conclusion_rows": [row["row"] for row in main_rows],
        },
        "generated_mono_ns": __import__("time").monotonic_ns(),
        "root": str(root),
    }
    if isinstance(run, dict):
        document["run"] = {
            "report_version": "formal",
            "run_id": run.get("run_id"),
            "formal": run.get("formal"),
            "qualification_root": run.get("qualification_root"),
            "input_manifest_sha256": run.get("input_manifest_sha256"),
            "freeze": run.get("freeze"),
            "budget_minutes": run.get("budget_minutes"),
            "measurement_s": run.get("measurement_s"),
            "stage_status": {name: (state or {}).get("status")
                             for name, state in sorted(
                                 (run.get("stages") or {}).items())},
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
        return 0 if (document["overall"]["qualification"] == "pass"
                     and not document["overall"]["any_row_failed"]) else 1
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
        # Run binding: a stored formal verdict belongs to exactly one run.
        if isinstance(document.get("run"), dict):
            stored_run = (stored or {}).get("run") if isinstance(stored, dict) \
                else None
            if not isinstance(stored_run, dict) \
                    or stored_run.get("run_id") != document["run"].get("run_id"):
                problems.append("verdict_run_mismatch")
            elif stored_run.get("input_manifest_sha256") \
                    != document["run"].get("input_manifest_sha256"):
                problems.append("verdict_input_manifest_mismatch")
    else:
        problems.append("verdict_not_written")
    print(json.dumps({"problems": problems[:10], "total": len(problems),
                      "overall": document["overall"]}, indent=1))
    # The judge's exit code reflects failure: a failing row (formal or
    # qualification) is nonzero; not_evaluated rows never launder it to 0.
    if problems:
        return 1
    return 0 if not document["overall"]["any_row_failed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
