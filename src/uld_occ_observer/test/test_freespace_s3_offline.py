"""S3 integration-fix offline tests (formal summary judges, report version
switch, unified budget/lock orchestration).

Covers the 2026-11-03 0715 review acceptance:
- the nine pseudo-summary counterexamples (o8/o5/o6 x passed==total of
  0 / true / 30 with no raw data) are all REJECTED -- never a pass;
- full healthy controls for the o5 matrix judge (4 groups, raw cumulative +
  cold samples) pass; short samples and doctored columns fail;
- started-but-missing is a FAIL, never-started stays not_evaluated with its
  recorded reason; the O5 dependency stops O6;
- the formal report version switch: qualification rows are recomputed from
  the recorded qualification root, the verdict is bound to run_id +
  input-manifest SHA, and a same-run tamper is a mismatch;
- the orchestrator: freeze gate refuses changed binaries, the input manifest
  binds frames by SHA, budget gating refuses to start a stage that cannot
  finish, and the single lock holder is the orchestrator itself.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

TEST = Path(__file__).resolve().parent
BENCH = TEST / "benchmark"
sys.path.insert(0, str(BENCH))

import freespace_verdict as verdict  # noqa: E402
import score_orchestrator as orchestrator  # noqa: E402

EVIDENCE = TEST.parents[2] / "docs/status/evidence/occ_freespace_final_20261102" \
           / "implementation_g4"


# --- fixtures -----------------------------------------------------------------

def write_json(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=1))


def run_root(tmp_path, run_id="run01", stages=None, qualification=None,
             with_s3_freeze=False):
    root = tmp_path / ("scoring_%s" % run_id)
    root.mkdir(parents=True, exist_ok=True)
    manifest = {
        "run_id": run_id, "formal": True, "budget_minutes": 480,
        "qualification_root": qualification if qualification is not None
        else "../implementation_g4",
        "stages": stages or {},
    }
    if with_s3_freeze:
        manifest["s3_freeze"] = _make_s3_freeze(tmp_path)
    write_json(root / "run_manifest.json", manifest)
    return root


def _make_s3_freeze(tmp_path):
    """A real-bytes S3 freeze beside the run root: tools and commands point
    at the actual files with their actual SHAs."""
    import hashlib

    def entry(path):
        path = Path(path)
        return {"path": str(path),
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}

    s3_dir = tmp_path / "scoring_s3"
    s3_dir.mkdir(exist_ok=True)
    frames = s3_dir / "inputs"
    frames.mkdir(exist_ok=True)
    (frames / "b00_unit.yaml").write_text("points: []\n")
    (frames / "b20_unit.yaml").write_text("points: []\n")
    commands = EVIDENCE.parent / "scoring_s3" / "COMMANDS.sh"
    freeze = {
        "head": "unit", "candidate": "3bb26935",
        "tools": {
            "score_orchestrator.py": entry(BENCH / "score_orchestrator.py"),
            "freespace_verdict.py": entry(BENCH / "freespace_verdict.py"),
        },
        "inputs": {path.name: entry(path) for path in sorted(frames.glob("*.yaml"))},
        "commands": {"COMMANDS.sh": entry(commands)},
    }
    path = s3_dir / "FREEZE_S3.json"
    path.write_text(json.dumps(freeze, indent=1, sort_keys=True))
    return {"path": str(path),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def started_stages(**overrides):
    stages = {"campaign": {"status": "completed", "exit_code": 0},
              "o8": {"status": "completed", "exit_code": 0},
              "o5": {"status": "completed", "exit_code": 0},
              "o6": {"status": "completed", "exit_code": 0}}
    stages.update(overrides)
    return stages


O5_PARAMS = {
    "r02_b00_chamfer_c30000_o50_x00": (0.02, 0, "chamfer", 30000, 0.5, 0.0),
    "r02_b00_chamfer_c100000_o50_x00": (0.02, 0, "chamfer", 100000, 0.5, 0.0),
    "r02_b20_chamfer_c30000_o50_x00": (0.02, 20, "chamfer", 30000, 0.5, 0.0),
    "r02_b20_chamfer_c100000_o50_x00": (0.02, 20, "chamfer", 100000, 0.5, 0.0),
}


def healthy_group(directory, group_id, integrate_ms=None, scored=30):
    """One healthy group: assembled artifact + raw cumulative + cold runs."""
    resolution, boxes, hull, cloud, outside, cross = O5_PARAMS[group_id]
    if integrate_ms is None:
        # 30k gate P95<=500/max<=1000; 100k gate P95<=2000/max<=5000.
        base = 120.0 if cloud == 30000 else 900.0
        integrate_ms = [base + (index % 5) for index in range(scored)]
    columns = {
        "integrate_ms": integrate_ms,
        "acquire_ms": [8.0 + (index % 3) for index in range(scored)],
        "roi_ms": [9.0 + (index % 3) for index in range(scored)],
        "surface_first_ms": [15.0 + (index % 4) for index in range(scored)],
        "surface_after_update_ms": [14.0 + (index % 4) for index in range(scored)],
        "surface_repeat_ms": [5.0 + (index % 2) for index in range(scored)],
        "init_s": [4.0 + (index % 3) * 0.1 for index in range(scored)],
    }
    artifact = {
        "gate": "o5_group", "id": group_id, "resolution": resolution,
        "boxes": boxes, "hull": hull, "cloud": cloud, "outside": outside,
        "cross": cross, "warmup": 3, "scored": scored,
        "warm_init_s": 6.5, "rss_kib": 700 * 1024,
        "resources": {"attribution_complete": True,
                      "unmarked_allocations": False,
                      "read_view_bytes": 32 * 1024 * 1024,
                      "cache_bytes": 16 * 1024 * 1024,
                      "read_view_cache_bytes_peak": 48 * 1024 * 1024},
    }
    artifact.update(columns)
    write_json(directory / ("%s.json" % group_id), artifact)
    # Raw cumulative: 3 warmup + scored + 1 successor, values aligned with
    # the assembled columns (the cross-check re-derives them).
    documents = []
    for index in range(3 + scored + 1):
        row = {
            "status": "OK",
            "integrate_ms": 100.0 + index,
            "surface_first_call": {"external_ms": 20.0 + index},
            "surface_second_call": {"external_ms": 6.0 + index % 3},
            "process_peak_rss_kib": 700 * 1024,
            "receipt": {"status": "INTEGRATED", "raw_points": cloud,
                        "accepted_points": cloud},
        }
        documents.append(row)
    for offset in range(scored):
        row = documents[3 + offset]
        row["integrate_ms"] = columns["integrate_ms"][offset] \
            if offset < len(columns["integrate_ms"]) else 999.0
        row["surface_first_call"] = {"external_ms":
                                     columns["surface_first_ms"][offset]}
        row["surface_second_call"] = {"external_ms":
                                      columns["surface_repeat_ms"][offset]}
    cumulative = directory / group_id / "cumulative" / "samples.yaml"
    cumulative.parent.mkdir(parents=True, exist_ok=True)
    cumulative.write_text("".join(
        yaml.safe_dump(row, sort_keys=False) + "\n---\n" for row in documents))
    # Cold raw samples: one per scored index.
    for offset in range(scored):
        cold = directory / group_id / ("f%03d" % (3 + offset)) \
            / ("cold-%03d" % (3 + offset)) / "samples.yaml"
        cold.parent.mkdir(parents=True, exist_ok=True)
        cold.write_text(yaml.safe_dump({
            "status": "OK", "integrate_ms": 110.0,
            "process_peak_rss_kib": 400 * 1024,
            "receipt": {"status": "INTEGRATED"},
        }, sort_keys=False))
    return artifact


def healthy_o5(root, **kwargs):
    directory = root / "o5"
    for group_id in O5_PARAMS:
        healthy_group(directory, group_id, **kwargs)
    return directory


# --- the nine review counterexamples ------------------------------------------

@pytest.mark.parametrize("stage", ["o8", "o5", "o6"])
@pytest.mark.parametrize("value", [0, True, 30])
def test_pseudo_summary_never_passes(tmp_path, stage, value):
    """The 0715 review's nine counterexamples: a summary.json with
    passed==total (0 / true / 30) and no raw data must never certify."""
    root = run_root(tmp_path, stages=started_stages())
    stage_dir = root / stage
    stage_dir.mkdir(parents=True)
    write_json(stage_dir / "summary.json",
               {"passed": value, "total": value})
    status, reasons, _measured, _sources = verdict._row_formal(root, stage)
    assert status != "pass", (
        "pseudo summary passed==total=%r certified Formal.%s" % (value, stage))
    assert status == "fail", (
        "a stage the orchestrator recorded as started produced no raw "
        "artifact; that is a fail, not a wash-out")


def test_pseudo_summary_without_run_record_is_not_a_pass(tmp_path):
    """Even with no run manifest at all, a bare summary certifies nothing:
    the legacy layout reads <root>/scoring/<stage>, and a directory with
    content but no raw artifacts is a fail, never a pass."""
    root = tmp_path / "scoring_bare"
    (root / "scoring" / "o5").mkdir(parents=True)
    write_json(root / "scoring" / "o5" / "summary.json",
               {"passed": 30, "total": 30})
    status, reasons, _m, _s = verdict._row_formal(root, "o5")
    assert status != "pass"
    assert status == "fail"


# --- healthy controls and matrix-judge fidelity ---------------------------------

def test_healthy_o5_full_control_passes(tmp_path):
    root = run_root(tmp_path, stages=started_stages())
    healthy_o5(root)
    status, reasons, measured, _sources = verdict._row_formal(root, "o5")
    assert status == "pass", reasons
    assert measured == {"groups": 4, "expected_groups": 4, "failed_groups": []}


def test_short_integrate_column_fails(tmp_path):
    root = run_root(tmp_path, stages=started_stages())
    short = [110.0 + index for index in range(29)]
    healthy_o5(root, integrate_ms=short)
    status, reasons, _m, _s = verdict._row_formal(root, "o5")
    assert status == "fail"
    assert any("group:" in reason for reason in reasons)


def test_over_gate_integrate_fails(tmp_path):
    root = run_root(tmp_path, stages=started_stages())
    hot = [600.0] * 30  # 30k gate: P95<=500ms
    healthy_o5(root, integrate_ms=hot)
    status, reasons, _m, _s = verdict._row_formal(root, "o5")
    assert status == "fail"


def test_doctored_assembled_column_vs_raw_fails(tmp_path):
    """A group json whose numbers disagree with the raw cumulative samples
    cannot certify the row."""
    root = run_root(tmp_path, stages=started_stages())
    healthy_o5(root)
    group_id = "r02_b00_chamfer_c30000_o50_x00"
    artifact = json.loads((root / "o5" / ("%s.json" % group_id)).read_text())
    artifact["integrate_ms"] = [value / 2 for value in artifact["integrate_ms"]]
    write_json(root / "o5" / ("%s.json" % group_id), artifact)
    status, reasons, _m, _s = verdict._row_formal(root, "o5")
    assert status == "fail"
    assert any("raw_mismatch" in reason for reason in reasons)


def test_missing_cold_raw_fails(tmp_path):
    root = run_root(tmp_path, stages=started_stages())
    healthy_o5(root)
    cold = root / "o5" / "r02_b00_chamfer_c30000_o50_x00" / "f017" / "cold-017"
    for path in cold.iterdir():
        path.unlink()
    status, reasons, _m, _s = verdict._row_formal(root, "o5")
    assert status == "fail"
    assert any("cold_raw_missing" in reason for reason in reasons)


def test_missing_group_is_a_fail_not_a_wash_out(tmp_path):
    root = run_root(tmp_path, stages=started_stages())
    healthy_o5(root)
    (root / "o5" / "r02_b20_chamfer_c100000_o50_x00.json").unlink()
    status, reasons, measured, _s = verdict._row_formal(root, "o5")
    assert status == "fail"
    assert measured["groups"] == 3


def test_never_started_stage_stays_not_evaluated_with_reason(tmp_path):
    root = run_root(tmp_path, stages=started_stages(
        o6={"status": "skipped",
            "not_evaluated_reason": "budget_insufficient_estimate_300min_remaining_45min"}))
    healthy_o5(root)  # o5 ran
    status, reasons, _m, _s = verdict._row_formal(root, "o6")
    assert status == "not_evaluated"
    assert "budget_insufficient" in reasons[0]


def test_o5_failure_stops_o6(tmp_path):
    root = run_root(tmp_path, stages=started_stages(
        o5={"status": "completed", "exit_code": 1},
        o6={"status": "skipped",
            "not_evaluated_reason": "o5_dependency_failed_exit_1"}))
    status, reasons, _m, _s = verdict._row_formal(root, "o6")
    assert status == "not_evaluated"
    assert "o5_dependency" in reasons[0]


# --- R pressure coverage: unreachable under the frozen load ---------------------

def _formal_r_artifact(pressure=False):
    """A formal R artifact whose every measured aspect is green; pressure
    materializes only when asked to. e2e values stay the factory's own
    (recomputed by the judge from the row timestamps)."""
    from test_freespace_g3_offline import full_r_artifact
    artifact = full_r_artifact()
    timeline = artifact.get("overload_timeline") or []
    for index, entry in enumerate(timeline):
        entry["receipt_status"] = "QUEUE_FULL" if (pressure and index == 0) \
            else "INTEGRATED"
    return artifact


def test_r_pressure_coverage_not_evaluated_when_unreachable(tmp_path):
    """The observed formal case: every overload input INTEGRATED at e2e far
    below the 10 Hz period -- the frozen load cannot materialize pressure.
    The execution contract scores the pressure COVERAGE not_evaluated (never
    auto-pressurize); the row cannot certify but nothing failed."""
    artifact = _formal_r_artifact(pressure=False)
    import freespace_judge as _judge
    result = _judge.judge_r_stage(artifact)
    assert result["reasons"] == ["overload_pressure_not_evaluated"]
    assert result["measured"]["overload_pressure_inputs"] == 0
    assert isinstance(result["measured"]["overload_e2e_ms_p95"], float)
    root = run_root(tmp_path, stages=started_stages())
    (root / "campaign").mkdir()
    write_json(root / "campaign" / "r.json", artifact)
    status, reasons, measured, _s = verdict._row_formal(root, "r")
    assert status == "not_evaluated"
    assert reasons == ["overload_pressure_not_evaluated"]
    assert measured["drain_in_flight"] == 0


def test_r_with_pressure_and_any_real_defect_still_fails(tmp_path):
    """Pressure materialized -> the row judges normally; and a real defect
    next to the missing pressure is still a FAIL, never laundered."""
    artifact = _formal_r_artifact(pressure=True)
    root = run_root(tmp_path, stages=started_stages())
    (root / "campaign").mkdir()
    write_json(root / "campaign" / "r.json", artifact)
    status, reasons, _m, _s = verdict._row_formal(root, "r")
    assert status == "pass", reasons
    # A genuine second finding keeps the row a fail even though the pressure
    # aspect alone would have been not_evaluated.
    artifact2 = _formal_r_artifact(pressure=False)
    artifact2["recovery_timeline"][0]["receipt_status"] = "INTEGRATION_FAILED"
    write_json(root / "campaign" / "r.json", artifact2)
    status, reasons, _m, _s = verdict._row_formal(root, "r")
    assert status == "fail"


# --- resumed stages: handoff wiring ----------------------------------------------

def test_schedule_accepts_verified_o5_evidence(tmp_path):
    """A resumed O6 carries the ALREADY-SCORED O5 verdict instead of
    re-running O5 (the observed wiring defect: separate invocations left
    o5_pass=None and O6 blocked with started:0)."""
    import campaign_executor as executor
    import campaign_entry
    import repair_s0_manifest  # noqa: F401  (import surface)
    class _FastClock:
        def __init__(self):
            self.elapsed = 0.0
        @property
        def elapsed_s(self):
            return self.elapsed
        def consume(self):
            self.elapsed += 0.01
        def advance(self, delta):
            self.elapsed += delta
        def remaining(self, budget_s):
            return budget_s - self.elapsed
    plan = campaign_entry.load_plan(
        "/home/adamliao/work/elfin_humble_ws/docs/status/evidence/"
        "occ_o6_o7_20261029/run01/fixtures")

    class _CountingAdapter:
        def __init__(self):
            self.clock = _FastClock()
            self.calls = []
        def profile_frame(self, group, index, warmup, dest):
            self.calls.append(group["id"])
            return {"integrate_ms": 1.0, "acquire_ms": 1.0, "roi_ms": 1.0,
                    "surface_first_ms": 1.0, "surface_repeat_ms": 1.0,
                    "surface_after_update_ms": 1.0, "init_s": 1.0,
                    "rss_kib": 1000, "warm_init_s": 1.0,
                    "resources": {"attribution_complete": True,
                                  "unmarked_allocations": False,
                                  "read_view_bytes": 1, "cache_bytes": 1,
                                  "read_view_cache_bytes_peak": 2}}
    adapter = _CountingAdapter()
    result = executor.schedule(plan, adapter, tmp_path / "o6",
                               stages=("o6",), scored=False,
                               frame_limit=1, allow_incomplete_dependencies=False,
                               o5_evidence=None)
    blocked = result["stages"][0]
    assert blocked["status"] == "blocked" and blocked["reason"] == "o5"
    # With a passing O5 evidence file the gate opens (non-scored frame_limit
    # keeps this fast; the wiring is what is under test).
    evidence = tmp_path / "o5_verdict.json"
    write_json(evidence, {"pass": True})
    adapter = _CountingAdapter()
    result = executor.schedule(plan, adapter, tmp_path / "o6b",
                               stages=("o6",), scored=False,
                               frame_limit=1, o5_evidence=str(evidence))
    assert result["stages"][0].get("status") != "blocked"
    assert adapter.calls, "o6 groups must run with verified o5 evidence"
    failing = tmp_path / "o5_verdict_fail.json"
    write_json(failing, {"pass": False})
    adapter = _CountingAdapter()
    result = executor.schedule(plan, adapter, tmp_path / "o6c",
                               stages=("o6",), scored=False,
                               frame_limit=1, o5_evidence=str(failing))
    assert result["stages"][0]["status"] == "blocked"


def test_resume_refuses_already_scored_units(tmp_path, monkeypatch):
    """The resume gates refuse BEFORE taking the lock: an already-scored L
    or O6 unit never repeats, and O6 without a passing O5 row never starts."""
    args = _Args()
    args.root = str(tmp_path / "run")
    args.freeze_json = str(EVIDENCE / "FREEZE.json")
    args.s3_freeze = None
    (tmp_path / "run").mkdir()
    write_json(tmp_path / "run" / "run_manifest.json",
               {"run_id": "run01", "formal": True, "stages": {},
                "measurement_s": 100.0})
    (tmp_path / "run" / "campaign").mkdir(parents=True)
    write_json(tmp_path / "run" / "campaign" / "l.json", {"stage": "l"})
    orch = orchestrator.Orchestrator(args)
    with pytest.raises(SystemExit, match="already scored"):
        orch.resume("l")
    (tmp_path / "run" / "o6").mkdir()
    with pytest.raises(SystemExit, match="O5 row"):
        orch.resume("o6")


def test_started_but_artifact_missing_fails(tmp_path):
    for stage in ("campaign", "o8", "o5", "o6"):
        root = run_root(tmp_path, stages=started_stages())
        row = {"campaign": "p", "o8": "o8", "o5": "o5", "o6": "o6"}[stage]
        status, reasons, _m, _s = verdict._row_formal(root, row)
        assert status == "fail", (stage, reasons)
        assert any("started_artifact_missing" in reason or
                   "runs_missing" in reason for reason in reasons), reasons


def test_r_dependency_skip_is_not_run_not_fail(tmp_path):
    """The runner's recorded dependency stop (P failed -> R skipped) leaves
    Formal.R not_evaluated with the reason -- not a wash-out of a real
    attempt."""
    root = run_root(tmp_path, stages=started_stages())
    campaign = root / "campaign"
    campaign.mkdir()
    for name in ("p_b00_w0.json", "p_b00_w1.json", "p_b00_w2.json"):
        (campaign / name).write_text("{}")
    write_json(campaign / "summary.json",
               {"skipped": {"reason": "stage_failure_stops_dependents",
                            "detail": "p_failed"}})
    status, reasons, _m, _s = verdict._row_formal(root, "r")
    assert status == "not_evaluated"
    assert "p_failed" in reasons[0]


# --- report version switch ------------------------------------------------------

def test_formal_verdict_binds_run_and_preserves_qualification(tmp_path):
    """A formal run root: qualification rows recompute against the recorded
    qualification root (preserved), formal rows read THIS run, and the
    document carries the run binding. Full chain -- the real gen4 evidence
    is the qualification root."""
    root = run_root(tmp_path, stages=started_stages(
        campaign={"status": "skipped",
                  "not_evaluated_reason": "not_synthesized_in_this_fixture"}),
        qualification=str(EVIDENCE), with_s3_freeze=True)
    healthy_o5(root)
    document = verdict.judge_evidence_root(root)
    assert document["run"]["report_version"] == "formal"
    assert document["run"]["run_id"] == "run01"
    assert Path(document["run"]["qualification_root"]).name == "implementation_g4"
    rows = {row["row"]: row for row in document["rows"]}
    # The preserved qualification verdict still holds on its own evidence.
    assert rows["F.matrix_360"]["status"] == "pass"
    assert rows["Evidence.validity"]["status"] == "pass"
    # Formal.O5 from this run's raw artifacts: a pass here.
    assert rows["Formal.O5"]["status"] == "pass"
    # The formal stages we did not synthesize stay honestly unassessed.
    assert rows["Formal.P"]["status"] == "not_evaluated"
    assert document["overall"]["formal"] == "fail"  # P/R/L/O8 not evaluated,
    # ... and at least one formal row is evaluated, so formal != not_evaluated


def _fast_formal_root(tmp_path):
    """A run root without a qualification pointer: the version-switch
    mechanics (run binding, tamper, recompute) do not need the slow
    qualification recompute."""
    root = run_root(tmp_path, stages=started_stages(), qualification="")
    healthy_o5(root)
    return root


def test_same_run_tamper_is_a_mismatch(tmp_path):
    root = _fast_formal_root(tmp_path)
    proc = subprocess.run(
        [sys.executable, str(BENCH / "freespace_verdict.py"), str(root), "write"],
        capture_output=True, text=True)
    assert proc.returncode == 1  # unassessed/failed rows present
    stored = json.loads((root / "verdict.json").read_text())
    stored["run"]["run_id"] = "run02"  # a same-run rewrite attempt
    (root / "verdict.json").write_text(json.dumps(stored))
    proc = subprocess.run(
        [sys.executable, str(BENCH / "freespace_verdict.py"), str(root), "check"],
        capture_output=True, text=True)
    assert proc.returncode != 0
    assert "verdict_run_mismatch" in proc.stdout


def test_deleted_samples_fail_the_row_after_write(tmp_path):
    """Deleting raw samples after the verdict was written cannot leave the
    stored pass standing: the recompute fails the row and the cross-check
    flags the mismatch."""
    root = _fast_formal_root(tmp_path)
    subprocess.run(
        [sys.executable, str(BENCH / "freespace_verdict.py"), str(root), "write"],
        capture_output=True, text=True)
    (root / "o5" / "r02_b00_chamfer_c30000_o50_x00.json").unlink()
    proc = subprocess.run(
        [sys.executable, str(BENCH / "freespace_verdict.py"), str(root), "check"],
        capture_output=True, text=True)
    assert proc.returncode != 0
    assert "verdict_row_mismatch_Formal.O5" in proc.stdout


def test_qualification_layout_unchanged(tmp_path):
    """The gen4 qualification root semantics survive: no run manifest ->
    Formal rows read <root>/scoring and stay not_evaluated when absent."""
    root = tmp_path / "implementation_gX"
    root.mkdir()
    status, reasons, _m, _s = verdict._row_formal(root, "o5")
    assert status == "not_evaluated"
    assert reasons == ["formal_o5_not_run"]


def test_raw_cross_check_uses_external_clock_key_and_tolerance(tmp_path):
    """The raw cumulative documents carry integrate_external_ms (the adapter
    copies it into the assembled integrate_ms column); yaml round-trips keep
    full precision, so the comparison carries a float tolerance. A doctored
    column is still caught."""
    root = run_root(tmp_path, stages=started_stages())
    healthy_o5(root)
    gid = "r02_b00_chamfer_c30000_o50_x00"
    cumulative = root / "o5" / gid / "cumulative" / "samples.yaml"
    docs = [row for row in yaml.safe_load_all(cumulative.read_text())
            if isinstance(row, dict)]
    for index, row in enumerate(docs):
        row["integrate_external_ms"] = row.pop("integrate_ms")
    cumulative.write_text("".join(
        yaml.safe_dump(row, sort_keys=False) + "\n---\n" for row in docs))
    status, reasons, _m, _s = verdict._row_formal(root, "o5")
    assert status == "pass", reasons
    # A real doctoring (halved column) still fails.
    artifact = json.loads((root / "o5" / ("%s.json" % gid)).read_text())
    artifact["integrate_ms"] = [v / 2 for v in artifact["integrate_ms"]]
    write_json(root / "o5" / ("%s.json" % gid), artifact)
    status, reasons, _m, _s = verdict._row_formal(root, "o5")
    assert status == "fail"
    assert any("raw_mismatch" in r for r in reasons)


def test_stage_attempt_prefers_latest_resume_record():
    run = {"stages": {
        "o6": {"status": "completed", "exit_code": 1},
        "o6_resume": {"status": "skipped",
                      "not_evaluated_reason": "budget_insufficient_estimate_415min"},
    }}
    attempt = verdict._stage_attempt(run, "o6")
    assert attempt["status"] == "skipped"
    assert verdict._stage_started(run, "o6") is False
    root_reason = attempt.get("not_evaluated_reason")
    assert root_reason and "budget_insufficient" in root_reason


def test_rehearsal_root_never_certifies_formal_rows(tmp_path):
    """A rehearsal run (formal: false) with a CLEAN raw judge result can
    still never certify a formal row -- the observed defect: the stability
    stage ignores input limits, so a rehearsal ran the full O8 protocol and
    the row passed. Clean + not formal = not_evaluated."""
    root = run_root(tmp_path, stages=started_stages(), with_s3_freeze=False)
    manifest = json.loads((root / "run_manifest.json").read_text())
    manifest["formal"] = False
    write_json(root / "run_manifest.json", manifest)
    healthy_o5(root)  # a fully clean raw o5
    status, reasons, _m, _s = verdict._row_formal(root, "o5")
    assert status == "not_evaluated"
    assert reasons == ["rehearsal_not_formal_run"]


# --- orchestrator ----------------------------------------------------------------

def freeze_document(tmp_path, binary, sha):
    freeze = tmp_path / "FREEZE.json"
    write_json(freeze, {
        "head": "3bb26935", "binaries": {
            "observer_node": {"path": str(binary), "sha256": sha}}})
    return freeze


def test_freeze_gate_accepts_unchanged_binary(tmp_path):
    binary = tmp_path / "observer_node"
    binary.write_bytes(b"payload")
    import hashlib
    sha = hashlib.sha256(binary.read_bytes()).hexdigest()
    report, problems = orchestrator.verify_product_shas(
        freeze_document(tmp_path, binary, sha))
    assert problems == []
    assert report["binaries_checked"] == 1


def test_freeze_gate_refuses_changed_binary(tmp_path):
    binary = tmp_path / "observer_node"
    binary.write_bytes(b"payload")
    report, problems = orchestrator.verify_product_shas(
        freeze_document(tmp_path, binary, "0" * 64))
    assert any("mismatch" in problem for problem in problems)
    assert report is not None


def test_input_manifest_binds_frames_by_sha(tmp_path):
    frames = tmp_path / "frames"
    frames.mkdir()
    (frames / "b00_a.yaml").write_text(yaml.safe_dump(
        {"points": [[0.0, 0.0, 0.0]] * 10, "observation_id": "x"}))
    (frames / "b20_a.yaml").write_text(yaml.safe_dump(
        {"points": [[0.1, 0.0, 0.0]] * 10, "observation_id": "y"}))
    manifest = orchestrator.build_input_manifest(frames, formal=True)
    by_tag = {entry["box_tag"]: entry for entry in manifest["frames"]}
    assert set(by_tag) == {0, 20}
    import hashlib
    assert by_tag[0]["sha256"] == hashlib.sha256(
        (frames / "b00_a.yaml").read_bytes()).hexdigest()
    assert manifest["stamp_rules"]["p_window"]["inputs"] == 300
    assert manifest["formal"] is True


class _Args:
    root = "unused"
    worktree = "/tmp/elfin-occ-freespace-impl-g4"
    qualification_root = "../implementation_g4"
    budget_minutes = 480.0
    run_id = "unit"
    rehearsal = False


def test_budget_gate_skips_unreachable_stage(tmp_path):
    """A stage whose conservative estimate does not fit the remaining budget
    is never started: recorded skipped with the reason."""
    args = _Args()
    args.root = str(tmp_path / "run")
    (tmp_path / "run").mkdir()
    orch = orchestrator.Orchestrator(args)
    orch.manifest["stages"] = {}
    # Simulate 470 of 480 minutes spent: a 25-minute estimate cannot fit.
    orch.started_mono = __import__("time").monotonic() - 470 * 60
    exit_code = orch.run_stage("o8", ["true"],
                               estimate_min=orchestrator.FORMAL_ESTIMATE_MIN["o8"])
    assert exit_code is None
    state = orch.manifest["stages"]["o8"]
    assert state["status"] == "skipped"
    assert "budget_insufficient" in state["not_evaluated_reason"]
    assert not (tmp_path / "run" / "logs" / "o8.log").exists()


def test_budget_gate_allows_fitting_stage(tmp_path):
    args = _Args()
    args.root = str(tmp_path / "run")
    (tmp_path / "run").mkdir()
    orch = orchestrator.Orchestrator(args)
    orch.started_mono = __import__("time").monotonic() - 60  # 1 minute spent
    exit_code = orch.run_stage("probe", ["true"], estimate_min=None)
    assert exit_code == 0
    assert orch.manifest["stages"]["probe"]["status"] == "completed"


def test_o6_decision_follows_o5():
    args = _Args()
    orch = orchestrator.Orchestrator(args)
    orch.manifest["stages"] = {"o5": {"exit_code": 1, "duration_s": 600.0}}
    estimate, reason = orch.o6_decision()
    assert estimate is None
    assert reason == "o5_dependency_failed_exit_1"
    orch.manifest["stages"] = {"o5": {"exit_code": 0, "duration_s": 600.0}}
    estimate, reason = orch.o6_decision()
    assert reason is None
    # 10 minutes for 4 groups -> 2.5 min/group -> 96 groups x1.3 margin.
    assert estimate == pytest.approx(2.5 * 96 * 1.3)
    args.rehearsal = True
    orch = orchestrator.Orchestrator(args)
    estimate, reason = orch.o6_decision()
    assert reason == "rehearsal_wiring_o5_only"


def test_single_lock_holder_layer():
    """The formal path holds the perf lock exactly once: the orchestrator.
    No executor the orchestrator invokes may flock it (no nested flock)."""
    for executor_name in ("freespace_campaign.py", "campaign_executor.py",
                          "live_schedule.py"):
        text = (BENCH / executor_name).read_text()
        assert "fcntl" not in text, "%s must not acquire locks" % executor_name
        assert "elfin-occ-backend-perf.lock" not in text, executor_name
    # The orchestrator is the only lock acquirer among the files the score
    # entry invokes.
    orchestrator_text = (BENCH / "score_orchestrator.py").read_text()
    assert "fcntl.flock" in orchestrator_text
    commands = EVIDENCE.parent / "scoring_s3" / "COMMANDS.sh"
    assert commands.is_file(), "the S3 scoring entry must exist"
    text = commands.read_text()
    score_section = text.split("  score)")[1].split(";;")[0]
    assert "o6_matrix" not in score_section
    assert '"$ORCHESTRATOR"' in score_section
    # The formal entry keeps the explicit gates the contract requires.
    assert "OCC_FS_SCORE_OPEN" in score_section
    assert "OCC_FS_SCORE_REHEARSAL" in score_section
    assert "--budget-minutes 480" in score_section
