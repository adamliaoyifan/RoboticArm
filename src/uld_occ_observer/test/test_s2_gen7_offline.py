"""S2 gen7 regressions: the three audit counterexamples and the acceptance
supplement's failure matrix.

Every test here reproduces a defect the gen6 review demonstrated (missing
receipts passing O8, warmup rows scored as formal samples, forged bundles
marked replayable) or pins a new judge/lifecycle rule (recovery continuity,
drain deadline honesty, timestamped RSS windows, occlusion vs duplicate-id).
"""
import json
import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent / "benchmark"))

import campaign_executor as executor
import live_schedule
import repair_s0_contract as contract
import repair_s0_judge as judge


# --------------------------------------------------------------- fixtures
def _adapter_with_groups(groups, frame_limit=None):
    adapter = object.__new__(live_schedule.LiveScheduleAdapter)
    adapter.frame_limit = frame_limit
    adapter._groups = groups
    return adapter


def _healthy_run():
    return judge.healthy_o8()["runs"][0]


# ---------------------------------------------------- audit counterexample 1
def test_o8_all_receipts_missing_fails_the_judge():
    run = _healthy_run()
    for entry in run["saturation_receipts"]:
        entry["events"] = 0
    case = judge.healthy_o8()
    case["runs"] = [dict(run, run_id=str(index)) for index in range(contract.O8_RUNS)]
    verdict = judge.judge_o8(case)
    assert not verdict["pass"]
    assert "saturation_receipt_missing" in verdict["reasons"]


def test_o8_single_missing_receipt_fails():
    run = _healthy_run()
    run["saturation_receipts"][7]["events"] = 0
    case = judge.healthy_o8()
    case["runs"] = [dict(run, run_id=str(index)) for index in range(contract.O8_RUNS)]
    verdict = judge.judge_o8(case)
    assert "saturation_receipt_missing" in verdict["reasons"]


def test_o8_rejected_saturation_receipt_fails():
    run = _healthy_run()
    run["saturation_receipts"][3]["status"] = "REJECTED"
    case = judge.healthy_o8()
    case["runs"] = [dict(run, run_id=str(index)) for index in range(contract.O8_RUNS)]
    verdict = judge.judge_o8(case)
    assert "saturation_receipt_rejected" in verdict["reasons"]


def test_o8_missing_counts_and_nonfinite_values_fail():
    run = _healthy_run()
    run["saturation_receipts"][0]["changed_keys"] = None
    run["saturation_receipts"][1]["rebuilt_blocks"] = float("nan")
    case = judge.healthy_o8()
    case["runs"] = [dict(run, run_id=str(index)) for index in range(contract.O8_RUNS)]
    verdict = judge.judge_o8(case)
    assert "saturation_count_missing" in verdict["reasons"]
    assert "nonfinite_value" in verdict["reasons"]


def test_o8_single_health_sample_does_not_certify_stability():
    run = _healthy_run()
    run["node_health_times_s"] = [70.0]
    run["node_health_count"] = 1
    case = judge.healthy_o8()
    case["runs"] = [dict(run, run_id=str(index)) for index in range(contract.O8_RUNS)]
    verdict = judge.judge_o8(case)
    assert "node_health_span" in verdict["reasons"]


def test_o8_rss_windows_need_real_samples():
    run = _healthy_run()
    run["rss_window_counts"] = [30, 30, 0]
    case = judge.healthy_o8()
    case["runs"] = [dict(run, run_id=str(index)) for index in range(contract.O8_RUNS)]
    verdict = judge.judge_o8(case)
    assert "rss_span" in verdict["reasons"]
    run = _healthy_run()
    run["rss_window_counts"] = None
    case = judge.healthy_o8()
    case["runs"] = [dict(run, run_id=str(index)) for index in range(contract.O8_RUNS)]
    assert "missing_observation" in judge.judge_o8(case)["reasons"]


def test_o8_healthy_fixture_passes():
    verdict = judge.judge_o8(judge.healthy_o8())
    assert verdict["pass"], verdict["reasons"]


def test_o8_occlusion_requires_blocking_input_and_region_oracle():
    case = judge.healthy_o8()
    case["holds"][0]["kind"] = "no_input"
    occlusion = next(hold for hold in case["holds"] if hold["kind"] == "occlusion")
    del occlusion["occlusion_input"]
    verdict = judge.judge_o8(case)
    assert "occlusion_input_missing" in verdict["reasons"]
    occlusion["occlusion_input"] = {"points_sha256": "x", "point_count": 5,
                                    "oracle_min_cell_x": 5, "plane_max_cell_x": 7}
    verdict = judge.judge_o8(case)
    assert "occlusion_geometry" in verdict["reasons"]
    occlusion["occlusion_input"]["oracle_min_cell_x"] = 9
    occlusion["input_accepted"] = False
    verdict = judge.judge_o8(case)
    assert "occlusion_not_accepted" in verdict["reasons"]
    occlusion["input_accepted"] = True
    occlusion["occluder_receipt"] = {"status": "COMPLETED", "changed_keys": 0}
    verdict = judge.judge_o8(case)
    assert "occlusion_no_effect" in verdict["reasons"]
    occlusion["occluder_receipt"]["changed_keys"] = 700
    occlusion["region_probability_unchanged"] = False
    verdict = judge.judge_o8(case)
    assert "probability" in verdict["reasons"]


def test_o8_duplicate_id_must_be_rejected_and_separate():
    case = judge.healthy_o8()
    duplicate = next(hold for hold in case["holds"] if hold["kind"] == "duplicate_id")
    duplicate["duplicate_rejected"] = False
    verdict = judge.judge_o8(case)
    assert "duplicate_not_rejected" in verdict["reasons"]
    duplicate["duplicate_rejected"] = True
    duplicate["freshness_advanced"] = True
    verdict = judge.judge_o8(case)
    assert "freshness" in verdict["reasons"]
    assert judge.judge_o8(judge.healthy_o8())["pass"]


def test_o8_hold_counts_cover_all_three_kinds():
    case = judge.healthy_o8()
    case["holds"] = [hold for hold in case["holds"] if hold["kind"] != "duplicate_id"]
    verdict = judge.judge_o8(case)
    assert "hold_count" in verdict["reasons"]


# ---------------------------------------------------- audit counterexample 2
def test_formal_scored_indices_select_exactly_rows_three_to_thirty_two():
    """The audit proved index-3 scored rows 0..29. The fix scores 3..32."""
    groups = {"g": {"samples": [{"marker": index} for index in range(34)],
                    "warm_init_s": 1.0, "resources": None, "rss_kib": 100}}
    adapter = _adapter_with_groups(groups)
    adapter._cold_start = lambda group, offset, dest: {"init_s": 0.5, "rss_kib": 1000}
    scored = [adapter.profile_frame({"id": "g"}, index, False, "/tmp/gen7-probe")["marker"]
              for index in range(3, 33)]
    assert scored == list(range(3, 33))
    warmup = [adapter.profile_frame({"id": "g"}, index, True, "/tmp/gen7-probe")["marker"]
              for index in range(3)]
    assert warmup == [0, 1, 2]


def test_scheduler_indices_flow_to_the_artifact_and_the_judge():
    """Marker counterexample through the scheduler->adapter->artifact path."""
    plan = {"o5_cases": [], "o6_groups": [], "o7_windows": [], "o8": {}}
    seen = []

    class Tracing(executor.OfflineAdapter):
        def profile_frame(self, group, index, warmup, dest):
            seen.append((group["id"], index, warmup))
            row = super().profile_frame(group, index, warmup, dest)
            row["marker"] = index
            return row

    # One group, formal shape: 3 warmup rows 0..2 then scored rows 3..32.
    group = {"id": "probe_g", "resolution": 0.02, "hull": "chamfer", "boxes": 0,
             "cloud": 30000, "outside": 0.5, "cross": 0.0, "warmup": 3, "scored": 30}
    plan["o5_cases"] = [{"group": group, "warmup": 3, "scored": 30}]
    out = Path("/tmp/gen7-index-flow")
    captured = []
    original_assemble = executor._assemble_group

    def capturing_assemble(group, samples, stage):
        captured.extend(samples)
        return original_assemble(group, samples, stage)

    executor._assemble_group = capturing_assemble
    try:
        executor.schedule(plan, Tracing(), out, stages=("o5",),
                          allow_incomplete_dependencies=True)
    finally:
        executor._assemble_group = original_assemble
    artifact = json.loads((out / "o5" / "probe_g.json").read_text())
    assert [seen[index][1] for index in range(3)] == [0, 1, 2]
    assert all(seen[index][2] for index in range(3))
    scored_indices = [index for _g, index, warm in seen if not warm]
    assert scored_indices == list(range(3, 33))
    # The assembled group carries exactly the scored rows 3..32.
    assert [sample["marker"] for sample in captured] == list(range(3, 33))
    assert len(artifact["integrate_ms"]) == contract.SCORED_SAMPLES


def test_profile_frame_keeps_cumulative_peak_rss_separate_from_cold_start():
    groups = {"g": {"samples": [{"marker": index, "rss_kib": 500000} for index in range(34)],
                    "warm_init_s": 1.0, "resources": None, "rss_kib": 500000}}
    adapter = _adapter_with_groups(groups)

    def fake_cold(group, offset, dest):
        return {"init_s": 0.5, "rss_kib": 60000}

    adapter._cold_start = fake_cold
    sample = adapter.profile_frame({"id": "g"}, 10, False, "/tmp/gen7-rss")
    assert sample["rss_kib"] == 500000
    assert sample["cold_start_rss_kib"] == 60000
    assert sample["init_s"] == 0.5


def test_peak_resources_scan_whole_run_not_last_row():
    documents = [
        {"resource_usage": {"read_view_bytes": 100, "cache_bytes": 5,
                            "certification": "READ_VIEW_CACHE_ATTRIBUTED",
                            "attribution_complete": True,
                            "unmarked_allocations": False}},
        {"resource_usage": {"read_view_bytes": 900, "cache_bytes": 3,
                            "certification": "READ_VIEW_CACHE_ATTRIBUTED",
                            "attribution_complete": True,
                            "unmarked_allocations": False}},
        {"resource_usage": {"read_view_bytes": 200, "cache_bytes": 2,
                            "certification": "READ_VIEW_CACHE_ATTRIBUTED",
                            "attribution_complete": True,
                            "unmarked_allocations": False}},
    ]
    resources = live_schedule._peak_resources_from_documents(documents)
    assert resources["read_view_bytes"] == 900
    assert resources["cache_bytes"] == 5
    assert resources["rows_scanned"] == 3
    assert resources["attribution_complete"] is True
    # A class missing everywhere stays missing; the judge rejects it.
    assert resources["store_tree_bytes"] is None
    documents[1]["resource_usage"]["attribution_complete"] = False
    assert live_schedule._peak_resources_from_documents(documents)["attribution_complete"] is False


# --------------------------------------------------------- O7 lifecycle
def test_recovery_judge_requires_continuity_record():
    case = judge.healthy_o7_recovery()
    del case["continuity"]
    verdict = judge.judge_o7_recovery(case)
    assert "continuity_missing" in verdict["reasons"]
    case["continuity"] = {"reused_observer": False, "observer_pid": 1}
    assert "continuity_missing" in judge.judge_o7_recovery(case)["reasons"]
    healthy = judge.healthy_o7_recovery()
    assert judge.judge_o7_recovery(healthy)["pass"], judge.judge_o7_recovery(healthy)
    restarted = judge.healthy_o7_recovery()
    restarted["continuity"]["reused_observer"] = False
    assert "continuity_missing" in judge.judge_o7_recovery(restarted)["reasons"]
    inverted = judge.healthy_o7_recovery()
    inverted["continuity"]["recovery_origin_mono_ns"] = 9_000_000_000
    inverted["continuity"]["drain_boundary_mono_ns"] = 10_000_000_000
    assert "continuity_boundary" in judge.judge_o7_recovery(inverted)["reasons"]


def test_ros_send_continues_the_overload_observer_into_recovery():
    """Offline lifecycle probe: the recovery window reuses the run object."""
    adapter = object.__new__(live_schedule.LiveScheduleAdapter)
    adapter.input_limit = 2
    adapter.out = Path("/tmp/gen7-continuity")
    adapter.overruns = []
    adapter._continuity = None
    adapter.plan_groups = [{"id": "g20"}]
    continued = []
    finished = []

    class FakeRun:
        def __init__(self, kind):
            self.window = {"id": "%s-w" % kind, "kind": kind}

        def continue_as(self, window, count):
            continued.append((self.window["kind"], window["kind"], count))
            return {"reused_observer": True, "observer_pid": 99,
                    "drain_boundary_mono_ns": 1, "recovery_origin_mono_ns": 2,
                    "epoch": 1, "observed_revision_at_recovery_open": 4}

    adapter._window = FakeRun("overload")
    adapter._finish_window = lambda: finished.append(True)
    recovery = {"id": "recovery", "kind": "recovery", "group_id": "g20", "inputs": 60}
    adapter.ros_send(recovery, 0, executor._input_id(recovery, 0), 0.0, 0.0)
    assert continued == [("overload", "recovery", 2)]
    assert not finished, "the overload run must not be torn down before recovery"
    assert adapter.ros_continuity(recovery)["reused_observer"] is True
    # A normal window after a normal window still starts fresh.
    adapter._window = FakeRun("normal")
    normal2 = {"id": "normal_b00_r1", "kind": "normal", "group_id": "g20", "inputs": 240}
    adapter._WindowRun = lambda *args, **kwargs: pytest.fail("fresh window expected")
    started = []
    adapter._window_group = lambda window: {"id": window["group_id"]}
    original_start = live_schedule._WindowRun.start

    def record_start(self, count):
        started.append(count)
    live_schedule._WindowRun.start = record_start
    try:
        adapter.ros_send(normal2, 0, executor._input_id(normal2, 0), 0.0, 0.0)
        assert started == [2]
        assert finished == [True], "leaving a normal window tears the run down"
    finally:
        live_schedule._WindowRun.start = original_start


def test_stability_stage_tears_down_the_lingering_o7_window(monkeypatch):
    """Two live observers on one domain split every service call: the O8 run
    must finish the O7 stage's still-open window before opening its own."""
    adapter = object.__new__(live_schedule.LiveScheduleAdapter)
    adapter.overruns = []
    adapter._stability = None
    adapter.eval_native = "stub-eval-native"
    adapter._window = NS(window={"id": "recovery", "kind": "recovery"})
    adapter._continuity = None
    adapter.out = Path("/tmp/gen7-o8-teardown")
    finished = []
    order = []
    adapter._finish_window = lambda: order.append("finish_o7")
    adapter.plan_o8 = {"ray_group": "o8_fixed_g", "duration_s": 180,
                       "saturation_frames": 30}
    adapter.plan_groups = [{"id": "g", "geometry_file": "g.yaml", "config_file": "c.yaml"}]

    class FakeStabilityRun:
        def __init__(self, adapter_, window, group, out):
            order.append("open_o8")
            self.window = window
            self.out = Path(out)
        def start_for_o8(self, count):
            self.origin_ns = time.monotonic_ns()
            self.clock_origin = 0.0
            self.session = NS(next_stamp=lambda: 1_000_000_000)
            self.publisher = NS(publish=lambda message: None)
            self.sleep_to = lambda wall_ns, label: None
    monkeypatch.setattr(live_schedule, "_WindowRun", FakeStabilityRun)
    monkeypatch.setattr(live_schedule, "_emit_frames",
                        lambda *args, **kwargs: [{"points_m": []}])
    monkeypatch.setattr(live_schedule, "cloud_from_frame",
                        lambda frame: (NS(header=NS(stamp=NS(sec=0, nanosec=0)),
                                          publish=lambda message: None), 0))
    adapter.stability_frame("o8_r0", 0)
    assert order == ["finish_o7", "open_o8"], order


def test_ros_drain_takes_one_pass_at_the_deadline():
    adapter = object.__new__(live_schedule.LiveScheduleAdapter)
    queries = []
    session = NS(query=lambda epoch, observation: queries.append(observation) or NS(status="INTEGRATING"))
    run = NS(real_ids={"i0": "a", "i1": "b"}, window={"id": "overload", "kind": "overload"},
             origin_ns=time.monotonic_ns() - 5_000_000_000, clock_origin=0.0,
             wall_deadline=lambda clock_deadline: time.monotonic_ns(),
             sleep_to=lambda wall_ns, label: None, session=session)
    adapter._window = run
    adapter.overruns = []
    started = time.monotonic()
    remaining = adapter.ros_drain({"id": "overload", "kind": "overload"}, deadline=5.0)
    assert remaining == 2
    assert len(queries) == 2, "one status pass over the ids, no retry loop"
    assert time.monotonic() - started < 5.0, "no post-deadline top-up waiting"
    assert run.drain_record["in_flight"] == 2


def test_normal_windows_query_the_immediate_interface():
    adapter = object.__new__(live_schedule.LiveScheduleAdapter)
    run = NS(window={"id": "n", "kind": "normal"}, origin_ns=time.monotonic_ns(),
             clock_origin=0.0, wall_deadline=lambda deadline: time.monotonic_ns(),
             sleep_to=lambda wall_ns, label: None, real_ids={"n-0": "x"}, session=NS())
    adapter._window = run
    adapter.overruns = []
    immediate = []
    waits = []
    adapter._immediate_query = lambda run_, started_ns: immediate.append(started_ns) or {"op": "immediate"}
    adapter._wait_chain = lambda run_, target, started_ns: waits.append(target) or {"op": "wait_snapshot"}
    normal = {"id": "n", "kind": "normal", "query_hz": 1}
    result = adapter.ros_query(normal, 0)
    assert result["op"] == "immediate" and immediate and not waits
    recovery = {"id": "recovery", "kind": "recovery", "query_hz": 1}
    run.real_ids = {"recovery-0": "x"}
    first = adapter.ros_query(recovery, 0)
    assert first["op"] == "wait_snapshot" and first["expected_observation_id"] == "recovery-0"
    later = adapter.ros_query(recovery, 1)
    assert later["op"] == "immediate"


# ---------------------------------------------------- audit counterexample 3
def _write_frame(path, observation_id="case-f0", stamp=1000000000):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "sensor_id: cam0\nobservation_id: %s\noptical_frame: d555_color_optical_frame\n"
        "clock_domain: ros\ngeometry_hash: h\nepoch: 1\nstamp_ns: %d\npoints_m:\n"
        "- [0.1, 0.1, 1.0]\ntransform:\n  stamp_ns: %d\n  translation: [0, 0, 0]\n"
        "  rotation: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n" % (observation_id, stamp, stamp))
    import hashlib
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_forged_bundle_is_neither_complete_nor_replayable(tmp_path):
    base = tmp_path / "forged"
    folder = base / "o5" / "cumulative"
    folder.mkdir(parents=True)
    (folder / "samples.yaml").write_text("{}\n")
    (folder / "cli.log").write_text("fixture\n")
    (folder / "manifest.yaml").write_text(
        "frames:\n- path: missing.yaml\n  sha256: fake\n")
    bundle = executor._save_failure(base, "o5", "injected_missing_input", "audit")
    manifest = json.loads((bundle / "manifest.json").read_text())
    assert manifest["capture_complete"] is False
    assert manifest["replay_possible"] is False
    assert any("missing.yaml" in item for item in manifest["missing"])
    assert manifest["input_problems"]


def test_failure_manifest_accepts_fully_verified_inputs(tmp_path):
    base = tmp_path / "real"
    folder = base / "o5" / "cumulative"
    folder.mkdir(parents=True)
    (folder / "samples.yaml").write_text("row\n")
    (folder / "cli.log").write_text("log\n")
    frame0 = _write_frame(folder / "f000.yaml", "case-f0", 1000000000)
    frame1 = _write_frame(folder / "f001.yaml", "case-f1", 1500000000)
    (folder / "manifest.yaml").write_text(
        "frames:\n- path: f000.yaml\n  sha256: %s\n- path: f001.yaml\n  sha256: %s\n"
        % (frame0, frame1))
    bundle = executor._save_failure(base, "o5", "two_verified_frames", "audit")
    manifest = json.loads((bundle / "manifest.json").read_text())
    assert manifest["capture_complete"] is True
    assert manifest["replay_possible"] is True
    assert not manifest["input_problems"]
    # A wrong hash on one frame drops both flags again.
    (folder / "manifest.yaml").write_text(
        "frames:\n- path: f000.yaml\n  sha256: %s\n- path: f001.yaml\n  sha256: deadbeef\n"
        % frame0)
    bundle = executor._save_failure(base, "o5", "hash_mismatch", "audit")
    manifest = json.loads((bundle / "manifest.json").read_text())
    assert manifest["capture_complete"] is False
    assert manifest["replay_possible"] is False


def test_stage_exception_saves_failure_before_teardown(tmp_path):
    """Missing binary: the formal stage entry must persist a bundle, exit != 0."""
    if not Path(contract.__file__).is_file():
        pytest.fail("benchmark modules missing")
    import campaign_entry
    import repair_s0_manifest
    fixtures = str(repair_s0_manifest.DEFAULT_FIXTURES)
    out = tmp_path / "stage-failure"
    argv = ["--mode", "stage", "--stage", "o5", "--out", str(out),
            "--cli", str(tmp_path / "missing-cli"), "--eval-native", str(tmp_path / "missing-eval"),
            "--node", str(tmp_path / "missing-node"), "--geometry", fixtures, "--config", fixtures,
            "--fixtures", fixtures, "--frame-limit", "1"]
    code = executor.main(argv)
    assert code == 3
    bundle = out / "failure" / "o5" / "manifest.json"
    assert bundle.is_file(), "failure bundle must exist before teardown"
    manifest = json.loads(bundle.read_text())
    assert manifest["reason"].startswith("FileNotFoundError")
    assert manifest["capture_complete"] is False
    assert manifest["replay_possible"] is False


def test_occluder_geometry_stays_clear_of_the_oracle_region():
    points = live_schedule.occluder_points()
    assert points, "plane must be populated"
    for x, y, z in points:
        # ULD frame: plane at -0.62, depth inside the valid band.
        assert z == pytest.approx(0.53, abs=1e-9)
        assert 0.52 <= z <= 6.0
        assert -1.5 <= x <= 1.5
    plane_cell = int((-0.62 + 0.745) / 0.02)
    assert plane_cell <= live_schedule._OCCLUDER_MAX_CELL_X
    assert live_schedule._ORACLE_MIN_CELL_X > live_schedule._OCCLUDER_MAX_CELL_X
    # Dense enough to occupy contiguous plane voxels at 2 cm.
    assert len(points) >= 1000


# ---------------------------------------------------------------- live
def _gen7_binaries():
    cli = os.environ.get("OCC_S2_PROFILE_CLI")
    native = os.environ.get("OCC_S2_EVAL_NATIVE")
    node = os.environ.get("OCC_S2_NODE")
    for path in (cli, native, node):
        if not path or not Path(path).is_file():
            pytest.fail("pin OCC_S2_PROFILE_CLI/EVAL_NATIVE/NODE to the gen7 install")
    return cli, native, node


def test_live_occluder_blocks_without_touching_the_old_region(tmp_path):
    """Live occlusion semantics: an accepted plane input whose rays stop at
    the plane. The old region beyond the plane must hash identically; the
    duplicate-id replay must be rejected and move nothing."""
    import yaml
    cli, native, node = _gen7_binaries()
    fixtures = Path("/home/adamliao/work/elfin_humble_ws/docs/status/evidence/"
                    "occ_o6_o7_20261029/run01/fixtures")
    plan = executor.campaign_entry.load_plan(str(fixtures))
    adapter = live_schedule.LiveScheduleAdapter(
        cli, native, node, str(fixtures / "geometry_chamfer.yaml"),
        str(fixtures / "config_chamfer_r0p02.yaml"), tmp_path)
    adapter.bind_plan(plan)
    try:
        adapter.clock.advance(1.0)
        # Build the fixed-ray map: three paced frames on the o8 ray group.
        for index in range(3):
            adapter.stability_frame("occl-check", index)
        run = adapter._stability["run"]
        adapter._await_terminal(run, adapter._stability["stamps"], timeout_s=30.0)
        before = adapter._hold_probe(run)
        assert "error" not in before, before
        assert before["columns"] > 0
        # The occluder: valid input, accepted, integrates new cells.
        receipt = live_schedule.publish_occluder(run.session, run.publisher,
                                                 live_schedule.occluder_points())
        assert receipt["status"] == "COMPLETED", receipt
        assert receipt["disposition"] == "integrated", receipt
        assert isinstance(receipt["changed_keys"], int) and receipt["changed_keys"] > 0, receipt
        after = adapter._hold_probe(run)
        assert "error" not in after, after
        assert after["content_hash"] == before["content_hash"], \
            "the old region beyond the plane must be untouched"
        assert after["last_integrated_stamp_ns"] > before["last_integrated_stamp_ns"]
        # Duplicate identity: unstored DUPLICATE receipt, map and freshness
        # unmoved (the replayed stamp is identical, so freshness cannot move).
        duplicate_before = adapter._hold_probe(run)
        stamp = int(adapter._stability["stamps"][-1])
        events_before = len(executor._receipt_events(run.session, str(stamp)))
        frame = run.o8_frames[0]
        message, _points = live_schedule.cloud_from_frame(frame)
        message.header.stamp.sec = stamp // 1_000_000_000
        message.header.stamp.nanosec = stamp % 1_000_000_000
        run.publisher.publish(message)
        time.sleep(1.0)
        events = executor._receipt_events(run.session, str(stamp))
        assert any(event.get("status") == "DUPLICATE" for event in events), events
        assert len(events) == events_before + 1
        duplicate_after = adapter._hold_probe(run)
        assert duplicate_after["content_hash"] == duplicate_before["content_hash"]
        assert duplicate_after["last_integrated_stamp_ns"] == \
            duplicate_before["last_integrated_stamp_ns"]
    finally:
        adapter.close()
