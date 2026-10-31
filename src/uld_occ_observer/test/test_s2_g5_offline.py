"""S2 gen5: observations pass through the scheduler, and bad windows stay bad."""
import inspect
import json
import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent / "benchmark"))
import campaign_entry  # noqa: E402
import campaign_executor  # noqa: E402

FIXTURES = Path("/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures")


def _plan():
    return campaign_entry.load_plan(FIXTURES)


def _window(plan, name):
    return next(item for item in plan["o7_windows"] if item["id"] == name)


def test_assembler_does_not_manufacture_healthy_observations():
    for name in ("_assemble_window", "_assemble_group", "_run_stability"):
        source = inspect.getsource(getattr(campaign_executor, name))
        assert "healthy_" not in source
        assert "_resources_ok" not in source
    plan = _plan()
    window = _window(plan, "normal_b00_r0")
    rows = [{
        "input_id": "score-%d" % index,
        "phase": "score",
        "disposition": "rejected",
        "reject_reason": "QUEUE_FULL",
        "terminal_count": 1,
        "receipt_received": False,
        "callback_enter_ns": 1100 + index,
        "publish_end_ns": 1000 + index,
    } for index in range(240)]
    artifact = campaign_executor._assemble_window(window, rows, [], None, None)
    assert artifact["queries"] == []
    assert artifact["inputs"][0]["disposition"] == "rejected"
    assert "integrate_ms" not in artifact["inputs"][0]
    assert "health_times_s" not in artifact
    verdict = campaign_executor.repair_s0_judge.judge_o7_normal(artifact)
    assert verdict["pass"] is False
    assert "all_rejected" in verdict["reasons"]


def test_rejected_and_unknown_windows_fail_through_schedule(tmp_path):
    plan = _plan()
    rejected = campaign_executor.schedule(
        plan, campaign_executor.OfflineAdapter(input_disposition="rejected", emit_queries=False),
        tmp_path / "rejected", stages=("o7",), allow_incomplete_dependencies=True)
    artifact = json.loads((tmp_path / "rejected" / "o7" / "normal_b00_r0.json").read_text())
    assert artifact["queries"] == []
    assert artifact["inputs"][0]["disposition"] == "rejected"
    assert artifact["inputs"][0]["reject_reason"] == "QUEUE_FULL"
    assert rejected["stages"][0]["judge"]["pass"] is False

    unknown = campaign_executor.schedule(
        plan, campaign_executor.OfflineAdapter(input_disposition="unknown", emit_queries=False),
        tmp_path / "unknown", stages=("o7",), allow_incomplete_dependencies=True)
    saved = json.loads((tmp_path / "unknown" / "o7" / "normal_b00_r0.json").read_text())
    assert saved["inputs"][0]["disposition"] == "unknown"
    assert saved["inputs"][0]["receipt_received"] is False
    assert saved["inputs"][0]["integrate_ms"] == 100000.0
    assert saved["queries"] == []
    assert unknown["stages"][0]["judge"]["pass"] is False


def test_faults_keep_their_fields_on_the_schedule_path(tmp_path):
    plan = _plan()

    def run(name, adapter, stage):
        return campaign_executor.schedule(
            plan, adapter, tmp_path / name, stages=(stage,), allow_incomplete_dependencies=True)

    stale = run("stale", campaign_executor.OfflineAdapter(recovery_fault="stale"), "o7")
    assert stale["stages"][0]["recovery"]["pass"] is False
    missing_health = run("health", campaign_executor.OfflineAdapter(drop_health=True), "o7")
    assert missing_health["stages"][0]["judge"]["pass"] is False
    duplicate = run("dup", campaign_executor.OfflineAdapter(duplicate_input_id=True), "o7")
    assert duplicate["stages"][0]["judge"]["pass"] is False
    resources = run("resources", campaign_executor.OfflineAdapter(omit_resources=True), "o5")
    assert resources["stages"][0]["judge"]["pass"] is False
    growth = run("growth", campaign_executor.OfflineAdapter(rss_growth=True), "o8")
    assert "rss_growth" in growth["stages"][0]["judge"]["reasons"]
    nodes = run("nodes", campaign_executor.OfflineAdapter(node_stable=False), "o8")
    assert "node_count" in nodes["stages"][0]["judge"]["reasons"]


def test_timeline_sends_before_collect_and_uses_the_window_clock(tmp_path):
    plan = _plan()
    adapter = campaign_executor.OfflineAdapter()
    campaign_executor.schedule(
        plan, adapter, tmp_path, stages=("o7",), allow_incomplete_dependencies=True)
    sends = [item for item in adapter.order if item[0] == "send" and item[1] == "normal_b00_r0"]
    collects = [item for item in adapter.order if item[0] == "collect" and item[1] == "normal_b00_r0"]
    assert len(sends) == 240
    assert collects[0][3] >= sends[-1][3]
    assert sends[1][3] - sends[0][3] == pytest.approx(0.5)
    queries = [item for item in adapter.query_calls if item[0] == "normal_b00_r0"]
    assert len(queries) == 120
    assert queries[1][2] - queries[0][2] == pytest.approx(1.0)
    overload = [item for item in adapter.order if item[0] == "send" and item[1] == "overload"]
    assert len(overload) == 600
    assert overload[1][3] - overload[0][3] == pytest.approx(0.1)
    assert adapter.drain_time - overload[-1][3] == pytest.approx(30.0)
    recovery = json.loads((tmp_path / "o7" / "recovery.json").read_text())
    assert recovery["queries"][0]["op"] == "wait_snapshot"
    assert [item["op"] for item in recovery["queries"][1:]] == ["immediate"] * 29


def test_profile_frames_keep_distinct_surface_times_and_directories(tmp_path):
    plan = _plan()
    adapter = campaign_executor.OfflineAdapter()
    campaign_executor.schedule(plan, adapter, tmp_path, stages=("o5",))
    saved = json.loads(next(path for path in (tmp_path / "o5").glob("*.json") if path.name != "verdict.json").read_text())
    assert saved["surface_first_ms"][0] == 10.0
    assert saved["surface_after_update_ms"][0] == 12.0
    assert saved["surface_repeat_ms"][0] == 8.0
    dests = [item["dest"] for item in adapter.profile_calls]
    assert len(dests) == len(set(dests))
    assert sum(item["cold_start"] for item in adapter.profile_calls) == 4 * 30


def test_completed_receipt_is_integrated_without_counting_as_subscription():
    expected = {"epoch": 1, "sensor_id": "cam0", "observation_id": "obs-1"}
    response = {
        "status": "COMPLETED",
        "receipt": {
            "status": "INTEGRATED",
            "sensor_id": "cam0",
            "observation_id": "obs-1",
            "identity": {"epoch": 1},
        },
    }
    got = campaign_executor.interpret_observation_status(response, expected, subscribed_receipt=False)
    assert got["disposition"] == "integrated"
    assert got["receipt_status"] == "INTEGRATED"
    assert got["receipt_received"] is False
    subscribed = campaign_executor.interpret_observation_status(response, expected, subscribed_receipt=True)
    assert subscribed["receipt_received"] is True
    for top in ("REJECTED", "EXPIRED", "NOT_SEEN"):
        failed = campaign_executor.interpret_observation_status(
            {"status": top, "receipt": {"status": top, "sensor_id": "cam0", "observation_id": "obs-1"}},
            expected)
        assert failed["disposition"] == top
        assert failed["disposition"] != "integrated"


def test_snapshot_identity_faults_are_not_overwritten():
    from types import SimpleNamespace as NS

    def identity(epoch, observed, revision=1):
        return NS(epoch=epoch, map_revision=revision, observed_revision=observed,
                  last_integrated_stamp_ns=100, geometry_hash="geometry",
                  config_digest="config", clock_domain="ros")

    def page(ident, occupied=True):
        return NS(status="OK", done=True, identity=ident, next_cursor=1,
                  x=[0], y=[0], active_count=[1], known_count=[1],
                  has_occupied=[occupied], top_z=[0.02])

    def run(wait_identity, read_identity, handle="lease-1"):
        class Session:
            node = NS(create_client=lambda *args: args[1])

            def wait_for(self, *args):
                return NS(status="OK", reason="", age_ns=10, handle=handle, identity=wait_identity)

            def wait_service(self, *args, **kwargs):
                pass

            def call(self, client, request, timeout):
                if str(client).endswith("/read_surface"):
                    return page(read_identity)
                if str(client).endswith("/read_cells"):
                    return NS(status="OK", identity=read_identity, states=[2] * 4096,
                              log_odds=[0.1] * 4096, done=True, next_cursor=4096)
                return NS(status="OK")

        return campaign_executor._read_snapshot_chain(Session(), {"epoch": 1, "observation_id": "target"})

    healthy = run(identity(1, 7), identity(1, 7))
    assert healthy["identity_stable"] is True
    assert healthy["handle"] == "lease-1"
    assert run(identity(1, 7), identity(2, 1))["identity_stable"] is False
    assert run(identity(1, 7), identity(1, 6))["identity_stable"] is False
    assert run(identity(1, 7, 2), identity(1, 7, 1))["identity_stable"] is False
    assert run(identity(1, 7), identity(1, 7), handle="")["identity_stable"] is False


def test_short_roi_and_omitted_coverage_do_not_pass():
    from types import SimpleNamespace as NS

    def identity(observed):
        return NS(epoch=1, map_revision=1, observed_revision=observed,
                  last_integrated_stamp_ns=100, geometry_hash="geometry",
                  config_digest="config", clock_domain="ros")

    def run(count, observed=7, minimum=None, states=None, odds=None, done=True,
            columns=None, shape=None, oracle=None, next_cursor=None):
        ident = identity(observed)
        columns = columns or {"x": [0], "y": [0], "active_count": [1], "known_count": [1],
                              "has_occupied": [True], "top_z": [0.02]}
        width = len(columns["x"])
        class Session:
            node = NS(create_client=lambda *args: args[1])

            def wait_for(self, *args):
                return NS(status="OK", reason="", age_ns=10, handle="lease", identity=ident)

            def wait_service(self, *args, **kwargs):
                pass

            def call(self, client, request, timeout):
                if str(client).endswith("/read_surface"):
                    return NS(status="OK", done=True, identity=ident,
                              next_cursor=0 if next_cursor is None else next_cursor,
                              **columns)
                if str(client).endswith("/read_cells"):
                    payload = [2] * count if states is None else states
                    return NS(status="OK", identity=ident, states=payload,
                              log_odds=[0.1] * len(payload) if odds is None else odds,
                              done=done, next_cursor=len(payload))
                return NS(status="OK")

        expected = {"epoch": 1, "observation_id": "target",
                    "geometry_hash": "geometry", "config_digest": "config", "clock_domain": "ros",
                    "roi_states": [2] * 4096}
        if minimum is not None:
            expected["min_observed_revision"] = minimum
        if shape is not None:
            expected["surface_shape"] = shape
        if oracle is not None:
            expected["surface_columns"] = oracle
        return campaign_executor._read_snapshot_chain(Session(), expected)

    healthy_columns = [[0], [0], [1], [1], [True], [0.02]]
    healthy = run(4096, shape=[1, 1], next_cursor=1, oracle=[healthy_columns])
    assert healthy["identity_stable"] is True, healthy["reasons"]
    assert healthy["content_sha256"] == campaign_executor._content_sha256(healthy["content_material"])
    short = run(1, shape=[1, 1], next_cursor=1)
    assert short["identity_stable"] is False
    assert "missing_roi" in short["reasons"]
    assert run(4096, observed=6, minimum=7, shape=[1, 1], next_cursor=1)["identity_stable"] is False
    omitted = run(4096, observed=6, shape=[1, 1], next_cursor=1)
    assert omitted["identity_stable"] is True
    assert "target_not_covered" not in omitted["reasons"]
    duplicate = run(4096, shape=[1, 1], next_cursor=2, columns={
        "x": [0, 0], "y": [0, 0], "active_count": [1, 1], "known_count": [1, 1],
        "has_occupied": [True, True], "top_z": [0.02, 0.02]})
    assert "duplicate_column" in duplicate["reasons"]
    missing = run(4096, shape=[1, 2], next_cursor=1)
    assert "missing_column" in missing["reasons"]
    outside = run(4096, shape=[1, 1], next_cursor=1, columns={
        "x": [4], "y": [0], "active_count": [1], "known_count": [1],
        "has_occupied": [True], "top_z": [0.02]})
    assert "column_range" in outside["reasons"]
    wrong = run(4096, states=[1] * 4096, shape=[1, 1], next_cursor=1)
    assert "content_oracle" in wrong["reasons"]
    assert wrong["identity_stable"] is False


def test_injected_query_time_does_not_stretch_sends(tmp_path):
    plan = _plan()
    plan["o7_windows"] = [plan["o7_windows"][0]]

    class SlowQuery(campaign_executor.OfflineAdapter):
        def ros_query(self, window, index):
            result = super().ros_query(window, index)
            self.clock.advance(1.0)
            return result

    adapter = SlowQuery()
    campaign_executor.schedule(
        plan, adapter, tmp_path, stages=("o7",), input_limit=2,
        allow_incomplete_dependencies=True)
    sends = [entry for entry in adapter.order if entry[0] == "send"]
    assert [entry[3] for entry in sends] == pytest.approx([0.0, 0.5])
    assert [entry[4] for entry in sends] == pytest.approx([0.0, 0.5])
    assert adapter.clock.elapsed_s > 1.0


def test_authorized_live_route_does_not_start_official_windows(tmp_path, monkeypatch, capsys):
    monkeypatch.setenv("ULD_OCC_S3_OPEN", "1")
    monkeypatch.setenv("ULD_OCC_S2_CONFIRM_OFFICIAL", "1")
    monkeypatch.setenv("ULD_OCC_S2_ADAPTER", "live")
    code = campaign_executor.main(["--mode", "stage", "--stage", "o5", "--out", str(tmp_path)])
    payload = json.loads(capsys.readouterr().out)
    assert code == 2
    assert payload["adapter"] == "LiveCampaign"
    assert payload["reason"] == "official_long_window_not_started"
    assert payload["started"]["o5"] == 0
    assert payload["official_samples"] == "not_run"


def test_open_stage_dispatches_the_injected_runner(tmp_path, monkeypatch):
    monkeypatch.delenv("ULD_OCC_S3_OPEN", raising=False)
    closed = campaign_executor.main(["--mode", "stage", "--stage", "o5", "--out", str(tmp_path / "closed")])
    assert closed == 2
    monkeypatch.setenv("ULD_OCC_S3_OPEN", "1")
    monkeypatch.delenv("ULD_OCC_S2_CONFIRM_OFFICIAL", raising=False)
    monkeypatch.setenv("ULD_OCC_S2_ADAPTER", "live")
    refused = campaign_executor.main(["--mode", "stage", "--stage", "all", "--out", str(tmp_path / "live")])
    assert refused == 2
    monkeypatch.setenv("ULD_OCC_S2_ADAPTER", "injected")
    opened = campaign_executor.main(["--mode", "stage", "--stage", "o5", "--out", str(tmp_path / "open")])
    assert opened == 0
    verdict = json.loads((tmp_path / "open" / "o5" / "verdict.json").read_text())
    assert verdict["pass"] is True
    assert os.environ["ULD_OCC_S3_OPEN"] == "1"


def test_ready_wait_respects_timeout_and_the_eight_limit(tmp_path):
    import threading
    import time
    import yaml
    parent = str(Path(__file__).resolve().parent)
    if parent not in sys.path:
        sys.path.insert(0, parent)
    import s2_rehearsal
    node = "/tmp/s2-install/uld_occ_observer/lib/uld_occ_observer/observer_node"
    geometry = "/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures/geometry_chamfer.yaml"
    config = "/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures/config_chamfer_r0p02.yaml"
    if not Path(node).is_file():
        pytest.fail("observer node is not built")
    geometry_hash = json.loads(yaml.safe_load(Path(geometry).read_text())["descriptor_json"])["geometry_hash"]
    os.environ["ROS_DOMAIN_ID"] = "49"
    import rclpy
    rclpy.init()
    env = os.environ.copy()
    session = s2_rehearsal.Session()
    observer = s2_rehearsal.start_observer(node, geometry, config, tmp_path, [
        "-p", "input_mode:=service", "-p", "sensor_id:=cam0", "-p", "behavior_test_hooks:=true",
    ], env)
    try:
        session.wait_service(session.integrate)
        session.wait_service(session.hook)
        session.wait_service(session.wait)
        first = s2_rehearsal._integrate(session, geometry_hash, "frame-a")
        assert first.receipt.status == "INTEGRATED"
        session.hook_set("hold_integrate", 1)
        blocked = {}

        def second():
            try:
                blocked["receipt"] = s2_rehearsal._integrate(session, geometry_hash, "frame-b")
            except Exception as exc:
                blocked["error"] = str(exc)

        worker = threading.Thread(target=second)
        worker.start()
        time.sleep(0.3)
        started = time.monotonic()
        waited = session.wait_for(1, "frame-a", 300)
        elapsed = time.monotonic() - started
        assert elapsed < 0.8, elapsed
        assert waited.status == "TIMEOUT", waited.status
        assert not waited.handle
        requests = []
        for index in range(8):
            request = s2_rehearsal.WaitSnapshot.Request()
            request.expected_epoch = 1
            request.sensor_id = "cam0"
            request.observation_id = "frame-a"
            request.timeout_ms = 2000
            request.max_age_ms = 2000
            requests.append(session.wait.call_async(request))
        time.sleep(0.1)
        started = time.monotonic()
        ninth = session.wait_for(1, "pressure-9", 400)
        assert time.monotonic() - started < 0.5
        assert ninth.status == "WAIT_PRESSURE", ninth.status
        assert not ninth.handle
        session.hook_set("hold_integrate", 0)
        worker.join(timeout=5)
        assert worker.is_alive() is False
        from uld_occ_observer.srv import ReleaseSnapshot
        release = session.node.create_client(ReleaseSnapshot, "/uld_occ_observer/release_snapshot")
        session.wait_service(release, 5)
        deadline = time.monotonic() + 2
        for future in requests:
            while not future.done():
                if time.monotonic() > deadline:
                    raise AssertionError("ready wait stayed blocked after the store lock was released")
                time.sleep(0.01)
            result = future.result()
            assert result.status == "OK", result.status
            assert result.handle
            freed = ReleaseSnapshot.Request()
            freed.handle = result.handle
            assert session.call(release, freed, 5).status == "OK"
        again = session.wait_for(1, "frame-a", 1000)
        assert again.status == "OK" and again.handle
        freed = ReleaseSnapshot.Request()
        freed.handle = again.handle
        assert session.call(release, freed, 5).status == "OK"
    finally:
        try:
            session.hook_set("hold_integrate", 0)
        except Exception:
            pass
        session.close()
        observer.stop()
        if rclpy.ok():
            rclpy.shutdown()
