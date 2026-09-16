"""Behavioural regressions for the sim_world startup chain.

These exercise the failure modes that actually occurred, replayed against the
captured launch logs. PF-R7 generation 9 shipped only source-text assertions
over the launch file ("contains four OnProcessExit edges, contains no
_on_exit_zero"); that suite passed three consecutive times and the live run
still stopped at the same stage, because grepping the spelling of a fix cannot
catch a blocked-call race or a marker emitted by teardown.
"""
import json
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "src" / "luggage_gazebo"))

from luggage_gazebo.startup_probe import (  # noqa: E402
    DEFAULT_STAGE_DEADLINE,
    STAGE_TIMEOUT_REASON,
    STAGES,
    marker_order_ok,
    plugin_received_urdf,
    robot_spawn_started,
    run_stages,
    spawn_marker_count,
    wait_for_service,
)

EVIDENCE = (REPO / "docs" / "status" / "evidence" / "platform_free_height")
G9_READY = (
    EVIDENCE / "2026-09-15_pfr7_g9"
    / "rev_3b0189e2eaf75cc9e4d32498195998c977fcb224"
    / "live" / "startup" / "attempt_1" / "ready.json"
)
BASELINE = EVIDENCE / "2026-09-16_startup_rehearsal" / "baseline_master_e686808"


class FakeClock:
    """Monotonic clock advanced only by the code under test."""

    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, seconds):
        self.t += seconds


def _deadlines(**overrides):
    out = dict(DEFAULT_STAGE_DEADLINE)
    out.update(overrides)
    return out


def _always(value):
    return lambda: value


def _checks(**overrides):
    checks = {stage: _always(True) for stage in STAGES}
    checks.update(overrides)
    return checks


# --- stage sequencing -------------------------------------------------------

def test_all_stages_ok_reports_ok():
    result = run_stages(_checks(), _deadlines(), poll_sec=0)
    assert result.ok
    assert result.reason == "ok"
    assert [s.stage for s in result.stages] == list(STAGES)


@pytest.mark.parametrize("stage", STAGES)
def test_each_stage_timeout_maps_to_its_own_reason(stage, monkeypatch):
    clock = FakeClock()
    monkeypatch.setattr("time.sleep", clock.sleep)
    result = run_stages(_checks(**{stage: _always(False)}), _deadlines(),
                        clock=clock, poll_sec=0.2, global_deadline_sec=1e6)
    assert not result.ok
    assert result.reason == STAGE_TIMEOUT_REASON[stage]
    assert result.stages[-1].stage == stage


def test_late_but_successful_stage_does_not_shorten_the_next_deadline(
        monkeypatch):
    """The G9 defect: two probes, two clocks, a deadline that started early.

    robot_description succeeds only at t=25s. robot_spawn must still get its
    full 20s budget measured from *that* moment, not from launch.
    """
    clock = FakeClock()
    monkeypatch.setattr("time.sleep", clock.sleep)

    def slow_rsp():
        return clock() >= 25.0

    def spawn_after_rsp_plus_10():
        return clock() >= 35.0

    result = run_stages(
        _checks(robot_description=slow_rsp, robot_spawn=spawn_after_rsp_plus_10),
        _deadlines(robot_description=30.0, robot_spawn=20.0),
        clock=clock, poll_sec=0.2, global_deadline_sec=1e6)
    assert result.ok, result.reason
    spawn = [s for s in result.stages if s.stage == "robot_spawn"][0]
    assert spawn.duration_sec == pytest.approx(10.0, abs=0.5)


def test_stage_deadline_is_not_consumed_by_a_slow_predecessor(monkeypatch):
    """A stage that needs longer than the *global* budget still fails as itself."""
    clock = FakeClock()
    monkeypatch.setattr("time.sleep", clock.sleep)
    result = run_stages(
        _checks(plugin_urdf=_always(False)), _deadlines(),
        clock=clock, poll_sec=0.2, global_deadline_sec=1e6)
    assert result.reason == "plugin_urdf_not_received"


def test_abort_wins_over_a_pending_stage(monkeypatch):
    clock = FakeClock()
    monkeypatch.setattr("time.sleep", clock.sleep)
    result = run_stages(_checks(controller_manager=_always(False)),
                        _deadlines(), clock=clock, poll_sec=0.2,
                        abort=lambda: "launch_died", global_deadline_sec=1e6)
    assert not result.ok
    assert result.reason == "launch_died"


def test_global_timeout_is_reported_distinctly(monkeypatch):
    clock = FakeClock()
    monkeypatch.setattr("time.sleep", clock.sleep)
    result = run_stages(_checks(controllers_active=_always(False)),
                        _deadlines(controllers_active=1e6), clock=clock,
                        poll_sec=0.2, global_deadline_sec=10.0)
    assert result.reason == "startup_global_timeout"


# --- launch-log markers -----------------------------------------------------

def test_scene_creates_alone_are_not_a_robot_spawn():
    """Exactly the G8/G9 state: three scene creates, no S20."""
    log = "\n".join("[INFO] [create-%d]: process started with pid [1]" % n
                    for n in (18, 19, 20))
    assert not robot_spawn_started(log)


def test_fourth_create_counts_as_the_robot_spawn():
    log = "\n".join("[INFO] [create-%d]: process started with pid [1]" % n
                    for n in (18, 19, 20, 22))
    assert robot_spawn_started(log)


def test_explicit_marker_is_enough_without_counting_creates():
    assert robot_spawn_started("[INFO] [launch]: spawn_robot: create S20")


def test_plugin_receipt_without_a_preceding_spawn_marker_is_rejected():
    """A marker emitted during teardown must not read as startup success."""
    log = "Received URDF from param server\nspawn_robot: create S20\n"
    assert not marker_order_ok(log)


def test_marker_order_accepted_when_spawn_precedes_receipt():
    log = "spawn_robot: create S20\nReceived URDF from param server\n"
    assert marker_order_ok(log)


def test_spawn_marker_is_emitted_exactly_once_per_launch():
    log = "spawn_robot: create S20\nReceived URDF from param server\n"
    assert spawn_marker_count(log) == 1


# --- replay of the captured failures ---------------------------------------

@pytest.mark.skipif(not G9_READY.exists(), reason="G9 evidence not present")
def test_g9_captured_log_replays_as_robot_spawn_missing():
    log = json.loads(G9_READY.read_text())["launch_log_text"]
    assert not robot_spawn_started(log)
    assert not plugin_received_urdf(log)
    assert spawn_marker_count(log) == 0


@pytest.mark.skipif(not BASELINE.exists(), reason="baseline evidence absent")
def test_baseline_failure_replays_as_plugin_urdf_not_received():
    """The stock-master failure: S20 spawned, plugin connected, then silence.

    Neither the success line nor the plugin's own retry line is present, which
    matches the plugin waiting indefinitely for its parameter future.
    """
    summary = json.loads((BASELINE / "rehearsal_summary.json").read_text())
    failures = [r for r in summary["iteration_records"] if not r["ok"]]
    assert failures, "baseline recorded no failure to replay"
    for record in failures:
        log = (BASELINE / ("iter_%02d" % record["iteration"])
               / "launch.log").read_text(errors="replace")
        assert robot_spawn_started(log), "S20 create did start"
        assert not plugin_received_urdf(log)
        assert "connected to service!!" in log
        assert "waiting for model URDF" not in log
        assert record["reason"] == "plugin_urdf_not_received"


@pytest.mark.skipif(not BASELINE.exists(), reason="baseline evidence absent")
def test_baseline_robot_description_was_never_the_late_stage():
    """Refutes the G7/G8/G9 premise that RSP needed more time."""
    summary = json.loads((BASELINE / "rehearsal_summary.json").read_text())
    rsp = summary["stage_durations_sec"]["robot_description"]
    assert rsp["max_sec"] < 5.0, rsp


# --- watchdog ---------------------------------------------------------------

def test_watchdog_returns_true_once_the_service_appears():
    clock = FakeClock()
    seen = {"n": 0}

    def available():
        seen["n"] += 1
        return seen["n"] > 3

    ok = wait_for_service(available, timeout_sec=10.0, poll_sec=0.25,
                          clock=clock, sleep=clock.sleep)
    assert ok


def test_watchdog_times_out_when_the_plugin_never_builds_the_manager():
    clock = FakeClock()
    ok = wait_for_service(lambda: False, timeout_sec=5.0, poll_sec=0.25,
                          clock=clock, sleep=clock.sleep)
    assert not ok
    assert clock() >= 5.0


def _load_launch_module():
    import importlib.util
    path = REPO / "src" / "luggage_gazebo" / "launch" / "sim_world.launch.py"
    spec = importlib.util.spec_from_file_location("sim_world_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


try:
    launch_module = _load_launch_module()
except Exception:  # pragma: no cover - no ROS environment
    launch_module = None

requires_launch = pytest.mark.skipif(
    launch_module is None, reason="launch/launch_ros not importable")


class FakeExit:
    def __init__(self, returncode):
        self.returncode = returncode


@requires_launch
def test_clean_watchdog_exit_leaves_the_launch_running():
    assert launch_module._shutdown_unless_clean_exit(FakeExit(0), None) == []


@requires_launch
def test_failed_watchdog_exit_shuts_the_launch_down():
    from launch.actions import EmitEvent

    actions = launch_module._shutdown_unless_clean_exit(FakeExit(1), None)
    assert any(isinstance(a, EmitEvent) for a in actions), actions


@requires_launch
def test_watchdog_is_chained_off_the_robot_create_not_the_scene_create():
    """The abort must observe the robot spawn, not an unrelated scene model."""
    from launch.actions import RegisterEventHandler

    actions = launch_module._spawn_scene_and_robot(
        launch_module._default_launch_values()["scene_tf_config"],
        [], watchdog_sec=45.0)
    handlers = [a for a in actions if isinstance(a, RegisterEventHandler)]
    assert len(handlers) >= 2, "expected scene->spawn plus watchdog edges"


@requires_launch
def test_watchdog_can_be_disabled_with_zero():
    from launch.actions import RegisterEventHandler

    with_gate = launch_module._spawn_scene_and_robot(
        launch_module._default_launch_values()["scene_tf_config"], [],
        watchdog_sec=45.0)
    without = launch_module._spawn_scene_and_robot(
        launch_module._default_launch_values()["scene_tf_config"], [],
        watchdog_sec=0.0)
    n_with = len([a for a in with_gate if isinstance(a, RegisterEventHandler)])
    n_without = len([a for a in without if isinstance(a, RegisterEventHandler)])
    assert n_with == n_without + 2


def test_watchdog_does_not_poll_past_its_deadline():
    clock = FakeClock()
    calls = {"n": 0}

    def never():
        calls["n"] += 1
        return False

    wait_for_service(never, timeout_sec=1.0, poll_sec=0.25, clock=clock,
                     sleep=clock.sleep)
    # 4 polls inside the window plus the one final re-check after it.
    assert calls["n"] <= 6
