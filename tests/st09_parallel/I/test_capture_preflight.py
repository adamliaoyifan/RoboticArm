"""The reference-capture preflight judges stream content, not topic names."""
import importlib.util
import math
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]


def _tool():
    spec = importlib.util.spec_from_file_location(
        "capture_preflight",
        REPO / "scripts" / "st09_i" / "capture_preflight.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


TOOL = _tool()
NAMES = list(TOOL.JOINTS)
SIX = [0.1, -1.2, 1.8, -2.2, -1.5, -0.6]
END = 1000.0


def controller_rows(n=150, rate=50.0, start=10.0, names=None, mutate=None,
                    last_rx=None):
    rows = []
    for k in range(n):
        row = {
            "stamp": start + k / rate,
            "rx_mono": END - (n - 1 - k) / rate if last_rx is None
            else last_rx - (n - 1 - k) / rate,
            "names": list(names or NAMES),
            "reference": list(SIX), "feedback": list(SIX),
            "error": [0.0] * 6,
        }
        if mutate:
            mutate(k, row)
        rows.append(row)
    return rows


def joint_rows(n=300, rate=100.0, start=10.0, last_rx=None, names=None,
               mutate=None):
    rows = []
    for k in range(n):
        row = {
            "stamp": start + k / rate,
            "rx_mono": END - (n - 1 - k) / rate if last_rx is None
            else last_rx - (n - 1 - k) / rate,
            "names": list(names or NAMES),
            "position": list(SIX),
        }
        if mutate:
            mutate(k, row)
        rows.append(row)
    return rows


def verdict(controller=None, joints=None):
    return TOOL.evaluate_capture(
        controller_rows() if controller is None else controller,
        joint_rows() if joints is None else joints, END)


def test_healthy_streams_pass_every_check():
    result = verdict()
    assert result["ok"], result["failed"]
    assert result["metrics"]["controller_state"]["rate_hz"] >= 49.0
    assert result["metrics"]["feedback_vs_joint_states_max_delta_rad"] == 0.0


def test_a_topic_with_no_messages_fails_even_if_it_exists():
    result = verdict(controller=[])
    assert not result["ok"]
    assert "controller_state_messages" in result["failed"]
    assert "controller_state_still_arriving" in result["failed"]


def test_joint_states_missing_fails():
    result = verdict(joints=[])
    assert not result["ok"]
    assert "joint_states_messages" in result["failed"]


def test_missing_feedback_fails():
    def drop(k, row):
        if k == 70:
            row["feedback"] = []
    assert "controller_state_feedback_present" in verdict(
        controller=controller_rows(mutate=drop))["failed"]


def test_missing_reference_and_error_fail_separately():
    def drop(k, row):
        row["reference"] = []
        row["error"] = [float("nan")] * 6
    failed = verdict(controller=controller_rows(mutate=drop))["failed"]
    assert "controller_state_reference_present" in failed
    assert "controller_state_error_present" in failed


def test_wrong_joint_names_fail():
    failed = verdict(controller=controller_rows(
        names=["a", "b", "c", "d", "e", "f"]))["failed"]
    assert "controller_state_joint_names" in failed


def test_permuted_joint_names_are_accepted():
    order = [NAMES[i] for i in (0, 1, 4, 2, 3, 5)]
    assert verdict(controller=controller_rows(names=order))["ok"]


def test_frozen_controller_stamp_fails():
    failed = verdict(controller=controller_rows(
        mutate=lambda k, row: row.update(stamp=10.0)))["failed"]
    assert "controller_state_stamps_advance" in failed


def test_low_rate_fails():
    failed = verdict(controller=controller_rows(n=110, rate=10.0))["failed"]
    assert "controller_state_rate" in failed


def test_non_overlapping_streams_fail():
    failed = verdict(joints=joint_rows(start=500.0))["failed"]
    assert "streams_overlap" in failed


def test_stream_that_stopped_before_the_window_ended_fails():
    failed = verdict(controller=controller_rows(last_rx=END - 2.0))["failed"]
    assert "controller_state_still_arriving" in failed
    failed = verdict(joints=joint_rows(last_rx=END - 2.0))["failed"]
    assert "joint_states_still_arriving" in failed


def test_non_finite_joint_position_fails():
    def bad(k, row):
        if k == 5:
            row["position"] = [math.nan] * 6
    failed = verdict(joints=joint_rows(mutate=bad))["failed"]
    assert "joint_states_names_finite" in failed


def test_joint_states_missing_a_joint_fails():
    failed = verdict(joints=joint_rows(
        names=NAMES[:5] + ["elfin_joint7"]))["failed"]
    assert "joint_states_names_finite" in failed


def test_tool_topic_matches_the_driver_and_the_bag():
    import sys
    sys.path.insert(0, str(REPO / "src" / "luggage_gazebo"))
    from luggage_gazebo.capture_budget import (
        CONTROLLER_STATE_TOPIC, bag_topics)
    assert TOOL.CONTROLLER_STATE_TOPIC == CONTROLLER_STATE_TOPIC
    assert CONTROLLER_STATE_TOPIC in bag_topics()
