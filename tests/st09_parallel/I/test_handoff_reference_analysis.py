"""Alignment maths for reference vs feedback vs independent joint states."""
import importlib.util
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]


def _tool():
    spec = importlib.util.spec_from_file_location(
        "handoff_reference_analysis",
        REPO / "scripts" / "st09_i" / "handoff_reference_analysis.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


TOOL = _tool()
Q0 = np.array([0.0, -1.7, 0.5, 0.0, 0.0, 0.0])
Q1 = np.array([-2.2, -1.2, 1.9, -2.2, -1.5, -0.6])
ACCEPT, DURATION = 100.0, 10.0


def build(actual_fn, success_offset=0.0, tail=8.0):
    ct = np.arange(ACCEPT - 1.0, ACCEPT + DURATION + tail, 0.02)
    frac = np.clip((ct - ACCEPT) / DURATION, 0.0, 1.0)[:, None]
    ref = Q0 + frac * (Q1 - Q0)
    fb = ref.copy()                      # open loop: feedback == reference
    err = np.zeros_like(ref)
    js_t = np.arange(ACCEPT - 1.0, ACCEPT + DURATION + tail, 0.01)
    jfrac = np.clip((js_t - ACCEPT) / DURATION, 0.0, 1.0)[:, None]
    js_ref = Q0 + jfrac * (Q1 - Q0)
    js_q = actual_fn(js_t, js_ref)
    return TOOL.analyze_leg(
        ct, ref, fb, err, js_t, js_q, ACCEPT, ACCEPT + DURATION + success_offset,
        DURATION, Q1)


def lagging_joint2(t, ref):
    # joint 2 stays 0.3 rad behind and closes at 0.077 rad/s after the end
    q = ref.copy()
    deficit = np.where(t < ACCEPT + DURATION, 0.3,
                       np.maximum(0.0, 0.3 - 0.077 * (t - ACCEPT - DURATION)))
    q[:, 1] -= deficit
    return q


def test_run08_shape_feedback_equals_reference_while_actual_lags():
    row = build(lagging_joint2)
    assert row["q1_max_feedback_minus_reference_rad"] < 1e-9
    assert row["q1_max_actual_minus_reference_rad"] > 0.29
    assert row["q1_max_actual_minus_reference_joint"] == "elfin_joint2"


def test_success_is_reference_end_not_arrival():
    row = build(lagging_joint2)
    assert abs(row["q2_success_minus_reference_end_sec"]) < 1e-6
    assert row["q2_reference_at_success_minus_last_knot_rad"] < 1e-6
    assert row["q2_actual_at_success_minus_last_knot_rad"] > 0.29
    assert row["q3_worst_joint_at_success"] == "elfin_joint2"


def test_convergence_time_after_success():
    row = build(lagging_joint2)
    assert row["q3_actual_minus_reference_at_success_rad"] > 0.29
    # 0.3 rad at 0.077 rad/s, to within 0.01 rad, then a 0.3 s hold.
    assert 3.6 < row["q3_success_to_converged_sec"] < 4.4


def test_a_tracking_arm_converges_immediately():
    row = build(lambda t, ref: ref.copy())
    assert row["q1_max_actual_minus_reference_rad"] < 1e-6
    assert row["q3_success_to_converged_sec"] < 0.05


def test_oscillation_with_zero_net_displacement_is_not_converged():
    def wobble(t, ref):
        q = ref.copy()
        q[:, 0] += 0.006 * np.sin(2 * np.pi * 6.0 * t)  # inside tolerance
        return q
    row = build(wobble)
    assert row["q3_converged_sim"] is None


def test_oscillation_outside_tolerance_is_not_converged():
    def wobble(t, ref):
        q = ref.copy()
        q[:, 2] += 0.03 * np.sin(2 * np.pi * 1.0 * t)
        return q
    assert build(wobble)["q3_converged_sim"] is None


def test_success_before_reference_end_is_reported():
    row = build(lambda t, ref: ref.copy(), success_offset=-2.0)
    assert row["q2_success_minus_reference_end_sec"] == -2.0
    assert row["q2_reference_at_success_minus_last_knot_rad"] > 0.1


def test_velocity_field_that_reads_still_while_moving_is_flagged():
    t = np.arange(0.0, 5.0, 0.01)
    q = np.zeros((len(t), 6))
    q[:, 1] = 0.077 * t                 # moving at 0.077 rad/s
    v = np.zeros_like(q)
    v[:, 1] = -0.0007                   # field says "still", wrong sign
    out = TOOL.velocity_reliability(t, q, v, 0.5, 4.0)
    assert out["false_still_fraction"] == 1.0
    assert out["false_moving_fraction"] == 0.0


def test_velocity_field_that_reads_moving_while_still_is_flagged():
    t = np.arange(0.0, 5.0, 0.01)
    q = np.zeros((len(t), 6))
    v = np.zeros_like(q)
    v[:, 0] = 0.05                      # would wrongly reject a still arm
    out = TOOL.velocity_reliability(t, q, v, 0.5, 4.0)
    assert out["false_moving_fraction"] == 1.0
