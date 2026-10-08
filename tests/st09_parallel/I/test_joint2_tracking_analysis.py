"""Behavioural tests for joint2_tracking_analysis on synthetic signals."""
import importlib.util
import sys
from pathlib import Path

import numpy as np

_TOOL = Path(__file__).resolve().parents[3] / "scripts/st09_i"
sys.path.insert(0, str(_TOOL))
spec = importlib.util.spec_from_file_location(
    "joint2_tracking_analysis", _TOOL / "joint2_tracking_analysis.py")
TOOL = importlib.util.module_from_spec(spec)
spec.loader.exec_module(TOOL)


def ramp(t):
    return np.clip(t - 1.0, 0.0, 1.0) * 0.4


def test_first_order_lag_is_preferred_over_pure_delay_for_a_servo_signal():
    t = np.arange(0.0, 4.0, 0.02)
    ref = ramp(t)
    fb = TOOL.simulate_first_order(t, ref, 10.0, 0.0)
    bag = {"ct": t, "fb": np.column_stack([fb] * 6)}
    ref_fn = lambda tt, j: np.interp(tt, t, ref)  # noqa: E731
    taus = TOOL.fit_delay(t, bag["fb"], ref_fn, 0.0, 4.0)
    cmp_ = TOOL.lag_model_comparison(bag, 0.0, 4.0, ref_fn, taus)
    row = cmp_["elfin_joint1"]
    assert row["better_model"] == "first_order"
    assert abs(row["first_order_time_constant_sec"] - 0.1) < 0.02


def test_pure_delay_signal_is_preferred_as_delay():
    t = np.arange(0.0, 4.0, 0.02)
    ref = ramp(t)
    fb = ramp(t - 0.3)
    bag = {"ct": t, "fb": np.column_stack([fb] * 6)}
    ref_fn = lambda tt, j: np.interp(tt, t, ref)  # noqa: E731
    taus = TOOL.fit_delay(t, bag["fb"], ref_fn, 0.0, 4.0)
    cmp_ = TOOL.lag_model_comparison(bag, 0.0, 4.0, ref_fn, taus)
    assert cmp_["elfin_joint1"]["better_model"] == "delay"
    assert abs(taus[0][0] - 0.3) < 0.02


def test_hold_decay_distinguishes_constant_rate_from_exponential():
    t = np.arange(0.0, 2.0, 0.02)
    ref = np.zeros((len(t), 6))
    fb_lin = np.zeros((len(t), 6))
    fb_lin[:, 1] = -(0.07 - 0.05 * t)             # constant 0.05 rad/s
    fb_exp = np.zeros((len(t), 6))
    fb_exp[:, 1] = -(0.07 * np.exp(-t / 0.5))     # exponential
    lin = TOOL.hold_decay({"ct": t, "fb": fb_lin, "ref": ref}, 0.0, 1.3, 1)
    exp = TOOL.hold_decay({"ct": t, "fb": fb_exp, "ref": ref}, 0.0, 1.9, 1)
    assert lin["linear_fit_rms_rad"] < 0.2 * lin["exponential_fit_rms_rad"]
    assert exp["exponential_fit_rms_rad"] < 0.2 * exp["linear_fit_rms_rad"]
    assert abs(lin["linear_rate_rad_s"] - 0.05) < 0.005


def test_delivered_ratio_flags_a_joint_stuck_below_its_demand():
    t = np.arange(0.0, 4.0, 0.02)
    ref = np.zeros((len(t), 6))
    ref[:, 1] = -0.3
    fb = np.zeros((len(t), 6))
    fb[:, 1] = -0.05 * t                       # moves at 0.05 rad/s only
    bag = {"ct": t, "fb": fb, "ref": ref}
    item = TOOL.delivered_ratio(bag, 0.0, 4.0, 1, gain=10.0)
    assert item["median"] < 0.2
    fb[:, 1] = -0.3 * (1 - np.exp(-10.0 * t))  # follows K=10
    ok = TOOL.delivered_ratio(bag, 0.0, 4.0, 1, gain=10.0)
    assert ok is None or ok["median"] > 0.8
