"""Boundary localization: where does the first deviation appear?

Verdicts (plan §1): "jtc_output" | "plugin_write" | "physical_response" |
"jtc_interface_unseparated" | "insufficient_evidence". All comparisons use
per-cycle samples only — NO interpolation between knots/samples (known
>0.01 rad interpolation artifacts in this campaign's history).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional
import json

import numpy as np

from .records_schema import load_run, WRITE_BRANCH_NAMES

VERDICT_JTC = "jtc_output"
VERDICT_PLUGIN = "plugin_write"
VERDICT_PHYSICS = "physical_response"
VERDICT_UNSEPARATED = "jtc_interface_unseparated"
VERDICT_INSUFFICIENT = "insufficient_evidence"


@dataclass
class BoundaryVerdict:
    verdict: str
    boundary: Optional[str] = None
    first_deviation_sim_ns: Optional[int] = None
    details: dict = field(default_factory=dict)


def _join_l2_l3(desired_times: np.ndarray, desired_pos: np.ndarray,
                write_times: np.ndarray, write_cmd: np.ndarray,
                max_offset_steps: int = 12) -> tuple:
    """Discover the fixed cycle offset with the best exact-ns match rate.

    Returns (offset_ns, coverage_fraction, (times, desired, cmd) or None).
    Exact timestamp equality = same control cycle; anything not matched is
    reported as coverage loss, never interpolated.
    """
    best = (0, 0.0, None)
    base = np.min(np.diff(write_times)) if len(write_times) > 1 else 1
    for k in range(max_offset_steps + 1):
        offset = k * base
        idx = np.searchsorted(write_times, desired_times + offset)
        idx_valid = idx < len(write_times)
        if not np.any(idx_valid):
            continue
        iv = idx[idx_valid]
        exact = write_times[iv] == (desired_times[idx_valid] + offset)
        n = int(np.sum(exact))
        cov = n / max(1, len(desired_times))
        if cov > best[1]:
            best = (offset, cov, (
                desired_times[idx_valid][exact],
                desired_pos[idx_valid][exact],
                write_cmd[iv[exact]]))
    return best


def analyze_run(run_dir: str | Path,
                controller_state_jsonl: Optional[str | Path] = None,
                joint_index: int = 0,
                pos_tol_rad: float = 0.01,
                vel_tol_rad_s: float = 0.05,
                position_branch: int = 3) -> BoundaryVerdict:
    """Localize the first deviation across the captured boundaries.

    Layers (per-cycle rows only):
      L2 desired  : JTC controller_state desired positions (50 Hz samples)
      L3 received : plugin write() cmd_received (= command-interface value)
      L3 applied  : plugin write() write_applied (post-conversion velocity)
      L4 actual   : plugin read() position/velocity + computed effort
    """
    run = load_run(run_dir)
    details: dict = {}
    if "plugin_write" not in run or "plugin_read" not in run:
        return BoundaryVerdict(
            VERDICT_INSUFFICIENT,
            details={"reason": "missing plugin streams",
                     "streams": sorted(k for k in run if k != "run_dir")})
    w = run["plugin_write"].records
    r = run["plugin_read"].records

    # Self-consistency of L3 (instrumentation sanity): recompute the P
    # conversion from the recorded inputs for position-branch rows.
    pv = w["branch"][:, joint_index] == position_branch
    if np.any(pv):
        update_rate = w["update_rate"][pv].astype(float)
        # gain is not recorded per row; solve it from the rows themselves.
        err = w["position_used"][pv, joint_index] - w["cmd_received"][pv, joint_index]
        vel = w["write_applied"][pv, joint_index]
        ok = np.abs(err) > 1e-9
        if np.any(ok):
            gains = -vel[ok] / (err[ok] * update_rate[ok])
            details["position_gain_estimate_median"] = float(np.median(gains))
            details["position_gain_estimate_spread"] = float(
                np.max(np.abs(gains - np.median(gains))))

    # L2 <-> L3 boundary (requires the JTC controller_state JSONL capture).
    l2_dev = None
    if controller_state_jsonl is not None:
        cs_path = Path(controller_state_jsonl)
        times, desired = [], []
        with open(cs_path, encoding="utf-8") as handle:
            for line in handle:
                try:
                    row = json.loads(line)
                except json.JSONDecodeError:
                    continue
                msg = row.get("msg") or row
                stamp = row.get("source_stamp_ns") or _header_stamp_ns(msg)
                des = (msg.get("desired") or {}).get("positions")
                if stamp is None or not des:
                    continue
                times.append(int(stamp))
                desired.append(float(des[joint_index]))
        if times:
            t2 = np.array(times, dtype=np.int64)
            d2 = np.array(desired)
            order = np.argsort(t2)
            t2, d2 = t2[order], d2[order]
            off, cov, matched = _join_l2_l3(
                t2, d2, w["sim_time_ns"], w["cmd_received"][:, joint_index])
            details["l2_l3_join_offset_ns"] = int(off)
            details["l2_l3_measured_coverage"] = float(cov)
            if matched is not None and cov > 0:
                mt, a, b = matched
                diff = np.abs(a - b)
                bad = np.flatnonzero(diff > pos_tol_rad)
                if len(bad):
                    l2_dev = int(mt[bad[0]])
                    details["l2_l3_first_excess_sim_ns"] = l2_dev
                details["l2_l3_max_abs_diff_rad"] = (
                    float(np.max(diff)) if len(diff) else None)
                details["l2_l3_excess_fraction"] = float(
                    len(bad) / max(1, len(diff)))

    # L3 applied <-> L4 actual boundary: matched by exact sim time. In the
    # real chain PreUpdate(t) applies the write, then the same iteration's
    # PostUpdate(t) runs the gated read — identical sim stamps.
    wt = w["sim_time_ns"]
    rt = r["sim_time_ns"]
    applied = w["write_applied"][:, joint_index]
    v_act = r["velocity"][:, joint_index]
    idx = np.searchsorted(wt, rt)
    ok = idx < len(wt)
    idx_c = idx[ok]
    exact = wt[idx_c] == rt[ok]
    details["l3_l4_exact_matches"] = int(np.sum(exact))
    l3l4_dev = None
    if np.any(exact):
        vel_err = np.abs(v_act[ok][exact] - applied[ok][exact])
        bad = np.flatnonzero(vel_err > vel_tol_rad_s)
        details["l3_l4_vel_err_max"] = float(np.max(vel_err))
        details["l3_l4_vel_err_p99"] = float(np.percentile(vel_err, 99))
        if len(bad):
            l3l4_dev = int(rt[ok][exact][bad[0]])
            details["l3_l4_first_excess_sim_ns"] = l3l4_dev

    # Conversion self-consistency: a large spread means the recorded applied
    # values do not follow the recorded inputs — the plugin conversion layer
    # (or the instrumentation) is inconsistent, which is a plugin-side
    # verdict, not a physics one.
    gain_spread = details.get("position_gain_estimate_spread")
    if gain_spread is not None and gain_spread > 1e-3:
        return BoundaryVerdict(
            VERDICT_PLUGIN, boundary="position_conversion_self_consistency",
            details=details)

    if l2_dev is not None or l3l4_dev is not None:
        if l2_dev is not None:
            return BoundaryVerdict(
                VERDICT_UNSEPARATED, boundary="jtc_output_vs_plugin_receive",
                first_deviation_sim_ns=l2_dev, details=details)
        return BoundaryVerdict(
            VERDICT_PHYSICS, boundary="plugin_applied_vs_physical_response",
            first_deviation_sim_ns=l3l4_dev, details=details)
    return BoundaryVerdict(VERDICT_INSUFFICIENT, details=details)


def _header_stamp_ns(msg: dict):
    stamp = (msg.get("header") or {}).get("stamp")
    if not stamp:
        return None
    sec = stamp.get("sec", 0)
    nan = stamp.get("nanosec", 0)
    return int(sec) * 1_000_000_000 + int(nan)
