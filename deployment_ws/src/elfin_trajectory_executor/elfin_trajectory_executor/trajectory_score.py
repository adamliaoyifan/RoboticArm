"""实机代码 (site / real-cell). Not part of the Gazebo simulation stack.

ROS-free trajectory run scorer. Reads JSON/JSONL dumps, not CPS reported vel.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

from .cps_parse import finite_diff_deg_s
from .waypoint_profile import SITE_REJECT_ACCEL_DEG, VEL_ACCEL_MARGIN_DEG

DEADBAND_DEG_S = 2.0
GAP_FAIL_S = 0.080
REVERSE_JUMP_DEG = 0.5
FINAL_JOINT_ERR_DEG = 0.5
TCP_ERR_MM = 5.0
JS_HOLE_S = 0.100
SERVO_DT_S = 0.020
JITTER_P99_S = 0.005
JITTER_MAX_S = 0.010
COMMAND_ACCEL_MAX_DEG = 60.0
CPS_40083 = 40083
CPS_20070 = 20070


def percentile(values: Sequence[float], p: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(float(v) for v in values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * (float(p) / 100.0)
    lo = int(math.floor(rank))
    hi = int(math.ceil(rank))
    if lo == hi:
        return ordered[lo]
    w = rank - lo
    return ordered[lo] * (1.0 - w) + ordered[hi] * w


def max_abs_speed_deg_s(prev_q: Sequence[float], curr_q: Sequence[float], dt_s: float) -> Optional[float]:
    fd = finite_diff_deg_s(prev_q, curr_q, dt_s)
    if fd is None:
        return None
    return max(abs(v) for v in fd)


def internal_zero_speed_gaps(
    samples: Sequence[Dict[str, Any]],
    *,
    deadband_deg_s: float = DEADBAND_DEG_S,
) -> List[float]:
    """Internal (not leading/trailing) intervals where |qdot| < deadband.

    ``samples`` items: ``{"t": float, "speed_deg_s": float}`` already sorted.
    """
    if len(samples) < 2:
        return []
    slow = [
        bool(abs(float(row.get("speed_deg_s", 0.0))) < deadband_deg_s)
        for row in samples
    ]
    first_move = next((i for i, flag in enumerate(slow) if not flag), None)
    last_move = next((i for i, flag in enumerate(reversed(slow)) if not flag), None)
    if first_move is None or last_move is None:
        return []
    last_move = len(slow) - 1 - last_move
    gaps: List[float] = []
    i = first_move
    while i <= last_move:
        if not slow[i]:
            i += 1
            continue
        start = i
        while i <= last_move and slow[i]:
            i += 1
        end = i - 1
        if start <= last_move and end >= first_move:
            dt = float(samples[end]["t"]) - float(samples[start]["t"])
            if dt > 0.0:
                gaps.append(dt)
    return gaps


def reverse_jumps_deg(
    samples: Sequence[Dict[str, Any]],
    *,
    threshold_deg: float = REVERSE_JUMP_DEG,
    desired: Optional[Sequence[Sequence[float]]] = None,
) -> List[Dict[str, Any]]:
    hits: List[Dict[str, Any]] = []
    for i in range(1, len(samples)):
        prev = samples[i - 1].get("q_deg") or []
        curr = samples[i].get("q_deg") or []
        if len(prev) < 6 or len(curr) < 6:
            continue
        for j in range(6):
            dq = float(curr[j]) - float(prev[j])
            if abs(dq) <= threshold_deg:
                continue
            if i >= 2:
                prev2 = samples[i - 2].get("q_deg") or []
                if len(prev2) >= 6:
                    prev_dq = float(prev[j]) - float(prev2[j])
                    # Reverse relative to the previous step.
                    if prev_dq * dq < 0.0 and abs(dq) > threshold_deg:
                        if _desired_has_jump(desired, i, j, dq, threshold_deg):
                            continue
                        hits.append(
                            {
                                "sample": i,
                                "joint": j,
                                "delta_deg": dq,
                            }
                        )
    return hits


def _desired_has_jump(
    desired: Optional[Sequence[Sequence[float]]],
    sample_i: int,
    joint: int,
    dq: float,
    threshold_deg: float,
) -> bool:
    if not desired or sample_i >= len(desired) or sample_i < 1:
        return False
    prev = desired[sample_i - 1]
    curr = desired[sample_i]
    if len(prev) < 6 or len(curr) < 6:
        return False
    want = float(curr[joint]) - float(prev[joint])
    return abs(want) > threshold_deg and (want * dq) > 0.0


def telemetry_holes_s(
    samples: Sequence[Dict[str, Any]],
    *,
    hole_s: float = JS_HOLE_S,
) -> List[float]:
    holes: List[float] = []
    for i in range(1, len(samples)):
        dt = float(samples[i]["t"]) - float(samples[i - 1]["t"])
        if dt > hole_s:
            holes.append(dt)
    return holes


def command_grid_jitter_s(
    command_times: Sequence[float],
    *,
    dt_s: float = SERVO_DT_S,
) -> List[float]:
    if len(command_times) < 2:
        return []
    jitters: List[float] = []
    t0 = float(command_times[0])
    for i, t in enumerate(command_times):
        expected = t0 + i * dt_s
        jitters.append(abs(float(t) - expected))
    return jitters


def score_run(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Score one hardware or synthetic run. Never uses CPS velocity as truth."""
    samples = _with_speed(list(payload.get("joint_samples") or []))
    commands = list(payload.get("commands") or [])
    events = list(payload.get("events") or [])
    desired = payload.get("desired_q_deg")
    command_times = list(payload.get("command_times") or [])
    underruns = int(payload.get("underruns") or 0)
    late_refills = int(payload.get("late_refills") or 0)
    accel_cap = float(payload.get("command_acceleration_deg") or COMMAND_ACCEL_MAX_DEG)

    event_names = [str(e.get("event", e) if isinstance(e, dict) else e) for e in events]
    cps_codes = [int(c.get("code", 0)) for c in commands if isinstance(c, dict)]
    illegal_accel = []
    illegal_vel = []
    for cmd in commands:
        if not isinstance(cmd, dict):
            continue
        accel = float(cmd.get("accel_deg", 0.0))
        vel = float(cmd.get("vel_deg", 0.0))
        if accel > accel_cap + 1e-9 or accel >= SITE_REJECT_ACCEL_DEG - 1e-9:
            illegal_accel.append(cmd)
        if vel > accel - VEL_ACCEL_MARGIN_DEG + 1e-9:
            illegal_vel.append(cmd)

    gaps = internal_zero_speed_gaps(samples)
    jumps = reverse_jumps_deg(samples, desired=desired)
    holes = telemetry_holes_s(samples)
    jitters = command_grid_jitter_s(command_times) if command_times else []

    final_err = _final_joint_error_deg(samples, payload.get("goal_q_deg"))
    tcp_err = _tcp_error_mm(samples, payload.get("goal_tcp_mm"))

    fail_40083 = CPS_40083 in cps_codes
    fail_20070 = CPS_20070 in cps_codes
    other_cps = [c for c in cps_codes if c not in (0, CPS_40083, CPS_20070)]
    continuity_fail = any(g >= GAP_FAIL_S - 1e-12 for g in gaps)
    jitter_p99 = percentile(jitters, 99.0) if jitters else None
    jitter_max = max(jitters) if jitters else None

    checks = {
        "zero_40083": not fail_40083,
        "zero_20070": not fail_20070,
        "zero_other_cps": not other_cps,
        "accel_le_cap": not illegal_accel,
        "vel_le_accel_minus_1": not illegal_vel,
        "no_reverse_jump": not jumps,
        "no_js_hole": not holes,
        "final_joint_err_ok": final_err is None or final_err <= FINAL_JOINT_ERR_DEG,
        "tcp_err_ok": tcp_err is None or tcp_err <= TCP_ERR_MM,
        "continuity_no_gap_80ms": not continuity_fail,
        "zero_underrun": underruns == 0,
        "zero_late_refill": late_refills == 0,
        "jitter_p99_ok": jitter_p99 is None or jitter_p99 <= JITTER_P99_S,
        "jitter_max_ok": jitter_max is None or jitter_max <= JITTER_MAX_S,
    }
    return {
        "checks": checks,
        "events": event_names,
        "cps_codes": cps_codes,
        "illegal_accel": illegal_accel,
        "illegal_vel": illegal_vel,
        "gap_count": len(gaps),
        "gap_median_s": percentile(gaps, 50.0),
        "gap_p95_s": percentile(gaps, 95.0),
        "gap_max_s": max(gaps) if gaps else 0.0,
        "gaps_s": gaps,
        "reverse_jumps": jumps,
        "js_holes_s": holes,
        "final_joint_err_deg": final_err,
        "tcp_err_mm": tcp_err,
        "underruns": underruns,
        "late_refills": late_refills,
        "jitter_p99_s": jitter_p99,
        "jitter_max_s": jitter_max,
        "pass_40083_fix": all(
            [
                checks["zero_40083"],
                checks["zero_20070"],
                checks["zero_other_cps"],
                checks["accel_le_cap"],
                checks["vel_le_accel_minus_1"],
            ]
        ),
        "pass_continuity": checks["continuity_no_gap_80ms"]
        and checks["zero_underrun"]
        and checks["zero_late_refill"],
    }


def _with_speed(samples: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    for i, row in enumerate(samples):
        item = dict(row)
        item["t"] = float(row["t"])
        if i == 0:
            item["speed_deg_s"] = float(row.get("speed_deg_s") or 0.0)
        else:
            dt = float(row["t"]) - float(samples[i - 1]["t"])
            speed = max_abs_speed_deg_s(
                samples[i - 1].get("q_deg") or [],
                row.get("q_deg") or [],
                dt,
            )
            item["speed_deg_s"] = float(
                row["speed_deg_s"] if row.get("speed_deg_s") is not None else (speed or 0.0)
            )
        out.append(item)
    return out


def _final_joint_error_deg(
    samples: Sequence[Dict[str, Any]],
    goal_q_deg: Optional[Sequence[float]],
) -> Optional[float]:
    if not samples or not goal_q_deg or len(goal_q_deg) < 6:
        return None
    last = samples[-1].get("q_deg") or []
    if len(last) < 6:
        return None
    return max(abs(float(last[i]) - float(goal_q_deg[i])) for i in range(6))


def _tcp_error_mm(
    samples: Sequence[Dict[str, Any]],
    goal_tcp_mm: Optional[Sequence[float]],
) -> Optional[float]:
    if not samples or not goal_tcp_mm or len(goal_tcp_mm) < 3:
        return None
    last = samples[-1].get("tcp_mm") or []
    if len(last) < 3:
        return None
    return math.sqrt(
        sum((float(last[i]) - float(goal_tcp_mm[i])) ** 2 for i in range(3))
    )


def load_jsonl(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            rows.append(json.loads(line))
    return rows


def score_files(
    *,
    joints_jsonl: Optional[Path] = None,
    commands_jsonl: Optional[Path] = None,
    events_jsonl: Optional[Path] = None,
    payload_json: Optional[Path] = None,
) -> Dict[str, Any]:
    if payload_json is not None:
        return score_run(json.loads(payload_json.read_text(encoding="utf-8")))
    payload: Dict[str, Any] = {
        "joint_samples": load_jsonl(joints_jsonl) if joints_jsonl else [],
        "commands": load_jsonl(commands_jsonl) if commands_jsonl else [],
        "events": load_jsonl(events_jsonl) if events_jsonl else [],
    }
    return score_run(payload)


def write_score(score: Dict[str, Any], dest: Path) -> None:
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(score, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main(argv=None) -> int:
    import argparse
    parser = argparse.ArgumentParser(
        description="Score an Elfin real-trajectory JSON/JSONL dump (ROS-free).",
    )
    parser.add_argument("--payload", type=Path, help="Single JSON object with joint_samples/commands/events")
    parser.add_argument("--joints", type=Path, help="JSONL joint samples {t,q_deg}")
    parser.add_argument("--commands", type=Path, help="JSONL commanded waypoints")
    parser.add_argument("--events", type=Path, help="JSONL executor events")
    parser.add_argument("--out", type=Path, help="Write score.json here")
    args = parser.parse_args(argv)
    score = score_files(
        payload_json=args.payload,
        joints_jsonl=args.joints,
        commands_jsonl=args.commands,
        events_jsonl=args.events,
    )
    text = json.dumps(score, indent=2, sort_keys=True)
    if args.out:
        write_score(score, args.out)
    else:
        print(text)
    ok = bool(score.get("pass_40083_fix"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
