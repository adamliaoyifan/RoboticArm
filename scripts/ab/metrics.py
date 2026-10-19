#!/usr/bin/env python3
"""Frozen metric computation for the A0/A'/B control runs.

Primary: XY offset at t = 38 SIM seconds from dynamic spawn (never wall
clock). Secondary: max offset, per-second speed, stop time, final
attitude, contact-partner changes. Pure offline: consumes the per-run
capture (dyn pose stream parsed by analyze_release_window) and emits
run_metrics.json. Pre-registered; changing any threshold or definition
after the nine runs start invalidates the set.
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "g3")))
from analyze_release_window import parse_pose_stream, parse_contacts  # noqa

T_PRIMARY_S = 38.0
STOP_V_MPS = 0.0005          # "stopped" = below 0.5 mm/s for >= 1 s
CONTACT_PARTNER_MIN_COUNT = 5
#: Validity gate (frozen): the capture must span >= 44 of the 45 sim
#: seconds from spawn, be strictly time-monotone, and leave no sample
#: gap > MAX_GAP_S anywhere up to the primary; the 38 s primary is
#: LINEARLY INTERPOLATED between the bracketing samples (an early last
#: frame cannot stand in for 38 s).
MIN_SPAN_S = 44.0
MAX_GAP_S = 0.5
#: [splice guard] A forward stamp gap larger than this is not a
#: delivery artifact — it is a suspected cross-instance/epoch splice.
MAX_SPLICE_FORWARD_GAP_S = 5.0


def normalize_delivery_reorder(samples):
    """[capture-layer fix, uniform for all arms] The gz `topic -e`
    pipe delivers a small number of complete, correctly-stamped
    messages late (run-01: 3 backward jumps < 1 s in 3,830 samples,
    reproduced on tmpfs — transport artifact, not simulation time
    travel). Sorting by stamp is lossless for complete messages; the
    guard bounds it to the known artifact signature so any REAL
    non-monotone (sim anomaly) still trips the validity gate.

    [splice guard 2026-10-19] Stamp sorting alone cannot rule out a
    stream stitched from TWO sim instances/epochs: two later-instance
    messages with earlier stamps interleave "cleanly" under sort. So
    beyond the backward-jump bounds, a FORWARD gap larger than any
    delivery artifact (MAX_SPLICE_FORWARD_GAP_S) marks the stream as
    a suspected splice and leaves it UNSORTED — the validity gate
    then rejects it instead of silently merging the instances."""
    ts = [s[0] for s in samples]
    bad = [i for i in range(1, len(ts)) if ts[i] <= ts[i - 1]]
    if not bad:
        fgap = [ts[i] - ts[i - 1] for i in range(1, len(ts))]
        if fgap and max(fgap) > MAX_SPLICE_FORWARD_GAP_S * 1e9:
            return samples, {"violations": 0, "sorted": False,
                            "guard": "suspected cross-instance splice "
                                     "(forward gap)"}
        return samples, {"violations": 0, "sorted": False}
    jumps = [ts[i - 1] - ts[i] for i in bad]
    frac = len(bad) / max(len(ts) - 1, 1)
    if max(jumps) > 2_000_000_000 or frac > 0.01:
        return samples, {"violations": len(bad), "sorted": False,
                        "guard": "exceeds artifact signature"}
    return sorted(samples, key=lambda s: s[0]), {
        "violations": len(bad), "sorted": True,
        "max_backward_ms": round(max(jumps) / 1e6, 1),
        "fraction": round(frac, 5)}


def compute_metrics(samples, contacts, model, t_spawn_s):
    """samples: [(t_ns, x, y, z, quat)] for `model`; contacts: parsed."""
    if not samples:
        return {"valid": False, "reason": "no_dynamic_samples"}
    samples, reorder = normalize_delivery_reorder(samples)
    # Validity gate: span, monotonicity, max gap up to the primary.
    ts = [s[0] for s in samples]
    if any(b <= a for a, b in zip(ts, ts[1:])):
        return {"valid": False, "reason": "time_not_monotone",
                "delivery_reorder": reorder}
    t0 = ts[0] / 1e9
    span = (ts[-1] - ts[0]) / 1e9
    if span < MIN_SPAN_S:
        return {"valid": False, "reason": "span_short",
                "span_s": round(span, 3)}
    primary_ts = [t for t in ts if (t - ts[0]) / 1e9 <= T_PRIMARY_S]
    gaps = [(b - a) / 1e9 for a, b in zip(primary_ts, primary_ts[1:])]
    if gaps and max(gaps) > MAX_GAP_S:
        return {"valid": False, "reason": "sample_gap",
                "max_gap_s": round(max(gaps), 3)}
    x0, y0 = samples[0][1], samples[0][2]
    rows = []
    prev = None
    for s in samples:
        t = s[0] / 1e9 - t0
        row = {"t_rel_s": round(t, 4),
               "xy_offset_m": round(math.hypot(s[1] - x0, s[2] - y0), 6)}
        if prev is not None:
            dt = (s[0] - prev[0]) / 1e9
            if dt > 0:
                row["v_xy_mps"] = round(
                    math.hypot(s[1] - prev[1], s[2] - prev[2]) / dt, 6)
        rows.append(row)
        prev = s

    # Primary at 38 sim s: linear interpolation between the samples
    # bracketing T_PRIMARY (fixed rule; early-last-frame cannot stand in).
    primary = None
    for i in range(len(samples) - 1):
        ta = (samples[i][0] - samples[0][0]) / 1e9
        tb = (samples[i + 1][0] - samples[0][0]) / 1e9
        if ta <= T_PRIMARY_S <= tb and tb > ta:
            a = (T_PRIMARY_S - ta) / (tb - ta)
            offa = math.hypot(samples[i][1] - x0, samples[i][2] - y0)
            offb = math.hypot(samples[i + 1][1] - x0,
                              samples[i + 1][2] - y0)
            primary = (1 - a) * offa + a * offb
            break
    if primary is None and rows and rows[-1]["t_rel_s"] >= T_PRIMARY_S:
        primary = rows[-1]["xy_offset_m"]

    stop_time = None
    stop_method = None
    stop_note = None
    # [stop-metric fix 2026-10-19] The box spawns at ZERO velocity: a
    # lateral-velocity-only rule matches the spawn/fall phase, not a
    # landing. Anchor the search at a reliable LANDING EVENT — here the
    # first time height becomes stable (|dz| below Z_STABLE_MPS for
    # >=1 s after having fallen) — then require the lateral speed to
    # stay under threshold from there to the END of the capture (no
    # re-exceeding). Without contact data this is a HEIGHT-STABILITY
    # ESTIMATE; the method is labeled in every report.
    Z_STABLE_MPS = 0.002
    landing_t = None
    z_stable_since = None
    for i in range(1, len(rows)):
        if "v_xy_mps" not in rows[i]:
            continue
        s, s_prev = samples[i], samples[i - 1]
        dt = (s[0] - s_prev[0]) / 1e9
        if dt <= 0:
            continue
        vz = abs((s[3] - s_prev[3]) / dt)
        if vz < Z_STABLE_MPS:
            z_stable_since = (z_stable_since or rows[i]["t_rel_s"])
            if rows[i]["t_rel_s"] - z_stable_since >= 1.0:
                landing_t = z_stable_since
                break
        else:
            z_stable_since = None
    if landing_t is not None:
        # Stop = the FIRST instant of the final quiescent suffix: the
        # sample after the LAST post-landing excursion at/above the
        # threshold, so no later sample ever re-exceeds it (a brief
        # post-landing slide then rest is a valid stop; motion that
        # lasts to the capture end is not).
        post = [(i, r) for i, r in enumerate(rows)
                if r["t_rel_s"] >= landing_t and "v_xy_mps" in r]
        if post:
            above = [i for i, r in post
                     if r["v_xy_mps"] >= STOP_V_MPS]
            if above:
                last_i = above[-1]
                tail = [r for i, r in post if i > last_i]
                t_end = rows[-1]["t_rel_s"]
                if tail and t_end - tail[0]["t_rel_s"] >= 1.0:
                    stop_time = tail[0]["t_rel_s"]
                    stop_method = "height_stability_estimate"
                else:
                    stop_note = ("lateral motion still above threshold "
                                 "at capture end after landing")
            else:
                stop_time = landing_t
                stop_method = "height_stability_estimate"

    partners = {}
    for t_ns, bodies in contacts:
        for name in bodies:
            if model in name:
                continue
            key = name.split("::")[0]
            partners[key] = partners.get(key, 0) + 1
    partner_changes = sorted(
        k for k, v in partners.items() if v >= CONTACT_PARTNER_MIN_COUNT)
    contact_contract_evaluated = bool(contacts)

    last = samples[-1]
    return {
        "valid": True,
        "model": model,
        "t_span_s": [rows[0]["t_rel_s"], rows[-1]["t_rel_s"]],
        "primary_xy_offset_at_38sim_m": primary,
        "delivery_reorder": reorder,
        "stop_time_s": stop_time,
        "stop_method": stop_method,
        "stop_note": stop_note,
        "landing_t_s": round(landing_t, 3) if landing_t is not None else None,
        "contact_contract_evaluated": contact_contract_evaluated,
        "verdict_scope": (
            "displacement_metric_evaluable"
            + ("/contact_contract_evaluated"
               if contact_contract_evaluated
               else "/contact_contract_not_evaluated")),
        "final_xy_displacement_from_spawn_m": [round(last[1] - x0, 5),
                                               round(last[2] - y0, 5)],
        "max_xy_offset_m": max(r["xy_offset_m"] for r in rows),
        "final_attitude_quat": list(last[4]) if last[4] else None,
        "contact_partners": partner_changes,
        "series": rows,
    }


def parse_box_contacts_jsonl(path):
    """[contact-bridge fix] record_box_stream.py JSONL -> the same
    [(t_ns, [names...])] structure parse_contacts returns, so the
    corrected metrics consume the NEW recorder's output. The banner
    line ({"recorder": "start"}) carries no contacts and is skipped."""
    if not os.path.isfile(path):
        return []
    events = []
    for raw in open(path, encoding="utf-8", errors="replace"):
        try:
            d = json.loads(raw)
        except ValueError:
            continue
        if "t_sim_ns" not in d:
            continue
        events.append((int(d["t_sim_ns"]), list(d.get("partners", []))))
    return events


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--model", default="pickup_box_0001_inch20")
    parser.add_argument("--t-spawn-sim", type=float, required=True)
    parser.add_argument("--out", default=None,
                        help="output filename inside run-dir "
                             "(default run_metrics.json; use "
                             "run_metrics_v2.json to keep the original "
                             "derivation)")
    args = parser.parse_args()
    dyn = parse_pose_stream(
        os.path.join(args.run_dir, "dyn_pose_stream.txt"), args.model)
    contacts = parse_box_contacts_jsonl(
        os.path.join(args.run_dir, "box_contacts.jsonl"))
    if not contacts:
        contacts = parse_contacts(
            os.path.join(args.run_dir, "contact_stream.txt"))
    metrics = compute_metrics(dyn, contacts, args.model,
                              args.t_spawn_sim)
    out = os.path.join(args.run_dir, args.out or "run_metrics.json")
    with open(out, "w") as handle:
        json.dump(metrics, handle, indent=2)
    summary = {k: v for k, v in metrics.items() if k != "series"}
    print(json.dumps(summary, indent=1))
    raise SystemExit(0 if metrics.get("valid") else 1)


if __name__ == "__main__":
    main()
