#!/usr/bin/env python3
"""Did static-box contact coincide with joint2 under-tracking?

Reads one capture run (bag, follow_trace.jsonl, contact_capture/, the dumped
SDF). No simulation. A contact class counts as coincident when at least half
of the samples with |joint2 reference-actual| > 0.03 rad have a contact of
that class within 0.05 s. Absence of a class is a finding only when
``absence_is_evidence`` says the capture can support it.

  contact_tracking_analysis.py --run DIR [--out-dir DIR]
"""
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import contact_capture as cap  # noqa: E402
import handoff_reference_analysis as base  # noqa: E402

UNDERTRACK_RAD = 0.03
COINCIDENT_FRACTION = 0.5
NEAR_SEC = 0.05


def rotate_by_inverse(quat_xyzw, vector):
    x, y, z, w = [float(v) for v in quat_xyzw]
    q = np.array([-x, -y, -z])
    v = np.asarray(vector, dtype=float)
    return v + 2.0 * w * np.cross(q, v) + 2.0 * np.cross(q, np.cross(q, v))


def inside_box(panel_xyz, box_xyz, box_quat, half_extents):
    local = rotate_by_inverse(box_quat, np.asarray(panel_xyz) - np.asarray(box_xyz))
    return bool(np.all(np.abs(local) <= np.asarray(half_extents) + 1e-9))


def nearest_fraction(times, events, tol=NEAR_SEC):
    """Fraction of ``times`` that have an event within ``tol``."""
    if not len(times):
        return None
    events = np.asarray(sorted(events), dtype=float)
    if not len(events):
        return 0.0
    idx = np.searchsorted(events, times)
    hits = 0
    for i, t in enumerate(times):
        cands = []
        if idx[i] < len(events):
            cands.append(events[idx[i]])
        if idx[i] > 0:
            cands.append(events[idx[i] - 1])
        if min(abs(c - t) for c in cands) <= tol:
            hits += 1
    return hits / float(len(times))


def half_extents_from_sdf(text):
    import re
    match = re.search(
        r"([\d.]+)\s*x\s*([\d.]+)\s*x\s*([\d.]+)\s*m", text or "")
    if not match:
        return None
    return tuple(float(v) / 2.0 for v in match.groups())


def summarize_windows(windows, joint2_t, joint2_err, contacts, follow):
    """``windows`` is [(name, t0, t1), ...]. Returns one row per window."""
    rows = []
    for name, t0, t1 in windows:
        m = (joint2_t >= t0) & (joint2_t <= t1)
        err = joint2_err[m]
        under = joint2_t[m][err > UNDERTRACK_RAD] if m.any() else np.array([])
        classes = {}
        for klass in ("suction_interface", "robot_body", "container", "other"):
            ev = [c["stamp"] for c in contacts
                  if c["klass"] == klass and t0 <= c["stamp"] <= t1]
            classes[klass] = {
                "messages": len(ev),
                "max_depth_m": max(
                    (c["depth_m"] for c in contacts
                     if c["klass"] == klass and t0 <= c["stamp"] <= t1),
                    default=None),
                "max_force_n": max(
                    (c["force_n"] for c in contacts
                     if c["klass"] == klass and t0 <= c["stamp"] <= t1),
                    default=None),
                "fraction_of_undertrack_samples": nearest_fraction(under, ev),
            }
        gaps = []
        ticks = [f for f in follow if t0 <= f["sim"] <= t1 and f.get("ok")]
        for a, b in zip(ticks, ticks[1:]):
            gaps.append(b["sim"] - a["sim"])
        rows.append({
            "name": name, "t0": t0, "t1": t1,
            "joint2_max_abs_error_rad": float(np.abs(err).max()) if m.any() else None,
            "joint2_undertrack_samples": int(len(under)),
            "contacts": classes,
            "set_pose_interval_median_sec": float(np.median(gaps)) if gaps else None,
            "follow_ticks": len(ticks),
        })
    return rows


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True)
    parser.add_argument("--out-dir", default="")
    args = parser.parse_args(argv)
    run = Path(args.run)
    out = Path(args.out_dir) if args.out_dir else run / "contact_tracking"
    out.mkdir(parents=True, exist_ok=True)
    bag = base.read_bag(run)
    follow = []
    trace = run / "follow_trace.jsonl"
    if trace.is_file():
        for line in trace.read_text().splitlines():
            if line.strip():
                row = json.loads(line)
                row["sim"] = base.wall_to_sim(bag, row["wall"])
                follow.append(row)
    contacts = []
    cpath = run / "contact_capture" / "contacts.jsonl"
    if cpath.is_file():
        contacts = [json.loads(l) for l in cpath.read_text().splitlines() if l.strip()]
        for row in contacts:
            row["klass"] = cap.classify_pair(row.get("a", ""), row.get("b", ""))
    poses = []
    ppath = run / "contact_capture" / "poses.jsonl"
    if ppath.is_file():
        poses = [json.loads(l) for l in ppath.read_text().splitlines() if l.strip()]
    sdf = (run / "collision_models" / "pickup_box.sdf").read_text() if (
        run / "collision_models" / "pickup_box.sdf").is_file() else ""
    half = half_extents_from_sdf(sdf)
    inside = []
    if half and follow:
        for row in follow:
            if row.get("ok"):
                inside.append(inside_box(
                    row["panel_xyz"], row["box_xyz"], row["box_quat"], half))
    panel_inside = bool(inside) and (sum(inside) / len(inside) > 0.5)
    legs = base.read_submitted_legs(run)
    events = base.read_controller_log(run)
    accepts = [w for w, k in events if k == "accept"]
    windows = []
    for index, leg in enumerate(legs):
        inside_acc = [a for a in accepts
                      if leg["submit_wall"] - 0.5 <= a <= leg["result_wall"] + 0.5]
        if len(inside_acc) != 1:
            continue
        t0 = base.wall_to_sim(bag, inside_acc[0])
        later = [a for a in accepts if a > inside_acc[0]]
        t1 = base.wall_to_sim(bag, later[0]) if later else leg["result_sim_stamp"]
        windows.append(("%s#%d" % (leg["name"], index), t0, t1))
    err = np.abs(bag["ref"][:, 1] - bag["fb"][:, 1]) if len(bag["ct"]) else np.array([])
    rows = summarize_windows(windows, bag["ct"], err, contacts, follow)
    status = {}
    sp = run / "contact_capture" / "recorder_status.json"
    if sp.is_file():
        status = json.loads(sp.read_text())
    covered = bool(poses) and bool(follow)
    publisher_ok = False
    trials = run / "trials.jsonl"
    if trials.is_file():
        for line in trials.read_text().splitlines():
            if not line.strip():
                continue
            gate = ((json.loads(line).get("extras") or {})
                    .get("contact_capture_gate") or {})
            for check in gate.get("checks") or []:
                if check.get("check") == "contact_topic_publisher":
                    publisher_ok = bool(check.get("ok"))
    result = {
        "run": str(run),
        "box_half_extents_m": half,
        "follow_ticks": len(follow),
        "pose_rows": len(poses),
        "contact_rows": len(contacts),
        "panel_inside_box_aabb_most_of_the_time": panel_inside,
        "recorder_status": status,
        "absence_is_evidence": cap.absence_is_evidence(
            saw_contact=bool(contacts), publisher_ok=publisher_ok,
            recorder_covered=covered, panel_inside_box=panel_inside),
        "undertrack_threshold_rad": UNDERTRACK_RAD,
        "legs": rows,
        "contact_publisher_ok": publisher_ok,
        "note": "absence_is_evidence uses the pre-motion gate's publisher "
                "check, whether any contact was recorded, whether pose and "
                "follow traces cover the run, and whether the suction frame "
                "sat inside the box bounds.",
    }
    # Follow error against recorded box pose, where both exist.
    if follow and poses:
        # The ``_visual`` entity stays at the origin; only the model
        # pose is where set_pose put the box.
        box_poses = [p for p in poses
                     if p["name"].startswith("pickup_box")
                     and not p["name"].endswith("_visual")
                     and "::" not in p["name"]]
        if box_poses:
            bt = np.array([p["stamp"] for p in box_poses])
            bq = np.array([p["xyz"] for p in box_poses])
            order = np.argsort(bt)
            bt, bq = bt[order], bq[order]
            ft = np.array([r["sim"] for r in follow if r.get("ok")])
            fq = np.array([r["box_xyz"] for r in follow if r.get("ok")])
            if len(bt) > 2 and len(ft):
                actual = np.column_stack([np.interp(ft, bt, bq[:, k]) for k in range(3)])
                delta = np.linalg.norm(actual - fq, axis=1)
                result["commanded_vs_actual_box_m"] = {
                    "samples": int(len(delta)),
                    "median": float(np.median(delta)),
                    "max": float(delta.max()),
                }
    text = json.dumps(result, indent=2, default=float)
    (out / "contact_tracking.json").write_text(text + "\n")
    print(text)
    try:
        import contact_tracking_figure as figure
        figure.render(result, bag, contacts, follow, out / "contact_tracking.png")
    except ImportError as exc:
        print("figure skipped:", exc)
    return 0


if __name__ == "__main__":
    sys.exit(main())
