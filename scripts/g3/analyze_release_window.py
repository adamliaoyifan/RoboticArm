#!/usr/bin/env python3
"""Offline analysis of a release-window observation run.

Answers, from the synchronized sim-clock capture:
  1. the static body's last pose and the dynamic body's spawn pose;
  2. per-step box kinematics from the first dynamic physics step
     (lateral offset, per-step velocity, attitude);
  3. the physics step where lateral displacement starts (first step
     whose lateral delta exceeds `--onset-m`), and which driver events
     (vacuum-off, static delete, dynamic spawn, retreat start) or box
     contacts co-occur within +-`--window-s`.

Usage: analyze_release_window.py <run-dir> [--model pickup_box_0001_inch20]
        [--onset-m 0.002] [--window-s 0.25]
"""
import argparse
import glob
import json
import math
import os
import re


def parse_pose_stream(path, model):
    """Parse gz protobuf-text Pose_V messages -> [(t_ns, x, y, z, qx..qw)].

    The text format repeats blocks:
      time { sec: N  nanosec: N }
      name: "model"  pose { position { x: .. } orientation { x: .. } }
    Multiple poses per message; we keep only `model`'s entries IN ORDER,
    tagged with the message time.
    """
    if not os.path.isfile(path):
        return []
    samples = []
    t_ns = None
    current_name = None
    pos = None
    quat = None

    def flush():
        nonlocal current_name, pos, quat
        if current_name == model and pos is not None:
            samples.append((t_ns, pos[0], pos[1], pos[2],
                            tuple(quat) if quat else None))
        current_name, pos, quat = None, None, None

    number = r"[-+0-9.eE]+"
    for line in open(path, encoding="utf-8", errors="replace"):
        stripped = line.strip()
        m = re.match(r"^time \{\s*sec:\s*(" + number + r")", stripped)
        if m:
            flush()
            sec = float(m.group(1))
            nan = re.search(r"nanosec:\s*(" + number + r")", stripped)
            t_ns = int(sec * 1_000_000_000 + float(nan.group(1)
                                                  if nan else 0.0))
            continue
        m = re.match(r'^name:\s*"([^"]+)"', stripped)
        if m:
            flush()
            current_name = m.group(1)
            continue
        if current_name == model:
            if stripped.startswith("pose {"):
                pos, quat = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]
            elif stripped.startswith("position {"):
                for key, idx in (("x:", 0), ("y:", 1), ("z:", 2)):
                    mm = re.search(key + r"\s*(" + number + r")", stripped)
                    if mm:
                        pos[idx] = float(mm.group(1))
            elif stripped.startswith("orientation {"):
                for key, idx in (("x:", 0), ("y:", 1), ("z:", 2), ("w:", 3)):
                    mm = re.search(key + r"\s*(" + number + r")", stripped)
                    if mm:
                        quat[idx] = float(mm.group(1))
    flush()
    return [s for s in samples if s[0] is not None]


def parse_contacts(path):
    """gz Contacts text -> [(t_ns, [other_body,...])]."""
    if not os.path.isfile(path):
        return []
    events = []
    t_ns = None
    bodies = []
    for line in open(path, encoding="utf-8", errors="replace"):
        stripped = line.strip()
        m = re.match(r"^time \{\s*sec:\s*([-+0-9.eE]+).*nanosec:\s*"
                     r"([-+0-9.eE]+)", stripped)
        if m:
            if bodies:
                events.append((t_ns, list(bodies)))
            t_ns = int(float(m.group(1)) * 1_000_000_000
                       + float(m.group(2)))
            bodies = []
            continue
        for m in re.finditer(r'body_1_name:\s*"([^"]+)"|'
                             r'body_2_name:\s*"([^"]+)"', stripped):
            name = m.group(1) or m.group(2)
            if name:
                bodies.append(name)
    if bodies:
        events.append((t_ns, list(bodies)))
    return events


def trial_events(run_dir):
    """Driver-side sim-clock event stamps from the trial dump."""
    events = []
    for trial_path in sorted(glob.glob(
            os.path.join(run_dir, "dumps", "box_*", "trial.json"))):
        try:
            trial = json.load(open(trial_path))
        except (OSError, ValueError):
            continue
        extras = trial.get("extras", {})
        verify = extras.get("verify") or {}
        timing = extras.get("release_dynamic_timing") or {}
        def add(t, name, detail=None):
            if t is not None:
                events.append({"t_ns": int(float(t) * 1e9), "event": name,
                               "detail": detail})
        add(timing.get("t_delete_req_sim"), "static_delete_req")
        add(timing.get("t_delete_done_sim"), "static_delete_done")
        add(timing.get("t_spawn_done_sim"), "dynamic_spawn_done")
        add(verify.get("detach_request_stamp"), "vacuum_off_req")
        add(verify.get("detach_response_stamp"), "vacuum_off_done")
        retreat = extras.get("retreat_start") or {}
        if isinstance(retreat, dict) and retreat.get("t_ros"):
            add(float(retreat["t_ros"]), "retreat_evidence")
        for key in ("box_pose_at_release", "box_pose_after_release",
                    "box_pose_after_drift_wait"):
            if verify.get(key) or extras.get(key):
                pose = verify.get(key) or extras.get(key)
                add(None, key, [round(float(v), 5) for v in pose[:3]])
    return sorted(events, key=lambda e: e["t_ns"])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir")
    parser.add_argument("--model", default="pickup_box_0001_inch20")
    parser.add_argument("--onset-m", type=float, default=0.002)
    parser.add_argument("--window-s", type=float, default=0.25)
    args = parser.parse_args()

    dyn = parse_pose_stream(
        os.path.join(args.run_dir, "dyn_pose_stream.txt"), args.model)
    static = parse_pose_stream(
        os.path.join(args.run_dir, "pose_stream.txt"), args.model)
    contacts = parse_contacts(
        os.path.join(args.run_dir, "contact_stream.txt"))
    events = trial_events(args.run_dir)

    report = {
        "model": args.model,
        "dynamic_samples": len(dyn),
        "static_samples": len(static),
        "contact_events": len(contacts),
        "events": events,
    }

    if static:
        t, x, y, z, _q = static[-1]
        report["static_last_pose"] = {
            "t_sim_s": t / 1e9, "xyz": [x, y, z]}
    if dyn:
        t0, x0, y0, z0, q0 = dyn[0]
        report["dynamic_spawn_pose"] = {
            "t_sim_s": t0 / 1e9, "xyz": [x0, y0, z0],
            "quat_xyzw": list(q0) if q0 else None}
        steps = []
        onset_index = None
        for i in range(1, len(dyn)):
            (ta, xa, ya, za, _qa), (tb, xb, yb, zb, _qb) = \
                dyn[i - 1], dyn[i]
            dt = (tb - ta) / 1e9
            if dt <= 0:
                continue
            lateral = math.hypot(xb - xa, yb - ya)
            report.setdefault("step_dt_stats", {})
            steps.append({
                "i": i, "t_sim_s": tb / 1e9, "dt_s": round(dt, 5),
                "lateral_step_m": round(lateral, 6),
                "lateral_vel_mps": round(lateral / dt, 4),
                "dz_m": round(zb - za, 6),
                "z_vel_mps": round((zb - za) / dt, 4),
                "xy": [round(xb, 5), round(yb, 5)],
            })
            if onset_index is None and lateral >= args.onset_m \
                    and (zb - za) < 0.0:
                onset_index = i
        report["first_steps"] = steps[:6]
        report["onset"] = None
        if onset_index is not None:
            onset_step = steps[onset_index - 1]
            t_onset = onset_step["t_sim_s"]
            win = args.window_s
            near_events = [
                {**e, "dt_s": round(e["t_ns"] / 1e9 - t_onset, 4)}
                for e in events
                if abs(e["t_ns"] / 1e9 - t_onset) <= win]
            near_contacts = [
                {"t_sim_s": t / 1e9,
                 "bodies": sorted(set(b for b in bodies
                                      if args.model not in b)),
                 "dt_s": round(t / 1e9 - t_onset, 4)}
                for t, bodies in contacts
                if abs(t / 1e9 - t_onset) <= win]
            report["onset"] = {
                "step_index": onset_index,
                "t_sim_s": t_onset,
                "lateral_step_m": onset_step["lateral_step_m"],
                "xy": onset_step["xy"],
                "events_within_window": near_events,
                "contacts_within_window": near_contacts,
            }
        if steps:
            last = steps[-1]
            first = steps[0]
            total_lateral = math.hypot(
                last["xy"][0] - dyn[0][1], last["xy"][1] - dyn[0][2])
            report["kinematics_summary"] = {
                "t_first_sim_s": first["t_sim_s"],
                "t_last_sim_s": last["t_sim_s"],
                "lateral_from_spawn_m": round(total_lateral, 5),
                "direction_xy": [
                    round(last["xy"][0] - dyn[0][1], 5),
                    round(last["xy"][1] - dyn[0][2], 5)],
                "z_drop_m": round(
                    dyn[-1][3] - dyn[0][3], 5),
            }

    out = os.path.join(args.run_dir, "release_window_analysis.json")
    with open(out, "w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, sort_keys=True)
    print(json.dumps({k: report[k] for k in (
        "dynamic_samples", "static_samples", "contact_events",
        "static_last_pose", "dynamic_spawn_pose", "onset",
        "kinematics_summary") if k in report}, indent=2))
    print("written: %s" % out)


if __name__ == "__main__":
    main()
