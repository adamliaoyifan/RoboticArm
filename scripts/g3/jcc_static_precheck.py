#!/usr/bin/env python3
"""Sidecar recorder + offline verdict for the bounded static precheck.

Part 1 (`record`): subscribe /elfin_arm_controller/controller_state,
/joint_states and /clock for N SIM seconds; write JSONL sidecars into the
run directory. No goals are ever sent (the precheck is static by contract).

Part 2 (`verdict`): offline gates over the recorder streams + sidecars:
  G1 the actually-loaded plugin is the instrumented build (gz process maps
     path + sha256 + recorder watermark);
  G2 three correlatable layers (plugin write/read/iteration streams,
     controller_state <-> cmd_received join with MEASURED coverage,
     joint_states <-> plugin_read agreement);
  G3 capture-on does not block control and capture-off has no capture
     burden (cadence/RTF on-vs-off, no capture artifacts in the off run).

Static-only: this precheck does NOT validate dynamic conversion behaviour
(varied commands / clamping / retained-previous) — those stay covered by
the offline fixtures and are explicitly NOT claimed here.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import time
from pathlib import Path

CS_TOPIC = "/elfin_arm_controller/controller_state"


def _stamp_ns(msg) -> int:
    h = getattr(msg, "header", None)
    if h is None:
        return 0
    return int(h.stamp.sec) * 1_000_000_000 + int(h.stamp.nanosec)


def record(run_dir: str, sim_seconds: float, wall_cap: float = 300.0) -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from control_msgs.msg import JointTrajectoryControllerState
    from sensor_msgs.msg import JointState
    from rosgraph_msgs.msg import Clock

    run = Path(run_dir)
    run.mkdir(parents=True, exist_ok=True)
    cs_path = run / "sidecar_controller_state.jsonl"
    js_path = run / "sidecar_joint_states.jsonl"
    clock_path = run / "sidecar_clock.jsonl"
    state = {"last_sim": -1.0, "start_sim": None, "cs": 0, "js": 0, "clock": 0}

    rclpy.init()
    node = Node("jcc_static_precheck_sidecar")
    best = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)

    def on_cs(msg):
        if state["start_sim"] is None:
            return
        row = {
            "stamp_ns": _stamp_ns(msg),
            "wall": time.time(),
            "desired_pos": list(msg.desired.positions),
            "actual_pos": list(msg.actual.positions),
        }
        with open(cs_path, "a") as fh:
            fh.write(json.dumps(row) + "\n")
        state["cs"] += 1

    def on_js(msg):
        if state["start_sim"] is None:
            return
        row = {
            "stamp_ns": _stamp_ns(msg),
            "wall": time.time(),
            "position": list(msg.position),
            "velocity": list(msg.velocity),
            "effort": list(msg.effort) if msg.effort else [],
        }
        with open(js_path, "a") as fh:
            fh.write(json.dumps(row) + "\n")
        state["js"] += 1

    def on_clock(msg):
        sim = msg.clock.sec + msg.clock.nanosec * 1e-9
        if state["start_sim"] is None:
            state["start_sim"] = sim
        state["last_sim"] = sim
        state["clock"] += 1
        # Throttled clock trace (10 Hz) for RTF and cadence bookkeeping.
        last_written = state.get("clock_written_sim")
        if last_written is None or sim - last_written >= 0.1:
            state["clock_written_sim"] = sim
            with open(clock_path, "a") as fh:
                fh.write(json.dumps({"sim": sim, "wall": time.time()}) + "\n")

    node.create_subscription(JointTrajectoryControllerState, CS_TOPIC, on_cs, best)
    node.create_subscription(JointState, "/joint_states", on_js, best)
    node.create_subscription(Clock, "/clock", on_clock, 200)
    wall_start = time.time()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        if state["start_sim"] is not None and (
                state["last_sim"] - state["start_sim"]) >= sim_seconds:
            break
        if time.time() - wall_start > wall_cap:
            break
    summary = {
        "sim_seconds_recorded": (state["last_sim"] - state["start_sim"])
        if state["start_sim"] is not None else 0.0,
        "rows_controller_state": state["cs"],
        "rows_joint_states": state["js"],
        "rows_clock": state["clock"],
    }
    (run / "sidecar_summary.json").write_text(json.dumps(summary, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps(summary))
    return 0


# ------------------------------------------------------------------ verdict --
def _gz_pids() -> list:
    # Fortress launches as "ign gazebo ..."; newer stacks as "gz sim ...".
    try:
        out = subprocess.run(["pgrep", "-f", "gazebo"], capture_output=True,
                             text=True, timeout=10)
        pids = [int(x) for x in out.stdout.split()]
    except Exception:
        return []
    result = []
    for pid in pids:
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as fh:
                cmd = fh.read().decode(errors="replace")
            if "sim_world" not in cmd:  # exclude our own tooling
                result.append(pid)
        except OSError:
            continue
    return result


def loaded_plugin_libraries(pid: int) -> dict:
    """Map which gz_ros2_control libs the gz server process has loaded."""
    found = {}
    try:
        with open(f"/proc/{pid}/maps") as fh:
            for line in fh:
                if "gz_ros2_control" not in line and "gz_hardware_plugins" not in line:
                    continue
                path = line.rstrip().split(" ", 5)[-1]
                if path.startswith("/"):
                    found[path] = None
    except OSError:
        pass
    out = {}
    for path in found:
        try:
            digest = hashlib.sha256(Path(path).read_bytes()).hexdigest()
        except OSError:
            digest = "unreadable"
        out[path] = digest
    return out


def verdict(on_dir: str, off_dir: str | None, install_prefix: str,
            sidecar_dir: str | None = None, launch_log: str | None = None) -> int:
    import numpy as np
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] /
                      "src/joint_command_capture/python"))
    from joint_command_capture.records_schema import read_stream, load_run

    sidecar_dir = sidecar_dir or on_dir
    report: dict = {"on_dir": on_dir, "sidecar_dir": sidecar_dir,
                    "off_dir": off_dir, "gates": {}}

    # G1: actually-loaded library identity + recorder watermark. The
    # strongest runtime identity: the "[jcc] capture mode=..." log line can
    # only be emitted by the instrumented plugin (the deb binary has no such
    # call); /proc maps are best-effort corroboration.
    maps = {}
    for pid in _gz_pids():
        maps.update(loaded_plugin_libraries(pid))
    snap = Path(sidecar_dir) / "gz_loaded_libraries.json"
    if not maps and snap.exists():
        maps = json.loads(snap.read_text())
    report["loaded_libraries"] = maps
    instrumented = any(str(install_prefix) in p and
                       ("libgz_hardware_plugins" in p or
                        "ros2_control-system" in p) for p in maps)
    jcc_log_line = ""
    if launch_log and Path(launch_log).exists():
        for line in Path(launch_log).read_text(errors="replace").splitlines():
            if "[jcc] capture mode=" in line:
                jcc_log_line = line.strip()
                break
    watermark = None
    health = Path(on_dir) / "health.json"
    if health.exists():
        watermark = json.loads(health.read_text()).get("watermark")
    report["recorder_watermark"] = watermark
    g1 = (instrumented or "[jcc] capture mode='window' active=1" in jcc_log_line) \
        and bool(watermark)
    report["gates"]["G1_loaded_instrumented_so"] = {
        "pass": bool(g1),
        "instrumented_path_found": instrumented,
        "instrumented_log_line": jcc_log_line,
        "watermark": watermark,
    }

    # G2: three correlatable layers.
    g2 = {"pass": False}
    try:
        run = load_run(on_dir)
        if "plugin_write" in run and "plugin_read" in run:
            w = run["plugin_write"].records
            r = run["plugin_read"].records
            wdt = np.diff(w["sim_time_ns"])
            rdt = np.diff(r["sim_time_ns"])
            join = int(np.sum(np.isin(r["sim_time_ns"], w["sim_time_ns"])))
            g2.update({
                "write_rows": int(len(w)),
                "read_rows": int(len(r)),
                "iteration_rows": int(len(run["iteration"].records))
                if "iteration" in run else 0,
                "write_sim_dt_ns_median": float(np.median(wdt)) if len(wdt) else None,
                "read_sim_dt_ns_median": float(np.median(rdt)) if len(rdt) else None,
                "write_read_exact_join": join,
                "write_read_join_fraction": join / max(1, len(r)),
            })
            # controller_state (L2) <-> cmd_received with measured coverage.
            cs_path = Path(sidecar_dir) / "sidecar_controller_state.jsonl"
            if cs_path.exists():
                times, des = [], []
                for line in cs_path.read_text().splitlines():
                    row = json.loads(line)
                    if row["stamp_ns"] and row.get("desired_pos"):
                        times.append(int(row["stamp_ns"]))
                        des.append(row["desired_pos"][0])
                if times:
                    t2 = np.array(times, dtype=np.int64)
                    d2 = np.array(des)
                    o = np.argsort(t2)
                    t2, d2 = t2[o], d2[o]
                    base = float(np.median(wdt)) if len(wdt) else 1e6
                    wt_all = w["sim_time_ns"]
                    best_cov, best_off, best_diff, best_mode = 0.0, None, None, None
                    for k in range(0, 13):
                        off = int(round(k * base))
                        idx = np.searchsorted(wt_all, t2 + off)
                        ok = idx < len(wt_all)
                        if not np.any(ok):
                            continue
                        iv = idx[ok]
                        exact = wt_all[iv] == (t2[ok] + off)
                        cov = float(np.sum(exact)) / max(1, len(t2))
                        if cov > best_cov:
                            best_cov, best_off, best_mode = cov, off, "exact"
                            best_diff = np.abs(d2[ok][exact] -
                                               w["cmd_received"][iv[exact], 0])
                    if best_cov < 0.9:
                        # Tolerance fallback (plan: time-match must carry an
                        # explicit sync-error column). Nearest write row
                        # within +-2 ms of each published stamp.
                        tol = 2_000_000
                        idx = np.clip(np.searchsorted(wt_all, t2), 0, len(wt_all) - 1)
                        cand = np.stack([np.maximum(idx - 1, 0), idx])
                        pick = cand[np.abs(wt_all[cand] - t2).argmin(axis=0),
                                    np.arange(len(t2))]
                        near = np.abs(wt_all[pick] - t2) <= tol
                        cov = float(np.sum(near)) / max(1, len(t2))
                        if cov > best_cov:
                            best_cov, best_mode = cov, "tolerance_2ms"
                            best_diff = np.abs(d2[near] -
                                               w["cmd_received"][pick[near], 0])
                            g2["l2_sync_error_ns_p50"] = float(np.median(
                                np.abs(wt_all[pick][near] - t2[near])))
                    g2["l2_join_mode"] = best_mode
                    g2["l2_join_offset_ns"] = best_off
                    g2["l2_measured_coverage"] = best_cov
                    g2["l2_max_abs_diff_rad"] = (
                        float(np.max(best_diff)) if best_diff is not None
                        and len(best_diff) else None)
            # joint_states <-> plugin_read agreement (same physical source).
            js_path = Path(sidecar_dir) / "sidecar_joint_states.jsonl"
            if js_path.exists():
                jt, jp = [], []
                for line in js_path.read_text().splitlines():
                    row = json.loads(line)
                    if row["stamp_ns"] and row.get("position"):
                        jt.append(int(row["stamp_ns"]))
                        jp.append(row["position"][0])
                if jt:
                    jt_a = np.array(jt, dtype=np.int64)
                    jp_a = np.array(jp)
                    o = np.argsort(jt_a)
                    jt_a, jp_a = jt_a[o], jp_a[o]
                    rt_all = r["sim_time_ns"]
                    idx = np.clip(np.searchsorted(rt_all, jt_a), 0, len(rt_all) - 1)
                    cand = np.stack([np.maximum(idx - 1, 0), idx])
                    pick = cand[np.abs(rt_all[cand] - jt_a).argmin(axis=0),
                                np.arange(len(jt_a))]
                    near = np.abs(rt_all[pick] - jt_a) <= 2_000_000
                    if np.any(near):
                        diff = np.abs(jp_a[near] - r["position"][pick[near], 0])
                        g2["js_vs_plugin_read_matches"] = int(np.sum(near))
                        g2["js_vs_plugin_read_max_diff_rad"] = float(np.max(diff))
                        g2["js_vs_plugin_read_sync_ns_p50"] = float(np.median(
                            np.abs(rt_all[pick][near] - jt_a[near])))
            l2_max = g2.get("l2_max_abs_diff_rad")
            js_max = g2.get("js_vs_plugin_read_max_diff_rad")
            g2["pass"] = bool(
                g2.get("write_rows", 0) > 1000 and
                g2.get("read_rows", 0) > 100 and
                g2.get("write_read_join_fraction", 0) > 0.99 and
                g2.get("l2_measured_coverage", 0) > 0.9 and
                l2_max is not None and l2_max < 0.01 and
                js_max is not None and js_max < 1e-6)
    except Exception as exc:  # evidence, not silence
        g2["error"] = f"{type(exc).__name__}: {exc}"
    report["gates"]["G2_layers_correlatable"] = g2

    # G3: burden on-vs-off (cadence + RTF), off-run has no capture artifacts.
    g3 = {"pass": False}

    def _cadence(run_dir):
        cs_p = Path(run_dir if run_dir != on_dir else sidecar_dir) / "sidecar_controller_state.jsonl"
        cl_p = Path(run_dir if run_dir != on_dir else sidecar_dir) / "sidecar_clock.jsonl"
        out = {}
        if cs_p.exists():
            rows = [json.loads(x) for x in cs_p.read_text().splitlines()]
            wall = [x["wall"] for x in rows if x.get("wall")]
            if len(wall) > 10:
                rate = (len(wall) - 1) / (wall[-1] - wall[0])
                out["controller_state_wall_rate_hz"] = rate
        if cl_p.exists():
            rows = [json.loads(x) for x in cl_p.read_text().splitlines()]
            if len(rows) > 10:
                sim = rows[-1]["sim"] - rows[0]["sim"]
                wall = rows[-1]["wall"] - rows[0]["wall"]
                if wall > 0:
                    out["rtf"] = sim / wall
        return out

    def _clock_rows(path):
        rows = []
        p = Path(path) / "sidecar_clock.jsonl"
        if p.exists():
            for line in p.read_text().splitlines():
                row = json.loads(line)
                rows.append(row)
        return rows

    on_cad = _cadence(sidecar_dir)
    g3.update({f"on_{k}": v for k, v in on_cad.items()})
    if off_dir:
        off_cad = _cadence(off_dir)
        g3.update({f"off_{k}": v for k, v in off_cad.items()})
        artifacts = [x.name for x in Path(off_dir).iterdir()
                     if x.name.endswith(".bin") or x.name in
                     ("health.json", "manifest_recorder.json", "ctl")]
        g3["off_run_capture_artifacts"] = artifacts
    on_rate = g3.get("on_controller_state_wall_rate_hz")
    off_rate = g3.get("off_controller_state_wall_rate_hz")
    rate_ok = (on_rate and off_rate and
               abs(on_rate - off_rate) / max(1e-6, off_rate) < 0.10)
    no_burden_ok = off_dir is None or not g3.get("off_run_capture_artifacts")
    g3["pass"] = bool(rate_ok and no_burden_ok)
    report["gates"]["G3_capture_burden"] = g3

    report["overall_pass"] = bool(g1 and g2.get("pass") and g3.get("pass"))
    out_path = Path(on_dir).parent / "PRECHECK_REPORT.json"
    out_path.write_text(json.dumps(report, indent=2, default=str))
    print(json.dumps(report, indent=2, default=str))
    return 0 if report["overall_pass"] else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="cmd", required=True)
    p_rec = sub.add_parser("record")
    p_rec.add_argument("run_dir")
    p_rec.add_argument("--sim-seconds", type=float, default=60.0)
    p_rec.add_argument("--wall-cap", type=float, default=300.0)
    p_map = sub.add_parser("maps")
    p_map.add_argument("out_json")
    p_ver = sub.add_parser("verdict")
    p_ver.add_argument("on_dir")
    p_ver.add_argument("--off-dir", default=None)
    p_ver.add_argument("--install-prefix",
                       default="/tmp/elfin-master-pack-baseline-g3/install")
    p_ver.add_argument("--sidecar-dir", default=None,
                       help="Directory holding sidecar_*.jsonl (default: on_dir)")
    p_ver.add_argument("--launch-log", default=None,
                       help="ON-phase launch log for the [jcc] identity line")
    args = parser.parse_args()
    if args.cmd == "record":
        return record(args.run_dir, args.sim_seconds, args.wall_cap)
    if args.cmd == "maps":
        libs = {}
        for pid in _gz_pids():
            libs.update(loaded_plugin_libraries(pid))
        Path(args.out_json).write_text(json.dumps(libs, indent=2))
        print(json.dumps(libs, indent=2))
        return 0
    return verdict(args.on_dir, args.off_dir, args.install_prefix,
                    sidecar_dir=args.sidecar_dir,
                    launch_log=args.launch_log)


if __name__ == "__main__":
    sys.exit(main())
