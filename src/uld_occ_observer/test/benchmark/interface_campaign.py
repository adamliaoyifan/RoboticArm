"""Run the frozen O5–O7 interface collection. Nothing here is a product score."""
from __future__ import annotations

import hashlib
import json
import os
import signal
import shutil
import sys
import subprocess
import time
from pathlib import Path

import interface_events
import sustained_run

WORKTREE = Path("/tmp/elfin-occ-octomap-perf-cursor-g1")
BENCH = WORKTREE / "src/uld_occ_observer/test/benchmark"
PROBE = WORKTREE / "diag/install/uld_occ_observer/lib/uld_occ_observer/interface_probe_cli"
NODE = WORKTREE / "diag/install/uld_occ_observer/lib/uld_occ_observer/observer_node"
SETUP = WORKTREE / "diag/install/setup.bash"
FIXTURES = Path("/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures")
POINTS = {
    0: Path("/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run03/o7/points_b00.json"),
    20: Path("/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run03/o7/points_b20.json"),
}
O6_MANIFEST = Path("/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/o6/groups/r01_b00_cuboid_c30000_o50_x50/warm/manifest.yaml")
PROFILE_SHA = "e05bde858017f2d7b358498ba42388a2893042b0eeb1974ed019d03a69311f60"
PROFILE_CANDIDATES = (
    Path("/tmp/occ-o6-product-install/lib/uld_occ_observer/observer_profile_cli"),
    Path("/tmp/occ-eval-cand-build/observer_profile_cli"),
)
LOCK = Path("/tmp/elfin-occ-backend-perf.lock")
DOMAIN = 23
REQUIRED_SERVER = (
    "callback_enter", "tf_lookup_begin", "tf_lookup_end", "convert_begin", "points_decoded",
    "enqueue_begin", "enqueue_result", "worker_dequeue", "integrate_start", "store_lock_acquired",
    "store_lock_release", "integrate_end", "receipt_publish_end", "query_callback_enter",
    "repository_lock_acquired", "roi_begin", "roi_end", "surface_begin", "surface_end",
    "surface_cache_locked", "response_assemble_end", "health_publish_begin", "trace_footer",
)


def sha256(path: Path):
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    return digest.hexdigest()


def write_points(src: Path, dest: Path):
    if dest.exists() and dest.stat().st_size:
        return
    points = json.loads(src.read_text())
    dest.write_text("".join("%s %s %s\n" % (item[0], item[1], item[2]) for item in points))


def every(count, step_ns, role=None):
    rows = []
    for index in range(count):
        row = {"planned_ns": index * step_ns}
        if role:
            row["role"] = role(index)
        rows.append(row)
    return rows


def scene(boxes):
    return sustained_run._scene(FIXTURES, boxes)


def ros_env():
    return sustained_run._ros_env(str(SETUP), DOMAIN)


def run_session(out, spec, run_id, geometry, config, points, commands, command_log):
    out = Path(out)
    if (out / "result.json").exists():
        raise SystemExit("refusing to overwrite %s" % out)
    spec_path = out.parent / (out.name + ".spec.json")
    spec_path.parent.mkdir(parents=True, exist_ok=True)
    spec_path.write_text(json.dumps(spec))
    command = [sys.executable, str(BENCH / "interface_session.py"),
               "--kind", spec["kind"], "--out", str(out), "--node", str(NODE), "--setup", str(SETUP),
               "--geometry", geometry, "--config", config, "--points", str(points),
               "--spec", str(spec_path), "--run-id", run_id, "--domain-id", str(DOMAIN)]
    commands.append(command)
    write_commands(command_log, commands)
    env = ros_env()
    env["ULD_OCC_INTERFACE_TRACE"] = "1"
    limits = {"rehearse": 120, "arm": 240, "normal": 300, "overload": 480, "recovery": 240}
    proc = subprocess.Popen(command, env=env)
    try:
        return proc.wait(timeout=limits.get(spec["kind"], 300))
    except subprocess.TimeoutExpired:
        proc.send_signal(signal.SIGTERM)
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        return 4


def rehearsal_ok(folder):
    server_path = Path(folder) / "server_events.jsonl"
    client_path = Path(folder) / "client_events.jsonl"
    if not server_path.exists() or server_path.stat().st_size == 0:
        return False, ["missing_server_trace"]
    server = [json.loads(line) for line in server_path.read_text().splitlines() if line]
    client = [json.loads(line) for line in client_path.read_text().splitlines() if line]
    missing = [name for name in REQUIRED_SERVER if not any(event["event"] == name for event in server)]
    cache = {event["status"] for event in server if event["event"] == "surface_cache_locked"}
    if "hit" not in cache or "miss" not in cache:
        missing.append("surface_cache_hit_and_miss")
    footer = server[-1]
    if footer.get("event") != "trace_footer" or footer.get("status") != "0":
        missing.append("footer_or_drops")
    if not interface_events.validate_trace(server)["ok"]:
        missing.append("server_trace_invalid")
    if not interface_events.validate_trace(client)["ok"]:
        missing.append("client_trace_invalid")
    if not any(event["event"] == "client_receipt" for event in client):
        missing.append("client_receipt")
    return not missing, missing


def probe_once(op, arm, dest, geometry, config, points, commands, command_log):
    dest.parent.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["ULD_OCC_INTERFACE_TRACE"] = "1" if arm == "on" else "0"
    env["ULD_OCC_TRACE_PATH"] = str(dest.with_suffix(".jsonl"))
    env["ULD_OCC_TRACE_RUN_ID"] = dest.stem
    command = [str(PROBE), "--op", op, "--geometry", geometry, "--config", config,
               "--points", str(points), "--out", str(dest)]
    commands.append(command + ["trace=%s" % arm])
    write_commands(command_log, commands)
    completed = subprocess.run(command, check=False, env=env, capture_output=True, text=True)
    if completed.returncode != 0:
        dest.with_suffix(".err").write_text(completed.stderr)
        return None
    return json.loads(dest.read_text())


def perturb(out, commands, command_log):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    found = scene(0)
    points = out / "points_b00.txt"
    write_points(POINTS[0], points)
    report = {"not_scored": True, "operations": {}, "content_consistent": True, "stop": False}
    for op in ("integrate", "acquire", "roi", "surface"):
        collected = {"on": [], "off": []}
        first = {"on": [], "off": []}
        second = {"on": [], "off": []}
        hashes = []
        for index, pair in enumerate(interface_events.iter_pairs()):
            for arm in pair:
                dest = out / op / ("%02d_%s.json" % (index, arm))
                row = probe_once(op, arm, dest, found["geometry_file"], found["config_file"], points, commands, command_log)
                if row is None:
                    report["stop"] = True
                    report["operations"][op] = {"status": "probe_failed"}
                    return report
                collected[arm].append(row["sample_ms"])
                hashes.append(row["content_sha256"])
                if op == "surface":
                    first[arm].append(row["surface_first_ms"])
                    second[arm].append(row["surface_second_ms"])
                if arm == "on":
                    trace = [json.loads(line) for line in dest.with_suffix(".jsonl").read_text().splitlines() if line]
                    valid = interface_events.validate_trace(trace)
                    footer = trace[-1] if trace else {}
                    if not valid["ok"] or footer.get("status") != "0":
                        report["stop"] = True
                        report["operations"][op] = {"status": "trace_invalid", "errors": valid["errors"]}
                        return report
        same = len(set(hashes)) == 1
        report["content_consistent"] = report["content_consistent"] and same
        entry = {
            "sample": interface_events.perturbation_ratio(collected["on"], collected["off"]),
            "content_sha256": hashes[0] if same else hashes,
            "content_consistent": same,
        }
        if op == "surface":
            entry["first"] = interface_events.perturbation_ratio(first["on"], first["off"])
            entry["repeat"] = interface_events.perturbation_ratio(second["on"], second["off"])
            entry["pools_mixed"] = False
        report["operations"][op] = entry
        if not same:
            report["stop"] = True
            return report
    (out / "PERTURBATION.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def profile_cli():
    for path in PROFILE_CANDIDATES:
        if path.exists() and sha256(path) == PROFILE_SHA:
            return path
    return None


def run_o6(out, commands, command_log):
    import yaml
    binary = profile_cli()
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    if binary is None:
        (out / "O6_ATTRIBUTION.json").write_text(json.dumps({
            "status": "not_evaluated", "reason": "frozen profile cli sha not found"}) + "\n")
        return 4
    geometry = str(FIXTURES / "geometry_cuboid.yaml")
    config = str(FIXTURES / "config_cuboid_r0p01.yaml")
    original = yaml.safe_load(O6_MANIFEST.read_text())
    base = O6_MANIFEST.parent
    for mode in ("off", "stages"):
        manifest = {"case_id": original["case_id"], "mode": mode, "surface": True,
                    "state_digest": True, "frames": []}
        for frame in original["frames"]:
            copied = dict(frame)
            copied["path"] = str((base / frame["path"]).resolve())
            manifest["frames"].append(copied)
        folder = out / mode
        folder.mkdir(parents=True, exist_ok=True)
        manifest_path = folder / "manifest.yaml"
        manifest_path.write_text(yaml.safe_dump(manifest))
        target = folder / "samples.yaml"
        command = [str(binary), "--geometry", geometry, "--config", config,
                   "--manifest", str(manifest_path), "--out", str(target)]
        commands.append(command)
        write_commands(command_log, commands)
        completed = subprocess.run(command, check=False, capture_output=True, text=True)
        (folder / "stderr.txt").write_text(completed.stderr)
        if completed.returncode != 0:
            return completed.returncode
    return 0


def environment():
    governor = Path("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor")
    return {
        "governor": governor.read_text().strip() if governor.exists() else "unknown",
        "nproc": os.cpu_count(),
        "platform": os.uname().sysname + "-" + os.uname().release + "-" + os.uname().machine,
        "tool_head": subprocess.check_output(["git", "-C", str(WORKTREE), "rev-parse", "HEAD"], text=True).strip(),
        "product_revision": "4c8f1862ec357cb01616f4b04be636227461cf44",
        "probe_sha256": sha256(PROBE) if PROBE.exists() else None,
        "node_sha256": sha256(NODE) if NODE.exists() else None,
        "clock": "CLOCK_MONOTONIC via clock_gettime and time.monotonic_ns; ROS stamps are identity only",
        "not_scored": True,
    }


def write_commands(path, commands):
    lines = ["#!/bin/bash", "set -u", "cd %s" % WORKTREE]
    for command in commands:
        lines.append(" ".join(json.dumps(part) for part in command))
    path.write_text("\n".join(lines) + "\n")
    path.chmod(0o755)


def main(argv):
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    parser.add_argument("--only", default="all")
    args = parser.parse_args(argv)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    commands = []
    if args.only in ("all", "coverage"):
        subprocess.check_call([sys.executable, str(BENCH / "interface_coverage.py"), str(out)])
    if args.only == "coverage":
        return 0
    import fcntl
    lock = open(LOCK, "a")
    waited = time.monotonic()
    fcntl.flock(lock, fcntl.LOCK_EX)
    lock_wait = time.monotonic() - waited
    started = time.monotonic()
    try:
        (out / "MANIFEST.json").write_text(json.dumps({
            **environment(), "lock_wait_s": lock_wait, "domain_id": DOMAIN,
            "budget_s": 120 * 60, "experiment_clock_excludes_lock_wait": True,
        }, indent=2) + "\n")
        write_commands(out / "COMMANDS.sh", [[
            sys.executable, str(BENCH / "interface_campaign.py"), "--out", str(out), "--only", args.only]])
        code = 0
        if args.only in ("all", "rehearse"):
            found = scene(0)
            spec = {
                "kind": "rehearse", "phase": "rehearse", "max_age_ms": 120000, "drain_s": 5,
                "inputs": every(2, 500_000_000),
                "queries": every(2, 1_000_000_000, lambda index: "rehearse"),
                "second_surface": True,
            }
            code = run_session(out / "rehearse", spec, "rehearse", found["geometry_file"],
                               found["config_file"], POINTS[0], commands, out / "COMMANDS.sh")
            ok, missing = rehearsal_ok(out / "rehearse")
            (out / "rehearse" / "TRACE_VALIDATION.json").write_text(json.dumps(
                {"ok": ok, "missing": missing}, indent=2) + "\n")
            if code != 0 or not ok:
                write_commands(out / "COMMANDS.sh", commands)
                print("rehearsal_failed", missing)
                return 4
        if args.only in ("all", "perturb"):
            if time.monotonic() - started > 120 * 60:
                return 4
            report = perturb(out / "perturb", commands, out / "COMMANDS.sh")
            if report.get("stop"):
                write_commands(out / "COMMANDS.sh", commands)
                print("perturbation_stop")
                return 4
        if args.only in ("all", "matrix"):
            order = [("b00_D0", 0, "D0"), ("b00_D1", 0, "D1"), ("b20_D1", 20, "D1"), ("b20_D0", 20, "D0")]
            for name, boxes, mode in order:
                found = scene(boxes)
                inputs = every(66, 500_000_000) if mode == "D1" else []
                spec = {
                    "kind": "arm", "phase": "writes" if mode == "D1" else "quiet",
                    "max_age_ms": 120000, "drain_s": 30, "inputs": inputs,
                    "queries": every(33, 1_000_000_000, lambda index: "warmup" if index < 3 else "measure"),
                }
                code = run_session(out / name, spec, name, found["geometry_file"], found["config_file"],
                                   POINTS[boxes], commands, out / "COMMANDS.sh")
                if code != 0:
                    write_commands(out / "COMMANDS.sh", commands)
                    return code
            found = scene(20)
            normal = {
                "kind": "normal", "phase": "normal", "max_age_ms": 2000, "drain_s": 30,
                "inputs": every(120, 500_000_000),
                "queries": every(60, 1_000_000_000, lambda index: "measure"),
            }
            code = run_session(out / "normal", normal, "normal", found["geometry_file"], found["config_file"],
                               POINTS[20], commands, out / "COMMANDS.sh")
            if code != 0:
                write_commands(out / "COMMANDS.sh", commands)
                return code
            overload = {
                "kind": "overload", "phase": "overload", "max_age_ms": 2000, "drain_s": 30,
                "inputs": every(600, 100_000_000),
                "queries": every(60, 1_000_000_000, lambda index: "measure"),
                "recovery": {
                    "inputs": every(60, 500_000_000),
                    "queries": every(30, 1_000_000_000, lambda index: "recovery"),
                    "drain_s": 30,
                },
            }
            code = run_session(out / "overload", overload, "overload", found["geometry_file"],
                               found["config_file"], POINTS[20], commands, out / "COMMANDS.sh")
            if code != 0:
                write_commands(out / "COMMANDS.sh", commands)
                return code
            sequence = ("R0", "R1", "R1", "R0", "R0", "R1")
            for index, arm in enumerate(sequence, start=1):
                spec = {
                    "kind": "recovery", "arm": arm, "phase": "recovery", "max_age_ms": 2000,
                    "idle_s": 30, "drain_s": 30,
                    "inputs": every(20, 500_000_000),
                    "queries": every(10, 1_000_000_000, lambda i: "first" if i == 0 else "follow"),
                }
                code = run_session(out / ("recovery_%s_%d" % (arm, index)), spec, "recovery-%s-%d" % (arm, index),
                                   found["geometry_file"], found["config_file"], POINTS[20], commands, out / "COMMANDS.sh")
                if code != 0:
                    write_commands(out / "COMMANDS.sh", commands)
                    return code
        if args.only in ("all", "o6"):
            code = run_o6(out / "o6", commands, out / "COMMANDS.sh")
            if code != 0:
                write_commands(out / "COMMANDS.sh", commands)
                return code
        write_commands(out / "COMMANDS.sh", commands)
        manifest = json.loads((out / "MANIFEST.json").read_text())
        manifest["experiment_s"] = time.monotonic() - started
        (out / "MANIFEST.json").write_text(json.dumps(manifest, indent=2) + "\n")
        return 0
    finally:
        fcntl.flock(lock, fcntl.LOCK_UN)
        lock.close()


if __name__ == "__main__":
    raise SystemExit(main(__import__("sys").argv[1:]))
