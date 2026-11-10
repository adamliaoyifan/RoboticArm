#!/usr/bin/env python3
"""O9 capture perturbation on one frozen profile CLI.

Each pair runs the same 30k frame twice, trace off and trace on, in AB/BA
order. First surface must be a cache miss. The official O9 judge scores the
ratios. Injected failures are stored beside the pairs and replayed; they are
not inserted into the ratio samples.
"""
import argparse
import fcntl
import hashlib
import json
import math
import os
import platform
import random
import subprocess
import sys
import time
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent / "benchmark"))
import capture_replay  # noqa: E402
import repair_s0_judge as judge  # noqa: E402


PAIRS = 30
POINTS = 30000
# Unified delivery contract §8.8: the whole-query chain is a REQUIRED O9
# operation. One box size per run (alternating), both yaws in one request.
FREESPACE_BOXES = ((0.30, 0.20, 0.20), (0.50, 0.30, 0.30))


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def emit_frame(eval_native, geometry, config, out):
    """One scored frame from occ_load, group r02_b00_chamfer_c30000_o50_x00."""
    dest = out / "emitted"
    completed = subprocess.run([
        eval_native, "--geometry", geometry, "--config", config, "--gate", "emit-window",
        "--group", "r02_b00_chamfer_c30000_o50_x00", "--boxes", "0", "--cloud", "30000",
        "--outside", "0.5", "--cross", "0", "--warmup", "0", "--repeat", "1", "--out", str(dest),
    ], text=True, capture_output=True)
    if completed.returncode != 0:
        raise RuntimeError(completed.stderr[-1000:] or completed.stdout[-1000:])
    frame = dest / "frames" / "f000.yaml"
    if not frame.is_file():
        raise RuntimeError("generator did not write f000.yaml")
    return frame


def run_cli(cli, geometry, config, manifest, out, trace, trace_path):
    env = os.environ.copy()
    env.pop("ULD_OCC_INTERFACE_TRACE", None)
    env.pop("ULD_OCC_TRACE_PATH", None)
    if trace:
        env["ULD_OCC_INTERFACE_TRACE"] = "1"
        env["ULD_OCC_TRACE_PATH"] = str(trace_path)
        env["ULD_OCC_TRACE_RUN_ID"] = trace_path.stem
    started = time.monotonic()
    completed = subprocess.run(
        [cli, "--geometry", geometry, "--config", config, "--manifest", str(manifest), "--out", str(out)],
        env=env, text=True, capture_output=True)
    return completed, time.monotonic() - started


def sample_metrics(path):
    document = next(yaml.safe_load_all(path.read_text()))
    if document.get("status") != "OK":
        raise RuntimeError(document.get("status"))
    first = document["surface_first_call"]
    second = document["surface_second_call"]
    if first.get("cache_hit") is not False or second.get("cache_hit") is not True:
        raise RuntimeError(f"surface cache state first={first.get('cache_hit')} second={second.get('cache_hit')}")
    if int(document.get("roi_cells") or 0) != 4096:
        raise RuntimeError("roi_cells=%s" % document.get("roi_cells"))
    if not document.get("roi_surface_content_sha256"):
        raise RuntimeError("missing content hash")
    return {
        "integrate_30k": float(document["integrate_external_ms"]),
        "roi": float(document["roi_ms"]),
        "surface_first": float(first["external_ms"]),
        "freespace_full": float(document["freespace_full_external_ms"]),
        "freespace_content": document["freespace_content_sha256"],
        "freespace_candidates": document.get("freespace_candidates"),
        "content": document["roi_surface_content_sha256"],
        "rss_kib": document.get("process_peak_rss_kib"),
    }


def environment():
    governor = "unknown"
    gov = Path("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor")
    if gov.is_file():
        governor = gov.read_text().strip()
    return {
        "governor": governor,
        "cpu_pinning": "none",
        "logical_cpus": os.cpu_count(),
        "client_executor_threads": 4,
        "integrate_threads": 1,
        "rmw": os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        "platform": platform.platform(),
        "python": platform.python_version(),
        "loadavg": Path("/proc/loadavg").read_text().split()[:3] if Path("/proc/loadavg").is_file() else [],
        "cpu_freq_khz": Path("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq").read_text().strip()
        if Path("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq").is_file() else "unknown",
        "context_switches": next((line.split()[1] for line in Path("/proc/stat").read_text().splitlines()
                                  if line.startswith("ctxt ")), "unknown"),
        "logical_cpus_are_utilization": False,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cli", required=True)
    parser.add_argument("--eval-native", required=True)
    parser.add_argument("--geometry", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--pairs", type=int, default=PAIRS)
    args = parser.parse_args()
    if args.pairs < 1:
        raise SystemExit("pairs must be positive")
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    lock_path = Path("/tmp/elfin-occ-backend-perf.lock")
    lock_path.touch()
    lock = lock_path.open("a+")
    print("waiting for /tmp/elfin-occ-backend-perf.lock", flush=True)
    wait_started = time.monotonic()
    fcntl.flock(lock, fcntl.LOCK_EX)
    lock_wait_s = time.monotonic() - wait_started
    print(f"lock held after {lock_wait_s:.3f}s", flush=True)
    try:
        geometry = yaml.safe_load(Path(args.geometry).read_text())
        config = yaml.safe_load(Path(args.config).read_text())
        geometry_hash = json.loads(geometry["descriptor_json"])["geometry_hash"]
        frame = emit_frame(args.eval_native, args.geometry, args.config, out)
        frame_sha = sha256_file(frame)
        operations = {name: [] for name in
                      ("roi", "integrate_30k", "surface_first", "freespace_full")}
        failures = []
        rss = []
        pair_count = args.pairs
        for index in range(pair_count):
            order = "AB" if index % 2 == 0 else "BA"
            sides = [("off", False), ("on", True)]
            if order == "BA":
                sides = list(reversed(sides))
            measured = {}
            contents = {}
            for label, trace in sides:
                run_dir = out / f"pair_{index:02d}_{label}"
                run_dir.mkdir()
                manifest = run_dir / "manifest.yaml"
                manifest.write_text(yaml.safe_dump({
                    "case_id": f"o9-{index:02d}-{label}",
                    "mode": "off",
                    "surface": True,
                    "state_digest": True,
                    # Whole-query chain (acquire -> all pages -> release);
                    # boxes alternate per pair, both yaws in one request.
                    "freespace": {"box": list(FREESPACE_BOXES[index % 2]),
                                  "yaw_mode": 2, "page_limit": 65536},
                    "frames": [{
                        "path": os.path.relpath(frame, run_dir),
                        "sha256": frame_sha,
                        "warmup": False,
                        "expect_status": "INTEGRATED",
                    }],
                }, sort_keys=False))
                samples = run_dir / "samples.yaml"
                completed, elapsed = run_cli(
                    args.cli, args.geometry, args.config, manifest, samples, trace,
                    run_dir / "trace.jsonl")
                (run_dir / "cli.log").write_text(completed.stdout + "\n" + completed.stderr)
                if completed.returncode != 0:
                    failures.append({"index": index, "label": label, "returncode": completed.returncode,
                                     "elapsed_s": elapsed, "stderr": completed.stderr[-400:]})
                    print(f"FAIL pair {index} {label} exit {completed.returncode}", flush=True)
                    break
                if trace:
                    expected_id = yaml.safe_load(frame.read_text())["observation_id"]
                    certified = capture_replay.certify_trace(
                        run_dir / "trace.jsonl",
                        capture_replay.expected_profile_trace(expected_id, run_id="trace"))
                    if not certified["capture_complete"]:
                        failures.append({"index": index, "label": label, "trace": certified["errors"]})
                        print(f"FAIL pair {index} trace {certified['errors']}", flush=True)
                        break
                metrics = sample_metrics(samples)
                measured[label] = metrics
                contents[label] = metrics["content"]
                if metrics["rss_kib"] is not None:
                    rss.append(metrics["rss_kib"])
            if "off" not in measured or "on" not in measured:
                for name in operations:
                    operations[name].append({"index": index, "order": order, "missing": True})
                continue
            freespace_same = measured["off"]["freespace_content"] == measured["on"]["freespace_content"] \
                and measured["off"]["freespace_candidates"] == measured["on"]["freespace_candidates"]
            mismatched = not contents.get("off") or contents.get("off") != contents.get("on") \
                or not freespace_same
            for name in operations:
                operations[name].append({
                    "index": index, "order": order, "off_ms": measured["off"][name],
                    "on_ms": measured["on"][name], "missing": False, "mismatched": mismatched,
                    "injected_fault": None,
                })
            print(f"pair {index} {order} integrate {measured['off']['integrate_30k']:.1f}/"
                  f"{measured['on']['integrate_30k']:.1f} ms", flush=True)
        case = {"gate": "o9", "operations": operations}
        verdict = judge.judge_case(case)
        epoch_frame = out / "epoch99.yaml"
        epoch_text = frame.read_text().replace("\nepoch: 1\n", "\nepoch: 99\n", 1)
        epoch_text = epoch_text.replace("observation_id: r02_b00_chamfer_c30000_o50_x00-0",
                                        "observation_id: epoch-reject", 1)
        epoch_frame.write_text(epoch_text)
        epoch_dir = out / "epoch_first"
        epoch_dir.mkdir()
        epoch_manifest = epoch_dir / "manifest.yaml"
        epoch_manifest.write_text(yaml.safe_dump({
            "case_id": "epoch-reject", "mode": "off", "surface": False, "state_digest": False,
            "frames": [{"path": os.path.relpath(epoch_frame, epoch_dir),
                        "sha256": sha256_file(epoch_frame), "warmup": False,
                        "expect_status": "EPOCH_MISMATCH"}],
        }, sort_keys=False))
        epoch_samples = epoch_dir / "samples.yaml"
        epoch_run = subprocess.run(capture_replay.profile_command(
            args.cli, args.geometry, args.config, epoch_manifest, epoch_samples),
            text=True, capture_output=True)
        epoch_replay = {"replay_possible": False, "executed": False}
        if epoch_run.returncode == 0:
            row = next(yaml.safe_load_all(epoch_samples.read_text()))
            receipt = row["receipt"]
            bundle = {
                "observation_id": receipt["observation_id"],
                "epoch": receipt["epoch"],
                "input_epoch": 99,
                "status": receipt["status"],
                "reason": receipt["status"],
                "sensor_id": receipt["sensor_id"],
                "input_file": str(epoch_frame),
                "input_sha256": sha256_file(epoch_frame),
                "geometry_file": args.geometry,
                "geometry_sha256": sha256_file(args.geometry),
                "config_file": args.config,
                "config_sha256": sha256_file(args.config),
                "command": capture_replay.profile_command(args.cli, args.geometry, args.config),
                "tf": {"present": True, "stamp_ns": receipt["input_stamp_ns"], "source": "input_transform"},
                "map_prefix": {"tree_nodes": row["tree_nodes"]},
            }
            bundle_path = out / "injected_epoch.json"
            bundle_path.write_text(json.dumps(bundle, indent=2) + "\n")
            epoch_replay = capture_replay.replay_failure(bundle_path, out / "epoch_replay")
        fabricated = capture_replay.replay_failure(
            "/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_review/fabricated_failure.json",
            out / "fabricated_unused")
        ratios = {}
        for name, pairs in operations.items():
            off = sorted(pair["off_ms"] for pair in pairs if "off_ms" in pair)
            on = sorted(pair["on_ms"] for pair in pairs if "on_ms" in pair)
            if len(off) == pair_count and len(on) == pair_count and off[math.ceil(0.95 * pair_count) - 1] > 0:
                ratios[name] = on[math.ceil(0.95 * pair_count) - 1] / off[math.ceil(0.95 * pair_count) - 1]
        payload = {
            "verdict": verdict,
            "ratios": ratios,
            "lock_wait_s": lock_wait_s,
            "failures": failures,
            "environment": environment(),
            "frame_sha256": frame_sha,
            "frame_points": POINTS,
            "cli_sha256": sha256_file(args.cli),
            "geometry_sha256": sha256_file(args.geometry),
            "config_sha256": sha256_file(args.config),
            "peak_rss_kib": max(rss) if rss else None,
            "fabricated_replay_possible": fabricated["replay_possible"],
            "fabricated_executed": fabricated["executed"],
            "epoch_replay": epoch_replay,
            "latency_gate": verdict.get("pass") is True and not failures,
            "capture_gate": not failures and epoch_replay.get("replay_possible") is True
            and epoch_replay.get("executed") is True and fabricated["replay_possible"] is False,
            "performance_certified": False,
            "qualification": "pending",
        }
        payload["qualification"] = (
            "pass" if payload["latency_gate"] and payload["capture_gate"] else "fail")
        (out / "o9_result.json").write_text(json.dumps(payload, indent=2) + "\n")
        print(json.dumps({"qualification": payload["qualification"], "ratios": ratios,
                          "reasons": verdict.get("reasons"), "epoch_replay": epoch_replay.get("replay_possible"),
                          "failures": len(failures)}, indent=2), flush=True)
        return 0 if payload["qualification"] == "pass" else 1
    finally:
        fcntl.flock(lock, fcntl.LOCK_UN)
        lock.close()


if __name__ == "__main__":
    raise SystemExit(main())
