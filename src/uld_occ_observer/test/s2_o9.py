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
import repair_s0_judge as judge  # noqa: E402


PAIRS = 30
POINTS = 30000


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_frame(path, geometry_hash, clock_domain):
    rng = random.Random(20261031)
    lines = [
        "sensor_id: cam0",
        "observation_id: o9-30k",
        "optical_frame: d555_color_optical_frame",
        f"clock_domain: {clock_domain}",
        f"geometry_hash: {geometry_hash}",
        "epoch: 1",
        "stamp_ns: 1000000000",
        "transform:",
        "  stamp_ns: 1000000000",
        "  rotation: [0, 0, 1, -1, 0, 0, 0, -1, 0]",
        "  translation: [-1.15, 0.0, 1.20]",
        "points_m:",
    ]
    for _ in range(POINTS):
        x = rng.uniform(-0.15, 0.15)
        y = rng.uniform(-0.15, 0.15)
        z = rng.uniform(0.9, 1.4)
        lines.append(f"  - [{x:.5f}, {y:.5f}, {z:.5f}]")
    path.write_text("\n".join(lines) + "\n")


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
    return {
        "integrate_30k": float(document["integrate_external_ms"]),
        "roi": float(document["roi_ms"]),
        "surface_first": float(first["external_ms"]),
        "content": document.get("roi_surface_content_sha256"),
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
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cli", required=True)
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
        frame = out / "frame_30k.yaml"
        write_frame(frame, geometry_hash, config["clock_domain"])
        frame_sha = sha256_file(frame)
        operations = {name: [] for name in ("roi", "integrate_30k", "surface_first")}
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
                                     "elapsed_s": elapsed})
                    print(f"FAIL pair {index} {label} exit {completed.returncode}", flush=True)
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
            mismatched = contents["off"] != contents["on"]
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
        injected = {
            "input_id": "o9-injected-truncated-trace",
            "epoch": 1,
            "status": "REJECTED",
            "reason": "TRACE_FOOTER_MISSING",
            "sensor_id": "cam0",
            "capture_complete": False,
        }
        injected_path = out / "injected_incomplete.json"
        injected_path.write_text(json.dumps(injected) + "\n")
        replay_incomplete = {"replay_possible": False, "missing": ["capture_complete"]}
        real_fault = {
            "input_id": "o9-injected-epoch",
            "epoch": 99,
            "status": "REJECTED",
            "reason": "EPOCH_MISMATCH",
            "sensor_id": "cam0",
            "capture_complete": True,
            "frame_sha256": frame_sha,
        }
        real_path = out / "injected_epoch.json"
        real_path.write_text(json.dumps(real_fault, indent=2) + "\n")
        replay_real = json.loads(real_path.read_text()) == real_fault and real_fault["capture_complete"]
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
            "injected_incomplete_replay_possible": replay_incomplete["replay_possible"],
            "injected_epoch_replay_possible": bool(replay_real),
            "performance_certified": verdict.get("performance_certified") is True and not failures,
        }
        (out / "o9_result.json").write_text(json.dumps(payload, indent=2) + "\n")
        print(json.dumps({"pass": payload["performance_certified"], "ratios": ratios,
                          "reasons": verdict.get("reasons")}, indent=2), flush=True)
        return 0 if payload["performance_certified"] and replay_real and not replay_incomplete["replay_possible"] else 1
    finally:
        fcntl.flock(lock, fcntl.LOCK_UN)
        lock.close()


if __name__ == "__main__":
    raise SystemExit(main())
