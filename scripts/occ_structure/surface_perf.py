#!/usr/bin/env python3
"""One surface_columns call per worker process, plus the A/B campaign driver.

Time and memory never share a process. Each side is warmed up 3 times,
then 30 scored pairs run AB on odd pair numbers and BA on even pair
numbers. P95 is the 29th sorted sample. Memory uses max(peak-current0).
"""

import argparse
import gc
import json
import os
import subprocess
import sys
import time
import tracemalloc
from pathlib import Path

GROUPS = (
    ("32-empty", 32, 32, 32, False),
    ("32-box", 32, 32, 32, True),
    ("90-empty", 90, 75, 40, False),
    ("90-box", 90, 75, 40, True),
)
SCRIPT = Path(__file__).resolve()


def build_mapper(nx, ny, nz, with_box):
    from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
    resolution = 0.02
    mapper = CargoVolumeMapper(
        (nx * resolution, ny * resolution, nz * resolution),
        (0.0, 0.0, nz * 0.01), 0.0, resolution, max_raycast_points=None)
    if (mapper.nx, mapper.ny, mapper.nz) != (nx, ny, nz):
        raise SystemExit("shape %s != requested" % ((mapper.nx, mapper.ny, mapper.nz),))
    if with_box:
        mapper.mark_placed_box((0.0, 0.0, 0.2), (0.2, 0.2, 0.2), yaw=0.0)
    return mapper


def once(group_name, metric):
    chosen = None
    for name, nx, ny, nz, with_box in GROUPS:
        if name == group_name:
            chosen = (nx, ny, nz, with_box)
    if chosen is None:
        raise SystemExit("unknown group %s" % group_name)
    mapper = build_mapper(*chosen)
    if metric == "probe":
        tracemalloc.start()
        current0 = tracemalloc.get_traced_memory()[0]
        buf = __import__("numpy").empty(1048576, dtype="float64")
        peak = tracemalloc.get_traced_memory()[1]
        print(json.dumps({"probe_bytes": int(peak - current0), "kept": int(buf.nbytes)}))
        return
    gc.collect()
    if metric == "time":
        if tracemalloc.is_tracing():
            tracemalloc.stop()
        gc.disable()
        try:
            start = time.perf_counter_ns()
            result = __import__(
                "luggage_perception.cargo_grid_projection",
                fromlist=["surface_columns"]).surface_columns(mapper)
            elapsed = time.perf_counter_ns() - start
        finally:
            gc.enable()
        assert result is not None
        print(json.dumps({"ns": int(elapsed)}))
        return
    tracemalloc.start()
    tracemalloc.clear_traces()
    current0 = tracemalloc.get_traced_memory()[0]
    tracemalloc.reset_peak()
    gc.disable()
    try:
        result = __import__(
            "luggage_perception.cargo_grid_projection",
            fromlist=["surface_columns"]).surface_columns(mapper)
        current, peak = tracemalloc.get_traced_memory()
    finally:
        gc.enable()
    assert result is not None
    print(json.dumps({
        "current0": int(current0),
        "current_after": int(current),
        "peak": int(peak),
        "delta": int(peak - current0),
    }))


def spawn(root, group, metric):
    env = os.environ.copy()
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    env["PYTHONPATH"] = ":".join([
        str(Path(root)),
        str(Path(root) / "src/luggage_perception"),
        str(Path(root) / "src/luggage_description"),
    ])
    completed = subprocess.run(
        [sys.executable, str(SCRIPT), "--once", "--group", group, "--metric", metric],
        cwd=str(root), env=env, text=True, capture_output=True)
    if completed.returncode != 0:
        raise SystemExit(completed.stderr or completed.stdout)
    return json.loads(completed.stdout.strip().splitlines()[-1])


def p95(values):
    return sorted(values)[28]


def campaign(baseline, candidate, out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    qualified = True
    summary = {"groups": {}, "aa_qualified": False, "verdict": "NOT_EVALUATED"}
    for name, *_rest in GROUPS:
        probe = spawn(baseline, name, "probe")
        summary["groups"][name] = {"probe_bytes": probe["probe_bytes"]}
        if probe["probe_bytes"] < int(0.9 * 8 * 1024 * 1024):
            qualified = False
        for metric, key in (("time", "ns"), ("memory", "delta")):
            for _ in range(3):
                spawn(baseline, name, metric)
                spawn(baseline, name, metric)
            left = [spawn(baseline, name, metric)[key] for _ in range(30)]
            right = [spawn(baseline, name, metric)[key] for _ in range(30)]
            stat_left = p95(left) if metric == "time" else max(left)
            stat_right = p95(right) if metric == "time" else max(right)
            ratio = None
            if min(stat_left, stat_right) > 0:
                ratio = max(stat_left, stat_right) / min(stat_left, stat_right)
            summary["groups"][name][metric + "_aa"] = {
                "left": stat_left, "right": stat_right, "max_over_min": ratio}
            if ratio is None or ratio > 1.10:
                qualified = False
    summary["aa_qualified"] = qualified
    if not qualified:
        (out / "summary.json").write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary, indent=2))
        return 2
    failed = False
    for name, *_rest in GROUPS:
        for metric, key in (("time", "ns"), ("memory", "delta")):
            for _ in range(3):
                spawn(baseline, name, metric)
                spawn(candidate, name, metric)
            values_a = []
            values_b = []
            for index in range(30):
                order = (baseline, candidate) if index % 2 == 0 else (candidate, baseline)
                first = spawn(order[0], name, metric)
                second = spawn(order[1], name, metric)
                if index % 2 == 0:
                    sample_a, sample_b = first, second
                else:
                    sample_b, sample_a = first, second
                values_a.append(sample_a[key])
                values_b.append(sample_b[key])
                rows.append({
                    "group": name, "metric": metric, "pair": index + 1,
                    "order": "AB" if index % 2 == 0 else "BA",
                    "a": sample_a, "b": sample_b,
                })
            stat_a = p95(values_a) if metric == "time" else max(values_b and values_a)
            if metric == "memory":
                stat_a = max(values_a)
                stat_b = max(values_b)
            else:
                stat_b = p95(values_b)
            ratio = stat_b / stat_a if stat_a else None
            summary["groups"][name][metric] = {
                "a": stat_a, "b": stat_b, "b_over_a": ratio}
            if ratio is None or ratio > 1.10:
                failed = True
    summary["verdict"] = "FAIL" if failed else "PASS"
    (out / "samples.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))
    (out / "summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))
    return 1 if failed else 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--group")
    parser.add_argument("--metric")
    parser.add_argument("--baseline")
    parser.add_argument("--candidate")
    parser.add_argument("--out")
    args = parser.parse_args()
    if args.once:
        once(args.group, args.metric)
        return
    raise SystemExit(campaign(args.baseline, args.candidate, args.out))


if __name__ == "__main__":
    main()
