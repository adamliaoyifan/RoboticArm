#!/usr/bin/env python3
"""F-gate offline matrix: 9 classes x 10 variants x 2 boxes x 2 yaws = 360 cases.

Each case runs the REAL product core through observer_cli: the generated
command sequence integrates the scene's synthetic rays, acquires leases
(pinning epochs/revisions where the class demands it), walks the freespace
pages, and releases. The judge compares the artifact against the independent
Python oracle (freespace_oracle.py) recomputed live from the same ray list --
no expectation is baked into the manifest.

Scene rays are axis-aligned at cell centers (rows sweep one layer of a row
band; plane fans drop to one z layer), so the analytic cell model and the
product's ray traversal agree cell-for-cell with no boundary ambiguity.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import freespace_contract as contract  # noqa: E402
import freespace_judge  # noqa: E402
import freespace_oracle  # noqa: E402

PLUS_X_ROTATION = [0, 0, 1, 0, -1, 0, 1, 0, 0]   # optical z = +x travel
DOWN_ROTATION = [1, 0, 0, 0, -1, 0, 0, 0, -1]    # optical z = -z travel


def rot_transpose_apply(rotation, d):
    return [rotation[0] * d[0] + rotation[3] * d[1] + rotation[6] * d[2],
            rotation[1] * d[0] + rotation[4] * d[1] + rotation[7] * d[2],
            rotation[2] * d[0] + rotation[5] * d[1] + rotation[8] * d[2]]


class CaseBuilder:
    def __init__(self, descriptor, resolution, config):
        self.descriptor = descriptor
        self.res = resolution
        self.config = config
        self.shape = (int(math.ceil(descriptor["length"] / resolution)),
                      int(math.ceil(descriptor["width"] / resolution)),
                      int(math.ceil((descriptor["ceiling_z"] - descriptor["floor_z"]) / resolution)))
        self.origin = (-descriptor["length"] / 2, -descriptor["width"] / 2,
                       descriptor["floor_z"])
        self.rays = []
        self.meta = {}
        self.commands = []
        self.stamp = 1_000_000_000
        self.count = 0

    def cell_center(self, x, y, z):
        return (self.origin[0] + (x + 0.5) * self.res,
                self.origin[1] + (y + 0.5) * self.res,
                self.origin[2] + (z + 0.5) * self.res)

    def observation(self, origin, rotation, endpoints, epoch=1):
        self.stamp += 10_000_000
        points = []
        for target in endpoints:
            d = [target[0] - origin[0], target[1] - origin[1], target[2] - origin[2]]
            points.append(rot_transpose_apply(rotation, d))
        self.commands.append({
            "op": "integrate",
            "sensor_id": "cam0",
            "observation_id": "obs%d" % self.count,
            "optical_frame": "synthetic",
            "clock_domain": self.config["clock_domain"],
            "geometry_hash": self.descriptor["geometry_hash"],
            "epoch": epoch,
            "stamp_ns": self.stamp,
            "transform": {"stamp_ns": self.stamp, "rotation": rotation,
                          "translation": list(origin)},
            "points_m": points,
        })
        self.count += 1

    def row_band(self, z_from, z_to, y_from, y_to, x_end, epoch=1):
        """Sweep contiguous layers [z_from, z_to] of a row band, one
        observation PER (row, layer): only a ray that starts outside the
        domain at the row's own (y, z) covers the row's full prefix -- any
        bundling sends diagonals that leave the band's low-x cells unknown
        and break the analytic model. Scene sizes stay small enough that the
        per-cell observation count is not the bottleneck."""
        for z in range(z_from, z_to + 1):
            self.rays.append({"kind": "row_block", "z": z, "y_from": y_from,
                              "y_to": y_to, "x_end": x_end})
            z_world = self.cell_center(0, 0, z)[2]
            for y in range(y_from, y_to):
                y_world = self.cell_center(0, y, z)[1]
                endpoints = [self.cell_center(x_end, y, z)]
                self.observation((-1.15, y_world, z_world), PLUS_X_ROTATION,
                                 endpoints, epoch)

    def occupied_strip(self, z, y_from, y_to, x0, x1, epoch=1):
        """A support plane: per row, one observation whose ray stops at EVERY
        column of [x0, x1). Traversed cells before the strip are free; every
        endpoint cell is occupied (an occupied key always wins over an
        earlier free mark). No vertical rays, so nothing above or below the
        layer is ever swept by this primitive."""
        self.rays.append({"kind": "occupied_strip", "z": z, "y_from": y_from,
                          "y_to": y_to, "x0": x0, "x1": x1})
        z_world = self.cell_center(0, 0, z)[2]
        for y in range(y_from, y_to):
            endpoints = [self.cell_center(x, y, z) for x in range(x0, x1)]
            self.observation((-1.15, self.cell_center(0, y, z)[1], z_world),
                             PLUS_X_ROTATION, endpoints, epoch)

    def acquire(self, expect="OK", **fields):
        command = {"op": "acquire", "now_ns": self.stamp + 1_000_000_000,
                   "expect_status": expect}
        command.update(fields)
        self.commands.append(command)
        return command

    def freespace(self, box, yaw_mode, cursor=0, limit=65536, handle=None):
        command = {"op": "freespace", "box": list(box), "yaw_mode": yaw_mode,
                   "cursor": cursor, "limit": limit}
        if handle is not None:
            command["handle"] = handle
        self.commands.append(command)

    def release(self, handle=None):
        command = {"op": "release"}
        if handle is not None:
            command["handle"] = handle
        self.commands.append(command)

    def reset(self, expected_epoch):
        self.commands.append({"op": "reset", "expected_epoch": expected_epoch})


def build_case(cls, seed, box, yaw, descriptor, resolution, config):
    sx, sy, sz = (int(math.ceil(descriptor["length"] / resolution)),
                  int(math.ceil(descriptor["width"] / resolution)),
                  int(math.ceil((descriptor["ceiling_z"] - descriptor["floor_z"]) / resolution)))
    builder = CaseBuilder(descriptor, resolution, config)
    yaw_mode = 0 if yaw == 0 else 1
    if cls == "empty_tree":
        builder.acquire(expect="NO_DATA")
        return builder
    if cls == "observed_empty":
        depth = 10 + seed % 4
        builder.row_band(0, depth, 10, 42, sx - 1)
        builder.acquire()
        builder.freespace(box, yaw_mode)
        if seed % 3 == 0:
            builder.freespace(box, yaw_mode, cursor=3, limit=5)
        builder.release()
        return builder
    if cls == "full_container":
        builder.occupied_strip(62 + seed % 2, 1, sy - 1, 1, sx - 1)
        builder.acquire()
        builder.freespace(box, yaw_mode)
        builder.release()
        return builder
    if cls == "partial_unknown":
        builder.row_band(0, 9, 10, 26 + 2 * seed, sx - 1)
        builder.acquire()
        builder.freespace(box, yaw_mode)
        builder.release()
        return builder
    if cls == "occlusion":
        occluder = 40 + seed
        builder.row_band(0, 11, 10, 22, sx - 1)
        builder.row_band(0, 11, 26, 38, sx - 1)
        builder.row_band(0, 11, 22, 26, occluder)
        builder.acquire()
        builder.freespace(box, yaw_mode)
        builder.release()
        return builder
    if cls == "floating_interlayer":
        gap_observed = seed % 2 == 0
        if gap_observed:
            builder.row_band(0, 21, 10, 40, sx - 1)
        else:
            builder.row_band(0, 7, 10, 40, sx - 1)
            builder.row_band(18, 21, 10, 40, sx - 1)
        builder.occupied_strip(7, 14, 26, 14, 22)
        builder.occupied_strip(18, 26, 34, 26, 32)
        builder.acquire()
        builder.freespace(box, yaw_mode)
        builder.release()
        return builder
    if cls == "boundary":
        builder.row_band(0, 9 + seed % 2, 68 + seed % 3, sy - 2, sx - 1)
        builder.acquire()
        builder.freespace(box, yaw_mode)
        builder.release()
        return builder
    if cls == "two_version":
        builder.row_band(0, 11, 10, 42, sx - 1)
        builder.meta["old_ray_count"] = len(builder.rays)
        builder.acquire()
        builder.freespace(box, yaw_mode)      # old view
        builder.occupied_strip(10, 14, 26, 14, 22)
        builder.acquire()
        builder.freespace(box, yaw_mode)      # new view
        builder.release()
        return builder
    if cls == "reset":
        builder.row_band(0, 11, 10, 42, sx - 1)
        builder.reset(1)
        builder.rays.clear()  # only the post-reset map is expected
        builder.acquire(expect="NO_DATA")
        builder.row_band(5, 5, 12, 16, sx - 1, epoch=2)
        builder.acquire(expect="EPOCH_MISMATCH", expected_epoch=1)
        builder.acquire()
        builder.freespace(box, yaw_mode)
        builder.release()
        return builder
    raise ValueError(cls)


def load_descriptor(geometry_path):
    document = yaml.safe_load(Path(geometry_path).read_text())
    return json.loads(document["descriptor_json"])


def load_config(config_path):
    document = yaml.safe_load(Path(config_path).read_text())
    return {key: document[key] for key in
            ("resolution_m", "depth_z_min_m", "depth_z_max_m", "clock_domain")}


def run_case(cli, geometry, config, builder, case_dir):
    commands_path = case_dir / "commands.yaml"
    responses_path = case_dir / "responses.yaml"
    commands_path.write_text(yaml.safe_dump(builder.commands, sort_keys=False))
    completed = subprocess.run(
        [cli, "--geometry", geometry, "--config", config,
         "--commands", str(commands_path), "--out", str(responses_path)],
        text=True, capture_output=True)
    responses = yaml.safe_load(responses_path.read_text()) if responses_path.exists() else None
    return completed, responses


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--geometry", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--cli", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--classes", default=",".join(contract.F_CLASSES))
    parser.add_argument("--seeds", type=int, default=contract.F_VARIANTS)
    args = parser.parse_args()
    descriptor = load_descriptor(args.geometry)
    config = load_config(args.config)
    resolution = float(config["resolution_m"])
    out = Path(args.out)
    (out / "cases").mkdir(parents=True, exist_ok=True)
    classes = args.classes.split(",")
    manifest = {"schema": contract.SCHEMA, "plan_revision": contract.PLAN_REVISION,
                "geometry": args.geometry, "config": args.config,
                "descriptor_json": json.dumps(descriptor, sort_keys=True),
                "resolution": resolution, "generator": "freespace_cases_v1",
                "cases": []}
    results = []
    for cls in classes:
        for seed in range(args.seeds):
            for box in contract.F_BOXES:
                for yaw in contract.F_YAWS:
                    case_id = "%s_s%02d_b%d_y%d" % (cls, seed,
                                                    contract.F_BOXES.index(box), yaw)
                    builder = build_case(cls, seed, box, yaw, descriptor, resolution, config)
                    case_dir = out / "cases" / case_id
                    case_dir.mkdir(parents=True, exist_ok=True)
                    completed, responses = run_case(args.cli, args.geometry, args.config,
                                                    builder, case_dir)
                    rays_manifest = {
                        "case_id": case_id, "class": cls, "seed": seed,
                        "box": list(box), "yaw": yaw,
                        "descriptor_json": json.dumps(descriptor, sort_keys=True),
                        "resolution": resolution,
                        "rays": builder.rays,
                        "meta": builder.meta,
                        "commands_sha256": hashlib.sha256(
                            (case_dir / "commands.yaml").read_bytes()).hexdigest(),
                        "cli_returncode": completed.returncode,
                    }
                    manifest["cases"].append(rays_manifest)
                    verdict = freespace_judge.judge_f_case(
                        rays_manifest, responses, descriptor, resolution)
                    results.append({"case_id": case_id, "class": cls,
                                    "verdict": verdict})
                    if verdict["reasons"]:
                        print("FAIL %s: %s" % (case_id, verdict["reasons"][:4]), flush=True)
    (out / "manifest.json").write_text(json.dumps(manifest, indent=1))
    passed = sum(1 for r in results if not r["verdict"]["reasons"])
    summary = {"total": len(results), "passed": passed,
               "failed": len(results) - passed,
               "failures": [r for r in results if r["verdict"]["reasons"]]}
    (out / "summary.json").write_text(json.dumps(summary, indent=1))
    print("F matrix: %d/%d passed" % (passed, len(results)))
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
