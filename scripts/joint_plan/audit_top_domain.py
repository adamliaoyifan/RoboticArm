#!/usr/bin/env python3
"""Measure strict observed support on the existing planar/dropout fixture.

This is diagnostic only: no runtime adoption, no ground truth in admission.
Keep boundary arrays so a zero-candidate case can be inspected and replayed.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
for package in ("luggage_perception", "luggage_description"):
    sys.path.insert(0, str(ROOT / "src" / package))

import numpy as np
from luggage_perception.eval.dynamic_suction_renderer import (
    NadirRenderer, DEFAULT_CAMERA, make_b1_case,
)
from luggage_perception.instance_depth_component import isolate_depth_component
from luggage_perception.dynamic_top_surface import estimate_dynamic_top_surface
from luggage_perception.suction_height_map import build_height_map
from luggage_perception.suction_top_domain import build_top_domain
from luggage_perception.suction_footprint_containment import check_footprint_containment


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    renderer = NadirRenderer()
    intrinsics = SimpleNamespace(**{k: DEFAULT_CAMERA[k]
                                   for k in ("fx", "fy", "cx", "cy")})
    report = dict(revision=subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        dirty=subprocess.check_output(["git", "status", "--porcelain"],
                                      cwd=ROOT, text=True).splitlines(),
        command=sys.argv, cases=[], capture_complete=False,
        scope="strict support sensitivity; not a live acceptance gate")
    for seed in (0, 1, 2):
        case, _ = make_b1_case(renderer, tilt_deg=0, yaw_deg=0,
                               noise_sigma_mm=2., seed=seed, stamp=12.)
        component = isolate_depth_component(case.depth_mm, bbox=case.bbox)
        region = np.zeros(case.depth_mm.shape, bool)
        x0, y0, x1, y1 = case.bbox
        region[y0:y1, x0:x1] = True
        top = estimate_dynamic_top_surface(case.depth_mm, component.mask,
                                          intrinsics, case.mat4, stamp=case.stamp,
                                          frame=case.frame, instance_region=region)
        if top.reason != "ok":
            raise RuntimeError("fixture has no admitted top: " + top.reason)
        hmap = build_height_map(case.depth_mm, intrinsics, case.mat4, top,
                               instance_region=region)
        domain = build_top_domain(hmap, top, case.depth_mm, region, intrinsics,
                                  case.mat4, stamp=case.stamp, frame_id=case.frame)
        checked = contained = 0
        for i in range(0, hmap.nu, 2):
            for j in range(0, hmap.nv, 2):
                center = (hmap.origin_u + (i+.5)*hmap.cell_size_m,
                          hmap.origin_v + (j+.5)*hmap.cell_size_m)
                result = check_footprint_containment(
                    domain.valid_cells, origin_uv=domain.origin_uv,
                    cell_size_m=domain.cell_size_m, center_uv=center, yaw_rad=0.,
                    footprint_size_xy_m=(.18, .18), boundary_margin_m=.015)
                checked += 1
                contained += int(result.contained)
        path = args.output / (case.case_id + ".npz")
        np.savez_compressed(path, depth_mm=case.depth_mm, region=region,
                            component=component.mask, labels=top.plane_labels,
                            patch=top.winning_patch_mask, domain=domain.valid_cells,
                            transform=case.mat4, basis=top.plane_basis,
                            plane_offset=top.plane_offset,
                            origin_uv=domain.origin_uv, cell_size_m=domain.cell_size_m)
        report["cases"].append(dict(case_id=case.case_id, seed=seed,
            observed_top=top.reason, checked=checked, contained=contained,
            valid_cells=int(domain.valid_cells.sum()), cells=domain.valid_cells.size,
            input_recipe="make_b1_case: tilt=0,yaw=0,noise=2mm,default dropout=5%",
            artifact=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    report.update(capture_complete=True, replay_possible=True)
    (args.output / "manifest.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report["cases"], indent=2))


if __name__ == "__main__":
    main()
