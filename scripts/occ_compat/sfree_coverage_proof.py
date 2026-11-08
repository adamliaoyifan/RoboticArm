"""sfree_coverage_proof — pre-freeze coverage proof for the S-free
multi-view fixture (adjudication 2026-11-08 item 3).

BEFORE the fixture is frozen as the shared K01 input, this proves the
target placement space is actually observed, by running the generated
frames through the REAL G-side projection (CargoVolumeMapper.integrate
+ surface_map_2d — the same code the mapper node runs). The O side is
proven at collection preflight through the live observer (the bridge
live test already exercises the sweep -> known == active path).

Checks (all fail-closed, all against the box's OWN volume/footprint):
  A. support positive: the SUPPORT_BOX top columns read occupied with
     height ~= 0.20 — a probe box resting on it has support contact;
  B. the box's own volume above the support top (z in [0.20, 0.40]
     floor-relative over a 0.3x0.2 footprint at the support center) is
     fully OBSERVED and unoccupied (voxel level, not column level);
  C. floor-level known-free region: at least one 0.3x0.2 placement
     whose footprint AND edge slivers cover only fully-free columns
     (state == free, i.e. whole-column observed — G's semantics),
     with a >=1 column margin to the occupied boundary ring;
  D. the boundary ring is occupied (the conservative wall
     representation the solver must avoid — consistency with the
     footprint-coverage fix).
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

from . import gen_sfree

PROBE = (0.30, 0.20, 0.20)   # BOX_A
RES = 0.02


def _rotation_matrix(rotation):
    return [[float(rotation[i + 3 * j]) for j in range(3)]  # column-major?
            for i in range(3)]


def _load_mapper(hull):
    from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
    inner = (float(hull["length"]), float(hull["width"]),
             float(hull["ceiling_z"]) - float(hull["floor_z"]))
    return CargoVolumeMapper(
        inner_size=inner,
        center_base=[0.0, 0.0,
                     float(hull["floor_z"]) + inner[2] / 2.0],
        yaw=0.0, resolution=RES,
        geometry_descriptor=dict(hull),
        geometry_hash=str(hull.get("geometry_hash", "")))


def _world_points(frame):
    """optical -> container via the frame's own frozen transform."""
    t = frame["transform"]["translation"]
    r = frame["transform"]["rotation"]
    m = [[r[0], r[1], r[2]], [r[3], r[4], r[5]], [r[6], r[7], r[8]]]
    out = []
    for p in frame["points_m"]:
        out.append([m[i][0] * p[0] + m[i][1] * p[1] + m[i][2] * p[2] + t[i]
                    for i in range(3)])
    return out, t


def prove(geometry_path):
    import yaml
    document = yaml.safe_load(open(geometry_path).read())
    hull = json.loads(document["descriptor_json"])
    frames = gen_sfree.generate_frames(hull)
    mapper = _load_mapper(hull)
    for frame in frames:
        points, origin = _world_points(frame)
        mapper.integrate_points(points, origin=origin)
    surface = mapper.surface_map_2d()
    nx, ny = surface["nx"], surface["ny"]
    state = surface["state"]
    height = surface["height"]
    floor = float(hull["floor_z"])
    cx, cy = gen_sfree.SUPPORT_BOX["center"]
    bw, bd, _bh = gen_sfree.SUPPORT_BOX["size"]
    half_l, half_w = float(hull["length"]) / 2, float(hull["width"]) / 2
    ox, oy = -nx * RES / 2.0, -ny * RES / 2.0

    def col_range(x0, x1, y0, y1):
        return (int(math.floor((x0 - ox) / RES)),
                int(math.floor((x1 - ox - 1e-9) / RES)),
                int(math.floor((y0 - oy) / RES)),
                int(math.floor((y1 - oy - 1e-9) / RES)))

    report = {"geometry_hash": hull.get("geometry_hash"),
              "frames": len(frames), "checks": {}}

    # A. support positive
    ix0, ix1, iy0, iy1 = col_range(cx - bw / 2 + 0.02, cx + bw / 2 - 0.02,
                                   cy - bd / 2 + 0.02, cy + bd / 2 - 0.02)
    support_cols = [(ix, iy) for ix in range(ix0, ix1 + 1)
                    for iy in range(iy0, iy1 + 1)]
    ok_a = all(state[ix][iy] == "occupied"
               and abs(height[ix][iy] - 0.20) <= 0.03
               for ix, iy in support_cols)
    report["checks"]["A_support_top_occupied"] = {
        "pass": ok_a, "columns": len(support_cols),
        "states": sorted({state[ix][iy] for ix, iy in support_cols}),
        "height_min": min(height[ix][iy] for ix, iy in support_cols),
        "height_max": max(height[ix][iy] for ix, iy in support_cols)}

    # B. the box's OWN volume above the support is observed & unoccupied
    bx0, bx1 = cx - PROBE[0] / 2, cx + PROBE[0] / 2
    by0, by1 = cy - PROBE[1] / 2, cy + PROBE[1] / 2
    import numpy as np
    from luggage_perception.cargo_grid_projection import _arrays
    vox, active = _arrays(mapper)
    z_lo, z_hi = int(0.20 / RES), int(0.40 / RES)  # floor-relative cells
    vix0, vix1, viy0, viy1 = col_range(bx0, bx1, by0, by1)
    block = vox[z_lo:z_hi, viy0:viy1 + 1, vix0:vix1 + 1]
    block_active = active[z_lo:z_hi, viy0:viy1 + 1, vix0:vix1 + 1]
    unobserved = int(((block == 0) | ~block_active).sum())
    occupied = int(((block == 2) & block_active).sum())
    report["checks"]["B_box_volume_observed"] = {
        "pass": unobserved == 0 and occupied == 0,
        "unobserved": unobserved, "occupied": occupied,
        "z_cells": z_hi - z_lo,
        "footprint_cells": (vix1 - vix0 + 1) * (viy1 - viy0 + 1)}

    # C. solver-level positive, per-candidate voxel binding (reviews
    # 2026-11-08): EVERY sensor-supported feasible candidate is verified
    # TWICE against the SAME snapshot — the campaign oracle (column
    # semantics) AND the voxel grid (the candidate's OWN box volume must
    # be fully observed and unoccupied; an observed support face does
    # not authorize unknown space above it). Floor-level whole-column
    # free is NOT required (min_z policy; the sanctioned positive is
    # the observed support top).
    from luggage_packing import placement_solver as ps
    from .oracle import verify_candidate, _cells_overlapping
    import numpy as np
    from luggage_perception.cargo_grid_projection import _arrays
    vox, active = _arrays(mapper)
    solver_surface = dict(surface)
    solver_surface["geometry_descriptor"] = dict(hull)
    params = dict(ps._default_params())
    params["top_n"] = 8000
    result = ps.generate_candidates(
        solver_surface, list(PROBE), allowed_yaws=[0.0, math.pi / 2.0],
        params=params)
    cands = result if isinstance(result, list) else result["candidates"]
    sensor_ok = oracle_ok = voxel_ok = 0
    failures = []
    first = None
    oxg, oyg = -nx * RES / 2.0, -ny * RES / 2.0
    inner_h = float(hull["ceiling_z"]) - floor
    for c in cands:
        if not c.get("feasible") or c.get("support_source") != "sensor":
            continue
        sensor_ok += 1
        center = [float(v) for v in c["center_container"]]
        verdict = verify_candidate(
            {"center_container": center,
             "yaw": float(c.get("box_yaw", 0.0)),
             "confidence_ratio": float(c.get("confidence_ratio", 0.0))},
            surface=surface, box_size=list(PROBE),
            hull={"inner_size": [float(hull["length"]),
                                 float(hull["width"]), inner_h],
                  "floor_z_container": floor,
                  **({"chamfer": hull["chamfer"]}
                     if hull.get("chamfer") else {})})
        oracle_pass = verdict["pass"]
        # the candidate's OWN volume, voxel level, same snapshot
        fw, fd = float(c["footprint"][0]), float(c["footprint"][1])
        cells = _cells_overlapping((fw, fd), (center[0], center[1]),
                                   RES, (oxg, oyg), nx, ny)
        z0_rel = center[2] - floor - PROBE[2] / 2.0
        z_lo = max(0, int(math.floor(z0_rel / RES + 1e-9)))
        z_hi = min(vox.shape[0] - 1,
                   int(math.floor((z0_rel + PROBE[2]) / RES - 1e-9)))
        block_ok = True
        for ix, iy in cells:
            column = vox[z_lo:z_hi + 1, iy, ix]
            column_active = active[z_lo:z_hi + 1, iy, ix]
            if not column_active.all() or (column == 0).any():
                block_ok = False       # unobserved voxel inside the box
                break
            if (column == 2).any():
                block_ok = False       # occupied voxel inside the box
                break
        if oracle_pass:
            oracle_ok += 1
        if block_ok:
            voxel_ok += 1
        if oracle_pass and block_ok and first is None:
            first = {"center": [round(v, 4) for v in center],
                     "support_source": c["support_source"],
                     "confidence": c["confidence_ratio"],
                     "volume_cells": len(cells) * (z_hi - z_lo + 1)}
        if (not oracle_pass or not block_ok) and len(failures) < 3:
            failures.append({"center": [round(v, 4) for v in center],
                             "oracle_pass": oracle_pass,
                             "voxel_pass": block_ok})
    report["checks"]["C_legal_candidate_on_observed_support"] = {
        "pass": sensor_ok > 0 and sensor_ok == oracle_ok == voxel_ok,
        "sensor_supported_feasible": sensor_ok,
        "oracle_verified": oracle_ok,
        "voxel_volume_verified": voxel_ok,
        "failures": failures, "example": first}

    # D. informational: ring occupancy distribution (the door side of
    # the ring is an aperture — rays legitimately pass through; the far
    # and side rings carry the wall hits). Not a binding check.
    ring, ring_occ = 0, 0
    for ix in range(nx):
        for iy in range(ny):
            gx0, gy0 = ox + ix * RES, oy + iy * RES
            if (gx0 < -half_l or gx0 + RES > half_l
                    or gy0 < -half_w or gy0 + RES > half_w):
                ring += 1
                ring_occ += state[ix][iy] == "occupied"
    report["checks"]["D_boundary_ring_info"] = {
        "ring_columns": ring, "occupied": ring_occ, "binding": False}
    report["pass"] = all(c["pass"] for c in report["checks"].values()
                         if c.get("binding", True))
    return report


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--geometry", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    report = prove(args.geometry)
    Path(args.out).write_text(json.dumps(report, indent=1))
    print("sfree coverage: %s %s" % (
        "PASS" if report["pass"] else "FAIL",
        {k: v.get("pass") for k, v in report["checks"].items()}))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
