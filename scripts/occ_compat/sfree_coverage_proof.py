"""sfree_coverage_proof — pre-freeze coverage proof for the S-free
multi-view fixture (adjudication 2026-11-08 item 3; reviews
2026-11-08_1751 round).

BEFORE the fixture is frozen as the shared K01 input, this proves the
target placement space is actually observed, by running the generated
frames through the REAL G-side projection (CargoVolumeMapper.integrate
+ surface_map_2d — the same code the mapper node runs). The O side is
proven live by sfree_o_rehearsal (observer -> bridge -> product solver
-> paged ReadCells evidence, both phases).

Checks (all fail-closed, all against the box's OWN volume/footprint):
  A. support positive: the SUPPORT_BOX top columns read occupied with
     height ~= 0.20 — a probe box resting on it has support contact;
  B. the box's own volume above the support top (z in [0.20, 0.40]
     floor-relative over a 0.3x0.2 footprint at the support center) is
     fully OBSERVED and unoccupied (voxel level, not column level);
  C. per-candidate triple binding, in the PRODUCT call shape: every
     sensor-supported candidate that generate_candidates authorizes
     under the same-snapshot volume_verifier must ALSO pass the
     independent campaign oracle (column semantics) AND read fully
     observed/unoccupied through placement_solver.volume_state_in_
     evidence (complete XYZ ranges — the frozen geometry the product
     itself ships). An observed support face never authorizes unknown
     space above it;
  N. negative control (reviews 2026-11-08_1751 finding 2): the
     single-origin (0.0, 1.20) subset still yields a surface that
     OFFERS stacking candidates (the counterfactual: the surface-only
     product path would authorize them), and the product volume gate
     rejects exactly those whose box volumes hold unobserved voxels —
     with the external scan agreeing on every flip;
  T. tolerance unification binding (finding 3): the solver's frozen
     constants equal the independent oracle's frozen policy values;
  D. informational: the boundary ring occupancy distribution.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

from . import gen_sfree
from .oracle import POLICY as ORACLE_POLICY, _OVERLAP_MIN_M, verify_candidate

PROBE = (0.30, 0.20, 0.20)   # BOX_A
RES = 0.02
#: The negative-control origin (offline counterpart of the live O
#: rehearsal's NEGATIVE_ORIGINS).
NEGATIVE_ORIGINS = [(0.0, 1.20)]


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


def _pipeline(hull, origins):
    """Real G projection over an origin subset -> (surface, evidence).

    The evidence is the SAME normalized same-snapshot voxel grid the
    product's volume gate consumes (x-fastest flat states; layer 0 at
    the hull floor), flattened from the mapper's own arrays."""
    import numpy as np
    from luggage_perception.cargo_grid_projection import _arrays
    frames = gen_sfree.generate_frames(hull, origins_yz=origins)
    mapper = _load_mapper(hull)
    for frame in frames:
        points, origin = _world_points(frame)
        mapper.integrate_points(points, origin=origin)
    surface = mapper.surface_map_2d()
    vox, active = _arrays(mapper)
    states = np.where(active, vox, 0).flatten(order="C").astype(int)
    nx, ny = int(surface["nx"]), int(surface["ny"])
    evidence = {
        "resolution": RES, "nx": nx, "ny": ny, "nz": int(vox.shape[0]),
        "origin_xy": [-nx * RES / 2.0, -ny * RES / 2.0],
        "floor_z": float(hull["floor_z"]),
        "states": states.tolist(),
    }
    return surface, evidence, len(frames)


def prove(geometry_path):
    import yaml
    document = yaml.safe_load(open(geometry_path).read())
    hull = json.loads(document["descriptor_json"])
    surface, evidence, n_frames = _pipeline(hull, gen_sfree.ORIGINS_YZ)
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
              "frames": n_frames, "checks": {}}

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
    mapper = _load_mapper(hull)
    for frame in gen_sfree.generate_frames(hull):
        points, origin = _world_points(frame)
        mapper.integrate_points(points, origin=origin)
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

    # C. per-candidate triple binding in the PRODUCT call shape
    from luggage_packing import placement_solver as ps
    solver_surface = dict(surface)
    solver_surface["geometry_descriptor"] = dict(hull)
    params = dict(ps._default_params())
    params["top_n"] = 8000
    params["keep_rejected"] = 50000
    verifier = ps.make_volume_verifier(evidence)
    guarded = ps.generate_candidates(
        solver_surface, list(PROBE), allowed_yaws=[0.0, math.pi / 2.0],
        params=params, volume_verifier=verifier)
    plain = ps.generate_candidates(
        solver_surface, list(PROBE), allowed_yaws=[0.0, math.pi / 2.0],
        params=params)

    def sensor_stack(candidates):
        return [c for c in candidates
                if c.get("feasible") and c.get("support_source") == "sensor"]

    inner_h = float(hull["ceiling_z"]) - floor
    oracle_ok = voxel_ok = 0
    failures = []
    first = None
    authorized = sensor_stack(guarded)
    for c in authorized:
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
        volume_pass = ps.volume_state_in_evidence(
            evidence, center, c["footprint"],
            center[2] - PROBE[2] * 0.5, center[2] + PROBE[2] * 0.5) is None
        if oracle_pass:
            oracle_ok += 1
        if volume_pass:
            voxel_ok += 1
        if not c.get("volume_verified"):
            failures.append({"center": [round(v, 4) for v in center],
                             "missing_volume_verified": True})
        if oracle_pass and volume_pass and first is None:
            first = {"center": [round(v, 4) for v in center],
                     "support_source": c["support_source"],
                     "confidence": c["confidence_ratio"],
                     "volume_cells": int(PROBE[0] / RES)
                     * int(PROBE[1] / RES)
                     * int(PROBE[2] / RES)}
        if (not oracle_pass or not volume_pass) and len(failures) < 3:
            failures.append({"center": [round(v, 4) for v in center],
                             "oracle_pass": oracle_pass,
                             "voxel_pass": volume_pass})
    plain_sensor = sensor_stack(plain)
    report["checks"]["C_legal_candidate_on_observed_support"] = {
        "pass": (bool(authorized) and len(authorized) == oracle_ok
                 == voxel_ok and len(authorized) == len(plain_sensor)
                 and not failures),
        "surface_offers": len(plain_sensor),
        "authorized_with_verifier": len(authorized),
        "oracle_verified": oracle_ok,
        "voxel_volume_verified": voxel_ok,
        "voxels_verified": len(authorized) * int(PROBE[0] / RES)
        * int(PROBE[1] / RES) * int(PROBE[2] / RES),
        "failures": failures, "example": first}

    # N. negative control: single-origin subset — the surface still
    # offers stacking candidates; the product gate drops exactly the
    # ones whose volumes hold unobserved voxels (external agreement).
    neg_surface, neg_evidence, neg_frames = _pipeline(
        hull, NEGATIVE_ORIGINS)
    neg_solver_surface = dict(neg_surface)
    neg_solver_surface["geometry_descriptor"] = dict(hull)
    neg_plain = ps.generate_candidates(
        neg_solver_surface, list(PROBE),
        allowed_yaws=[0.0, math.pi / 2.0], params=params)
    neg_guarded = ps.generate_candidates(
        neg_solver_surface, list(PROBE), allowed_yaws=[0.0, math.pi / 2.0],
        params=params, volume_verifier=ps.make_volume_verifier(neg_evidence))
    neg_offers = [c for c in sensor_stack(neg_plain)
                  if c["center_container"][2] > floor + 0.15]
    neg_authorized = [c for c in sensor_stack(neg_guarded)
                      if c["center_container"][2] > floor + 0.15]
    neg_auth_ids = {c["candidate_id"] for c in neg_authorized}
    neg_dropped = [c for c in neg_offers
                   if c["candidate_id"] not in neg_auth_ids]

    def neg_external(c):
        center = [float(v) for v in c["center_container"]]
        return ps.volume_state_in_evidence(
            neg_evidence, center, c["footprint"],
            center[2] - PROBE[2] * 0.5, center[2] + PROBE[2] * 0.5)

    agree_auth = all(neg_external(c) is None for c in neg_authorized)
    agree_drop = all(neg_external(c) is not None for c in neg_dropped)
    report["checks"]["N_unknown_above_support_rejects"] = {
        "pass": (bool(neg_offers) and bool(neg_authorized)
                 and bool(neg_dropped) and agree_auth and agree_drop),
        "frames": neg_frames,
        "surface_offers": len(neg_offers),
        "authorized_with_verifier": len(neg_authorized),
        "rejected_by_gate": len(neg_dropped),
        "external_agreement_authorized": agree_auth,
        "external_agreement_dropped": agree_drop}

    # T. tolerance unification binding (reviews 2026-11-08_1751 f.3)
    report["checks"]["T_frozen_tolerance_unification"] = {
        "pass": (ps.PENETRATION_EPS_M == ORACLE_POLICY["float_tolerance_m"]
                 and ps.OVERLAP_MIN_M == _OVERLAP_MIN_M),
        "solver_penetration_eps": ps.PENETRATION_EPS_M,
        "oracle_float_tolerance_m": ORACLE_POLICY["float_tolerance_m"],
        "solver_overlap_min_m": ps.OVERLAP_MIN_M,
        "oracle_overlap_min_m": _OVERLAP_MIN_M}

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
