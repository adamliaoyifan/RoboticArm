#!/usr/bin/env python3
"""Pre-gate v2 for the A0/A'/B control: bidirectional check of A' and B
against the FROZEN COMMON BASELINE (design_baseline.py), plus A0's
deviation map for the record.

  Check 1 (baseline -> asset): grids on every baseline cavity face
  (floor, ceiling, -Y wall, +Y vertical, +Y chamfer slope, -X aperture
  frame, +X end) must lie within 0.5 mm of the asset — missing faces
  and gaps are grid cells beyond threshold.
  Check 2 (asset -> baseline): the asset's own surface samples must not
  protrude into the cavity beyond 0.5 mm (samples may dip at most
  0.5 mm past a baseline plane).
  A0: deviation map only (it is the production baseline, kept as-is).

0.5 mm is the experiment-control threshold, not a product margin.
"""
import argparse
import json
import os
import sys

import numpy as np
import open3d as o3d
import trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from design_baseline import (load_baseline, chamfer_y, inside_cavity,  # noqa
                             panels)
from build_aprim import wedge_box  # noqa: E402

LIMIT = 0.0005


def baseline_face_grids(b, step=0.02):
    """Grid points ON the baseline cavity faces (the contact region)."""
    grids = {}
    x_lo, x_hi, y_lo, y_hi = b["x_lo"], b["x_hi"], b["y_lo"], b["y_hi"]
    fz, cz = b["floor_z"], b["ceiling_z"]
    wz = b["chamfer_wall_z"]

    def grid(axis, plane_v, a_lo, a_hi, c_lo, c_hi):
        a = np.arange(a_lo + step, a_hi - step, step)
        c = np.arange(c_lo + step, c_hi - step, step)
        pts = []
        for u in a:
            for v in c:
                p = {axis: plane_v}
                others = [ax for ax in "xyz" if ax != axis]
                p[others[0]], p[others[1]] = float(u), float(v)
                pts.append([p["x"], p["y"], p["z"]])
        return np.array(pts)

    grids["floor"] = grid("z", fz, x_lo, x_hi, y_lo, y_hi)
    grids["ceiling"] = grid("z", cz, x_lo, x_hi, y_lo, b["chamfer_floor_y"])
    grids["wall_neg_y"] = grid("y", y_lo, x_lo, x_hi, fz, cz)
    grids["wall_pos_y_vertical"] = grid("y", y_hi, x_lo, x_hi, fz, wz)
    # Chamfer slope: parametrize by z, y = chamfer_y(z).
    pts = []
    for z in np.arange(wz + step, cz - step, step):
        y = chamfer_y(b, z)
        for x in np.arange(x_lo + step, x_hi - step, step):
            pts.append([x, y, z])
    grids["chamfer_slope"] = np.array(pts)
    # Aperture frame inner faces (the tunnel walls through the -X panel).
    ap = b["aperture"]
    for name, (axis, plane_v, a_lo, a_hi, c_lo, c_hi) in {
        "ap_sill_top": ("z", ap["z_lo"], ap["y_lo"], ap["y_hi"],
                        b["o_x_lo"], x_lo),
        "ap_lintel_bottom": ("z", ap["z_hi"], ap["y_lo"], ap["y_hi"],
                             b["o_x_lo"], x_lo),
        "ap_side_neg_y": ("y", ap["y_lo"], b["o_x_lo"], x_lo,
                          ap["z_lo"], ap["z_hi"]),
        "ap_side_pos_y": ("y", ap["y_hi"], b["o_x_lo"], x_lo,
                          ap["z_lo"], ap["z_hi"]),
    }.items():
        grids[name] = grid(axis, plane_v, a_lo, a_hi, c_lo, c_hi)
    # +X end pentagon: sample x-plane grid then keep points inside the
    # cavity cross-section, with a boundary margin so grid points never
    # straddle the slope line (a point 1 mm past the slope is wall
    # interior, not a missing face).
    end = grid("x", x_hi, y_lo, y_hi, fz, cz)
    keep = np.array([inside_cavity(b, p, eps=0.0025) for p in end])
    grids["end_pos_x"] = end[keep]
    return grids


def dist_to_mesh(scene, pts):
    pts = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
    return scene.compute_distance(
        o3d.core.Tensor(pts)).numpy()


def dist_to_b(boxes, pts):
    best = np.full(len(pts), np.inf)
    for bxx in boxes:
        c = np.array(bxx["center"])
        s = np.array(bxx["size"]) / 2
        roll = bxx.get("roll") or 0.0
        local = pts - c
        if roll:
            cr, sr = np.cos(roll), np.sin(roll)
            rot = np.array([[1, 0, 0], [0, cr, sr], [0, -sr, cr]])
            local = local.dot(rot.T)
        clamped = np.clip(local, -s, s)
        best = np.minimum(best, np.linalg.norm(local - clamped, axis=1))
    return best


def asset_inner_samples(mesh_path):
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    mesh.compute_vertex_normals()
    v = np.asarray(mesh.vertices)
    t = np.asarray(mesh.triangles)
    vn = np.asarray(mesh.vertex_normals)
    cen = v[t].mean(axis=1)
    fn = vn[t].mean(axis=1)
    fn /= (np.linalg.norm(fn, axis=1, keepdims=True) + 1e-12)
    inward = ((fn * (v.mean(axis=0) - cen)).sum(axis=1) > 0)
    return cen[inward]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--aprime-stl", required=True)
    parser.add_argument("--bprim-manifest", required=True)
    parser.add_argument("--a0-stl", default="")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    b = load_baseline()
    grids = baseline_face_grids(b)

    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(
        o3d.io.read_triangle_mesh(args.aprime_stl)))
    boxes = json.load(open(args.bprim_manifest))["bprim"]["boxes"]

    report = {"limit_mm": 0.5, "baseline_source": b["source"],
              "aprime": {}, "bprim": {}}
    for arm, dist_fn in (("aprime", lambda p: dist_to_mesh(scene, p)),
                         ("bprim", lambda p: dist_to_b(boxes, p))):
        per_face = {}
        worst = 0.0
        for name, pts in grids.items():
            if len(pts) == 0:
                per_face[name] = {"points": 0, "max_mm": None,
                                  "beyond_0p5mm": 0}
                continue
            d = dist_fn(pts)
            per_face[name] = {
                "points": int(len(pts)),
                "max_mm": round(float(d.max()) * 1000, 3),
                "beyond_0p5mm": int((d > LIMIT).sum()),
            }
            worst = max(worst, float(d.max()))
        report[arm] = {"faces": per_face,
                       "pass": bool(worst <= 0.0005 + 1e-9)}

    # Check 2: asset surfaces must not protrude into the cavity.
    for arm, mesh_path in (("aprime", args.aprime_stl),):
        samples = asset_inner_samples(mesh_path)
        inside_deep = np.array([
            inside_cavity(b, p, eps=0.0005) for p in samples])
        report[arm]["protrusion_samples"] = int(len(samples))
        report[arm]["protruding_beyond_0p5mm"] = int(inside_deep.sum())
        report[arm]["pass"] = bool(report[arm]["pass"]
                                   and inside_deep.sum() == 0)

    # B protrusion: sample the box faces (rotated wedge included).
    b_pts = []
    for bxx in boxes:
        c = np.array(bxx["center"])
        s = np.array(bxx["size"]) / 2
        roll = bxx.get("roll") or 0.0
        for ax in range(3):
            for sign in (-1, 1):
                face = np.array(np.meshgrid(
                    *[np.linspace(-s[i], s[i], 9) if i != ax
                      else [sign * s[ax]] for i in range(3)])).reshape(
                        3, -1).T
                if roll:
                    cr, sr = np.cos(roll), np.sin(roll)
                    rot = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
                    face = face.dot(rot.T)
                b_pts.append(face + c)
    b_pts = np.concatenate(b_pts)
    b_deep = np.array([inside_cavity(b, p, eps=0.0005) for p in b_pts])
    report["bprim"]["protrusion_samples"] = int(len(b_pts))
    report["bprim"]["protruding_beyond_0p5mm"] = int(b_deep.sum())
    report["bprim"]["pass"] = bool(report["bprim"]["pass"]
                                   and b_deep.sum() == 0)

    # A0 deviation map (record only).
    if args.a0_stl and os.path.isfile(args.a0_stl):
        scene0 = o3d.t.geometry.RaycastingScene()
        scene0.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(
            o3d.io.read_triangle_mesh(args.a0_stl)))
        a0 = {}
        for name, pts in grids.items():
            if len(pts) == 0:
                a0[name] = {"max_mm": None, "beyond_0p5mm": 0}
                continue
            d = dist_to_mesh(scene0, pts)
            a0[name] = {"max_mm": round(float(d.max()) * 1000, 3),
                        "beyond_0p5mm": int((d > LIMIT).sum())}
        report["a0_deviation_map"] = a0

    report["pass"] = bool(report["aprime"]["pass"]
                          and report["bprim"]["pass"])
    with open(args.out, "w") as handle:
        json.dump(report, handle, indent=2)
    print(json.dumps({"aprime_pass": report["aprime"]["pass"],
                      "bprim_pass": report["bprim"]["pass"],
                      "pass": report["pass"]}))
    for arm in ("aprime", "bprim"):
        worst = max((f["max_mm"] or 0.0)
                    for f in report[arm]["faces"].values())
        print("%s worst face deviation: %.3f mm" % (arm, worst))
    raise SystemExit(0 if report["pass"] else 1)


if __name__ == "__main__":
    main()
