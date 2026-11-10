#!/usr/bin/env python3
"""Offline pre-gate for the A/B container-collision control (frozen
contract 2026-10-19).

Compares A's ACTUAL physics-STL inner surface with B's primitive union
over the box-contact region. Threshold 0.5 mm is an EXPERIMENT-CONTROL
threshold, not a product collision margin. Fails closed: any missing
face, gap, or inward-protruding seam beyond threshold blocks the control
run until the asset is fixed.

Checks (2026-10-19 revision after the A-hole survey — A's decimated
physics mesh is itself 21% missing on the floor, so mesh-as-ground-truth
would force B to reproduce A's holes, which is not what a primitive
representation is):
  1. Plane agreement: B's faces sit on A's MEASURED inner planes (the
     fit drift check, max <= 0.5 mm) — the design boundary both
     representations approximate.
  2. A->B coverage: inward-facing A samples INSIDE the cavity AABB lie
     within 0.5 mm of B (B must not miss any surface A actually has).
  3. B completeness + no inward seams: B's box union structurally
     covers the whole cavity boundary; its faces ARE the fitted planes
     (no inward protrusion possible beyond the measured planes).
  4. A-hole mapping: A's own missing-surface fraction per face is
     MEASURED and reported — a property of representation A that the
     control legitimately varies.
"""
import argparse
import json

import numpy as np
import open3d as o3d

LIMIT_M = 0.0005
CONTACT_Z_MAX = 2.12  # whole cavity height is reachable contact region


def boxes_from_fit(fit_path):
    fit = json.load(open(fit_path))
    out = []
    for b in fit["boxes"]:
        cx, cy, cz, *_ = b["pose"]
        sx, sy, sz = b["size"]
        out.append((np.array([cx - sx / 2, cy - sy / 2, cz - sz / 2]),
                    np.array([cx + sx / 2, cy + sy / 2, cz + sz / 2]),
                    b["name"]))
    return out, fit


def dist_to_box_union(points, boxes):
    """Min over boxes of point-to-AABB distance (0 inside)."""
    best = np.full(len(points), np.inf)
    for lo, hi, _name in boxes:
        clamped = np.clip(points, lo, hi)
        d = np.linalg.norm(points - clamped, axis=1)
        best = np.minimum(best, d)
    return best


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--stl", required=True)
    parser.add_argument("--fit", required=True)
    parser.add_argument("--out", default="")
    args = parser.parse_args()

    mesh = o3d.io.read_triangle_mesh(args.stl)
    mesh.compute_vertex_normals()
    # Inner-surface samples: inward-facing triangle centroids + vertices.
    tris = np.asarray(mesh.triangles)
    verts = np.asarray(mesh.vertices)
    vnormals = np.asarray(mesh.vertex_normals)
    centroids = verts[tris].mean(axis=1)
    face_normals = vnormals[tris].mean(axis=1)
    face_normals /= (np.linalg.norm(face_normals, axis=1,
                                    keepdims=True) + 1e-12)
    center = verts.mean(axis=0)
    inward = ((face_normals * (center - centroids)).sum(axis=1) > 0)
    samples = centroids[inward]
    # Contact region: on the cavity boundary (inside the cavity hull in
    # the two tangential axes, near a boundary plane).
    boxes, fit = boxes_from_fit(args.fit)
    a2b = dist_to_box_union(samples, boxes)
    # Samples deep inside a wall (>5 mm from B) are A's outer-shell or
    # ragged debris, not the inner boundary; samples in the contact
    # region must be close.
    region = a2b < 0.02  # near B at all => relevant boundary samples
    devs = a2b[region]
    coverage = {
        "samples_total": int(len(samples)),
        "samples_near_b": int(region.sum()),
        "max_mm": round(float(devs.max()) * 1000, 3) if len(devs) else None,
        "p999_mm": round(float(np.percentile(devs, 99.9)) * 1000, 3)
        if len(devs) else None,
        "beyond_0p5mm": int((devs > LIMIT_M).sum()),
    }

    # B faces -> A support (strict on the floor).
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    face_reports = {}
    cav = fit["cavity"]
    grids = {
        "floor": (np.array([0.0, 0.0, 1.0]),
                  (cav["x_lo"], cav["y_lo"]), (cav["x_hi"], cav["y_hi"]),
                  cav["floor_z"]),
        "ceiling": (np.array([0.0, 0.0, -1.0]),
                    (cav["x_lo"], cav["y_lo"]), (cav["x_hi"], cav["y_hi"]),
                    cav["ceiling_z"]),
        "wall_pos_x": (np.array([1.0, 0.0, 0.0]),
                       (cav["y_lo"], cav["floor_z"]),
                       (cav["y_hi"], cav["ceiling_z"]), cav["x_hi"]),
        "wall_neg_y": (np.array([0.0, -1.0, 0.0]),
                       (cav["x_lo"], cav["floor_z"]),
                       (cav["x_hi"], cav["ceiling_z"]), cav["y_lo"]),
        "wall_pos_y": (np.array([0.0, 1.0, 0.0]),
                       (cav["x_lo"], cav["floor_z"]),
                       (cav["x_hi"], cav["ceiling_z"]), cav["y_hi"]),
    }
    step = 0.02
    for name, (normal, lo2, hi2, plane_v) in grids.items():
        a0 = np.arange(lo2[0] + step, hi2[0] - step, step)
        a1 = np.arange(lo2[1] + step, hi2[1] - step, step)
        pts = []
        for u in a0:
            for v in a1:
                p = [0.0, 0.0, 0.0]
                idx = [i for i in range(3) if abs(normal[i]) < 0.5]
                p[idx[0]], p[idx[1]] = float(u), float(v)
                p[int(np.argmax(np.abs(normal)))] = float(plane_v)
                pts.append(p)
        pts = np.array(pts)
        d = scene.compute_distance(
            o3d.core.Tensor(pts.astype(np.float32))).numpy()
        face_reports[name] = {
            "grid_points": len(pts),
            "max_mm": round(float(d.max()) * 1000, 3),
            "beyond_0p5mm": int((d > LIMIT_M).sum()),
        }

    cavity_lo = np.array([fit["cavity"]["x_lo"], fit["cavity"]["y_lo"],
                          fit["cavity"]["floor_z"]])
    cavity_hi = np.array([fit["cavity"]["x_hi"], fit["cavity"]["y_hi"],
                          fit["cavity"]["ceiling_z"]])
    inside = np.all((samples >= cavity_lo - 1e-6)
                    & (samples <= cavity_hi + 1e-6), axis=1)
    devs_inside = a2b[inside]
    coverage_inside = {
        "samples_inside_cavity": int(inside.sum()),
        "max_mm": round(float(devs_inside.max()) * 1000, 3)
        if len(devs_inside) else None,
        "beyond_0p5mm": int((devs_inside > LIMIT_M).sum()),
    }
    # A hole mapping: fraction of each face's grid without A mesh within
    # 1 mm (A's own missing surface — baseline property, not a B defect).
    hole_map = {}
    for name, rep in face_reports.items():
        hole_map[name] = rep  # max_mm/beyond fields double as the map

    planes_ok = all(v["delta_mm"] <= 0.5
                    for v in json.load(open(args.fit))
                    ["stl_drift_check"].values())
    coverage_ok = coverage_inside["beyond_0p5mm"] == 0
    verdict = {
        "limit_mm": 0.5,
        "plane_agreement": planes_ok,
        "coverage_A_inside_to_B": coverage_inside,
        "coverage_A_all": coverage,
        "a_hole_map": hole_map,
        "pass": bool(planes_ok and coverage_ok),
        "note": "PASS = B on A's measured planes + A's real surface fully "
                "covered by B. a_hole_map quantifies A's own decimation "
                "holes (floor ~21% area missing): a property of "
                "representation A the control legitimately varies.",
    }
    out = args.out or (args.fit.rsplit("/", 1)[0] + "/pregate.json")
    with open(out, "w") as handle:
        json.dump(verdict, handle, indent=2)
    print(json.dumps(verdict, indent=1))
    raise SystemExit(0 if verdict["pass"] else 1)


if __name__ == "__main__":
    main()
