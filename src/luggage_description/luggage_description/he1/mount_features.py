"""Label arm_realsense_v1.3.stl holes in eef_mount_adapter millimetres."""

from __future__ import division

from collections import defaultdict

import numpy as np
import trimesh

from luggage_description.he1.geometry import as_unit, fit_cylinder_from_points, fit_plane


def load_mount_mesh(path):
    mesh = trimesh.load(path, force="mesh")
    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError("expected a triangle mesh at %s" % path)
    return mesh


def _boundary_loops(mesh, face_idx):
    edges = []
    for fi in face_idx:
        verts = mesh.faces[int(fi)]
        for a, b in ((verts[0], verts[1]), (verts[1], verts[2]), (verts[2], verts[0])):
            edges.append(tuple(sorted((int(a), int(b)))))
    counts = defaultdict(int)
    for edge in edges:
        counts[edge] += 1
    boundary = [edge for edge, count in counts.items() if count == 1]
    adj = defaultdict(list)
    for a, b in boundary:
        adj[a].append(b)
        adj[b].append(a)
    seen = set()
    loops = []
    for start in adj:
        if start in seen:
            continue
        loop = [start]
        seen.add(start)
        prev = None
        cur = start
        while True:
            nxts = adj[cur]
            nxt = None
            for cand in nxts:
                if cand != prev:
                    nxt = cand
                    break
            if nxt is None or nxt == start:
                break
            if nxt in seen:
                break
            loop.append(nxt)
            seen.add(nxt)
            prev, cur = cur, nxt
        loops.append(loop)
    return loops


def _faces_for_normal(mesh, target, tol=0.12):
    target = as_unit(target)
    dots = mesh.face_normals.dot(target)
    return np.where(np.abs(np.abs(dots) - 1.0) < tol)[0]


def circular_loops(mesh, normal, radius_lo=0.8, radius_hi=6.0, circular_std=0.2):
    faces = _faces_for_normal(mesh, normal)
    loops = []
    for loop in _boundary_loops(mesh, faces):
        if len(loop) < 6:
            continue
        pts = mesh.vertices[loop]
        centre = pts.mean(axis=0)
        radii = np.linalg.norm(pts - centre, axis=1)
        mean_r = float(radii.mean())
        std_r = float(radii.std())
        if mean_r < radius_lo or mean_r > radius_hi:
            continue
        if std_r > circular_std * max(mean_r, 1e-6):
            continue
        loops.append({
            "centre": centre,
            "radius": mean_r,
            "std": std_r,
            "points": pts,
            "normal": as_unit(normal),
            "support": len(loop),
        })
    return loops


def _cluster_loops(loops, tol=3.0):
    used = [False] * len(loops)
    groups = []
    for i, loop in enumerate(loops):
        if used[i]:
            continue
        group = [loop]
        used[i] = True
        for j in range(i + 1, len(loops)):
            if used[j]:
                continue
            if np.linalg.norm(loops[j]["centre"] - loop["centre"]) <= tol:
                group.append(loops[j])
                used[j] = True
        groups.append(group)
    return groups


def _hole_from_group(mesh, group):
    centres = np.vstack([item["centre"] for item in group])
    pts = np.vstack([item["points"] for item in group])
    axis = None
    if len(group) >= 2:
        delta = group[-1]["centre"] - group[0]["centre"]
        if np.linalg.norm(delta) > 1e-6:
            axis = as_unit(delta)
    if axis is None:
        axis = as_unit(group[0]["normal"])
    # Wall vertices: near the mean radius around the axis.
    fitted = fit_cylinder_from_points(pts, axis)
    # Refine with nearby mesh vertices.
    centre = fitted["centre"]
    nearby = mesh.vertices[
        np.linalg.norm(mesh.vertices - centre, axis=1) < fitted["radius"] + 2.5
    ]
    if len(nearby) >= 12:
        fitted = fit_cylinder_from_points(nearby, axis)
    fitted["loop_support"] = int(sum(item["support"] for item in group))
    fitted["loop_count"] = len(group)
    fitted["centroid_mean"] = centres.mean(axis=0)
    return fitted


def _dedupe_xyz(holes, tol=3.0):
    uniq = []
    for hole in holes:
        centre = np.asarray(hole["centre"], dtype=np.float64)
        matched = False
        for i, other in enumerate(uniq):
            if np.linalg.norm(centre - other["centre"]) <= tol:
                if hole.get("support", 0) > other.get("support", 0):
                    uniq[i] = hole
                matched = True
                break
        if not matched:
            uniq.append(hole)
    return uniq


def _merge_through_holes(holes, xy_tol=3.5, max_axial_mm=18.0):
    """Merge entry/exit of one through-hole. Mid360 corners share XY but are ~48 mm apart."""
    remaining = list(holes)
    merged = []
    while remaining:
        seed = remaining.pop(0)
        group = [seed]
        rest = []
        seed_xy = np.asarray(seed["centre"][:2], dtype=np.float64)
        for item in remaining:
            dxy = float(np.linalg.norm(np.asarray(item["centre"][:2]) - seed_xy))
            d3 = float(np.linalg.norm(np.asarray(item["centre"]) - np.asarray(seed["centre"])))
            if dxy <= xy_tol and d3 <= max_axial_mm:
                group.append(item)
            else:
                rest.append(item)
        remaining = rest
        centres = np.vstack([np.asarray(item["centre"], dtype=np.float64) for item in group])
        best = min(group, key=lambda item: item["radius"])
        hole = dict(best)
        hole["centre"] = centres.mean(axis=0)
        hole["loop_count"] = int(sum(item.get("loop_count", 1) for item in group))
        hole["support"] = int(sum(item.get("support", 0) for item in group))
        if len(group) >= 2:
            delta = centres[-1] - centres[0]
            if np.linalg.norm(delta) > 1e-6:
                hole["axis"] = as_unit(delta)
        merged.append(hole)
    return merged


def _intersect_axis_plane(origin, axis, plane_c, plane_n):
    origin = np.asarray(origin, dtype=np.float64)
    axis = as_unit(axis)
    plane_c = np.asarray(plane_c, dtype=np.float64)
    plane_n = as_unit(plane_n)
    denom = float(axis.dot(plane_n))
    if abs(denom) < 1e-9:
        return origin
    t = float((plane_c - origin).dot(plane_n) / denom)
    return origin + t * axis


def _fit_bar_seating(mesh):
    """Mating face on the camera-outward side of the inclined bar."""
    target = as_unit(np.array([0.0, 0.26, -0.97]))
    dots = mesh.face_normals.dot(target)
    faces = np.where(dots > 0.92)[0]
    centres = mesh.triangles_center[faces]
    keep = (
        (centres[:, 1] > 70.0)
        & (centres[:, 1] < 95.0)
        & ((centres[:, 0] < -15.0) | (centres[:, 0] > 55.0))
    )
    faces = faces[keep]
    if len(faces) < 8:
        raise RuntimeError("bar seating faces not distinguished")
    points = mesh.triangles[faces].reshape(-1, 3)
    centroid, normal, rms, support = fit_plane(points)
    if float(normal.dot(target)) < 0.0:
        normal = -normal
    return {
        "id": "STL_BAR_SEAT",
        "centroid": centroid,
        "normal": normal,
        "rms": rms,
        "support": support,
        "source": "least-squares fit of camera-outward bar faces",
    }


def extract_mount_features(mesh):
    inventory = {
        "triangle_count": int(len(mesh.faces)),
        "vertex_count": int(len(mesh.vertices)),
        "watertight": bool(mesh.is_watertight),
        "bounds_mm": mesh.bounds.tolist(),
        "extents_mm": mesh.extents.tolist(),
        "units": "millimetres",
        "connected_components": 1,
    }
    y_loops = circular_loops(mesh, (0.0, 1.0, 0.0), 0.9, 5.0, 0.25)
    z_loops = circular_loops(mesh, (0.0, 0.0, 1.0), 0.9, 5.0, 0.25)
    inclined_loops = circular_loops(mesh, (0.0, -0.26, 0.97), 0.9, 5.0, 0.35)
    all_loops = y_loops + z_loops + inclined_loops

    bar_candidates = []
    eef_candidates = []
    mid360_candidates = []
    for group in _cluster_loops(all_loops, tol=4.0):
        try:
            hole = _hole_from_group(mesh, group)
        except (ValueError, np.linalg.LinAlgError):
            continue
        c = hole["centre"]
        r_loop = float(np.mean([item["radius"] for item in group]))
        hole["loop_radius"] = r_loop
        r = min(float(hole["radius"]), r_loop)
        if c[1] > 95.0 and 1.0 <= r <= 2.2 and 0.0 <= c[0] <= 45.0:
            mid360_candidates.append(hole)
        elif c[1] > 70.0 and r <= 2.2 and (c[0] < 0.0 or c[0] > 55.0):
            bar_candidates.append(hole)
        elif c[1] < 5.0 and 1.2 <= r <= 2.2 and 5.0 < c[0] < 40.0:
            eef_candidates.append(hole)

    bar = _merge_through_holes(_dedupe_xyz(bar_candidates, 3.0), 3.5, 18.0)
    eef = _merge_through_holes(_dedupe_xyz(eef_candidates, 3.0), 3.5, 18.0)
    mid = _dedupe_xyz(mid360_candidates, 6.0)
    bar = sorted(bar, key=lambda h: h["centre"][0])
    eef = sorted(eef, key=lambda h: h["centre"][0])
    mid = sorted(mid, key=lambda h: (h["centre"][0], h["centre"][2]))

    if len(bar) != 4:
        raise RuntimeError("expected 4 bar holes, found %d" % len(bar))
    if len(eef) < 2:
        raise RuntimeError("EEF two-hole interface not distinguished")
    if len(mid) < 4:
        raise RuntimeError("Mid360 hole set not distinguished")

    seating = _fit_bar_seating(mesh)
    for i, hole in enumerate(bar):
        hole["id"] = "STL_BAR_H%d" % i
        hole["cylinder_centre"] = np.asarray(hole["centre"], dtype=np.float64)
        hole["centre"] = _intersect_axis_plane(
            hole["centre"], hole["axis"], seating["centroid"], seating["normal"])
    for i, hole in enumerate(eef[:2]):
        hole["id"] = "STL_EEF_H%d" % i
    for i, hole in enumerate(mid[:4]):
        hole["id"] = "STL_MID360_H%d" % i

    mid_pts = np.vstack([h["centre"] for h in mid[:4]])
    pocket = {
        "id": "STL_MID360_POCKET",
        "centroid": mid_pts.mean(axis=0),
        "source": "centroid of four Mid360 pad holes",
    }

    return {
        "mesh": inventory,
        "bar_holes": bar,
        "eef_holes": eef[:2],
        "mid360_holes": mid[:4],
        "mid360_pocket": pocket,
        "bar_seating": seating,
        "distinguished": True,
    }
