"""Eval-only table-patch crop + ICP between D555 depth and Livox.

Frozen revision: ``config/livox_d555_table_icp_frozen.yaml``
(``2026-09-14_br_corner_and_sides``). In one common frame: take the
highest-mean-height camera patch (the table), keep Livox points near that
patch, then yaw+translation ICP Livox onto D555 using the BR corner plus
the bottom and right edges. Does not write URDF or touch online nodes.
"""
from __future__ import division

import json
import os

import numpy as np
from scipy.spatial import ConvexHull, cKDTree

from luggage_description.livox_d555_align import (
    nn_rmse,
    rotation_deg,
    transform_points,
    translation_m,
    voxel_downsample,
)
from luggage_perception.eval.gate4_dump import write_ply_xyz
from luggage_perception.eval.lidar_camera_calib_export import (
    ransac_plane,
    write_ply_xyzrgb,
)

FROZEN_TABLE_ICP_YAML = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "config", "livox_d555_table_icp_frozen.yaml"))
FROZEN_REVISION = "2026-09-14_br_corner_and_sides"


def load_frozen_table_icp(path=None):
    """Load the frozen table-ICP knobs and accepted TF. Eval-only."""
    import yaml
    cfg_path = path or FROZEN_TABLE_ICP_YAML
    with open(cfg_path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict) or data.get("revision") != FROZEN_REVISION:
        raise ValueError("unexpected frozen table ICP revision in %s" % cfg_path)
    return data


def _jsonable(obj):
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (np.floating, np.integer)):
        return obj.item()
    if isinstance(obj, dict):
        return {str(k): _jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_jsonable(v) for v in obj]
    return obj


def read_ply_points(path):
    """Read binary little-endian XYZ or XYZRGB PLY. Returns (xyz, rgb_or_none)."""
    with open(path, "rb") as handle:
        header = []
        while True:
            line = handle.readline()
            if not line:
                raise ValueError("truncated PLY header: %s" % path)
            header.append(line)
            if line.strip() == b"end_header":
                break
        n = None
        props = []
        fmt = ""
        for raw in header:
            parts = raw.decode("ascii", "replace").strip().split()
            if not parts:
                continue
            if parts[0] == "format":
                fmt = " ".join(parts[1:])
            elif parts[:2] == ["element", "vertex"]:
                n = int(parts[2])
            elif parts[0] == "property" and parts[1] != "list":
                props.append((parts[1], parts[2]))
        if n is None:
            raise ValueError("no vertex count in %s" % path)
        if not fmt.startswith("binary_little_endian"):
            raise ValueError("only binary_little_endian PLY: %s" % path)
        dtype_map = {
            "float": "<f4", "float32": "<f4",
            "double": "<f8", "float64": "<f8",
            "uchar": "u1", "uint8": "u1",
            "char": "i1", "int8": "i1",
            "ushort": "<u2", "uint16": "<u2",
            "short": "<i2", "int16": "<i2",
            "uint": "<u4", "uint32": "<u4",
            "int": "<i4", "int32": "<i4",
        }
        descr = []
        for ptype, name in props:
            if ptype not in dtype_map:
                raise ValueError("unsupported PLY property %s in %s" % (ptype, path))
            descr.append((name, dtype_map[ptype]))
        rec = np.frombuffer(handle.read(), dtype=np.dtype(descr), count=n)
    xyz = np.column_stack([rec["x"], rec["y"], rec["z"]]).astype(np.float64)
    rgb = None
    if set(("red", "green", "blue")).issubset(rec.dtype.names):
        rgb = np.column_stack([rec["red"], rec["green"], rec["blue"]]).astype(
            np.uint8)
    return xyz, rgb


LABEL_INTERIOR = 0
LABEL_EDGE = 1
LABEL_CORNER = 2
LABEL_BOTTOM_EDGE = 3
LABEL_RIGHT_EDGE = 4
ICP_CHANNELS = (LABEL_CORNER, LABEL_BOTTOM_EDGE, LABEL_RIGHT_EDGE)


def voxel_downsample_weighted(points, weights, voxel):
    """Keep the highest-weight point in each voxel so corners survive."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    w = np.asarray(weights, dtype=np.float64).reshape(-1)
    if pts.shape[0] == 0:
        return pts, w
    if voxel <= 0 or pts.shape[0] == 1:
        return pts, w[:len(pts)]
    keys = np.floor(pts / float(voxel)).astype(np.int64)
    span = np.maximum(keys.max(axis=0) - keys.min(axis=0) + 1, 1)
    packed = (
        (keys[:, 0] - keys[:, 0].min())
        + (keys[:, 1] - keys[:, 1].min()) * span[0]
        + (keys[:, 2] - keys[:, 2].min()) * span[0] * span[1]
    )
    order = np.argsort(-w, kind="mergesort")
    _, first = np.unique(packed[order], return_index=True)
    idx = np.sort(order[first])
    return pts[idx], w[idx]


def _se2_weighted(p, q, weights, allow_yaw=True):
    """Yaw+XY from weighted 2D correspondences. p,q are Nx3."""
    w = np.asarray(weights, dtype=np.float64).reshape(-1)
    w = np.maximum(w, 1e-9)
    w = w / w.sum()
    pc = (w[:, None] * p[:, :2]).sum(axis=0)
    qc = (w[:, None] * q[:, :2]).sum(axis=0)
    tz = float(np.sum(w * (q[:, 2] - p[:, 2])))
    step = np.eye(4)
    if not allow_yaw:
        t2 = qc - pc
        r2 = np.eye(2)
        step[0, 3] = t2[0]
        step[1, 3] = t2[1]
        step[2, 3] = tz
        return step, t2, r2, tz
    h = ((p[:, :2] - pc) * w[:, None]).T.dot(q[:, :2] - qc)
    u, _s, vt = np.linalg.svd(h)
    r2 = vt.T.dot(u.T)
    if np.linalg.det(r2) < 0:
        vt = vt.copy()
        vt[1] *= -1.0
        r2 = vt.T.dot(u.T)
    t2 = qc - r2.dot(pc)
    step[:2, :2] = r2
    step[0, 3] = t2[0]
    step[1, 3] = t2[1]
    step[2, 3] = tz
    return step, t2, r2, tz


def icp_xy_yaw(source, target, max_iter=40, max_dist=0.10, voxel=0.015,
               min_pairs=25, src_weight=None, dst_weight=None,
               corner_w=400.0, allow_yaw=False):
    """ICP with yaw + XYZ translation. Feature weights raise edge/corner score."""
    src_all = np.asarray(source, dtype=np.float64).reshape(-1, 3)
    dst_all = np.asarray(target, dtype=np.float64).reshape(-1, 3)
    if src_weight is None:
        src_weight = np.ones(len(src_all))
    if dst_weight is None:
        dst_weight = np.ones(len(dst_all))
    src0, sw = voxel_downsample_weighted(src_all, src_weight, voxel)
    dst, dw = voxel_downsample_weighted(dst_all, dst_weight, voxel)
    if src0.shape[0] < min_pairs or dst.shape[0] < min_pairs:
        src0 = np.asarray(src_all, dtype=np.float64).reshape(-1, 3)
        dst = np.asarray(dst_all, dtype=np.float64).reshape(-1, 3)
        sw = np.asarray(src_weight, dtype=np.float64).reshape(-1)[:len(src0)]
        dw = np.asarray(dst_weight, dtype=np.float64).reshape(-1)[:len(dst)]
    if src0.shape[0] < 2 or dst.shape[0] < 2:
        raise RuntimeError(
            "not enough overlap for ICP (src=%d dst=%d)"
            % (src0.shape[0], dst.shape[0]))
    tree = cKDTree(dst[:, :2])
    dst_corner = dst[dw >= 0.5 * float(corner_w)]
    tree_c = cKDTree(dst_corner[:, :2]) if len(dst_corner) >= 2 else None
    corner_src = sw >= 0.5 * float(corner_w)
    T = np.eye(4)
    last = None
    thresh = float(max_dist)
    for it in range(int(max_iter)):
        src = transform_points(T, src0)
        dist, idx = tree.query(src[:, :2], k=1)
        if tree_c is not None and np.any(corner_src):
            d2, i2 = tree_c.query(src[corner_src][:, :2], k=1)
            # prefer a corner match when it is not much farther
            use = d2 <= np.maximum(dist[corner_src] * 1.25, 0.03)
            orig = np.flatnonzero(corner_src)
            # map corner-subset index to dst rows
            corner_rows = np.flatnonzero(dw >= 0.5 * float(corner_w))
            for k, src_i in enumerate(orig):
                if not use[k]:
                    continue
                dist[src_i] = d2[k]
                idx[src_i] = corner_rows[int(i2[k])]
        keep = dist < thresh
        if int(keep.sum()) < min_pairs:
            thresh *= 1.4
            if thresh > 0.25:
                break
            continue
        p = src[keep]
        q = dst[idx[keep]]
        pair_w = sw[keep] * dw[idx[keep]]
        step, t2, r2, tz = _se2_weighted(p, q, pair_w, allow_yaw=allow_yaw)
        T = step.dot(T)
        rmse = float(np.sqrt(np.average(dist[keep] ** 2, weights=pair_w)))
        last = {
            "iter": it + 1,
            "pairs": int(keep.sum()),
            "rmse": rmse,
            "median": float(np.median(dist[keep])),
            "T": T,
            "n_src": int(src0.shape[0]),
            "n_dst": int(dst.shape[0]),
            "n_src_corner": int(corner_src.sum()),
            "n_dst_corner": int(len(dst_corner)),
        }
        yaw = float(np.arctan2(r2[1, 0], r2[0, 0]))
        if np.linalg.norm(t2) < 1e-5 and abs(yaw) < 1e-5 and abs(tz) < 1e-5:
            break
        thresh = max(0.025, 0.75 * thresh)
    if last is None:
        raise RuntimeError("ICP found no correspondences")
    return T, last


def hull_sharp_corners(points, min_turn_deg=50.0, max_keep=6):
    """Convex-hull vertices whose turning is near 90 deg."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    xy = pts[:, :2]
    empty = (np.zeros((0, 3), dtype=np.float64), np.zeros((0,), dtype=np.int64))
    if xy.shape[0] < 8:
        return empty
    try:
        hull = ConvexHull(xy)
    except Exception:
        return empty
    verts = hull.vertices
    scored = []
    n = len(verts)
    for i in range(n):
        a = xy[verts[i - 1]]
        b = xy[verts[i]]
        c = xy[verts[(i + 1) % n]]
        v1 = a - b
        v2 = c - b
        n1 = np.linalg.norm(v1)
        n2 = np.linalg.norm(v2)
        if n1 < 1e-9 or n2 < 1e-9:
            continue
        ang = float(np.degrees(np.arccos(np.clip(
            np.dot(v1 / n1, v2 / n2), -1.0, 1.0))))
        if ang < float(min_turn_deg) or ang > 160.0:
            continue
        scored.append((abs(ang - 90.0), int(verts[i]), ang))
    scored.sort(key=lambda row: row[0])
    scored = scored[:int(max_keep)]
    idx = np.array([row[1] for row in scored], dtype=np.int64)
    return pts[idx], idx


def classify_table_features(points, cell=0.012, min_cell_n=3, hull_band=0.018):
    """Label interior/edge/corner from occupancy boundary + hull turning."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    labels = np.zeros((len(pts),), dtype=np.int32)
    if len(pts) < 8:
        return labels, pts[:0]
    cell = float(cell)
    ix = np.floor(pts[:, 0] / cell).astype(np.int64)
    iy = np.floor(pts[:, 1] / cell).astype(np.int64)
    keys = np.column_stack([ix, iy])
    uniq, _inverse, counts = np.unique(
        keys, axis=0, return_inverse=True, return_counts=True)
    solid = set()
    for i, xy in enumerate(uniq):
        if int(counts[i]) >= int(min_cell_n):
            solid.add((int(xy[0]), int(xy[1])))
    if not solid:
        solid = set((int(x), int(y)) for x, y in uniq)
    edge_cells = set()
    corner_cells = set()
    for cx, cy in solid:
        east = (cx + 1, cy) in solid
        west = (cx - 1, cy) in solid
        north = (cx, cy + 1) in solid
        south = (cx, cy - 1) in solid
        n_empty = 4 - int(east) - int(west) - int(south) - int(north)
        if n_empty <= 0:
            continue
        empty_ew = (not east) and (not west)
        empty_ns = (not north) and (not south)
        if n_empty == 1 or empty_ew or empty_ns:
            edge_cells.add((cx, cy))
        else:
            corner_cells.add((cx, cy))
    xy = pts[:, :2]
    try:
        hull = ConvexHull(xy)
        verts = hull.vertices
        dmin = np.full(len(pts), np.inf)
        for i in range(len(verts)):
            a = xy[verts[i]]
            b = xy[verts[(i + 1) % len(verts)]]
            ab = b - a
            ln2 = max(float(np.dot(ab, ab)), 1e-12)
            t = np.clip((xy - a).dot(ab) / ln2, 0.0, 1.0)
            proj = a + t[:, None] * ab
            dmin = np.minimum(dmin, np.linalg.norm(xy - proj, axis=1))
        labels[dmin <= float(hull_band)] = LABEL_EDGE
    except Exception:
        pass
    for i, (x, y) in enumerate(keys):
        key = (int(x), int(y))
        if key in edge_cells:
            labels[i] = max(int(labels[i]), LABEL_EDGE)
        if key in corner_cells:
            labels[i] = LABEL_CORNER
    _corners, cidx = hull_sharp_corners(pts)
    if len(cidx):
        labels[cidx] = LABEL_CORNER
        d, _ = cKDTree(pts[cidx][:, :2]).query(xy, k=1)
        labels[d <= max(cell * 1.6, 0.02)] = LABEL_CORNER
    return labels, pts[labels == LABEL_CORNER]


def feature_weights(labels, interior=1.0, edge=12.0, corner=400.0):
    lab = np.asarray(labels, dtype=np.int32)
    w = np.full(lab.shape[0], float(interior), dtype=np.float64)
    w[lab == LABEL_EDGE] = float(edge)
    w[lab == LABEL_BOTTOM_EDGE] = float(edge)
    w[lab == LABEL_RIGHT_EDGE] = float(edge)
    w[lab == LABEL_CORNER] = float(corner)
    return w


def _hull_or_labeled_corners(points, labels=None):
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if len(pts) == 0:
        return pts
    sharp, _ = hull_sharp_corners(pts)
    if len(sharp) >= 2:
        return sharp
    if labels is not None and np.any(np.asarray(labels) == LABEL_CORNER):
        return pts[np.asarray(labels) == LABEL_CORNER]
    return pts


def visible_table_corner(points, labels=None):
    """Floor-side table corner: min-x then max-y among sharp hull corners."""
    src = _hull_or_labeled_corners(points, labels)
    if len(src) == 0:
        return None
    xy = src[:, :2]
    if len(src) >= 2:
        left2 = src[np.argsort(xy[:, 0])[:2]]
        return left2[int(np.argmax(left2[:, 1]))]
    return src[0]


def bottom_right_table_corner(points, labels=None):
    """Clearest Livox table corner: maximize x minus y (right, then down)."""
    src = _hull_or_labeled_corners(points, labels)
    if len(src) == 0:
        pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        src = pts if len(pts) else src
    if len(src) == 0:
        return None
    xy = src[:, :2]
    score = xy[:, 0] - xy[:, 1]
    return src[int(np.argmax(score))]


def focus_anchor_corners(points, labels, anchor, radius=0.025):
    """Keep CORNER only near *anchor*; other occupancy corners become EDGE."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    lab = np.asarray(labels, dtype=np.int32).copy()
    if len(pts) == 0 or anchor is None:
        return lab
    d = np.linalg.norm(pts[:, :2] - np.asarray(anchor, dtype=np.float64)[:2], axis=1)
    near = d <= float(radius)
    was_feat = (lab == LABEL_EDGE) | (lab == LABEL_CORNER)
    lab[lab == LABEL_CORNER] = LABEL_EDGE
    lab[near & was_feat] = LABEL_CORNER
    return lab


def _dist_to_polyline(xy, chain):
    """Min distance from Nx2 points to a polyline (rows of chain)."""
    pts = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    chain = np.asarray(chain, dtype=np.float64).reshape(-1, 2)
    if len(pts) == 0:
        return np.zeros((0,), dtype=np.float64)
    if len(chain) < 2:
        if len(chain) == 1:
            return np.linalg.norm(pts - chain[0], axis=1)
        return np.full(len(pts), np.inf)
    dmin = np.full(len(pts), np.inf)
    for i in range(len(chain) - 1):
        a = chain[i]
        b = chain[i + 1]
        ab = b - a
        ln2 = max(float(np.dot(ab, ab)), 1e-12)
        t = np.clip((pts - a).dot(ab) / ln2, 0.0, 1.0)
        proj = a + t[:, None] * ab
        dmin = np.minimum(dmin, np.linalg.norm(pts - proj, axis=1))
    return dmin


def _hull_turn_deg(xy, verts, i):
    n = len(verts)
    a = xy[verts[i - 1]]
    b = xy[verts[i]]
    c = xy[verts[(i + 1) % n]]
    v1 = a - b
    v2 = c - b
    n1 = np.linalg.norm(v1)
    n2 = np.linalg.norm(v2)
    if n1 < 1e-9 or n2 < 1e-9:
        return 0.0
    return float(np.degrees(np.arccos(np.clip(
        np.dot(v1 / n1, v2 / n2), -1.0, 1.0))))


def br_incident_polylines(points, br, min_turn_deg=50.0):
    """Hull chains from the BR vertex: bottom (horizontal) and right (vertical)."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    empty = np.zeros((0, 2), dtype=np.float64)
    if len(pts) < 8 or br is None:
        return empty, empty
    xy = pts[:, :2]
    br_xy = np.asarray(br, dtype=np.float64)[:2]
    try:
        hull = ConvexHull(xy)
    except Exception:
        return empty, empty
    verts = hull.vertices
    i0 = int(np.argmin(np.linalg.norm(xy[verts] - br_xy, axis=1)))
    n = len(verts)

    def walk(sign):
        idxs = [int(verts[i0])]
        for k in range(1, n - 1):
            j = (i0 + sign * k) % n
            idxs.append(int(verts[j]))
            if k >= 1 and _hull_turn_deg(xy, verts, j) >= float(min_turn_deg):
                if np.linalg.norm(xy[verts[j]] - xy[verts[i0]]) >= 0.08:
                    break
        return xy[np.array(idxs, dtype=np.int64)]

    chains = [walk(1), walk(-1)]
    bottom, right = empty, empty
    for chain in chains:
        if len(chain) < 2:
            continue
        delta = chain[-1] - chain[0]
        if abs(float(delta[0])) >= abs(float(delta[1])):
            bottom = chain
        else:
            right = chain
    return bottom, right


def label_br_incident_edges(points, labels, br, band=0.016):
    """Mark the bottom and right sides that meet at the BR corner.

    Uses occupancy edges in an L around the BR point, then adds hull-chain
    support so a slightly rotated table still counts as those two sides.
    """
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    lab = np.asarray(labels, dtype=np.int32).copy()
    if len(pts) == 0 or br is None:
        return lab
    xy = pts[:, :2]
    br_xy = np.asarray(br, dtype=np.float64)[:2]
    edgeish = (
        (lab == LABEL_EDGE)
        | (lab == LABEL_BOTTOM_EDGE)
        | (lab == LABEL_RIGHT_EDGE)
    )
    bottom_geo = (
        edgeish
        & (xy[:, 1] <= br_xy[1] + float(band))
        & (xy[:, 0] <= br_xy[0] - 0.008)
    )
    right_geo = (
        edgeish
        & (xy[:, 0] >= br_xy[0] - float(band))
        & (xy[:, 1] >= br_xy[1] + 0.008)
    )
    lab[bottom_geo & (lab != LABEL_CORNER)] = LABEL_BOTTOM_EDGE
    lab[right_geo & (lab != LABEL_CORNER)] = LABEL_RIGHT_EDGE
    bottom, right = br_incident_polylines(pts, br)
    if len(bottom) >= 2:
        near_b = _dist_to_polyline(xy, bottom) <= float(band)
        lab[near_b & edgeish & (lab != LABEL_CORNER)] = LABEL_BOTTOM_EDGE
    if len(right) >= 2:
        near_r = _dist_to_polyline(xy, right) <= float(band)
        lab[near_r & edgeish & (lab != LABEL_CORNER)] = LABEL_RIGHT_EDGE
    return lab


def _channel_weights(labels, corner_mass=4.0, edge_mass=1.5):
    """Equalize total mass per ICP channel so the long edge cannot drown yaw."""
    lab = np.asarray(labels, dtype=np.int32)
    w = np.zeros(len(lab), dtype=np.float64)
    masses = {
        LABEL_CORNER: float(corner_mass),
        LABEL_BOTTOM_EDGE: float(edge_mass),
        LABEL_RIGHT_EDGE: float(edge_mass),
    }
    for ch, mass in masses.items():
        mask = lab == ch
        n = int(mask.sum())
        if n:
            w[mask] = mass / float(n)
    return w


def icp_channel_matched(source, src_labels, target, dst_labels,
                        max_iter=40, max_dist=0.10, voxel=0.01,
                        min_pairs=8, allow_yaw=True,
                        src_weight=None, dst_weight=None):
    """ICP with same-channel correspondences (corner/bottom/right)."""
    src_all = np.asarray(source, dtype=np.float64).reshape(-1, 3)
    dst_all = np.asarray(target, dtype=np.float64).reshape(-1, 3)
    src_lab = np.asarray(src_labels, dtype=np.int32).reshape(-1)
    dst_lab = np.asarray(dst_labels, dtype=np.int32).reshape(-1)
    if src_weight is None:
        src_weight = np.ones(len(src_all))
    if dst_weight is None:
        dst_weight = np.ones(len(dst_all))
    src_chunks, sw_chunks, sl_chunks = [], [], []
    dst_chunks, dw_chunks, dl_chunks = [], [], []
    for ch in ICP_CHANNELS:
        sm = src_lab == ch
        dm = dst_lab == ch
        if int(sm.sum()) == 0 or int(dm.sum()) == 0:
            continue
        spts, sw = voxel_downsample_weighted(
            src_all[sm], np.asarray(src_weight)[sm], voxel)
        dpts, dw = voxel_downsample_weighted(
            dst_all[dm], np.asarray(dst_weight)[dm], voxel)
        if len(spts) == 0 or len(dpts) == 0:
            continue
        src_chunks.append(spts)
        sw_chunks.append(sw)
        sl_chunks.append(np.full(len(spts), ch, dtype=np.int32))
        dst_chunks.append(dpts)
        dw_chunks.append(dw)
        dl_chunks.append(np.full(len(dpts), ch, dtype=np.int32))
    if not src_chunks or not dst_chunks:
        raise RuntimeError("not enough BR corner/side points for ICP")
    src0 = np.concatenate(src_chunks, axis=0)
    sw = np.concatenate(sw_chunks, axis=0)
    sl = np.concatenate(sl_chunks, axis=0)
    dst = np.concatenate(dst_chunks, axis=0)
    dw = np.concatenate(dw_chunks, axis=0)
    dl = np.concatenate(dl_chunks, axis=0)
    trees = {}
    for ch in ICP_CHANNELS:
        rows = np.flatnonzero(dl == ch)
        if len(rows) >= 1:
            trees[ch] = (cKDTree(dst[rows, :2]), rows)
    T = np.eye(4)
    last = None
    thresh = float(max_dist)
    for it in range(int(max_iter)):
        src = transform_points(T, src0)
        dist = np.full(len(src), np.inf)
        idx = np.zeros(len(src), dtype=np.int64)
        for ch, (tree, rows) in trees.items():
            sel = sl == ch
            if not np.any(sel):
                continue
            d, j = tree.query(src[sel, :2], k=1)
            dist[sel] = d
            idx[sel] = rows[np.asarray(j, dtype=np.int64)]
        keep = dist < thresh
        if int(keep.sum()) < min_pairs:
            thresh *= 1.4
            if thresh > 0.25:
                break
            continue
        p = src[keep]
        q = dst[idx[keep]]
        pair_w = sw[keep] * dw[idx[keep]]
        step, t2, r2, tz = _se2_weighted(p, q, pair_w, allow_yaw=allow_yaw)
        T = step.dot(T)
        rmse = float(np.sqrt(np.average(dist[keep] ** 2, weights=pair_w)))
        last = {
            "iter": it + 1,
            "pairs": int(keep.sum()),
            "rmse": rmse,
            "median": float(np.median(dist[keep])),
            "T": T,
            "n_src": int(src0.shape[0]),
            "n_dst": int(dst.shape[0]),
            "n_src_corner": int((sl == LABEL_CORNER).sum()),
            "n_dst_corner": int((dl == LABEL_CORNER).sum()),
            "n_src_bottom": int((sl == LABEL_BOTTOM_EDGE).sum()),
            "n_src_right": int((sl == LABEL_RIGHT_EDGE).sum()),
            "n_dst_bottom": int((dl == LABEL_BOTTOM_EDGE).sum()),
            "n_dst_right": int((dl == LABEL_RIGHT_EDGE).sum()),
        }
        yaw = float(np.arctan2(r2[1, 0], r2[0, 0]))
        if np.linalg.norm(t2) < 1e-5 and abs(yaw) < 1e-5 and abs(tz) < 1e-5:
            break
        thresh = max(0.025, 0.75 * thresh)
    if last is None:
        raise RuntimeError("ICP found no correspondences")
    return T, last


def visible_corner_residual(cam_pts, cam_labels, lid_pts, lid_labels):
    corner = visible_table_corner(cam_pts, cam_labels)
    if corner is None or len(lid_pts) == 0:
        return {"ok": False, "residual_m": None, "camera_xyz": None, "livox_xyz": None}
    lid_c = np.asarray(lid_pts, dtype=np.float64).reshape(-1, 3)
    if lid_labels is not None and np.any(np.asarray(lid_labels) == LABEL_CORNER):
        lid_c = lid_c[np.asarray(lid_labels) == LABEL_CORNER]
    else:
        sharp, _ = hull_sharp_corners(lid_pts)
        if len(sharp):
            lid_c = sharp
    dist, idx = cKDTree(lid_c[:, :2]).query(corner[:2].reshape(1, 2), k=1)
    hit = lid_c[int(idx[0])]
    return {
        "ok": True,
        "residual_m": float(dist[0]),
        "camera_xyz": [float(v) for v in corner],
        "livox_xyz": [float(v) for v in hit],
    }


def _labeled_corner_centroid(points, labels):
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if labels is None or len(pts) == 0:
        return None
    lab = np.asarray(labels)
    if not np.any(lab == LABEL_CORNER):
        return None
    return pts[lab == LABEL_CORNER].mean(axis=0)


def br_corner_residual(cam_pts, cam_labels, lid_pts, lid_labels):
    """XY gap between bottom-right corner clusters (centroid, else hull vertex)."""
    cam_br = _labeled_corner_centroid(cam_pts, cam_labels)
    if cam_br is None:
        cam_br = bottom_right_table_corner(cam_pts, cam_labels)
    lid_br = _labeled_corner_centroid(lid_pts, lid_labels)
    if lid_br is None:
        lid_br = bottom_right_table_corner(lid_pts, lid_labels)
    if cam_br is None or lid_br is None:
        return {"ok": False, "residual_m": None, "camera_xyz": None, "livox_xyz": None}
    dist = float(np.linalg.norm(cam_br[:2] - lid_br[:2]))
    return {
        "ok": True,
        "residual_m": dist,
        "camera_xyz": [float(v) for v in cam_br],
        "livox_xyz": [float(v) for v in lid_br],
    }


def nearby_mask(points, other, radius):
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    ref = np.asarray(other, dtype=np.float64).reshape(-1, 3)
    if pts.shape[0] == 0 or ref.shape[0] == 0:
        return np.zeros((pts.shape[0],), dtype=bool)
    dist, _ = cKDTree(ref).query(pts, k=1)
    return dist < float(radius)


def hull_support_mask(points, ref, band):
    """Keep points inside the XY convex hull of ref, or within band of it."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    ref = np.asarray(ref, dtype=np.float64).reshape(-1, 3)
    if pts.shape[0] == 0:
        return np.zeros((0,), dtype=bool)
    if ref.shape[0] < 8:
        return nearby_mask(pts, ref, band)
    xy = pts[:, :2]
    rxy = ref[:, :2]
    try:
        hull = ConvexHull(rxy)
    except Exception:
        return nearby_mask(pts, ref, band)
    verts = rxy[hull.vertices]
    inside = np.zeros(len(pts), dtype=bool)
    try:
        from scipy.spatial import Delaunay
        tri = Delaunay(verts)
        inside = tri.find_simplex(xy) >= 0
    except Exception:
        pass
    dmin = np.full(len(pts), np.inf)
    hv = hull.vertices
    for i in range(len(hv)):
        a = rxy[hv[i]]
        b = rxy[hv[(i + 1) % len(hv)]]
        ab = b - a
        ln2 = max(float(np.dot(ab, ab)), 1e-12)
        t = np.clip((xy - a).dot(ab) / ln2, 0.0, 1.0)
        proj = a + t[:, None] * ab
        dmin = np.minimum(dmin, np.linalg.norm(xy - proj, axis=1))
    return inside | (dmin <= float(band))


def statistical_outlier_mask(points, k=8, std_mult=1.5):
    """Keep points whose kNN spacing is not a statistical outlier."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    n = len(pts)
    if n < 12:
        return np.ones(n, dtype=bool)
    kk = min(int(k) + 1, n)
    dist, _ = cKDTree(pts).query(pts, k=kk)
    mean_d = dist[:, 1:].mean(axis=1)
    mu = float(mean_d.mean())
    sd = float(mean_d.std())
    return mean_d <= mu + float(std_mult) * max(sd, 1e-6)


def radius_outlier_mask(points, radius=0.03, min_neighbors=3):
    """Keep points that have enough neighbors in a small radius."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if len(pts) == 0:
        return np.zeros((0,), dtype=bool)
    counts = cKDTree(pts).query_ball_point(pts, r=float(radius), return_length=True)
    return np.asarray(counts, dtype=np.int32) >= int(min_neighbors) + 1


def xy_component_labels(points, cell=0.015):
    """Integer id per XY-grid connected component (8-connected)."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    n = len(pts)
    if n == 0:
        return np.zeros((0,), dtype=np.int32)
    cell = float(cell)
    keys = [
        (int(np.floor(x / cell)), int(np.floor(y / cell)))
        for x, y in pts[:, :2]
    ]
    remaining = set(keys)
    labels = np.full(n, -1, dtype=np.int32)
    cid = 0
    while remaining:
        seed = remaining.pop()
        comp = set(_flood_from(seed, dict((k, True) for k in remaining | {seed})))
        remaining -= comp
        for i, key in enumerate(keys):
            if key in comp:
                labels[i] = cid
        cid += 1
    return labels


def supported_component_mask(lid, cam, cell=0.015, min_inside_frac=0.2):
    """Keep Livox XY components that overlap the camera hull; drop orphans."""
    lid = np.asarray(lid, dtype=np.float64).reshape(-1, 3)
    cam = np.asarray(cam, dtype=np.float64).reshape(-1, 3)
    n = len(lid)
    if n == 0:
        return np.zeros((0,), dtype=bool), 0
    inside = hull_support_mask(lid, cam, band=0.0)
    labels = xy_component_labels(lid, cell=cell)
    keep = np.zeros(n, dtype=bool)
    n_orphan = 0
    for cid in np.unique(labels):
        if cid < 0:
            continue
        mask = labels == cid
        frac = float(inside[mask].mean()) if int(mask.sum()) else 0.0
        if frac >= float(min_inside_frac):
            keep[mask] = True
        else:
            n_orphan += int(mask.sum())
    return keep, n_orphan


def reject_livox_outliers(lid, cam, inlier_radius=0.02, hull_band=0.01,
                          sor_k=8, sor_std=1.5, radius=0.03, min_neighbors=3,
                          component_cell=0.015, min_inside_frac=0.2):
    """Drop Livox that is out of camera range or isolated before ICP."""
    lid = np.asarray(lid, dtype=np.float64).reshape(-1, 3)
    cam = np.asarray(cam, dtype=np.float64).reshape(-1, 3)
    n = len(lid)
    reasons = {
        "n_input": int(n),
        "n_keep": 0,
        "n_far_from_camera": 0,
        "n_outside_hull": 0,
        "n_orphan_component": 0,
        "n_statistical": 0,
        "n_radius": 0,
        "inlier_radius_m": float(inlier_radius),
        "hull_band_m": float(hull_band),
    }
    if n == 0 or len(cam) == 0:
        return np.zeros(n, dtype=bool), reasons
    far = ~nearby_mask(lid, cam, inlier_radius)
    outside = ~hull_support_mask(lid, cam, hull_band)
    keep = ~(far | outside)
    orphan = np.zeros(n, dtype=bool)
    if int(keep.sum()) >= 8:
        supp, n_orph = supported_component_mask(
            lid[keep], cam, cell=component_cell,
            min_inside_frac=min_inside_frac)
        orphan[keep] = ~supp
        keep = keep & ~orphan
        reasons["n_orphan_component"] = int(n_orph)
    sor = np.ones(n, dtype=bool)
    rad = np.ones(n, dtype=bool)
    if int(keep.sum()) >= 12:
        sor_sub = statistical_outlier_mask(
            lid[keep], k=sor_k, std_mult=sor_std)
        rad_sub = radius_outlier_mask(
            lid[keep], radius=radius, min_neighbors=min_neighbors)
        sor[keep] = sor_sub
        rad[keep] = rad_sub
        keep = keep & sor & rad
    reasons["n_keep"] = int(keep.sum())
    reasons["n_far_from_camera"] = int(far.sum())
    reasons["n_outside_hull"] = int(outside.sum())
    reasons["n_statistical"] = int((~sor).sum())
    reasons["n_radius"] = int((~rad).sum())
    return keep, reasons


def demote_interior_corners(points, labels, band=0.022):
    """Keep corner labels only on the outer hull; interior junctions become edges."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    lab = np.asarray(labels, dtype=np.int32).copy()
    if len(pts) < 8 or not np.any(lab == LABEL_CORNER):
        return lab
    xy = pts[:, :2]
    try:
        hull = ConvexHull(xy)
    except Exception:
        return lab
    dmin = np.full(len(pts), np.inf)
    hv = hull.vertices
    for i in range(len(hv)):
        a = xy[hv[i]]
        b = xy[hv[(i + 1) % len(hv)]]
        ab = b - a
        ln2 = max(float(np.dot(ab, ab)), 1e-12)
        t = np.clip((xy - a).dot(ab) / ln2, 0.0, 1.0)
        proj = a + t[:, None] * ab
        dmin = np.minimum(dmin, np.linalg.norm(xy - proj, axis=1))
    interior = (lab == LABEL_CORNER) & (dmin > float(band))
    lab[interior] = LABEL_EDGE
    return lab


def far_livox_feature_mask(lid, lid_labels, cam, cam_labels, radius=0.04):
    """Livox edge/corner points with no nearby camera edge/corner."""
    lid = np.asarray(lid, dtype=np.float64).reshape(-1, 3)
    lab_l = np.asarray(lid_labels)
    lab_c = np.asarray(cam_labels)
    far = np.zeros(len(lid), dtype=bool)
    is_feat = (lab_l == LABEL_EDGE) | (lab_l == LABEL_CORNER)
    if not np.any(is_feat) or len(cam) == 0:
        return far
    cam_feat = cam[(lab_c == LABEL_EDGE) | (lab_c == LABEL_CORNER)]
    if len(cam_feat) == 0:
        cam_feat = cam
    dist, _ = cKDTree(cam_feat).query(lid[is_feat], k=1)
    far[np.flatnonzero(is_feat)] = dist > float(radius)
    return far


def highest_mean_height_patch(points, cell=0.04, z_band=0.03, min_cell_n=15):
    """XY-grid cells whose mean Z is within z_band of the highest occupied cell."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    empty = {
        "ok": False,
        "reason": "empty",
        "points": np.zeros((0, 3), dtype=np.float64),
        "mask": np.zeros((pts.shape[0],), dtype=bool),
        "n_cells": 0,
        "max_mean_z": None,
    }
    if pts.shape[0] < int(min_cell_n):
        empty["reason"] = "too_few_points"
        return empty
    cell = float(cell)
    ix = np.floor(pts[:, 0] / cell).astype(np.int64)
    iy = np.floor(pts[:, 1] / cell).astype(np.int64)
    keys = np.column_stack([ix, iy])
    uniq, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    means = np.zeros(len(uniq), dtype=np.float64)
    np.add.at(means, inverse, pts[:, 2])
    means = means / np.maximum(counts, 1)
    occupied = counts >= int(min_cell_n)
    if not np.any(occupied):
        empty["reason"] = "no_dense_cells"
        return empty
    max_i = int(np.argmax(np.where(occupied, means, -1e9)))
    max_mean = float(means[max_i])
    z_mask = pts[:, 2] >= (max_mean - float(z_band))
    if int(z_mask.sum()) < int(min_cell_n):
        empty["reason"] = "no_high_points"
        return empty
    cell_xy = {}
    for x, y, keep in zip(ix, iy, z_mask):
        if not keep:
            continue
        cell_xy[(int(x), int(y))] = True
    seed = (int(uniq[max_i][0]), int(uniq[max_i][1]))
    keep_cells = set(_flood_from(seed, cell_xy))
    if len(keep_cells) < 1:
        keep_cells = set(_largest_component(cell_xy))
    mask = z_mask & np.array(
        [(int(x), int(y)) in keep_cells for x, y in keys], dtype=bool)
    patch = pts[mask]
    return {
        "ok": True,
        "reason": "ok",
        "points": patch,
        "mask": mask,
        "n_cells": int(len(keep_cells)),
        "max_mean_z": max_mean,
        "patch_mean_z": float(patch[:, 2].mean()) if len(patch) else None,
        "n_points": int(len(patch)),
    }


def _flood_from(seed, cell_xy):
    if seed not in cell_xy:
        return _largest_component(cell_xy)
    stack = [seed]
    seen = set(stack)
    while stack:
        cx, cy = stack.pop()
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nxt = (cx + dx, cy + dy)
                if nxt in cell_xy and nxt not in seen:
                    seen.add(nxt)
                    stack.append(nxt)
    return seen


def _largest_component(cell_xy):
    remaining = set(cell_xy.keys())
    best = []
    while remaining:
        seed = remaining.pop()
        comp = _flood_from(seed, {k: True for k in remaining | {seed}})
        remaining -= set(comp)
        if len(comp) > len(best):
            best = list(comp)
    return best


def densest_xy_component(points, cell=0.04):
    """Keep the XY-grid component that contains the most points."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if pts.shape[0] == 0:
        return pts, np.zeros((0,), dtype=bool)
    cell = float(cell)
    ix = np.floor(pts[:, 0] / cell).astype(np.int64)
    iy = np.floor(pts[:, 1] / cell).astype(np.int64)
    cell_xy = dict(((int(x), int(y)), True) for x, y in zip(ix, iy))
    remaining = set(cell_xy.keys())
    keys = [(int(x), int(y)) for x, y in zip(ix, iy)]
    best_mask = np.ones((len(pts),), dtype=bool)
    best_n = -1
    while remaining:
        seed = remaining.pop()
        comp = set(_flood_from(seed, dict((k, True) for k in remaining | {seed})))
        remaining -= comp
        mask = np.array([key in comp for key in keys], dtype=bool)
        n = int(mask.sum())
        if n > best_n:
            best_n = n
            best_mask = mask
    return pts[best_mask], best_mask


def crop_aabb(points, ref, xy_margin=0.08, z_margin=0.04):
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    ref = np.asarray(ref, dtype=np.float64).reshape(-1, 3)
    if pts.shape[0] == 0 or ref.shape[0] == 0:
        return pts.reshape(0, 3), np.zeros((pts.shape[0],), dtype=bool)
    lo = ref.min(axis=0) - np.array([xy_margin, xy_margin, z_margin])
    hi = ref.max(axis=0) + np.array([xy_margin, xy_margin, z_margin])
    mask = np.all((pts >= lo) & (pts <= hi), axis=1)
    return pts[mask], mask


def table_edge_report(cam_pts, lid_pts, cam_labels=None, lid_labels=None):
    """AABB, hull, and the bottom-right corner residual (max-x then min-y)."""
    cam = np.asarray(cam_pts, dtype=np.float64).reshape(-1, 3)
    lid = np.asarray(lid_pts, dtype=np.float64).reshape(-1, 3)
    empty = {
        "ok": False,
        "corner_residual_m": None,
        "br_corner_residual_m": None,
        "visible_corner_residual_m": None,
        "edge_gap_m": None,
        "aabb_delta_m": None,
    }
    if cam.shape[0] < 8 or lid.shape[0] < 8:
        empty["reason"] = "too_few"
        return empty
    cam_xy = cam[:, :2]
    lid_xy = lid[:, :2]
    cam_min, cam_max = cam_xy.min(0), cam_xy.max(0)
    lid_min, lid_max = lid_xy.min(0), lid_xy.max(0)
    aabb_delta = {
        "min_x": float(lid_min[0] - cam_min[0]),
        "max_x": float(lid_max[0] - cam_max[0]),
        "min_y": float(lid_min[1] - cam_min[1]),
        "max_y": float(lid_max[1] - cam_max[1]),
    }
    vis = visible_corner_residual(cam, cam_labels, lid, lid_labels)
    br = br_corner_residual(cam, cam_labels, lid, lid_labels)
    corner_res = br.get("residual_m")
    vis_res = vis.get("residual_m")
    edge_gap = None
    try:
        cam_hull = ConvexHull(cam_xy)
        lid_hull = ConvexHull(lid_xy)
    except Exception:
        return {
            "ok": br.get("ok", False),
            "reason": "hull_failed",
            "corner_residual_m": corner_res,
            "br_corner_residual_m": corner_res,
            "visible_corner_residual_m": vis_res,
            "br_corner": br,
            "visible_corner": vis,
            "edge_gap_m": None,
            "aabb_delta_m": aabb_delta,
        }
    cam_c = cam_xy[cam_hull.vertices]
    lid_c = lid_xy[lid_hull.vertices]
    dist, _ = cKDTree(lid_c).query(cam_c, k=1)
    hull_min = float(np.min(dist))
    verts = cam_hull.vertices
    edges = []
    for i in range(len(verts)):
        a = cam_xy[verts[i]]
        b = cam_xy[verts[(i + 1) % len(verts)]]
        edges.append((float(np.linalg.norm(b - a)), a, b))
    edges.sort(key=lambda row: row[0], reverse=True)
    gaps = []
    for _length, a, b in edges[:2]:
        ab = b - a
        ln = max(float(np.linalg.norm(ab)), 1e-9)
        tang = ab / ln
        nrm = np.array([-tang[1], tang[0]])
        t = (lid_xy - a).dot(tang)
        on = (t >= -0.02) & (t <= ln + 0.02)
        if int(on.sum()) < 8:
            continue
        signed = (lid_xy[on] - a).dot(nrm)
        near = np.abs(signed) < 0.06
        if int(near.sum()) < 8:
            continue
        gaps.append(float(np.median(signed[near])))
    if gaps:
        edge_gap = float(np.mean(np.abs(gaps)))
    return {
        "ok": True,
        "reason": "ok",
        "corner_residual_m": corner_res,
        "br_corner_residual_m": corner_res,
        "visible_corner_residual_m": vis_res,
        "hull_min_corner_m": hull_min,
        "br_corner": br,
        "visible_corner": vis,
        "edge_gap_m": edge_gap,
        "aabb_delta_m": aabb_delta,
        "cam_aabb": {
            "min": [float(v) for v in cam_min],
            "max": [float(v) for v in cam_max],
        },
        "lid_aabb": {
            "min": [float(v) for v in lid_min],
            "max": [float(v) for v in lid_max],
        },
    }


def _label_colors(labels):
    lab = np.asarray(labels, dtype=np.int32)
    cols = np.zeros((len(lab), 3), dtype=np.uint8)
    cols[:] = (40, 90, 120)
    cols[lab == LABEL_EDGE] = (255, 210, 70)
    cols[lab == LABEL_BOTTOM_EDGE] = (255, 120, 40)
    cols[lab == LABEL_RIGHT_EDGE] = (80, 255, 120)
    cols[lab == LABEL_CORNER] = (0, 230, 255)
    return cols


def _overlay_colors(cam_pts, cam_rgb, lid_pts, lid_rgb=(255, 80, 200)):
    cam_pts = np.asarray(cam_pts).reshape(-1, 3)
    lid_pts = np.asarray(lid_pts).reshape(-1, 3)
    if cam_rgb is None or len(cam_rgb) != len(cam_pts):
        cols = np.tile(np.array([[80, 180, 255]], dtype=np.uint8), (len(cam_pts), 1))
    else:
        cols = np.asarray(cam_rgb, dtype=np.uint8).reshape(-1, 3)
    lid_c = np.tile(np.array([lid_rgb], dtype=np.uint8), (len(lid_pts), 1))
    return np.concatenate([cam_pts, lid_pts], axis=0), np.concatenate(
        [cols, lid_c], axis=0)


def _write_xy_png(path, cam_pts, lid_pts, title, cam_labels=None, lid_labels=None,
                  cam_corner=None, lid_corner=None):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return False
    cam = np.asarray(cam_pts).reshape(-1, 3)
    lid = np.asarray(lid_pts).reshape(-1, 3)
    fig, ax = plt.subplots(1, 1, figsize=(8, 7), dpi=140)
    if len(cam):
        if cam_labels is None:
            ax.scatter(cam[:, 0], cam[:, 1], s=2, c="#7ec8ff", label="D555 table")
        else:
            lab = np.asarray(cam_labels)
            ax.scatter(cam[lab == LABEL_INTERIOR][:, 0],
                       cam[lab == LABEL_INTERIOR][:, 1],
                       s=1, c="#355a73", label="D555 interior")
            ax.scatter(cam[lab == LABEL_EDGE][:, 0], cam[lab == LABEL_EDGE][:, 1],
                       s=6, c="#ffd24a", label="D555 other edge")
            ax.scatter(cam[lab == LABEL_BOTTOM_EDGE][:, 0],
                       cam[lab == LABEL_BOTTOM_EDGE][:, 1],
                       s=8, c="#ff7828", label="D555 bottom")
            ax.scatter(cam[lab == LABEL_RIGHT_EDGE][:, 0],
                       cam[lab == LABEL_RIGHT_EDGE][:, 1],
                       s=8, c="#50ff78", label="D555 right")
            ax.scatter(cam[lab == LABEL_CORNER][:, 0],
                       cam[lab == LABEL_CORNER][:, 1],
                       s=18, c="#00e5ff", marker="o", label="D555 corner")
    if len(lid):
        rng = np.random.RandomState(0)
        lid_p, lab_p = lid, None if lid_labels is None else np.asarray(lid_labels)
        if len(lid_p) > 12000:
            idx = rng.choice(len(lid_p), 12000, replace=False)
            lid_p = lid_p[idx]
            if lab_p is not None:
                lab_p = lab_p[idx]
        if lab_p is None:
            ax.scatter(lid_p[:, 0], lid_p[:, 1], s=2, c="#e040a0", alpha=0.4,
                       label="Livox table")
        else:
            ax.scatter(lid_p[lab_p == LABEL_INTERIOR][:, 0],
                       lid_p[lab_p == LABEL_INTERIOR][:, 1],
                       s=1, c="#7a3060", alpha=0.35, label="Livox interior")
            ax.scatter(lid_p[lab_p == LABEL_EDGE][:, 0],
                       lid_p[lab_p == LABEL_EDGE][:, 1],
                       s=6, c="#ff7ad9", label="Livox other edge")
            ax.scatter(lid_p[lab_p == LABEL_BOTTOM_EDGE][:, 0],
                       lid_p[lab_p == LABEL_BOTTOM_EDGE][:, 1],
                       s=8, c="#ff9a40", label="Livox bottom")
            ax.scatter(lid_p[lab_p == LABEL_RIGHT_EDGE][:, 0],
                       lid_p[lab_p == LABEL_RIGHT_EDGE][:, 1],
                       s=8, c="#7cff9a", label="Livox right")
            ax.scatter(lid_p[lab_p == LABEL_CORNER][:, 0],
                       lid_p[lab_p == LABEL_CORNER][:, 1],
                       s=18, c="#ff2d6a", marker="x", label="Livox corner")
    if cam_corner is not None:
        ax.scatter([cam_corner[0]], [cam_corner[1]], s=90, c="cyan",
                   marker="*", zorder=6, label="camera BR corner")
    if lid_corner is not None:
        ax.scatter([lid_corner[0]], [lid_corner[1]], s=90, c="lime",
                   marker="*", zorder=6, label="livox BR corner")
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(title)
    ax.legend(loc="best", fontsize=8)
    ax.set_facecolor("#111")
    fig.patch.set_facecolor("#222")
    ax.tick_params(colors="w")
    ax.xaxis.label.set_color("w")
    ax.yaxis.label.set_color("w")
    ax.title.set_color("w")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    return True


def run_table_patch_icp(
    cam_pts,
    lid_pts,
    cam_rgb=None,
    cell=0.04,
    z_band=0.03,
    min_cell_n=15,
    xy_margin=0.08,
    z_margin=0.04,
    near_radius=0.12,
    inlier_radius=0.02,
    hull_band=0.01,
    icp_max_dist=0.12,
    icp_voxel=0.015,
    interior_w=1.0,
    edge_w=12.0,
    corner_w=400.0,
    anchor_radius=0.025,
    corners_only=False,
):
    """Crop highest camera table + nearby Livox, then ICP Livox onto D555."""
    cam_pts = np.asarray(cam_pts, dtype=np.float64).reshape(-1, 3)
    lid_pts = np.asarray(lid_pts, dtype=np.float64).reshape(-1, 3)
    if cam_rgb is not None:
        cam_rgb = np.asarray(cam_rgb).reshape(-1, 3)
        if len(cam_rgb) != len(cam_pts):
            cam_rgb = None

    patch = highest_mean_height_patch(
        cam_pts, cell=cell, z_band=z_band, min_cell_n=min_cell_n)
    if not patch["ok"]:
        return {"ok": False, "stage": "highest_patch", "patch": patch}

    table_cam = patch["points"]
    table_rgb = cam_rgb[patch["mask"]] if cam_rgb is not None else None
    fitted = ransac_plane(table_cam, n_iter=100, thresh=0.012)
    if fitted is not None:
        normal, offset, inl = fitted
        table_cam = table_cam[inl]
        if table_rgb is not None:
            table_rgb = table_rgb[inl]
        plane = {
            "normal": [float(v) for v in normal],
            "offset": float(offset),
            "n_inliers": int(inl.sum()),
        }
    else:
        plane = None

    table_cam, main_mask = densest_xy_component(table_cam, cell=cell)
    if table_rgb is not None:
        table_rgb = np.asarray(table_rgb)[main_mask]

    lid_aabb, _ = crop_aabb(
        lid_pts, table_cam, xy_margin=xy_margin, z_margin=z_margin)
    lid_near_mask = nearby_mask(lid_aabb, table_cam, near_radius)
    lid_coarse = lid_aabb[lid_near_mask]
    inlier_mask, outlier_info = reject_livox_outliers(
        lid_coarse, table_cam,
        inlier_radius=inlier_radius, hull_band=hull_band)
    lid_rejected = lid_coarse[~inlier_mask]
    lid_near = lid_coarse[inlier_mask]
    cam_near = table_cam
    cam_near_rgb = table_rgb

    cam_dense, cam_dense_mask = densest_xy_component(cam_near, cell=min(cell, 0.02))
    lid_dense, lid_dense_mask = densest_xy_component(lid_near, cell=min(cell, 0.02))
    cam_labels = np.zeros((len(cam_near),), dtype=np.int32)
    lid_labels = np.zeros((len(lid_near),), dtype=np.int32)
    cam_sub, cam_corners = classify_table_features(cam_dense)
    lid_sub, lid_corners_local = classify_table_features(lid_dense)
    cam_labels[cam_dense_mask] = cam_sub
    cam_labels = demote_interior_corners(cam_near, cam_labels)
    cam_br = bottom_right_table_corner(cam_near, cam_labels)
    cam_labels = focus_anchor_corners(
        cam_near, cam_labels, cam_br, radius=anchor_radius)
    cam_labels = label_br_incident_edges(
        cam_near, cam_labels, cam_br, band=max(0.016, float(anchor_radius) * 0.7))
    lid_br = None
    if len(lid_near):
        lid_labels[lid_dense_mask] = lid_sub
        lid_labels = demote_interior_corners(lid_near, lid_labels)
        lid_br = bottom_right_table_corner(lid_near, lid_labels)
        lid_labels = focus_anchor_corners(
            lid_near, lid_labels, lid_br, radius=anchor_radius)
        lid_labels = label_br_incident_edges(
            lid_near, lid_labels, lid_br,
            band=max(0.016, float(anchor_radius) * 0.7))
        far_feat = far_livox_feature_mask(
            lid_near, lid_labels, cam_near, cam_labels, radius=inlier_radius)
        far_feat = far_feat & (lid_labels == LABEL_EDGE)
        if lid_br is not None:
            d_br = np.linalg.norm(
                lid_near[:, :2] - np.asarray(lid_br)[:2], axis=1)
            far_feat = far_feat & (d_br > float(anchor_radius))
        if np.any(far_feat):
            lid_rejected = np.concatenate([lid_rejected, lid_near[far_feat]], axis=0) \
                if len(lid_rejected) else lid_near[far_feat]
            keep_feat = ~far_feat
            lid_near = lid_near[keep_feat]
            lid_labels = lid_labels[keep_feat]
            outlier_info["n_far_feature"] = int(far_feat.sum())
            outlier_info["n_keep"] = int(len(lid_near))
        else:
            outlier_info["n_far_feature"] = 0
    else:
        outlier_info["n_far_feature"] = 0
    lid_corners = lid_near[lid_labels == LABEL_CORNER] if len(lid_near) else lid_near
    cam_corners = cam_near[cam_labels == LABEL_CORNER] if len(cam_near) else cam_near
    outlier_info["n_rejected"] = int(len(lid_rejected))
    cam_w = _channel_weights(cam_labels)
    lid_w = _channel_weights(lid_labels)

    before_edges = table_edge_report(
        cam_near, lid_near, cam_labels=cam_labels, lid_labels=lid_labels)
    before_rmse, before_pairs = (
        nn_rmse(lid_near, cam_near, max_dist=0.08)
        if len(cam_near) and len(lid_near) else (float("nan"), 0))

    icp = None
    T = np.eye(4)
    lid_icp = lid_near
    lid_labels_icp = lid_labels
    icp_mode = "corners_only" if corners_only else "br_corner_and_sides"
    try:
        if corners_only:
            if len(lid_corners) < 2 or len(cam_corners) < 2:
                raise RuntimeError(
                    "not enough corners for ICP (src=%d dst=%d)"
                    % (len(lid_corners), len(cam_corners)))
            T, icp = icp_xy_yaw(
                lid_corners, cam_corners, max_iter=50, max_dist=icp_max_dist,
                voxel=min(float(icp_voxel), 0.008), min_pairs=2,
                src_weight=np.ones(len(lid_corners)),
                dst_weight=np.ones(len(cam_corners)),
                corner_w=1.0, allow_yaw=False)
        else:
            src_m = np.isin(lid_labels, ICP_CHANNELS)
            dst_m = np.isin(cam_labels, ICP_CHANNELS)
            if int(src_m.sum()) < 4 or int(dst_m.sum()) < 4:
                raise RuntimeError(
                    "not enough BR corner/side points (src=%d dst=%d)"
                    % (int(src_m.sum()), int(dst_m.sum())))
            T, icp = icp_channel_matched(
                lid_near[src_m], lid_labels[src_m],
                cam_near[dst_m], cam_labels[dst_m],
                max_iter=50, max_dist=icp_max_dist,
                voxel=min(float(icp_voxel), 0.01), min_pairs=6,
                allow_yaw=True,
                src_weight=lid_w[src_m], dst_weight=cam_w[dst_m])
        if isinstance(icp, dict):
            icp["mode"] = icp_mode
        lid_icp = transform_points(T, lid_near)
        lid_labels_icp = lid_labels
    except Exception as exc:
        icp = {"error": str(exc), "mode": icp_mode}

    after_edges = table_edge_report(
        cam_near, lid_icp, cam_labels=cam_labels, lid_labels=lid_labels_icp)
    after_rmse, after_pairs = (
        nn_rmse(lid_icp, cam_near, max_dist=0.08)
        if len(cam_near) and len(lid_icp) else (float("nan"), 0))

    br_before = (before_edges.get("br_corner") or {}).get("residual_m")
    br_after = (after_edges.get("br_corner") or {}).get("residual_m")
    vis_before = (before_edges.get("visible_corner") or {}).get("residual_m")
    vis_after = (after_edges.get("visible_corner") or {}).get("residual_m")
    corner = br_after
    coincide = (
        icp is not None and "error" not in icp
        and corner is not None and corner <= 0.02
        and (br_before is None or corner <= br_before + 1e-6)
    )
    return {
        "ok": icp is not None and "error" not in icp,
        "stage": "icp" if (icp is None or "error" not in icp) else "icp_failed",
        "patch": {
            "n_cells": patch["n_cells"],
            "max_mean_z": patch["max_mean_z"],
            "patch_mean_z": patch["patch_mean_z"],
            "n_table_before_plane": int(patch["n_points"]),
        },
        "plane": plane,
        "n_camera_table": int(len(table_cam)),
        "n_livox_aabb": int(len(lid_aabb)),
        "n_camera_near": int(len(cam_near)),
        "n_livox_near": int(len(lid_near)),
        "n_livox_rejected": int(len(lid_rejected)),
        "outlier": outlier_info,
        "n_camera_edge": int((cam_labels == LABEL_EDGE).sum()),
        "n_camera_bottom_edge": int((cam_labels == LABEL_BOTTOM_EDGE).sum()),
        "n_camera_right_edge": int((cam_labels == LABEL_RIGHT_EDGE).sum()),
        "n_camera_corner": int((cam_labels == LABEL_CORNER).sum()),
        "n_livox_edge": int((lid_labels == LABEL_EDGE).sum()),
        "n_livox_bottom_edge": int((lid_labels == LABEL_BOTTOM_EDGE).sum()),
        "n_livox_right_edge": int((lid_labels == LABEL_RIGHT_EDGE).sum()),
        "n_livox_corner": int((lid_labels == LABEL_CORNER).sum()),
        "feature_weights": {
            "interior": float(interior_w),
            "edge": float(edge_w),
            "corner": float(corner_w),
            "anchor_radius_m": float(anchor_radius),
        },
        "near_radius_m": float(near_radius),
        "xy_margin_m": float(xy_margin),
        "z_margin_m": float(z_margin),
        "br_corner_before_m": br_before,
        "br_corner_after_m": br_after,
        "visible_corner_before_m": vis_before,
        "visible_corner_after_m": vis_after,
        "icp_mode": icp_mode,
        "before": {
            "nn_rmse_m": None if not np.isfinite(before_rmse) else float(before_rmse),
            "nn_pairs": int(before_pairs),
            "edges": before_edges,
        },
        "after": {
            "nn_rmse_m": None if not np.isfinite(after_rmse) else float(after_rmse),
            "nn_pairs": int(after_pairs),
            "edges": after_edges,
        },
        "icp": {
            k: (v.tolist() if hasattr(v, "tolist") else v)
            for k, v in (icp or {}).items()
        },
        "T_livox_to_camera": T.tolist(),
        "icp_translation_m": translation_m(T),
        "icp_rotation_deg": rotation_deg(T),
        "coincide_table_corner_and_edge": bool(coincide),
        "clouds": {
            "camera_table": cam_near,
            "camera_rgb": cam_near_rgb,
            "camera_labels": cam_labels,
            "livox_table": lid_near,
            "livox_icp": lid_icp,
            "livox_labels": lid_labels,
            "livox_outliers": lid_rejected,
            "camera_corners": cam_corners,
            "livox_corners": transform_points(T, lid_corners) if len(lid_corners) else lid_corners,
            "livox_corners_before": lid_corners,
        },
    }


def dump_table_patch_icp(cam_ply, lid_ply, out_dir, **kwargs):
    cam_pts, cam_rgb = read_ply_points(cam_ply)
    lid_pts, _lid_rgb = read_ply_points(lid_ply)
    report = run_table_patch_icp(cam_pts, lid_pts, cam_rgb=cam_rgb, **kwargs)
    os.makedirs(out_dir, exist_ok=True)
    clouds = report.pop("clouds", {})
    cam_t = clouds.get("camera_table", np.zeros((0, 3)))
    cam_c = clouds.get("camera_rgb")
    lid_t = clouds.get("livox_table", np.zeros((0, 3)))
    lid_i = clouds.get("livox_icp", np.zeros((0, 3)))
    lid_out = clouds.get("livox_outliers", np.zeros((0, 3)))
    cam_lab = clouds.get("camera_labels")
    lid_lab = clouds.get("livox_labels")
    vis_b = (report.get("before", {}).get("edges") or {}).get("br_corner") or {}
    vis_a = (report.get("after", {}).get("edges") or {}).get("br_corner") or {}
    if cam_c is None:
        write_ply_xyz(os.path.join(out_dir, "table_camera.ply"), cam_t)
    else:
        write_ply_xyzrgb(os.path.join(out_dir, "table_camera.ply"), cam_t, cam_c)
    write_ply_xyz(os.path.join(out_dir, "table_livox_before.ply"), lid_t)
    write_ply_xyz(os.path.join(out_dir, "table_livox_after_icp.ply"), lid_i)
    if len(lid_out):
        write_ply_xyz(os.path.join(out_dir, "table_livox_outliers.ply"), lid_out)
    if cam_lab is not None and len(cam_t):
        write_ply_xyzrgb(
            os.path.join(out_dir, "table_camera_features.ply"),
            cam_t, _label_colors(cam_lab))
        write_ply_xyz(
            os.path.join(out_dir, "table_camera_corners.ply"),
            cam_t[np.asarray(cam_lab) == LABEL_CORNER])
        write_ply_xyz(
            os.path.join(out_dir, "table_camera_edges.ply"),
            cam_t[np.asarray(cam_lab) == LABEL_EDGE])
    if lid_lab is not None and len(lid_t):
        write_ply_xyzrgb(
            os.path.join(out_dir, "table_livox_features_before.ply"),
            lid_t, _label_colors(lid_lab))
        write_ply_xyz(
            os.path.join(out_dir, "table_livox_corners_before.ply"),
            lid_t[np.asarray(lid_lab) == LABEL_CORNER])
        write_ply_xyz(
            os.path.join(out_dir, "table_livox_edges_before.ply"),
            lid_t[np.asarray(lid_lab) == LABEL_EDGE])
    if lid_lab is not None and len(lid_i):
        write_ply_xyzrgb(
            os.path.join(out_dir, "table_livox_features_after_icp.ply"),
            lid_i, _label_colors(lid_lab))
    if len(cam_t) and len(lid_t):
        pts, cols = _overlay_colors(cam_t, cam_c, lid_t)
        write_ply_xyzrgb(os.path.join(out_dir, "overlay_before.ply"), pts, cols)
    if len(cam_t) and len(lid_i):
        pts, cols = _overlay_colors(cam_t, cam_c, lid_i, lid_rgb=(80, 255, 120))
        write_ply_xyzrgb(os.path.join(out_dir, "overlay_after_icp.ply"), pts, cols)
    _write_xy_png(
        os.path.join(out_dir, "table_xy_before.png"), cam_t, lid_t,
        "Table features before ICP",
        cam_labels=cam_lab, lid_labels=lid_lab,
        cam_corner=vis_b.get("camera_xyz"), lid_corner=vis_b.get("livox_xyz"))
    _write_xy_png(
        os.path.join(out_dir, "table_xy_after_icp.png"), cam_t, lid_i,
        "Table features after BR-corner + side-edge ICP",
        cam_labels=cam_lab, lid_labels=lid_lab,
        cam_corner=vis_a.get("camera_xyz"), lid_corner=vis_a.get("livox_xyz"))
    serial = dict(report)
    serial["frozen_revision"] = FROZEN_REVISION
    serial["camera_ply"] = os.path.abspath(cam_ply)
    serial["livox_ply"] = os.path.abspath(lid_ply)
    serial["out_dir"] = os.path.abspath(out_dir)
    with open(os.path.join(out_dir, "table_icp.json"), "w", encoding="utf-8") as handle:
        json.dump(_jsonable(serial), handle, indent=2)
        handle.write("\n")
    md = [
        "# Table patch ICP (BR corner + bottom/right edges)",
        "",
        "- Camera table / bottom / right / corner: `%s` / `%s` / `%s` / `%s`" % (
            report.get("n_camera_near"), report.get("n_camera_bottom_edge"),
            report.get("n_camera_right_edge"), report.get("n_camera_corner")),
        "- Livox table / bottom / right / corner: `%s` / `%s` / `%s` / `%s`" % (
            report.get("n_livox_near"), report.get("n_livox_bottom_edge"),
            report.get("n_livox_right_edge"), report.get("n_livox_corner")),
        "- Livox rejected as out-of-range: `%s`" % report.get("n_livox_rejected"),
        "- ICP mode: `%s`" % report.get("icp_mode"),
        "- Feature weights interior/edge/corner: `%s`" % report.get("feature_weights"),
        "- ICP translation: `%.4f` m  rotation: `%.3f` deg" % (
            report.get("icp_translation_m") or 0.0,
            report.get("icp_rotation_deg") or 0.0),
        "- Bottom-right corner residual before: `%s` m" % report.get("br_corner_before_m"),
        "- Bottom-right corner residual after: `%s` m" % report.get("br_corner_after_m"),
        "- After edge gap: `%s` m" % (
            (report.get("after", {}).get("edges") or {}).get("edge_gap_m")),
        "- Coincide corner+edge: `%s`" % report.get("coincide_table_corner_and_edge"),
        "",
    ]
    with open(os.path.join(out_dir, "INDEX.md"), "w", encoding="utf-8") as handle:
        handle.write("\n".join(md) + "\n")
    return _jsonable(serial)
