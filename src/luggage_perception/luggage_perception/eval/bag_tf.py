"""Eval-only stamped TF tree from bag ``/tf`` + ``/tf_static``.

Offline: no tf2 buffer, no latest-time fallback. A lookup uses the
transform at or before the query stamp (static always available). Missing
links fail closed so a pick is never reported in a guessed frame.
"""
from __future__ import division

import math

import numpy as np


def _quat_to_rot(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3, dtype=np.float64)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def transform_to_matrix(tf):
    """``geometry_msgs/Transform`` or TransformStamped.transform -> 4x4."""
    t = tf.translation
    r = tf.rotation
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = _quat_to_rot(r.x, r.y, r.z, r.w)
    mat[0, 3] = float(t.x)
    mat[1, 3] = float(t.y)
    mat[2, 3] = float(t.z)
    return mat


def invert_matrix(mat):
    out = np.eye(4, dtype=np.float64)
    rot = mat[:3, :3]
    out[:3, :3] = rot.T
    out[:3, 3] = -rot.T.dot(mat[:3, 3])
    return out


def apply_matrix(mat, points):
    """Transform (N,3) points by a 4x4 matrix."""
    pts = np.asarray(points, dtype=np.float64)
    if pts.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    ones = np.ones((pts.shape[0], 1), dtype=np.float64)
    homo = np.concatenate((pts[:, :3], ones), axis=1)
    return homo.dot(mat.T)[:, :3]


def _stamp_ns_from_header(header):
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return 0
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class BagTfBuffer(object):
    """Parent<-child tree with static edges and stamped dynamic edges."""

    def __init__(self, max_dt_ns=100_000_000):
        self.max_dt_ns = int(max_dt_ns)
        self._static = {}   # child -> (parent, 4x4)
        self._dynamic = {}  # child -> list[(stamp_ns, parent, 4x4)]
        self._sorted = False

    def add_tf_message(self, msg, static=False):
        transforms = getattr(msg, "transforms", None) or []
        for stamped in transforms:
            self.add_transform(stamped, static=static)

    def add_transform(self, stamped, static=False):
        parent = str(stamped.header.frame_id)
        child = str(stamped.child_frame_id)
        if not parent or not child or parent == child:
            return
        mat = transform_to_matrix(stamped.transform)
        if static:
            self._static[child] = (parent, mat)
            return
        stamp = _stamp_ns_from_header(stamped.header)
        self._dynamic.setdefault(child, []).append((stamp, parent, mat))
        self._sorted = False

    def set_static(self, parent, child, mat):
        """Replace the static parent<-child edge (eval TF overrides)."""
        parent = str(parent)
        child = str(child)
        if not parent or not child or parent == child:
            return
        self._static[child] = (parent, np.asarray(mat, dtype=np.float64))

    def _ensure_sorted(self):
        if self._sorted:
            return
        for child, rows in self._dynamic.items():
            rows.sort(key=lambda row: row[0])
        self._sorted = True

    def _parent_of(self, child, stamp_ns):
        hist = self._dynamic.get(child)
        if hist:
            self._ensure_sorted()
            best = None
            for row_stamp, parent, mat in hist:
                dt = int(row_stamp) - int(stamp_ns)
                adt = abs(dt)
                if adt > self.max_dt_ns:
                    if row_stamp > stamp_ns:
                        break
                    continue
                cand = (adt, int(row_stamp), parent, mat)
                if (best is None or cand[0] < best[0]
                        or (cand[0] == best[0] and cand[1] < best[1])):
                    best = cand
            if best is not None:
                return best[2], best[3]
        return self._static.get(child)

    def _chain_to_root(self, frame, stamp_ns):
        chain = []
        cur = str(frame)
        seen = set()
        while cur and cur not in seen:
            seen.add(cur)
            edge = self._parent_of(cur, stamp_ns)
            if edge is None:
                break
            parent, mat = edge
            chain.append((cur, parent, mat))
            cur = parent
        return chain, cur

    def _root_matrix(self, frame, stamp_ns):
        """4x4 taking *frame* points into the root, plus the root name."""
        chain, root = self._chain_to_root(frame, stamp_ns)
        mat = np.eye(4, dtype=np.float64)
        for _child, _parent, edge in chain:
            mat = edge.dot(mat)
        return mat, root

    def path_from_to(self, ancestor, descendant, stamp_ns):
        """Parent→child hops from *ancestor* down to *descendant*, or None."""
        ancestor = str(ancestor)
        descendant = str(descendant)
        if not ancestor or not descendant:
            return None
        if ancestor == descendant:
            return []
        hops_up = []
        chain, _root = self._chain_to_root(descendant, stamp_ns)
        for child, parent, mat in chain:
            hops_up.append((parent, child, np.asarray(mat, dtype=np.float64)))
            if parent == ancestor:
                hops_up.reverse()
                return hops_up
        return None

    def lookup_via_named_chain(self, frames, stamp_ns):
        """Compose T taking last-frame points into first frame via named hops.

        Each consecutive pair must be ancestor→descendant. Missing hop → None.
        """
        frames = [str(f) for f in frames]
        if len(frames) < 2:
            return None
        composed = np.eye(4, dtype=np.float64)
        for parent, child in zip(frames[:-1], frames[1:]):
            hops = self.path_from_to(parent, child, stamp_ns)
            if hops is None:
                return None
            for _parent, _child, edge in hops:
                composed = composed.dot(np.asarray(edge, dtype=np.float64))
        return composed

    def lookup_via_eof_mounter(
            self, target_frame, source_frame, stamp_ns,
            eof_frame="elfin_end_link", mounter_frame="eef_mount_adapter",
            panel_frame="suction_panel"):
        """Source → adapter → suction_panel → EOF → target.

        Requires the panel hop. Fails if any named hop is missing.
        """
        target = str(target_frame)
        source = str(source_frame)
        eof = str(eof_frame)
        mounter = str(mounter_frame)
        panel = str(panel_frame)
        hops_sensor = self.path_from_to(mounter, source, stamp_ns)
        hops_adapter = self.path_from_to(panel, mounter, stamp_ns)
        hops_panel = self.path_from_to(eof, panel, stamp_ns)
        t_tgt_eof = self.lookup_matrix(target, eof, stamp_ns)
        if (hops_sensor is None or hops_adapter is None or hops_panel is None
                or t_tgt_eof is None):
            return None
        t_eof_src = np.eye(4, dtype=np.float64)
        for _parent, _child, edge in hops_panel + hops_adapter + hops_sensor:
            t_eof_src = t_eof_src.dot(edge)
        return t_tgt_eof.dot(t_eof_src)

    def lookup_matrix(self, target_frame, source_frame, stamp_ns):
        """4x4 taking *source* points into *target*, or None."""
        target = str(target_frame)
        source = str(source_frame)
        if not target or not source:
            return None
        if target == source:
            return np.eye(4, dtype=np.float64)
        src_mat, src_root = self._root_matrix(source, stamp_ns)
        tgt_mat, tgt_root = self._root_matrix(target, stamp_ns)
        if not src_root or not tgt_root or src_root != tgt_root:
            return None
        return invert_matrix(tgt_mat).dot(src_mat)

    def transform_points(self, points, target_frame, source_frame, stamp_ns):
        mat = self.lookup_matrix(target_frame, source_frame, stamp_ns)
        if mat is None:
            return None
        return apply_matrix(mat, points)

    def frames(self):
        names = set(self._static) | set(self._dynamic)
        for parent, _mat in self._static.values():
            names.add(parent)
        for rows in self._dynamic.values():
            for _stamp, parent, _mat in rows:
                names.add(parent)
        return names
