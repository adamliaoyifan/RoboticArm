"""Eval-only stamped TF tree from bag ``/tf`` + ``/tf_static``.

Offline: no tf2 buffer, no latest-time fallback. A lookup uses the
transform nearest the query stamp inside ``max_dt_ns`` (static always
available; exact-stamp and at-or-before hits are the same thing once the
window is honoured). Missing links fail closed so a pick is never
reported in a guessed frame.

Performance: after ingestion, per-child histories are finalized into
stamp/translation/quaternion/matrix arrays and lookups go through a
bisect nearest-stamp search (same tie-break as the aux joins) plus a
bounded memo — the replay's repeated same-stamp chain walks (cargo
points, pick point, waypoints, TCP series) then cost one dict hit each.
"""
from __future__ import division

import math
from collections import OrderedDict

import numpy as np

from luggage_perception.eval.bag_frame_join import nearest_stamp

_MEMO_MAX = 4096


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


def _slerp(q0, q1, alpha):
    """Spherical linear interpolation of two quaternions (xyzw).

    Shortest-path (negates q1 when the dot is negative); degenerate
    (near-parallel) pairs fall back to normalized lerp. Local on
    purpose — interpolation is an eval-only concern, no new deps.
    """
    q0 = np.asarray(q0, dtype=np.float64)
    q1 = np.asarray(q1, dtype=np.float64)
    n0 = np.linalg.norm(q0)
    n1 = np.linalg.norm(q1)
    if n0 < 1e-12 or n1 < 1e-12:
        return q1 / n1 if n1 > 1e-12 else q0
    q0 = q0 / n0
    q1 = q1 / n1
    dot = float(np.clip(np.dot(q0, q1), -1.0, 1.0))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 1.0 - 1e-10:
        out = q0 + alpha * (q1 - q0)
        return out / np.linalg.norm(out)
    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    return (math.sin((1.0 - alpha) * theta) / sin_theta) * q0 + (
        math.sin(alpha * theta) / sin_theta) * q1


def _matrix_from_pose(translation, quat):
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = _quat_to_rot(quat[0], quat[1], quat[2], quat[3])
    mat[0, 3] = float(translation[0])
    mat[1, 3] = float(translation[1])
    mat[2, 3] = float(translation[2])
    return mat


def _stamp_ns_from_header(header):
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return 0
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class _EdgeHistory(object):
    """Finalized per-child dynamic edge: parallel stamp/parent/pose arrays.

    Equal-stamp rows keep only the first insertion (the linear scan this
    replaced did the same via strict comparisons), so lookups are exact
    bisect searches. Translations and quaternions ride along for the
    interpolation path; ``mats`` stores the original transform_to_matrix
    outputs so nearest-stamp numbers stay bit-identical.
    """

    __slots__ = ("stamps", "stamps_list", "parents", "translations",
                 "quats", "mats")

    def __init__(self, rows):
        seen = set()
        stamps, parents = [], []
        translations, quats, mats = [], [], []
        for stamp, parent, mat, trans, quat in rows:
            if stamp in seen:
                continue
            seen.add(stamp)
            stamps.append(int(stamp))
            parents.append(parent)
            mats.append(mat)
            translations.append(trans)
            quats.append(quat)
        self.stamps = np.asarray(stamps, dtype=np.int64)
        self.stamps_list = [int(s) for s in stamps]
        self.parents = parents
        self.translations = (np.asarray(translations, dtype=np.float64)
                             if translations else np.zeros((0, 3)))
        self.quats = (np.asarray(quats, dtype=np.float64)
                      if quats else np.zeros((0, 4)))
        self.mats = (np.asarray(mats, dtype=np.float64)
                     if mats else np.zeros((0, 4, 4)))

    def nearest(self, stamp_ns, max_dt_ns):
        """(parent, mat) nearest *stamp_ns* within the window, or None."""
        hit = nearest_stamp(self.stamps_list, int(stamp_ns),
                            int(max_dt_ns))
        if hit is None:
            return None
        idx, _dt = hit
        return self.parents[idx], self.mats[idx]

    def interpolated(self, stamp_ns, max_dt_ns, max_gap_ns=None):
        """(parent, mat, gap_ns) between the bracketing samples, or None.

        Exact stamp hits return the stored matrix verbatim. No bracketing
        pair inside ``max_gap_ns`` (default ``max_dt_ns``) fails closed —
        the caller falls back to the nearest-stamp semantics, so enabling
        interpolation can only refine lookups, never remove one. Bracket
        parents must agree, else None.
        """
        stamps = self.stamps_list
        if not stamps:
            return None
        query = int(stamp_ns)
        limit = int(max_gap_ns) if max_gap_ns else int(max_dt_ns)
        import bisect
        idx = bisect.bisect_left(stamps, query)
        if idx < len(stamps) and stamps[idx] == query:
            return self.parents[idx], self.mats[idx], 0
        lo, hi = idx - 1, idx
        if lo < 0 or hi >= len(stamps):
            return None  # never extrapolate past the sampled range
        gap = int(stamps[hi] - stamps[lo])
        if gap > limit:
            return None
        if self.parents[lo] != self.parents[hi]:
            return None
        alpha = (query - stamps[lo]) / float(gap)
        translation = (
            (1.0 - alpha) * self.translations[lo]
            + alpha * self.translations[hi])
        quat = _slerp(self.quats[lo], self.quats[hi], alpha)
        return self.parents[lo], _matrix_from_pose(translation, quat), gap


class BagTfBuffer(object):
    """Parent<-child tree with static edges and stamped dynamic edges."""

    def __init__(self, max_dt_ns=100_000_000):
        self.max_dt_ns = int(max_dt_ns)
        self._static = {}   # child -> (parent, 4x4)
        self._dynamic = {}  # child -> list[(stamp_ns, parent, mat, ...)]
        self._history = {}  # child -> _EdgeHistory (built on demand)
        self._sorted = False
        self._memo = OrderedDict()
        # Interpolation accounting (populated only when lookups run with
        # interpolate=True; see _record_interpolation).
        self.stats = {"lookups": 0, "interpolated_edges": 0,
                      "max_edge_gap_ns": 0}
        self.interpolation_log = []

    def reset_stats(self):
        """Zero the interpolation accounting (per-frame snapshotting)."""
        self.stats = {"lookups": 0, "interpolated_edges": 0,
                      "max_edge_gap_ns": 0}

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
        else:
            stamp = _stamp_ns_from_header(stamped.header)
            t = stamped.transform.translation
            r = stamped.transform.rotation
            # Original quaternion/translation ride along: interpolation
            # (Phase 3) must slerp the published quaternion, not one
            # recovered from the matrix (sign ambiguity).
            self._dynamic.setdefault(child, []).append(
                (stamp, parent, mat,
                 (float(t.x), float(t.y), float(t.z)),
                 (float(r.x), float(r.y), float(r.z), float(r.w))))
            self._sorted = False
        self._memo.clear()

    def set_static(self, parent, child, mat):
        """Replace the static parent<-child edge (eval TF overrides)."""
        parent = str(parent)
        child = str(child)
        if not parent or not child or parent == child:
            return
        self._static[child] = (parent, np.asarray(mat, dtype=np.float64))
        self._memo.clear()

    def finalize(self):
        """Build the per-child lookup arrays (idempotent, O(n log n))."""
        if not self._sorted:
            for child, rows in self._dynamic.items():
                rows.sort(key=lambda row: row[0])
            self._sorted = True
            self._history = {}
        if not self._history and self._dynamic:
            for child, rows in self._dynamic.items():
                self._history[child] = _EdgeHistory(rows)

    def _ensure_sorted(self):
        self.finalize()

    def _parent_of(self, child, stamp_ns, interpolate=False,
                   max_gap_ns=None):
        self.finalize()
        history = self._history.get(child)
        if history is not None:
            if interpolate:
                hit = history.interpolated(stamp_ns, self.max_dt_ns,
                                           max_gap_ns)
                if hit is not None:
                    parent, mat, gap = hit
                    if gap > 0:  # exact hits are not interpolation
                        self._record_interpolation(child, stamp_ns, gap)
                    return parent, mat
            hit = history.nearest(stamp_ns, self.max_dt_ns)
            if hit is not None:
                return hit
        return self._static.get(child)

    def _record_interpolation(self, child, stamp_ns, gap_ns):
        """Honest accounting: every interpolated edge is counted and the
        last 256 are logged with their gap (the eval report says which
        numbers were produced under interpolation and by how much)."""
        self.stats["interpolated_edges"] += 1
        self.stats["max_edge_gap_ns"] = max(
            int(self.stats["max_edge_gap_ns"]), int(gap_ns))
        self.interpolation_log.append({
            "child": str(child), "stamp_ns": int(stamp_ns),
            "gap_ns": int(gap_ns)})
        if len(self.interpolation_log) > 256:
            del self.interpolation_log[:-256]

    def _chain_to_root(self, frame, stamp_ns, interpolate=False,
                       max_gap_ns=None):
        chain = []
        cur = str(frame)
        seen = set()
        while cur and cur not in seen:
            seen.add(cur)
            edge = self._parent_of(cur, stamp_ns, interpolate=interpolate,
                                   max_gap_ns=max_gap_ns)
            if edge is None:
                break
            parent, mat = edge
            chain.append((cur, parent, mat))
            cur = parent
        return chain, cur

    def _root_matrix(self, frame, stamp_ns, interpolate=False,
                     max_gap_ns=None):
        """4x4 taking *frame* points into the root, plus the root name."""
        chain, root = self._chain_to_root(
            frame, stamp_ns, interpolate=interpolate,
            max_gap_ns=max_gap_ns)
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

    def lookup_matrix(self, target_frame, source_frame, stamp_ns,
                      interpolate=False, max_gap_ns=None):
        """4x4 taking *source* points into *target*, or None.

        Results are memoized per (target, source, stamp, mode): the
        replay asks the same chain question for the cargo cloud, the pick
        point and every waypoint of one frame. ``add_transform``/
        ``set_static`` clear the memo so a stale matrix can never be
        served.

        ``interpolate=False`` (the default) keeps today's fail-closed
        nearest-stamp semantics. ``interpolate=True`` refines each
        dynamic edge by lerping/slerping its bracketing samples when they
        sit within ``max_gap_ns`` (default ``max_dt_ns``); an unbracketed
        stamp falls back to nearest, so enabling interpolation can only
        change numbers, never remove a lookup. Interpolated edges are
        counted in ``stats`` and logged (bounded) in ``interpolation_log``.
        """
        target = str(target_frame)
        source = str(source_frame)
        if not target or not source:
            return None
        self.stats["lookups"] += 1
        key = (target, source, int(stamp_ns), bool(interpolate),
               int(max_gap_ns) if max_gap_ns else 0)
        if key in self._memo:
            self._memo.move_to_end(key)
            result, gap = self._memo[key]
            # A memoized matrix was computed under interpolation or it
            # was not; serving it again must account the same way, or a
            # frame whose lookups were all memo hits would report
            # "nearest" for interpolated numbers.
            if gap is not None:
                self._record_interpolation("(memo)", stamp_ns, gap)
            return result
        edges_before = self.stats["interpolated_edges"]
        if target == source:
            result = np.eye(4, dtype=np.float64)
        else:
            src_mat, src_root = self._root_matrix(
                source, stamp_ns, interpolate=interpolate,
                max_gap_ns=max_gap_ns)
            tgt_mat, tgt_root = self._root_matrix(
                target, stamp_ns, interpolate=interpolate,
                max_gap_ns=max_gap_ns)
            if not src_root or not tgt_root or src_root != tgt_root:
                result = None
            else:
                result = invert_matrix(tgt_mat).dot(src_mat)
        gap = (self.stats["max_edge_gap_ns"]
               if self.stats["interpolated_edges"] > edges_before
               else None)
        self._memo[key] = (result, gap)
        if len(self._memo) > _MEMO_MAX:
            self._memo.popitem(last=False)
        return result

    def transform_points(self, points, target_frame, source_frame,
                         stamp_ns, interpolate=False, max_gap_ns=None):
        mat = self.lookup_matrix(
            target_frame, source_frame, stamp_ns,
            interpolate=interpolate, max_gap_ns=max_gap_ns)
        if mat is None:
            return None
        return apply_matrix(mat, points)

    # ------------------------------------------------------------------
    # Sidecar (de)hydration: the replay index caches TF edges alongside
    # the stamp index so a warm site-pick replay skips the /tf stream.
    # ------------------------------------------------------------------

    def export_edges(self):
        """{child: edge dict} snapshot for npz serialization."""
        self.finalize()
        edges = {}
        for child, history in self._history.items():
            edges[child] = {
                "stamps": history.stamps,
                "parents": history.parents,
                "translations": history.translations,
                "quats": history.quats,
                "mats": history.mats,
            }
        return edges

    def load_edges(self, edges, static_edges=None):
        """Restore buffers written by ``save_edges_npz`` helpers.

        ``edges``: {child: {"stamps", "parents", "translations",
        "quats", "mats"}} (numpy arrays or lists). ``static_edges``:
        {child: [parent, 4x4]}.
        """
        self._dynamic = {}
        self._history = {}
        self._sorted = False
        for child, edge in edges.items():
            stamps = np.asarray(edge["stamps"], dtype=np.int64)
            parents = [str(p) for p in edge["parents"]]
            translations = np.asarray(edge["translations"],
                                      dtype=np.float64).reshape(-1, 3)
            quats = np.asarray(edge["quats"], dtype=np.float64).reshape(
                -1, 4)
            mats = np.asarray(edge["mats"], dtype=np.float64).reshape(
                -1, 4, 4)
            history = _EdgeHistory([])
            history.stamps = stamps
            history.stamps_list = [int(s) for s in stamps]
            history.parents = parents
            history.translations = translations
            history.quats = quats
            history.mats = mats
            self._history[str(child)] = history
            self._dynamic[str(child)] = [
                (int(stamps[i]), parents[i], mats[i],
                 tuple(translations[i]), tuple(quats[i]))
                for i in range(len(stamps))]
        self._static = {}
        for child, (parent, mat) in (static_edges or {}).items():
            self._static[str(child)] = (
                str(parent), np.asarray(mat, dtype=np.float64))
        self._memo.clear()
        # Rows were rebuilt ascending from the arrays; skip the re-sort.
        self._sorted = True

    @staticmethod
    def _npz_escape(name):
        # npz keys become zip entry names: escape separators so a frame
        # id can never nest inside a directory-like entry.
        return str(name).replace("/", "%2F")

    @staticmethod
    def _npz_unescape(name):
        return str(name).replace("%2F", "/")

    def save_edges_npz(self, path):
        """Write static+dynamic edges to an npz sidecar file."""
        edges = self.export_edges()
        payload = {
            "dynamic_children": np.asarray(
                sorted(self._npz_escape(c) for c in edges), dtype=np.str_),
        }
        for child, edge in edges.items():
            key = self._npz_escape(child)
            payload["dyn_%s__stamps" % key] = edge["stamps"]
            payload["dyn_%s__parents" % key] = np.asarray(
                edge["parents"], dtype=np.str_)
            payload["dyn_%s__translations" % key] = edge["translations"]
            payload["dyn_%s__quats" % key] = edge["quats"]
            payload["dyn_%s__mats" % key] = edge["mats"]
        payload["static_children"] = np.asarray(
            sorted(self._npz_escape(c) for c in self._static),
            dtype=np.str_)
        for child, (parent, mat) in self._static.items():
            key = self._npz_escape(child)
            payload["sta_%s__parent" % key] = np.asarray(
                parent, dtype=np.str_)
            payload["sta_%s__mat" % key] = np.asarray(
                mat, dtype=np.float64)
        with open(path, "wb") as handle:
            np.savez(handle, **payload)

    def load_edges_npz(self, path):
        """Restore from ``save_edges_npz`` output (replaces content)."""
        with np.load(path, allow_pickle=False) as data:
            edges = {}
            for escaped in [str(c) for c in data["dynamic_children"]]:
                child = self._npz_unescape(escaped)
                edges[child] = {
                    "stamps": data["dyn_%s__stamps" % escaped],
                    "parents": [str(p) for p in
                                data["dyn_%s__parents" % escaped]],
                    "translations": data[
                        "dyn_%s__translations" % escaped],
                    "quats": data["dyn_%s__quats" % escaped],
                    "mats": data["dyn_%s__mats" % escaped],
                }
            static_edges = {}
            for escaped in [str(c) for c in data["static_children"]]:
                child = self._npz_unescape(escaped)
                static_edges[child] = (
                    str(data["sta_%s__parent" % escaped]),
                    data["sta_%s__mat" % escaped])
        self.load_edges(edges, static_edges=static_edges)

    def frames(self):
        names = set(self._static) | set(self._dynamic)
        for parent, _mat in self._static.values():
            names.add(parent)
        for rows in self._dynamic.values():
            for row in rows:
                names.add(row[1])
        return names
