#!/usr/bin/env python3
"""Compare a measured box to ground truth. Numpy/stdlib only, no ROS.

Eval-only (``luggage_perception.eval``). The live detector does not import
this. Primary occupancy score is top-down footprint IoU. Scalar yaw is
scored only when the *GT* aspect ratio is elongated; near-square PCA
heading is noise.
"""

from __future__ import division

import math
from dataclasses import dataclass

import numpy as np

from luggage_perception.box_geometry import (
    MIN_ASPECT_FOR_YAW,
    fold_yaw_pi,
    footprint_iou,
    is_near_square,
    wrap_to_pi,
)


@dataclass(frozen=True)
class BoxObservation:
    x: float
    y: float
    z: float
    yaw: float
    width: float
    depth: float
    height: float


@dataclass(frozen=True)
class AccuracyResult:
    ok: bool
    err_xy: float
    err_z: float
    err_xyz: float
    err_width: float
    err_depth: float
    err_height: float
    err_yaw: float
    swapped: bool
    reason: str
    iou: float
    near_square: bool


def _size_yaw_candidates(measured, gt):
    """Two axis assignments: as-labelled, or 90° + W/D swap."""
    e = wrap_to_pi(measured.yaw - gt.yaw)
    return (
        (gt.width, gt.depth, fold_yaw_pi(e), False),
        (gt.depth, gt.width, fold_yaw_pi(e - math.pi / 2.0), True),
    )


class DetectionAccuracy:
    """Gate: footprint IoU always; scalar yaw only if GT is elongated."""

    def __init__(self, tol_xy=0.03, tol_z=0.02, tol_size=0.05, tol_yaw=0.15,
                 min_aspect=MIN_ASPECT_FOR_YAW, tol_iou=0.60):
        self._tol_xy = float(tol_xy)
        self._tol_z = float(tol_z)
        self._tol_size = float(tol_size)
        self._tol_yaw = float(tol_yaw)
        self._min_aspect = float(min_aspect)
        self._tol_iou = float(tol_iou)

    def compare(self, measured, gt):
        dx = float(measured.x) - float(gt.x)
        dy = float(measured.y) - float(gt.y)
        dz = float(measured.z) - float(gt.z)
        err_xy = math.hypot(dx, dy)
        err_xyz = math.hypot(err_xy, dz)
        err_height = float(measured.height) - float(gt.height)
        near_square = is_near_square(gt.width, gt.depth, self._min_aspect)

        iou = footprint_iou(
            measured.x, measured.y, measured.yaw, measured.width, measured.depth,
            gt.x, gt.y, gt.yaw, gt.width, gt.depth,
        )

        def _score(cand):
            gt_w, gt_d, err_yaw, _swapped = cand
            return (abs(float(measured.width) - gt_w)
                    + abs(float(measured.depth) - gt_d)
                    + abs(err_yaw))

        candidates = _size_yaw_candidates(measured, gt)
        best = min(candidates, key=_score)
        gt_w, gt_d, err_yaw, swapped = best
        err_width = float(measured.width) - gt_w
        err_depth = float(measured.depth) - gt_d
        if near_square:
            err_yaw = 0.0

        reasons = []
        if iou < self._tol_iou:
            reasons.append("iou")
        if err_xy > self._tol_xy:
            reasons.append("xy")
        if abs(dz) > self._tol_z:
            reasons.append("z")
        if (abs(err_width) > self._tol_size
                or abs(err_depth) > self._tol_size
                or abs(err_height) > self._tol_size):
            reasons.append("size")
        if not near_square and abs(err_yaw) > self._tol_yaw:
            reasons.append("yaw")

        if not reasons:
            reason = "near_square" if near_square else "ok"
            ok = True
        else:
            reason = reasons[0]
            ok = False

        return AccuracyResult(
            ok=ok,
            err_xy=err_xy,
            err_z=dz,
            err_xyz=err_xyz,
            err_width=err_width,
            err_depth=err_depth,
            err_height=err_height,
            err_yaw=err_yaw,
            swapped=swapped,
            reason=reason,
            iou=iou,
            near_square=near_square,
        )

    def summarize(self, results):
        n = len(results)
        empty = {
            "n": 0,
            "pass_rate": 0.0,
            "n_near_square": 0,
            "n_swapped": 0,
            "p50": {},
            "p95": {},
        }
        if n == 0:
            return empty

        def _col(getter):
            return np.array([getter(r) for r in results], dtype=np.float64)

        series = {
            "err_xy": _col(lambda r: r.err_xy),
            "err_z": _col(lambda r: abs(r.err_z)),
            "err_width": _col(lambda r: abs(r.err_width)),
            "err_depth": _col(lambda r: abs(r.err_depth)),
            "err_height": _col(lambda r: abs(r.err_height)),
            "err_yaw": _col(lambda r: abs(r.err_yaw)),
            "iou": _col(lambda r: r.iou),
        }
        p50 = {k: float(np.percentile(v, 50)) for k, v in series.items()}
        p95 = {k: float(np.percentile(v, 95)) for k, v in series.items()}
        n_ok = sum(1 for r in results if r.ok)
        return {
            "n": n,
            "pass_rate": n_ok / float(n),
            "n_near_square": sum(1 for r in results if r.near_square),
            "n_swapped": sum(1 for r in results if r.swapped),
            "p50": p50,
            "p95": p95,
        }
