#!/usr/bin/env python3
"""Top-down box footprint geometry. Numpy only, no ROS."""

from __future__ import division

import math

import numpy as np

# Gate: GT max/min below this → scalar yaw is not scored (PCA heading is noise).
# Runtime yaw_valid uses a separate eigenvalue threshold in the estimator.
MIN_ASPECT_FOR_YAW = 1.15

_EPS = 1e-12


def wrap_to_pi(angle):
    """Wrap *angle* to (-pi, pi]."""
    a = math.atan2(math.sin(angle), math.cos(angle))
    if a <= -math.pi + 1e-15:
        return math.pi
    return a


def fold_yaw_pi(angle):
    """Fold a wrapped yaw into (-pi/2, pi/2] (180° box symmetry)."""
    e = wrap_to_pi(angle)
    if e > math.pi / 2.0:
        e -= math.pi
    elif e < -math.pi / 2.0:
        e += math.pi
    return e


def aspect_ratio(width, depth):
    """max(W, D) / min(W, D). Degenerate sizes return 1.0 or inf."""
    w = abs(float(width))
    d = abs(float(depth))
    shorter = min(w, d)
    longer = max(w, d)
    if shorter < _EPS:
        return float("inf") if longer >= _EPS else 1.0
    return longer / shorter


def is_near_square(width, depth, min_aspect=MIN_ASPECT_FOR_YAW):
    """True when the footprint is too square for a meaningful heading."""
    return aspect_ratio(width, depth) < float(min_aspect)


def oriented_footprint_corners(x, y, yaw, width, depth):
    """Return (4, 2) CCW corners of an oriented rectangle in XY.

    *width* is the extent along *yaw*; *depth* is the extent 90° CCW from yaw.
    """
    c = math.cos(float(yaw))
    s = math.sin(float(yaw))
    hw = 0.5 * abs(float(width))
    hd = 0.5 * abs(float(depth))
    local = np.array(
        ((-hw, -hd), (hw, -hd), (hw, hd), (-hw, hd)),
        dtype=np.float64,
    )
    rot = np.array(((c, -s), (s, c)), dtype=np.float64)
    return local.dot(rot.T) + np.array((float(x), float(y)), dtype=np.float64)


def _shoelace(poly):
    if poly is None or len(poly) < 3:
        return 0.0
    x = poly[:, 0]
    y = poly[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def _ensure_ccw(poly):
    if _shoelace(poly) < 0.0:
        return poly[::-1].copy()
    return np.asarray(poly, dtype=np.float64)


def _is_left_or_on(point, a, b):
    return ((b[0] - a[0]) * (point[1] - a[1])
            - (b[1] - a[1]) * (point[0] - a[0])) >= -1e-12


def _intersect(s, e, a, b):
    ds = e - s
    dc = b - a
    denom = ds[0] * dc[1] - ds[1] * dc[0]
    if abs(denom) < 1e-18:
        return np.asarray(s, dtype=np.float64)
    t = ((a[0] - s[0]) * dc[1] - (a[1] - s[1]) * dc[0]) / denom
    return s + t * ds


def _clip_polygon(subject, clipper):
    """Sutherland–Hodgman clip of a convex subject against a convex CCW clipper."""
    output = [np.asarray(p, dtype=np.float64) for p in subject]
    n_clip = len(clipper)
    for i in range(n_clip):
        if not output:
            break
        a = clipper[i]
        b = clipper[(i + 1) % n_clip]
        inp = output
        output = []
        s = inp[-1]
        for e in inp:
            e_in = _is_left_or_on(e, a, b)
            s_in = _is_left_or_on(s, a, b)
            if e_in:
                if not s_in:
                    output.append(_intersect(s, e, a, b))
                output.append(np.asarray(e, dtype=np.float64))
            elif s_in:
                output.append(_intersect(s, e, a, b))
            s = e
    if not output:
        return np.zeros((0, 2), dtype=np.float64)
    return np.vstack(output)


def polygon_area(poly):
    """Absolute area of a simple polygon given as (N, 2) vertices."""
    return abs(_shoelace(np.asarray(poly, dtype=np.float64)))


def footprint_iou(
        x0, y0, yaw0, width0, depth0,
        x1, y1, yaw1, width1, depth1):
    """Intersection-over-union of two oriented XY rectangles.

    180° flips and 90°+W/D swaps of the same physical box score ~1 because
    the occupied footprints match. A true 90° rotation *without* a size swap
    scores low.
    """
    if min(abs(width0), abs(depth0), abs(width1), abs(depth1)) < _EPS:
        return 0.0
    a = _ensure_ccw(oriented_footprint_corners(x0, y0, yaw0, width0, depth0))
    b = _ensure_ccw(oriented_footprint_corners(x1, y1, yaw1, width1, depth1))
    area_a = polygon_area(a)
    area_b = polygon_area(b)
    if area_a < _EPS or area_b < _EPS:
        return 0.0
    inter_poly = _clip_polygon(a, b)
    inter = polygon_area(inter_poly)
    union = area_a + area_b - inter
    if union < _EPS:
        return 1.0
    return float(max(0.0, min(1.0, inter / union)))
