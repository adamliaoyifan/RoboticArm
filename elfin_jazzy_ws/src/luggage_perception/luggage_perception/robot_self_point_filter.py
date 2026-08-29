#!/usr/bin/env python3
"""Geometric robot self-filter for depth point clouds in base frame.

Independent preprocessing module: removes points belonging to the robot arm,
suction panel, connector, and camera. Does not manage box instances or mapper state.

Pure Python (no rospy). Callers pass a tf2_ros.Buffer (or compatible mock) for
per-link pose lookup.
"""

from __future__ import division

import math


def _quat_conj(qx, qy, qz, qw):
    return -qx, -qy, -qz, qw


def _quat_rotate(qx, qy, qz, qw, vx, vy, vz):
    """Rotate vector v by quaternion q (Hamilton convention)."""
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def _point_in_box(local_x, local_y, local_z, cx, cy, cz, hx, hy, hz):
    return (
        abs(local_x - cx) <= hx
        and abs(local_y - cy) <= hy
        and abs(local_z - cz) <= hz
    )


def _point_in_sphere(local_x, local_y, local_z, cx, cy, cz, radius):
    dx = local_x - cx
    dy = local_y - cy
    dz = local_z - cz
    return dx * dx + dy * dy + dz * dz <= radius * radius


def _validate_body(body, default_padding):
    if not isinstance(body, dict):
        raise ValueError("body must be a dict")
    name = body.get("name")
    frame = body.get("frame")
    btype = body.get("type")
    if not name or not frame or not btype:
        raise ValueError("body requires name, frame, type: %r" % body)
    center = body.get("center", [0.0, 0.0, 0.0])
    if len(center) != 3:
        raise ValueError("body %s center must have 3 elements" % name)
    padding = float(body.get("padding", default_padding))
    if btype == "sphere":
        radius = float(body.get("radius", 0.1)) + padding
        return {
            "name": str(name),
            "frame": str(frame),
            "type": "sphere",
            "center": [float(center[0]), float(center[1]), float(center[2])],
            "radius": radius,
        }
    if btype == "box":
        size = body.get("size", [0.1, 0.1, 0.1])
        if len(size) != 3:
            raise ValueError("body %s size must have 3 elements" % name)
        half = [float(size[0]) * 0.5 + padding,
                float(size[1]) * 0.5 + padding,
                float(size[2]) * 0.5 + padding]
        return {
            "name": str(name),
            "frame": str(frame),
            "type": "box",
            "center": [float(center[0]), float(center[1]), float(center[2])],
            "half_size": half,
        }
    raise ValueError("body %s unsupported type %r" % (name, btype))


def bodies_from_config(config):
    """Parse self_filter section from a YAML-loaded dict."""
    section = config.get("self_filter", config)
    padding = float(section.get("padding", 0.03))
    raw_bodies = section.get("bodies", [])
    return [_validate_body(b, padding) for b in raw_bodies]


class RobotSelfPointFilter:
    """Filter robot self points from base-frame (x, y, z) lists."""

    def __init__(
        self, enabled=True, base_frame="elfin_base_link", bodies=None,
        padding=0.03, allow_latest_fallback=True,
    ):
        self.enabled = bool(enabled)
        self.base_frame = str(base_frame)
        self.padding = float(padding)
        self.allow_latest_fallback = bool(allow_latest_fallback)
        self.bodies = []
        if bodies:
            for b in bodies:
                self.bodies.append(_validate_body(b, self.padding))
        self._last_stats = self._empty_stats()

    @classmethod
    def from_config_dict(cls, config):
        section = config.get("self_filter", config)
        return cls(
            enabled=bool(section.get("enabled", True)),
            base_frame=str(section.get("base_frame", "elfin_base_link")),
            bodies=section.get("bodies", []),
            padding=float(section.get("padding", 0.03)),
            allow_latest_fallback=bool(
                section.get("allow_latest_tf_fallback", True)
            ),
        )

    @classmethod
    def load_yaml(cls, path):
        import yaml
        with open(path, "r") as handle:
            data = yaml.safe_load(handle) or {}
        return cls.from_config_dict(data)

    @property
    def last_stats(self):
        return dict(self._last_stats)

    def _empty_stats(self):
        return {
            "enabled": self.enabled,
            "raw_count": 0,
            "filtered_count": 0,
            "dropped_self": 0,
            "tf_missing_links": [],
        }

    def _lookup_transform(self, tf_buffer, frame, stamp):
        """Return (tx, ty, tz, qx, qy, qz, qw) base->frame or None."""
        try:
            import rospy
            timeout = rospy.Duration(0.05)
        except ImportError:
            timeout = None

        lookup_args = (self.base_frame, frame, stamp)
        try:
            if timeout is not None:
                transform = tf_buffer.lookup_transform(*lookup_args, timeout)
            else:
                transform = tf_buffer.lookup_transform(*lookup_args)
        except Exception:
            if stamp is not None and self.allow_latest_fallback:
                try:
                    import rospy
                    latest = rospy.Time(0)
                except ImportError:
                    latest = None
                if latest is not None and stamp != latest:
                    try:
                        if timeout is not None:
                            transform = tf_buffer.lookup_transform(
                                self.base_frame, frame, latest, timeout
                            )
                        else:
                            transform = tf_buffer.lookup_transform(
                                self.base_frame, frame, latest
                            )
                    except Exception:
                        return None
                else:
                    return None
            else:
                return None

        t = transform.transform.translation
        r = transform.transform.rotation
        return (t.x, t.y, t.z, r.x, r.y, r.z, r.w)

    def _base_to_local(self, px, py, pz, tx, ty, tz, qx, qy, qz, qw):
        dx = px - tx
        dy = py - ty
        dz = pz - tz
        qcx, qcy, qcz, qcw = _quat_conj(qx, qy, qz, qw)
        return _quat_rotate(qcx, qcy, qcz, qcw, dx, dy, dz)

    def _point_inside_body(self, lx, ly, lz, body, extra_padding=0.0):
        cx, cy, cz = body["center"]
        padding = max(0.0, float(extra_padding))
        if body["type"] == "sphere":
            return _point_in_sphere(
                lx, ly, lz, cx, cy, cz, body["radius"] + padding)
        half = body["half_size"]
        return _point_in_box(
            lx, ly, lz, cx, cy, cz,
            half[0] + padding, half[1] + padding, half[2] + padding,
        )

    def point_intersects_robot(
            self, point, tf_buffer, stamp=None, extra_padding=0.0):
        """Return ``(intersects, missing_frames)`` for one base-frame point.

        ``extra_padding`` expands every configured body. Callers representing
        an obstacle volume can pass its bounding-sphere radius, preventing a
        box whose center is just outside a robot body from overlapping it.
        Missing transforms are explicit so safety-critical callers can fail
        closed instead of silently accepting an unclassified obstacle.
        """
        if not self.enabled or not self.bodies:
            return False, []
        px, py, pz = point
        if not all(math.isfinite(v) for v in (px, py, pz)):
            return False, []
        missing = []
        transforms = {}
        for body in self.bodies:
            frame = body["frame"]
            if frame not in transforms:
                transforms[frame] = self._lookup_transform(
                    tf_buffer, frame, stamp)
            tf = transforms[frame]
            if tf is None:
                missing.append(frame)
                continue
            lx, ly, lz = self._base_to_local(px, py, pz, *tf)
            if self._point_inside_body(
                    lx, ly, lz, body, extra_padding=extra_padding):
                return True, sorted(set(missing))
        return False, sorted(set(missing))

    def filter_points(self, points, tf_buffer, stamp=None):
        """Return filtered copy of base-frame points; drop those on robot links."""
        filtered, _kept_indices = self.filter_points_with_indices(
            points, tf_buffer, stamp=stamp
        )
        return filtered

    def filter_points_with_indices(self, points, tf_buffer, stamp=None):
        """Return ``(filtered_points, original_indices)`` for metadata alignment."""
        raw_count = len(points)
        if not self.enabled or not self.bodies or raw_count == 0:
            self._last_stats = {
                "enabled": self.enabled,
                "raw_count": raw_count,
                "filtered_count": raw_count,
                "dropped_self": 0,
                "tf_missing_links": [],
            }
            return list(points), list(range(raw_count))

        transforms = {}
        tf_missing = []
        for body in self.bodies:
            frame = body["frame"]
            if frame in transforms:
                continue
            tf = self._lookup_transform(tf_buffer, frame, stamp)
            if tf is None:
                tf_missing.append(frame)
            transforms[frame] = tf

        result = []
        kept_indices = []
        dropped = 0
        for point_index, (px, py, pz) in enumerate(points):
            if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(pz)):
                result.append((px, py, pz))
                kept_indices.append(point_index)
                continue
            inside = False
            for body in self.bodies:
                tf = transforms.get(body["frame"])
                if tf is None:
                    continue
                lx, ly, lz = self._base_to_local(px, py, pz, *tf)
                if self._point_inside_body(lx, ly, lz, body):
                    inside = True
                    break
            if inside:
                dropped += 1
            else:
                result.append((px, py, pz))
                kept_indices.append(point_index)

        self._last_stats = {
            "enabled": True,
            "raw_count": raw_count,
            "filtered_count": len(result),
            "dropped_self": dropped,
            "tf_missing_links": sorted(set(tf_missing)),
        }
        return result, kept_indices
