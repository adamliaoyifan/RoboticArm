#!/usr/bin/env python3
"""ROS-independent joint-motion gate for eye-in-hand point clouds."""

from __future__ import division

import math


VELOCITY = "velocity"
DISPLACEMENT = "displacement"
CRITERIA = (VELOCITY, DISPLACEMENT)


class MotionStabilityGate:
    """Track moving/settling/stable state from timestamped joint samples."""

    def __init__(
        self,
        joint_names=None,
        velocity_threshold=0.02,
        settle_time_sec=0.5,
        joint_state_timeout_sec=1.0,
        enabled=True,
        criterion=DISPLACEMENT,
    ):
        self.joint_names = list(joint_names or [])
        self.velocity_threshold = abs(float(velocity_threshold))
        self.settle_time_sec = max(0.0, float(settle_time_sec))
        self.displacement_tolerance = (
            self.velocity_threshold * self.settle_time_sec)
        self.joint_state_timeout_sec = max(0.0, float(joint_state_timeout_sec))
        self.enabled = bool(enabled)
        self.criterion = str(criterion).strip().lower()
        if self.criterion not in CRITERIA:
            raise ValueError(
                "motion stability criterion must be one of %s, got %r"
                % (", ".join(CRITERIA), criterion))
        self._last_positions = {}
        self._last_sample_stamp = None
        self._last_update_time = None
        self._settling_since = None
        self._stable_since = None
        self._max_velocity = None
        self._position_window = []
        self._peak_excursion = 0.0
        self._excursion_joint = ""
        self._state = "disabled" if not self.enabled else "unknown"

    @staticmethod
    def _stamp_seconds(stamp):
        if stamp is None:
            return None
        if hasattr(stamp, "to_sec"):
            return float(stamp.to_sec())
        return float(stamp)

    def _selected_names(self, names):
        available = set(names)
        if self.joint_names:
            if not set(self.joint_names).issubset(available):
                return None
            return list(self.joint_names)
        return list(names)

    def _invalidate(self, now_sec=None):
        self._state = "unknown"
        self._stable_since = None
        self._settling_since = None
        self._position_window = []
        self._peak_excursion = 0.0
        self._excursion_joint = ""
        self._last_positions = {}
        self._last_sample_stamp = None
        if now_sec is not None:
            self._last_update_time = now_sec
        return self._state

    def _window_excursion(self):
        extremes = {}
        for _sample_time, positions in self._position_window:
            for name, value in positions.items():
                low, high = extremes.get(name, (value, value))
                extremes[name] = (min(low, value), max(high, value))
        worst = 0.0
        worst_joint = ""
        for name, (low, high) in extremes.items():
            span = high - low
            if span > worst:
                worst = span
                worst_joint = name
        return worst, worst_joint

    def _update_displacement(self, positions, now_sec):
        self._position_window.append((now_sec, dict(positions)))
        cutoff = now_sec - self.settle_time_sec
        # Keep the last sample on or before the cutoff. Without that boundary
        # sample, sparse JointState timing could make the observed window
        # shorter than settle_time_sec and allow an early stable verdict.
        while (
                len(self._position_window) > 1
                and self._position_window[1][0] <= cutoff):
            self._position_window.pop(0)

        self._peak_excursion, self._excursion_joint = self._window_excursion()
        observed_span = now_sec - self._position_window[0][0]
        if observed_span < self.settle_time_sec:
            self._state = "settling"
            self._stable_since = None
            return self._state

        if self._peak_excursion >= self.displacement_tolerance:
            self._state = "moving"
            self._stable_since = None
            return self._state

        self._state = "stable"
        if self._stable_since is None:
            self._stable_since = now_sec
        return self._state

    def _update_velocity(self, now_sec):
        if self._max_velocity > self.velocity_threshold:
            self._state = "moving"
            self._settling_since = None
            self._stable_since = None
            return self._state

        if self._settling_since is None:
            self._settling_since = now_sec
        if now_sec - self._settling_since >= self.settle_time_sec:
            self._state = "stable"
            if self._stable_since is None:
                self._stable_since = now_sec
        else:
            self._state = "settling"
            self._stable_since = None
        return self._state

    def update(self, names, positions, velocities=None, stamp=None, now=None):
        """Update state. Returns the current state string."""
        if not self.enabled:
            self._state = "disabled"
            return self._state

        names = list(names or [])
        positions = list(positions or [])
        velocities = list(velocities or [])
        selected = self._selected_names(names)
        stamp_sec = self._stamp_seconds(stamp)
        now_sec = self._stamp_seconds(now)
        if now_sec is None:
            now_sec = stamp_sec
        if (
            selected is None
            or not selected
            or len(positions) < len(names)
            or stamp_sec is None
            or now_sec is None
        ):
            return self._invalidate(now_sec)
        if self._last_update_time is not None and now_sec < self._last_update_time:
            return self._invalidate(now_sec)

        index = {name: i for i, name in enumerate(names)}
        measured = []
        current_positions = {}
        have_reported_velocity = len(velocities) >= len(names)
        dt = None
        if self._last_sample_stamp is not None:
            dt = stamp_sec - self._last_sample_stamp

        for name in selected:
            i = index[name]
            position = float(positions[i])
            if not math.isfinite(position):
                return self._invalidate(now_sec)
            current_positions[name] = position
            if have_reported_velocity:
                velocity = abs(float(velocities[i]))
            elif dt is not None and dt > 1e-6 and name in self._last_positions:
                velocity = abs(position - self._last_positions[name]) / dt
            else:
                velocity = None
            if velocity is not None and not math.isfinite(velocity):
                if self.criterion == VELOCITY:
                    return self._invalidate(now_sec)
                velocity = None
            if velocity is not None:
                measured.append(velocity)
            self._last_positions[name] = position

        self._last_sample_stamp = stamp_sec
        self._last_update_time = now_sec
        self._max_velocity = max(measured) if measured else None
        if self.criterion == DISPLACEMENT:
            return self._update_displacement(current_positions, now_sec)
        if len(measured) != len(selected):
            # The first position-only sample cannot produce a velocity yet,
            # but it must remain available so the next sample can estimate it.
            self._state = "unknown"
            self._stable_since = None
            self._settling_since = None
            return self._state
        return self._update_velocity(now_sec)

    def state(self, now=None):
        if not self.enabled:
            return "disabled"
        now_sec = self._stamp_seconds(now)
        if (
            now_sec is not None
            and self._last_update_time is not None
            and self.joint_state_timeout_sec > 0.0
            and now_sec - self._last_update_time > self.joint_state_timeout_sec
        ):
            return "stale"
        return self._state

    def accepts_cloud(self, cloud_stamp, now=None):
        if not self.enabled:
            return True
        if self.state(now) != "stable" or self._stable_since is None:
            return False
        stamp_sec = self._stamp_seconds(cloud_stamp)
        if stamp_sec is None:
            return False
        # A zero stamp (uninitialized header) would always be < stable_since
        # and silently starve the pipeline. Treat it as "now" so a current
        # cloud captured while stable is still accepted.
        if stamp_sec == 0.0:
            now_sec = self._stamp_seconds(now)
            if now_sec is not None:
                stamp_sec = now_sec
        return stamp_sec >= self._stable_since

    @property
    def stable_since(self):
        return self._stable_since

    @property
    def max_velocity(self):
        return self._max_velocity

    @property
    def peak_excursion(self):
        return self._peak_excursion

    def diagnostics(self, now=None):
        return {
            "enabled": self.enabled,
            "criterion": self.criterion,
            "state": self.state(now),
            "stable_since": self._stable_since or 0.0,
            "max_velocity": self._max_velocity if self._max_velocity is not None else -1.0,
            "velocity_threshold": self.velocity_threshold,
            "peak_excursion": self._peak_excursion,
            "displacement_tolerance": self.displacement_tolerance,
            "excursion_joint": self._excursion_joint,
        }


def status_geometry_stable(data):
    """True when preprocessor status says the arm is settled for geometry.

    ``geometry_ok`` is the motion-gate verdict. ``motion_gate.state`` must be
    ``stable`` (or ``disabled`` when the gate is turned off in yaml).
    """
    if not isinstance(data, dict):
        return False
    flags = data.get("flags") or {}
    if not bool(flags.get("geometry_ok")):
        return False
    state = (data.get("motion_gate") or {}).get("state")
    return state in ("stable", "disabled")


def detection_replay_fields(status_data, cloud_stamp_sec, now_sec, clock_type):
    """Stamp fields for ``/luggage/perception/detection/latest``.

    Ages are node-clock minus cloud header, not status recv time. The 1 Hz
    preprocessor timer republishes the same ``primary_stamp``.
    """
    geometry_ok_stamp = 0.0
    if isinstance(status_data, dict):
        try:
            geometry_ok_stamp = float(
                status_data.get("last_geometry_ok_stamp") or 0.0)
        except (TypeError, ValueError):
            geometry_ok_stamp = 0.0
    fields = {
        "geometry_ok_stamp": geometry_ok_stamp,
        "clock_type": str(clock_type),
    }
    if cloud_stamp_sec is None:
        return fields
    cloud_stamp = float(cloud_stamp_sec)
    fields["cloud_stamp"] = cloud_stamp
    fields["cloud_age_sec"] = float(now_sec) - cloud_stamp
    return fields
