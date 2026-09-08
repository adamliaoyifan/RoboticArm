#!/usr/bin/env python3
"""Platform-free detection pipeline gating (no ROS).

Owns the E2 policy that sits between the pure estimators
(:mod:`luggage_perception.top_support_estimator`) and the ROS detector
adapter:

- only a fresh ``measure`` cargo cloud may create FULL_3D; a
  ``hold_track`` cloud must not be fused with a new raw cloud;
- support fitting requires settled input (preprocessor ``geometry_ok``)
  and a raw cloud from the *same* acquisition stamp;
- support modes: ``auto``, ``configured``, ``auto_then_configured``,
  ``top_only``;
- a catalog width/depth match may populate a prior height, but the result
  stays ``height_valid=false`` (HEIGHT_SOURCE_CATALOG_PRIOR).

Stateful temporal filtering delegates to
:class:`~luggage_perception.top_support_estimator.SupportStabilityFilter`
(update/copy_output ownership pattern).
"""

from __future__ import division

import math
import time
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

from luggage_perception.top_support_estimator import (
    DETECT_CARGO_SEGMENTATION_REQUIRED,
    DETECT_SUPPORT_STAMP_MISMATCH,
    DETECT_SUPPORT_STATUS_MALFORMED,
    DETECT_SUPPORT_STATUS_MISSING,
    DETECT_SUPPORT_STATUS_STALE,
    DETECT_TOP_UNOBSERVABLE,
    HEIGHT_SOURCE_UNAVAILABLE,
    SupportPlaneEstimate,
    SupportStabilityFilter,
    TopSupportConfig,
    compose_box_geometry,
    estimate_local_support,
    estimate_top_surface,
)

SUPPORT_MODES = ("auto", "configured", "auto_then_configured", "top_only")

# Support-gate label -> machine-readable support reason (PF-R3): gates that
# skip support fitting still publish *why*, so TOP_ONLY frames are
# debuggable and no gate can masquerade as another.
GATE_SUPPORT_REASONS = {
    "hold_track": DETECT_SUPPORT_STAMP_MISMATCH,
    "geometry_not_settled": "DETECT_SUPPORT_UNSTABLE",
    "raw_stamp_mismatch": DETECT_SUPPORT_STAMP_MISMATCH,
    "status_missing": DETECT_SUPPORT_STATUS_MISSING,
    "status_malformed": DETECT_SUPPORT_STATUS_MALFORMED,
    "status_stale": DETECT_SUPPORT_STATUS_STALE,
    "cargo_unsegmented": DETECT_CARGO_SEGMENTATION_REQUIRED,
    "mode_top_only": "DETECT_SUPPORT_UNOBSERVABLE",
    "mode_configured": "DETECT_SUPPORT_UNOBSERVABLE",
    "no_top": "DETECT_SUPPORT_UNOBSERVABLE",
}


class GeometryStatusGate:
    """Same-acquisition validation of preprocessor status (PF-R3 rework).

    The preprocessor publishes one JSON payload per acquisition whose
    ``flags.geometry_ok`` is evidence for that acquisition only — the
    robot can start moving between frames, so status for acquisition
    N-1 must never authorize support fitting for acquisition N. Payloads
    are buffered keyed by their exact ``(sec, nanosec)`` primary stamp
    (integer fields when the producer provides them; float64 fallback
    with a 1 us epsilon, which only covers cross-producer float
    representation error — float64 ulp at the ROS epoch is ~0.24 us —
    never a frame period).

    Fail-closed outcomes, each with its own machine reason:

    - exact-stamp entry with ``geometry_ok=true``: support may run;
    - entry with ``geometry_ok=false``: ``geometry_not_settled``;
    - no entry for this stamp and the buffer's newest stamp is older:
      ``status_stale`` (status stream lags this acquisition — includes
      the N-1 case);
    - no entry and the newest buffered stamp is newer (out-of-order
      cloud) or the buffer is empty: ``status_missing``;
    - most recently received payload unusable: ``status_malformed``.
    """

    #: float-fallback tolerance: covers float64 representation error
    #: only (~0.24 us at the ROS epoch), not any frame period.
    STAMP_EPSILON_SEC = 1e-6

    def __init__(self, maxlen=16):
        self.maxlen = max(1, int(maxlen))
        self._entries = {}     # (sec, nanosec) -> geometry_ok bool
        self._order = []       # receipt order of keys
        self._last_malformed = False

    def update(self, payload, receipt_sec=None):
        """Buffer one status payload by its stamped acquisition.

        Malformed payloads (wrong shape, no usable stamp/flag) have no
        key and only set the malformed marker, so they can never
        authorize anything.
        """
        del receipt_sec  # receipt time is not evidence; the stamp is
        self._last_malformed = False
        if not isinstance(payload, dict):
            self._last_malformed = True
            return
        flags = payload.get("flags")
        if not isinstance(flags, dict) or "geometry_ok" not in flags:
            self._last_malformed = True
            return
        key = self._stamp_key(payload)
        if key is None:
            self._last_malformed = True
            return
        if key not in self._entries:
            self._order.append(key)
        self._entries[key] = bool(flags["geometry_ok"])
        while len(self._order) > self.maxlen:
            evicted = self._order.pop(0)
            self._entries.pop(evicted, None)

    @staticmethod
    def _stamp_key(payload):
        """Exact (sec, nanosec) key, or None when the payload lacks one."""
        sec = payload.get("primary_stamp_sec")
        nanosec = payload.get("primary_stamp_nanosec")
        if isinstance(sec, int) and isinstance(nanosec, int):
            return (sec, nanosec)
        stamp = payload.get("primary_stamp")
        if isinstance(stamp, (int, float)) and math.isfinite(float(stamp)):
            whole = int(stamp)
            ns = int(round((float(stamp) - whole) * 1e9))
            if ns >= 1000000000:
                whole += 1
                ns -= 1000000000
            return (whole, ns)
        return None

    def evaluate(self, cloud_stamp_sec, cloud_stamp_nanosec=None):
        """Validate status evidence for one cloud acquisition.

        ``cloud_stamp_nanosec`` gives the exact integer stamp when the
        caller has it (it always does: the ROS header). Returns
        ``(ok, reason)``.
        """
        if cloud_stamp_nanosec is not None:
            key = (int(cloud_stamp_sec), int(cloud_stamp_nanosec))
            match = key if key in self._entries else None
        else:
            match = None
            try:
                stamp = float(cloud_stamp_sec)
            except (TypeError, ValueError):
                return False, "status_malformed"
            for candidate in self._entries:
                if abs((candidate[0] + candidate[1] * 1e-9) - stamp) \
                        <= self.STAMP_EPSILON_SEC:
                    match = candidate
                    break
        if match is not None:
            if self._entries[match]:
                return True, ""
            return False, "geometry_not_settled"
        if self._last_malformed:
            return False, "status_malformed"
        if self._entries:
            newest = max(
                k[0] + k[1] * 1e-9 for k in self._entries)
            try:
                cloud = float(cloud_stamp_sec) + (
                    0.0 if cloud_stamp_nanosec is None
                    else 1e-9 * float(cloud_stamp_nanosec))
            except (TypeError, ValueError):
                return False, "status_malformed"
            if newest < cloud - self.STAMP_EPSILON_SEC:
                return False, "status_stale"
        return False, "status_missing"


@dataclass
class PipelineResult:
    """One acquisition's platform-free detection outcome."""

    top_valid: bool
    top_reason: str
    support: Optional[SupportPlaneEstimate]
    height_valid: bool
    height_source: int
    box: object = None            # BoxGeometryEstimate
    support_gate: str = ""        # why support fitting was skipped
    height_selected_by: str = ""  # mode fallback trace (auto_then_configured)
    timing: dict = field(default_factory=dict)


def _catalog_prior_height(width, depth, catalog_entries, tolerance):
    """Best width/depth catalog match -> prior height, or None.

    Height is deliberately not part of the metric: it is the unknown we
    are trying to prior-ize.
    """
    best = None
    best_err = float("inf")
    for entry in catalog_entries or []:
        ew, ed, eh = entry["size"]
        for cw, cd in ((ew, ed), (ed, ew)):
            err = abs(cw - width) + abs(cd - depth)
            if (err < best_err and abs(cw - width) < tolerance
                    and abs(cd - depth) < tolerance):
                best_err = err
                best = float(eh)
    return best


class PlatformFreeDetector:
    """Stateful gate + estimator composition for one luggage instance."""

    def __init__(self, config=None, support_mode="auto",
                 catalog_entries=None, catalog_tolerance=0.08,
                 stability_window=5, stability_max_z_spread=0.015):
        if support_mode not in SUPPORT_MODES:
            raise ValueError(
                "support_mode must be one of %s" % (SUPPORT_MODES,))
        self.config = config or TopSupportConfig()
        self.support_mode = support_mode
        self.catalog_entries = list(catalog_entries or [])
        self.catalog_tolerance = float(catalog_tolerance)
        self._stability = SupportStabilityFilter(
            window=stability_window, max_z_spread=stability_max_z_spread)

    def reset(self):
        """Drop temporal state (new luggage instance spawned)."""
        self._stability.update(None)

    def update(self, cargo_points_world, raw_points_world, *,
               source="measure", geometry_ok=True,
               raw_same_stamp=True, platform_z=None, stamp_sec=0.0,
               cargo_segmented=True, geometry_gate_reason=None):
        """Run one acquisition through the gate.

        ``raw_points_world`` is the preprocessed raw depth cloud decoded
        into the world frame; ``raw_same_stamp`` says it carries the same
        acquisition stamp as the cargo cloud. ``cargo_segmented=False``
        means ``cargo_points_world`` is an unsegmented raw depth cloud:
        the update fails closed with
        ``DETECT_CARGO_SEGMENTATION_REQUIRED`` before any fitting, in
        every support mode (PF-R2 — the dominant platform plane must
        never become a valid luggage top). ``geometry_gate_reason`` is
        the :class:`GeometryStatusGate` label when the status evidence
        was missing/malformed/stale (PF-R3): it overrides the boolean
        and publishes its own support reason. Returns
        :class:`PipelineResult`.
        """
        result = PipelineResult(
            top_valid=False, top_reason=DETECT_TOP_UNOBSERVABLE,
            support=None, height_valid=False,
            height_source=HEIGHT_SOURCE_UNAVAILABLE)
        timing = result.timing
        _t_total = time.monotonic()

        if not cargo_segmented:
            result.top_reason = DETECT_CARGO_SEGMENTATION_REQUIRED
            result.support_gate = "cargo_unsegmented"
            self._stability.update(None)
            return result

        points = np.asarray(
            cargo_points_world if cargo_points_world is not None else [],
            dtype=np.float64).reshape(-1, 3)
        n_points = int(len(points))
        if n_points < int(self.config.min_top_points):
            result.top_reason = "DETECT_TOO_FEW_POINTS"
            self._stability.update(None)
            return result

        top = estimate_top_surface(
            points, _workspace_pair(self.config), self.config,
            timing=timing)
        if top is None:
            result.top_reason = DETECT_TOP_UNOBSERVABLE
            self._stability.update(None)
            return result
        top.stamp = float(stamp_sec)
        result.top_valid = True
        result.top_reason = "ok"

        _t0 = time.monotonic()
        support, gate = self._fit_support(
            top, raw_points_world, source, geometry_ok, raw_same_stamp,
            stamp_sec, geometry_gate_reason, timing=timing)
        timing["support_gate_total_ms"] = (
            time.monotonic() - _t0) * 1000.0
        result.support = support
        result.support_gate = gate

        _t0 = time.monotonic()
        box = self._compose(top, support, platform_z)
        timing["compose_ms"] = (time.monotonic() - _t0) * 1000.0
        if (support is None and not box.height_valid
                and result.support_gate in GATE_SUPPORT_REASONS):
            # Surface the gate's own reason (status_missing/stale/...),
            # not the generic UNOBSERVABLE default.
            box.reason = GATE_SUPPORT_REASONS[result.support_gate]
        result.box = box
        result.height_valid = bool(box.height_valid)
        result.height_source = int(box.height_source)
        timing["pipeline_total_ms"] = (
            time.monotonic() - _t_total) * 1000.0
        return result

    def _fit_support(self, top, raw_points_world, source, geometry_ok,
                     raw_same_stamp, stamp_sec, geometry_gate_reason=None,
                     timing=None):
        """Gate + fit the local support plane. Returns (support, gate).

        Stability-window policy (PF-R5 field evidence): only *positive
        motion evidence* (``geometry_not_settled``) and unusable fitted
        supports reset the window — the scene may have moved, so past
        measurements are no longer valid. Absence of evidence
        (missing/malformed/stale status, raw stamp mismatch, hold_track)
        adds no measurement and must not reset: those misses were
        callback-ordering artifacts that cleared the window several times
        per trial and starved FULL_3D. The window still only ever
        contains *fitted, same-instance* support Zs.
        """
        if self.support_mode == "top_only":
            self._stability.update(None)
            return None, "mode_top_only"
        if self.support_mode == "configured":
            self._stability.update(None)
            return None, "mode_configured"
        if source != "measure":
            # hold_track cargo is an earlier acquisition; fusing it with a
            # new raw cloud would fake a measured height.
            return None, "hold_track"
        if geometry_gate_reason in ("status_missing", "status_malformed",
                                    "status_stale"):
            # PF-R3: status evidence does not cover this acquisition; the
            # frame stays TOP_ONLY with the distinct machine reason.
            return None, geometry_gate_reason
        if not geometry_ok:
            self._stability.update(None)
            return None, "geometry_not_settled"
        if raw_points_world is None or not raw_same_stamp:
            # No evidence about the support this frame (buffer race);
            # skip the window rather than reset it.
            return (
                SupportPlaneEstimate(
                    support_z=float("nan"),
                    stamp=float(stamp_sec),
                    reason=DETECT_SUPPORT_STAMP_MISMATCH),
                "raw_stamp_mismatch")
        support = estimate_local_support(
            np.asarray(raw_points_world, dtype=np.float64).reshape(-1, 3),
            top, _workspace_pair(self.config), self.config,
            timing=timing)
        support.stamp = float(stamp_sec)
        if support.reason != "ok":
            # Per-frame rejection (coverage/sides) must not enter the Z
            # stability window: a filled window would otherwise flip it
            # to "ok" and fake a measured height.
            self._stability.update(None)
            return support, ""
        filtered = self._stability.update(support)
        if filtered is not None:
            filtered.stamp = float(stamp_sec)
        return (filtered if filtered is not None else support), ""

    def _compose(self, top, support, platform_z):
        """Apply the support-mode fallback chain and catalog prior."""
        support_ok = (
            support is not None and support.reason == "ok"
            and np.isfinite(support.support_z))
        if self.support_mode == "configured" or (
                self.support_mode == "auto_then_configured"
                and not support_ok):
            if platform_z is not None and np.isfinite(float(platform_z)):
                box = compose_box_geometry(top, None, platform_z=platform_z)
                return box
        prior = _catalog_prior_height(
            top.width, top.depth, self.catalog_entries,
            self.catalog_tolerance)
        return compose_box_geometry(
            top, support if support_ok else None,
            catalog_height=prior)


def _workspace_pair(config):
    return (config.workspace_center_xy, config.workspace_half_extents)
