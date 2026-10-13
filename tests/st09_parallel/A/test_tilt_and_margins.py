"""ST09-A acceptance: unsupported-tilt rejection and named margins.

A gentle support ramp rises less than ``support_tol`` per cell, so the
support ratio reads it as full contact; the contact-cell height span over
the footprint axis exposes it and the candidate is rejected as
``support_tilt_unsupported``. Margins are named per class in every
candidate's detail (static walls incl. the chamfer face / top clearance /
support contact), each with its own parameter source.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from helpers import (
    BOX_WDH,
    PARAMS_FULL,
    chamfered_surface,
    surface_height_fn,
)

from luggage_packing.placement_solver import (
    REASON_SUPPORT_TILT_UNSUPPORTED,
    generate_candidates,
    solve_placement,
)


def ramp_surface(period=13, total_span=0.04):
    """Sawtooth x-ramp: each footprint-length window phases through a full
    0.04 m rise, so every cell stays within ``support_tol`` of the window
    peak (support_ratio reads full contact) while the contact plane slopes
    across the footprint — the box would tilt on it."""
    surface = chamfered_surface()
    step = total_span / (period - 1)
    for ix in range(surface["nx"]):
        for iy in range(surface["ny"]):
            surface["height"][ix][iy] = (ix % period) * step
    return surface


class TestUnsupportedTilt(object):
    def test_ramp_rejected_as_tilt_not_full_support(self):
        candidates = generate_candidates(
            ramp_surface(), list(BOX_WDH), allowed_yaws=[0.0],
            params=PARAMS_FULL)
        assert candidates
        rejected = [
            c for c in candidates
            if c["reason"] == REASON_SUPPORT_TILT_UNSUPPORTED]
        assert rejected, "the ramp must be exposed as tilt"
        for candidate in rejected:
            assert candidate["feasible"] is False
            assert candidate["verdict"] == "support_tilt_unsupported"
            assert candidate["support_score"] >= 0.6, (
                "the ramp reads as full support by ratio — that is exactly "
                "why the tilt span gate exists")
            detail = candidate["detail"]["tilt"]
            assert detail["support_span_m"] > 0.0
            assert detail["support_tilt_ratio"] > detail[
                "max_support_tilt_ratio"]

    def test_flat_support_passes_with_zero_tilt(self):
        surface = chamfered_surface()
        result = solve_placement(surface, list(BOX_WDH), params=PARAMS_FULL)
        assert result["success"] is True
        selected = result["selected"]
        assert selected["reason"] == "ok"
        assert selected["detail"]["tilt"]["support_span_m"] == 0.0

    def test_moderate_step_within_threshold_is_not_tilt(self):
        """A support span under the ratio threshold (0.02 m over 0.646 m =
        0.031 < 0.05) stays legal support."""
        surface = chamfered_surface()
        for ix in range(surface["nx"]):
            for iy in range(surface["ny"]):
                surface["height"][ix][iy] = (
                    0.02 if iy >= surface["ny"] // 2 else 0.0)
        candidates = generate_candidates(
            surface, list(BOX_WDH), allowed_yaws=[0.0], params=PARAMS_FULL)
        feasible = [c for c in candidates if c["feasible"]]
        assert feasible, "sub-threshold spans remain legal support"


class TestNamedMargins(object):
    def test_margins_named_per_class_in_detail(self):
        surface = chamfered_surface()
        candidates = generate_candidates(
            surface, list(BOX_WDH), allowed_yaws=[0.0], params=PARAMS_FULL)
        for candidate in candidates[:20]:
            margins = candidate["detail"]["margins"]
            assert set(margins) >= {
                "hull_static_wall_and_chamfer_m",
                "top_clearance_m",
                "support_contact_tol_m",
            }
            assert margins["top_clearance_m"] == 0.03
            assert margins["support_contact_tol_m"] == 0.05

    def test_hull_margin_parameter_reaches_the_kernel_gate(self):
        """A box face exactly on the flat wall is legal at margin 0 and
        illegal with a 2 cm wall margin — the recorded margin is the one the
        kernel enforced."""
        surface = chamfered_surface()
        # center the deepest window band so the +X face touches the wall:
        # yaw0 footprint half-length 0.325, wall at +0.745 -> center 0.42
        exact = solve_placement(
            surface, [0.646, 0.409, 0.277], allowed_yaws=[0.0],
            params=dict(PARAMS_FULL, hull_margin=0.0))
        assert exact["success"] is True
        widened = solve_placement(
            surface, [0.646, 0.409, 0.277], allowed_yaws=[0.0],
            params=dict(PARAMS_FULL, hull_margin=0.02))
        # with a 2 cm inset the deepest band no longer fits; the solver must
        # still succeed (shallower) or, if nothing fits, say so honestly
        if widened["success"]:
            assert widened["selected"]["depth_m"] < exact["selected"]["depth_m"]
        assert widened["candidates"][0]["detail"]["margins"][
            "hull_static_wall_and_chamfer_m"] == 0.02
