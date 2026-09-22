#!/usr/bin/env python3
"""ST-2 suction-patch tests (ROS-free, plain pytest; D1 entry point).

Covers the DYNAMIC-SUCTION plan's B gates at unit level: contact-model
schema behaviour (B0), planar acceptance and metric fidelity (B1),
cross-plane/bimodal rejection with exact-threshold semantics (B2), island
selection and edge fail-closed (B3), no-seal surfaces (B4), identity
mismatch (B5), determinism and bounds (B6). The matrix-scale execution
and latency/RSS evidence run through
``luggage_perception.eval.dynamic_suction_acceptance``.
"""

from __future__ import division

import json
import math
import os
import tempfile
import time
import unittest
from types import SimpleNamespace

import numpy as np

from luggage_description.suction_contact_model import (
    CONTACT_MODEL_BAD_TYPE,
    CONTACT_MODEL_DUPLICATE_KEY,
    CONTACT_MODEL_MISSING_KEY,
    CONTACT_MODEL_NAN,
    CONTACT_MODEL_NON_POSITIVE,
    CONTACT_MODEL_OVERSIZED,
    SuctionContactModelError,
    load_suction_contact_model,
    validate_contact_model_fields,
    ContactModel,
)
from luggage_perception.eval.dynamic_suction_renderer import (
    DEFAULT_CAMERA,
    NadirRenderer,
    _surface_poly,
    make_b1_case,
    make_b2_case,
    make_b3_case,
    make_b4_case,
)
from luggage_perception.instance_depth_component import (
    isolate_depth_component,
)
from luggage_perception.dynamic_top_surface import (
    estimate_dynamic_top_surface,
)
from luggage_perception.suction_patch_evaluator import (
    SUCTION_CANDIDATE_IDENTITY_MISMATCH,
    SUCTION_REJECT_ADJACENT_STEP,
    SUCTION_REJECT_BIMODAL,
    SUCTION_REJECT_PLANE_COVERAGE,
    SuctionPatchEvaluator,
    center_suction_evaluation,
    suction_identity_mismatch,
)

INTR = SimpleNamespace(fx=DEFAULT_CAMERA["fx"], fy=DEFAULT_CAMERA["fy"],
                       cx=DEFAULT_CAMERA["cx"], cy=DEFAULT_CAMERA["cy"])
CONFIG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          os.pardir, "config")
MODULE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          os.pardir, "luggage_perception")
DESCRIPTION_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               os.pardir, os.pardir, os.pardir,
                               "luggage_description", "luggage_description")

_GOOD_FIELDS = {
    "model_version": 1, "contact_frame": "suction_contact_frame",
    "footprint_type": "rectangle", "footprint_size_xy_m": [0.12, 0.12],
    "boundary_margin_m": 0.015, "cell_size_m": 0.005,
    "candidate_grid_m": 0.010, "min_valid_cell_fraction": 0.90,
    "min_mask_coverage": 0.95, "min_connected_plane_fraction": 0.90,
    "max_rms_residual_m": 0.0025, "max_p95_residual_m": 0.0040,
    "max_peak_to_valley_m": 0.0060, "max_normal_deviation_p95_deg": 5.0,
    "max_adjacent_step_m": 0.0040, "adjacent_step_distance_m": 0.010,
    "bimodal_min_separation_m": 0.0050, "bimodal_min_fraction": 0.15,
    "max_normal_tilt_deg": 8.0, "max_candidates": 5,
    "min_candidate_separation_m": 0.050, "max_candidate_iou": 0.25,
    "max_rejected_diagnostics": 64,
}


def make_model(footprint=0.12, **overrides):
    fields = dict(_GOOD_FIELDS)
    fields["footprint_size_xy_m"] = [footprint, footprint]
    fields.update(overrides)
    return ContactModel(**validate_contact_model_fields(fields))


def run_evaluator(case, model, instance_id="box-1", generation=7):
    """component -> dynamic top -> height map -> candidates."""
    comp = isolate_depth_component(case.depth_mm, bbox=case.bbox)
    if not comp.ok:
        return comp, None, None
    x0, y0, x1, y1 = case.bbox
    region = np.zeros(case.depth_mm.shape, dtype=bool)
    region[max(0, y0):max(0, y1), max(0, x0):max(0, x1)] = True
    top = estimate_dynamic_top_surface(
        case.depth_mm, comp.mask, INTR, case.mat4, stamp=case.stamp,
        frame=case.frame, instance_region=region)
    if top.reason != "ok":
        return comp, top, None
    evaluator = SuctionPatchEvaluator(model)
    evaluator.update(case.depth_mm, INTR, case.mat4, top,
                     instance_region=region, stamp=case.stamp,
                     frame_id=case.frame, instance_id=instance_id,
                     generation=generation)
    return comp, top, evaluator.copy_output()


class TestContactModelSchema(unittest.TestCase):
    """B0: schema rejections carry machine-readable reasons."""

    def _reject(self, reason, mutate):
        fields = dict(_GOOD_FIELDS)
        mutate(fields)
        with self.assertRaises(SuctionContactModelError) as ctx:
            validate_contact_model_fields(fields)
        self.assertEqual(ctx.exception.reason, reason)

    def test_missing_key(self):
        self._reject(CONTACT_MODEL_MISSING_KEY,
                     lambda f: f.pop("model_version"))

    def test_nan_footprint(self):
        self._reject(CONTACT_MODEL_NAN,
                     lambda f: f.__setitem__(
                         "footprint_size_xy_m", [float("nan"), 0.12]))

    def test_negative_dimension(self):
        self._reject(CONTACT_MODEL_NON_POSITIVE,
                     lambda f: f.__setitem__(
                         "footprint_size_xy_m", [0.12, -0.05]))

    def test_oversized_footprint(self):
        self._reject(CONTACT_MODEL_OVERSIZED,
                     lambda f: f.__setitem__(
                         "footprint_size_xy_m", [0.31, 0.31]))

    def test_bad_type(self):
        self._reject(CONTACT_MODEL_BAD_TYPE,
                     lambda f: f.__setitem__("model_version", "one"))

    def test_shipped_config_loads_with_hash(self):
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            os.pardir, os.pardir, "luggage_description",
                            "config", "suction_contact_model.yaml")
        model = load_suction_contact_model(path)
        self.assertEqual(model.model_version, 1)
        self.assertEqual(model.footprint_size_xy_m, (0.18, 0.18))
        self.assertEqual(len(model.identity_hash), 64)

    def test_duplicate_yaml_key_rejected(self):
        text = ("model_version: 1\ncontact_frame: f\n"
                "footprint_type: rectangle\nfootprint_type: circle\n")
        with tempfile.NamedTemporaryFile("w", suffix=".yaml",
                                         delete=False) as fh:
            fh.write(text)
            path = fh.name
        try:
            with self.assertRaises(SuctionContactModelError) as ctx:
                load_suction_contact_model(path)
            self.assertEqual(ctx.exception.reason,
                             CONTACT_MODEL_DUPLICATE_KEY)
        finally:
            os.unlink(path)


class TestPlanarSurfaces(unittest.TestCase):
    """B1 essentials: presence, on-surface, normal fidelity."""

    def setUp(self):
        self.renderer = NadirRenderer()

    def test_flat_top_yields_candidates(self):
        case, normal = make_b1_case(self.renderer, 0.0, 30.0, 1.0, 11, 1.0)
        comp, top, out = run_evaluator(case, make_model(0.12))
        self.assertEqual(top.reason, "ok", getattr(top, "reason", None))
        self.assertTrue(out.ok)
        self.assertGreaterEqual(len(out.accepted), 1)
        record = out.accepted[0]
        # Selected position keeps the margin-shrunk footprint inside the
        # true surface: |centre| <= surface_half - (footprint_half -
        # boundary_margin).
        self.assertLess(abs(record.center_world[0] + 1.0),
                        0.55 / 2 - (0.12 / 2 - 0.015) + 0.005)
        self.assertLess(abs(record.center_world[1]),
                        0.40 / 2 - (0.12 / 2 - 0.015) + 0.005)
        err = math.degrees(math.acos(min(
            1.0, abs(float(np.asarray(record.normal_world) @ normal)))))
        self.assertLessEqual(err, 4.0)

    def test_tilted_top_normal_error_bounded(self):
        case, normal = make_b1_case(self.renderer, 6.0, 0.0, 2.0, 29, 1.0)
        comp, top, out = run_evaluator(case, make_model(0.08))
        self.assertTrue(out.ok)
        self.assertGreaterEqual(len(out.accepted), 1)
        record = out.accepted[0]
        err = math.degrees(math.acos(min(
            1.0, abs(float(np.asarray(record.normal_world) @ normal)))))
        self.assertLessEqual(err, 4.0)


class TestCrossPlaneRejection(unittest.TestCase):
    """B2: steps crossing the footprint are never accepted."""

    def setUp(self):
        self.renderer = NadirRenderer()

    def _reasons(self, out):
        return {r[1] for r in out.rejected}

    def test_step_6mm_crossing_rejected(self):
        case, step = make_b2_case(self.renderer, 6.0, 0.0, 0.0, 11, 1.0)
        comp, top, out = run_evaluator(case, make_model(0.18))
        self.assertTrue(out.ok)
        # Candidates fully on one side stay sealable; a footprint whose
        # centre is within half a footprint of the boundary crosses the
        # step and must never be accepted (unsafe-accept count 0).
        for record in out.accepted:
            dist = abs(record.center_world[1])     # boundary: y = 0
            self.assertGreaterEqual(dist, 0.18 / 2 - 1e-6)
        # And the crossing region did reject on discontinuity gates.
        self.assertTrue(self._reasons(out) & {
            SUCTION_REJECT_ADJACENT_STEP, SUCTION_REJECT_BIMODAL,
            SUCTION_REJECT_PLANE_COVERAGE,
            "SUCTION_REJECT_PEAK_TO_VALLEY",
            "SUCTION_REJECT_NORMAL_DEVIATION"})

    def test_step_20mm_crossing_rejected(self):
        case, step = make_b2_case(self.renderer, 20.0, 45.0, 40.0, 29, 1.0)
        comp, top, out = run_evaluator(case, make_model(0.12))
        # Boundary at 45 deg through (+40 mm along its normal): accepted
        # candidates must sit wholly on one side of the boundary line.
        ang = math.radians(45.0)
        line_nrm = np.array([math.sin(ang), -math.cos(ang)])
        offset = 0.040
        for record in out.accepted:
            dist = abs(record.center_world[0] * line_nrm[0]
                       + record.center_world[1] * line_nrm[1] - offset)
            self.assertGreaterEqual(dist, 0.12 / 2 - 1e-6)

    def test_step_2mm_below_threshold_reported_not_rejected_as_unsafe(
            self):
        # Below-rejection results are diagnostics, not sealability proof:
        # candidates may exist but the case is only counted as evidence
        # that the gates do not misfire on small texture.
        case, step = make_b2_case(self.renderer, 2.0, 0.0, 0.0, 11, 1.0)
        comp, top, out = run_evaluator(case, make_model(0.08))
        self.assertTrue(out.ok)

    def test_exact_thresholds_deterministic(self):
        # Plan B2: comparisons are `>` for the 4 mm adjacent-step gate and
        # `>=` for the 5 mm bimodal gate, deterministically. The
        # exact-threshold cases are rendered NOISE-FREE: with sigma=1 mm
        # noise a literal 4.0 mm measurement straddles the gate and no
        # implementation could be deterministic at the threshold.
        case4, _ = make_b2_case(self.renderer, 4.0, 0.0, 0.0, 11, 1.0,
                                noise_sigma_mm=0.0)
        comp, top, out4 = run_evaluator(case4, make_model(0.12))
        self.assertTrue(out4.ok)
        self.assertNotIn(SUCTION_REJECT_ADJACENT_STEP,
                         {r[1] for r in out4.rejected})
        case5, _ = make_b2_case(self.renderer, 5.0, 0.0, 0.0, 11, 1.0,
                                noise_sigma_mm=0.0)
        comp, top, out5 = run_evaluator(case5, make_model(0.18))
        self.assertTrue(out5.ok)
        # The boundary runs through the surface centre: a 5.0 mm
        # separation with both sides >= 15 % means every crossing
        # footprint must be rejected (bimodal `>=` semantics).
        for record in out5.accepted:
            self.assertGreaterEqual(abs(record.center_world[1]),
                                    0.18 / 2 - 1e-6)


class TestIslandSelection(unittest.TestCase):
    """B3: exactly one planar island that fits the footprint + margin."""

    def setUp(self):
        self.renderer = NadirRenderer()

    def test_interior_island_selected(self):
        case = make_b3_case(self.renderer, (-1.0, 0.0), 0.0, 11, 1.0,
                            island_size=0.32)
        comp, top, out = run_evaluator(case, make_model(0.18))
        self.assertTrue(out.ok)
        self.assertGreaterEqual(len(out.accepted), 1)
        for record in out.accepted:
            self.assertLess(abs(record.center_world[0] + 1.0),
                            0.32 / 2 - 0.18 / 2 - 0.015 + 0.01)
            self.assertLess(abs(record.center_world[1]),
                            0.32 / 2 - 0.18 / 2 - 0.015 + 0.01)

    def test_edge_island_fails_closed(self):
        # Island near the mask edge and too small to host the footprint
        # even with the coverage gates' slack: the physical margin is
        # unavailable. Fail-closed may happen at the plane stage (the
        # tiny island holds < 15% of the component, so no sealable plane
        # survives) or at the patch gates — both produce zero candidates.
        case = make_b3_case(self.renderer, (-1.20, 0.0), 0.0, 11, 1.0,
                            island_size=0.15)
        comp, top, out = run_evaluator(case, make_model(0.18))
        if out is None:
            self.assertNotEqual(getattr(top, "reason", "no_top"), "ok")
            return
        self.assertTrue(out.ok)
        self.assertEqual(len(out.accepted), 0)


class TestNoSealSurfaces(unittest.TestCase):
    """B4: ridges, folds, seams, sparse depth, checkerboards, undersized
    surfaces yield zero candidates."""

    def setUp(self):
        self.renderer = NadirRenderer()

    def test_all_kinds_zero_candidates(self):
        for kind in ("ridge", "fold", "seam", "sparse", "checkerboard",
                     "too_small"):
            case = make_b4_case(self.renderer, kind, 11, 1.0)
            comp, top, out = run_evaluator(case, make_model(0.18))
            if out is None:
                # Component/top-level fail-closed (e.g. the undersized
                # surface never isolates) also yields zero candidates.
                self.assertFalse(comp.ok or (
                    top is not None and top.reason == "ok"), kind)
                continue
            self.assertTrue(out.ok, kind)
            self.assertEqual(len(out.accepted), 0,
                             "%s produced a candidate" % kind)


class TestIdentityAndDeterminism(unittest.TestCase):
    """B5/B6 essentials."""

    def setUp(self):
        self.renderer = NadirRenderer()
        self.case, _ = make_b1_case(self.renderer, 3.0, 30.0, 1.0, 11, 1.0)
        self.model = make_model(0.12)

    def _record(self):
        comp, top, out = run_evaluator(self.case, self.model)
        return out.accepted[0]

    def test_identity_mismatch_rejections(self):
        record = self._record()
        base = dict(stamp=1.0, frame_id="world", instance_id="box-1",
                    generation=7)
        self.assertIsNone(suction_identity_mismatch(record, **base))
        checks = [
            ("stamp", dict(stamp=1.1)),
            ("frame", dict(frame_id="camera")),
            ("generation", dict(generation=8)),
            ("instance_id", dict(instance_id="box-2")),
        ]
        for name, override in checks:
            fields = dict(base)
            fields.update(override)
            mismatch = suction_identity_mismatch(record, **fields)
            self.assertIsNotNone(mismatch, name)
            self.assertEqual(mismatch[0],
                             SUCTION_CANDIDATE_IDENTITY_MISMATCH)
        # Float tolerance covers representation error only.
        fields = dict(base)
        fields["stamp"] = base["stamp"] + 1e-9
        fields["tolerance_sec"] = 1e-6
        self.assertIsNone(suction_identity_mismatch(record, **fields))

    def test_repeat_inputs_identical_outputs(self):
        results = []
        for _ in range(20):
            comp, top, out = run_evaluator(self.case, self.model)
            results.append((
                tuple((c.candidate_id, c.rank) for c in out.accepted),
                tuple((r[0], r[1]) for r in out.rejected),
                tuple(c.score for c in out.accepted)))
        for other in results[1:]:
            self.assertEqual(results[0][:2], other[:2])
            for a, b in zip(results[0][2], other[2]):
                self.assertLessEqual(abs(a - b), 1e-12)

    def test_bounds_and_copy_independence(self):
        comp, top, out = run_evaluator(self.case, self.model)
        self.assertLessEqual(len(out.accepted), 5)
        self.assertLessEqual(len(out.rejected), 64)
        out.accepted[0].rank = 99            # caller mutation...
        comp, top, out2 = run_evaluator(self.case, self.model)
        self.assertEqual(out2.accepted[0].rank, 1)   # ...cannot corrupt


MODULES = (
    "suction_patch_evaluator.py",
    "suction_height_map.py",
)


def _true_polygon_region(renderer, case):
    """Pixel mask of the case's true top polygon (the B1 harness input).

    Mirrors ``_polygon_region`` in the acceptance runner: same
    half-plane rasterization as the renderer itself.
    """
    poly = _surface_poly(tuple(case.gt_center_xy), case.gt_size_wh,
                         case.gt_yaw_deg)
    wx, wy = renderer.world_xy_at(case.gt_top_z)
    inside_pos = np.ones(wx.shape, dtype=bool)
    inside_neg = np.ones(wx.shape, dtype=bool)
    for i in range(len(poly)):
        x0_, y0_ = poly[i]
        x1_, y1_ = poly[(i + 1) % len(poly)]
        cross = ((x1_ - x0_) * (wy - y0_) - (y1_ - y0_) * (wx - x0_))
        inside_pos &= cross >= 0.0
        inside_neg &= cross <= 0.0
    return inside_pos | inside_neg


class TestStepGateResponse(unittest.TestCase):
    """B1/B2: the sustained-discontinuity gate on the smoothed field.

    Response table (measured 2026-09-17): a noiseless hard step of ``s``
    measures >= 0.84*s for any boundary angle, so the frozen 4 mm gate
    keeps its exact-``>`` semantics (4.0 mm stays accepted, >= 5 mm
    rejected) while the noise floor sits ~42% below the gate.
    """

    GATE = 0.0040
    CELL = 0.005
    MAXD = 0.010

    def _step_field(self, step_m, angle_deg):
        n = 48
        a = math.radians(angle_deg)
        i = np.arange(n)[:, None] - n // 2
        j = np.arange(n)[None, :] - n // 2
        field = np.where(i * math.sin(a) + j * math.cos(a) >= 0.0,
                         step_m, 0.0)
        return field, np.ones((n, n), dtype=bool)

    def _gate(self, field, valid):
        return SuctionPatchEvaluator._max_adjacent_step(
            field, valid, self.CELL, self.MAXD, self.GATE)

    def test_exact_threshold_semantics(self):
        for angle in (0.0, 45.0):
            for step_mm in (4.0, 5.0, 6.0, 10.0, 20.0):
                field, valid = self._step_field(step_mm / 1000.0, angle)
                _, count = self._gate(field, valid)
                if step_mm > 4.0:
                    self.assertGreater(
                        count, 0,
                        "step %.1f mm at %.0f deg must reject"
                        % (step_mm, angle))
                else:
                    self.assertEqual(
                        count, 0,
                        "step %.1f mm at %.0f deg must stay accepted "
                        "(strict > 4.0 mm)" % (step_mm, angle))

    def test_reported_max_tracks_step_height(self):
        field, valid = self._step_field(0.006, 0.0)
        max_step, count = self._gate(field, valid)
        self.assertGreater(count, 0)
        self.assertGreater(max_step, 0.004)
        self.assertLessEqual(max_step, 0.006 + 1e-9)


class TestStepGateNoiseFloor(unittest.TestCase):
    """B1: flat planes at real camera noise density must not trip.

    Cell medians at ~2 points per 5 mm cell carry sigma up to ~2 mm;
    the smoothed-field block gate keeps the confirmed count at zero and
    the largest single difference well under the 4 mm gate.
    """

    def test_flat_noise_confirmed_count_zero(self):
        gate = 0.0040
        for sigma_mm in (1.35, 2.0):
            rng = np.random.default_rng(20260917)
            worst_single = 0.0
            for _ in range(200):
                raw = rng.normal(0.0, sigma_mm / 1000.0, (37, 37))
                # The gate consumes the robust surface: median of the
                # 5x5 neighbourhood, exactly the height map's smooth_h
                # contract (>= 20/25 valid; fully valid here).
                pad = np.pad(raw, 2, mode="reflect")
                view = np.lib.stride_tricks.sliding_window_view(
                    pad, (5, 5))
                smooth = np.median(view, axis=(-2, -1))
                valid = np.ones((37, 37), dtype=bool)
                max_step, count = SuctionPatchEvaluator._max_adjacent_step(
                    smooth, valid, 0.005, 0.010, gate)
                self.assertEqual(count, 0,
                                 "sigma %.2f mm tripped the gate" % sigma_mm)
                worst_single = max(worst_single, max_step)
            # 42% margin to the gate at sigma = 2 mm.
            self.assertLess(worst_single, gate)


class TestBoundaryMarginRotatedCorners(unittest.TestCase):
    """Regressions for the 2026-09-17 rotated-corner off-surface rows.

    At yaw 30/60 the true eroded surface is the ROTATED rectangle; the
    margin-square gate keeps rank-1 inside it for every footprint (the
    B1 harness's historical axis-aligned reference falsely rejected
    these).
    """

    CASES = ((0.0, 60.0, 29), (6.0, 30.0, 29), (6.0, 30.0, 47))

    def test_rank1_inside_rotated_eroded_surface(self):
        renderer = NadirRenderer()
        for tilt, yaw, seed in self.CASES:
            case, _ = make_b1_case(renderer, tilt, yaw, 0.0, seed, 1.0)
            region = _true_polygon_region(renderer, case)
            comp = isolate_depth_component(case.depth_mm, bbox=case.bbox)
            top = estimate_dynamic_top_surface(
                case.depth_mm, comp.mask, INTR, case.mat4,
                stamp=case.stamp, frame=case.frame,
                instance_region=region)
            evaluator = SuctionPatchEvaluator(make_model(0.08))
            evaluator.update(case.depth_mm, INTR, case.mat4, top,
                             instance_region=region, stamp=case.stamp,
                             frame_id=case.frame, instance_id="box-1",
                             generation=7)
            out = evaluator.copy_output()
            self.assertGreaterEqual(len(out.accepted), 1,
                                    case.case_id)
            rec = out.accepted[0]
            dx, dy = rec.center_world[0] + 1.0, rec.center_world[1]
            a = math.radians(yaw)
            lx, ly = dx * math.cos(a) + dy * math.sin(a), \
                -dx * math.sin(a) + dy * math.cos(a)
            self.assertLessEqual(abs(lx), 0.55 / 2 - 0.015, case.case_id)
            self.assertLessEqual(abs(ly), 0.40 / 2 - 0.015, case.case_id)


class TestB1NoiseTwoMmFootprint18(unittest.TestCase):
    """Regressions for the three 2026-09-16 B1 failures.

    fp=0.18 + noise 2 mm produced zero candidates through the raw-block
    step gate (5.1-5.3 mm noise spikes); these must now accept with the
    rank-1 inside the true 15 mm-eroded surface.
    """

    CASES = ((0.0, 30.0, 47), (6.0, 0.0, 29), (6.0, 60.0, 47))

    def test_all_three_accept_on_surface(self):
        renderer = NadirRenderer()
        model = make_model(0.18)
        for tilt, yaw, seed in self.CASES:
            case, _ = make_b1_case(renderer, tilt, yaw, 2.0, seed, 1.0)
            region = _true_polygon_region(renderer, case)
            comp = isolate_depth_component(case.depth_mm, bbox=case.bbox)
            self.assertTrue(comp.ok, case.case_id)
            top = estimate_dynamic_top_surface(
                case.depth_mm, comp.mask, INTR, case.mat4,
                stamp=case.stamp, frame=case.frame,
                instance_region=region)
            self.assertEqual(top.reason, "ok", case.case_id)
            evaluator = SuctionPatchEvaluator(model)
            evaluator.update(case.depth_mm, INTR, case.mat4, top,
                             instance_region=region, stamp=case.stamp,
                             frame_id=case.frame, instance_id="box-1",
                             generation=7)
            out = evaluator.copy_output()
            self.assertGreaterEqual(
                len(out.accepted), 1, case.case_id)
            rec = out.accepted[0]
            self.assertLessEqual(
                abs(rec.center_world[0] + 1.0), 0.55 / 2 - 0.015,
                case.case_id)
            self.assertLessEqual(
                abs(rec.center_world[1]), 0.40 / 2 - 0.015, case.case_id)


class TestEvaluatorLatency(unittest.TestCase):
    """B6 micro-benchmark; runs only under SUCTION_LATENCY_TEST=1."""

    def test_no_seal_p95_under_half_budget(self):
        if os.environ.get("SUCTION_LATENCY_TEST") != "1":
            self.skipTest("set SUCTION_LATENCY_TEST=1 to run")
        renderer = NadirRenderer()
        case = make_b4_case(renderer, "ridge", 11, 1.0)
        model = make_model(0.18)
        comp = isolate_depth_component(case.depth_mm, bbox=case.bbox)
        x0, y0, x1, y1 = case.bbox
        region = np.zeros(case.depth_mm.shape, dtype=bool)
        region[y0:y1, x0:x1] = True
        top = estimate_dynamic_top_surface(
            case.depth_mm, comp.mask, INTR, case.mat4, stamp=case.stamp,
            frame=case.frame, instance_region=region)
        evaluator = SuctionPatchEvaluator(model)
        evaluator.update(case.depth_mm, INTR, case.mat4, top,
                         instance_region=region, stamp=case.stamp,
                         frame_id=case.frame, instance_id="box-1",
                         generation=7)
        self.assertEqual(len(evaluator.copy_output().accepted), 0)
        latencies = []
        for k in range(20):
            t0 = time.monotonic()
            evaluator.update(case.depth_mm, INTR, case.mat4, top,
                             instance_region=region,
                             stamp=case.stamp + k, frame_id=case.frame,
                             instance_id="box-1", generation=7)
            latencies.append((time.monotonic() - t0) * 1000.0)
        p95 = float(np.percentile(latencies, 95))
        self.assertLess(p95, 25.0, "p95 %.1f ms exceeds half budget"
                        % p95)



class TestArchitectureIsolation(unittest.TestCase):
    """A0-style: no ROS imports and no I/O in the new algorithm modules."""

    FORBIDDEN = ("rclpy", "rospy", "tf2_ros", "sensor_msgs", "luggage_msgs",
                 "builtin_interfaces", "std_msgs", "geometry_msgs")

    def test_no_ros_imports_or_io(self):
        for name in MODULES:
            with open(os.path.join(MODULE_DIR, name)) as fh:
                source = fh.read()
            for token in self.FORBIDDEN:
                self.assertNotIn("import %s" % token, source,
                                 "%s must not import %s" % (name, token))
            for token in ("open(", "np.load", "np.save", "yaml.",
                          "os.remove"):
                self.assertNotIn(token, source,
                                 "%s must not perform file I/O" % name)


class TestCenterSuctionCandidate(unittest.TestCase):

    def test_one_candidate_at_top_center(self):
        top = SimpleNamespace(
            center_xy=np.array([1.25, -0.4]),
            top_z=0.62,
            plane_basis=(
                np.array([1.0, 0.0, 0.0]),
                np.array([0.0, 1.0, 0.0]),
                np.array([0.0, 0.0, 1.0]),
            ),
        )
        evaluation = center_suction_evaluation(
            top, stamp=12.5, frame_id="world", instance_id="box-a",
            generation=4, model_version=1, model_hash="abc")
        self.assertTrue(evaluation.ok)
        self.assertEqual(evaluation.reason, "center")
        self.assertEqual(evaluation.rejected, ())
        self.assertEqual(len(evaluation.accepted), 1)
        record = evaluation.accepted[0]
        self.assertEqual(record.candidate_id, "center")
        self.assertEqual(record.rank, 0)
        self.assertEqual(record.center_world, (1.25, -0.4, 0.62))
        self.assertEqual(record.quaternion_xyzw, (0.0, 0.0, 0.0, 1.0))
        self.assertEqual(record.instance_id, "box-a")
        self.assertEqual(record.generation, 4)
        self.assertEqual(record.model_version, 1)
        self.assertEqual(record.model_hash, "abc")
        self.assertIsNone(suction_identity_mismatch(
            record, 12.5, "world", "box-a", 4))


if __name__ == "__main__":
    unittest.main()
