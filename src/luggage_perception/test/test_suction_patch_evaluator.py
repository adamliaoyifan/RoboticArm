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


if __name__ == "__main__":
    unittest.main()
