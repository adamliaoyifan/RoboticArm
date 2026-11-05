#!/usr/bin/env python3
"""HULL-GATES G3 offline evidence matrix (plan 33ba6745 §6, dispatch e90bddb7).

Five gate groups against the frozen deployment ULD descriptor
(luggage_description/config/scene_tf.yaml: 1.49 x 1.97 x 1.48 m, floor_z
0.53, ceiling_z 2.01, +Y chamfer floor_y 0.55 / wall_z 0.90, -X aperture):

  H1  descriptor/hash admission (kernel strictness, silent-degrade policy);
  H2  height-dependent feasible center domain vs an independent
      half-space oracle (normal-distance margin on the chamfer slope);
  H3  final-placement containment through the REAL selection chain
      (generate_candidates -> gates -> sort -> solve_placement selection),
      plus the node adapter (real ``_constraint_verdict``) when rclpy is
      available;
  H4  horizontal support and chamfer containment as separate gates
      (the slope is a boundary, never a platform; unknown is not filled);
  H5  independent OBB interval oracle vs the frozen SAT predicate
      ``_polygon_overlaps(_rect_corners(...))`` -- evidence first, no
      "old oracle agrees therefore correct" argument anywhere.

Oracle independence: no oracle below calls the kernel/solver helpers it
judges (``oriented_box_corners``, ``_rect_corners``, ``y_max_at_z``); the
hull oracle is rebuilt from the raw yaml numbers via point-to-line
distance. Frozen EPS = 1e-9 everywhere; no extra collision tolerance.
"""

import copy
import math
import os
import sys
import unittest

import yaml

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

from luggage_description.container_geometry import (  # noqa: E402
    cuboid_from_inner_size,
    contains_oriented_box,
    contains_oriented_box_through_aperture,
    descriptor_from_scene_config,
    normalize_descriptor,
    payload_center_y_interval,
    y_bounds_for_z_interval,
)
from luggage_packing.placement_solver import (  # noqa: E402
    REASON_INSUFFICIENT_SUPPORT,
    REASON_OUTSIDE_HULL,
    REASON_UNKNOWN_ABOVE_FLOOR,
    generate_candidates,
    solve_placement,
)

EPS = 1e-9
SCENE_PATH = os.path.join(
    PKG_ROOT, "..", "luggage_description", "config", "scene_tf.yaml")
with open(SCENE_PATH, "r", encoding="utf-8") as _handle:
    SCENE = yaml.safe_load(_handle)
_INNER = SCENE["container"]["inner"]
_CHAMFER = _INNER.get("chamfer") or {}

# Raw deployment numbers, parsed straight from the yaml (independent of the
# kernel code path under test).
RAW_HALF_X = 0.5 * float(_INNER["length"])
RAW_HALF_Y = 0.5 * float(_INNER["width"])
RAW_FLOOR_Z = float(_INNER["floor_z"])
RAW_CEILING_Z = float(_INNER["ceiling_z"])
RAW_FLOOR_Y = float(_CHAMFER["floor_y"])
RAW_WALL_Z = float(_CHAMFER["wall_z"])
RAW_WALL_Y = float(_CHAMFER.get("wall_y", 0.5 * float(_INNER["width"])))
INNER_SIZE = (float(_INNER["length"]), float(_INNER["width"]),
              float(_INNER["ceiling_z"]) - float(_INNER["floor_z"]))

KERNEL = descriptor_from_scene_config(SCENE)
DEPLOY_DESCRIPTOR = KERNEL.descriptor()
CUBOID_DESCRIPTOR = cuboid_from_inner_size(
    (1.49, 1.97, 1.48), floor_z=0.53).descriptor()

BOX = (0.44, 0.33, 0.60)  # conservative flat-pack footprint used throughout


# ----------------------------------------------------------------------
# Independent oracles (no kernel/solver helper reuse)


def oracle_y_limit(z, margin=0.0):
    """Usable +Y at ``z`` from the raw chamfer points.

    Below ``wall_z`` the limit is the chamfer line through
    (floor_y, floor_z) -> (wall_y, wall_z); a ``margin`` is enforced as a
    point-to-line distance, which is the plane-normal semantics the plan
    freezes (never "subtract the same number in Y").
    """
    if z >= RAW_WALL_Z - EPS:
        return RAW_HALF_Y - margin
    slope = (RAW_WALL_Y - RAW_FLOOR_Y) / (RAW_WALL_Z - RAW_FLOOR_Z)
    line_y = RAW_FLOOR_Y + slope * (z - RAW_FLOOR_Z)
    return min(RAW_HALF_Y - margin, line_y - margin * math.sqrt(1 + slope * slope))


def oracle_point_inside(x, y, z, margin=0.0, aperture=False):
    """Mirror of the kernel's closed-face point semantics, independently
    derived. ``aperture`` skips the -X door face (transit semantics)."""
    if x > RAW_HALF_X - margin + EPS:
        return False
    if not aperture and x < -RAW_HALF_X + margin - EPS:
        return False
    if y < -RAW_HALF_Y + margin - EPS or y > RAW_HALF_Y - margin + EPS:
        return False
    if z < RAW_FLOOR_Z + margin - EPS or z > RAW_CEILING_Z - margin + EPS:
        return False
    return y <= oracle_y_limit(z, margin) + EPS


def oracle_box_contained(center, size, yaw=0.0, margin=0.0,
                         aperture=False, floor_margin=None):
    """Full-containment verdict for one oriented box.

    ``floor_margin`` (default ``margin``) exempts a floor-resting box from
    the floor margin while every other face keeps ``margin`` -- the
    planner validator's documented "lateral clearance only, never the
    floor" rule.
    """
    cx, cy, cz = (float(v) for v in center)
    sx, sy, sz = (0.5 * float(v) for v in size)
    floor_tol = margin if floor_margin is None else float(floor_margin)
    cos_y, sin_y = math.cos(float(yaw)), math.sin(float(yaw))
    for dx in (-sx, sx):
        for dy in (-sy, sy):
            for dz in (-sz, sz):
                x = cx + cos_y * dx - sin_y * dy
                y = cy + sin_y * dx + cos_y * dy
                z = cz + dz
                if x > RAW_HALF_X - margin + EPS:
                    return False
                if (not aperture
                        and x < -RAW_HALF_X + margin - EPS):
                    return False
                if (y < -RAW_HALF_Y + margin - EPS
                        or y > RAW_HALF_Y - margin + EPS):
                    return False
                if z < RAW_FLOOR_Z + floor_tol - EPS:
                    return False
                if z > RAW_CEILING_Z - margin + EPS:
                    return False
                if y > oracle_y_limit(z, margin) + EPS:
                    return False
    return True


def oracle_rot_half_extents(size_xy, yaw):
    sx, sy = (0.5 * float(v) for v in size_xy)
    cos_y, sin_y = abs(math.cos(float(yaw))), abs(math.sin(float(yaw)))
    return (cos_y * sx + sin_y * sy, sin_y * sx + cos_y * sy)


def obb_interval_overlaps(a, b, tolerance=1e-9):
    """H5 oracle: rectangle overlap via center/interval projection.

    ``a``/``b`` are ``(cx, cy, yaw, half_len, half_wid)``. The interval of
    each rectangle on an axis is center-projection +/- the extent built
    from the rectangle's own axes -- no corners, no polygon walk, nothing
    shared with the predicate under test.
    """

    def axes(yaw):
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        return (cos_y, sin_y), (-sin_y, cos_y)

    (au, av), (bu, bv) = axes(a[2]), axes(b[2])
    for axis in (au, av, bu, bv):
        ax, ay = axis
        a_center = a[0] * ax + a[1] * ay
        a_radius = (abs(au[0] * ax + au[1] * ay) * a[3]
                    + abs(av[0] * ax + av[1] * ay) * a[4])
        b_center = b[0] * ax + b[1] * ay
        b_radius = (abs(bu[0] * ax + bu[1] * ay) * b[3]
                    + abs(bv[0] * ax + bv[1] * ay) * b[4])
        gap_lo = (b_center - b_radius) - (a_center + a_radius)
        gap_hi = (a_center - a_radius) - (b_center + b_radius)
        if max(gap_lo, gap_hi) > -tolerance:
            return False
    return True


def _axis_gaps(a, b):
    """Signed separation on the four rectangle axes; the max is the metric
    the grown-rect same-layer clearance test actually implements."""
    def axes(yaw):
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        return (cos_y, sin_y), (-sin_y, cos_y)

    (au, av), (bu, bv) = axes(a[2]), axes(b[2])
    gaps = []
    for axis in (au, av, bu, bv):
        ax, ay = axis
        a_center = a[0] * ax + a[1] * ay
        a_radius = (abs(au[0] * ax + au[1] * ay) * a[3]
                    + abs(av[0] * ax + av[1] * ay) * a[4])
        b_center = b[0] * ax + b[1] * ay
        b_radius = (abs(bu[0] * ax + bu[1] * ay) * b[3]
                    + abs(bv[0] * ax + bv[1] * ay) * b[4])
        gaps.append(max((b_center - b_radius) - (a_center + a_radius),
                        (a_center - a_radius) - (b_center + b_radius)))
    return gaps


def uld_map(res=0.05, state="unknown", height=0.0, descriptor=None,
            center_base=None, yaw=0.0):
    """A surface_map_2d matching the planner floor-prior contract."""
    inner_l, inner_w, inner_h = INNER_SIZE
    nx = max(1, int(round(inner_l / res)))
    ny = max(1, int(round(inner_w / res)))
    if center_base is None:
        center_base = [0.0, 0.0, RAW_FLOOR_Z + 0.5 * inner_h]
    surface = {
        "resolution": res,
        "nx": nx,
        "ny": ny,
        "inner_size": [inner_l, inner_w, inner_h],
        "floor_z": 0.0,
        "center_base": list(center_base),
        "yaw": float(yaw),
        "height": [[height] * ny for _ in range(nx)],
        "state": [[state] * ny for _ in range(nx)],
        "confidence": [["none"] * ny for _ in range(nx)],
    }
    if descriptor is not None:
        surface["geometry_descriptor"] = copy.deepcopy(descriptor)
        surface["geometry_hash"] = str(descriptor.get("geometry_hash") or "")
    return surface


def enumerate_all(surface, box=BOX, yaws=(0.0, math.pi / 2), params=None):
    cfg = {"top_n": 10 ** 9, "keep_rejected": 10 ** 9}
    if params:
        cfg.update(params)
    totals = {}
    out = generate_candidates(surface, list(box), allowed_yaws=list(yaws),
                              params=cfg, totals=totals)
    return out, totals


def feasible(candidates):
    return [c for c in candidates if c.get("feasible")]


# ----------------------------------------------------------------------
# H5: independent OBB interval oracle vs the frozen SAT predicate


class TestH5IndependentObbOracle(unittest.TestCase):
    """Evidence-first SAT validation. A disagreement outside the frozen
    1e-9 tolerance band is a counterexample and fails the suite; the
    grown-rect clearance metric is measured and labeled POLICY (a mixed
    norm, conservative direction), not a defect."""

    YAWS = [math.radians(a) for a in (0.0, 5.0, 45.0, 90.0)]

    @staticmethod
    def _solver_overlap(a, b, tolerance=1e-9):
        from luggage_packing.placement_solver import (
            _polygon_overlaps, _rect_corners)
        left = _rect_corners(a[0], a[1], a[2], 2 * a[3], 2 * a[4])
        right = _rect_corners(b[0], b[1], b[2], 2 * b[3], 2 * b[4])
        return _polygon_overlaps(left, right, tolerance)

    def test_matrix_no_counterexample(self):
        """Sweep yaw pairs x offsets; disagreements must sit inside the
        tolerance band (solver@1e-9 == oracle@0), never beyond it."""
        counterexamples = []
        tolerance_band = 0
        samples = 0
        b_shapes = [(0.22, 0.165), (0.30, 0.20)]
        for yaw_a in self.YAWS:
            for yaw_b in self.YAWS:
                for step in range(-22, 23):
                    dx = step * 0.025
                    for step_y in range(-22, 23):
                        dy = step_y * 0.025
                        for bh, bw in b_shapes:
                            a = (0.0, 0.0, yaw_a, 0.22, 0.165)
                            b = (dx, dy, yaw_b, bh, bw)
                            samples += 1
                            solver = self._solver_overlap(a, b)
                            oracle = obb_interval_overlaps(a, b, 1e-9)
                            if solver == oracle:
                                continue
                            # Disagreement: legal only if it is the frozen
                            # tolerance semantics (solver@tol == oracle@0).
                            if (solver == obb_interval_overlaps(a, b, 0.0)
                                    and solver == self._solver_overlap(a, b, 0.0)):
                                tolerance_band += 1
                                continue
                            counterexamples.append((a, b, solver, oracle))
        self.assertEqual(
            [], counterexamples,
            "SAT counterexample(s) beyond the frozen tolerance: %s"
            % counterexamples[:3])
        self.assertGreater(samples, 60000)

    def test_symmetric_under_argument_swap(self):
        for yaw_a in self.YAWS:
            for yaw_b in self.YAWS:
                for step in range(-10, 11):
                    a = (0.0, 0.0, yaw_a, 0.22, 0.165)
                    b = (step * 0.05, 0.03, yaw_b, 0.30, 0.20)
                    self.assertEqual(
                        self._solver_overlap(a, b),
                        self._solver_overlap(b, a),
                        (a, b))

    def test_translation_invariance(self):
        for yaw_a in self.YAWS:
            for yaw_b in self.YAWS:
                base_a = (0.1, -0.05, yaw_a, 0.22, 0.165)
                base_b = (0.25, 0.02, yaw_b, 0.30, 0.20)
                shifted_a = (base_a[0] + 1.7, base_a[1] - 0.9, yaw_a,
                             0.22, 0.165)
                shifted_b = (base_b[0] + 1.7, base_b[1] - 0.9, yaw_b,
                             0.30, 0.20)
                self.assertEqual(
                    self._solver_overlap(base_a, base_b),
                    self._solver_overlap(shifted_a, shifted_b),
                    (base_a, base_b))

    def test_corotation_invariance(self):
        for yaw_a in self.YAWS:
            for yaw_b in self.YAWS:
                for extra in (0.3, 1.1):
                    a = (0.0, 0.0, yaw_a, 0.22, 0.165)
                    b = (0.30, 0.04, yaw_b, 0.30, 0.20)
                    rot = lambda r, t: (  # noqa: E731
                        r[0] * math.cos(t) - r[1] * math.sin(t),
                        r[0] * math.sin(t) + r[1] * math.cos(t),
                        r[2] + t, r[3], r[4])
                    self.assertEqual(
                        self._solver_overlap(a, b),
                        self._solver_overlap(rot(a, extra), rot(b, extra)),
                        (a, b, extra))

    def test_separation_tangency_microoverlap(self):
        # Touching (gap exactly 0) is NOT overlap: support faces may touch.
        a = (0.0, 0.0, 0.0, 0.22, 0.165)
        touch = (0.22 + 0.30, 0.0, 0.0, 0.30, 0.20)
        self.assertFalse(self._solver_overlap(a, touch))
        self.assertFalse(obb_interval_overlaps(a, touch, 1e-9))
        micro = (0.22 + 0.30 - 1e-6, 0.0, 0.0, 0.30, 0.20)
        self.assertTrue(self._solver_overlap(a, micro))
        self.assertTrue(obb_interval_overlaps(a, micro, 1e-9))
        gap = (0.22 + 0.30 + 1e-6, 0.0, 0.0, 0.30, 0.20)
        self.assertFalse(self._solver_overlap(a, gap))
        self.assertFalse(obb_interval_overlaps(a, gap, 1e-9))

    def test_policy_same_layer_growth_semantics(self):
        """POLICY (labeled, not a defect): the same-layer clearance test
        grows each footprint by the clearance IN ITS OWN FRAME and tests
        overlap. For a rotated rectangle that inflates a projection on an
        outside axis by c(|cos|+|sin|) in [c, c*sqrt(2)], so the enforced
        separation is direction-dependent between 2c and ~2.83c --
        conservative (over-rejecting) for diagonal neighbors. This test
        pins (a) the grown-overlap verdicts equal the independent oracle
        on the grown rectangles, and (b) that the mixed band is real."""
        clearance = 0.02
        checked = 0
        mixed_cases = 0
        for yaw_b in (math.radians(5.0), math.radians(45.0)):
            for step in range(-20, 21):
                for step_y in range(-20, 21):
                    a = (0.0, 0.0, 0.0, 0.22, 0.165)
                    b = (step * 0.03, step_y * 0.03, yaw_b, 0.22, 0.165)
                    grown_a = (a[0], a[1], a[2],
                               a[3] + clearance, a[4] + clearance)
                    grown_b = (b[0], b[1], b[2],
                               b[3] + clearance, b[4] + clearance)
                    grown_overlap = self._solver_overlap(grown_a, grown_b)
                    self.assertEqual(
                        grown_overlap,
                        obb_interval_overlaps(grown_a, grown_b, 1e-9),
                        (a, b))
                    checked += 1
                    if (grown_overlap
                            and max(_axis_gaps(a, b)) > 2 * clearance + 1e-12):
                        mixed_cases += 1
        self.assertGreater(checked, 3000)
        self.assertGreater(
            mixed_cases, 0,
            "expected rotated pairs where the four-axis gap of the "
            "originals exceeds 2c yet the grown test still overlaps (the "
            "direction-dependent mixed-norm band)")


# ----------------------------------------------------------------------
# H2: feasible center domain vs the independent half-space oracle


class TestH2CenterDomainVsOracle(unittest.TestCase):

    Z_BANDS = (
        (RAW_FLOOR_Z, RAW_FLOOR_Z + 0.30),
        (RAW_FLOOR_Z, RAW_WALL_Z),
        (RAW_WALL_Z - 0.001, RAW_WALL_Z + 0.001),
        (RAW_WALL_Z - 0.20, RAW_WALL_Z + 0.20),
        (RAW_WALL_Z + 0.05, RAW_CEILING_Z - 0.05),
    )
    SIZES = ((0.44, 0.33), (0.33, 0.44), (0.60, 0.50))
    MARGINS = (0.0, 0.01)
    YAWS = [math.radians(a) for a in (0.0, 5.0, 45.0, 90.0)]

    def test_payload_center_y_interval_matches_oracle(self):
        for size in self.SIZES:
            for yaw in self.YAWS:
                for margin in self.MARGINS:
                    _hx, hy = oracle_rot_half_extents(size, yaw)
                    for z0, z1 in self.Z_BANDS:
                        kernel_lo, kernel_hi = payload_center_y_interval(
                            KERNEL, (size[0], size[1], 0.6), z0, z1,
                            yaw=yaw, margin=margin)
                        oracle_hi = min(oracle_y_limit(z0, margin),
                                        oracle_y_limit(z1, margin)) - hy
                        oracle_lo = -RAW_HALF_Y + margin + hy
                        self.assertAlmostEqual(
                            kernel_hi, oracle_hi, delta=EPS,
                            msg=(size, math.degrees(yaw), margin, z0, z1))
                        self.assertAlmostEqual(
                            kernel_lo, oracle_lo, delta=EPS,
                            msg=(size, math.degrees(yaw), margin, z0, z1))

    def test_y_bounds_for_z_interval_matches_oracle(self):
        for margin in self.MARGINS:
            for z0, z1 in self.Z_BANDS:
                kernel_lo, kernel_hi = y_bounds_for_z_interval(
                    KERNEL, z0, z1, margin=margin)
                steps = 64
                oracle_hi = min(
                    oracle_y_limit(z0 + (z1 - z0) * i / steps, margin)
                    for i in range(steps + 1))
                self.assertAlmostEqual(kernel_hi, oracle_hi, delta=EPS,
                                       msg=(margin, z0, z1))
                self.assertAlmostEqual(
                    kernel_lo, -RAW_HALF_Y + margin, delta=EPS)

    def test_center_domain_is_corner_semantics(self):
        """Center inside the interval => the whole box is inside (every
        corner); center 1 mm beyond => a corner violates the chamfer
        plane at floor level (the tightest face for a floor band)."""
        size = (0.44, 0.33, 0.60)
        lo, hi = payload_center_y_interval(
            KERNEL, size, RAW_FLOOR_Z, RAW_FLOOR_Z + 0.60, margin=0.0)
        self.assertTrue(oracle_box_contained(
            (0.0, hi, RAW_FLOOR_Z + 0.30), size, margin=0.0))
        self.assertFalse(oracle_box_contained(
            (0.0, hi + 0.001, RAW_FLOOR_Z + 0.30), size, margin=0.0))
        self.assertTrue(oracle_box_contained(
            (0.0, lo, RAW_FLOOR_Z + 0.30), size, margin=0.0))
        self.assertFalse(oracle_box_contained(
            (0.0, lo - 0.001, RAW_FLOOR_Z + 0.30), size, margin=0.0))

    def test_margin_is_normal_distance_on_slope(self):
        """1 mm inside/outside the frozen margin on the chamfer slope,
        checked as a point-to-line distance by the oracle. The box rests
        on the floor, so the oracle exempts the floor margin (the y
        interval itself is floor-independent)."""
        size = (0.44, 0.33, 0.60)
        center = (0.0, 0.30, RAW_FLOOR_Z + 0.30)
        lo, hi = payload_center_y_interval(
            KERNEL, size, RAW_FLOOR_Z, RAW_FLOOR_Z + 0.60, margin=0.05)
        self.assertTrue(oracle_box_contained(
            (0.0, hi, RAW_FLOOR_Z + 0.30), size, margin=0.05,
            floor_margin=0.0))
        self.assertFalse(oracle_box_contained(
            (0.0, hi + 0.001, RAW_FLOOR_Z + 0.30), size, margin=0.05,
            floor_margin=0.0))


# ----------------------------------------------------------------------
# H1: descriptor admission


class TestH1DescriptorAdmission(unittest.TestCase):

    def test_deployment_descriptor_strict_and_hashed(self):
        self.assertIsNotNone(KERNEL.chamfer)
        self.assertEqual(KERNEL.chamfer.floor_y, RAW_FLOOR_Y)
        self.assertEqual(KERNEL.chamfer.wall_z, RAW_WALL_Z)
        rebuilt = normalize_descriptor(copy.deepcopy(DEPLOY_DESCRIPTOR))
        self.assertEqual(rebuilt.geometry_hash, KERNEL.geometry_hash)

    def test_wrong_hash_rejected(self):
        bad = copy.deepcopy(DEPLOY_DESCRIPTOR)
        bad["geometry_hash"] = "0" * 64
        with self.assertRaises(ValueError):
            normalize_descriptor(bad)

    def test_malformed_chamfer_rejected(self):
        bad = copy.deepcopy(DEPLOY_DESCRIPTOR)
        bad["chamfer"]["wall_z"] = RAW_CEILING_Z + 0.1
        with self.assertRaises(ValueError):
            normalize_descriptor(bad)
        bad = copy.deepcopy(DEPLOY_DESCRIPTOR)
        bad["chamfer"]["floor_y"] = RAW_HALF_Y + 0.5
        with self.assertRaises(ValueError):
            normalize_descriptor(bad)
        bad = copy.deepcopy(DEPLOY_DESCRIPTOR)
        bad["chamfer"]["side"] = "negative_y"
        with self.assertRaises(ValueError):
            normalize_descriptor(bad)

    def test_explicit_cuboid_fixture_stays_legal(self):
        hull = normalize_descriptor(copy.deepcopy(CUBOID_DESCRIPTOR))
        self.assertIsNone(hull.chamfer)
        self.assertAlmostEqual(
            hull.ceiling_z - hull.floor_z, INNER_SIZE[2], delta=EPS)

    def test_surface_hull_missing_descriptor_is_legacy_none(self):
        from luggage_packing.placement_solver import _surface_hull
        self.assertIsNone(_surface_hull(uld_map(descriptor=None)))

    def test_surface_hull_invalid_descriptor_fails_closed(self):
        """A map that CARRIES a descriptor which does not normalize is a
        foreign map, not a legacy fixture: the hull gate must fail closed
        (named error), never silently degrade to rectangle semantics."""
        from luggage_packing.placement_solver import _surface_hull
        bad = copy.deepcopy(DEPLOY_DESCRIPTOR)
        bad["geometry_hash"] = "0" * 64
        with self.assertRaises(ValueError):
            _surface_hull(uld_map(descriptor=bad))


# ----------------------------------------------------------------------
# H3: final-placement containment through the real selection chain


class TestH3FullChainContainment(unittest.TestCase):
    """The chain under test is the production offline chain:
    ``solve_placement`` = real enumeration -> real gates (solver hull gate
    + optional injected validator) -> frozen sort -> selection. No
    predicate-only shortcuts for the chain claims; the one predicate-level
    test below is labeled as such."""

    @staticmethod
    def _violations(cands, margin, floor_exempt):
        bad = []
        for cand in cands:
            center = cand["center_container"]
            ok = oracle_box_contained(
                center, (cand["footprint"][0], cand["footprint"][1],
                         cand["size"][2]),
                margin=margin, aperture=False,
                floor_margin=0.0 if floor_exempt else margin)
            if not ok:
                bad.append((cand["candidate_id"], center, cand["reason"]))
        return bad

    def test_margin0_no_feasible_candidate_escapes_the_hull(self):
        """Negative-repro core: at the production margin (0), no feasible
        final slot ever puts the box outside the closed seven-face hull --
        grid enumeration bounds every window inside the inner AABB, so the
        aperture predicate cannot smuggle a protruding box through."""
        surface = uld_map(res=0.05, state="unknown", height=0.0,
                          descriptor=DEPLOY_DESCRIPTOR)
        cands, totals = enumerate_all(surface)
        self.assertGreater(totals["n_enumerated"], 0)
        self.assertGreater(len(feasible(cands)), 0)
        self.assertEqual([], self._violations(feasible(cands), 0.0, False))

    def test_margin01_floor_margin_must_not_kill_floor_rest(self):
        """hull_margin=0.01 is a documented eval profile value. The final
        hull gate must charge it to walls/ceiling, never to the floor: a
        floor-resting box touches the floor by design. An empty container
        must answer with feasible floor slots, not BIN_FULL."""
        surface = uld_map(res=0.05, state="unknown", height=0.0,
                          descriptor=DEPLOY_DESCRIPTOR)
        result = solve_placement(
            surface, list(BOX), allowed_yaws=[0.0, math.pi / 2],
            params={"top_n": 10 ** 9, "keep_rejected": 10 ** 9,
                    "hull_margin": 0.01})
        self.assertTrue(result["success"], result.get("message"))
        self.assertEqual([], self._violations(
            feasible(result["candidates"]), 0.01, True))

    def test_margin01_door_face_margin_enforced(self):
        """Final placement includes the -X door face: with hull_margin
        0.01, a window whose box corner sits inside the margin band at the
        door face is a margin violation and must be rejected. (Transit
        through the aperture keeps its own semantics elsewhere.)"""
        surface = uld_map(res=0.05, state="unknown", height=0.0,
                          descriptor=DEPLOY_DESCRIPTOR)
        cands, _ = enumerate_all(
            surface, params={"hull_margin": 0.01})
        self.assertGreater(len(feasible(cands)), 0)
        # No feasible candidate may violate the door-face margin ...
        for cand in feasible(cands):
            center = cand["center_container"]
            corner_x = center[0] - 0.5 * cand["footprint"][0]
            self.assertGreaterEqual(
                corner_x, -RAW_HALF_X + 0.01 - EPS,
                "feasible door-face slot inside the margin band: %s @ %s"
                % (cand["candidate_id"], center))
        # ... and the band is actually populated with rejections near the
        # door (windows whose box corner lands in (-half_x, -half_x+0.01]).
        band = [c for c in cands
                if not c["feasible"] and c["reason"] == REASON_OUTSIDE_HULL
                and (c["center_container"][0] - 0.5 * c["footprint"][0])
                < -RAW_HALF_X + 0.01 + EPS]
        self.assertGreater(len(band), 0)

    def test_map_frame_offset_and_yaw_do_not_change_hull_verdicts(self):
        """The hull gate is container-frame; a non-zero map placement
        (world transform) must not change any per-window verdict."""
        base = uld_map(res=0.05, descriptor=DEPLOY_DESCRIPTOR)
        moved = uld_map(res=0.05, descriptor=DEPLOY_DESCRIPTOR,
                        center_base=[1.2, -0.4, RAW_FLOOR_Z + 0.74],
                        yaw=0.6)
        base_cands, _ = enumerate_all(base)
        moved_cands, _ = enumerate_all(moved)
        key = lambda c: (  # noqa: E731
            c["candidate_id"], c["feasible"], c["reason"],
            round(c["center_container"][0], 9),
            round(c["center_container"][1], 9),
            round(c["center_container"][2], 9))
        self.assertEqual([key(c) for c in base_cands],
                         [key(c) for c in moved_cands])

    def test_envelope_semantics_envelope_is_what_is_gated(self):
        """Nominal-legal-but-envelope-illegal. Chain part: every feasible
        nominal slot is oracle-legal for its own footprint (the gate
        consumes the size it is given). Predicate part (labeled): at one
        shared center near the chamfer line, the final-placement hull
        gate accepts the nominal footprint and rejects the strictly
        larger conservative envelope -- the grid alone cannot discriminate
        both (quantization), so the size semantics is pinned directly."""
        nominal_fp = (0.44, 0.33)
        envelope_fp = (0.48, 0.38)
        surface = uld_map(res=0.05, descriptor=DEPLOY_DESCRIPTOR)
        nominal_c, _ = enumerate_all(surface)
        nominal_feasible = feasible(nominal_c)
        self.assertGreater(len(nominal_feasible), 0)
        for cand in nominal_feasible:
            self.assertTrue(oracle_box_contained(
                cand["center_container"],
                (cand["footprint"][0], cand["footprint"][1],
                 cand["size"][2]), margin=0.0),
                cand["candidate_id"])
        # PREDICATE-LEVEL (labeled): one center whose nominal corner sits
        # inside the chamfer line and whose envelope corner crosses it.
        from luggage_packing.placement_solver import _hull_legal
        center = [0.0, 0.375, RAW_FLOOR_Z + 0.30]
        self.assertTrue(oracle_box_contained(
            center, nominal_fp + (0.60,), margin=0.0))
        self.assertFalse(oracle_box_contained(
            center, envelope_fp + (0.62,), margin=0.0))
        legal_nominal, _ = _hull_legal(
            center, nominal_fp, 0.60, KERNEL, 0.0)
        legal_envelope, _ = _hull_legal(
            center, envelope_fp, 0.62, KERNEL, 0.0)
        self.assertTrue(legal_nominal)
        self.assertFalse(legal_envelope)

    def test_res_02_and_01_chain_containment(self):
        """The margin-0 containment guarantee holds across the frozen
        resolution matrix (0.02 fast path; 0.01 is the deployment grid —
        one enumeration, ~seconds, still deterministic)."""
        for res in (0.02, 0.01):
            surface = uld_map(res=res, descriptor=DEPLOY_DESCRIPTOR)
            cands, totals = enumerate_all(surface)
            self.assertGreater(totals["n_enumerated"], 0)
            self.assertGreater(len(feasible(cands)), 0,
                               "no feasible candidate at res=%s" % res)
            self.assertEqual(
                [], self._violations(feasible(cands), 0.0, False),
                "res=%s feasible slot escapes the hull" % res)

    def test_z_bottom_at_wall_z_boundaries(self):
        """Boxes whose bottom lands 1 mm below / at / 1 mm above the
        chamfer wall_z knee: containment must track the corner-at-knee
        geometry exactly (oracle cross-check per candidate)."""
        for delta in (-0.001, 0.0, 0.001):
            peak = RAW_WALL_Z - RAW_FLOOR_Z + delta
            surface = uld_map(res=0.05, state="sensor", height=peak,
                              descriptor=DEPLOY_DESCRIPTOR)
            cands, _ = enumerate_all(surface)
            self.assertGreater(len(feasible(cands)), 0, delta)
            self.assertEqual(
                [], self._violations(feasible(cands), 0.0, False),
                "delta=%s feasible slot escapes the hull" % delta)

    def test_repeated_enumeration_is_identical(self):
        """Determinism: three runs produce byte-identical candidate
        lists (ids, order, scores, reasons)."""
        surface = uld_map(res=0.05, descriptor=DEPLOY_DESCRIPTOR)
        runs = [enumerate_all(surface)[0] for _ in range(3)]
        for other in runs[1:]:
            self.assertEqual(runs[0], other)

    def test_predicate_level_transit_vs_final_split(self):
        """PREDICATE-LEVEL (labeled): the aperture predicate answers True
        for a box protruding out of the -X door -- correct for route
        transit, wrong for final placement. The chain-level protection is
        proven by the tests above; this pins the kernel split itself."""
        center = (-RAW_HALF_X - 0.10, 0.0, RAW_FLOOR_Z + 0.30)
        size = (0.44, 0.33, 0.60)
        self.assertTrue(contains_oriented_box_through_aperture(
            KERNEL, center, size, margin=0.0))
        self.assertFalse(contains_oriented_box(
            KERNEL, center, size, margin=0.0))
        # And the final-placement entry keeps floor contact legal while
        # charging walls (the floor_margin contract used by the solver).
        self.assertTrue(contains_oriented_box(
            KERNEL, (0.0, 0.0, RAW_FLOOR_Z + 0.30), size, margin=0.01,
            floor_margin=0.0))
        self.assertFalse(contains_oriented_box(
            KERNEL, (0.0, RAW_FLOOR_Y, RAW_FLOOR_Z + 0.30), size,
            margin=0.01, floor_margin=0.0))


# ----------------------------------------------------------------------
# H4: support and chamfer containment are two separate gates


class TestH4SupportChamferSeparation(unittest.TestCase):

    def test_chamfer_slope_is_boundary_not_platform(self):
        """Floor-prior windows whose footprint crosses floor_y into the
        chamfer slope must be rejected by the hull gate; the same windows
        are legal under an explicit cuboid descriptor. Unknown slope cells
        are never silently flattened into floor."""
        chamfered = uld_map(res=0.05, descriptor=DEPLOY_DESCRIPTOR)
        cuboid = uld_map(res=0.05, descriptor=CUBOID_DESCRIPTOR)
        cands, _ = enumerate_all(chamfered)
        cub_cands, _ = enumerate_all(cuboid)
        cub_feasible = {c["candidate_id"] for c in feasible(cub_cands)}
        slope_rejected = [
            c for c in cands
            if c["candidate_id"] in cub_feasible
            and not c["feasible"]
            and c["reason"] == REASON_OUTSIDE_HULL]
        self.assertGreater(
            len(slope_rejected), 0,
            "chamfer gate is not load-bearing: no cuboid-legal window was "
            "rejected by the chamfered hull")
        for cand in slope_rejected:
            center = cand["center_container"]
            self.assertFalse(oracle_box_contained(
                center, (cand["footprint"][0], cand["footprint"][1],
                         cand["size"][2]), margin=0.0))

    def test_hull_legal_but_support_insufficient(self):
        """Hull containment says nothing about support: a window whose
        contact ratio is below the frozen 0.6 is rejected by the support
        gate even though the pose is fully inside the hull."""
        surface = uld_map(res=0.05, state="sensor", height=0.0,
                          descriptor=DEPLOY_DESCRIPTOR)
        # Left half of the container floor observed 0.20 m high, right
        # half at 0: a footprint straddling the step with a contact ratio
        # below the frozen 0.6 must be lost to the support gate, not the
        # hull gate.
        nx, ny = surface["nx"], surface["ny"]
        cells_x, cells_y = 9, 7  # 0.44 x 0.33 at 0.05
        boundary = nx // 2
        for ix in range(nx):
            for iy in range(ny):
                if ix < boundary:
                    surface["height"][ix][iy] = 0.20
        cands, _ = enumerate_all(surface)
        checked = 0
        for cand in cands:
            if cand["footprint_class"] != 0:
                continue
            high_cols = max(0, min(cand["ix0"] + cells_x, boundary)
                            - cand["ix0"])
            ratio = high_cols / float(cells_x)
            if not (0.0 < ratio < 0.6):
                continue  # not a discriminating straddle window
            center = cand["center_container"]
            if not oracle_box_contained(
                    center, (cand["footprint"][0], cand["footprint"][1],
                             cand["size"][2]), margin=0.0):
                continue  # hull violations are covered by their own tests
            checked += 1
            self.assertFalse(cand["feasible"], cand["candidate_id"])
            self.assertEqual(cand["reason"], REASON_INSUFFICIENT_SUPPORT)
        self.assertGreater(
            checked, 0, "no hull-legal low-ratio straddle window found")

    def test_unknown_above_floor_is_never_blind_support(self):
        surface = uld_map(res=0.05, state="unknown", height=0.0,
                          descriptor=DEPLOY_DESCRIPTOR)
        nx, ny = surface["nx"], surface["ny"]
        for ix in range(nx):
            for iy in range(ny):
                surface["height"][ix][iy] = 0.30
        cands, _ = enumerate_all(surface)
        feasible_ids = [c["candidate_id"] for c in feasible(cands)]
        self.assertEqual(
            [], feasible_ids,
            "unobserved 0.30 m columns must not become blind support")
        reasons = {c["reason"] for c in cands if not c["feasible"]}
        self.assertIn(REASON_UNKNOWN_ABOVE_FLOOR, reasons)


# ----------------------------------------------------------------------
# H3 node adapter (real callback; skipped without rclpy)


def _load_node_module():
    import importlib.util
    path = os.path.join(PKG_ROOT, "scripts", "placement_planner_node.py")
    spec = importlib.util.spec_from_file_location(
        "placement_planner_node_hg3", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TestH3NodeAdapterLive(unittest.TestCase):
    """The real node adapter (``_constraint_verdict`` with the node's own
    hull_contains callback) plus one full service round trip. Launch-free:
    one planner node in one executor, no Gazebo, no MoveIt."""

    @classmethod
    def setUpClass(cls):
        try:
            import rclpy  # noqa: F401
            from luggage_msgs.srv import ComputePlacement  # noqa: F401
        except ImportError as exc:
            raise unittest.SkipTest("rclpy/luggage_msgs unavailable: %s" % exc)
        cls.module = _load_node_module()

    def setUp(self):
        from luggage_description._share import ENV_SCENE_TF
        import rclpy
        from rclpy.executors import SingleThreadedExecutor

        self.rclpy = rclpy
        rclpy.init()
        self.executor = SingleThreadedExecutor()
        self.nodes = []
        self.addCleanup(self._shutdown)

        previous = os.environ.get(ENV_SCENE_TF)
        os.environ[ENV_SCENE_TF] = SCENE_PATH
        try:
            self.planner = self.module.PlacementPlannerNode()
        finally:
            if previous is None:
                os.environ.pop(ENV_SCENE_TF, None)
            else:
                os.environ[ENV_SCENE_TF] = previous
        self.nodes.append(self.planner)
        self.executor.add_node(self.planner)

    def _shutdown(self):
        for node in self.nodes:
            self.executor.remove_node(node)
            node.destroy_node()
        self.executor.shutdown()
        self.rclpy.shutdown()

    def _spin_until(self, predicate, timeout_sec=10.0):
        import time
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)
            if predicate():
                return True
        return False

    def _door_face_candidate(self):
        """A feasible candidate whose Y span sits inside the aperture
        shadow, so the door-face shift is judged by the hull gate, not
        pre-empted by the (correctly ordered) aperture gate."""
        surface = uld_map(res=0.05, descriptor=DEPLOY_DESCRIPTOR)
        cands, _ = enumerate_all(surface)
        for cand in feasible(cands):
            half_w = 0.5 * cand["footprint"][1]
            cy = cand["center_container"][1]
            if -0.928 + 1e-3 <= cy - half_w and cy + half_w <= 0.398 - 1e-3:
                return cand, cands
        raise AssertionError("no feasible candidate inside the aperture "
                             "Y shadow")

    def test_real_adapter_rejects_door_face_margin_violation(self):
        from rclpy.parameter import Parameter

        self.planner.set_parameters(
            [Parameter("hull_margin", value=0.01)])
        try:
            best, _cands = self._door_face_candidate()
            # The door-flush twin of a real feasible candidate: same pose
            # shifted so the box touches x=-half_x exactly. At the eval
            # profile margin 0.01 that corner sits inside the margin band.
            cand = copy.deepcopy(best)
            cand["center_container"] = [
                -RAW_HALF_X + 0.5 * cand["footprint"][0],
                best["center_container"][1],
                best["center_container"][2]]
            reason, _cap = self.planner._constraint_verdict(
                cand, [], list(BOX))
            self.assertEqual(reason, "outside_hull",
                             "real adapter must reject a door-face margin "
                             "violation (got %r)" % reason)
            # ... and a genuinely protruding box is rejected even at
            # margin 0 (the band-independent case).
            self.planner.set_parameters(
                [Parameter("hull_margin", value=0.0)])
            cand["center_container"] = [
                -RAW_HALF_X - 0.01 + 0.5 * cand["footprint"][0],
                best["center_container"][1],
                best["center_container"][2]]
            reason, _cap = self.planner._constraint_verdict(
                cand, [], list(BOX))
            self.assertEqual(reason, "outside_hull",
                             "real adapter must reject a protruding box "
                             "(got %r)" % reason)
        finally:
            self.planner.set_parameters(
                [Parameter("hull_margin", value=0.0)])

    def test_service_round_trip_slot_fully_contained(self):
        """Full production round trip: publish the cargo map on the real
        transport, call the real service, then verify EVERY feasible
        candidate in the latched last_result dump (container frame) plus
        the preferred final slot against the oracle."""
        import json

        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from std_msgs.msg import String

        surface = uld_map(res=0.05, descriptor=DEPLOY_DESCRIPTOR)
        # [G1] Admission is mandatory: the map carries the observer-bound
        # identity and a latched VALID validity publication plus a fresh
        # joint_states sample open the gate (the pre-G1 test published a
        # bare map against a gate that then did not bind).
        surface["epoch"] = 1
        surface["validity_generation"] = 3
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        last = {}
        self.planner.create_subscription(
            String, "/placement_planner/last_result",
            lambda msg: last.update(payload=json.loads(msg.data)), qos)
        validity_pub = self.planner.create_publisher(
            String, "/uld_occ_observer/validity", qos)
        validity_pub.publish(String(data=json.dumps({
            "state": "VALID", "epoch": 1, "generation": 3,
            "reason": "observed", "reset_id": "",
            "transition_mono_ns": 1, "map_revision": surface.get(
                "map_revision", 1) or 1, "observed_revision": surface.get(
                "map_revision", 1) or 1, "last_integrated_stamp_ns": 1,
            "geometry_hash": surface.get("geometry_hash", ""),
            "config_digest": "", "clock_domain": "ros"})))
        pub = self.planner.create_publisher(
            String, "/luggage/cargo_map/surface_2d", qos)
        # The validity stream must bind BEFORE the map arrives (an
        # admission attempt against the boot sentinel is a final reject
        # for that delivery; the map is not re-delivered).
        self.assertTrue(
            self._spin_until(lambda: self.planner._validity_seen
                             and self.planner._admission.validity.state
                             == "VALID"),
            "planner never bound the validity stream")
        # The recovery rule needs a robot sample strictly NEWER than the
        # validity arrival, so it is captured (and stamped) afterwards.
        from sensor_msgs.msg import JointState
        from builtin_interfaces.msg import Time as TimeMsg
        import time as time_module
        now = time_module.time()
        robot = JointState()
        robot.header.stamp = TimeMsg(
            sec=int(now), nanosec=int((now % 1) * 1e9))
        robot.name = ["joint_1"]
        robot_pub = self.planner.create_publisher(
            JointState, "/joint_states", 10)
        robot_pub.publish(robot)
        self.assertTrue(
            self._spin_until(lambda: self.planner._admission.robot_state
                             is not None),
            "planner never recorded the robot sample")
        pub.publish(String(data=json.dumps(surface)))
        self.assertTrue(
            self._spin_until(lambda: self.planner._surface is not None),
            "planner never admitted the cargo map")

        from luggage_msgs.msg import DetectedLuggage
        from luggage_msgs.srv import ComputePlacement
        request = ComputePlacement.Request()
        box = DetectedLuggage()
        box.width, box.depth, box.height = BOX
        box.height_valid = True
        box.height_source = 1
        box.pose.orientation.w = 1.0
        request.box = box
        client = self.planner.create_client(
            ComputePlacement, "/placement_planner/compute_placement")
        self.assertTrue(self._spin_until(client.service_is_ready, 10.0))
        future = client.call_async(request)
        self.assertTrue(self._spin_until(future.done, 60.0),
                        "ComputePlacement timed out")
        response = future.result()
        self.assertTrue(response.success, response.message)
        self.assertGreater(len(response.candidates), 0)
        self.assertNotEqual(
            (response.slot.width, response.slot.depth,
             response.slot.height), (0.0, 0.0, 0.0))
        self.assertTrue(
            self._spin_until(lambda: last.get("payload", {}).get("success")),
            "last_result dump never arrived")
        dump = last["payload"]
        rows = [c for c in dump.get("candidates", [])
                if c.get("feasible")]
        self.assertGreater(len(rows), 0)
        for cand in rows:
            self.assertTrue(
                oracle_box_contained(
                    cand["center_container"],
                    (cand["footprint"][0], cand["footprint"][1],
                     cand["size"][2]),
                    margin=0.0, aperture=False),
                cand["center_container"])
        preferred = dump.get("preferred_candidate") or {}
        self.assertTrue(oracle_box_contained(
            preferred["center_container"],
            (preferred["footprint"][0], preferred["footprint"][1],
             preferred["size"][2]), margin=0.0, aperture=False),
            preferred["center_container"])


if __name__ == "__main__":
    unittest.main()
