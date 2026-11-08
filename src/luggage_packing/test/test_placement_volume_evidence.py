"""Same-snapshot volume authorization (reviews 2026-11-08_1751 finding 2).

A 2D surface cannot distinguish "the space above an observed support top
is observed" from "the space above it is unknown" — both read as an
occupied column with the same top. These tests pin the product-side
gate: with a volume_verifier bound to the same snapshot's voxel
evidence, the SAME support surface authorizes stacking when the box
volume reads observed free, and rejects it when any voxel inside the
candidate's own volume — boundary layers included — is unknown or
occupied. Evidence that does not cover the volume rejects fail-closed.
"""
from __future__ import annotations

import math
import os
import unittest

import pytest

try:
    from luggage_packing.luggage_packing import placement_solver as ps
except ImportError:  # ament-flattened install
    from luggage_packing import placement_solver as ps

RES = 0.02
NX, NY, NZ = 75, 99, 74          # grid 1.50 x 1.98 x 1.48
HX, HY = 1.49 / 2.0, 1.97 / 2.0
FLOOR = 0.53
CEIL = 2.01
INNER_H = CEIL - FLOOR
BOX = [0.10, 0.10, 0.10]
CARGO = {(ix, iy) for ix in range(28, 33) for iy in range(38, 43)}
STACK_Z = FLOOR + 0.20 + 0.05    # box center resting on the cargo top


def _surface_with_cargo():
    """surface_map_2d v1 dict: a 5x5 observed cargo block (tops at 0.20
    floor-relative) on an otherwise observed-free container — the
    stacking-positive surface from test_placement_edge_occupancy."""
    def states(ix, iy):
        if (ix, iy) in CARGO:
            return "occupied"
        return "free"

    return {
        "resolution": RES, "nx": NX, "ny": NY,
        "inner_size": [1.49, 1.97, INNER_H],
        "floor_z": 0.0,
        "center_base": [0.0, 0.0, FLOOR + INNER_H / 2.0],
        "yaw": 0.0,
        "height": [[(0.20 if (ix, iy) in CARGO else 0.0) for iy in range(NY)]
                   for ix in range(NX)],
        "state": [[states(ix, iy) for iy in range(NY)] for ix in range(NX)],
        "confidence": [["sensor" for _ in range(NY)] for _ in range(NX)],
        # the deployment chamfer hull (user ruling 2026-11-08: the real
        # chain is chamfer-only; the cuboid fixture was deleted)
        "geometry_descriptor": {
            "length": 1.49, "width": 1.97, "ceiling_z": CEIL,
            "floor_z": FLOOR, "frame_id": "container_link",
            "schema_version": 1,
            "chamfer": {"floor_y": 0.55, "side": "positive_y",
                        "wall_y": 0.985, "wall_z": 0.9},
            "geometry_hash": "cb41f55f243786d50ab30997291083ab"
                             "40467507abcf78a00e55da77ffc00856",
        },
    }


def _index(ix, iy, iz):
    return ix + NX * (iy + NY * iz)


def _evidence(above="free", unknown_layers=()):
    """Same-snapshot evidence: cargo occupied up to 0.20 (layers 0..9);
    everything above free unless a boundary-layer control overrides."""
    states = [1] * (NX * NY * NZ)
    for (ix, iy) in CARGO:
        for iz in range(10):
            states[_index(ix, iy, iz)] = 2
    for iz in unknown_layers:
        for index in range(NX * NY * iz, NX * NY * (iz + 1)):
            states[index] = 0
    if above == "unknown":
        for index in range(NX * NY * 10, NX * NY * NZ):
            states[index] = 0
    return {"resolution": RES, "nx": NX, "ny": NY, "nz": NZ,
            "origin_xy": [-NX * RES / 2.0, -NY * RES / 2.0],
            "floor_z": FLOOR, "states": states}


def _stacking(c):
    return (c.get("feasible") and c.get("support_source") == "sensor"
            and abs(c["center_container"][2] - STACK_Z) < 1e-6)


def _params():
    params = dict(ps._default_params())
    params["top_n"] = 8000
    params["keep_rejected"] = 8000   # default 6 drops rejected candidates
    return params


def test_same_support_unknown_above_rejects_observed_free_allows():
    """Reviews' required behavior pair on ONE support surface: with the
    box volume above the support top unknown, the sensor candidate is
    rejected; with it observed free, the same candidate is authorized
    (and carries volume_verified). Without a verifier the surface alone
    still offers the stack — the counterfactual that makes the pair a
    product-behavior proof, not a fixture property."""
    surface = _surface_with_cargo()
    plain = ps.generate_candidates(surface, BOX, allowed_yaws=[0.0],
                                   params=_params())
    plain_sensor = [c for c in plain if c.get("feasible")
                    and c.get("support_source") == "sensor"]
    assert any(_stacking(c) for c in plain_sensor), (
        "counterfactual: the surface must offer the stacking candidate")

    guarded = ps.generate_candidates(
        surface, BOX, allowed_yaws=[0.0], params=_params(),
        volume_verifier=ps.make_volume_verifier(_evidence(above="unknown")))
    sensor = [c for c in guarded if c.get("feasible")
              and c.get("support_source") == "sensor"]
    assert sensor, "floor-level placements stay authorized (selective gate)"
    assert not any(_stacking(c) for c in sensor), (
        "unknown above the support must never be authorized as known-free")
    rejected = [c for c in guarded
                if c.get("reason") == ps.REASON_VOLUME_UNKNOWN]
    assert rejected and any(
        abs(c["center_container"][2] - STACK_Z) < 1e-6 for c in rejected), (
        "the stacking candidates must reject with volume_unknown")

    authorized = ps.generate_candidates(
        surface, BOX, allowed_yaws=[0.0], params=_params(),
        volume_verifier=ps.make_volume_verifier(_evidence(above="free")))
    ok_sensor = [c for c in authorized if c.get("feasible")
                 and c.get("support_source") == "sensor"]
    assert any(_stacking(c) for c in ok_sensor)
    assert all(c.get("volume_verified") for c in ok_sensor), (
        "every authorized sensor candidate is volume-bound")


def test_boundary_layers_are_actually_read():
    """Complete XYZ ranges: for a box at floor-relative [0.20, 0.40) the
    checked layers are 10..19 — an unknown TOP or BOTTOM boundary layer
    alone must reject (each carries 2cm of real thickness), while the
    support-contact layer 9 below the box is face contact, not volume
    overlap, and must not reject."""
    center = [-0.135, -0.175, FLOOR + 0.30]   # over the cargo block
    footprint = (0.10, 0.10)

    def verdict(bottom_rel, top_rel, unknown_layers=()):
        bottom = FLOOR + bottom_rel
        top = FLOOR + top_rel
        return ps.volume_state_in_evidence(
            _evidence(unknown_layers=unknown_layers),
            [center[0], center[1], (bottom + top) / 2.0],
            footprint, bottom, top)

    assert verdict(0.20, 0.40) is None, "fully observed volume passes"
    assert verdict(0.20, 0.40, unknown_layers=(19,)) == (
        ps.REASON_VOLUME_UNKNOWN), "top boundary layer must be read"
    assert verdict(0.20, 0.40, unknown_layers=(10,)) == (
        ps.REASON_VOLUME_UNKNOWN), "bottom boundary layer must be read"
    assert verdict(0.20, 0.30) is None, "shorter box on the same support"
    # for [0.20, 0.30) the complete positive-overlap range is 10..14
    # (layer 15 starts at the box top: face contact, zero overlap)
    assert verdict(0.20, 0.30, unknown_layers=(14,)) == (
        ps.REASON_VOLUME_UNKNOWN)
    assert verdict(0.20, 0.30, unknown_layers=(15,)) is None, (
        "layer 15 starts at the box top — zero overlap is not volume")
    # layer 9 is the cargo top cell the box RESTS on: face contact is
    # not volume overlap (its occupied state must not reject the box).
    evidence = _evidence()
    for (ix, iy) in CARGO:
        evidence["states"][_index(ix, iy, 9)] = 2
    assert ps.volume_state_in_evidence(
        evidence, center, footprint, FLOOR + 0.20, FLOOR + 0.30) is None
    # occupied INSIDE the box volume rejects as a collision class
    evidence = _evidence()
    evidence["states"][_index(30, 40, 12)] = 2
    assert ps.volume_state_in_evidence(
        evidence, center, footprint, FLOOR + 0.20, FLOOR + 0.30) == (
        ps.REASON_VOLUME_OCCUPIED)


def test_short_or_uncovered_evidence_rejects_fail_closed():
    """ROI insufficiency / empty range / uncovered volume are NOT proofs:
    the fail-closed reason is volume_evidence_unavailable."""
    center = [-0.135, -0.175, FLOOR + 0.25]
    footprint = (0.10, 0.10)

    short = _evidence()
    short["nz"] = 15               # the [0.20, 0.40) box needs layer 19
    assert ps.volume_state_in_evidence(
        short, center, footprint, FLOOR + 0.20, FLOOR + 0.40) == (
        ps.REASON_VOLUME_UNVERIFIED)

    outside = _evidence()
    outside["origin_xy"] = [0.0, 0.0]   # grid no longer under the box
    assert ps.volume_state_in_evidence(
        outside, center, footprint, FLOOR + 0.20, FLOOR + 0.30) == (
        ps.REASON_VOLUME_UNVERIFIED)

    # a degenerate zero-height range is not a proof of anything
    assert ps.volume_state_in_evidence(
        _evidence(), center, footprint, FLOOR + 0.20, FLOOR + 0.20) == (
        ps.REASON_VOLUME_UNVERIFIED)

    broken = ps.make_volume_verifier({"error": "cells_short_3_of_10"})
    assert ps.REASON_VOLUME_UNVERIFIED in broken({"center_container": [0, 0, 0],
                                                  "footprint": [1, 1],
                                                  "size": [1, 1, 1]})
    for bad in (None, {}, {"error": "cells_status_NO_DATA"}):
        assert ps.REASON_VOLUME_UNVERIFIED in ps.make_volume_verifier(bad)(
            {"center_container": [0, 0, 0], "footprint": [1, 1],
             "size": [1, 1, 1]})


def test_volume_evidence_from_pages_assembly():
    """Paged same-snapshot reads: identity binding and completeness are
    fail-closed; a good assembly yields working evidence."""
    identity = {"geometry_hash": "h", "map_revision": 7, "epoch": 1,
                "observed_revision": 3}
    page = lambda states, ident: {"status": "OK", "states": states,
                                  "identity": ident}
    good = ps.volume_evidence_from_pages(
        [page([1] * 20, identity), page([1] * 10, identity)],
        resolution=RES, nx=2, ny=3, nz=5,
        origin_xy=[0.0, 0.0], floor_z=FLOOR, identity=identity)
    assert "error" not in good and len(good["states"]) == 30

    drifted = dict(identity, map_revision=8)
    assert "error" in ps.volume_evidence_from_pages(
        [page([1] * 20, identity), page([1] * 10, drifted)],
        resolution=RES, nx=2, ny=3, nz=5,
        origin_xy=[0.0, 0.0], floor_z=FLOOR, identity=identity)
    assert "error" in ps.volume_evidence_from_pages(
        [page([1] * 20, identity)],           # short assembly
        resolution=RES, nx=2, ny=3, nz=5,
        origin_xy=[0.0, 0.0], floor_z=FLOOR, identity=identity)
    assert "error" in ps.volume_evidence_from_pages(
        [{"status": "INVALID_ROI_OR_PAGE", "states": [],
          "identity": identity}],
        resolution=RES, nx=2, ny=3, nz=5,
        origin_xy=[0.0, 0.0], floor_z=FLOOR, identity=identity)


def test_frozen_tolerance_values():
    """The unified frozen rules (reviews 2026-11-08_1751 finding 3):
    penetration at 1e-9 (the oracle's float_tolerance_m) and the 1e-4
    per-axis sliver floor (the oracle's _OVERLAP_MIN_M). Cross-equality
    with the independent oracle is asserted in the campaign proof
    scripts (product tests cannot import tooling)."""
    assert ps.PENETRATION_EPS_M == 1e-9
    assert ps.OVERLAP_MIN_M == 1e-4


def test_raising_verifier_rejects_fail_closed():
    surface = _surface_with_cargo()
    calls = []

    def exploding(candidate):
        calls.append(candidate["candidate_id"])
        raise RuntimeError("observer lease expired")

    result = ps.generate_candidates(surface, BOX, allowed_yaws=[0.0],
                                    params=_params(),
                                    volume_verifier=exploding)
    sensor = [c for c in result if c.get("support_source") == "sensor"]
    assert calls, "the verifier must be consulted"
    assert any(c.get("reason") == ps.REASON_VOLUME_UNVERIFIED for c in sensor)
    assert not any(c.get("feasible") for c in sensor), (
        "a broken evidence path authorizes nothing")


# ---------------------------------------------------------------------------
# Node boundary: single-lease compute (user ruling 2026-11-08 — one lease
# supplies BOTH the surface and the volume evidence; no mixed two-reader
# mode, failures fail the request closed).

def _load_node_module():
    import importlib.util
    path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "scripts", "placement_planner_node.py")
    spec = importlib.util.spec_from_file_location(
        "placement_planner_node_vol", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TestNodeComputeInputs(unittest.TestCase):
    """The compute-inputs factory at the node boundary: default keeps the
    legacy admitted-surface path with no verifier; the observer source
    requires the whole single-lease chain and fails closed without it
    (the real chain is exercised live end-to-end)."""

    @classmethod
    def setUpClass(cls):
        try:
            import rclpy  # noqa: F401
            from luggage_msgs.srv import ComputePlacement  # noqa: F401
            from uld_occ_observer.srv import GetSnapshot  # noqa: F401
        except ImportError as exc:
            raise unittest.SkipTest("rclpy/luggage_msgs unavailable: %s" % exc)
        cls.module = _load_node_module()

    def setUp(self):
        import threading

        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from luggage_description._share import ENV_SCENE_TF
        pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        scene_path = os.path.join(
            pkg_root, "..", "luggage_description", "config", "scene_tf.yaml")
        previous = os.environ.get(ENV_SCENE_TF)
        os.environ[ENV_SCENE_TF] = scene_path
        self._previous_scene_env = (ENV_SCENE_TF, previous)
        rclpy.init()
        self.rclpy = rclpy
        self.executor = SingleThreadedExecutor()
        self._spin = threading.Thread(target=self.executor.spin, daemon=True)
        self.planner = self.module.PlacementPlannerNode()
        self.executor.add_node(self.planner)
        self._spin.start()
        self.addCleanup(self._shutdown)

    def _shutdown(self):
        self.executor.remove_node(self.planner)
        self.planner.destroy_node()
        self.executor.shutdown()
        self.rclpy.shutdown()
        key, previous = self._previous_scene_env
        if previous is None:
            os.environ.pop(key, None)
        else:
            os.environ[key] = previous

    def _set(self, name, value):
        self.planner.set_parameters([
            self.rclpy.parameter.Parameter(name, value=value)])

    def test_default_source_keeps_legacy_path(self):
        self._set("volume_evidence_source", "")
        self._set("structural_search", False)
        surface, verifier = self.planner._compute_inputs()
        assert verifier is None
        assert surface == self.planner._active_surface()

    def test_unknown_backend_fails_closed(self):
        self._set("volume_evidence_source", "not_a_backend")
        with pytest.raises(ValueError):
            self.planner._compute_inputs()

    def test_observer_without_services_fails_closed(self):
        """No observer runs in this environment: the single-lease chain
        cannot produce (surface, evidence), so nothing may authorize.
        The live chain is proven by the end-to-end run."""
        self._set("volume_evidence_source", "observer")
        self._set("volume_service_timeout_s", 0.2)
        with pytest.raises(RuntimeError):
            self.planner._compute_inputs()
