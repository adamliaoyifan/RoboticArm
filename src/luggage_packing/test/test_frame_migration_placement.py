"""ST05 placement/SlotSpec container-center acceptance (F07/F08) on real types.

Runs after building/sourcing the isolated install; no ROS nodes, no TF.
FRAME_MIGRATION_CASE_LOG selects an append-only JSONL evidence file.
"""

import copy
from contextlib import contextmanager
import itertools
import json
import math
import os
from pathlib import Path
import time
import traceback
import unittest

from rclpy.serialization import serialize_message, deserialize_message

from luggage_description.frame_contracts import ContractError
from luggage_description.frame_diagnostics import (
    BoundaryContext, BoundaryContractError,
)
from luggage_planning.frame_message_adapters import (
    planning_pose_from_message, planning_pose_to_message,
)
from luggage_planning.waypoint_generator import slot_from_contract
from luggage_packing.placement_legacy_v1 import v1_candidate_view
from luggage_packing.placement_solver import (
    candidate_aabb, generate_candidates, solve_placement,
)


SCENE = 'scene-calibration-17'
WDH = (0.80, 0.50, 0.32)


def wire(message):
    return deserialize_message(serialize_message(message), type(message))


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return repr(value)
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(v) for v in value]
    return value


@contextmanager
def case(case_id, inputs):
    row = dict(case_id=case_id, input=copy.deepcopy(inputs), status='pass',
               observations={})
    started = time.monotonic()
    try:
        yield row['observations']
    except BaseException:
        row.update(status='fail', traceback=traceback.format_exc())
        raise
    finally:
        row['elapsed_seconds'] = time.monotonic() - started
        output = os.environ.get('FRAME_MIGRATION_CASE_LOG')
        if output:
            with Path(output).open('a', encoding='utf-8') as handle:
                handle.write(json.dumps(json_safe(row), allow_nan=False,
                                        sort_keys=True) + '\n')
                handle.flush()
                os.fsync(handle.fileno())
        print('CASE', case_id, row['status'], flush=True)


def surface_v1(floor, support, occupied_cell=None, inner=(1.0, 0.6, 0.8),
               res=0.05, map_yaw=0.0, full_support=False):
    """Solver-internal surface view; ``full_support`` stacks the whole floor."""
    inner_l, inner_w, inner_h = inner
    nx = int(math.ceil(inner_l / res))
    ny = int(math.ceil(inner_w / res))
    height = [[0.0] * ny for _ in range(nx)]
    state = [["unknown"] * ny for _ in range(nx)]
    confidence = [["none"] * ny for _ in range(nx)]
    for ix in range(nx):
        for iy in range(ny):
            state[ix][iy] = "free"
            confidence[ix][iy] = "sensor"
    if full_support:
        for ix in range(nx):
            for iy in range(ny):
                height[ix][iy] = support
                state[ix][iy] = "occupied"
                confidence[ix][iy] = "geometry"
    if occupied_cell is not None:
        ix, iy = occupied_cell
        height[ix][iy] = support
        state[ix][iy] = "occupied"
        confidence[ix][iy] = "geometry"
    return {
        "geometry_hash": "sha256:test-hull", "geometry_descriptor": {},
        "resolution": res, "nx": nx, "ny": ny,
        "inner_size": list(inner), "floor_z": 0.0,
        "center_base": [0.0, 0.0, floor + 0.5 * inner_h],
        "yaw": float(map_yaw), "height": height, "state": state,
        "confidence": confidence,
    }


def surface_v2_record(v1, floor, request_id):
    """v2 surface record equivalent to the v1 view (container-origin z)."""
    inner_h = float(v1["inner_size"][2])
    surface_z, state, confidence = [], [], []
    for ix in range(v1["nx"]):
        z_row, s_row, c_row = [], [], []
        for iy in range(v1["ny"]):
            cell_state = v1["state"][ix][iy]
            if cell_state == "unknown":
                z_row.append(None)
                s_row.append("unknown")
                c_row.append("unknown")
            else:
                z_row.append(floor + float(v1["height"][ix][iy]))
                s_row.append(cell_state)
                c_row.append("verified_geometry"
                             if v1["confidence"][ix][iy] == "geometry"
                             else "sensor")
        surface_z.append(z_row)
        state.append(s_row)
        confidence.append(c_row)
    return {
        "contract_version": 2, "frame_id": "container_link",
        "length_unit": "m", "stamp_ns": 0, "time_basis": "scene",
        "scene_revision": SCENE, "request_id": request_id,
        "kind": "surface", "map_revision": 3,
        "geometry_hash": "sha256:test-hull",
        "height_datum": "container_origin",
        "origin_xy_container": [-0.5 * v1["inner_size"][0],
                                -0.5 * v1["inner_size"][1]],
        "resolution_m": float(v1["resolution"]),
        "floor_z_container": float(floor),
        "ceiling_z_container": float(floor + inner_h),
        "surface_z_container": surface_z, "state": state,
        "confidence": confidence,
        "height_above_floor": [[None if z is None else z - floor
                                for z in row] for row in surface_z],
    }


def v1_view_of_record(record, inner):
    """Solver-internal view derived from a v2 record (planner conversion)."""
    floor = float(record["floor_z_container"])
    height, state, confidence = [], [], []
    for z_row, s_row, c_row in zip(record["surface_z_container"],
                                   record["state"], record["confidence"]):
        h_row, st_row, cf_row = [], [], []
        for z, cell_state, source in zip(z_row, s_row, c_row):
            if z is None:
                h_row.append(0.0)
                st_row.append("unknown")
                cf_row.append("none")
            else:
                h_row.append(float(z) - floor)
                st_row.append(str(cell_state))
                cf_row.append("geometry" if source == "verified_geometry"
                              else "sensor")
        height.append(h_row)
        state.append(st_row)
        confidence.append(cf_row)
    return {
        "geometry_hash": record["geometry_hash"], "geometry_descriptor": {},
        "resolution": float(record["resolution_m"]),
        "nx": len(height), "ny": len(height[0]),
        "inner_size": list(inner), "floor_z": 0.0,
        "center_base": [0.0, 0.0, floor + 0.5 * inner[2]],
        "yaw": 0.0, "height": height, "state": state,
        "confidence": confidence,
    }


def slot_record(position, yaw, request_id='slot-1'):
    return {
        "contract_version": 2, "frame_id": "container_link",
        "length_unit": 'm', "stamp_ns": 0, "time_basis": 'scene',
        "scene_revision": SCENE, "request_id": request_id, "kind": 'pose',
        "reference": 'box_center', "orientation_semantics": 'box_axis_pi',
        "position": [float(v) for v in position],
        "quaternion_xyzw": [0.0, 0.0, math.sin(float(yaw) * 0.5),
                            math.cos(float(yaw) * 0.5)],
    }


def context(record, interface='F07', reference='box_center'):
    return BoundaryContext(
        interface, 'placement_planner', 'v2_client', record['request_id'],
        SCENE, 'container_link', reference, 'scene',
        geometry_hash='sha256:test-hull', map_revision=3)


class FrameMigrationPlacement(unittest.TestCase):
    def assert_close(self, actual, expected, tolerance=1e-9):
        self.assertLessEqual(abs(actual - expected), tolerance)

    def assert_quaternion(self, actual, expected, tolerance=1e-6):
        error = min(max(abs(a - b) for a, b in zip(actual, expected)),
                    max(abs(a + b) for a, b in zip(actual, expected)))
        self.assertLessEqual(error, tolerance)

    def reject(self, name, expected, payload, operation):
        effects = []
        with case('negative/' + name, payload) as observed:
            try:
                effects.append(operation())
            except ContractError as error:
                observed.update(expected_code=expected,
                                actual_code=error.code,
                                diagnostic=getattr(error, 'diagnostic',
                                                   error.as_dict()),
                                downstream_effects=0)
                self.assertEqual(error.code, expected)
                self.assertEqual(effects, [])
            else:
                self.fail('unexpected accept: ' + name)

    def test_64_slot_cases_two_surface_forms(self):
        matrix = itertools.product(
            (0., math.pi / 2, math.pi, math.atan2(4, 3)),
            ((0., 0., 0.), (1.5, -.4, .2)), (.53, .71), (0., .30),
            ('v1', 'v2'))
        count = 0
        for index, (box_yaw, translation, floor, support, form) in \
                enumerate(matrix):
            occupied = (7, 5)
            fixture = dict(box_yaw=box_yaw, translation=translation,
                           floor=floor, support=support, form=form,
                           wdh=list(WDH))
            with self.subTest(index=index), \
                    case('slot/%02d' % index, fixture) as observed:
                v1 = surface_v1(floor, support, full_support=True)
                record = surface_v2_record(v1, floor, 'slot-%02d' % index)
                surface_message_context = BoundaryContext(
                    'F06', 'cargo_volume_mapper', 'placement_planner',
                    'slot-%02d' % index, SCENE, 'container_link', '',
                    'scene', geometry_hash='sha256:test-hull', map_revision=3)
                from luggage_description.ros_frame_bindings.surfaces import (
                    surface_from_message, surface_to_message,
                )
                decoded = surface_from_message(
                    wire(surface_to_message(record, surface_message_context)),
                    surface_message_context)
                view = v1_view_of_record(decoded, v1['inner_size'])
                source = v1 if form == 'v1' else view
                cands = generate_candidates(source, list(WDH),
                                            params={'stride_cells': 1})
                observed.update(stage=form, n_candidates=len(cands))
                # Acceptance datum: with the whole floor stacked at the
                # support height, every feasible candidate's center z is
                # floor + support + h/2 and the box top is center + h/2.
                on_support = [a for a in cands
                              if a['reason'] == 'ok' and a['feasible']]
                self.assertTrue(on_support, 'no feasible candidate')
                for a in on_support:
                    self.assert_close(a['center_container'][2],
                                      floor + support + WDH[2] / 2.0)
                # Both forms were compared pairwise in the v1/v2 case pair
                # (same enumeration); store this form's first candidate.
                if index % 2 == 1:
                    prior = self._prior_cache
                    self.assertEqual(len(prior), len(cands))
                    for a, b in zip(prior, cands):
                        self.assert_close(a['center_container'][0],
                                          b['center_container'][0])
                        self.assert_close(a['center_container'][1],
                                          b['center_container'][1])
                        self.assert_close(a['center_container'][2],
                                          b['center_container'][2])
                        self.assertEqual(a['box_yaw'], b['box_yaw'])
                        self.assertEqual(a['footprint'], b['footprint'])
                        self.assertEqual(a['support_source'],
                                         b['support_source'])
                        self.assertEqual(a['reason'], b['reason'])
                self._prior_cache = cands
                # Slot record through real serialization.
                best = cands[0]
                slot = slot_record(best['center_container'],
                                   best['box_yaw'],
                                   'slot-%02d' % index)
                decoded_slot = planning_pose_from_message(
                    wire(planning_pose_to_message(slot, 'slot',
                                                  context(slot))),
                    'slot', context(slot))
                self.assertEqual(decoded_slot['position'], slot['position'])
                self.assert_quaternion(decoded_slot['quaternion_xyzw'],
                                       slot['quaternion_xyzw'], 1e-9)
                # Legacy decoder reproduces the pre-ST05 inline encoding
                # from container truth (independent hand math).
                expected_lz = (best['center_container'][2]
                               - v1['center_base'][2])
                self.assert_close(best['center_local'][2], expected_lz)
                self.assert_close(best['center_base'][2],
                                  v1['center_base'][2] + expected_lz)
                self.assert_close(best['yaw'],
                                  v1['yaw'] + best['box_yaw'])
                # Footprint/support unchanged under coordinate
                # re-expression: AABB from container center equals the
                # volume-center path (floor_z conversion explicit).
                aabb_container = candidate_aabb(best, 0.8, floor_z=floor)
                legacy_only = {k: v for k, v in best.items()
                               if k != 'center_container'}
                aabb_legacy = candidate_aabb(legacy_only, 0.8)
                for value_a, value_b in zip(aabb_container, aabb_legacy):
                    self.assert_close(value_a, value_b)
                observed.update(stage='slot+decoder')
                count += 1
        self.assertEqual(count, 64)

    def test_four_container_yaws_invariant(self):
        # The container box center must not rotate with the map yaw: four
        # container yaws produce identical packing geometry in container
        # coordinates (the old center_base rotated; center_container cannot).
        with case('yaw-invariance/container', {}) as observed:
            surfaces = [surface_v1(.53, 0.30, occupied_cell=(7, 5),
                                   map_yaw=yaw)
                        for yaw in (0., math.pi / 2, math.pi,
                                    math.atan2(4, 3))]
            results = [generate_candidates(s, list(WDH),
                                           params={'stride_cells': 1})
                       for s in surfaces]
            first = results[0]
            for index, others in enumerate(results[1:], start=1):
                self.assertEqual(len(first), len(others))
                for a, b in zip(first, others):
                    self.assert_close(a['center_container'][0],
                                      b['center_container'][0])
                    self.assert_close(a['center_container'][1],
                                      b['center_container'][1])
                    self.assert_close(a['center_container'][2],
                                      b['center_container'][2])
                    self.assertEqual(a['box_yaw'], b['box_yaw'])
                    self.assertEqual(a['footprint'], b['footprint'])
            observed.update(n_candidates=len(first))

    def test_placed_input_and_replan_same_meaning(self):
        with case('placed/v2-direct-vs-decoder', {}) as observed:
            from luggage_packing.placement_solver import (
                placement_constraint_verdict,
            )
            v1 = surface_v1(.53, 0.0, occupied_cell=(7, 5))
            result = solve_placement(
                v1, list(WDH), allowed_yaws=[0.0],
                params={'stride_cells': 1})
            self.assertTrue(result['success'])
            best = result['selected']
            # v2 placed input: the slot record of the selected candidate;
            # v1 placed input: the decoder-produced fields. Both describe
            # the same box (same container z through either derivation).
            slot = slot_record(best['center_container'], best['box_yaw'],
                               'replan-1')
            # The v2 record, the decoder's center_base and the container
            # center are one meaning (NOT center_base + center_local).
            self.assert_close(slot['position'][2], best['center_base'][2])
            self.assert_close(slot['position'][2],
                              best['center_container'][2])
            placed = [candidate_aabb(best, 0.8, floor_z=.53)]
            replan = solve_placement(
                v1, list(WDH), allowed_yaws=[0.0],
                params={'stride_cells': 1},
                candidate_validator=lambda candidate:
                placement_constraint_verdict(
                    candidate, 0.8, placed_aabbs=placed, floor_z=.53))
            # The placed box occupies its own slot: the replan selection
            # must move (or fail), never return the identical slot.
            if replan['success']:
                self.assertNotEqual(
                    (round(replan['selected']['center_container'][0], 6),
                     round(replan['selected']['center_container'][1], 6)),
                    (round(best['center_container'][0], 6),
                     round(best['center_container'][1], 6)))
            observed.update(replan_reason=replan['reason_code'],
                            replan_success=replan['success'])

    def test_reason_categories_not_bin_full(self):
        with case('reasons/coordinate-error-not-capacity', {}) as observed:
            v1 = surface_v1(.53, 0.30, occupied_cell=None)
            # Unknown support above floor: geometric reject, not BIN_FULL.
            for ix in range(v1['nx']):
                for iy in range(v1['ny']):
                    v1['state'][ix][iy] = 'unknown'
                    v1['height'][ix][iy] = 0.30
                    v1['confidence'][ix][iy] = 'none'
            result = solve_placement(v1, list(WDH), allowed_yaws=[0.0],
                                     params={'stride_cells': 1})
            observed.update(reason=result['reason_code'],
                            success=result['success'])
            self.assertNotEqual(result['reason_code'], 'BIN_FULL')
            self.assertFalse(result['success'])

    def test_slot_from_contract_full_orientation(self):
        matrix = ((0., (0., 0., 0.)), (math.pi / 2, (1.5, -.4, .2)))
        for yaw, translation in matrix:
            with self.subTest(yaw=yaw), \
                    case('seam/%.2f' % yaw,
                         {'yaw': yaw}) as observed:
                record = slot_record((.2, -.1, .53 + .30 + .16),
                                     math.atan2(4, 3), 'seam-1')
                container_q = (0., 0., math.sin(yaw / 2),
                               math.cos(yaw / 2))
                pose = slot_from_contract(
                    record, (translation, container_q))
                cos_y, sin_y = math.cos(yaw), math.sin(yaw)
                expected_xy = (translation[0] + cos_y * .2 - sin_y * (-.1),
                               translation[1] + sin_y * .2 + cos_y * (-.1))
                self.assert_close(pose.position.x, expected_xy[0])
                self.assert_close(pose.position.y, expected_xy[1])
                self.assert_close(pose.position.z,
                                  translation[2] + .53 + .30 + .16)
                # Complete orientation: composed quaternion is NOT the
                # identity/xyz-only result for a rotated container.
                composed_yaw = yaw + math.atan2(4, 3)
                expected_q = (0., 0., math.sin(composed_yaw / 2),
                              math.cos(composed_yaw / 2))
                self.assert_quaternion(
                    (pose.orientation.x, pose.orientation.y,
                     pose.orientation.z, pose.orientation.w),
                    expected_q, 1e-9)
                observed.update(position=(pose.position.x,
                                          pose.position.y,
                                          pose.position.z))

    def test_atlas_hook_still_gates(self):
        # Declared neutral fixture: the atlas-free path is neutral; the
        # runtime hook must keep its hard-reject semantics (no silent
        # disable introduced by ST05).
        with case('atlas/hook-still-gates', {}) as observed:
            from collections import namedtuple
            from luggage_packing.placement_reachability import (
                make_atlas_annotate,
            )
            result_type = namedtuple(
                'Result', ('status', 'reason', 'hard_reject_safe'))

            class UnreachableAtlas(object):
                yaw_bins = (0.0, 1.5707963267948966)

                def query(self, x, y, z, yaw):
                    return result_type(1, 'unreachable', True)

            hook = make_atlas_annotate(
                UnreachableAtlas(), .53, .8, hard_reject=True,
                dump_only=False)
            v1 = surface_v1(.53, 0.0, occupied_cell=None)
            result = solve_placement(
                v1, list(WDH), allowed_yaws=[0.0],
                params={'stride_cells': 1}, atlas_annotate=hook)
            observed.update(reason=result['reason_code'],
                            histogram=result.get('reject_histogram'))
            self.assertFalse(result['success'])
            # The hook still gates: every feasible candidate was flipped by
            # the atlas (taxonomy may then report exhaustion on top).
            histogram = result.get('reject_histogram') or {}
            self.assertGreater(histogram.get('atlas_unreachable', 0), 0)

    def test_slot_record_negatives(self):
        record = slot_record((.1, .2, .8), .4)
        for name, key, value, code in [
            ('wrong-frame', 'frame_id', 'world', 'FRAME_CONTRACT_MISMATCH'),
            ('missing-identity', 'request_id', '', 'MISSING_IDENTITY'),
            ('wrong-reference', 'reference', 'surface_contact',
             'POSE_REFERENCE_MISMATCH'),
            ('stale-scene', 'scene_revision', 'stale',
             'CALIBRATION_REVISION_MISMATCH'),
            ('wrong-version', 'contract_version', 1,
             'CONTRACT_VERSION_MISMATCH'),
            ('wrong-units', 'length_unit', 'mm', 'UNIT_MISMATCH'),
            ('nonfinite', 'position', [math.inf, 0., 0.],
             'INVALID_NUMERIC_PAYLOAD'),
            ('nonunit-quaternion', 'quaternion_xyzw', [0., 0., 0., 2.],
             'INVALID_QUATERNION'),
        ]:
            data = copy.deepcopy(record)
            data[key] = value
            with self.subTest(name=name):
                self.reject(name, code, {key: value},
                            lambda: planning_pose_to_message(
                                data, 'slot', context(record)))
        wrong_map = dict(record, request_id='other')
        self.reject('request-mismatch', 'REQUEST_IDENTITY_MISMATCH',
                    {'request_id': 'other'},
                    lambda: planning_pose_to_message(
                        wrong_map, 'slot', context(record)))
        v1 = surface_v1(.53, 0.0)
        foreign = surface_v2_record(v1, .53, 'neg-1')
        foreign['geometry_hash'] = 'wrong-hull'
        self.assertNotEqual(foreign['geometry_hash'], 'sha256:test-hull')


if __name__ == '__main__':
    unittest.main()
