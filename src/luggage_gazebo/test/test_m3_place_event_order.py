#!/usr/bin/env python3
"""Place-phase M3 notes stay in EVENT_ORDER, and views return to the portal."""

import os
import sys
import tempfile
import unittest

_SCRIPTS = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from cargo_map_integrate_eval_driver import (  # noqa: E402
    CargoMapIntegrateDriver,
)
from luggage_gazebo.integration_acceptance import (  # noqa: E402
    EVENT_ORDER,
    FROZEN_HASHES,
    extract_acceptance,
)
from luggage_gazebo.place_metrics import PlaceTrial  # noqa: E402
from m3_three_box_driver import (  # noqa: E402
    GtReleaseStream,
    M3ThreeBoxDriver,
    classify_gt_gap,
)


class _Notes(object):
    def __init__(self):
        self._m3_notes = []
        self._m3_request_id = "m3-box-01"
        self._place_state = "INIT"
        self._vacuum_state = {"attached": True, "fail_reason": ""}
        self._t = 0.0

    def ros_now_sec(self):
        self._t += 1.0
        return self._t

    _m3_note = M3ThreeBoxDriver._m3_note
    _m3_note_place_attach = M3ThreeBoxDriver._m3_note_place_attach
    _m3_note_route_boundaries = M3ThreeBoxDriver._m3_note_route_boundaries
    _m3_note_return_if_place_finished = (
        M3ThreeBoxDriver._m3_note_return_if_place_finished)


class TestPlacePhaseNotes(unittest.TestCase):
    def test_pick_start_home_is_not_return_complete(self):
        driver = _Notes()
        driver._place_state = "INIT"
        driver._m3_note_return_if_place_finished(True)
        self.assertEqual(driver._m3_notes, [])

    def test_successful_place_home_notes_return_complete_once(self):
        driver = _Notes()
        driver._place_state = "HOME"
        driver._m3_note_return_if_place_finished(True)
        driver._t = 100.0
        driver._m3_note_return_if_place_finished(True)
        stages = [note["stage"] for note in driver._m3_notes]
        self.assertEqual(stages, ["return_complete"])
        self.assertEqual(driver._m3_notes[0]["t_ros"], 1.0)
        self.assertEqual(driver._m3_notes[0]["request_id"], "m3-box-01")
        self.assertEqual(driver._m3_notes[0]["source"], "m3_note")

    def test_failed_home_does_not_note_return_complete(self):
        driver = _Notes()
        driver._place_state = "HOME"
        driver._m3_note_return_if_place_finished(False)
        self.assertEqual(driver._m3_notes, [])

    def test_unattached_vacuum_does_not_count_as_place_attach(self):
        driver = _Notes()
        driver._vacuum_state = {"attached": False, "fail_reason": ""}
        driver._m3_note_place_attach()
        self.assertEqual(driver._m3_notes, [])

    def test_route_boundaries_precede_execution_and_attach(self):
        driver = _Notes()
        driver._m3_note("observation")
        driver._m3_note_route_boundaries()
        driver._m3_note("execution")
        driver._m3_note_place_attach()
        driver._m3_note("release")
        stages = [note["stage"] for note in driver._m3_notes]
        self.assertEqual(stages, [
            "observation", "slot", "selected_path", "probe",
            "execution", "attach", "release",
        ])
        stamps = [note["t_ros"] for note in driver._m3_notes]
        self.assertEqual(stamps, sorted(stamps))
        self.assertLess(stamps[stages.index("observation")],
                        stamps[stages.index("return_complete")]
                        if "return_complete" in stages else stamps[-1] + 1)
        ids = {note["request_id"] for note in driver._m3_notes}
        self.assertEqual(ids, {"m3-box-01"})


class _Views(object):
    def __init__(self, dump_dir):
        self.calls = []
        self._args = type("Args", (), {"dump_dir": dump_dir})()
        self._portal_exit_proof = None
        self._last_joint_state = {"position": [0.2] * 6}
        self._exit_yaw = {}
        self._stats = None

    def _exit_to_portal(self):
        self.calls.append("portal")
        self._last_joint_state = {"position": [0.4] * 6}
        self._portal_exit_proof = {"proof_ns": 88000 + len(self.calls)}
        return True, "portal goto ok"

    def ros_now_sec(self):
        return 40.0

    def _lookup_xyz_quat(self, _parent, _child):
        self.calls.append("tf")
        return [0.0, 0.0, 1.0], [0.0, 0.0, 0.0, 1.0]

    def _placed_box_container(self, _slot, _pick):
        return [0.0, 0.0, 0.5], [0.5, 0.4, 0.25], 0.0

    def call_srv(self, *_args, **_kwargs):
        return type("Stats", (), {"map_revision": 1})()

    def _execute_view_candidates(self, *_args, **_kwargs):
        self.calls.append("aim")
        return [], {"index": 0, "suction_xyz": [-0.7, 0.6, 1.8]}

    def _integrate_fresh_views(self, *_args, **_kwargs):
        self.calls.append("integrate")
        return {
            "success": True,
            "footprint_hits": 4,
            "expected_map_revision": 1,
            "resulting_map_revision": 2,
        }

    def _accept_view(self, *_args, **_kwargs):
        self.calls.append("accept")

    _run_post_place_views = CargoMapIntegrateDriver._run_post_place_views
    _restore_portal_after_views = (
        CargoMapIntegrateDriver._restore_portal_after_views)


class TestPortalRemintAfterViews(unittest.TestCase):
    def test_views_return_to_the_portal_before_home(self):
        trial = PlaceTrial(index=0)
        with tempfile.TemporaryDirectory() as dump:
            driver = _Views(dump)
            pick = type("Pick", (), {"height": 0.25})()
            slot_meta = {"pose_world": {"position": [-1.5, 0.58, 0.65]}}
            driver._run_post_place_views(trial, None, slot_meta, pick)
        self.assertEqual(
            [name for name in driver.calls if name in ("portal", "aim", "accept")],
            ["portal", "aim", "accept", "portal"])
        record = trial.extras["cargo_view_portal_return"]
        self.assertTrue(record["ok"])
        self.assertEqual(record["t_ros"], 40.0)
        self.assertEqual(len(record["final_joint_values"]), 6)
        self.assertGreater(record["proof_ns"], 0)

    def test_failed_reexit_is_named_and_clears_the_proof_stamp(self):
        trial = PlaceTrial(index=0)

        class _Fail(_Views):
            def _exit_to_portal(self):
                self.calls.append("portal")
                self._portal_exit_proof = None
                return False, "goto failed"

        with tempfile.TemporaryDirectory() as dump:
            driver = _Fail(dump)
            driver._restore_portal_after_views(trial)
        self.assertEqual(trial.fail_code, "PORTAL_EXIT_FAILED")
        self.assertEqual(trial.extras["cargo_view_portal_return"]["proof_ns"], 0)
        self.assertFalse(trial.extras["cargo_view_portal_return"]["ok"])


def _hash():
    return "a" * 64


def _passing_case():
    hashes = {key: _hash() for key in FROZEN_HASHES}
    hashes["scene"] = "b" * 64
    hashes["hull"] = "c" * 64
    events = []
    for index, stage in enumerate(EVENT_ORDER):
        row = {
            "stage": stage,
            "request_id": "m3-box-01",
            "success": True,
            "scene_revision": hashes["scene"],
            "geometry_hash": hashes["hull"],
            "input_map_revision": 1,
            "acquisition_stamp_ns": 40_000_000_000 + index * 1_000_000_000,
            "receiver_stamp_ns": 1_000 + index,
            "event_seq": index + 1,
            "stamp_source": "measured_note",
        }
        if stage in ("measured_integration", "map_update"):
            row["output_map_revision"] = 2
        events.append(row)
    evidence = {
        "complete": True,
        "hashes": dict(hashes),
        "cases": [{
            "box_id": "carryon@01",
            "request_id": "m3-box-01",
            "events": events,
            "measured": {
                "integration_success": True,
                "return_success": True,
                "payload_retained": True,
                "input_map_revision": 1,
                "output_map_revision": 2,
                "sensor_occupied_cells": 66,
                "floor_through_candidates": 0,
            },
            "gt_reference": {
                "xy_error_m": 0.0042,
                "z_error_m": -0.0008,
                "drift_m": 0.0,
                "roll_deg": -0.14,
                "pitch_deg": 0.27,
                "center_inside_hull": True,
                "footprint_reference_cells": 80,
                "footprint_sensor_cells": 51,
            },
        }],
        "handoffs": [],
    }
    frozen = {
        "hashes": hashes,
        "box_count": 1,
        "sequence_ids": ["carryon"],
        "event_order": list(EVENT_ORDER),
        "release_sample_interval_s": 0.15,
        "drift_sample_interval_s": 1.0,
        "replay_command": ["python3", "replay"],
    }
    return evidence, frozen


class _Handoff(object):
    def __init__(self, sequence):
        self._m3_sequence = list(sequence)
        self._m3_handoffs = []

    def _m3_box_id(self, ordinal, catalog):
        name = catalog or (
            self._m3_sequence[ordinal - 1]
            if ordinal <= len(self._m3_sequence) else "box")
        return "%s@%02d" % (name, ordinal)

    _m3_record_handoff = M3ThreeBoxDriver._m3_record_handoff


class TestHandoffCount(unittest.TestCase):
    def test_one_box_session_records_no_handoff(self):
        driver = _Handoff(["carryon"])
        driver._m3_record_handoff(1, 2)
        self.assertEqual(driver._m3_handoffs, [])

    def test_three_box_session_records_two_handoffs(self):
        driver = _Handoff(["carryon", "standard", "carryon"])
        driver._m3_record_handoff(1, 2)
        driver._m3_record_handoff(2, 3)
        driver._m3_record_handoff(3, 4)
        pairs = [(row["from"], row["to"]) for row in driver._m3_handoffs]
        self.assertEqual(pairs, [(1, 2), (2, 3)])


class TestSingleBoxAcceptanceFixture(unittest.TestCase):
    def test_ordered_place_phase_events_pass(self):
        evidence, frozen = _passing_case()
        verdict = extract_acceptance(evidence, frozen)
        self.assertEqual(verdict["status"], "pass", verdict)

    def test_return_complete_before_observation_fails_causal_order(self):
        evidence, frozen = _passing_case()
        events = evidence["cases"][0]["events"]
        by_stage = {row["stage"]: row for row in events}
        by_stage["return_complete"]["acquisition_stamp_ns"] = (
            by_stage["observation"]["acquisition_stamp_ns"] - 1)
        verdict = extract_acceptance(evidence, frozen)
        self.assertIn("case[0]:EVENT_CAUSAL_ORDER", verdict["failures"])
        self.assertNotEqual(verdict["status"], "pass")


def _bare_stream(model="pickup_box_0001_carryon"):
    stream = GtReleaseStream.__new__(GtReleaseStream)
    stream.model = model
    stream.path = "/tmp/unused-release-gt.jsonl"
    stream._samples = __import__("collections").deque()
    stream._lock = __import__("threading").Lock()
    stream._pose_blocks = 0
    stream._stdout_lines = 0
    stream._parse_failures = 0
    stream._unparsed_text = ""
    stream._seen_names = []
    stream._stderr_tail = ""
    stream._proc = None
    return stream


class TestGtStreamEmptyRecord(unittest.TestCase):
    def test_other_model_names_are_model_not_found(self):
        stream = _bare_stream()
        stream._stderr_tail = "topic not advertised"
        stream._accept(
            'pose {\n  name: "container"\n  position { x: 0 y: 0 z: 0 }\n'
            '  orientation { x: 0 y: 0 z: 0 w: 1 }\n}',
            None)
        record = stream.failure_record()
        self.assertEqual(record["gap"], "model_not_found")
        self.assertEqual(record["pose_blocks"], 1)
        self.assertEqual(record["seen_names"], ["container"])
        self.assertEqual(record["stderr_tail"], "topic not advertised")
        self.assertEqual(record["sample_count"], 0)

    def test_named_target_without_position_is_parse_failed(self):
        stream = _bare_stream()
        stream._accept('pose {\n  name: "pickup_box_0001_carryon"\n}', None)
        record = stream.failure_record()
        self.assertEqual(record["gap"], "parse_failed")
        self.assertEqual(record["parse_failures"], 1)
        self.assertEqual(record["sample_count"], 0)

    def test_silent_live_process_is_no_messages(self):
        self.assertEqual(classify_gt_gap({
            "sample_count": 0, "stdout_lines": 0, "pose_blocks": 0,
            "seen_names": [], "model": "pickup_box_0001_carryon",
            "parse_failures": 0, "ign_returncode": None,
        }), "no_messages")

    def test_dead_process_with_no_stdout_is_process_exited(self):
        self.assertEqual(classify_gt_gap({
            "sample_count": 0, "stdout_lines": 0, "pose_blocks": 0,
            "seen_names": [], "model": "pickup_box_0001_carryon",
            "parse_failures": 0, "ign_returncode": 1,
        }), "process_exited")

    def test_old_sample_does_not_pass_the_release_gate(self):
        stream = _bare_stream()
        stream._samples.append({
            "recv_ns": __import__("time").monotonic_ns() - 2_000_000_000,
            "xyz": [0.0, 0.0, 0.0],
        })
        ok, record = stream.release_gate(fresh_s=0.5)
        self.assertFalse(ok)
        self.assertEqual(record["gap"], "stale_sample")
        self.assertGreater(record["latest_age_s"], 0.5)

    def test_omitted_zero_axis_still_parses(self):
        stream = _bare_stream()
        stream._file = __import__("io").StringIO()
        stream._accept(
            'pose {\nname: "pickup_box_0001_carryon"\nid: 49\n'
            'position {\nx: 1\nz: 0.785\n}\norientation {\nw: 1\n}\n}',
            None)
        record = stream.failure_record()
        self.assertEqual(record["parse_failures"], 0)
        self.assertEqual(record["sample_count"], 1)
        self.assertEqual(stream._samples[-1]["xyz"], [1.0, 0.0, 0.785])
        self.assertEqual(stream._samples[-1]["quat_xyzw"][3], 1.0)

    def test_fresh_sample_passes_the_release_gate(self):
        stream = _bare_stream()
        stream._samples.append({
            "recv_ns": __import__("time").monotonic_ns(),
            "xyz": [0.0, 0.0, 0.0],
        })
        ok, record = stream.release_gate(fresh_s=0.5)
        self.assertTrue(ok)
        self.assertEqual(record["gap"], "")


if __name__ == "__main__":
    unittest.main()
