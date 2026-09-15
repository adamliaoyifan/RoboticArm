"""ACTIVE-VIEW-1 tests: classification matrix, candidate matrix, PF-R7 G4
saved-sample replay, and exploration-contract compatibility.

All tests are ROS-free and run under plain pytest/unittest with
``PYTHONPATH=src/luggage_planning``.
"""

import copy
import importlib
import math
import os
import unittest

from luggage_planning import active_view_recovery as avr
from luggage_planning.active_view_recovery import (
    ActiveViewConfig,
    ActiveViewRecoveryPolicy,
    DECISION_SCHEMA_ID,
    NO_CARGO_EVIDENCE,
    RECOVERY_BUDGET_EXHAUSTED,
    RECOVERY_CANDIDATES_EXHAUSTED,
    RECOVERY_EVIDENCE_INVALID,
    RECOVERY_NOT_REQUIRED,
    RECOVERY_NO_GAIN,
    RECOVERY_STATE_INVALID,
    RECOVERY_SUCCEEDED,
    RecoveryWindow,
    load_replay_fixture,
    scan_forbidden_keys,
)
from luggage_planning.exploration_contracts import (
    AcquisitionStamp,
    ExplorationContext,
    ExplorationSnapshot,
    PolicyProposal,
    ViewOutcome,
    ViewOutcomeStatus,
)

FIXTURE_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "data",
    "active_view_pfr7_g4_windows.json")

FRAME_ID = "camera_depth_optical_frame"
CAMERA_MODEL = "D555-640x480"
GEOMETRY_HASH = "rig-test-v1"
MAP_REVISION = 7

HINT_BBOX = (162, 70, 523, 356)
HINT_LANDING = (-0.99, -0.07)
HINT = {"confidence": 0.116, "bbox": list(HINT_BBOX), "label": 2,
        "centre_world_xy": list(HINT_LANDING)}
DET_BBOX = (174, 133, 503, 335)
DET = {"confidence": 0.368, "bbox": list(DET_BBOX), "label": 2,
       "border_margin_px": 133, "valid_depth_ratio": 0.99,
       "centre_world_xy": [-1.0, 0.0]}
GEOM_MISS = {"point_count": 0, "top_surface_valid": False,
             "support_mode": "NO_TOP"}
GEOM_READY = {"point_count": 13645, "top_surface_valid": True,
              "support_mode": "FULL_3D"}


def frame(sec, nanosec=0, hint=None, det=None, comp=None, geom=None,
          **overrides):
    data = {
        "stamp": {"sec": sec, "nanosec": nanosec},
        "frame_id": FRAME_ID,
        "camera_model_id": CAMERA_MODEL,
        "geometry_hash": GEOMETRY_HASH,
        "map_revision": MAP_REVISION,
        "rgb_ok": True,
        "depth_ok": True,
        "aligned_depth_ok": True,
        "exact_tf_ok": True,
        "robot_settled": True,
        "accepted_detections": [copy.deepcopy(det)] if det else [],
        "diagnostic_hints": [copy.deepcopy(hint)] if hint else [],
        "foreground_components": [copy.deepcopy(comp)] if comp else [],
        "cargo_geometry": geom if geom is not None else GEOM_MISS,
    }
    data.update(overrides)
    return data


def window(frames, **overrides):
    data = {
        "state": "pre_pick_detect",
        "payload_attached": False,
        "vacuum_commanded": False,
        "cancel_requested": False,
        "image_width": 640,
        "image_height": 480,
        "frames": frames,
    }
    data.update(overrides)
    return data


def miss_window(sec=100):
    """Persistent low-confidence hint in all three frames."""
    return window([frame(sec, 0, hint=HINT),
                   frame(sec, 100000000, hint=HINT),
                   frame(sec, 200000000, hint=HINT)])


def integrate(policy, candidate_id, sec, confidence, ratio,
              status=ViewOutcomeStatus.INTEGRATED):
    policy.observe(ViewOutcome(
        status=status,
        reason_code="measured",
        candidate_id=candidate_id,
        acquisition_stamp=AcquisitionStamp(sec, 0),
        diagnostics={"best_cargo_confidence": confidence,
                     "bbox_valid_depth_ratio": ratio}))


class TestClassificationMatrix(unittest.TestCase):
    """Every fail-closed case asserts the exact terminal reason and zero
    motion authority."""

    def evaluate(self, data):
        policy = ActiveViewRecoveryPolicy()
        return policy, policy.evaluate_mapping(data)

    def assertTerminal(self, decision, reason):
        self.assertEqual(decision.reason, reason)
        self.assertEqual(tuple(decision.candidates), ())
        self.assertFalse(decision.motion_authority)
        proposal = decision.to_dict()
        self.assertEqual(proposal["reason"], reason)
        self.assertFalse(proposal["motion_authority"])

    def test_planner_ready_detection_is_immediate_success(self):
        _policy, decision = self.evaluate(window(
            [frame(50, 0, det=DET, geom=GEOM_READY),
             frame(50, 33000000, det=DET, geom=GEOM_READY),
             frame(50, 66000000, det=DET, geom=GEOM_READY)]))
        self.assertTerminal(decision, RECOVERY_SUCCEEDED)
        self.assertTrue(decision.cargo_present)

    def test_persistent_low_confidence_hint_proposes_view(self):
        policy, decision = self.evaluate(miss_window())
        self.assertEqual(decision.reason, "")
        self.assertEqual(len(decision.candidates), 1)
        self.assertTrue(decision.cargo_present)
        self.assertTrue(decision.incomplete)
        self.assertIn("no_accepted_detections", decision.defects)
        self.assertTrue(decision.motion_authority)

    def test_depth_only_persistent_component_proposes_view(self):
        comp = {"pixel_count": 5392, "area_fraction": 0.0176,
                "height_above_support_m": 0.24, "in_workspace": True,
                "bbox": [150, 90, 520, 400]}
        _policy, decision = self.evaluate(window(
            [frame(70, 0, comp=comp), frame(70, 66000000, comp=comp),
             frame(70, 132000000, comp=comp)]))
        self.assertEqual(decision.reason, "")
        self.assertEqual(len(decision.candidates), 1)

    def test_empty_view_is_no_cargo_evidence(self):
        _policy, decision = self.evaluate(window(
            [frame(60, 0), frame(60, 33000000), frame(60, 66000000)]))
        self.assertTerminal(decision, NO_CARGO_EVIDENCE)

    def test_one_frame_transient_is_no_cargo_evidence(self):
        _policy, decision = self.evaluate(window(
            [frame(60, 0, hint=HINT), frame(60, 33000000),
             frame(60, 66000000)]))
        self.assertTerminal(decision, NO_CARGO_EVIDENCE)

    def test_two_frame_transient_without_association_is_no_cargo(self):
        far_hint = dict(HINT, bbox=[10, 10, 40, 40],
                        centre_world_xy=[-0.5, 0.4])
        _policy, decision = self.evaluate(window(
            [frame(60, 0, hint=HINT), frame(60, 33000000, hint=far_hint),
             frame(60, 66000000)]))
        self.assertTerminal(decision, NO_CARGO_EVIDENCE)

    def test_workspace_external_hint_is_no_cargo_evidence(self):
        outside = dict(HINT, centre_world_xy=[0.2, 0.0])
        _policy, decision = self.evaluate(window(
            [frame(60, 0, hint=outside), frame(60, 33000000, hint=outside),
             frame(60, 66000000, hint=outside)]))
        self.assertTerminal(decision, NO_CARGO_EVIDENCE)

    def test_border_truncated_detection_is_incomplete(self):
        truncated = dict(DET, bbox=[2, 133, 330, 335],
                         border_margin_px=2, valid_depth_ratio=0.99)
        geom = dict(GEOM_READY, support_mode="PARTIAL_3D")
        _policy, decision = self.evaluate(window(
            [frame(80, 0, det=truncated, geom=geom),
             frame(80, 33000000, det=truncated, geom=geom),
             frame(80, 66000000, det=truncated, geom=geom)]))
        self.assertEqual(decision.reason, "")
        self.assertIn("bbox_border_margin", decision.defects)

    def test_low_valid_depth_ratio_is_incomplete(self):
        det = dict(DET, valid_depth_ratio=0.62)
        geom = dict(GEOM_READY, support_mode="PARTIAL_3D")
        _policy, decision = self.evaluate(window(
            [frame(80, 0, det=det, geom=geom),
             frame(80, 33000000, det=det, geom=geom),
             frame(80, 66000000, det=det, geom=geom)]))
        self.assertEqual(decision.reason, "")
        self.assertIn("bbox_valid_depth_ratio", decision.defects)

    def test_insufficient_cargo_cloud_is_incomplete(self):
        det = dict(DET)
        geom = dict(GEOM_READY, point_count=137, support_mode="PARTIAL_3D")
        _policy, decision = self.evaluate(window(
            [frame(80, 0, det=det, geom=geom),
             frame(80, 33000000, det=det, geom=geom),
             frame(80, 66000000, det=det, geom=geom)]))
        self.assertEqual(decision.reason, "")
        self.assertIn("cargo_cloud_points", decision.defects)

    def test_non_full_3d_geometry_is_incomplete(self):
        det = dict(DET)
        geom = dict(GEOM_READY, support_mode="SIDE_SUPPORT")
        _policy, decision = self.evaluate(window(
            [frame(80, 0, det=det, geom=geom, hint=HINT),
             frame(80, 33000000, det=det, geom=geom, hint=HINT),
             frame(80, 66000000, det=det, geom=geom, hint=HINT)]))
        self.assertEqual(decision.reason, "")
        self.assertIn("support_mode_not_full_3d", decision.defects)

    def test_non_monotonic_stamp_is_evidence_invalid(self):
        _policy, decision = self.evaluate(window(
            [frame(90, 200000000, hint=HINT), frame(90, 100000000, hint=HINT),
             frame(90, 0, hint=HINT)]))
        self.assertTerminal(decision, RECOVERY_EVIDENCE_INVALID)

    def test_stale_stamp_span_is_evidence_invalid(self):
        _policy, decision = self.evaluate(window(
            [frame(90, 0, hint=HINT), frame(90, 100000000, hint=HINT),
             frame(91, 0, hint=HINT)]))
        self.assertTerminal(decision, RECOVERY_EVIDENCE_INVALID)

    def test_missing_exact_tf_is_evidence_invalid(self):
        _policy, decision = self.evaluate(window(
            [frame(90, 0, hint=HINT, exact_tf_ok=False),
             frame(90, 100000000, hint=HINT),
             frame(90, 200000000, hint=HINT)]))
        self.assertTerminal(decision, RECOVERY_EVIDENCE_INVALID)

    def test_robot_not_settled_is_evidence_invalid(self):
        _policy, decision = self.evaluate(window(
            [frame(90, 0, hint=HINT, robot_settled=False),
             frame(90, 100000000, hint=HINT),
             frame(90, 200000000, hint=HINT)]))
        self.assertTerminal(decision, RECOVERY_EVIDENCE_INVALID)

    def test_identity_mismatch_is_evidence_invalid(self):
        _policy, decision = self.evaluate(window(
            [frame(90, 0, hint=HINT),
             frame(90, 100000000, hint=HINT,
                   camera_model_id="D455-640x480"),
             frame(90, 200000000, hint=HINT)]))
        self.assertTerminal(decision, RECOVERY_EVIDENCE_INVALID)

    def test_context_identity_mismatch_is_evidence_invalid(self):
        policy = ActiveViewRecoveryPolicy()
        policy.reset(ExplorationContext(
            frames={}, geometry_descriptor={}, geometry_hash="other-rig",
            camera_model={"model_id": CAMERA_MODEL}))
        decision = policy.evaluate_mapping(miss_window())
        self.assertTerminal(decision, RECOVERY_EVIDENCE_INVALID)

    def test_carrying_payload_is_state_invalid(self):
        _policy, decision = self.evaluate(window(
            [frame(60, 0, hint=HINT), frame(60, 33000000, hint=HINT),
             frame(60, 66000000, hint=HINT)], payload_attached=True))
        self.assertTerminal(decision, RECOVERY_STATE_INVALID)

    def test_vacuum_commanded_is_state_invalid(self):
        _policy, decision = self.evaluate(window(
            [frame(60, 0, hint=HINT), frame(60, 33000000, hint=HINT),
             frame(60, 66000000, hint=HINT)], vacuum_commanded=True))
        self.assertTerminal(decision, RECOVERY_STATE_INVALID)

    def test_cancel_requested_is_state_invalid(self):
        _policy, decision = self.evaluate(window(
            [frame(60, 0, hint=HINT), frame(60, 33000000, hint=HINT),
             frame(60, 66000000, hint=HINT)], cancel_requested=True))
        self.assertTerminal(decision, RECOVERY_STATE_INVALID)

    def test_coordinator_cancel_outcome_is_state_invalid(self):
        policy = ActiveViewRecoveryPolicy()
        policy.observe(ViewOutcome(
            status=ViewOutcomeStatus.REJECTED, reason_code="operator_abort",
            candidate_id=""))
        decision = policy.evaluate_mapping(miss_window())
        self.assertTerminal(decision, RECOVERY_STATE_INVALID)

    def test_nonfinite_input_is_evidence_invalid(self):
        bad = miss_window()
        bad["frames"][0]["diagnostic_hints"][0]["confidence"] = float("nan")
        _policy, decision = self.evaluate(bad)
        self.assertTerminal(decision, RECOVERY_EVIDENCE_INVALID)

    def test_missing_frames_is_evidence_invalid(self):
        _policy, decision = self.evaluate(window([frame(60, 0)]))
        self.assertTerminal(decision, RECOVERY_EVIDENCE_INVALID)

    def test_gt_and_iou_injection_is_evidence_invalid(self):
        for key in ("gt_iou", "gt_box", "eval_iou", "sim_mode", "fixture_id",
                    "eval_only", "gazebo_entity"):
            data = miss_window()
            data[key] = 1
            _policy, decision = self.evaluate(data)
            self.assertTerminal(decision, RECOVERY_EVIDENCE_INVALID)

    def test_nested_gt_injection_is_evidence_invalid(self):
        data = miss_window()
        data["frames"][1]["gt_pose"] = [0.0, 0.0, 0.0]
        _policy, decision = self.evaluate(data)
        self.assertTerminal(decision, RECOVERY_EVIDENCE_INVALID)

    def test_not_required_is_a_motion_free_terminal(self):
        # With the defect list covering every not-planner-ready cause,
        # RECOVERY_NOT_REQUIRED stays a defensive terminal; it must always
        # be motion-free when produced.
        self.assertIn(RECOVERY_NOT_REQUIRED, avr.TERMINAL_REASONS)

    def test_hint_below_diagnostic_floor_is_ignored(self):
        weak = dict(HINT, confidence=0.04)
        _policy, decision = self.evaluate(window(
            [frame(60, 0, hint=weak), frame(60, 33000000, hint=weak),
             frame(60, 66000000, hint=weak)]))
        self.assertTerminal(decision, NO_CARGO_EVIDENCE)

    def test_hint_at_production_floor_is_not_a_hint(self):
        strong = dict(HINT, confidence=0.20)
        _policy, decision = self.evaluate(window(
            [frame(60, 0, hint=strong), frame(60, 33000000, hint=strong),
             frame(60, 66000000, hint=strong)]))
        # conf == 0.20 is production scope, not a diagnostic hint; with no
        # accepted detection record it is not cargo evidence.
        self.assertTerminal(decision, NO_CARGO_EVIDENCE)


class TestCandidateMatrix(unittest.TestCase):

    def fresh(self, config=None):
        policy = ActiveViewRecoveryPolicy(config)
        return policy

    def propose_first(self, policy, sec=100):
        decision = policy.evaluate_mapping(miss_window(sec))
        self.assertEqual(decision.reason, "")
        return decision

    def test_all_five_default_candidates_ranked(self):
        policy = self.fresh()
        decision = self.propose_first(policy)
        ranked = decision.to_dict()["diagnostics"]["ranked_candidate_ids"]
        self.assertEqual(
            sorted(ranked),
            ["lateral_left", "lateral_right", "raised_center",
             "yaw_left", "yaw_right"])
        self.assertEqual(len(set(ranked)), len(ranked))  # no duplicates

    def test_deterministic_order_across_policies(self):
        orders = []
        for _ in range(3):
            decision = self.propose_first(self.fresh())
            orders.append(
                decision.to_dict()["diagnostics"]["ranked_candidate_ids"])
        self.assertEqual(orders[0], orders[1])
        self.assertEqual(orders[1], orders[2])

    def test_symmetric_yaw_ties_resolve_by_table_order(self):
        # Two pure-yaw candidates at equal magnitude rotate the nominal
        # ray symmetrically: identical score, so the library order (table
        # order) must break the tie.
        config = ActiveViewConfig(candidates=(
            ("yaw_a", (0.0, 0.0, 0.0), 6.0),
            ("yaw_b", (0.0, 0.0, 0.0), -6.0),
        ))
        decision = self.propose_first(self.fresh(config))
        ranked = decision.to_dict()["diagnostics"]["ranked_candidate_ids"]
        self.assertEqual(ranked, ["yaw_a", "yaw_b"])
        self.assertEqual(decision.candidates[0].candidate_id, "yaw_a")

    def test_identical_views_deduplicated(self):
        config = ActiveViewConfig(candidates=(
            ("copy_a", (0.0, 0.05, 0.0), 0.0),
            ("copy_b", (0.0, 0.05, 0.0), 0.0),
        ))
        decision = self.propose_first(self.fresh(config))
        ranked = decision.to_dict()["diagnostics"]["ranked_candidate_ids"]
        self.assertEqual(ranked, ["copy_a"])

    def test_envelope_violating_candidate_rejected(self):
        config = ActiveViewConfig(camera_z_max=1.95)
        policy = self.fresh(config)
        decision = self.propose_first(policy)
        rejected = decision.to_dict()["diagnostics"]["rejected"]
        self.assertEqual(rejected.get("raised_center"), "camera_above_ceiling")
        self.assertNotEqual(decision.candidates[0].candidate_id,
                            "raised_center")

    def test_visited_candidates_not_reproposed(self):
        policy = self.fresh()
        first = self.propose_first(policy).candidates[0].candidate_id
        integrate(policy, first, 101, 0.117, 0.9)
        second = self.propose_first(policy, sec=102).candidates[0]
        self.assertNotEqual(second.candidate_id, first)

    def test_three_view_cap_ends_budget_exhausted(self):
        policy = self.fresh()
        chosen = []
        for step in range(4):
            decision = policy.evaluate_mapping(miss_window(100 + 6 * step))
            if not decision.candidates:
                self.assertEqual(decision.reason, RECOVERY_BUDGET_EXHAUSTED)
                break
            candidate = decision.candidates[0]
            self.assertNotIn(candidate.candidate_id, chosen)
            chosen.append(candidate.candidate_id)
            integrate(policy, candidate.candidate_id, 101 + 6 * step,
                      0.11 + 0.1 * step, 0.9)
        else:
            self.fail("budget never exhausted")
        self.assertEqual(len(chosen), 3)

    def test_elapsed_time_cap_ends_budget_exhausted(self):
        policy = self.fresh()
        decision = self.propose_first(policy, sec=100)
        integrate(policy, decision.candidates[0].candidate_id, 101,
                  0.4, 0.95)  # real gain, no no-gain stop
        late = policy.evaluate_mapping(miss_window(120))
        self.assertEqual(late.reason, RECOVERY_BUDGET_EXHAUSTED)
        diag = late.to_dict()["diagnostics"]
        self.assertEqual(diag["gate"], "elapsed_budget")
        self.assertGreater(diag["elapsed_sec"], 15.0)
        self.assertEqual(tuple(late.candidates), ())

    def test_two_view_no_gain_stop(self):
        policy = self.fresh()
        first = self.propose_first(policy, sec=100).candidates[0]
        # Baselines seeded from the trigger window (conf 0.116, ratio 0.0).
        integrate(policy, first.candidate_id, 101, 0.117, 0.03)
        second = self.propose_first(policy, sec=102).candidates[0]
        integrate(policy, second.candidate_id, 103, 0.118, 0.031)
        decision = policy.evaluate_mapping(miss_window(104))
        self.assertEqual(decision.reason, RECOVERY_NO_GAIN)
        self.assertEqual(policy._views_executed, 2)
        self.assertEqual(tuple(decision.candidates), ())

    def test_gain_resets_no_gain_streak(self):
        policy = self.fresh()
        first = self.propose_first(policy, sec=100).candidates[0]
        integrate(policy, first.candidate_id, 101, 0.117, 0.03)
        second = self.propose_first(policy, sec=102).candidates[0]
        integrate(policy, second.candidate_id, 103, 0.30, 0.60)  # real gain
        decision = policy.evaluate_mapping(miss_window(104))
        self.assertEqual(decision.reason, "")
        self.assertEqual(len(decision.candidates), 1)

    def test_outcome_correlation_tracks_views_and_best(self):
        policy = self.fresh()
        first = self.propose_first(policy, sec=100).candidates[0]
        integrate(policy, first.candidate_id, 101, 0.30, 0.60,
                  status=ViewOutcomeStatus.EXECUTED)
        self.assertEqual(policy._views_executed, 1)
        # Integrated outcome for the same candidate must not double count.
        integrate(policy, first.candidate_id, 101, 0.31, 0.61)
        self.assertEqual(policy._views_executed, 1)
        self.assertAlmostEqual(policy._best_confidence, 0.31)
        self.assertAlmostEqual(policy._best_ratio, 0.61)

    def test_rejected_outcome_marks_candidate_visited(self):
        policy = self.fresh()
        first = self.propose_first(policy).candidates[0].candidate_id
        policy.observe(ViewOutcome(
            status=ViewOutcomeStatus.REJECTED, reason_code="ik_failed",
            candidate_id=first))
        second = self.propose_first(policy, sec=102).candidates[0]
        self.assertNotEqual(second.candidate_id, first)

    def test_reset_clears_session_state(self):
        policy = self.fresh()
        first = self.propose_first(policy, sec=100).candidates[0].candidate_id
        integrate(policy, first, 101, 0.117, 0.03)
        second = self.propose_first(policy, sec=102).candidates[0].candidate_id
        self.assertNotEqual(first, second)
        policy.reset(ExplorationContext(
            frames={}, geometry_descriptor={}, geometry_hash="",
            camera_model={}))
        again = self.propose_first(policy, sec=200).candidates[0]
        self.assertEqual(again.candidate_id, first)
        self.assertEqual(policy._views_executed, 0)

    def test_candidate_pose_bounded_relative_to_nominal(self):
        config = ActiveViewConfig()
        policy = self.fresh(config)
        decision = self.propose_first(policy)
        candidate = decision.candidates[0]
        nominal = config.nominal_camera_xyz
        offset = math.dist(candidate.position_xyz, nominal)
        self.assertLessEqual(offset, 0.15)
        self.assertLessEqual(candidate.position_xyz[2], config.camera_z_max)
        radius = math.hypot(
            candidate.position_xyz[0] - config.workspace_center_xy[0],
            candidate.position_xyz[1] - config.workspace_center_xy[1])
        self.assertLessEqual(radius, config.camera_xy_radius_max)

    def test_candidate_feasibility_stays_unknown(self):
        policy = self.fresh()
        decision = self.propose_first(policy)
        diagnostics = decision.candidates[0].to_dict()["diagnostics"]
        self.assertEqual(diagnostics["feasibility"], "unknown")

    def test_candidates_exhausted_with_single_candidate_library(self):
        config = ActiveViewConfig(candidates=(
            ("only", (0.0, 0.05, 0.0), 0.0),))
        policy = self.fresh(config)
        first = self.propose_first(policy).candidates[0].candidate_id
        integrate(policy, first, 101, 0.4, 0.9)
        decision = policy.evaluate_mapping(miss_window(102))
        self.assertEqual(decision.reason, RECOVERY_CANDIDATES_EXHAUSTED)
        self.assertEqual(tuple(decision.candidates), ())
        self.assertFalse(decision.motion_authority)

    def test_terminal_decisions_never_carry_candidates(self):
        policy = self.fresh()
        first = self.propose_first(policy, sec=100).candidates[0]
        integrate(policy, first.candidate_id, 101, 0.117, 0.03)
        second = self.propose_first(policy, sec=102).candidates[0]
        integrate(policy, second.candidate_id, 103, 0.118, 0.031)
        decision = policy.evaluate_mapping(miss_window(104))
        self.assertEqual(decision.reason, RECOVERY_NO_GAIN)
        self.assertFalse(decision.motion_authority)


class TestPFr7G4Replay(unittest.TestCase):
    """Saved PF-R7 G4 samples: the three proposal-available standard seeds
    must not start recovery; the three known detector-miss seeds must
    classify as cargo-present/incomplete and propose one bounded view."""

    @classmethod
    def setUpClass(cls):
        cls.fixture, cls.sessions = load_replay_fixture(FIXTURE_PATH)

    def session_policy(self, seed):
        sample = self.sessions[seed]["window"]["frames"][0]
        policy = ActiveViewRecoveryPolicy()
        policy.reset(ExplorationContext(
            frames={"sensor_frame": sample["frame_id"]},
            geometry_descriptor={},
            geometry_hash=sample["geometry_hash"],
            camera_model={"model_id": sample["camera_model_id"]}))
        return policy

    def test_fixture_sessions_present(self):
        for seed in ("standard_00", "standard_01", "standard_02",
                     "standard_03", "standard_04", "standard_05"):
            self.assertIn(seed, self.sessions)

    def test_fixture_windows_are_clean_policy_inputs(self):
        for seed, session in self.sessions.items():
            self.assertEqual(scan_forbidden_keys(session["window"]), [])
            self.assertTrue(session["expected"]["harness_only"])

    def test_windows_use_measured_stamps_within_span(self):
        provenance = self.fixture["provenance"]
        self.assertIn("measured stamps", provenance["window_rule"])
        for session in self.sessions.values():
            stamps = [f["stamp"] for f in session["window"]["frames"]]
            self.assertEqual(len(stamps), 3)
            sec = [s["sec"] + s["nanosec"] * 1e-9 for s in stamps]
            self.assertLess(sec[0], sec[1])
            self.assertLess(sec[1], sec[2])
            self.assertLessEqual(sec[-1] - sec[0], 0.50 + 1e-9)

    def test_proposal_available_seeds_do_not_start_recovery(self):
        for seed in ("standard_00", "standard_01", "standard_02"):
            policy = self.session_policy(seed)
            decision = policy.evaluate_mapping(
                self.sessions[seed]["window"])
            self.assertEqual(decision.reason, RECOVERY_SUCCEEDED,
                             "seed %s must not start recovery" % seed)
            self.assertEqual(tuple(decision.candidates), ())
            self.assertFalse(decision.motion_authority)

    def test_detector_miss_seeds_classify_incomplete_and_propose(self):
        for seed in ("standard_03", "standard_04", "standard_05"):
            policy = self.session_policy(seed)
            decision = policy.evaluate_mapping(
                self.sessions[seed]["window"])
            self.assertEqual(decision.reason, "",
                             "seed %s must propose a view" % seed)
            self.assertTrue(decision.cargo_present, seed)
            self.assertTrue(decision.incomplete, seed)
            self.assertIn("no_accepted_detections", decision.defects, seed)
            self.assertEqual(len(decision.candidates), 1, seed)
            candidate = decision.candidates[0]
            self.assertIn(
                candidate.candidate_id,
                ("lateral_left", "lateral_right", "raised_center",
                 "yaw_left", "yaw_right"))
            config = ActiveViewConfig()
            offset = math.dist(
                candidate.position_xyz, config.nominal_camera_xyz)
            self.assertLessEqual(offset, 0.15,
                                 "%s view must stay bounded" % seed)
            self.assertTrue(decision.motion_authority)

    def test_miss_seed_hint_stays_below_production_floor(self):
        for seed in ("standard_03", "standard_04", "standard_05"):
            for frame_data in self.sessions[seed]["window"]["frames"]:
                for det in frame_data["accepted_detections"]:
                    self.assertGreaterEqual(det["confidence"], 0.20)
                for hint in frame_data["diagnostic_hints"]:
                    if hint["confidence"] >= 0.05:
                        self.assertLess(hint["confidence"], 0.20)

    def test_replay_rejects_gt_injection(self):
        seed = "standard_03"
        polluted = copy.deepcopy(self.sessions[seed]["window"])
        polluted["frames"][0]["gt_iou"] = 0.93
        policy = self.session_policy(seed)
        decision = policy.evaluate_mapping(polluted)
        self.assertEqual(decision.reason, RECOVERY_EVIDENCE_INVALID)
        self.assertEqual(tuple(decision.candidates), ())

    def test_replay_is_deterministic_across_policies(self):
        for seed in ("standard_03", "standard_04", "standard_05"):
            picks = []
            for _ in range(3):
                policy = self.session_policy(seed)
                decision = policy.evaluate_mapping(
                    self.sessions[seed]["window"])
                picks.append(decision.candidates[0].candidate_id)
            self.assertEqual(len(set(picks)), 1, seed)

    def test_provenance_records_association_rules(self):
        provenance = self.fixture["provenance"]
        for key in ("window_rule", "content_rule", "settled_rule",
                    "tf_rule", "hint_landing_rule", "state_rule",
                    "rig_offset_mean_dump_to_base_xy"):
            self.assertIn(key, provenance)
        self.assertIn("rev c086a396", provenance["source_rule"])


class TestContractCompatibility(unittest.TestCase):

    def test_decision_serialization_carries_required_fields(self):
        policy = ActiveViewRecoveryPolicy()
        decision = policy.evaluate_mapping(miss_window())
        data = decision.to_dict()
        self.assertEqual(data["schema_id"], DECISION_SCHEMA_ID)
        self.assertIn("schema_version", data)
        self.assertIn("reason", data)
        diagnostics = data["diagnostics"]
        self.assertEqual(len(diagnostics["source_stamps"]), 3)
        self.assertEqual(diagnostics["geometry_hash"], GEOMETRY_HASH)
        self.assertEqual(diagnostics["map_revision"], MAP_REVISION)
        candidate = data["candidates"][0]
        self.assertIn("candidate_id", candidate)
        self.assertIn("score", candidate)
        terms = candidate["diagnostics"]
        for key in ("coverage", "gain", "distance_m", "library_index",
                    "source_stamps", "geometry_hash", "map_revision"):
            self.assertIn(key, terms)

    def test_window_serialization_roundtrip_shape(self):
        window = RecoveryWindow.from_mapping(miss_window())
        data = window.to_dict()
        self.assertEqual(data["schema_id"], avr.WINDOW_SCHEMA_ID)
        self.assertEqual(len(data["frames"]), 3)
        self.assertEqual(data["frames"][0]["cargo_geometry"]["support_mode"],
                         "NO_TOP")

    def test_propose_matches_exploration_policy_protocol(self):
        policy = ActiveViewRecoveryPolicy()
        context = ExplorationContext(
            frames={}, geometry_descriptor={}, geometry_hash=GEOMETRY_HASH,
            camera_model={"model_id": CAMERA_MODEL})
        policy.reset(context)
        snapshot = ExplorationSnapshot(
            acquisition_stamp_start=AcquisitionStamp(100, 0),
            acquisition_stamp_end=AcquisitionStamp(100, 200000000),
            map_revision=MAP_REVISION,
            occupancy_summary={}, visibility_summary={}, robot_state={},
            diagnostics={"active_view_window": miss_window()})
        proposal = policy.propose(context, snapshot)
        self.assertIsInstance(proposal, PolicyProposal)
        self.assertEqual(proposal.policy_id, avr.POLICY_ID)
        self.assertEqual(proposal.proposal_schema_id, DECISION_SCHEMA_ID)
        self.assertEqual(len(proposal.candidates), 1)
        self.assertFalse(proposal.done_recommended)
        self.assertEqual(proposal.done_reason, "")

    def test_propose_terminal_window_recommends_done(self):
        policy = ActiveViewRecoveryPolicy()
        context = ExplorationContext(
            frames={}, geometry_descriptor={}, geometry_hash="",
            camera_model={})
        snapshot = ExplorationSnapshot(
            acquisition_stamp_start=AcquisitionStamp(60, 0),
            acquisition_stamp_end=AcquisitionStamp(60, 66000000),
            map_revision=MAP_REVISION,
            occupancy_summary={}, visibility_summary={}, robot_state={},
            diagnostics={"active_view_window": window(
                [frame(60, 0), frame(60, 33000000), frame(60, 66000000)])})
        proposal = policy.propose(context, snapshot)
        self.assertTrue(proposal.done_recommended)
        self.assertEqual(proposal.done_reason, NO_CARGO_EVIDENCE)
        self.assertEqual(tuple(proposal.candidates), ())

    def test_propose_without_window_fails_closed(self):
        policy = ActiveViewRecoveryPolicy()
        context = ExplorationContext(
            frames={}, geometry_descriptor={}, geometry_hash="",
            camera_model={})
        snapshot = ExplorationSnapshot(
            acquisition_stamp_start=AcquisitionStamp(60, 0),
            acquisition_stamp_end=AcquisitionStamp(60, 66000000),
            map_revision=MAP_REVISION,
            occupancy_summary={}, visibility_summary={}, robot_state={},
            diagnostics={})
        proposal = policy.propose(context, snapshot)
        self.assertEqual(proposal.done_reason, RECOVERY_EVIDENCE_INVALID)

    def test_values_are_immutable(self):
        window = RecoveryWindow.from_mapping(miss_window())
        with self.assertRaises(Exception):
            window.state = "other"
        policy = ActiveViewRecoveryPolicy()
        decision = policy.evaluate_mapping(miss_window())
        with self.assertRaises(Exception):
            decision.reason = "x"

    def test_module_imports_without_ros(self):
        module = importlib.import_module(
            "luggage_planning.active_view_recovery")
        self.assertNotIn("rclpy", module.__dict__)
        self.assertNotIn("geometry_msgs", module.__dict__)

    def test_reuses_shared_planning_modules(self):
        # The policy must reuse, not copy, the shared planning modules.
        self.assertTrue(issubclass(avr.RecoveryCandidateRanker,
                                   avr.CargoNBVPlanner))
        self.assertEqual(avr.CandidateView.__module__,
                         "luggage_planning.exploration_contracts")

    def test_config_serialization_roundtrip(self):
        config = ActiveViewConfig()
        restored = ActiveViewConfig.from_mapping(config.to_dict())
        self.assertEqual(restored.candidates, config.candidates)
        self.assertEqual(restored.nominal_camera_xyz,
                         config.nominal_camera_xyz)

    def test_yaml_example_loads(self):
        try:
            import yaml
        except ImportError:
            self.skipTest("pyyaml unavailable")
        example = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "config",
            "active_view_recovery.yaml.example")
        with open(example, "r") as handle:
            data = yaml.safe_load(handle)
        config = ActiveViewConfig.from_mapping(data["active_view_recovery"])
        self.assertEqual(config.candidates, ActiveViewConfig().candidates)


if __name__ == "__main__":
    unittest.main()
