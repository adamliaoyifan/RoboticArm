"""ST09-B acceptance (b)/(c): pure target synthesis and IK goal sets.

Covers, against ``luggage_planning.place.place_targets``:

- the eight sim/deploy target fixtures at <= 1e-9 against independent
  scipy expectations, including the theta+pi eccentric heading;
- q/-q and pi/-pi representation equivalence (never a refusal);
- every 5.1 negative as a named :class:`TargetSynthesisError` with the
  frozen Reason, and no partial output (zero business effect);
- container-candidate -> world BoxTarget composition;
- goal_joint_sets semantics: no tool-yaw pruning, exact-q dedupe only,
  same-XYZ different-q goals stay distinct, failed seeds contribute
  nothing, and (real-URDF, skippable) rotation-valued q/-q refinement.
"""

from __future__ import division

import dataclasses
import math

import pytest

np = pytest.importorskip("numpy")
scipy_rot = pytest.importorskip("scipy.spatial.transform").Rotation

from luggage_planning.planning_contract.attachment import (  # noqa: E402
    AttachmentSourceKind,
    AttachmentValidity,
    PlaceTarget,
    quaternion_angle_between,
)
from luggage_planning.planning_contract.fixtures import (  # noqa: E402
    BOX_WDH,
    make_attachment,
    make_box_target,
    target_fixture_matrix,
)
from luggage_planning.planning_contract.planning import (  # noqa: E402
    GeometryVerdict,
    PlacementCandidate,
)
from luggage_planning.planning_contract.reasons import Reason  # noqa: E402

from luggage_planning.place.place_targets import (  # noqa: E402
    GoalSeed,
    TargetSynthesisError,
    box_target_from_candidate,
    expand_place_targets,
    goal_joint_sets,
)

PURE_TOL = 1e-9


def _expected_tcp(record, heading, box_position):
    """Independent scipy expectation: T_world_tcp per heading."""
    rot_tcp_box = scipy_rot.from_quat(record.t_tcp_box_quaternion_xyzw)
    rot_box_tcp = rot_tcp_box.inv()
    t_box_tcp = -rot_box_tcp.apply(np.array(record.t_tcp_box_translation))
    rot_world_box = scipy_rot.from_euler("z", heading)
    position = np.array(box_position) + rot_world_box.apply(t_box_tcp)
    rotation = rot_world_box * rot_box_tcp
    return position, rotation


def _assert_target_matches(target, expected_position, expected_rotation):
    for got, want in zip(target.position, expected_position):
        assert abs(got - want) <= PURE_TOL
    quat = scipy_rot.from_quat(target.quaternion_xyzw)
    assert (quat.inv() * expected_rotation).magnitude() <= PURE_TOL


class TestTargetFixtureMatrix:
    """(b) eight sim/deploy fixtures, both symmetry headings, <= 1e-9."""

    @pytest.mark.parametrize("kind,scene,calibration", target_fixture_matrix())
    @pytest.mark.parametrize("heading_theta", [0.0, 0.3 - math.pi])
    def test_fixture_targets_match_scipy(self, kind, scene, calibration,
                                         heading_theta):
        record = make_attachment(
            kind, scene_revision=scene, calibration_id=calibration)
        box = make_box_target(record, heading_theta=heading_theta)
        targets = expand_place_targets(box, record)
        headings = (heading_theta, heading_theta + math.pi)
        assert tuple(t.heading_theta for t in targets) == pytest.approx(
            headings, abs=PURE_TOL)
        for target, heading in zip(targets, headings):
            assert target.attachment_digest == record.digest()
            position, rotation = _expected_tcp(record, heading, box.position)
            _assert_target_matches(target, position, rotation)

    @pytest.mark.parametrize("kind,scene,calibration", target_fixture_matrix())
    def test_symmetry_none_keeps_single_heading(self, kind, scene, calibration):
        record = make_attachment(
            kind, scene_revision=scene, calibration_id=calibration,
            symmetry="none")
        targets = expand_place_targets(make_box_target(record), record)
        assert len(targets) == 1

    def test_eccentric_theta_plus_pi_is_a_different_tcp(self):
        record = make_attachment("eccentric")
        first, second = expand_place_targets(
            make_box_target(record, heading_theta=0.2), record)
        distance = math.sqrt(sum(
            (a - b) ** 2 for a, b in zip(first.position, second.position)))
        assert distance == pytest.approx(
            2.0 * math.hypot(0.15, 0.08), abs=PURE_TOL)
        assert quaternion_angle_between(
            first.quaternion_xyzw, second.quaternion_xyzw) == pytest.approx(
            math.pi, abs=PURE_TOL)


class TestRepresentationEquivalence:
    """q/-q and pi/-pi are the same rotation: never a refusal, never drift."""

    def test_negated_box_quaternion_same_targets(self):
        record = make_attachment("relative_yaw")
        box = make_box_target(record, heading_theta=0.4)
        flipped = dataclasses.replace(
            box, quaternion_xyzw=tuple(-v for v in box.quaternion_xyzw))
        for plain, negated in zip(expand_place_targets(box, record),
                                  expand_place_targets(flipped, record)):
            assert plain.position == pytest.approx(negated.position, abs=PURE_TOL)
            assert quaternion_angle_between(
                plain.quaternion_xyzw, negated.quaternion_xyzw) <= PURE_TOL

    def test_pi_and_minus_pi_headings_equivalent(self):
        record = make_attachment("centered", symmetry="none")
        plus = expand_place_targets(
            make_box_target(record, heading_theta=math.pi), record)
        minus = expand_place_targets(
            make_box_target(record, heading_theta=-math.pi), record)
        assert len(plus) == len(minus) == 1
        assert plus[0].position == pytest.approx(minus[0].position, abs=PURE_TOL)
        assert quaternion_angle_between(
            plus[0].quaternion_xyzw, minus[0].quaternion_xyzw) <= PURE_TOL


class TestNamedNegatives:
    """(c) every 5.1 refusal arrives named, with zero business effect."""

    def test_invalidated_attachment_missing(self):
        record = make_attachment(
            "centered", validity=AttachmentValidity.INVALIDATED,
            invalidation_reason="slip")
        with pytest.raises(TargetSynthesisError) as err:
            expand_place_targets(make_box_target(record), record)
        assert err.value.reason is Reason.ATTACHMENT_MISSING

    def test_sim_gt_source_barred_as_measured(self):
        record = make_attachment(
            "centered", source=AttachmentSourceKind.REFERENCE_SIM_GT)
        with pytest.raises(TargetSynthesisError) as err:
            expand_place_targets(make_box_target(record), record)
        assert err.value.reason is Reason.ATTACHMENT_GT_AS_MEASURED

    def test_pre_attach_estimate_requires_flag(self):
        record = make_attachment(
            "centered", source=AttachmentSourceKind.MEASURED_PRE_ATTACH_ESTIMATE)
        with pytest.raises(TargetSynthesisError) as err:
            expand_place_targets(make_box_target(record), record)
        assert err.value.reason is Reason.ATTACHMENT_SOURCE_UNQUALIFIED
        # Pre-attach initial selection stays legal with the explicit flag.
        targets = expand_place_targets(
            make_box_target(record), record, require_attach_confirmed=False)
        assert len(targets) >= 1

    def test_request_mismatch(self):
        record = make_attachment("centered")
        box = make_box_target(record)
        other = dataclasses.replace(box, request_id="req-other")
        with pytest.raises(TargetSynthesisError) as err:
            expand_place_targets(other, record)
        assert err.value.reason is Reason.ATTACHMENT_REQUEST_MISMATCH

    def test_scene_frame_mismatch(self):
        record = make_attachment("centered")
        box = make_box_target(record)
        other = dataclasses.replace(box, scene_revision="scene-other")
        with pytest.raises(TargetSynthesisError) as err:
            expand_place_targets(other, record)
        assert err.value.reason is Reason.ATTACHMENT_FRAME_MISMATCH

    def test_target_stamp_before_acquisition(self):
        record = make_attachment("centered", stamp_ns=2_000_000_000)
        box = make_box_target(record, stamp_ns=1_000_000_000)
        with pytest.raises(TargetSynthesisError) as err:
            expand_place_targets(box, record)
        assert err.value.reason is Reason.STAMP_INVALID

    def test_expired_attachment(self):
        record = make_attachment("centered", stamp_ns=1_000_000_000)
        from luggage_planning.planning_contract.attachment import ATTACHMENT_MAX_AGE_NS
        box = make_box_target(record, stamp_ns=record.acquisition_stamp_ns + ATTACHMENT_MAX_AGE_NS + 1)
        with pytest.raises(TargetSynthesisError) as err:
            expand_place_targets(box, record)
        assert err.value.reason is Reason.ATTACHMENT_EXPIRED

    def test_negative_detail_is_preserved(self):
        record = make_attachment(
            "centered", validity=AttachmentValidity.SUPERSEDED,
            invalidation_reason="re_attach")
        with pytest.raises(TargetSynthesisError) as err:
            expand_place_targets(make_box_target(record), record)
        assert err.value.code == Reason.ATTACHMENT_MISSING.value
        assert "superseded" in err.value.detail


def _candidate(heading=0.0, position=(0.20, -0.10, 0.5 * BOX_WDH[2])):
    return PlacementCandidate(
        candidate_id="cand-b-1",
        footprint_class=0,
        heading_theta=heading,
        position_container=tuple(float(v) for v in position),
        quaternion_xyzw_container=(0.0, 0.0, math.sin(heading / 2.0),
                                   math.cos(heading / 2.0)),
        support_layer_z=0.0,
        depth_m=0.6,
        verdict=GeometryVerdict.LEGAL,
        score=1.0,
        detail={"route_clearance_m": 0.05},
    )


class TestBoxTargetFromCandidate:
    """Container-frame candidate pose -> planning-frame BoxTarget."""

    def test_identity_transform_keeps_container_pose(self):
        record = make_attachment("centered")
        candidate = _candidate(heading=0.7)
        box = box_target_from_candidate(candidate, record, stamp_ns=2_000_000_000)
        assert box.position == pytest.approx(candidate.position_container)
        assert box.request_id == record.request_id
        assert box.scene_revision == record.scene_revision
        assert box.heading_theta == pytest.approx(0.7)
        expand_place_targets(box, record)  # composes cleanly with the contract

    def test_world_transform_matches_scipy(self):
        record = make_attachment("eccentric")
        candidate = _candidate(heading=0.0)
        t_frame = (1.2, -0.4, 0.05)
        q_frame = scipy_rot.from_euler("xyz", (0.0, 0.0, -math.pi / 2))
        box = box_target_from_candidate(
            candidate, record, stamp_ns=2_000_000_000,
            t_frame_container=t_frame,
            q_frame_container=q_frame.as_quat())
        expected_position = np.array(t_frame) + q_frame.apply(
            np.array(candidate.position_container))
        assert box.position == pytest.approx(expected_position, abs=PURE_TOL)
        expected_rot = q_frame * scipy_rot.from_quat(
            candidate.quaternion_xyzw_container)
        assert (scipy_rot.from_quat(box.quaternion_xyzw).inv()
                * expected_rot).magnitude() <= PURE_TOL

    def test_request_override_surfaces_named_mismatch(self):
        record = make_attachment("centered")
        box = box_target_from_candidate(
            _candidate(), record, stamp_ns=2_000_000_000,
            request_id="req-elsewhere")
        with pytest.raises(TargetSynthesisError) as err:
            expand_place_targets(box, record)
        assert err.value.reason is Reason.ATTACHMENT_REQUEST_MISMATCH


def _fake_refine(solutions):
    """refine(pos, quat, seed) returning scripted per-seed outcomes."""
    calls = []

    def refine(position, quaternion, seed):
        calls.append((tuple(position), tuple(quaternion), tuple(seed)))
        return solutions[tuple(seed)]

    refine.calls = calls
    return refine


class TestGoalJointSets:
    """No tool-yaw pruning; exact-q dedupe; same-XYZ different-q distinct."""

    @staticmethod
    def _targets():
        record = make_attachment("centered", symmetry="none")
        return expand_place_targets(make_box_target(record), record)

    def test_all_seed_families_kept_in_order(self):
        seeds = [
            GoalSeed((0.1, 0.2, 0.3, 0.4, 0.5, 0.6), "f1", "atlas-transit"),
            GoalSeed((-0.1, 0.2, -0.3, 0.4, -0.5, 0.6), "f2", "atlas-contact"),
            GoalSeed((1.1, 0.2, 0.3, 0.4, 0.5, -0.6), "f1", "current"),
        ]
        outcomes = {
            seed.q: (list(seed.q), {"pos_err_m": 1e-6, "rot_err_rad": 1e-5})
            for seed in seeds}
        refine = _fake_refine(outcomes)
        (goal_set,) = goal_joint_sets(
            self._targets(), seeds_for_target=lambda _t: seeds, refine=refine)
        assert [g.q for g in goal_set.goals] == [s.q for s in seeds]
        assert [g.ik_family for g in goal_set.goals] == ["f1", "f2", "f1"]
        assert goal_set.goals[0].position_error_m == pytest.approx(1e-6)

    def test_exact_q_dedupe_only(self):
        seeds = [
            GoalSeed((0.1, 0.2, 0.3, 0.4, 0.5, 0.6), "f1", "atlas"),
            GoalSeed((0.1 + 5e-10, 0.2, 0.3, 0.4, 0.5, 0.6), "f2", "neighbour"),
        ]
        outcomes = {
            seed.q: (list(seed.q), {"pos_err_m": 0.0, "rot_err_rad": 0.0})
            for seed in seeds}
        (goal_set,) = goal_joint_sets(
            self._targets(), seeds_for_target=lambda _t: seeds, refine=_fake_refine(outcomes))
        assert len(goal_set.goals) == 1
        assert goal_set.goals[0].provenance == "atlas"  # first occurrence wins

    def test_same_tcp_position_different_q_both_kept(self):
        seeds = [
            GoalSeed((0.1, 0.2, 0.3, 0.4, 0.5, 0.6), "f1", "atlas"),
            GoalSeed((-3.0, 0.2, 0.3, 0.4, 0.5, 0.6), "f3", "pi-twin"),
        ]
        outcomes = {
            seed.q: (list(seed.q), {"pos_err_m": 0.0, "rot_err_rad": 0.0})
            for seed in seeds}
        (goal_set,) = goal_joint_sets(
            self._targets(), seeds_for_target=lambda _t: seeds, refine=_fake_refine(outcomes))
        assert len({g.q for g in goal_set.goals}) == 2

    def test_failed_seed_contributes_nothing(self):
        seeds = [
            GoalSeed((0.1, 0.2, 0.3, 0.4, 0.5, 0.6), "f1", "atlas"),
            GoalSeed((9.9, 9.9, 9.9, 9.9, 9.9, 9.9), "bad", "unreachable"),
        ]
        outcomes = {
            seeds[0].q: (list(seeds[0].q),
                         {"pos_err_m": 1e-6, "rot_err_rad": 1e-5}),
            seeds[1].q: (None, {"pos_err_m": 0.4, "rot_err_rad": 0.5}),
        }
        (goal_set,) = goal_joint_sets(
            self._targets(), seeds_for_target=lambda _t: seeds, refine=_fake_refine(outcomes))
        assert [g.provenance for g in goal_set.goals] == ["atlas"]

    def test_empty_goal_set_is_a_prior_gap_not_an_error(self):
        (goal_set,) = goal_joint_sets(
            self._targets(), seeds_for_target=lambda _t: [],
            refine=_fake_refine({}))
        assert goal_set.goals == ()

    def test_needs_urdf_without_refine(self):
        with pytest.raises(ValueError):
            goal_joint_sets(
                self._targets(), seeds_for_target=lambda _t: [])


class TestRealUrdfGoals:
    """Real offline_ik refinement (skipped when no install overlay)."""

    #: Known-reachable sim-scene contact class (2026-09-24 adaptive-lift
    #: evidence, mirrored from test_frame_migration_atlas).
    SEED_Q = (0.936, -1.025, 1.268, -1.819, -1.567, -0.635)
    CONTAINER_IN_BASE = ((0.02, -1.51, -0.86), (0.0, 0.0, -math.pi / 2.0))

    @pytest.fixture(autouse=True)
    def _load(self):
        try:
            from luggage_planning.motion.offline_ik import load_urdf
            urdf = load_urdf()
        except Exception as err:  # no install overlay / xacro missing
            pytest.skip("offline_ik load_urdf unavailable: %s" % err)
        path = urdf.path_to("suction_contact_frame")
        self.tcp_path = path[path.index("elfin_joint1"):]
        self.urdf = urdf

    def _place_target(self, quaternion_xyzw):
        from luggage_planning.atlas.frame_contracts import tool_target_in_base
        position, _tool_down = tool_target_in_base(
            -0.37, -0.61, 0.875, math.pi / 2.0, *self.CONTAINER_IN_BASE)
        record = make_attachment("centered", symmetry="none")
        return position, PlaceTarget(
            candidate_id="cand-b-1",
            heading_theta=0.0,
            tcp_frame=record.tcp_frame,
            frame_id="base_link",
            position=position,
            quaternion_xyzw=tuple(quaternion_xyzw),
            attachment_digest=record.digest())

    def _run(self, quaternion_xyzw):
        target = self._place_target(quaternion_xyzw)[1]
        (goal_set,) = goal_joint_sets(
            (target,),
            seeds_for_target=lambda _t: [GoalSeed(
                self.SEED_Q, "contact", "atlas-fixture")],
            urdf=self.urdf, tcp_path=self.tcp_path)
        return goal_set.goals

    def test_refine_converges_and_matches_fk(self):
        from luggage_planning.atlas.frame_contracts import TOOL_DOWN_QUAT
        position, _target = self._place_target(TOOL_DOWN_QUAT)
        goals = self._run(TOOL_DOWN_QUAT)
        assert len(goals) == 1
        tf = self.urdf.tcp(list(goals[0].q), self.tcp_path)
        for got, want in zip(tf[:3, 3], position):
            assert abs(got - want) <= 5e-4

    def test_q_minus_q_target_refines_identically(self):
        from luggage_planning.atlas.frame_contracts import TOOL_DOWN_QUAT
        forward = self._run(TOOL_DOWN_QUAT)
        negated = self._run(tuple(-v for v in TOOL_DOWN_QUAT))
        assert forward and negated
        assert forward[0].q == pytest.approx(negated[0].q, abs=1e-6)
