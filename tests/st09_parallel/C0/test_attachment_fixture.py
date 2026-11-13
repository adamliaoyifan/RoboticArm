"""C0 attachment contract tests: the 8 target fixtures vs independent
matrix expectations (<=1e-9), the 5.1 nine negatives, and the provider
fixture semantics.
"""

from __future__ import annotations

import math

import pytest

from luggage_planning.planning_contract import (
    ATTACHMENT_SCHEMA_VERSION,
    AttachmentRecord,
    AttachmentSourceKind,
    AttachmentValidity,
    BoxTarget,
    PlaceTarget,
    Reason,
    resolve_place_targets,
)
from luggage_planning.planning_contract._base import ContractViolation
from luggage_planning.planning_contract.attachment import (
    ATTACHMENT_MAX_AGE_NS,
    GRASP_TOLERANCE_M,
    GRASP_TOLERANCE_RAD,
    TF_SYNC_MAX_SKEW_NS,
    compose_transform,
    expand_symmetric_heading,
    invert_transform,
)
from luggage_planning.planning_contract.fixtures import (
    DEPLOY_CALIBRATION,
    DEPLOY_SCENE,
    SIM_CALIBRATION,
    SIM_SCENE,
    FakeAttachmentProvider,
    make_attachment,
    make_box_target,
    target_fixture_matrix,
)

SCIPI_ROTATION = pytest.importorskip(
    "scipy.spatial.transform").Rotation


def _independent_expected_tcp(box_position, box_quat_xyzw, t_box_tcp, q_box_tcp):
    """Expected T_world_tcp via scipy: inverse(T_tcp_box) then compose."""
    import numpy as np
    r_world_box = SCIPI_ROTATION.from_quat(box_quat_xyzw)
    r_box_tcp = SCIPI_ROTATION.from_quat(q_box_tcp)
    # T_world_tcp = T_world_box * T_box_tcp
    r_world_tcp = r_world_box * r_box_tcp
    t_world_tcp = np.asarray(box_position) + r_world_box.apply(np.asarray(t_box_tcp))
    return tuple(t_world_tcp), tuple(r_world_tcp.as_quat())


def _attachment_t_box_tcp(record: AttachmentRecord):
    """Author-side T_box_tcp recovered by inverting the stored T_tcp_box."""
    return invert_transform(record.t_tcp_box_translation,
                            record.t_tcp_box_quaternion_xyzw)


class TestTargetFixtureMatrix:
    @pytest.mark.parametrize("kind,scene,calibration", target_fixture_matrix())
    def test_eight_fixtures_match_independent_expectation(self, kind, scene, calibration):
        record = make_attachment(kind, scene_revision=scene, calibration_id=calibration)
        target = make_box_target(record)
        targets = resolve_place_targets(target, record)
        target_t = next(t for t in targets
                        if t.heading_theta == target.heading_theta)
        t_box_tcp, q_box_tcp = _attachment_t_box_tcp(record)
        exp_t, exp_q = _independent_expected_tcp(
            target.position, target.quaternion_xyzw, t_box_tcp, q_box_tcp)
        assert max(abs(a - b) for a, b in zip(target_t.position, exp_t)) <= 1e-9
        # quaternion equivalence: rotation angle between them is 0 (sign-free)
        from luggage_planning.planning_contract.attachment import quaternion_angle_between
        assert quaternion_angle_between(target_t.quaternion_xyzw, exp_q) <= 1e-9

    def test_centre_height_never_readds_half_box(self):
        record = make_attachment("centered")
        target = make_box_target(record, position=(0.2, 0.0, 0.5 * record.dimensions_wdh[2]))
        resolved = next(t for t in resolve_place_targets(target, record)
                        if t.heading_theta == target.heading_theta)
        _, q_box_tcp = _attachment_t_box_tcp(record)
        exp_t, _ = _independent_expected_tcp(
            target.position, target.quaternion_xyzw,
            (0.0, 0.0, 0.5 * record.dimensions_wdh[2]), q_box_tcp)
        assert max(abs(a - b) for a, b in zip(resolved.position, exp_t)) <= 1e-9

    def test_theta_plus_pi_eccentric_targets_differ(self):
        record = make_attachment("eccentric", symmetry="pi")
        target = make_box_target(record, heading_theta=0.0)
        first, second = resolve_place_targets(target, record)
        assert first.heading_theta != second.heading_theta
        assert first.position != second.position  # eccentric grasp: distinct TCP
        distance = math.dist(first.position, second.position)
        assert distance == pytest.approx(2 * math.hypot(0.15, 0.08), abs=1e-9)

    def test_symmetry_none_forbids_theta_plus_pi(self):
        record = make_attachment("eccentric", symmetry="none")
        target = make_box_target(record)
        (only,) = resolve_place_targets(target, record)
        assert only.heading_theta == 0.0
        assert expand_symmetric_heading(0.0, "none") == (0.0,)
        assert expand_symmetric_heading(0.0, "pi") == (0.0, math.pi)

    def test_q_neg_q_representation_equivalence(self):
        record = make_attachment("relative_yaw")
        target = make_box_target(record)
        one = next(t for t in resolve_place_targets(target, record)
                   if t.heading_theta == target.heading_theta)
        flipped = BoxTarget(
            candidate_id=target.candidate_id, request_id=target.request_id,
            frame_id=target.frame_id, scene_revision=target.scene_revision,
            heading_theta=target.heading_theta,
            position=target.position,
            quaternion_xyzw=tuple(-v for v in target.quaternion_xyzw),
            stamp_ns=target.stamp_ns)
        other = next(t for t in resolve_place_targets(flipped, record)
                     if t.heading_theta == flipped.heading_theta)
        # same rotation despite q/-q: positions equal, orientations equal in angle
        assert one.position == other.position
        from luggage_planning.planning_contract.attachment import quaternion_angle_between
        assert quaternion_angle_between(one.quaternion_xyzw, other.quaternion_xyzw) <= 1e-9


class TestAttachmentNegatives:
    """The 5.1 nine negative classes: named rejection, no fake valid record."""

    def _record(self, **overrides):
        base = dict(
            schema_version=ATTACHMENT_SCHEMA_VERSION, cargo_id="c", box_id="b",
            request_id="r", cycle_id="y", attachment_incarnation=0,
            tcp_frame="suction_contact_frame", box_frame="box",
            dimensions_wdh=(0.646, 0.409, 0.277), symmetry="pi",
            t_tcp_box_translation=(0.0, 0.0, -0.1385),
            t_tcp_box_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            acquisition_stamp_ns=1_000_000_000, scene_revision="s",
            calibration_id="c", source=AttachmentSourceKind.MEASURED_ATTACH_CONFIRMED,
            source_method="same-stamp measured", evidence_digest="d" * 64,
            validity=AttachmentValidity.VALID)
        base.update(overrides)
        return AttachmentRecord(**base)

    def test_missing(self):
        provider = FakeAttachmentProvider([])
        with pytest.raises(ContractViolation) as err:
            provider.latest_record("req-none")
        assert Reason.ATTACHMENT_MISSING.value in str(err.value)

    def test_nan_transform(self):
        with pytest.raises(ContractViolation):
            self._record(t_tcp_box_translation=(float("nan"), 0.0, 0.0))

    def test_non_positive_dimensions(self):
        with pytest.raises(ContractViolation):
            self._record(dimensions_wdh=(0.646, 0.409, 0.0))

    def test_illegal_quaternion(self):
        with pytest.raises(ContractViolation):
            self._record(t_tcp_box_quaternion_xyzw=(0.0, 0.0, 0.0, 0.0))

    def test_expired(self):
        record = self._record(acquisition_stamp_ns=0)
        target = BoxTarget(
            candidate_id="c", request_id="r", frame_id="world",
            scene_revision="s", heading_theta=0.0, position=(0, 0, 0),
            quaternion_xyzw=(0, 0, 0, 1),
            stamp_ns=ATTACHMENT_MAX_AGE_NS + 1)
        with pytest.raises(ContractViolation) as err:
            resolve_place_targets(target, record)
        assert Reason.ATTACHMENT_EXPIRED.value in str(err.value)

    def test_wrong_frame(self):
        record = self._record(scene_revision="scene-a")
        target = BoxTarget(
            candidate_id="c", request_id="r", frame_id="world",
            scene_revision="scene-b", heading_theta=0.0, position=(0, 0, 0),
            quaternion_xyzw=(0, 0, 0, 1), stamp_ns=1_000_000_000)
        with pytest.raises(ContractViolation) as err:
            resolve_place_targets(target, record)
        assert Reason.ATTACHMENT_FRAME_MISMATCH.value in str(err.value)

    def test_wrong_request(self):
        record = self._record(request_id="r1")
        target = BoxTarget(
            candidate_id="c", request_id="r2", frame_id="world",
            scene_revision="s", heading_theta=0.0, position=(0, 0, 0),
            quaternion_xyzw=(0, 0, 0, 1), stamp_ns=1_000_000_000)
        with pytest.raises(ContractViolation) as err:
            resolve_place_targets(target, record)
        assert Reason.ATTACHMENT_REQUEST_MISMATCH.value in str(err.value)

    def test_reattach_invalidates_old_record(self):
        provider = FakeAttachmentProvider([self._record()])
        note = provider.invalidate(Reason.ATTACHMENT_REATTACH, {})
        assert note.reason is Reason.ATTACHMENT_REATTACH
        with pytest.raises(ContractViolation):
            provider.latest_record("r")

    def test_gt_as_measured(self):
        gt = self._record(source=AttachmentSourceKind.REFERENCE_SIM_GT)
        target = BoxTarget(
            candidate_id="c", request_id="r", frame_id="world",
            scene_revision="s", heading_theta=0.0, position=(0, 0, 0),
            quaternion_xyzw=(0, 0, 0, 1), stamp_ns=1_000_000_000)
        with pytest.raises(ContractViolation) as err:
            resolve_place_targets(target, gt)
        assert Reason.ATTACHMENT_GT_AS_MEASURED.value in str(err.value)
        provider = FakeAttachmentProvider([gt])
        with pytest.raises(ContractViolation):
            provider.latest_record("r")

    def test_pre_attach_estimate_never_grades_a_route(self):
        estimate = self._record(
            source=AttachmentSourceKind.MEASURED_PRE_ATTACH_ESTIMATE)
        target = BoxTarget(
            candidate_id="c", request_id="r", frame_id="world",
            scene_revision="s", heading_theta=0.0, position=(0, 0, 0),
            quaternion_xyzw=(0, 0, 0, 1), stamp_ns=1_000_000_000)
        with pytest.raises(ContractViolation) as err:
            resolve_place_targets(target, estimate)
        assert Reason.ATTACHMENT_SOURCE_UNQUALIFIED.value in str(err.value)
        # initial selection may use it explicitly
        targets = resolve_place_targets(
            target, estimate, require_attach_confirmed=False)
        assert len(targets) == 2  # symmetry pi: theta and theta+pi


class TestProviderFixtureSemantics:
    def test_fake_provider_roundtrip_and_invalidation(self):
        record = make_attachment("centered")
        provider = FakeAttachmentProvider([record])
        again = provider.latest_record(record.request_id)
        assert again.digest() == record.digest()
        note = provider.invalidate(Reason.ATTACHMENT_SLIP, {"observed_m": 0.03})
        assert note.incarnation == record.attachment_incarnation
        assert provider.invalidations == (note,)

    def test_record_digest_stability(self):
        assert make_attachment("centered").digest() == make_attachment("centered").digest()
        assert make_attachment("centered").digest() != make_attachment("eccentric").digest()


class TestPolicyValuesMirrorContractJson:
    def test_constants_match_resolved_contract(self):
        import json
        from pathlib import Path
        contract = json.loads(
            (Path(__file__).resolve().parents[3] / "src" / "luggage_planning"
             / "config" / "planning_contract" / "resolved_planning_contract.json").read_text())
        att = contract["attachment_policy"]
        assert att["tf_sync_max_skew_ns"]["value"] == TF_SYNC_MAX_SKEW_NS
        assert att["attachment_max_age_ns"]["value"] == ATTACHMENT_MAX_AGE_NS
        assert att["grasp_binding_tolerance_m"]["value"] == GRASP_TOLERANCE_M
        assert att["grasp_binding_tolerance_rad"]["value"] == GRASP_TOLERANCE_RAD
