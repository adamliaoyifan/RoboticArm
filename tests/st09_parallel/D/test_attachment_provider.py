"""ST09-D acceptance: the measured attachment provider.

Mirrors the frozen AttachmentProvider protocol semantics and the C0
nine-negative matrix: publish-once on valid observation + attach
confirmation, duplicate idempotency, incarnation on re-attach, and named
refusals for wrong-time TF, missing pose, invalid sources, slip and
unconfirmed attach — across the sim/deploy identity pair.
"""

import math
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from luggage_planning.attachment_provider import (
    MeasuredAttachmentProvider,
    MeasuredPose,
)

from st09_contracts._base import ContractViolation
from st09_contracts.attachment import AttachmentSourceKind, AttachmentValidity
from st09_contracts.fixtures import (
    BOX_WDH,
    DEPLOY_CALIBRATION,
    DEPLOY_SCENE,
    SIM_CALIBRATION,
    SIM_SCENE,
)
from st09_contracts.reasons import Reason

STAMP = 1_000_000_000
MEASURED_METHOD = "detection_pose+tcp_tf:same-stamp"

IDENTITIES = {
    "sim": (SIM_SCENE, SIM_CALIBRATION),
    "deploy": (DEPLOY_SCENE, DEPLOY_CALIBRATION),
}


def make_provider(identity="sim"):
    scene, calibration = IDENTITIES[identity]
    return MeasuredAttachmentProvider(scene, calibration)


def box_pose(position=(1.2, 0.1, 0.85), quaternion=(0, 0, 0, 1),
             stamp_ns=STAMP):
    return MeasuredPose("world", position, quaternion, stamp_ns)


def tcp_pose(position=(1.2, 0.1, 0.99), quaternion=(0, 0, 0, 1),
             stamp_ns=STAMP):
    return MeasuredPose("suction_contact_frame", position, quaternion,
                        stamp_ns)


def record(provider, request_id="req-1", box=None, tcp=None,
           confirmed=True, method=MEASURED_METHOD,
           kind=AttachmentSourceKind.MEASURED_ATTACH_CONFIRMED,
           dimensions=BOX_WDH, allow_pre_attach=False):
    return provider.record_attachment(
        request_id, box if box is not None else box_pose(),
        tcp if tcp is not None else tcp_pose(),
        attach_confirmed=confirmed, source_kind=kind, source_method=method,
        dimensions_wdh=dimensions, cargo_id="cargo-1", box_id="box-1",
        cycle_id="cycle-1", allow_pre_attach=allow_pre_attach)


def refusal_of(exc):
    return str(exc).split(":", 1)[0]


class TestPublication(object):
    def test_valid_observation_plus_confirmation_publishes_once(self):
        for identity in IDENTITIES:
            provider = make_provider(identity)
            first = record(provider)
            assert first.usable_for_route is True
            assert first.attachment_incarnation == 0
            assert first.source is AttachmentSourceKind.MEASURED_ATTACH_CONFIRMED
            scene, calibration = IDENTITIES[identity]
            assert first.scene_revision == scene
            assert first.calibration_id == calibration
            # exactly one record: the duplicate event is idempotent
            again = record(provider)
            assert again is first

    def test_transform_matches_independent_scipy(self):
        provider = make_provider()
        quaternion = Rotation.from_euler("xyz", (0.10, -0.05, 0.40)
                                         ).as_quat()
        tcp = tcp_pose(position=(0.9, -0.15, 0.77), quaternion=quaternion)
        box = box_pose(position=(1.05, -0.02, 0.62),
                       quaternion=Rotation.from_euler("xyz", (0.0, 0.0, 0.25)
                                                      ).as_quat())
        record(provider, box=box, tcp=tcp)
        rec = provider.latest_record("req-1")
        t_world_tcp = Transformation(tcp.position, tcp.quaternion_xyzw)
        t_world_box = Transformation(box.position, box.quaternion_xyzw)
        expect = t_world_tcp.inverse().compose(t_world_box)
        assert np.max(np.abs(np.asarray(rec.t_tcp_box_translation)
                             - np.asarray(expect.t))) <= 1e-9
        relative = (Rotation.from_quat(rec.t_tcp_box_quaternion_xyzw).inv()
                    * expect.rot)
        assert relative.magnitude() <= 1e-9

    def test_pre_attach_estimate_is_never_route_grade(self):
        provider = make_provider()
        estimate = record(provider, confirmed=False, allow_pre_attach=True,
                          kind=AttachmentSourceKind.MEASURED_PRE_ATTACH_ESTIMATE,
                          method="pre_attach_contact_estimate")
        assert estimate.source is \
            AttachmentSourceKind.MEASURED_PRE_ATTACH_ESTIMATE
        assert estimate.usable_for_route is False


class TestReattachAndSlip(object):
    def test_reattach_allocates_new_incarnation_and_invalidates(self):
        provider = make_provider()
        record(provider)
        later = STAMP + 5_000_000_000
        second = record(
            provider,
            box=box_pose((0.5, -0.3, 0.6), stamp_ns=later),
            tcp=tcp_pose((0.5, -0.3, 0.74), stamp_ns=later))
        assert second.attachment_incarnation == 1
        assert [str(note.reason) for note in provider.invalidations] == [
            Reason.ATTACHMENT_REATTACH.value]
        assert provider.latest_record("req-1").attachment_incarnation == 1

    def test_slip_recheck_invalidates_beyond_binding_tolerance(self):
        provider = make_provider()
        record(provider)
        assert provider.slip_recheck("req-1", box_pose(), tcp_pose()) is True
        drifted = tcp_pose(position=(1.35, 0.1, 0.99))
        assert provider.slip_recheck("req-1", box_pose(), drifted) is False
        reasons = [str(note.reason) for note in provider.invalidations]
        assert Reason.ATTACHMENT_SLIP.value in reasons
        try:
            provider.latest_record("req-1")
            raised = False
        except ContractViolation as exc:
            raised = True
            assert refusal_of(exc) == Reason.ATTACHMENT_MISSING.value
        assert raised

    def test_expiry_applies_the_placement_window(self):
        provider = make_provider()
        record(provider)
        provider.expire_stale("req-1", STAMP + 60_000_000_000)
        assert provider.latest_record("req-1")  # exactly at the window: valid
        provider.expire_stale("req-1", STAMP + 60_000_000_001)
        try:
            provider.latest_record("req-1")
            raised = False
        except ContractViolation:
            raised = True
        assert raised
        assert [str(n.reason) for n in provider.invalidations] == [
            Reason.ATTACHMENT_EXPIRED.value]


class TestNegatives(object):
    def test_wrong_time_tf_is_refused(self):
        provider = make_provider()
        try:
            record(provider, tcp=tcp_pose(stamp_ns=STAMP + 1))
            raised = False
        except ContractViolation as exc:
            raised = True
            assert refusal_of(exc) == Reason.ATTACHMENT_SOURCE_UNQUALIFIED.value
        assert raised

    def test_missing_pose_is_refused(self):
        provider = make_provider()
        for missing in ("box", "tcp"):
            try:
                provider.record_attachment(
                    "req-missing",
                    box_pose() if missing == "tcp" else None,
                    tcp_pose() if missing == "box" else None,
                    attach_confirmed=True,
                    source_kind=AttachmentSourceKind.MEASURED_ATTACH_CONFIRMED,
                    source_method=MEASURED_METHOD,
                    dimensions_wdh=BOX_WDH,
                    cargo_id="cargo-1", box_id="box-1", cycle_id="cycle-1")
                raised = False
            except ContractViolation as exc:
                raised = True
                assert refusal_of(exc) == Reason.ATTACHMENT_MISSING.value
            assert raised, missing

    def test_invalid_sources_are_refused(self):
        provider = make_provider()
        cases = [
            ("gt_kind", dict(kind=AttachmentSourceKind.REFERENCE_SIM_GT,
                             method="gt_relative_transform"),
             Reason.ATTACHMENT_GT_AS_MEASURED.value),
            ("gt_pull", dict(method="get_current_box_pull"),
             Reason.ATTACHMENT_SOURCE_UNQUALIFIED.value),
            ("planned_slot", dict(method="planned_slot_pose"),
             Reason.ATTACHMENT_SOURCE_UNQUALIFIED.value),
            ("pca_only", dict(method="pca_yaw_only_estimate"),
             Reason.ATTACHMENT_SOURCE_UNQUALIFIED.value),
            ("physics_lock", dict(method="gazebo_physics_weld"),
             Reason.ATTACHMENT_SOURCE_UNQUALIFIED.value),
        ]
        for label, overrides, expected in cases:
            try:
                record(provider, request_id="neg-%s" % label, **overrides)
                raised = False
            except ContractViolation as exc:
                raised = True
                assert refusal_of(exc) == expected, label
            assert raised, label

    def test_unconfirmed_attach_never_publishes(self):
        provider = make_provider()
        try:
            record(provider, confirmed=False)
            raised = False
        except ContractViolation as exc:
            raised = True
            assert refusal_of(exc) == Reason.ATTACHMENT_UNCONFIRMED.value
        assert raised
        assert provider.record_for("req-1") is None

    def test_invalid_dimensions_and_transform_are_refused(self):
        provider = make_provider()
        try:
            record(provider, dimensions=(0.0, 0.4, 0.3))
            raised = False
        except ContractViolation as exc:
            raised = True
            assert refusal_of(exc) == \
                Reason.ATTACHMENT_INVALID_DIMENSIONS.value
        assert raised
        try:
            record(provider, request_id="req-nan",
                   tcp=tcp_pose(quaternion=(0.0, 0.0, 0.0, 0.0)))
            raised = False
        except ContractViolation as exc:
            raised = True
            assert refusal_of(exc) == \
                Reason.ATTACHMENT_INVALID_TRANSFORM.value
        assert raised


class Transformation(object):
    """Independent scipy transform for cross-checking the frozen math."""

    def __init__(self, t, q_xyzw):
        self.t = np.asarray(t, dtype=float)
        self.rot = Rotation.from_quat(q_xyzw)

    def compose(self, other):
        return Transformation(
            self.t + self.rot.apply(other.t),
            (self.rot * other.rot).as_quat())

    def inverse(self):
        inv = self.rot.inv()
        return Transformation(inv.apply(-self.t), inv.as_quat())
