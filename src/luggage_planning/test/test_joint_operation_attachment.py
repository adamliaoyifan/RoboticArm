"""Provider-side two-stamp freeze, immutability and legacy contract gates."""
from dataclasses import replace
import pytest

from luggage_planning.attachment_provider import MeasuredAttachmentProvider, MeasuredPose
from st09_contracts._base import ContractViolation
from st09_contracts.attachment import AttachmentSourceKind, ATTACHMENT_MAX_AGE_NS
from st09_contracts.attachment_geometry import freeze_geometry
from st09_contracts.payload_envelope import EnvelopeLimits
from st09_contracts.reasons import Reason
from st09_contracts.stationary_attachment import StationaryAttachmentEvidence, measured_pose_digest


@pytest.fixture
def source():
    provider = MeasuredAttachmentProvider('scene', 'calibration')
    box = MeasuredPose('world', (0., 0., 1.), (0., 0., 0., 1.), 1_000_000_000)
    tcp = MeasuredPose('suction_contact_frame', (0., 0., 1.05), (1., 0., 0., 0.), 2_000_000_000)
    geometry = freeze_geometry((.2, .15, .1), (0., 0., .05), (1., 0., 0., 0.),
                              EnvelopeLimits(5., (.1, .1, .1), (.01, .02), .005))
    proof = StationaryAttachmentEvidence('request', 'box', 3, 'scene', box.stamp_ns,
        tcp.stamp_ns, measured_pose_digest(box), measured_pose_digest(tcp),
        'a'*64, 'b'*64, 'c'*64, 'd'*64, geometry.digest())
    return provider, box, tcp, geometry, proof


def freeze(source, **changes):
    provider, box, tcp, geometry, proof = source
    args = dict(evidence=proof, conservative_geometry=geometry,
                now_ns=tcp.stamp_ns, cycle_id='cycle')
    args.update(changes)
    return provider.record_stationary_attachment('request', box, tcp, **args)


def test_distinct_stamps_are_preserved_without_extending_age(source):
    provider, box, tcp, geometry, proof = source
    record = freeze(source)
    assert record.source.route_grade and record.acquisition_stamp_ns == box.stamp_ns
    assert provider.stationary_evidence_for('request').tcp_confirmation_stamp_ns == tcp.stamp_ns
    assert provider.stationary_evidence_for('request') == proof
    assert provider.generation_for('request') == 3
    assert record.evidence_digest == proof.digest()
    assert freeze(source) is record and record.attachment_incarnation == 0
    provider.expire_stale('request', box.stamp_ns + ATTACHMENT_MAX_AGE_NS + 1)
    with pytest.raises(ContractViolation): provider.latest_record('request')
    with pytest.raises(ContractViolation, match='SEALED_ATTACHMENT_REBIND_REFUSED'): freeze(source)


@pytest.mark.parametrize('field,value', [
    ('request_id', 'other'), ('box_id', 'other'), ('generation', 4),
    ('scene_revision', 'other'), ('frozen_binding_sha256', 'd'*64),
    ('actual_footprint_sha256', 'd'*64), ('vacuum_response_sha256', 'e'*64),
    ('vacuum_request_sha256', 'e'*64),
    ('geometry_sha256', 'd'*64), ('box_pose_sha256', 'd'*64), ('tcp_pose_sha256', 'd'*64),
])
def test_sealed_relation_cannot_be_rebound(source, field, value):
    provider, _, _, _, proof = source
    original = freeze(source)
    with pytest.raises(ContractViolation): freeze(source, evidence=replace(proof, **{field:value}))
    assert provider.latest_record('request') is original
    assert provider.stationary_evidence_for('request') == proof


def test_legacy_entry_cannot_overwrite_stationary_seal(source):
    provider, box, tcp, geometry, _ = source
    original = freeze(source)
    same_stamp_tcp = MeasuredPose(tcp.frame, tcp.position, tcp.quaternion_xyzw, box.stamp_ns)
    with pytest.raises(ContractViolation, match='SEALED_ATTACHMENT_REBIND_REFUSED'):
        provider.record_attachment('request', box, same_stamp_tcp, attach_confirmed=True,
            source_kind=AttachmentSourceKind.MEASURED_ATTACH_CONFIRMED, source_method='measured',
            dimensions_wdh=geometry.measured_dimensions_wdh, cargo_id='box',box_id='box',cycle_id='cycle')
    assert provider.latest_record('request') is original


@pytest.mark.parametrize('now_ns', [0, 1_999_999_999, 61_000_000_001])
def test_future_or_expired_acquisition_does_not_freeze(source, now_ns):
    with pytest.raises(ContractViolation, match='ACQUISITION_NOT_CURRENT'): freeze(source,now_ns=now_ns)
    assert source[0].record_for('request') is None


@pytest.mark.parametrize('changes', [dict(box_acquisition_stamp_ns=0),
    dict(tcp_confirmation_stamp_ns=999_999_999), dict(frozen_binding_sha256='bad'),
    dict(generation=True)])
def test_invalid_two_stamp_proof_is_rejected(source, changes):
    with pytest.raises(ContractViolation): replace(source[-1],**changes)


def test_legacy_unequal_stamps_still_refused(source):
    provider, box, tcp, geometry, _ = source
    with pytest.raises(ContractViolation, match='not same-stamp'):
        provider.record_attachment('request',box,tcp,attach_confirmed=True,
            source_kind=AttachmentSourceKind.MEASURED_ATTACH_CONFIRMED,source_method='measured',
            dimensions_wdh=geometry.measured_dimensions_wdh,cargo_id='box',box_id='box',cycle_id='cycle')
    assert provider.record_for('request') is None
