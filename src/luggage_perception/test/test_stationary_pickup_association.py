"""Conditional correspondence from actual paired measurements, no identity shortcut."""
from dataclasses import FrozenInstanceError, replace
import json

import numpy as np
import pytest

from luggage_perception.stationary_pickup_association import (
    StationaryPickupAssociation, measure_pickup_world_targets,
)
from pickup_continuation_fixtures import (
    SETTLED_NS, candidate, case_archive, decode, fixture_policy, planar_case, planar_view,
)


def test_changed_id_unique_measured_match_has_lossless_replay(case_archive):
    case = planar_case(); original = tuple(v.digest() for v in case)
    proof = case_archive.associate(case)
    assert proof.accepted and proof.reason == 'CONDITIONAL_UNIQUE_STATIONARY_MATCH'
    assert case[0].instance_id != case[2].instance_id and case[0].generation != case[2].generation
    assert proof.candidates[0].joint_matches == 224 and proof.sampled_points == 256
    assert tuple(v.digest() for v in case) == original
    raw = decode(json.loads((case_archive.directory/'association_01_inputs.json').read_text()))
    assert raw['case'] == case
    from luggage_perception.stationary_pickup_association import associate_stationary_pickup
    replayed = associate_stationary_pickup(*raw['case'], **raw['arguments'])
    assert replayed == proof and replayed.digest() == proof.digest()
    with pytest.raises(FrozenInstanceError): proof.accepted = False
    with pytest.raises(ValueError): case[1].acquisition.association_inputs.rgb.array()[0, 0] = 0


@pytest.mark.parametrize('held', [False, True])
def test_every_reported_competitor_including_held_duplicate_is_considered(case_archive, held):
    proof = case_archive.associate(planar_case(candidates=(candidate(), candidate(held=held))))
    assert not proof.accepted and proof.reason == 'AMBIGUOUS_REPORTED_CANDIDATES'
    assert len(proof.candidates) == 2 and all(c.verdict == 'COMPATIBLE' for c in proof.candidates)


@pytest.mark.parametrize('change,reason', [
    ('rgb_missing', 'EXACT_ASSOCIATION_INPUTS_UNAVAILABLE'),
    ('depth_conflict', 'NO_UNIQUE_MEASURED_CORRESPONDENCE'),
    ('rgb_conflict', 'NO_UNIQUE_MEASURED_CORRESPONDENCE'),
    ('depth_missing', 'UNRESOLVED_REPORTED_CANDIDATE'),
    ('clip', 'INSUFFICIENT_PROJECTED_WORLD_OVERLAP'),
    ('outside', 'INSUFFICIENT_PROJECTED_WORLD_OVERLAP'),
    ('behind', 'INSUFFICIENT_PROJECTED_WORLD_OVERLAP'),
    ('selected', 'SELECTED_CANDIDATE_NOT_MATCHED'),
    ('model', 'ASSOCIATION_FRAME_OR_MODEL_CHANGED'),
    ('binding', 'ASSOCIATION_DOMAIN_VIEW_MISMATCH'),
    ('no_source', 'INSUFFICIENT_ORIGINAL_OBSERVED_POINTS'),
])
def test_missing_or_conflicting_evidence_never_turns_into_identity(case_archive, change, reason):
    options = {}
    if change == 'rgb_missing': options['with_rgb'] = False
    elif change == 'depth_conflict': options['depth'] = np.full((16, 16), 1100, np.uint16)
    elif change == 'rgb_conflict': options['rgb'] = np.zeros((16, 16, 3), np.uint8)
    elif change == 'depth_missing': options['depth'] = np.zeros((16, 16), np.uint16)
    elif change in ('clip', 'outside'): options['camera_x'] = .2 if change == 'clip' else 2.
    elif change == 'selected':
        options.update(candidates=(candidate(bbox=(0, 0, 1, 1)), candidate()), selected=0)
    elif change == 'model': options['domain_changes'] = dict(contact_model_hash='other-model')
    before_domain, before_view, after_domain, after_view = planar_case(**options)
    if change == 'binding': after_view = replace(after_view, domain_sha256='0'*64)
    elif change == 'no_source':
        before_domain = replace(before_domain, support=bytes(100))
        before_view = before_view.acquisition.bind(before_domain)
    elif change == 'behind':
        matrix = np.asarray(after_view.acquisition.world_from_optical).reshape(4, 4).copy()
        matrix[:3, 3] = (0., 0., 0.)  # Downward camera below the measured plane.
        after_view = replace(after_view, acquisition=replace(after_view.acquisition,
                             world_from_optical=tuple(matrix.ravel())))
    proof = case_archive.associate((before_domain, before_view, after_domain, after_view))
    assert not proof.accepted and proof.reason == reason


@pytest.mark.parametrize('settle', [0, 1_000_000_000, 3_000_000_000, 4_000_000_000, True])
def test_actual_acquisition_must_be_strictly_after_actual_settle(case_archive, settle):
    proof = case_archive.associate(planar_case(), settled_stamp_ns=settle)
    assert not proof.accepted and proof.reason == 'POST_SETTLE_NEW_ACQUISITION_REQUIRED'


def test_stationary_premise_and_source_stamp_cannot_be_assumed(case_archive):
    case = planar_case()
    proof = case_archive.associate(case, stationary=False)
    assert not proof.accepted and proof.reason == 'STATIONARY_PREMISE_REQUIRED'
    domain, view = case[2:]
    domain = replace(domain, stamp_ns=domain.stamp_ns+1)
    proof = case_archive.associate((*case[:2], domain, view))
    assert not proof.accepted and proof.reason == 'ASSOCIATION_DOMAIN_VIEW_MISMATCH'


def test_measured_conflicting_competitor_can_be_excluded_and_sample_bound_is_explicit(case_archive):
    # This candidate's actual bbox has insufficient world overlap, unlike a duplicate.
    proof = case_archive.associate(planar_case(candidates=(candidate(), candidate(bbox=(0, 0, 2, 2)))),
                                   policy=fixture_policy(max_samples=32))
    assert proof.accepted and proof.sampled_points == 32
    assert proof.candidates[1].verdict == 'INCOMPATIBLE_WORLD_OVERLAP'


def test_one_match_with_unresolved_competitor_is_not_unique(case_archive):
    depth = np.full((16, 16), 1000, np.uint16); depth[:2, :12] = 0
    proof = case_archive.associate(planar_case(camera_x=0., depth=depth,
        candidates=(candidate(), candidate(bbox=(0, 0, 14, 16), held=True))))
    assert not proof.accepted and proof.reason == 'UNRESOLVED_REPORTED_CANDIDATE'
    assert [c.verdict for c in proof.candidates] == ['COMPATIBLE', 'UNRESOLVED_MEASUREMENTS']


@pytest.mark.parametrize('values', [dict(depth_residual_m=0.), dict(depth_residual_m=float('nan')),
    dict(rgb_l2_difference=-1.), dict(rgb_l2_difference=500.), dict(bbox_margin_pixels=-1.),
    dict(min_paired_points=2), dict(min_fraction=0.), dict(min_fraction=1.01),
    dict(max_samples=5), dict(max_samples=65537), dict(provenance=''), dict(min_fraction=True)])
def test_policy_is_explicit_named_and_bounded(case_archive, values):
    # JSON deliberately represents the nonfinite test value as text, never a measurement.
    case_archive.save('policy_rejection_input', {k: str(v) for k, v in values.items()})
    with pytest.raises(ValueError, match='EXPLICIT_STATIONARY'): fixture_policy(**values)


def test_policy_and_raw_measurements_are_digest_bound(case_archive):
    case = planar_case(); first = case_archive.associate(case)
    second = case_archive.associate(case, policy=fixture_policy(rgb_l2_difference=21.))
    assert second.accepted and first.policy_sha256 != second.policy_sha256
    assert first.digest() != second.digest()
    view = case[3]; inputs = view.acquisition.association_inputs
    rgb = inputs.rgb.array().copy(); rgb[0, 0, 0] ^= 1
    changed = replace(view, acquisition=replace(view.acquisition, association_inputs=replace(inputs,
                      rgb=replace(inputs.rgb, rgb8=rgb.tobytes()))))
    assert changed.digest() != view.digest()
    case_archive.save('changed_view', changed)
    with pytest.raises(ValueError, match='UNIQUE_SELECTED'):
        replace(first, candidates=())


def test_projection_reports_raw_region_without_inventing_face_support(case_archive):
    region = np.ones((16, 16), bool); region[7:9, 7:9] = False
    domain, view = planar_view(region=region)
    case_archive.save('projection_input', dict(domain=domain, view=view, target=((0., 0., 1.),)))
    values = measure_pickup_world_targets(view, ((0., 0., 1.),))
    case_archive.save('projection_output', {k: None if v is None else v.tolist() for k, v in values.items()})
    assert values['depth_valid'].tolist() == [True] and values['in_region'].tolist() == [False]
    assert domain.cells().all()  # Neither the projection nor matching fills/mutates support.
