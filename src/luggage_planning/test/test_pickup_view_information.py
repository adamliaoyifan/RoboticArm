"""Information is compared on the original physical region, never bitmap totals."""
from dataclasses import asdict, replace

import numpy as np
import pytest

from luggage_planning.joint_operation.pickup_observation import PickupObservationNeed
from luggage_planning.joint_operation.view_information import compare_pickup_region
from pickup_continuation_fixtures import case_archive, fixture_policy, planar_case, planar_view


def original_need(domain, view, *, shift=0.):
    corners = ((-.05+shift, -.05, 1.), (.05+shift, -.05, 1.),
               (.05+shift, .05, 1.), (-.05+shift, .05, 1.))
    return PickupObservationNeed(domain.digest(), view.digest(), 'original-contact',
        (shift, 0., 1.), tuple(v for corner in corners for v in corner), (), 'detection_bbox', 16, 0, 0)


def compare(archive, case, **changes):
    proof = archive.associate(case)
    need = changes.pop('need', original_need(*case[:2]))
    arguments = dict(sample_shape=(4, 4)); arguments.update(changes)
    archive.save('information_inputs', dict(need=asdict(need), association=proof,
                                          policy=fixture_policy(), arguments=arguments))
    result = compare_pickup_region(need, case[1], case[3], proof, fixture_policy(), **arguments)
    archive.save('information_output', asdict(result))
    return result


def test_localized_missing_measurements_gain_on_identical_world_targets(case_archive):
    depth = np.full((16, 16), 1000, np.uint16); depth[6:10, 6:10] = 0
    case = (*planar_view(depth=depth), *planar_view(newer=True))
    digests = tuple(v.digest() for v in case)
    result = compare(case_archive, case)
    assert result.gained_samples == 16 and result.lost_samples == 0
    assert result.before_measured == (False,)*16 and result.after_measured == (True,)*16
    targets = np.asarray(result.targets_world).reshape(-1, 3)
    assert np.all(np.abs(targets[:, :2]) < .05) and np.all(targets[:, 2] == 1.)
    assert tuple(v.digest() for v in case) == digests


def test_unrelated_support_grid_growth_does_not_create_information_gain(case_archive):
    case = planar_case(domain_changes=dict(shape=(20, 20), support=bytes([1])*400,
                                           origin_uv_top_m=(-.5, -.5)))
    result = compare(case_archive, case)
    assert result.gained_samples == result.lost_samples == 0
    assert all(result.before_measured) and all(result.after_measured)


@pytest.mark.parametrize('missing', ['depth', 'region'])
def test_world_region_outside_image_or_unmeasured_does_not_gain(case_archive, missing):
    depth = np.full((16, 16), 1000, np.uint16)
    region = np.ones((16, 16), bool)
    if missing == 'depth': depth[6:10, 4:8] = 0
    else: region[6:10, 4:8] = False
    case = (*planar_view(), *planar_view(newer=True, depth=depth, region=region))
    result = compare(case_archive, case)
    assert result.gained_samples == 0 and result.lost_samples == 16


def test_unrelated_world_region_is_not_seen_just_because_global_bitmap_grows(case_archive):
    case = planar_case(domain_changes=dict(shape=(20, 20), support=bytes([1])*400,
                                           origin_uv_top_m=(-.5, -.5)))
    result = compare(case_archive, case, need=original_need(*case[:2], shift=2.))
    assert result.gained_samples == result.lost_samples == 0 and not any(result.after_measured)


@pytest.mark.parametrize('damage', ['source', 'view', 'policy', 'rejected'])
def test_information_cannot_be_authorized_by_an_unbound_or_rejected_match(case_archive, damage):
    case = planar_case(); proof = case_archive.associate(case)
    need = original_need(*case[:2]); policy = fixture_policy()
    if damage == 'source': need = replace(need, domain_sha256='0'*64)
    elif damage == 'view': need = replace(need, view_sha256='0'*64)
    elif damage == 'policy': policy = fixture_policy(depth_residual_m=.006)
    else: proof = replace(proof, accepted=False, reason='AMBIGUOUS_REPORTED_CANDIDATES')
    case_archive.save('refused_information_inputs', dict(need=asdict(need), proof=proof, policy=policy))
    with pytest.raises(ValueError, match='BINDING|ACCEPTED_MEASURED'):
        compare_pickup_region(need, case[1], case[3], proof, policy, sample_shape=(4, 4))


@pytest.mark.parametrize('shape', [(0, 4), (True, 4), (65, 65), (4,), (1, 1, 1)])
def test_explicit_information_work_bound(case_archive, shape):
    with pytest.raises(ValueError, match='BOUNDED_REGION_SAMPLE'):
        compare(case_archive, planar_case(), sample_shape=shape)


def test_same_targets_count_real_measurement_loss_and_digest_changes(case_archive):
    case = planar_case(); result = compare(case_archive, case)
    assert result.digest() == result.digest()
    assert result.digest() != replace(result, query_id='different-query').digest()
