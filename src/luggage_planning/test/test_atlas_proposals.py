"""Paired proposal/consumer behavior; ports here are deterministic test doubles.

This proves storage, transforms, branch continuation and fallback, not physical
IK/collision validity. Production ports must bind the current carrying scene.
"""
from dataclasses import asdict, replace
import json
import math
import os

import numpy as np
import pytest
import yaml

from luggage_planning.atlas_frame_contracts import atlas_contract_record, payload_hash
from luggage_planning.atlas_proposals import (
    AtlasProposalReader, AtlasQualification, artifact_digest, refine_atlas_pairs,
)
from luggage_planning.joint_operation_connection import GeometricConnection
from luggage_planning.placement_prior_contracts import Transform


def artifact(tmp_path, *, mutate=None, meta_mutate=None, payload=None):
    shape = (1, 1, 1, 4)
    data = dict(status=np.full(shape, 3, np.uint8),
                opening_connected=np.ones(shape, bool),
                contact_ik=np.ones(shape, bool), transit_ik=np.ones(shape, bool),
                solution_count=np.full(shape, 3, np.uint8),
                joint_margin=np.full(shape, .2), manipulability=np.full(shape, .3),
                neighbor_confidence=np.full(shape, .9), lift_clearance=np.full(shape, .25),
                branch_kind=np.full(shape, 2, np.uint8),
                contact_seeds=np.zeros(shape+(3, 6)), transit_seeds=np.zeros(shape+(3, 6)),
                transit_target=np.zeros(shape+(3, 3)), transit_lift=np.full(shape+(3,), np.nan))
    for index in range(3):
        data['contact_seeds'][..., index, :] = index+1
        data['transit_seeds'][..., index, :] = -index-1
        data['transit_target'][..., index, :] = (.2+index*.1, -.4, 1.5)
    bins = [0., math.pi/2, math.pi, 3*math.pi/2]
    payload = payload if payload is not None else {'enabled': False}
    phash = payload_hash(payload)
    meta = dict(schema_version=3, grid=dict(frame='container_link',
                resolution_xyz=.15, origin=[0., 0., .5], size=[1, 1, 1], yaw_bins=bins),
                dependencies=dict(scene_tf_hash='scene', urdf_hash='urdf',
                ik_link='suction_contact_frame', ik_group='elfin_arm'), payload=payload,
                contract=atlas_contract_record(bins, 'scene', 'urdf', 'geometry', phash))
    if mutate:
        mutate(data)
    if meta_mutate:
        meta_mutate(meta)
    npz, metadata = tmp_path/'atlas.npz', tmp_path/'atlas.yaml'
    np.savez(npz, **data)
    metadata.write_text(yaml.safe_dump(meta))
    qualification = AtlasQualification(artifact_digest(npz.read_bytes(), metadata.read_bytes()),
        'scene', 'urdf', 'geometry', phash, 'test-fixture-only')
    reader = AtlasProposalReader(npz, metadata, qualification)
    return reader, qualification, npz, metadata


def query(reader, **kwargs):
    return reader.query(.08, .09, .61, **kwargs)


def test_sample_and_exact_query_positions_are_distinct_and_pairs_are_not_flattened(tmp_path):
    reader, source, *_ = artifact(tmp_path)
    result = query(reader)
    assert result.source == source and result.reason == 'reachable'
    assert result.query_xyz_container == (.08, .09, .61)
    assert result.sample_xyz_container == (.075, .075, .575)
    assert [b.storage_index for b in result.branches] == [0, 1, 2]
    assert [b.contact_joints[0] for b in result.branches] == [1., 2., 3.]
    assert [b.transit_joints[0] for b in result.branches] == [-1., -2., -3.]
    assert result.branches[1].transit_xyz_container == pytest.approx((.3, -.4, 1.5))
    assert result.branches[1].transit_lift_m is None  # predecessor, not vertical lift
    assert result.cell_branch_kind == 2 and not result.payload.enabled
    assert result.edge_evidence == 'MISSING_REQUIRES_ONLINE_CONNECTION'
    assert result.joint_margin_rad == pytest.approx(.2)
    assert result.manipulability == pytest.approx(.3)


def test_payload_sampling_geometry_is_preserved_but_does_not_bypass_current_check(tmp_path):
    payload = dict(enabled=True, shape='box', size=[.55, .4, .25],
                   attach_link='suction_contact_frame', offset=[.01, 0., -.125],
                   touch_links=['suction_panel'])
    reader, *_ = artifact(tmp_path, payload=payload)
    proposals = query(reader)
    assert proposals.payload.enabled and proposals.payload.size == (.55, .4, .25)
    assert proposals.payload.offset == (.01, 0., -.125)
    assert proposals.payload.touch_links == ('suction_panel',)
    result, *_ = consume(proposals, failure='payload')
    assert len(result.pairs) == 2  # first transit rejected despite sampled overlay


def test_nan_seed_does_not_shift_second_branch_to_first_transit(tmp_path):
    reader, *_ = artifact(tmp_path, mutate=lambda d: d['contact_seeds'].__setitem__(
        (0, 0, 0, 0, 0, 0), np.nan))
    result = query(reader)
    assert [b.storage_index for b in result.branches] == [1, 2]
    assert result.branches[0].transit_joints[0] == -2.


def test_all_zero_joint_solution_is_not_assumed_padding(tmp_path):
    reader, *_ = artifact(tmp_path, mutate=lambda d: d['contact_seeds'].__setitem__(
        (0, 0, 0, 0, 0, slice(None)), 0.))
    assert query(reader).branches[0].contact_joints == (0.,)*6


@pytest.mark.parametrize('field', ['transit_target', 'transit_seeds'])
def test_missing_transit_provenance_retains_contact_only(tmp_path, field):
    reader, *_ = artifact(tmp_path, mutate=lambda d: d.pop(field))
    result = query(reader)
    assert len(result.branches) == 3
    assert all(not b.transit_joints and b.reason == 'CONTACT_ONLY' for b in result.branches)


@pytest.mark.parametrize('change', ['unknown', 'zero_count', 'bad_count'])
def test_unknown_and_invalid_counts_never_create_padded_branches(tmp_path, change):
    def mutate(data):
        if change == 'unknown':
            data['status'].fill(0)
        else:
            data['solution_count'].fill(0 if change == 'zero_count' else 4)
    reader, *_ = artifact(tmp_path, mutate=mutate)
    assert not query(reader).branches


def test_missing_qualification_or_changed_bytes_never_supplies_seeds(tmp_path):
    reader, source, npz, meta = artifact(tmp_path)
    assert not query(AtlasProposalReader(npz, meta)).branches
    assert query(AtlasProposalReader(npz, meta)).reason == 'ATLAS_UNQUALIFIED'
    meta.write_text(meta.read_text()+'\n# changed bytes\n')
    assert query(AtlasProposalReader(npz, meta, source)).reason == 'ATLAS_ARTIFACT_MISMATCH'
    assert len(query(reader).branches) == 3  # reader owns original bytes


@pytest.mark.parametrize('kind', ['dependency', 'contract', 'payload', 'grid_yaw', 'grid_size', 'grid_origin'])
def test_metadata_identity_mismatch_rejects_even_with_matching_file_digest(tmp_path, kind):
    def change(meta):
        if kind == 'dependency':
            meta['dependencies']['urdf_hash'] = 'other'
        elif kind == 'contract':
            meta['contract']['reference'] = 'other_tcp'
        elif kind == 'payload':
            meta['payload']['enabled'] = True
        elif kind == 'grid_yaw':
            meta['grid']['yaw_bins'] = [0., 0., 0., 0.]
        elif kind == 'grid_size':
            meta['grid']['size'] = [1.5, 1, 1]
        else:
            meta['grid']['origin'] = [0., 0., float('nan')]
    reader, *_ = artifact(tmp_path, meta_mutate=change)
    assert not query(reader).branches
    assert reader.reason != 'QUALIFIED'


def test_bad_metadata_type_is_unknown_not_a_loader_crash(tmp_path):
    _, source, npz, meta = artifact(tmp_path)
    meta.write_text('not a metadata mapping\n')
    source = replace(source, artifact_sha256=artifact_digest(npz.read_bytes(), meta.read_bytes()))
    reader = AtlasProposalReader(npz, meta, source)
    assert query(reader).reason == 'ATLAS_INVALID_METADATA' and not query(reader).branches


def test_directed_pi_bins_and_query_misses_are_not_pi_folded(tmp_path):
    reader, *_ = artifact(tmp_path)
    assert query(reader, yaw=math.pi).cell[-1] == 2
    assert query(reader, yaw=-math.pi).cell[-1] == 2
    assert query(reader, yaw=math.pi/2).cell[-1] == 1
    assert not query(reader, yaw=.4).branches
    assert not reader.query(10., 0., .6).branches


def consume(proposals, *, source=None, failure='', expire_stage='', cancel=False):
    calls = []
    time = [0.]
    exact = Transform((.081, .092, .618), (1., 0., 0., 0.))
    world_container = Transform((1., 2., 3.), (0., 0., math.sqrt(.5), math.sqrt(.5)))
    def refine(pose, seed, **kwargs):
        calls.append(('refine', pose, seed))
        if (failure == 'contact' and seed[0] == 1.) or (failure == 'transit' and seed[0] == -1.):
            return None
        if expire_stage == 'refine': time[0] = 2.
        return (seed[0]*.001,)*6
    def check(q, **kwargs):
        calls.append(('check', q))
        if expire_stage == 'check': time[0] = 2.
        return not (failure == 'payload' and q[0] == -.001)
    def connect(a, b, **kwargs):
        calls.append(('connect', a, b))
        if expire_stage == 'connect': time[0] = 2.
        ok = not (failure == 'connection' and a[0] == -.001)
        positions = (a, b) if ok else ()
        if failure == 'malformed': positions = (b, a)
        return GeometricConnection(ok, ok, 'test', positions, 2)
    result = refine_atlas_pairs(proposals, expected_source=source or proposals.source,
        exact_contact_world=exact, world_from_container=world_container,
        refine=refine, check_carrying=check, connect=connect,
        deadline=1., cancelled=lambda: cancel, clock=lambda: time[0])
    if os.environ.get('ATLAS_PAIR_CASE_LOG'):
        def encode(value):
            return asdict(value) if hasattr(value, '__dataclass_fields__') else value
        with open(os.environ['ATLAS_PAIR_CASE_LOG'], 'a', encoding='utf-8') as stream:
            stream.write(json.dumps(dict(proposals=asdict(proposals), failure=failure,
                expire_stage=expire_stage, cancel=cancel, exact_contact=asdict(exact),
                world_container=asdict(world_container), calls=calls, result=asdict(result)),
                default=encode, allow_nan=False)+'\n')
    return result, calls, exact, world_container


@pytest.mark.parametrize('failure', ['contact', 'transit', 'payload', 'connection'])
def test_failed_first_branch_continues_to_other_pairs(tmp_path, failure):
    reader, *_ = artifact(tmp_path)
    result, calls, exact, container = consume(query(reader), failure=failure)
    assert len(result.pairs) == 2 and not result.fallback_required
    assert ':1' == result.pairs[0].branch_id[-2:]
    assert result.pairs[0].contact_joints == (.002,)*6
    assert result.pairs[0].transit_joints == (-.002,)*6
    assert calls[0][1] == exact  # exact off-centre, not the grid sample
    assert result.pairs[0].transit_world.translation == pytest.approx((1.4, 2.3, 4.5))
    assert result.pairs[0].transit_world != exact  # predecessor's actual target retained
    assert any(c[0] == 'check' and c[1] == (.002,)*6 for c in calls)


def test_missing_target_still_refines_contacts_for_online_fallback(tmp_path):
    reader, *_ = artifact(tmp_path, mutate=lambda d: d.pop('transit_target'))
    result, calls, *_ = consume(query(reader))
    assert result.fallback_required and not result.pairs and len(result.contact_options) == 3
    assert not any(c[0] == 'connect' for c in calls)


def test_stale_source_cannot_call_any_port(tmp_path):
    reader, source, *_ = artifact(tmp_path)
    result, calls, *_ = consume(query(reader), source=replace(source, validation_id='other'))
    assert result.reason == 'ATLAS_SOURCE_MISMATCH' and not calls


@pytest.mark.parametrize('stage', ['refine', 'check', 'connect'])
def test_deadline_discards_partial_results_and_does_not_continue(tmp_path, stage):
    reader, *_ = artifact(tmp_path)
    result, calls, *_ = consume(query(reader), expire_stage=stage)
    assert result.reason == 'CANCELLED_OR_DEADLINE' and not result.pairs
    assert not result.contact_options
    assert [c[0] for c in calls] == {
        'refine': ['refine'], 'check': ['refine', 'check'],
        'connect': ['refine', 'check', 'refine', 'check', 'connect']}[stage]


def test_cancelled_and_malformed_connection_do_not_yield_pair(tmp_path):
    reader, *_ = artifact(tmp_path)
    result, calls, *_ = consume(query(reader), cancel=True)
    assert not result.pairs and not calls
    result, *_ = consume(query(reader), failure='malformed')
    assert not result.pairs and result.fallback_required
