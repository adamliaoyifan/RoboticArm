"""Assembly behavior with explicit port doubles; no physical FK certificate."""
from dataclasses import replace
import hashlib
import math

import pytest

from luggage_planning.joint_operation_connection import GeometricConnection
from luggage_planning.joint_operation_contracts import admit_operation_plan, operation_digest
from luggage_planning.joint_operation_planner import (
    JointOperationPlanner, LocalEndpointEvidence, LocalJointBranch,
    OperationCandidate, OperationCertification,
)
from test_joint_operation_contracts import example_plan
from test_joint_operation_scenes import baseline, compose


def joints(value, j6=None):
    return (value,)*5+(value if j6 is None else j6,)


def branch(name, value, *, j6=None):
    return LocalJointBranch(name, (joints(value, j6), joints(value+.005, j6)),
        (joints(value+.005, j6), joints(value+.01, j6)))


def fixture():
    plan = example_plan()
    scenes = compose(baseline())
    identity = replace(plan.identity,
        snapshot_id=hashlib.sha256(repr(scenes.base_identity).encode()).hexdigest())
    candidate = OperationCandidate('candidate', plan.world_from_pick_box,
        plan.world_from_pick_tool, plan.world_from_place_box, plan.world_from_place_tool,
        plan.world_from_settled_box, plan.predicted_attachment, scenes,
        plan.world_from_pick_tool, plan.world_from_pick_tool,
        plan.world_from_place_tool, plan.world_from_place_tool)
    return identity, candidate


class Clock:
    now = 10.
    def __call__(self):
        return self.now


class Endpoints:
    def __init__(self, identity, candidate):
        self.calls = []
        self.evidence = LocalEndpointEvidence(candidate.candidate_id, identity.model_sha256,
            candidate.scenes.digests, candidate.predicted_attachment.digest(), True, 'fixture',
            (branch('p0', .01), branch('p1', .04, j6=6.4)), (branch('s0', .08, j6=6.5),))
    def solve(self, candidate, **kwargs):
        self.calls.append((candidate, kwargs))
        return self.evidence


class Connections:
    def __init__(self, fail_first_carry=False):
        self.calls = []
        self.fail_first_carry = fail_first_carry
        self.alter = lambda result: result
    def connect(self, snapshot, start, goal, **kwargs):
        self.calls.append((snapshot, start, goal, kwargs))
        if self.fail_first_carry and kwargs['request_id'].endswith(':p0:s0:carry'):
            return GeometricConnection(False, False, 'search exhausted')
        n = max(1, math.ceil(max(abs(a-b) for a,b in zip(start,goal))/.009))
        rows = (start,)+tuple(tuple(a+(b-a)*i/n for a,b in zip(start,goal))
                             for i in range(1,n))+(goal,)
        return self.alter(GeometricConnection(True, True, 'fixture', rows, len(rows)))


class Certification:
    def __init__(self):
        self.calls = []
        self.alter = lambda receipt: receipt
    def certify(self, draft, candidate, **kwargs):
        self.calls.append((draft, candidate, kwargs))
        return self.alter(OperationCertification(draft.identity, draft.candidate_id,
            operation_digest(draft), candidate.scenes.digests, True, 'fixture only', 'fixture-cert'))


def ports(**kwargs):
    identity, candidate = fixture()
    ep, connection, cert, clock = Endpoints(identity, candidate), Connections(), Certification(), Clock()
    planner = JointOperationPlanner(ep, connection, cert, velocity_scale=.2,
        acceleration_scale=.1, certification_reserve_s=.2, clock=clock, **kwargs)
    return identity, candidate, ep, connection, cert, clock, planner


def prepare(bundle, **kwargs):
    identity, candidate, _, _, _, _, planner = bundle
    return planner.prepare(identity, joints(0.), (candidate,), budget_s=5., **kwargs)


def test_failed_carry_tries_another_pick_branch_and_preserves_full_joint6_winding():
    bundle = ports()
    identity, candidate, ep, connection, cert, _, _ = bundle
    connection.fail_first_carry = True
    result = prepare(bundle)
    assert result.status == 'PLAN_READY'
    plan = result.plan
    admit_operation_plan(plan, identity, plan.plan_digest)
    assert len(ep.calls) == 1 and len(connection.calls) == 4 and len(cert.calls) == 1
    assert [c[0] for c in connection.calls] == [candidate.scenes.pick, candidate.scenes.carry]*2
    assert plan.legs[1].knots[-1].positions[-1] == 6.4
    assert plan.legs[4].knots[-1].positions[-1] == 6.5
    assert tuple(leg.phase for leg in plan.legs) == ('pick', 'pick', 'carry', 'carry', 'carry', 'retreat')
    assert tuple(b.before_leg_index for b in plan.barriers) == (2, 5)
    assert all(a.time_from_start_ns < b.time_from_start_ns for leg in plan.legs
               for a,b in zip(leg.knots, leg.knots[1:]))
    assert plan.world_from_settled_box == candidate.world_from_settled_box
    assert plan.certificate_id == 'fixture-cert'
    assert cert.calls[0][0].certificate_id == 'pending-native-certification'
    assert cert.calls[0][2]['deadline'] > connection.calls[0][3]['deadline']


@pytest.mark.parametrize('field,value', [('complete',False), ('model_sha256','stale'),
    ('candidate_id','other'), ('phase_scene_digests',('stale',)*3), ('attachment_digest','stale'),
    ('pick_branches',()), ('place_branches',())])
def test_unbound_or_incomplete_local_evidence_never_connects(field,value):
    bundle = ports()
    ep, connection, cert = bundle[2:5]
    ep.evidence = replace(ep.evidence, **{field:value})
    result = prepare(bundle)
    assert result.status == 'SEARCH_INCOMPLETE' and result.plan is None
    assert not connection.calls and not cert.calls


@pytest.mark.parametrize('field,value', [('passed',False), ('certificate_id',''),
    ('candidate_id','other'), ('draft_digest','stale'), ('phase_scene_digests',('stale',)*3)])
def test_bad_certification_returns_no_partial_plan(field,value):
    bundle = ports()
    cert = bundle[4]
    cert.alter = lambda receipt: replace(receipt, **{field:value})
    result = prepare(bundle)
    assert result.status == 'SEARCH_INCOMPLETE' and result.plan is None
    assert cert.calls and any(row[1] == 'WHOLE_CHAIN_CERTIFICATION_REFUSED' for row in result.trace)


def test_missing_certificate_or_wrong_independent_identity_refuses():
    for alter in (lambda receipt: None,
                  lambda receipt: replace(receipt, identity=replace(receipt.identity, map_revision=8))):
        bundle = ports()
        bundle[4].alter = alter
        assert prepare(bundle).plan is None


@pytest.mark.parametrize('field,value', [('snapshot_id','stale'), ('model_sha256','other'),
                                       ('geometry_hash','other'), ('map_revision',8)])
def test_candidate_must_match_pinned_request_before_local_search(field,value):
    identity, candidate, ep, connection, cert, _, planner = ports()
    result = planner.prepare(replace(identity, **{field:value}), joints(0.), (candidate,), budget_s=5.)
    assert result.plan is None and not ep.calls and not connection.calls and not cert.calls


def test_deadline_after_native_call_discards_success_and_respects_cancel():
    bundle = ports()
    _, _, _, connection, cert, clock, _ = bundle
    def late(result):
        clock.now += 5.
        return result
    connection.alter = late
    assert prepare(bundle).plan is None
    assert not cert.calls
    cancelled = ports()
    assert prepare(cancelled, cancelled=lambda: True).status == 'CANCELLED'
    assert not cancelled[2].calls


def test_deadline_after_certificate_never_returns_ready():
    bundle = ports()
    def late(receipt):
        bundle[5].now += 5.
        return receipt
    bundle[4].alter = late
    result = prepare(bundle)
    assert result.plan is None and result.status == 'SEARCH_INCOMPLETE'


def test_branch_budget_bounds_failed_connections_and_alignment_is_cached():
    bundle = ports(max_branch_attempts=2)
    ep, connection = bundle[2:4]
    ep.evidence = replace(ep.evidence, place_branches=(branch('s0',.08), branch('s1',.12)))
    connection.fail_first_carry = True
    original = connection.connect
    def fail_carry(snapshot,start,goal,**kwargs):
        response = original(snapshot,start,goal,**kwargs)
        return GeometricConnection(False,False,'fixture miss') if kwargs['request_id'].endswith(':carry') else response
    connection.connect = fail_carry
    result = prepare(bundle)
    assert result.plan is None
    assert len(connection.calls) == 3  # one cached alignment, two carrying attempts
    assert result.trace[-1][1] == 'BRANCH_CAP_OR_DEADLINE'


def test_release_world_obstacle_failure_from_certifier_refuses_entire_plan():
    bundle = ports()
    def blocked(receipt):
        assert bundle[4].calls[-1][1].scenes.retreat != bundle[4].calls[-1][1].scenes.carry
        return replace(receipt, passed=False, reason='released box blocks retreat')
    bundle[4].alter = blocked
    assert prepare(bundle).plan is None


def test_connection_endpoint_change_and_undersampling_do_not_reach_certifier():
    for alter in (lambda r: replace(r,positions=(joints(1.),)+r.positions[1:]),
                  lambda r: replace(r,positions=(r.positions[0],joints(2.),r.positions[-1]))):
        bundle=ports()
        bundle[3].alter=alter
        assert prepare(bundle).plan is None and not bundle[4].calls


def test_failed_pick_alignment_skips_its_place_alternatives():
    bundle=ports(max_branch_attempts=2)
    ep,connection=bundle[2:4]
    ep.evidence=replace(ep.evidence,place_branches=(branch('s0',.08),branch('s1',.12)))
    original=connection.connect
    def miss(snapshot,start,goal,**kwargs):
        response=original(snapshot,start,goal,**kwargs)
        return GeometricConnection(False,False,'align miss') if ':p0:' in kwargs['request_id'] else response
    connection.connect=miss
    assert prepare(bundle).status=='PLAN_READY'
    assert len(connection.calls)==3
    assert all(':p0:s1:' not in c[3]['request_id'] for c in connection.calls)
