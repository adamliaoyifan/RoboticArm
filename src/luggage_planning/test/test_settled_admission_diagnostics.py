"""Runtime diagnostics identify the exact checked map and preserve admission."""
from dataclasses import asdict
import json
from types import SimpleNamespace

import pytest

from test_placement_prior_requests import confirmed_grid, confirmed_volume
from luggage_mapping.source import OperationMapSource
from luggage_planning.joint_operation.runtime_ports import OperationRuntimePorts
from luggage_planning.placement_prior.contracts import Transform
from luggage_planning.placement_prior.requests import admit_placement_targets


I = Transform((0.,0.,0.), (0.,0.,0.,1.))


@pytest.mark.parametrize('state,expected', [(1, True), (0, False), (2, False)])
def test_runtime_records_actual_admission_without_motion_or_map_write(state, expected):
    volume = confirmed_volume()
    volume.states = bytes([state])*len(volume.states)
    source = OperationMapSource('scene','hull',2,297,2375,10,'a'*64,'ros',1,'b'*64)
    recorded = []
    host = SimpleNamespace(released=False, container=I,
        last_map=SimpleNamespace(source=source, surface=confirmed_grid(),volume=volume),
        record=lambda name,raw:recorded.append((name,json.loads(raw))))
    geometry = SimpleNamespace(measured_dimensions_wdh=(.2,.1,.2),digest=lambda:'geometry-bound')
    target = Transform((0.,0.,.35),(0.,0.,0.,1.))
    assert OperationRuntimePorts.settled_admission(host,None,geometry,target) is expected
    assert len(recorded)==1
    name, detail = recorded[0]
    assert name=='settled_box_admission.json'
    assert detail['map_source']==asdict(source)
    assert detail['geometry_digest']=='geometry-bound'
    assert detail['verdict']['accepted'] is expected
    assert detail['verdict']['nominal_dimensions_wdh']==[.2,.1,.2]
    assert detail['verdict']['center_container']==[0.,0.,.35]
    assert len(json.dumps(detail)) < 4096
    # No actuator, map writer, or RPC exists on the host; any such use fails.


@pytest.mark.parametrize('matches', [False,True])
def test_released_target_rule_is_recorded_without_replacing_it(matches):
    source = OperationMapSource('scene','hull',2,297,2375,10,'a'*64,'ros',1,'b'*64)
    recorded = []
    other = Transform((.001,0.,0.),I.quaternion_xyzw)
    host = SimpleNamespace(released=True,released_target=I if matches else other,
        last_map=SimpleNamespace(source=source),record=lambda name,raw:recorded.append(json.loads(raw)))
    geometry = SimpleNamespace(digest=lambda:'geometry-bound')
    assert OperationRuntimePorts.settled_admission(host,None,geometry,I) is matches
    assert recorded[0]['verdict']['policy']=='bound_released_target'
    assert recorded[0]['verdict']['reason'] == (None if matches else 'released_target_mismatch')


def test_recorder_failure_is_not_swallowed_or_turned_into_admission():
    def broken(_):
        raise ValueError('EVIDENCE_CAP')
    with pytest.raises(ValueError,match='EVIDENCE_CAP'):
        admit_placement_targets(confirmed_grid(),(.2,.1,.2),
            (Transform((0.,0.,.35),(0.,0.,0.,1.)),),volume=confirmed_volume(),on_verdict=broken)


def test_nonhorizontal_target_has_explicit_refusal():
    target = Transform((0.,0.,.35),(.01,0.,0.,(.9999)**.5))
    records = []
    assert admit_placement_targets(confirmed_grid(),(.2,.1,.2),(target,),
        volume=confirmed_volume(),on_verdict=records.append) == (False,)
    assert records[0]['reason']=='nonhorizontal_placement_target'
