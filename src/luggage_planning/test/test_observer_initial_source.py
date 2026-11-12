from dataclasses import replace
from types import SimpleNamespace
import json
import pytest
from test_joint_operation_observer import configuration, identity, columns_for
from luggage_mapping.observer import assemble_snapshot, declaration_only, lease_age_admitted
from luggage_mapping.source import OperationMapSource, map_source_to_message, map_source_from_message


def test_initial_condition_has_distinct_source_and_not_sensor_freshness(tmp_path):
    config,_,_=configuration(tmp_path)
    source=identity(config)
    source.update(observed_revision=0,last_integrated_stamp_ns=0,initialization_id='empty-session',
                  declared_by='operator',initialization_stamp_ns=10)
    snapshot=assemble_snapshot(config,source,2,columns_for(config,[1]*64),[1]*64,scene_revision='scene',request_id='init')
    record=snapshot.surface.record()
    assert record['time_basis']=='scene' and record['stamp_ns']==10
    assert set(v for row in record['confidence'] for v in row)=={'initial_empty'}
    assert map_source_from_message(map_source_to_message(snapshot.source))==snapshot.source
    lease=SimpleNamespace(identity=SimpleNamespace(**source),age_ns=10**12,stale=False)
    assert lease_age_admitted(lease,1000)
    source.update(observed_revision=1,last_integrated_stamp_ns=20)
    lease.identity=SimpleNamespace(**source)
    assert not lease_age_admitted(lease,1000)
    assert not declaration_only(source)


def test_zero_sensor_counters_need_explicit_initial_provenance(tmp_path):
    config,_,_=configuration(tmp_path)
    fields=dict(scene_revision='scene',content_sha256='a'*64,validity_generation=2,**identity(config))
    fields.update(observed_revision=0,last_integrated_stamp_ns=0)
    with pytest.raises(ValueError):OperationMapSource(**fields)
    fields.update(initialization_id='empty',declared_by='',initialization_stamp_ns=1)
    with pytest.raises(ValueError):OperationMapSource(**fields)


def test_common_placement_uses_initial_free_volume_and_preserves_label(tmp_path):
    from luggage_packing import placement_solver as ps
    config,_,_=configuration(tmp_path)
    source=identity(config)
    source.update(observed_revision=0,last_integrated_stamp_ns=0,initialization_id='session',declared_by='operator',initialization_stamp_ns=10)
    captured=assemble_snapshot(config,source,2,columns_for(config,[1]*64),[1]*64,scene_revision='scene',request_id='init')
    surface=json.loads(captured.surface.confirmation_json)
    evidence=dict(resolution=.1,nx=4,ny=4,nz=4,origin_xy=[-.2,-.2],floor_z=.6,states=[1]*64)
    candidates=ps.generate_candidates(surface,[.1,.1,.1],allowed_yaws=[0.,1.57079632679,3.14159265359,4.71238898038],
        params={'clearance_margin':0.,'same_layer_clearance':0.,'min_clearance_above':0.,'top_n':1000},
        volume_verifier=ps.make_volume_verifier(evidence))
    feasible=[c for c in candidates if c['feasible']]
    assert feasible and all(c['support_source']=='initial_empty' and c.get('volume_verified') for c in feasible)
    refused=ps.generate_candidates(surface,[.1,.1,.1],allowed_yaws=[0.],
        params={'clearance_margin':0.,'same_layer_clearance':0.,'min_clearance_above':0.},
        volume_verifier=lambda c:'volume_unknown')
    assert not any(c['feasible'] for c in refused)
