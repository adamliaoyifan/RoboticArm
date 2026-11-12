import json
from types import SimpleNamespace
import pytest
from luggage_planning.placement_prior.surface import ConfirmedSurface
from luggage_planning.placement_prior.volume import blocked_volume_runs


def fixture():
    surface=dict(scene_revision='scene',geometry_hash='hull',map_revision=4,
        origin_xy_container=[0.,0.],floor_z_container=0.,ceiling_z_container=.5,
        resolution_m=.1,state=[['unknown']])
    confirmed=ConfirmedSurface(json.dumps(surface).encode(),b'cdr',
                               json.dumps(dict(acquisition_stamp_ns=42)).encode())
    volume=SimpleNamespace(success=True,frame_id='container_link',scene_revision='scene',
        geometry_hash='hull',map_revision=4,acquisition_stamp_ns=42,map_source='measured',
        occupancy_confirmed=True,has_pending_changes=False,dimensions_xyz=[1,1,5],
        resolution_m=.1,origin_container=[0.,0.,0.],states=[0,1,1,2,0],active=[True]*5)
    return confirmed,volume


def test_unknown_and_occupied_preserved_but_free_gap_not_filled():
    surface,volume=fixture()
    runs=blocked_volume_runs(volume,surface)
    assert len(runs)==2
    assert runs[0][0]==pytest.approx((.05,.05,.05))
    assert runs[0][1]==pytest.approx((.1,.1,.1))
    assert runs[1][0]==pytest.approx((.05,.05,.4))
    assert runs[1][1]==pytest.approx((.1,.1,.2))
    assert volume.states==[0,1,1,2,0]


@pytest.mark.parametrize('field,value',[
    ('map_revision',5),('acquisition_stamp_ns',43),('geometry_hash','other'),
    ('scene_revision','other'),('has_pending_changes',True),('occupancy_confirmed',False),
    ('map_source','static_empty'),('states',[0]),('states',[0,1,9,2,0]),
    ('origin_container',[0.,0.,.1]),('dimensions_xyz',[1,1,4])])
def test_malformed_or_unjoinable_volume_refused(field,value):
    surface,volume=fixture();setattr(volume,field,value)
    with pytest.raises(ValueError):blocked_volume_runs(volume,surface)


def test_inactive_hull_cell_not_exported_as_obstacle():
    surface,volume=fixture();volume.active=[False]*5
    assert blocked_volume_runs(volume,surface)==()


def test_fine_owner_volume_above_legacy_million_cell_limit_is_admitted():
    # A free volume produces no obstacles but must pass the shared 8M bound.
    from types import SimpleNamespace
    import json
    nx=ny=nz=101
    record=dict(scene_revision='s',geometry_hash='g',map_revision=1,
                resolution_m=.01,origin_xy_container=[0.,0.],
                floor_z_container=0.,ceiling_z_container=1.01,
                state=[['free']*ny for _ in range(nx)])
    surface=SimpleNamespace(record=lambda:record,
        confirmation_json=json.dumps(dict(acquisition_stamp_ns=42)))
    volume=SimpleNamespace(success=True,frame_id='container_link',scene_revision='s',
        geometry_hash='g',map_revision=1,acquisition_stamp_ns=42,map_source='measured',
        occupancy_confirmed=True,has_pending_changes=False,dimensions_xyz=(nx,ny,nz),
        states=bytes([1])*(nx*ny*nz),active=[True]*(nx*ny*nz),
        resolution_m=.01,origin_container=[0.,0.,0.])
    assert blocked_volume_runs(volume,surface)==()
