import pytest
from luggage_planning.placement_prior_surface import ConfirmedSurfaceStore


def pair():
    surface = dict(map_revision=3, geometry_hash='hull', frame_id='container_link',
                   scene_revision='calibration', floor_z_container=.5,
                   state=[['occupied','unknown']], surface_z_container=[[.7,None]])
    legacy = dict(map_revision=3,geometry_hash='hull',frame='container_local',
                  map_source='measured',occupancy_confirmed=True,
                  acquisition_stamp_ns=123,pending_changes=[],
                  state=[['occupied','unknown']],height=[[.2,0.]])
    return surface,legacy


def test_async_join_copies_data_and_returns_independent_readers():
    surface,legacy=pair();store=ConfirmedSurfaceStore()
    store.update_typed(surface,b'cdr');assert store.current() is None
    store.update_confirmation(legacy);captured=store.current();assert captured
    legacy['height'][0][0]=9
    surface['surface_z_container'][0][0]=99
    assert store.current().record()['surface_z_container'][0][0]==.7
    captured.record()['state'][0][0]='free'
    assert store.current().record()['state'][0][0]=='occupied'


@pytest.mark.parametrize('field,value',[
    ('map_source','release_integrated'),('map_source','static_empty'),
    ('occupancy_confirmed',False),('acquisition_stamp_ns',0),
    ('pending_changes',[{'event':'box_placed'}]),('map_revision',4),
    ('geometry_hash','other'),('height',[[.3,0.]]),
    ('state',[['free','unknown']]),
])
def test_unverified_or_mismatched_snapshot_not_admitted(field,value):
    surface,legacy=pair();store=ConfirmedSurfaceStore()
    store.update_typed(surface,b'cdr');store.update_confirmation(legacy)
    assert store.current()
    legacy[field]=value;store.update_confirmation(legacy)
    assert store.current() is None


def test_new_revision_invalidates_until_both_publications_arrive():
    surface,legacy=pair();store=ConfirmedSurfaceStore()
    store.update_typed(surface,b'old');store.update_confirmation(legacy)
    previous=store.current()
    surface['map_revision']=4;store.update_typed(surface,b'new')
    assert store.current() is None
    legacy['map_revision']=4;store.update_confirmation(legacy)
    assert store.current().identity != previous.identity
    assert previous.surface_cdr == b'old'


def test_committed_geometry_requires_owner_confirmation_gate():
    surface,legacy=pair();legacy['placed_count']=1
    for required in (False,True):
        store=ConfirmedSurfaceStore(confirmation_required=required)
        store.update_typed(surface,b'cdr');store.update_confirmation(legacy)
        assert bool(store.current()) is required


def test_late_old_revision_does_not_resurrect_obsolete_map():
    surface,legacy=pair();store=ConfirmedSurfaceStore()
    store.update_typed(surface,b'old');store.update_confirmation(legacy)
    surface['map_revision']=4;legacy['map_revision']=4
    store.update_typed(surface,b'new');store.update_confirmation(legacy)
    surface['map_revision']=3;legacy['map_revision']=3
    store.update_typed(surface,b'old');store.update_confirmation(legacy)
    assert store.current().record()['map_revision']==4
