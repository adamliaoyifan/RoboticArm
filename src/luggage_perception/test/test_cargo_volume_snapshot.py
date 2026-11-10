from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.cargo_volume_snapshot import copy_volume


def test_snapshot_is_a_copy_and_export_never_changes_revision():
    mapper=CargoVolumeMapper((1.,1.,1.),(0.,0.,.5),0.,.1)
    before=mapper.stats()['map_revision']
    snapshot=copy_volume(mapper)
    assert mapper.stats()['map_revision']==before
    assert snapshot['dimensions_xyz']==(10,10,10)
    assert snapshot['origin_container']==(-.5,-.5,0.)
    mapper.mark_occupied_world(0.,0.,.5)
    assert snapshot['states']!=bytes(mapper._grid)
