import argparse,json,sys
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--root',required=True);a=p.parse_args();root=Path(a.root)
sys.path[:0]=[str(root),str(root/'src/luggage_perception'),str(root/'src/luggage_description')]
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import OccupancyMapStore
s=OccupancyMapStore(CargoVolumeMapper((.12,.12,.12),(0,0,.06),0,.02),epoch='alias-repro')
q=s.acquire_query_snapshot()._data() if hasattr(s,'acquire_query_snapshot') else None
before=s.snapshot().known_ratio[0][0]
s.v1_surface_dict()['known_ratio'][0][0]=.777
result={'legacy_before':before,'legacy_after':s.snapshot().known_ratio[0][0],'query_after':q.map_snapshot.known_ratio[0][0] if q else None,'primary_states':list(set(s.mapper._grid))}
print(json.dumps(result))
assert result['legacy_after']==.777 and result['primary_states']==[0]
