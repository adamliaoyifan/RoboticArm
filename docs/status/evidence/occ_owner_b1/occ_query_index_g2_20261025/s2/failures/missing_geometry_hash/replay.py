import argparse,json,sys
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--root',required=True);a=p.parse_args();root=Path(a.root)
sys.path[:0]=[str(root),str(root/'src/luggage_perception'),str(root/'src/luggage_description')]
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import OccupancyMapStore,MapUpdate,UpdateKind
s=OccupancyMapStore(CargoVolumeMapper((.12,.12,.12),(0,0,.06),0,.02),epoch='epoch')
u=MapUpdate(operation_id='0',kind=UpdateKind.INTEGRATE_VIEW,points=((.01,.01,.03),),settled=True,schema_version=1,origin=(0,0,.15),stamp_key=(0,0),acquisition_stamp_ns=1)
r=s.apply_update(u)
print(json.dumps({'status':r.status.value,'reason':r.reason,'revision':r.identity.revision,'missing_input':'geometry_hash'}))
assert not r.accepted and r.reason=='GEOMETRY_HASH_MISMATCH'
