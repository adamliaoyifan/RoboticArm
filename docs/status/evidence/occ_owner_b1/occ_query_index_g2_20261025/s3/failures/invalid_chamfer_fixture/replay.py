"""Replay invalid pre-construction fixture; no ROS graph or motion."""
import argparse,sys
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--root',required=True);a=p.parse_args()
r=Path(a.root).resolve()
sys.path[:0]=[str(r/'src/luggage_perception'),str(r/'src/luggage_description')]
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_description.container_geometry import ContainerGeometry,Chamfer
h=ContainerGeometry(1,'container_link',.2,.2,0.,.4,Chamfer('positive_y',.1,.2,.1))
try:
    CargoVolumeMapper((.4,.4,.4),(0,0,.2),0,.02,geometry_descriptor=h.descriptor())
except ValueError as e:
    assert 'chamfer.wall_y' in str(e)
    print('reproduced:',e)
else:
    raise AssertionError('expected invalid-hull refusal')
