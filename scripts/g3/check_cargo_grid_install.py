#!/usr/bin/env python3
"""Installed, no-motion mapper startup/parameter check; no Gazebo stack."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import time
import rclpy
from rclpy.parameter import Parameter
from luggage_perception.cargo_volume_snapshot import copy_volume
from luggage_description.cargo_grid_parameters import default_resolution


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--workspace',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True);args=parser.parse_args()
    ws=args.workspace.resolve();out=args.out;out.mkdir(parents=True,exist_ok=True)
    files={
        'luggage_description':['luggage_description/cargo_grid_contract.py','luggage_description/cargo_grid_parameters.py','config/cargo_grid.yaml'],
        'luggage_perception':['luggage_perception/cargo_grid_projection.py','luggage_perception/cargo_volume_mapper.py',
                             'luggage_perception/cargo_volume_snapshot.py','luggage_perception/occupancy_map_store.py','scripts/cargo_volume_mapper_node.py'],
        'luggage_packing':['scripts/placement_planner_node.py'],
        'luggage_planning':['luggage_planning/placement_prior_volume.py','luggage_planning/hardware_placement_launch.py','launch/hardware_pick.launch.py'],
        'luggage_gazebo':['launch/sim_world.launch.py']}
    hashes={}
    for package,paths in files.items():
        for relative in paths:
            source=ws/'src'/package/relative
            if relative.startswith(package+'/'):
                installed=ws/'install'/package/'local/lib/python3.10/dist-packages'/relative
            elif relative.startswith('scripts/'):
                installed=ws/'install'/package/'lib'/package/Path(relative).name
            else:installed=ws/'install'/package/'share'/package/relative
            if source.read_bytes()!=installed.read_bytes():raise RuntimeError('stale install: '+str(installed))
            hashes[str(installed)]=hashlib.sha256(installed.read_bytes()).hexdigest()
    import luggage_perception.cargo_volume_mapper as implementation
    if not str(Path(implementation.__file__).resolve()).startswith(str(ws/'install')):
        raise RuntimeError('mapper not imported from intended install: '+implementation.__file__)
    node_path=ws/'install/luggage_perception/lib/luggage_perception/cargo_volume_mapper_node.py'
    spec=importlib.util.spec_from_file_location('installed_mapper_node',node_path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    rows=[]
    for resolution in (None,.02,.05):
        argv=['--ros-args','-p','scene_tf_config:='+str(ws/'src/luggage_description/config/scene_tf_deploy_update.yaml')]
        if resolution is not None:argv+=['-p','resolution:='+str(resolution)]
        rclpy.init(args=argv,domain_id=37);node=None
        try:
            start=time.perf_counter();node=module.CargoVolumeMapperNode();elapsed=time.perf_counter()-start
            expected=default_resolution() if resolution is None else resolution
            identity=node._store.identity;volume=copy_volume(node._mapper);surface=node._store.snapshot()
            changed=node.set_parameters([Parameter('resolution',value=.03)])[0]
            assert not changed.successful and node.get_parameter('resolution').value==expected
            assert node._store.identity==identity
            assert node._mapper.resolution==surface.resolution==volume['resolution_m']==expected
            rows.append(dict(requested_resolution=resolution,effective_resolution=expected,
                voxel_count=len(volume['states']),startup_s=elapsed,hot_change_rejected=not changed.successful,
                map_unchanged=node._store.identity==identity,epoch=identity.epoch))
        finally:
            if node is not None:node.shutdown_tf();node.destroy_node()
            rclpy.shutdown()
    (out/'installed_runtime.json').write_text(json.dumps(dict(hashes=hashes,rows=rows,pass_=True),indent=2))
    print(json.dumps(rows))

if __name__=='__main__':main()
