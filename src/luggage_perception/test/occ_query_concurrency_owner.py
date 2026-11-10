"""Original installed mapper with real NodeRuntime and passive boundary trace."""
import argparse
from collections import deque
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import resource
import signal
import sys
import threading
import time


def save(path,value):
    temporary=path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value,indent=2,sort_keys=True)+'\n');temporary.replace(path)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('out','source','install'):parser.add_argument('--'+key,required=True)
    parser.add_argument('--domain',type=int,required=True)
    parser.add_argument('--boxes',type=int,choices=(0,5,20),required=True)
    parser.add_argument('--resolution',type=float,choices=(.01,.02,.05),required=True)
    args=parser.parse_args()
    os.environ.update(ROS_DOMAIN_ID=str(args.domain),ROS_LOCALHOST_ONLY='1')
    from ament_index_python.packages import get_package_prefix
    from luggage_description.runtime_identity import build_identity
    from luggage_description.runtime_graph import endpoint_registry,identity_endpoints
    from luggage_description.ros_frame_bindings.runtime import NodeRuntime
    from luggage_perception.occupancy_map_store import OccupancyMapStore,MapUpdate,UpdateKind
    import numpy as np
    import yaml
    import luggage_perception.cargo_volume_mapper as mapper_module
    source=Path(args.source).resolve();install=Path(args.install).resolve()
    out=Path(args.out).resolve();out.mkdir(parents=True,exist_ok=False)
    loaded=Path(mapper_module.__file__).resolve()
    if not loaded.is_relative_to(install):raise RuntimeError('mixed mapper install')
    for path in (source/'src/luggage_perception/luggage_perception').glob('*.py'):
        if path.read_bytes()!=(loaded.parent/path.name).read_bytes():
            raise RuntimeError('source/install mismatch: '+path.name)
    node_path=Path(get_package_prefix('luggage_perception'))/'lib/luggage_perception/cargo_volume_mapper_node.py'
    if node_path.read_bytes()!=(source/'src/luggage_perception/scripts/cargo_volume_mapper_node.py').read_bytes():
        raise RuntimeError('node/source mismatch')
    spec=importlib.util.spec_from_file_location('installed_mapper',node_path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    # Production report collection measures the installed node, not the
    # external fixture entry point. Fixture provenance remains separate.
    sys.modules['__main__']=module
    peers=['cargo_volume_mapper','occ_g2_load']
    assets=source/'src/luggage_planning/data/reachability_atlas'
    scene=source/'src/luggage_description/config/scene_tf.yaml'
    identity=build_identity(scene_tf_config=str(scene),calibration_revision='occ-g2-frozen',
        urdf_text=(assets/'robot_description.xml').read_text(),
        srdf_text=(source/'src/luggage_description/config/S20_with_camera.srdf').read_text(),
        tool_link='suction_contact_frame',atlas_files=(assets/'s20_container_collision_aware.npz',
        assets/'s20_container_collision_aware.yaml'),endpoints=identity_endpoints(endpoint_registry(peers)))
    events=deque(maxlen=100_000);nodes=[];stop_requested=[]
    def event(stage,**values):
        events.append(dict(stage=stage,monotonic_ns=time.perf_counter_ns(),thread=threading.get_ident(),**values))
    def trace_method(cls,name):
        original=getattr(cls,name)
        def traced(instance,*values,**kwargs):
            request_id=''
            if values:
                query=getattr(values[0],'query',None)
                request_id=getattr(query,'request_id',getattr(values[0],'view_request_id',''))
            def boundary_fields():
                store=getattr(instance,'_store',None)
                fields=dict(request_id=request_id)
                if store is not None:fields['snapshot_identity']=list(store.identity)
                if name=='_on_volume' and values:
                    fields['volume_premise']={key:getattr(values[0],key) for key in (
                        'scene_revision','geometry_hash','expected_map_revision',
                        'expected_epoch','expected_content_digest')}
                return fields
            event(name+'_begin',**boundary_fields())
            try:return original(instance,*values,**kwargs)
            finally:event(name+'_end',**boundary_fields())
        setattr(cls,name,traced)
    for name in ('_on_stats','_on_volume','_on_integrate','_integrate_view'):
        trace_method(module.CargoVolumeMapperNode,name)
    if importlib.util.find_spec('luggage_perception.cargo_map_query_service') is not None:
        from luggage_perception.cargo_map_query_service import MapQueryService
        trace_method(MapQueryService,'_execute')
    original_snapshot=OccupancyMapStore.snapshot
    def snapshot_read(store,*values,**kwargs):
        result=original_snapshot(store,*values,**kwargs)
        event('snapshot_read',snapshot_identity=list(result.identity))
        return result
    OccupancyMapStore.snapshot=snapshot_read
    original_report=NodeRuntime._on_report
    def report(runtime,message):
        original_report(runtime,message)
        event('heartbeat',peer=message.peer,sequence=int(message.heartbeat_sequence),
            status=runtime.receive_trace[-1]['status'],readiness=runtime.readiness())
    NodeRuntime._on_report=report
    # Keep callback and transaction-lock waits distinct. These wrappers
    # observe existing boundaries without changing ownership or locking.
    for name in ('apply_update','_apply_locked'):
        original=getattr(OccupancyMapStore,name)
        def traced(store,*values,_original=original,_name=name,**kwargs):
            event(_name+'_begin')
            try:return _original(store,*values,**kwargs)
            finally:event(_name+'_end')
        setattr(OccupancyMapStore,name,traced)
    original_cloud=module.CargoVolumeMapperNode._on_cloud
    cloud_stamps=deque(maxlen=128)
    cloud_ack_lock=threading.Lock()
    def cloud(node,message):
        begin=time.perf_counter_ns();original_cloud(node,message)
        stamp=[int(message.header.stamp.sec),int(message.header.stamp.nanosec)]
        event('cloud_ingest',stamp=stamp,points=int(message.width*message.height),begin_ns=begin)
        with cloud_ack_lock:
            cloud_stamps.append(stamp)
            save(out/'cloud_ack.json',dict(stamps=list(cloud_stamps),received_ns=time.perf_counter_ns()))
    module.CargoVolumeMapperNode._on_cloud=cloud
    original_init=module.CargoVolumeMapperNode.__init__
    def initialize(node):
        original_init(node);nodes.append(node)
        m,s=node._mapper,node._store
        for i,(z,y,x) in enumerate((z,y,x) for z in range(4) for y in (-.48,.18) for x in (-.5,0.,.5)):
            if i>=args.boxes:break
            result=s.apply_update(MapUpdate(operation_id='setup-'+str(i),kind=UpdateKind.ADD_BOX_V1,
                box=((x,y,m._hull.floor_z+.11+.24*z),(.35,.53,.22),0.),box_id='load-box-'+str(i)))
            if not result.accepted:raise RuntimeError('setup geometry: '+result.reason)
        assert len(m._placed_boxes)==args.boxes
        save(out/'ready.json',dict(identity=identity,peers=peers,boxes=args.boxes,
            mapper_config=dict(inner_size=[m.inner_l,m.inner_w,m.inner_h],center_base=list(m.center),
                yaw=m.yaw,resolution=m.resolution,geometry_descriptor=m._hull.descriptor(),
                max_raycast_points=m.max_raycast_points,minimum_z=node._floor_z+max(0.,node._min_z_above_floor),
                require_confirmation=node._require_confirmed_map,pose_at_confirm=node._world_from_container),
            floor_z=m._hull.floor_z,ceiling_z=m._hull.ceiling_z,shape=[m.nx,m.ny,m.nz],
            query_resources=dict(s._query_registry.resource_summary()) if hasattr(s,'_query_registry') else {},
            map_revision=s.identity.revision,epoch=s.identity.epoch,content_digest=s.identity.content_digest))
    module.CargoVolumeMapperNode.__init__=initialize
    def capture(*_):
        if not nodes:return
        node=nodes[0]
        with node._store._txn_lock:
            m=node._mapper
            np.savez_compressed(out/'boundary_map.npz',states=np.frombuffer(m._grid,'u1'),
                active=np.frombuffer(m._active,'u1'),source=np.frombuffer(m._source,'u1'),
                labels=np.frombuffer(m._labels,'<i8'),instances=np.frombuffer(m._instance_ids,'<i8'),
                logodds=np.asarray(m._occupancy.values),locked=np.asarray(m._occupancy.geometry_locked),
                shadow=np.frombuffer(m._sensor_values,'<f8'),shadow_seen=np.frombuffer(m._sensor_seen,'u1'),
                shadow_labels=np.frombuffer(m._sensor_labels,'<i8'),shadow_instances=np.frombuffer(m._sensor_instance_ids,'<i8'))
            save(out/'boundary.json',dict(stats=m.stats(),surface=node._store.v1_surface_dict(),
                ledger=node._store.committed_dict('container_link',None)))
        runtime_snapshot=node._frame_runtime.snapshot()
        runtime_snapshot['observed_at_monotonic']=time.monotonic()
        save(out/'runtime.json',runtime_snapshot)
        if hasattr(node._store,'_query_registry'):
            save(out/'query_resources.json',dict(node._store._query_registry.resource_summary()))
        save(out/'capture.json',dict(capture_complete=True,replay_possible=True,
            captured_ns=time.time_ns(),input='frozen source + ready + client cloud/requests',
            window='client heartbeat 10s pre/5s post; consistent owner map after drain'))
    signal.signal(signal.SIGUSR1,capture)
    def stop(*_):stop_requested.append(True);raise KeyboardInterrupt()
    signal.signal(signal.SIGUSR2,stop)
    manifest=dict(started_ns=time.time_ns(),identity=identity,peers=peers,production_main=True,
        executor_override=False,heartbeat_period_s=.25,lease_s=2.,loaded_node=str(node_path),
        node_sha256=hashlib.sha256(node_path.read_bytes()).hexdigest(),fixture_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        resolution=args.resolution)
    save(out/'manifest.json',manifest)
    # A JSON identity must be a ROS STRING, not parsed as a YAML mapping by
    # the -p argument parser. Let YAML quote the measured string explicitly.
    parameters={'cargo_volume_mapper':{'ros__parameters':dict(scene_tf_config=str(scene),
        resolution=args.resolution,frame_scene_revision=identity['scene_revision'],frame_runtime=True,
        frame_identity_json=json.dumps(identity),frame_peers=','.join(peers))}}
    parameter_path=out/'parameters.yaml'
    parameter_path.write_text(yaml.safe_dump(parameters,sort_keys=False))
    try:
        module.main(['--ros-args','--params-file',str(parameter_path)])
    except Exception as error:
        import rclpy
        if stop_requested and not rclpy.ok() and 'rcl_shutdown already called' in str(error):
            manifest['post_scoring_teardown_error']=repr(error)
        else:
            manifest['error']=repr(error);raise
    finally:
        manifest.update(ended_ns=time.time_ns(),rss_peak_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
        save(out/'manifest.json',manifest)
        with (out/'trace.jsonl').open('w') as stream:
            for row in events:stream.write(json.dumps(row)+'\n')
    return 0


if __name__=='__main__':raise SystemExit(main())
