"""Isolated installed placement service and genuine OCC input publisher.

The owner runs the installed production main/executor. The fixture client
also serves the stored expanded URDF through the normal parameter service;
no robot, controller or motion stack is created.
"""
import argparse
import hashlib
import importlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import time


def save(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True)+'\n')
    temporary.replace(path)


def verify_install(source, install):
    from ament_index_python.packages import get_package_prefix
    hashes = {}
    for package in ('luggage_perception','luggage_description','luggage_packing','luggage_planning'):
        module = importlib.import_module(package)
        directory = Path(module.__file__).resolve().parent
        if not directory.is_relative_to(install):
            raise RuntimeError('mixed product install: '+str(directory))
        for original in sorted((source/'src'/package/package).glob('*.py')):
            loaded = directory/original.name
            if loaded.read_bytes() != original.read_bytes():
                raise RuntimeError('installed/source mismatch: '+str(original))
            hashes[str(original.relative_to(source))] = hashlib.sha256(loaded.read_bytes()).hexdigest()
    path = Path(get_package_prefix('luggage_packing'))/'lib/luggage_packing/placement_planner_node.py'
    if path.read_bytes() != (source/'src/luggage_packing/scripts/placement_planner_node.py').read_bytes():
        raise RuntimeError('installed planner/source mismatch')
    hashes['placement_planner_node.py'] = hashlib.sha256(path.read_bytes()).hexdigest()
    return path,hashes


def owner(args,out,source,install):
    from rclpy.parameter import Parameter
    from luggage_description.runtime_identity import scene_revision_from_path
    path,hashes = verify_install(source,install)
    spec = importlib.util.spec_from_file_location('installed_placement_node',path)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    original_init = module.PlacementPlannerNode.__init__
    original_publish = module.PlacementPlannerNode._publish_last
    manifest = dict(production_main=True,executor_override=False,loaded_node=str(path),
        hashes=hashes,started_ns=time.time_ns(),requests=0)
    scene = source/'src/luggage_description/config/scene_tf.yaml'

    def initialize(node):
        original_init(node,parameter_overrides=[Parameter('scene_tf_config',value=str(scene)),
            Parameter('frame_scene_revision',value=scene_revision_from_path(str(scene))),
            Parameter('structural_search',value=True)])
        def ready():
            if (node._surface is not None and getattr(node,'_committed',None) is not None
                    and node._urdf_identity is not None and node._urdf_identity.ready):
                save(out/'ready.json',dict(map_revision=node._surface['map_revision'],
                    geometry_hash=node._geometry_hash,urdf_hash=node._urdf_identity.hash))
                node._fixture_ready_timer.cancel()
        node._fixture_ready_timer = node.create_timer(.1,ready)

    def publish(node,payload):
        original_publish(node,payload)
        index = manifest['requests'];manifest['requests'] += 1
        save(out/('result-%03d.json'%index),payload)

    module.PlacementPlannerNode.__init__ = initialize
    module.PlacementPlannerNode._publish_last = publish
    def stop(*_):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGUSR2,stop)
    save(out/'manifest.json',manifest)
    try:
        module.main()
    finally:
        manifest.update(ended_ns=time.time_ns())
        save(out/'manifest.json',manifest)
    return 0


def client(args,out,source,install):
    import numpy as np
    import yaml
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    from rclpy.serialization import serialize_message
    from std_msgs.msg import String
    from luggage_msgs.msg import SurfaceContractV2
    from luggage_msgs.srv import ComputePlacement
    from luggage_description.runtime_identity import scene_revision_from_path
    from luggage_description.frame_diagnostics import BoundaryContext
    from luggage_perception.frame_message_adapters import measured_surface_to_message
    verify_install(source,install)
    cases = Path(__file__).with_name('occ_query_placement_cases.py')
    spec = importlib.util.spec_from_file_location('placement_cases',cases)
    fixture = importlib.util.module_from_spec(spec);spec.loader.exec_module(fixture)
    scene_path = source/'src/luggage_description/config/scene_tf.yaml'
    scene = yaml.safe_load(scene_path.read_text())
    scene_revision = scene_revision_from_path(str(scene_path))
    store,geometry,points = fixture.create_case(scene,args.resolution,args.boxes,args.fallback)
    np.savez_compressed(out/'input.npz',points=points,geometry=np.asarray([
        [*center,*size,yaw] for center,size,yaw in geometry]).reshape((-1,7)))
    save(out/'surface-v1.json',store.v1_surface_dict())
    save(out/'ledger.json',store.committed_dict('container_link',None))
    urdf = source/'src/luggage_planning/data/reachability_atlas/robot_description.xml'
    rclpy.init()
    node = rclpy.create_node('robot_state_publisher',parameter_overrides=[
        Parameter('robot_description',value=urdf.read_text())],
        automatically_declare_parameters_from_overrides=True)
    qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    surface_pub = node.create_publisher(SurfaceContractV2,'/luggage/cargo_map/surface_v2',qos)
    ledger_pub = node.create_publisher(String,'/luggage/cargo_map/committed',qos)
    service = node.create_client(ComputePlacement,'/placement_planner/compute_placement')
    rows=[];result=dict(outcome='not_evaluated',required=30)
    def publish():
        record = store.v2_record(scene_revision,'map-rev-'+str(store.identity.revision))
        context = BoundaryContext('F06','cargo_volume_mapper','placement_planner',
            record['request_id'],scene_revision,'container_link','','scene',
            store.mapper.geometry_hash,store.identity.revision)
        surface = measured_surface_to_message(record,context)
        ledger = String(data=json.dumps(store.committed_dict('container_link',None)))
        surface_pub.publish(surface);ledger_pub.publish(ledger)
        return surface,ledger
    try:
        surface,ledger = publish()
        (out/'surface.cdr').write_bytes(serialize_message(surface))
        (out/'ledger.cdr').write_bytes(serialize_message(ledger))
        deadline = time.monotonic()+90.
        while not (Path(args.owner)/'ready.json').exists():
            if time.monotonic()>deadline:
                raise RuntimeError('placement input/URDF readiness deadline')
            rclpy.spin_once(node,timeout_sec=.05)
        if not service.wait_for_service(timeout_sec=5.):
            raise RuntimeError('placement service unavailable')
        for index in range(-1,args.warmup+args.samples):
            request = ComputePlacement.Request()
            request.box.width,request.box.depth,request.box.height = fixture.BOX
            request.box.height_valid,request.box.height_source = True,1
            request.geometry_hash,request.max_candidates = store.mapper.geometry_hash,5
            (out/('%03d.request.cdr'%index)).write_bytes(serialize_message(request))
            begin = time.perf_counter_ns()
            # Snapshot projection, contract construction/publication and the
            # actual request/serialization are all inside the timed interval.
            publish()
            future = service.call_async(request)
            rclpy.spin_until_future_complete(node,future,timeout_sec=20.)
            if not future.done() or future.exception():
                raise RuntimeError('placement request deadline/error')
            response = future.result()
            wire = serialize_message(response)
            elapsed = time.perf_counter_ns()-begin
            (out/('%03d.response.cdr'%index)).write_bytes(wire)
            diagnostic_path = Path(args.owner)/('result-%03d.json'%(index+1))
            diagnostic = json.loads(diagnostic_path.read_text())
            actual_source = (diagnostic.get('structural') or {}).get('batch_source')
            expected = 'grid_fallback' if args.fallback else 'structural'
            row=dict(index=index,cold=index==-1,warmup=0<=index<args.warmup,
                elapsed_ns=elapsed,success=response.success,reason=response.reason_code,
                source=actual_source,expected_source=expected,
                map_revision=diagnostic.get('map_revision'),
                response_sha256=hashlib.sha256(wire).hexdigest())
            rows.append(row)
            with (out/'samples.jsonl').open('a') as stream:
                stream.write(json.dumps(row)+'\n')
            if not response.success or actual_source!=expected or elapsed>20_000_000_000:
                result=dict(outcome='fail',first_failure=row)
                break
        else:
            samples=sorted(row['elapsed_ns']/1e9 for row in rows if row['index']>=args.warmup)
            if len(samples)==30:
                result=dict(outcome='pass' if samples[28]<=5 and samples[-1]<=20 else 'fail',
                    p95_s=samples[28],max_s=samples[-1],samples=30)
    except Exception as error:
        result=dict(outcome='fail',error=repr(error))
    finally:
        result.update(requested_samples=args.samples,capture_complete=True,
            replay_possible=True,fixture='input.npz + source archive + surface/ledger CDR',
            received_samples=len(rows),overall_g2='not_evaluated')
        save(out/'result.json',result)
        node.destroy_node();rclpy.shutdown()
    return int(result['outcome']!='pass')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('role',choices=('owner','client'))
    for name in ('out','source','install','owner'):
        parser.add_argument('--'+name,required=name!='owner')
    parser.add_argument('--domain',type=int,required=True)
    parser.add_argument('--resolution',type=float,default=.01)
    parser.add_argument('--boxes',type=int,default=0)
    parser.add_argument('--fallback',action='store_true')
    parser.add_argument('--samples',type=int,default=30)
    parser.add_argument('--warmup',type=int,default=3)
    args=parser.parse_args()
    os.environ.update(ROS_DOMAIN_ID=str(args.domain),ROS_LOCALHOST_ONLY='1')
    out=Path(args.out).resolve();out.mkdir(parents=True,exist_ok=False)
    return (owner if args.role=='owner' else client)(args,out,Path(args.source).resolve(),Path(args.install).resolve())


if __name__=='__main__':
    raise SystemExit(main())
