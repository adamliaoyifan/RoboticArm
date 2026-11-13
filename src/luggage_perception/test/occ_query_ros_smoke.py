"""Installed, isolated-domain read-only ROS wiring rehearsal; not a scored gate."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import resource
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True)
    parser.add_argument('--scene', required=True)
    parser.add_argument('--domain', type=int, required=True)
    parser.add_argument('--resolution', type=float, choices=(.01, .02, .05), default=.01)
    args = parser.parse_args()
    if not 100 <= args.domain <= 232:
        parser.error('use a dedicated test domain in 100..232')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.serialization import serialize_message
    from ament_index_python.packages import get_package_prefix
    import luggage_perception
    import luggage_description
    import luggage_msgs
    import luggage_planning.planning_contract
    from luggage_msgs import srv
    from geometry_msgs.msg import Point

    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    module_path = Path(get_package_prefix('luggage_perception'))/'lib/luggage_perception/cargo_volume_mapper_node.py'
    paths = {m.__name__: str(Path(m.__file__).resolve()) for m in
             (luggage_perception, luggage_description, luggage_msgs, luggage_planning.planning_contract)}
    if any('/tmp/occ-query-g2-install/' not in path for path in paths.values()):
        raise RuntimeError('all product imports must use the independent install: %r' % paths)
    manifest = dict(scored=False, profile='empty-map ROS wiring rehearsal', domain=args.domain,
        resolution=args.resolution, source_modules=paths, node_path=str(module_path),
        node_sha256=hashlib.sha256(module_path.read_bytes()).hexdigest(),
        scene_sha256=hashlib.sha256(Path(args.scene).read_bytes()).hexdigest(),
        created_clients=[], created_nodes=[], samples=[], started_ns=time.time_ns())
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    spec = importlib.util.spec_from_file_location('installed_occ_mapper', module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rclpy.init(args=['--ros-args', '-p', 'scene_tf_config:='+str(Path(args.scene).resolve()),
                     '-p', 'resolution:='+str(args.resolution)])
    owner = client = executor = None
    try:
        owner = module.CargoVolumeMapperNode()
        client = rclpy.create_node('occ_query_g2_read_rehearsal')
        manifest['created_nodes'] = [owner.get_name(), client.get_name()]
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(owner)
        executor.add_node(client)
        cases = (
            (srv.GetCargoMapStats, '/cargo_map/get_stats'),
            (srv.GetCargoMapSummaryV2, '/cargo_map/get_summary_v2'),
            (srv.QueryCargoCellsV2, '/cargo_map/query_cells_v2'),
            (srv.QueryCargoDistanceV2, '/cargo_map/query_distance_v2'),
            (srv.GetCargoSurfacePatchV2, '/cargo_map/get_surface_patch_v2'),
        )
        for kind, name in cases:
            endpoint = client.create_client(kind, name)
            manifest['created_clients'].append(name)
            if not endpoint.wait_for_service(timeout_sec=5.):
                raise RuntimeError('service unavailable: '+name)
            for iteration in range(3):
                request = kind.Request()
                if hasattr(request, 'query'):
                    request.query.request_id = name+'-'+str(iteration)
                    request.query.frame_id = 'container_link'
                    request.query.use_current = True
                    request.query.release_after_query = True
                    request.query.limit = 2
                if kind is srv.QueryCargoDistanceV2:
                    request.points = [Point(x=0., y=0., z=1.)]
                    request.max_radius_m = .3
                case = out/(kind.__name__+'_'+str(iteration))
                case.with_suffix('.request.cdr').write_bytes(serialize_message(request))
                started = time.perf_counter_ns()
                future = endpoint.call_async(request)
                executor.spin_until_future_complete(future, timeout_sec=5.)
                elapsed = time.perf_counter_ns()-started
                if not future.done() or future.exception():
                    raise RuntimeError('query deadline/error: '+name)
                response = future.result()
                case.with_suffix('.response.cdr').write_bytes(serialize_message(response))
                status = response.meta.status if hasattr(response, 'meta') else ('OK' if response.success else response.message)
                manifest['samples'].append(dict(service=name, iteration=iteration,
                    elapsed_ns=elapsed, status=status))
                if status != 'OK':
                    raise RuntimeError('query refused: '+name+': '+status)
            client.destroy_client(endpoint)
        manifest['outcome'] = 'rehearsal_complete'
    except Exception as error:
        manifest.update(outcome='rehearsal_failed', error=repr(error), capture_complete=True,
                        replay_possible=True, replay='rerun this script with identical scene/install')
        raise
    finally:
        manifest['ended_ns'] = time.time_ns()
        manifest['rss_peak_kib'] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        (out/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
        if executor is not None:
            executor.shutdown(timeout_sec=5.)
        if owner is not None:
            owner.shutdown_tf()
            owner.destroy_node()
        if client is not None:
            client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(json.dumps(dict(outcome=manifest['outcome'], samples=len(manifest['samples']),
                         artifacts=str(out))))


if __name__ == '__main__':
    main()
