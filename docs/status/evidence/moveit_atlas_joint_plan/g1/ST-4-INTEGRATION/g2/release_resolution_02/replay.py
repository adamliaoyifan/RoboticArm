"""Diagnostic: exact original observations/retreat, native O at10cm and2cm.

Private scenes only; no controller, scene writes, Gazebo or production changes.
The zero-padding variant is a causal ablation, never a runtime policy.
"""
import copy
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.convert import message_to_ordereddict
import yaml
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
from moveit_msgs.msg import PlanningScene
from luggage_msgs.srv import ValidateRouteV2
from uld_occ_observer.srv import IntegrateObservation, ReadCells
from luggage_planning.joint_operation_observer import ObserverConfiguration, ObserverSnapshotPort
from luggage_planning.placement_prior_snapshot import build_private_snapshot
from luggage_planning.placement_prior_contracts import Transform
from ament_index_python.packages import get_package_prefix

OUTPUT = Path(__file__).resolve().parent
SOURCE = OUTPUT.parent / 'native_cycle_02/stack'
CASE = SOURCE / 'preparation/test_actual_complete_o_operation_returns_observed_success[observer]'
OPERATION = CASE / 'operation/5016802095082708_9c7a42546e91f3d9'


def sha(data):
    return hashlib.sha256(data).hexdigest()


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=5.)
    return process.returncode


def main():
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    manifest = dict(started_ns=time.time_ns(), controller_goals=0, scene_writes=0,
        gazebo=False, hardware=False, capture_complete=False, replay_possible=False,
        revision=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        dirty=subprocess.check_output(['git', 'status', '--porcelain'], text=True).splitlines(),
        command='source /tmp/joint_operation_o_env.sh; python3 ' + str(Path(__file__).relative_to(Path.cwd())),
        source_files={}, records=[], native_exits=[], variant_count=0)
    summary = dict(variants=[], owner_snapshots=[])
    def source(path):
        data = path.read_bytes()
        manifest['source_files'][str(path)] = dict(sha256=sha(data), bytes=len(data))
        return data
    def record(folder, name, message):
        data = message if isinstance(message, bytes) else serialize_message(message)
        path = folder / ('%04d_' % len(manifest['records']) + name)
        path.write_bytes(data)
        manifest['records'].append(dict(file=str(path.relative_to(OUTPUT)), sha256=sha(data), bytes=len(data)))
        save(OUTPUT / 'manifest.json', manifest)
    original = deserialize_message(source(OPERATION / '00751_request'), ValidateRouteV2.Request)
    original_response = deserialize_message(source(OPERATION / '00752_response'), ValidateRouteV2.Response)
    original_cells = deserialize_message(source(OPERATION / '00745_observer_read_cells_response'), ReadCells.Response)
    last_stamp = original_cells.identity.last_integrated_stamp_ns
    observations = []
    for path in [SOURCE / '00_observer_integrate_observation_request.cdr'] + sorted(CASE.glob('*_camera_observation.cdr')):
        request = deserialize_message(source(path), IntegrateObservation.Request)
        stamp = request.cloud.header.stamp.sec * 10**9 + request.cloud.header.stamp.nanosec
        if stamp <= last_stamp:
            observations.append((request, stamp))
    assert len(observations) == 39
    geometry = source(SOURCE / 'observer/geometry.yaml')
    config_data = yaml.safe_load(source(SOURCE / 'observer/observer.yaml'))
    params = SOURCE / 'parameters.yaml'
    source(params)
    observer_binary = Path(get_package_prefix('uld_occ_observer')) / 'lib/uld_occ_observer/observer_node'
    native_binary = Path(os.environ['JOINT_NATIVE_CERTIFIER_EXECUTABLE'])
    manifest['binaries'] = {str(p): sha(p.read_bytes()) for p in [observer_binary, native_binary]}
    manifest['original_leg_sha256'] = sha(json.dumps(message_to_ordereddict(original.legs[0]), sort_keys=True).encode())
    manifest['original_snapshot_observed_revision'] = original_cells.identity.observed_revision
    save(OUTPUT / 'manifest.json', manifest)
    context, node, executor, native = None, None, None, None
    native_log = (OUTPUT / 'native.log').open('wb')
    try:
        native = subprocess.Popen([str(native_binary), '--ros-args', '--params-file', str(params)],
            env=dict(os.environ, ROS_DOMAIN_ID='184', ROS_LOCALHOST_ONLY='1'), stdout=native_log, stderr=subprocess.STDOUT)
        context = rclpy.context.Context()
        rclpy.init(context=context, domain_id=184)
        node = rclpy.create_node('release_resolution_diagnostic', context=context)
        executor = rclpy.executors.SingleThreadedExecutor(context=context)
        executor.add_node(node)
        validate = node.create_client(ValidateRouteV2, '/validate_route_v2')
        assert validate.wait_for_service(timeout_sec=10.), 'native readiness'
        for resolution in [.10, .02]:
            folder = OUTPUT / ('resolution_' + str(resolution))
            folder.mkdir()
            (folder / 'geometry.yaml').write_bytes(geometry)
            config = dict(config_data, resolution_m=resolution)
            (folder / 'observer.yaml').write_text(yaml.safe_dump(config))
            cfg = ObserverConfiguration.load(folder / 'geometry.yaml', folder / 'observer.yaml')
            log = (folder / 'observer.log').open('wb')
            process = subprocess.Popen([str(observer_binary), '--ros-args', '-p',
                'geometry_file:=' + str(folder / 'geometry.yaml'), '-p',
                'config_file:=' + str(folder / 'observer.yaml'), '-p', 'use_sim_time:=true'],
                env=dict(os.environ, ROS_DOMAIN_ID='184', ROS_LOCALHOST_ONLY='1'), stdout=log, stderr=subprocess.STDOUT)
            def call(client, request, deadline, label, **kwargs):
                record(folder, label + '_request.cdr', request)
                future = client.call_async(request)
                executor.spin_until_future_complete(future, timeout_sec=max(0., deadline - time.monotonic()))
                assert future.done(), label + ' deadline'
                result = future.result()
                record(folder, label + '_response.cdr', result)
                return result
            def capture(label, message):
                record(folder, label + '.cdr', message)
            port = ObserverSnapshotPort(node, configuration=cfg, call=call, recorder=capture)
            integrate = node.create_client(IntegrateObservation, '/uld_occ_observer/integrate_observation')
            clock = node.create_publisher(Clock, '/clock', 10)
            try:
                for client in [integrate, *port.clients.values()]:
                    assert client.wait_for_service(timeout_sec=10.), 'observer readiness'
                deadline = time.monotonic() + 3.
                while clock.get_subscription_count() == 0 and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.01)
                assert clock.get_subscription_count() > 0
                for i, (request, stamp) in enumerate(observations):
                    for _ in range(10):
                        clock.publish(Clock(clock=Time(sec=stamp // 10**9, nanosec=stamp % 10**9)))
                        executor.spin_once(timeout_sec=.01)
                    result = call(integrate, request, time.monotonic() + 10., 'integrate_' + str(i))
                    assert result.receipt.status == 'INTEGRATED', str(result.receipt)
                snap = port.capture(scene_revision='scene', request_id='resolution-control', expected_epoch=1,
                                    expected_revision=None, deadline=time.monotonic() + 30.)
                # Consume asynchronous mandatory lease releases before owner teardown.
                deadline = time.monotonic() + 3.
                while any(not port._released(f) for f in port._pending_releases) and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.01)
                assert all(port._released(f) for f in port._pending_releases)
                states = snap.volume.states
                stats = dict(resolution_m=resolution, dimensions=list(cfg.dimensions_xyz),
                    state_counts={str(k): states.count(k) for k in (0, 1, 2)},
                    observed_revision=snap.source.observed_revision,
                    original_coarse_state_parity=states == bytes(original_cells.states) if resolution == .10 else None)
                summary['owner_snapshots'].append(stats)
                save(OUTPUT / 'summary.json', summary)
                assert snap.source.observed_revision == 39
                if resolution == .10:
                    assert stats['original_coarse_state_parity'], 'coarse O reconstruction mismatch'
                for margin in ([.05] if resolution == .10 else [.05, 0.]):
                    variant = copy.deepcopy(original)
                    scene_index = variant.legs[0].load_state_index
                    baseline = copy.deepcopy(original.load_state_scenes[scene_index])
                    baseline.world.collision_objects = [o for o in baseline.world.collision_objects
                                                       if not o.id.startswith('placement-prior-occ-')]
                    private = build_private_snapshot(snap.surface, baseline, original_response.model_sha256,
                        Transform((0., 0., 0.), (0., 0., 0., 1.)), collision_margin_m=margin, volume=snap.volume)
                    scene = deserialize_message(private.scene_cdr, PlanningScene)
                    variant.load_state_scenes[scene_index] = scene
                    variant.scene_digest = sha(b''.join(serialize_message(s) for s in variant.load_state_scenes))
                    assert message_to_ordereddict(variant.legs[0]) == message_to_ordereddict(original.legs[0])
                    response = call(validate, variant, time.monotonic() + 10., 'native_padding_' + str(margin))
                    result = dict(resolution_m=resolution, padding_m=margin, all_valid=response.all_valid,
                        response_result=response.result, detail=response.detail, legs=[],
                        occ_object_count=sum(o.id.startswith('placement-prior-occ-') for o in scene.world.collision_objects))
                    for leg in response.leg_results:
                        result['legs'].append(dict(edge_id=leg.edge_id, reason=leg.reason,
                            first_invalid_sample=leg.first_invalid_sample, n_samples=leg.n_samples,
                            contacts=[dict(body1=c.body_1, body2=c.body_2, depth_m=c.depth,
                                           position=[c.position.x,c.position.y,c.position.z]) for c in leg.contacts]))
                    summary['variants'].append(result)
                    save(OUTPUT / 'summary.json', summary)
                    assert response.result == 0 and len(response.leg_results) == 1
                    assert response.model_sha256 == original_response.model_sha256
                    assert response.leg_results[0].n_samples == 494
                    if resolution == .10:
                        leg = response.leg_results[0]
                        assert not response.all_valid and leg.first_invalid_sample == 0 and leg.reason == 'COLLISION'
                        assert {(c.body_1, c.body_2) for c in leg.contacts} == {
                            ('placement-prior-occ-voxel-' + str(i), 'suction_contact_frame') for i in range(4)}
            finally:
                manifest['native_exits'].append(dict(resolution_m=resolution, observer_exit=stop(process)))
                log.close()
                node.destroy_publisher(clock)
                for client in [integrate, *port.clients.values()]:
                    node.destroy_client(client)
        for item in manifest['records']:
            assert sha((OUTPUT / item['file']).read_bytes()) == item['sha256']
        manifest.update(capture_complete=True, replay_possible=True, variant_count=len(summary['variants']), status='completed')
    except BaseException as error:
        manifest.update(status='failed', error=repr(error))
        raise
    finally:
        if executor:
            executor.shutdown(timeout_sec=2.)
        if node:
            node.destroy_node()
        if context and context.ok():
            context.shutdown()
        if native:
            manifest['certifier_exit'] = stop(native)
        native_log.close()
        manifest['ended_ns'] = time.time_ns()
        save(OUTPUT / 'manifest.json', manifest)
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
