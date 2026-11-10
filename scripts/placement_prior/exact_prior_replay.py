#!/usr/bin/env python3
"""Replay measured Prepare inputs through isolated production prior services.

Recorded scene/volume/publications are supplied by read-only fixture services.
The owner's confirmation-required setting is an explicit replay assumption,
not a fresh live owner measurement. No MoveIt execution, simulator or motion.
"""
import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import threading
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--prepare-index', type=int, default=0)
    parser.add_argument('--surface-directory', type=Path, required=True)
    parser.add_argument('--scene-tf', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--requested-slot', type=Path,
                        help='Explicit PoseContractV2 CDR target for a derived same-target experiment; preserves measured detection')
    parser.add_argument('--negative-checks', action='store_true',
                        help='Inject stale revision and unconfirmed-map negatives after cold/cache checks')
    parser.add_argument('--prefetch-template', type=Path,
                        help='Explicit hypothetical box_wdh/grasps_flat JSON; wait for automatic background preparation')
    parser.add_argument('--foreground-joints', type=Path,
                        help='Captured JointState CDR applied only after template completion; derived motion-context check')
    args = parser.parse_args()
    if args.foreground_joints and not args.prefetch_template:
        parser.error('foreground-joints requires prefetch-template')
    from process_guard import dedicated_domain_processes
    if os.environ.get('ROS_DOMAIN_ID') != '8' or dedicated_domain_processes('8'):
        raise RuntimeError('empty dedicated ROS domain 8 required')
    if args.prepare_index < 0:
        parser.error('prepare-index must be nonnegative')
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.serialization import deserialize_message, serialize_message
    from std_msgs.msg import String
    from rcl_interfaces.srv import GetParameters
    from rcl_interfaces.msg import ParameterValue
    from moveit_msgs.msg import PlanningScene
    from moveit_msgs.srv import GetPlanningScene
    from luggage_msgs.msg import SurfaceContractV2, PoseContractV2
    from luggage_msgs.srv import PreparePlacementPriorV2, GetCargoVolumeV2
    from luggage_planning.prior_resources import ResourceWindow
    boundary = args.capture / 'prior_boundaries'
    rows = [json.loads(line) for line in (boundary / 'boundaries.jsonl').read_text().splitlines()]
    starts = [i for i, row in enumerate(rows) if row['boundary'] == 'prepare_request']
    start = starts[args.prepare_index]
    end = starts[args.prepare_index + 1] if args.prepare_index + 1 < len(starts) else len(rows)
    segment = rows[start:end]
    inputs = {}
    def read(path, message_type):
        data = path.read_bytes()
        inputs[str(path.resolve())] = hashlib.sha256(data).hexdigest()
        return deserialize_message(data, message_type)
    def captured(name, message_type):
        return read(boundary / next(row['file'] for row in segment if row['boundary'] == name), message_type)
    request = captured('prepare_request', PreparePlacementPriorV2.Request)
    if args.requested_slot:
        request.slots = [read(args.requested_slot, PoseContractV2)]
    historical = captured('prepare_response', PreparePlacementPriorV2.Response)
    scene = captured('baseline_scene', PlanningScene)
    volume_request = captured('volume_request', GetCargoVolumeV2.Request)
    volume = captured('volume_response', GetCargoVolumeV2.Response)
    typed = read(args.surface_directory / 'surface_v2.cdr', SurfaceContractV2)
    legacy = read(args.surface_directory / 'surface_2d.cdr', String)
    confirmation = json.loads(legacy.data)
    scene_revision = hashlib.md5(args.scene_tf.read_bytes()).hexdigest()
    inputs[str(args.scene_tf.resolve())] = hashlib.sha256(args.scene_tf.read_bytes()).hexdigest()
    if (not request.measured or not volume.success or not historical.measured
            or typed.map_revision != request.expected_map_revision
            or volume.map_revision != typed.map_revision
            or typed.identity.scene_revision != scene_revision
            or volume.scene_revision != scene_revision
            or volume.geometry_hash != typed.geometry_hash
            or request.geometry_hash != typed.geometry_hash
            or confirmation['map_revision'] != typed.map_revision
            or confirmation['acquisition_stamp_ns'] != volume.acquisition_stamp_ns):
        raise ValueError('captured prior/surface/volume identity mismatch')
    if not 0 < request.budget_s <= 10 or not 0 < request.candidate_limit <= 32:
        raise ValueError('captured preparation exceeds bounded experiment policy')
    args.output.mkdir(parents=True, exist_ok=False)
    report = dict(success=False, scored=False, F2='not_evaluated', performance='not_evaluated',
        scope='exact measured input replay via read-only fixture providers',
        assumptions=['require_confirmed_map=true fixture owner setting; not a live-owner read'],
        inputs=inputs, revision=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        dirty=subprocess.check_output(['git','status','--porcelain'],text=True), results=[])
    if args.requested_slot:
        report['scope'] = 'derived explicit-target experiment with unchanged measured detection; not exact historical request replay'
        report['request_changes'] = ['slots replaced by one explicit target']
    (args.output / 'replay_script.py').write_bytes(Path(__file__).read_bytes())
    rclpy.init()
    node = rclpy.create_node('recorded_prior_inputs')
    counts = dict(scene=0, volume=0, parameters=0)
    def get_scene(query, reply):
        counts['scene'] += 1
        reply.scene = scene
        return reply
    def get_volume(query, reply):
        counts['volume'] += 1
        if query != volume_request:
            raise ValueError('prior changed exact captured volume query')
        return volume
    def get_parameters(query, reply):
        counts['parameters'] += 1
        if query.names != ['require_confirmed_map']:
            raise ValueError('unexpected fixture parameter query')
        reply.values = [ParameterValue(type=1, bool_value=True)]
        return reply
    node.create_service(GetPlanningScene, '/get_planning_scene', get_scene)
    node.create_service(GetCargoVolumeV2, '/cargo_map/get_volume_v2', get_volume)
    node.create_service(GetParameters, '/recorded_occ/get_parameters', get_parameters)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    typed_pub = node.create_publisher(SurfaceContractV2, '/luggage/cargo_map/surface_v2', qos)
    legacy_pub = node.create_publisher(String, '/luggage/cargo_map/surface_2d', qos)
    client = node.create_client(PreparePlacementPriorV2, '/placement_prior/prepare')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    command = ['ros2', 'launch', 'luggage_planning', 'placement_prior.launch.py',
        'scene_tf_config:=' + str(args.scene_tf), 'frame_scene_revision:=' + scene_revision,
        'map_parameter_service:=/recorded_occ/get_parameters',
        'evidence_dir:=' + str(args.output / 'boundaries')]
    if args.prefetch_template:
        from luggage_planning.placement_prior_prefetch import PrefetchTemplate
        data=args.prefetch_template.read_bytes()
        template=json.loads(data)
        validated=PrefetchTemplate(template['box_wdh'],template['grasps_flat'])
        inputs[str(args.prefetch_template.resolve())]=hashlib.sha256(data).hexdigest()
        command.extend(['prefetch_enabled:=true',
            'prefetch_box_wdh:='+json.dumps(list(validated.box_wdh)),
            'prefetch_grasps_flat:='+json.dumps(template['grasps_flat'])])
        report['scope']='functional automatic template prefetch followed by genuine measured input; not performance scoring'
    (args.output / 'command.json').write_text(json.dumps(command, indent=2))
    log = (args.output / 'prior.log').open('w')
    proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    resource = None
    label = None
    try:
        if not client.wait_for_service(timeout_sec=30.):
            raise RuntimeError('prior startup timeout')
        deadline = time.monotonic() + 10.
        while typed_pub.get_subscription_count() < 1 or legacy_pub.get_subscription_count() < 1:
            if time.monotonic() > deadline:
                raise RuntimeError('surface subscriber readiness timeout')
            time.sleep(.02)
        typed_pub.publish(typed)
        legacy_pub.publish(legacy)
        time.sleep(.2)
        if args.prefetch_template:
            # Observe the real timer completion, never manufacture a cache entry.
            deadline=time.monotonic()+25.
            background=None
            journal=args.output/'boundaries/boundaries.jsonl'
            while time.monotonic()<deadline and background is None:
                if proc.poll() is not None:raise RuntimeError('prior exited during prefetch')
                if journal.exists():
                    for line in journal.read_text().splitlines():
                        try:row=json.loads(line)
                        except json.JSONDecodeError:continue
                        if row['boundary']!='prepare_response':continue
                        candidate=deserialize_message((journal.parent/row['file']).read_bytes(),PreparePlacementPriorV2.Response)
                        if not candidate.measured:
                            background=candidate
                            break
                if background is None:time.sleep(.05)
            if background is None:raise TimeoutError('automatic prefetch did not complete')
            (args.output/'template.response.cdr').write_bytes(serialize_message(background))
            report['template']={'success':background.success,'reason':background.reason,
                'choices':len(background.choices),'elapsed_s':background.elapsed_s}
            if not background.success:raise ValueError('template prefetch failed: '+background.reason)
            if args.foreground_joints:
                from sensor_msgs.msg import JointState
                scene.robot_state.joint_state=read(args.foreground_joints,JointState)
                report['derived_scene_change']='only robot_state.joint_state replaced after template completion'
        labels = ['after_prefetch' if args.prefetch_template else 'cold', 'cached'] + (['stale_revision', 'unconfirmed'] if args.negative_checks else [])
        for label in labels:
            submitted = deepcopy(request)
            if label == 'stale_revision':
                submitted.expected_map_revision += 1
            if label == 'unconfirmed':
                invalidated = dict(confirmation, occupancy_confirmed=False)
                legacy_pub.publish(String(data=json.dumps(invalidated)))
                time.sleep(.2)
            (args.output / (label + '.request.cdr')).write_bytes(serialize_message(submitted))
            with ResourceWindow([os.getpid()], [proc.pid]) as resource:
                started = time.monotonic()
                future = client.call_async(submitted)
                ready = threading.Event()
                future.add_done_callback(lambda unused: ready.set())
                if not ready.wait(20.):
                    raise RuntimeError('bounded Prepare call timeout')
                response = future.result()
                elapsed = time.monotonic() - started
            (args.output / (label + '.resources.json')).write_text(json.dumps(resource.result, indent=2))
            (args.output / (label + '.response.cdr')).write_bytes(serialize_message(response))
            outcome = dict(label=label, success=response.success, reason=response.reason,
                           cache_hit=response.cache_hit, service_elapsed_s=response.elapsed_s,
                           elapsed_s=elapsed, choice_count=len(response.choices))
            report['results'].append(outcome)
            if label in ('stale_revision', 'unconfirmed'):
                expected = 'snapshot identity mismatch' if label == 'stale_revision' else 'NO_CONFIRMED_OCC_SNAPSHOT'
                outcome['injected_negative'] = True
                if response.success or response.choices or expected not in response.reason:
                    raise ValueError('negative input was not refused for its expected reason')
                continue
            if not response.success or not response.choices:
                raise ValueError('prior preparation failed: ' + response.reason)
            if response.model_sha256 != historical.model_sha256:
                raise ValueError('native prior model differs from captured model')
            if (label in ('cold','after_prefetch') and response.cache_hit) or (label == 'cached' and not response.cache_hit):
                raise ValueError('unexpected cache state')
        report['success'] = True
    except Exception as exc:
        report['error'] = repr(exc)
    finally:
        if resource is not None and hasattr(resource, 'result'):
            (args.output / (label + '.resources.json')).write_text(json.dumps(resource.result, indent=2))
        report['provider_calls'] = counts
        (args.output / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
        subprocess.run(['bash','scripts/stop_sim.sh','--owned-pgid',str(proc.pid)],
            stdout=(args.output/'stop.log').open('w'),stderr=subprocess.STDOUT,check=True)
        proc.wait(timeout=10)
        log.close()
        executor.shutdown(timeout_sec=5.)
        spin.join(timeout=5.)
        node.destroy_node()
        rclpy.shutdown()
        (args.output / 'domain_after.json').write_text(json.dumps(dedicated_domain_processes('8')))
    print(json.dumps({k:v for k,v in report.items() if k != 'inputs'}))
    return 0 if report['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
