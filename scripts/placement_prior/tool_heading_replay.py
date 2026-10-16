#!/usr/bin/env python3
"""Compare recorded rigid-tool checks with a local TCP half-turn.

Diagnostic only: does not authorize a different grasp, solve IK, or certify a
route. Keeps box poses, TCP origins, frozen scene, and safety settings unchanged.
"""
import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import threading


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--scene-tf', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    from process_guard import dedicated_domain_processes
    if os.environ.get('ROS_DOMAIN_ID') != '8' or dedicated_domain_processes('8'):
        raise RuntimeError('empty isolated domain 8 required')
    import rclpy
    import yaml
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.serialization import deserialize_message, serialize_message
    from ament_index_python.packages import get_package_share_directory
    from luggage_description.xacro_robot_with_scene_base import expand_and_patch
    from luggage_msgs.srv import CheckPlacementToolV2
    from scipy.spatial.transform import Rotation
    share = Path(get_package_share_directory('luggage_description'))
    xml, _ = expand_and_patch(str(share/'urdf/elfin_s20_with_camera.urdf.xacro'),
                              config_path=str(args.scene_tf), xacro_args=[])
    semantic = (share/'config/S20_with_camera.srdf').read_text().replace(
        'name="elfin_s20_with_camera"', 'name="S20"', 1)
    model_hash = hashlib.sha256((xml + semantic).encode()).hexdigest()
    boundary = args.capture/'boundaries'
    rows = [json.loads(x) for x in (boundary/'boundaries.jsonl').read_text().splitlines()]
    inputs = []
    for row in rows:
        if row['boundary'] == 'collision_request':
            path = boundary/row['file']
            request = deserialize_message(path.read_bytes(), CheckPlacementToolV2.Request)
            if request.model_sha256 != model_hash or len(request.tcp_poses) != 1:
                raise ValueError('requires matching model and one captured tool pose')
            inputs.append((path, request))
    if not 1 <= len(inputs) <= 4:
        raise ValueError('one to four exact inputs required')
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'robot.urdf').write_text(xml)
    (args.output/'robot.srdf').write_text(semantic)
    params = args.output/'parameters.yaml'
    params.write_text(yaml.safe_dump({'placement_tool_check': {'ros__parameters': {
        'robot_description': xml, 'robot_description_semantic': semantic}}}))
    report = dict(scope=__doc__, complete=False, revision=subprocess.check_output(
        ['git','rev-parse','HEAD'],text=True).strip(), dirty=subprocess.check_output(
        ['git','status','--porcelain'],text=True), model_sha256=model_hash, results=[],
        sources={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p,_ in inputs})
    (args.output/'replay_script.py').write_bytes(Path(__file__).read_bytes())
    rclpy.init()
    node = rclpy.create_node('tool_heading_diagnostic')
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    client = node.create_client(CheckPlacementToolV2, '/placement_prior/check_tool')
    log = (args.output/'checker.log').open('w')
    command = ['ros2','run','luggage_planning','placement_tool_check_node',
               '--ros-args','--params-file',str(params)]
    (args.output/'command.json').write_text(json.dumps(command))
    proc = subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        if not client.wait_for_service(timeout_sec=20.):
            raise RuntimeError('checker readiness timeout')
        for index, (path, original) in enumerate(inputs):
            for half_turn in (False, True):
                request = deepcopy(original)
                label = '%02d-%s' % (index, 'half-turn' if half_turn else 'original')
                if half_turn:
                    q = request.tcp_poses[0].orientation
                    rotated = (Rotation.from_quat([q.x,q.y,q.z,q.w]) *
                               Rotation.from_rotvec([0.,0.,3.141592653589793])).as_quat()
                    q.x,q.y,q.z,q.w = map(float, rotated)
                    request.request_id += '-half-turn'
                (args.output/(label+'.request.cdr')).write_bytes(serialize_message(request))
                future = client.call_async(request)
                event = threading.Event()
                future.add_done_callback(lambda unused: event.set())
                if not event.wait(15.):
                    raise RuntimeError('checker timeout')
                reply = future.result()
                (args.output/(label+'.response.cdr')).write_bytes(serialize_message(reply))
                if (reply.request_id != request.request_id or reply.snapshot_id != request.snapshot_id
                        or reply.model_sha256 != model_hash or not reply.complete):
                    raise ValueError('incomplete or mismatched response')
                contacts = sorted(set((c.body_1,c.body_2) for c in reply.contacts))
                report['results'].append(dict(label=label,source=str(path),valid=list(reply.valid),
                    reason=reply.reason,contacts=contacts,service_time_s=reply.service_time_s))
        report['complete'] = True
    except Exception as exc:
        report['error'] = repr(exc)
    finally:
        (args.output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
        with (args.output/'stop.log').open('w') as stop:
            subprocess.run(['bash','scripts/stop_sim.sh','--owned-pgid',str(proc.pid)],
                           stdout=stop,stderr=subprocess.STDOUT,check=True)
        proc.wait(timeout=10)
        executor.shutdown(timeout_sec=5.)
        spin.join(timeout=5.)
        node.destroy_node()
        rclpy.shutdown()
        log.close()
        (args.output/'domain_after.json').write_text(json.dumps(dedicated_domain_processes('8')))
    print(json.dumps({'complete':report['complete'],'results':[
        {'label':r['label'],'valid':r['valid']} for r in report['results']]}))
    return 0 if report['complete'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
