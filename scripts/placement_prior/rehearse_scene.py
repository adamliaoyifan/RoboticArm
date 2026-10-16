#!/usr/bin/env python3
"""Exclusive unscored rehearsal; optional measured one/two-box cycles."""
import argparse
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--launch-params-file', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--timeout', type=float, default=60.)
    parser.add_argument('--simulate-cycle', action='store_true', help='Run one non-scored measured pick/place cycle with prior')
    parser.add_argument('--cycle-timeout', type=float, default=360.)
    parser.add_argument('--cycle-count', type=int, choices=(1, 2), default=1,
                        help='Measured cycles in the same scene; placed occupancy is retained')
    parser.add_argument('--bootstrap-depth-map', action='store_true')
    parser.add_argument('--bootstrap-depth-targets', default='')
    parser.add_argument('--bootstrap-depth-samples', type=int, choices=range(3,9), default=3)
    parser.add_argument('--acquire-inputs', action='store_true')
    parser.add_argument('--cycle-fault', choices=('', 'pre_pick', 'pre_place'), default='')
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[2]
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    residual = subprocess.run(['pgrep', '-af', 'ros2 launch luggage_gazebo|ign gazeb[o]|gz sim|ros_gz_bridge|parameter_bridge|clock_bridge|camera_bridge'], capture_output=True, text=True)
    if residual.stdout.strip():
        raise RuntimeError('existing sim: '+residual.stdout)
    if os.environ.get('ROS_DOMAIN_ID') != '7':
        raise RuntimeError('requires dedicated ROS_DOMAIN_ID=7')
    from process_guard import dedicated_domain_processes
    domain_residuals=dedicated_domain_processes()
    (out/'domain_before.json').write_text(json.dumps(domain_residuals,indent=2))
    if domain_residuals:
        raise RuntimeError('dedicated ROS domain is occupied: '+str([row['pid'] for row in domain_residuals]))
    params = shlex.split(Path(args.launch_params_file).read_text())
    result = {'scored': False, 'ok': False, 'revision': subprocess.check_output(['git','rev-parse','HEAD'], cwd=repo, text=True).strip(), 'parameters': params}
    (out/'source.diff').write_bytes(subprocess.check_output(['git','diff'], cwd=repo))
    os.environ['ST09_I_CERT_CAPTURE_DIR']=str(out/'cert_boundaries')
    os.environ['ST09_PORTAL_CAPTURE_DIR']=str(out/'portal_boundaries')
    os.environ['ELFIN_SUPPORT_CAPTURE']=str(out/'support_geometry')
    log = (out/'launch.log').open('w')
    launch = subprocess.Popen(['ros2','launch','luggage_gazebo','sim_world.launch.py',*params], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    result['launch_pid']=launch.pid
    Path(os.environ.get('ELFIN_SIM_PIDFILE','/tmp/elfin_humble_sim.pid')).write_text(str(launch.pid))
    start = time.monotonic()
    node = None
    sidecar = None
    bag = None
    sidecar_log = None
    bag_log = None
    try:
        import rclpy
        from rclpy.serialization import serialize_message
        from moveit_msgs.srv import GetPlanningScene
        from moveit_msgs.msg import PlanningSceneComponents
        rclpy.init()
        node = rclpy.create_node('prior_scene_rehearsal')
        client = node.create_client(GetPlanningScene, '/get_planning_scene')
        required = {'/trajectory_selector/select_place_path_v2', '/validate_route_v2'}
        future = None
        while time.monotonic()-start < args.timeout:
            if launch.poll() is not None:
                result['launch_exit_code']=launch.returncode
                raise RuntimeError('launch exited before readiness: '+str(launch.returncode))
            rclpy.spin_once(node, timeout_sec=.1)
            services = {name for name, _ in node.get_service_names_and_types()}
            if client.service_is_ready() and future is None:
                request = GetPlanningScene.Request()
                request.components.components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
                future = client.call_async(request)
            if future is not None and future.done():
                response = future.result()
                ids = {obj.id for obj in response.scene.world.collision_objects}
                (out/'scene.response.cdr').write_bytes(serialize_message(response))
                result.update(objects=sorted(ids), services=sorted(services), clock_publishers=node.count_publishers('/clock'))
                if {'airport_container_real','robot_pedestal'} <= ids and required <= services and result['clock_publishers'] == 1:
                    log.flush()
                    # Scene presence alone precedes the final ACM operation.
                    if 'auto sync_static_scene: static scene synced' in (out/'launch.log').read_text():
                        result['ok'] = True
                        break
                future = None
        if not result['ok']:
            result['reason'] = 'scene_or_service_readiness_timeout'
        elif args.simulate_cycle:
            from luggage_description.runtime_identity import scene_revision_from_path
            options=dict(item.split(':=',1) for item in params)
            scene=options['scene_tf_config']
            sidecar_log=(out/'prior.log').open('w')
            sidecar=subprocess.Popen(['ros2','launch','luggage_planning','placement_prior.launch.py',
                'scene_tf_config:='+scene,'frame_scene_revision:='+scene_revision_from_path(scene),
                'evidence_dir:='+str(out/'prior_boundaries')],stdout=sidecar_log,stderr=subprocess.STDOUT,
                start_new_session=True)
            until=time.monotonic()+20.
            while time.monotonic()<until:
                rclpy.spin_once(node,timeout_sec=.1)
                if '/placement_prior/prepare' in {name for name,_ in node.get_service_names_and_types()}:
                    break
                if sidecar.poll() is not None:raise RuntimeError('prior launch failed')
            else:raise RuntimeError('prior service readiness timeout')
            bag_log=(out/'bag.log').open('w')
            bag=subprocess.Popen(['ros2','bag','record','-o',str(out/'bag'),
                '--max-bag-size','134217728', '/clock','/tf','/tf_static','/joint_states',
                '/monitored_planning_scene','/luggage/cargo_map/surface_2d',
                '/luggage/cargo_map/surface_v2','/motion_planner/last_boundary',
                '/frame_contracts/v2/runtime_identity'],
                stdout=bag_log,stderr=subprocess.STDOUT,start_new_session=True)
            command=['ros2','run','luggage_gazebo','motion_occ_eval_driver.py',
                '--frame-runtime','--frame-peer','motion_occ_driver','--placement-prior',
                '--payload','vacuum','--goto-timeout','60','--observe-pose','pickup_observe_sim',
                '--scene-tf-config',scene,'--out',str(out/'cycle'),'--dump-dir',str(out/'dumps'),
                '--rehearsal',str(out/'cycle'),'--n',str(args.cycle_count)]
            if args.bootstrap_depth_map:
                command.extend(['--bootstrap-depth-map', '--bootstrap-depth-samples',
                                str(args.bootstrap_depth_samples)])
            if args.bootstrap_depth_targets:
                command.extend(['--bootstrap-depth-targets',args.bootstrap_depth_targets])
            if args.acquire_inputs:
                command.append('--prior-acquisition-only')
            if args.cycle_fault:
                command.extend(['--rehearsal-fault',args.cycle_fault])
            result['driver_command']=command
            with (out/'driver.log').open('w') as driver_log:
                driver=subprocess.run(command,stdout=driver_log,stderr=subprocess.STDOUT,timeout=args.cycle_timeout)
            result['startup_ok']=result['ok']
            result['driver_exit_code']=driver.returncode
            result['ok']=driver.returncode == 0
            result['scope']='non-scored functional acquisition; inspect cycle evidence for verdict'

    except Exception as exc:
        result['ok']=False
        result['reason'] = repr(exc)
    finally:
        result['elapsed_s'] = time.monotonic()-start
        (out/'result.json').write_text(json.dumps(result, indent=2))
        for process in (bag,sidecar):
            if process is not None and process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM)
                    process.wait(timeout=10)
        for handle in (bag_log,sidecar_log):
            if handle is not None:handle.close()
        if node is not None:
            node.destroy_node()
            rclpy.try_shutdown()
        with (out/'stop.log').open('w') as stop:
            subprocess.run(['bash',str(repo/'scripts/stop_sim.sh'), '--owned-pgid', str(launch.pid)], cwd=repo, stdout=stop, stderr=subprocess.STDOUT, timeout=60)
        log.close()
    print(json.dumps({key:value for key,value in result.items() if key not in ('parameters','services')}))
    return 0 if result['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
