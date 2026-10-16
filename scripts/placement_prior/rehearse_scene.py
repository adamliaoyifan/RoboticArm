#!/usr/bin/env python3
"""Non-scored exclusive sim scene/service rehearsal; never spawns a box."""
import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--launch-params-file', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--timeout', type=float, default=60.)
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[2]
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    residual = subprocess.run(['pgrep', '-af', 'ros2 launch luggage_gazebo|ign gazeb[o]|gz sim|ros_gz_bridge|parameter_bridge|clock_bridge|camera_bridge'], capture_output=True, text=True)
    if residual.stdout.strip():
        raise RuntimeError('existing sim: '+residual.stdout)
    if os.environ.get('ROS_DOMAIN_ID') != '7':
        raise RuntimeError('requires dedicated ROS_DOMAIN_ID=7')
    params = shlex.split(Path(args.launch_params_file).read_text())
    result = {'scored': False, 'ok': False, 'revision': subprocess.check_output(['git','rev-parse','HEAD'], cwd=repo, text=True).strip(), 'parameters': params}
    (out/'source.diff').write_bytes(subprocess.check_output(['git','diff'], cwd=repo))
    log = (out/'launch.log').open('w')
    launch = subprocess.Popen(['ros2','launch','luggage_gazebo','sim_world.launch.py',*params], stdout=log, stderr=subprocess.STDOUT)
    Path(os.environ.get('ELFIN_SIM_PIDFILE','/tmp/elfin_humble_sim.pid')).write_text(str(launch.pid))
    start = time.monotonic()
    node = None
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
                raise RuntimeError('launch exited before readiness')
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
    except Exception as exc:
        result['reason'] = repr(exc)
    finally:
        result['elapsed_s'] = time.monotonic()-start
        (out/'result.json').write_text(json.dumps(result, indent=2))
        if node is not None:
            node.destroy_node()
            rclpy.try_shutdown()
        with (out/'stop.log').open('w') as stop:
            subprocess.run(['bash',str(repo/'scripts/stop_sim.sh')], cwd=repo, stdout=stop, stderr=subprocess.STDOUT, timeout=60)
        log.close()
    print(json.dumps({key:value for key,value in result.items() if key not in ('parameters','services')}))
    return 0 if result['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
