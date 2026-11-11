"""Non-scored native attachment/observation hold for a release carriage.

No release or joint motion is requested. This is a startup/capture gate before
withdrawal calibration, not a physical stability or whole-action verdict.
"""
import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import shutil
import subprocess
import threading
import time
import traceback

from assets import sha256
from execution_policy import ExecutionPolicy
from transport_capture import JsonTopicCapture

ROOT = Path(__file__).resolve().parents[2]
COORD = Path('/home/adamliao/work/elfin_humble_ws')


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def assert_sim_slot_free():
    for line in subprocess.check_output(['ps', '-eo', 'pid,comm,args'], text=True).splitlines()[1:]:
        fields = line.split(None, 2)
        if len(fields) < 3 or fields[1] in ('bash', 'sh', 'python3', 'python', 'timeout'):
            continue
        if any(name in fields[2] for name in ('ign gazebo', 'ign-gazebo-server', 'gz sim',
                'ros2 launch luggage_gazebo', 'sim_world.launch.py', 'parameter_bridge')):
            raise RuntimeError('SIM_SLOT_OCCUPIED:' + line[:240])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-dir', type=Path, required=True)
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--policy', type=Path)
    args = parser.parse_args()
    out = args.evidence.resolve()
    if not out.is_relative_to(COORD / 'docs/status/evidence'):
        raise ValueError('PRIMARY_EVIDENCE_REQUIRED')
    policy = ExecutionPolicy.load(args.policy)
    spec = json.loads((args.scene_dir / 'release_scene.json').read_text())
    world = args.scene_dir / 'release.sdf'
    if sha256(world) != spec['world_sha256'] or sha256(spec['native_plugin']) != spec['native_plugin_sha256']:
        raise ValueError('RELEASE_SCENE_CHANGED')
    assert_sim_slot_free()
    out.mkdir(parents=True, exist_ok=False)
    source = out / 'source'; source.mkdir()
    hashes = {}
    for path in list(Path(__file__).parent.glob('*.py')) + list(Path(__file__).parent.glob('*.yaml')) + list(
            Path(__file__).with_name('native').glob('*')):
        if path.is_file():
            shutil.copyfile(path, source / path.name); hashes[path.name] = sha256(path)
    shutil.copyfile(world, out / 'release.sdf')
    shutil.copyfile(args.scene_dir / 'release_scene.json', out / 'release_scene.json')
    env = dict(os.environ, IGN_PARTITION='uld_upper_release_rehearsal', ROS_DOMAIN_ID='7')
    manifest = dict(profile='unscored_native_release_carriage_attached_hold', scored_units=0,
        revision=subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip(),
        dirty_count=len(subprocess.check_output(['git', '-C', str(ROOT), 'status', '--porcelain'], text=True).splitlines()),
        command=__import__('sys').argv, environment={k: env[k] for k in ('IGN_PARTITION', 'ROS_DOMAIN_ID')},
        source_hashes=hashes, started_wall=time.time(), hold_s=policy.stability_dwell_s,
        execution_policy=asdict(policy), execution_policy_revision=policy.revision,
        capture_complete=False, replay_possible=False, teardown_complete=False)
    save(out / 'manifest.json', manifest)
    lock = threading.Lock(); metrics = dict(samples=0, max_gap_s=0., contacts_available=0,
        invalid_hold_samples=0)
    latest = None; first = None; failure = None
    def accept(row):
        nonlocal latest, first
        try:
            native = json.loads(row['data'])
        except Exception:
            save(out / 'bad_native_row.json', row)
            raise
        stamp = native['sim_time_s']
        with lock:
            if latest is not None:
                gap = stamp - latest['sim_time_s']
                if gap <= 0:
                    save(out / 'bad_native_row.json', row)
                    raise ValueError('NATIVE_STAMP_NOT_INCREASING')
                metrics['max_gap_s'] = max(metrics['max_gap_s'], gap)
            latest = native
            if first is None:
                first = native
            metrics['samples'] += 1
            metrics['contacts_available'] += bool(native.get('contacts_component_available'))
            metrics['invalid_hold_samples'] += not (native.get('ready') and native.get('attached')
                and native.get('attachment_joint_component_present')
                and native.get('pose_command_count') == 0
                and not native.get('payload_pose_command_component_present'))
        return stamp
    capture = JsonTopicCapture(env, out, 'native', '/upper_layer/vacuum/state', accept,
        max_bytes=policy.max_case_bytes - policy.failure_capture_reserve_bytes)
    process = None; log = None; result = None
    try:
        capture.start()
        log = (out / 'launch.log').open('w')
        process = subprocess.Popen(['ign', 'gazebo', '-s', '-r', '-v', '3', str(world)],
            env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        (out / 'launch.pid').write_text(str(process.pid))
        Path('/tmp/elfin_upper_release_sim.pid').write_text(str(process.pid))
        deadline = time.monotonic() + min(60., policy.wall_budget_s)
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError('RELEASE_SIM_STARTUP_EXITED')
            if capture.error:
                raise RuntimeError(capture.error)
            with lock:
                current = latest; beginning = first
            if current and beginning and current['sim_time_s'] - beginning['sim_time_s'] >= policy.stability_dwell_s:
                break
            time.sleep(.02)
        else:
            raise TimeoutError('RELEASE_NATIVE_READINESS_TIMEOUT')
        with lock:
            save(out / 'first_native.json', first); save(out / 'last_native.json', latest)
            observation = dict(metrics)
        info = subprocess.run(['ign', 'topic', '-i', '-t', '/upper_layer/vacuum/state'],
            env=env, capture_output=True, text=True, timeout=10)
        (out / 'topic_info.txt').write_text(info.stdout + info.stderr)
        clock = subprocess.run(['ign', 'topic', '-i', '-t', '/world/airport_loading/clock'],
            env=env, capture_output=True, text=True, timeout=10)
        (out / 'clock_info.txt').write_text(clock.stdout + clock.stderr)
        checks = dict(ready=current['ready'], attached=current['attached'],
            native_constraint=current['attachment_joint_component_present'],
            native_creation_once=current['attachment_create_count'] == 1,
            no_detach=current['attachment_detach_count'] == 0,
            no_box_commands=current['pose_command_count'] == 0 and not current['payload_pose_command_component_present'],
            joint_names=current['joint_names'] == ['release_x', 'release_z'],
            complete_joints=current['joint_positions_complete'] and len(current['q']) == 2,
            lower_set=set(current['observed_model_poses_world']) == {
                b['name'] for b in json.loads((args.scene_dir / 'source_fixture.json').read_text())['boxes']},
            native_steps=observation['max_gap_s'] <= .001 + 1e-9,
            whole_hold_attached=observation['invalid_hold_samples'] == 0,
            one_native_publisher=info.returncode == 0 and info.stdout.split('Subscribers')[0].count('ignition.msgs.StringMsg') == 1,
            one_clock_publisher=clock.returncode == 0 and clock.stdout.split('Subscribers')[0].count('ignition.msgs.Clock') == 1,
            all_contacts_available=observation['contacts_available'] == observation['samples'])
        result = dict(checks=checks, valid=all(checks.values()), metrics=observation,
            physical_release='not_evaluated', controller_motion='not_evaluated', scored_units=0)
        if not result['valid']:
            raise RuntimeError('RELEASE_ATTACHED_HOLD_GATE_FAILED')
    except Exception as exc:
        failure = dict(error=str(exc), traceback=traceback.format_exc())
        save(out / 'failure.json', failure)
    finally:
        # Preserve the complete bounded startup unit before owned teardown.
        capture.stop()
        with lock:
            save(out / 'capture_metrics.json', dict(metrics, error=capture.error))
        if result is not None:
            save(out / 'result.json', result)
        manifest.update(capture_complete=capture.count > 0 and not capture.error,
            replay_possible=capture.count > 0 and not capture.error)
        save(out / 'manifest.json', manifest)
        if process is not None:
            stopped = subprocess.run(['bash', str(COORD / 'scripts/stop_sim.sh'), '--owned-pgid',
                str(process.pid), '--pidfile', '/tmp/elfin_upper_release_sim.pid'], capture_output=True, text=True, timeout=30)
            (out / 'teardown.log').write_text(stopped.stdout + stopped.stderr)
            process.wait(timeout=5)
            manifest['teardown_complete'] = stopped.returncode == 0
        if log:
            log.close()
        manifest.update(ended_wall=time.time(), failure=failure)
        save(out / 'manifest.json', manifest)
    if failure:
        raise RuntimeError(failure['error'])
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
