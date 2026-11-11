#!/usr/bin/env python3
"""Bounded standalone fixture settling check; no arm motion or placement scoring."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
COORD = Path('/home/adamliao/work/elfin_humble_ws')


def decode_messages(raw):
    decoder = json.JSONDecoder()
    index = 0
    while index < len(raw):
        if raw[index].isspace():
            index += 1
            continue
        value, index = decoder.raw_decode(raw, index)
        yield value


def measure(spec, messages):
    initial = {b['name']: np.asarray(b['pose_world'][:3]) for b in spec['boxes']}
    maxima = {name: dict(samples=0, horizontal_m=0., vertical_m=0., angle_rad=0.) for name in initial}
    times = []
    for message in messages:
        stamp = message.get('header', {}).get('stamp', {})
        stamp_s = float(stamp.get('sec', 0)) + float(stamp.get('nsec', 0))*1e-9
        times.append(stamp_s)
        for pose in message.get('pose', []):
            name = pose.get('name', '')
            if name not in initial:
                continue
            p = pose['position']
            delta = np.array([p.get(k,0.) for k in ('x','y','z')])-initial[name]
            q = pose['orientation']
            rotation = Rotation.from_quat([q.get(k, 0.) for k in ('x','y','z')] + [q.get('w',1.)])
            baseline = next(b for b in spec['boxes'] if b['name'] == name)['pose_world'][3:]
            angle = (rotation * Rotation.from_euler('xyz', baseline).inv()).magnitude()
            m = maxima[name]
            m['samples'] += 1
            m['horizontal_m'] = max(m['horizontal_m'], float(np.linalg.norm(delta[:2])))
            m['vertical_m'] = max(m['vertical_m'], abs(float(delta[2])))
            m['angle_rad'] = max(m['angle_rad'], float(angle))
    end = max(times, default=0.)
    complete = end >= 4.9 and all(m['samples'] >= 20 for m in maxima.values())
    passed = complete and all(m['horizontal_m'] <= .003 and m['vertical_m'] <= .010
                              and m['angle_rad'] <= .01 for m in maxima.values())
    return dict(pass_=passed, capture_complete=complete, max_sim_stamp_s=end, models=maxima,
                thresholds=dict(horizontal_m=.003, vertical_m=.010, angle_rad=.01,
                                min_final_sim_stamp_s=4.9, min_samples_per_box=20))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-dir', type=Path, required=True)
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--wall-timeout', type=float, default=180.)
    args = parser.parse_args()
    out = args.evidence.resolve()
    expected_root = COORD/'docs/status/evidence'
    if not out.is_relative_to(expected_root):
        parser.error('evidence must live in the primary workspace docs/status/evidence')
    out.mkdir(parents=True, exist_ok=False)
    for name in ('standalone.sdf','overlay.sdf','fixture.json'):
        shutil.copy2(args.scene_dir/name, out/name)
    revision = subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    dirty = subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).splitlines()
    (out/'dirty_paths.txt').write_text('\n'.join(dirty)+'\n')
    env = dict(os.environ, IGN_PARTITION=f'uld_upper_scene_{os.getpid()}', ROS_DOMAIN_ID='7',
               IGN_HOMEDIR=str(out/'ign_home'))
    spec = json.loads((out/'fixture.json').read_text())
    iterations = int(round(5.0/spec['physics_step_s']))+1
    command = ['ign','gazebo','-s','-r','--iterations',str(iterations),'-v','3',str(out/'standalone.sdf')]
    snapshot = out/'source_snapshot'
    snapshot.mkdir()
    hashes = {}
    for source in sorted((ROOT/'research/uld_upper_layer').glob('*.py')) + [ROOT/'src/luggage_gazebo/launch/sim_world.launch.py']:
        shutil.copy2(source, snapshot/source.name)
        hashes[str(source.relative_to(ROOT))] = hashlib.sha256(source.read_bytes()).hexdigest()
    manifest = dict(revision=revision, dirty_count=len(dirty), source_hashes=hashes,
                    wall_timeout_s=args.wall_timeout, command=command,
                    partition=env['IGN_PARTITION'], started_wall=time.time(),
                    scope='gravity stability only, no robot or pushing', replay_possible=True,
                    capture_complete=False, teardown='pending')
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    proc = subscriber = None
    summary = dict(pass_=False, reason='not_started')
    try:
        with (out/'poses.jsonl').open('w') as poses, (out/'topic.log').open('w') as topiclog, (out/'gazebo.log').open('w') as log:
            subscriber = subprocess.Popen(['ign','topic','-e','--json-output','-t',
                                           '/world/airport_loading/pose/info'],
                                          stdout=poses,stderr=topiclog,env=env,start_new_session=True)
            proc = subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
            (out/'launch.pid').write_text(str(proc.pid)+'\n')
            deadline = time.monotonic()+args.wall_timeout
            while proc.poll() is None:
                if time.monotonic() >= deadline:
                    raise TimeoutError(f'Gazebo did not finish 5 simulation seconds within {args.wall_timeout} wall seconds')
                if (out/'poses.jsonl').stat().st_size > 80_000_000:
                    raise RuntimeError('capture size limit exceeded')
                time.sleep(.2)
            subscriber.send_signal(signal.SIGINT)
            subscriber.wait(timeout=5)
        summary = measure(json.loads((out/'fixture.json').read_text()),
                          decode_messages((out/'poses.jsonl').read_text()))
        summary['gazebo_returncode'] = proc.returncode
        summary['pass_'] = summary['pass_'] and proc.returncode == 0
        manifest['capture_complete'] = summary['capture_complete']
    except Exception as exc:
        summary = dict(pass_=False, reason=f'{type(exc).__name__}: {exc}', capture_complete=False)
    finally:
        # Persist boundary inputs and any failure before teardown.
        (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        if subscriber and subscriber.poll() is None:
            subscriber.send_signal(signal.SIGINT)
            subscriber.wait(timeout=5)
        if proc:
            result = subprocess.run([str(COORD/'scripts/stop_sim.sh'),'--owned-pgid',str(proc.pid),
                                     '--pidfile',str(out/'launch.pid'),'--wait','3'],
                                    capture_output=True,text=True)
            (out/'teardown.log').write_text(result.stdout+result.stderr)
            manifest['teardown'] = 'stopped' if result.returncode == 0 else 'failed'
        manifest['ended_wall'] = time.time()
        manifest['artifacts'] = sorted(str(p.relative_to(out)) for p in out.rglob('*') if p.is_file())
        (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps({k:v for k,v in summary.items() if k != 'models'},indent=2))
    print(f'Evidence: {out}')
    return 0 if summary['pass_'] and manifest['teardown']=='stopped' else 1


if __name__ == '__main__':
    raise SystemExit(main())
