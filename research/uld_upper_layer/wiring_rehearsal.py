#!/usr/bin/env python3
"""Unscored real controller hold and detached ideal-vacuum wiring rehearsal."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import time
import traceback
from assets import sha256
from controller_io import ArmController
from runtime_io import RuntimeIO

ROOT=Path(__file__).resolve().parents[2]
EVIDENCE=Path('/home/adamliao/work/elfin_humble_ws/docs/status/evidence')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-dir',type=Path,required=True)
    parser.add_argument('--evidence',type=Path,required=True)
    args=parser.parse_args();output=args.evidence.resolve()
    if not output.is_relative_to(EVIDENCE):raise ValueError('PRIMARY_EVIDENCE_REQUIRED')
    fixture=json.loads((args.scene_dir/'fixture.json').read_text())
    if not fixture.get('ideal_vacuum'):raise ValueError('EXPLICIT_IDEAL_VACUUM_REQUIRED')
    for name,expected in fixture['source_hashes'].items():
        path=Path(name) if Path(name).is_absolute() else ROOT/name
        if sha256(path)!=expected:raise ValueError('FIXTURE_SOURCE_CHANGED:'+name)
    output.mkdir(parents=True,exist_ok=False)
    source=output/'source';source.mkdir()
    hashes={}
    for path in list(Path(__file__).parent.glob('*.py'))+list(Path(__file__).with_name('native').glob('*')):
        shutil.copy2(path,source/path.name);hashes[path.name]=sha256(path)
    shutil.copy2(args.scene_dir/'fixture.json',output/'fixture.json')
    manifest=dict(scored_units=0,profile='controller_hold_and_detached_vacuum_wiring',
        source_hashes=hashes,started_wall=time.time(),command=list(__import__('sys').argv),
        revision=subprocess.check_output(['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip())
    runtime=RuntimeIO(ROOT,output,args.scene_dir)
    def save(name,value):(output/name).write_text(json.dumps(value,indent=2)+'\n')
    save('manifest.json',manifest)
    try:
        runtime.start();before=runtime.snapshot();save('before.json',before)
        first=runtime.toggle_vacuum(False)
        second=runtime.toggle_vacuum(False)
        if second['processed_commands']<=first['processed_commands']:
            raise RuntimeError('DETACH_ACK_NOT_PROCESSED')
        q=before['q'];trajectory=dict(positions=[q,q],velocities=[[0.]*6,[0.]*6],time_from_start=[0.,1.])
        save('commanded_trajectory.json',trajectory)
        def monitor(observation):
            if observation['capture_error']:return 'CAPTURE_ERROR:'+observation['capture_error']
            state=observation['vacuum']
            if state is None or state['attached']:return 'VACUUM_EXPECTED_DETACHED'
            if state['pose_command_count']!=0:return 'DETACHED_PAYLOAD_POSE_COMMAND'
        result=ArmController(runtime).execute(trajectory,monitor,30.)
        after=runtime.snapshot();save('after.json',after)
        drift=max(abs(a-b) for a,b in zip(after['q'],q))
        if drift>=.001:raise RuntimeError('JOINT_HOLD_DRIFT:'+str(drift))
        save('summary.json',dict(status='pass',scope='wiring_only_no_attachment_or_upper_action',
            scored_units=0,joint_hold_max_drift_rad=drift,detached_pose_commands=0,
            processed_detach_commands=second['processed_commands'],controller_error_code=result['error_code']))
    except BaseException:
        (output/'error.txt').write_text(traceback.format_exc());raise
    finally:
        save('terminal_observation.json',runtime.snapshot());runtime.stop()
        manifest['ended_wall']=time.time();save('manifest.json',manifest)


if __name__=='__main__':main()
