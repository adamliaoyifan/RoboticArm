#!/usr/bin/env python3
"""Source-bound ideal-input geometry experiment. Never drives a robot."""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import platform
import shutil
import subprocess
import traceback
from assets import load, sha256
from candidates import generate_actions
from ideal_snapshot import capture_fixture
from planning_types import UpperConfig

ROOT=Path(__file__).resolve().parents[2]
EVIDENCE=Path('/home/adamliao/work/elfin_humble_ws/docs/status/evidence')


def save(path,value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-dir',required=True,type=Path)
    parser.add_argument('--evidence',required=True,type=Path)
    parser.add_argument('--config',type=Path)
    args=parser.parse_args()
    output=args.evidence.resolve()
    if not output.is_relative_to(EVIDENCE):raise ValueError('evidence must be in primary workspace')
    output.mkdir(parents=True,exist_ok=False)
    config=UpperConfig.load(args.config)
    sources=output/'source';sources.mkdir()
    hashes={}
    for source in sorted(Path(__file__).parent.glob('*.py'))+list(Path(__file__).parent.glob('*.yaml')):
        shutil.copy2(source,sources/source.name);hashes[str(source.relative_to(ROOT))]=sha256(source)
    shutil.copy2(args.scene_dir/'fixture.json',output/'fixture.json')
    manifest=dict(started_at=datetime.now(timezone.utc).isoformat(),python=platform.python_version(),
        revision=subprocess.check_output(['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip(),
        dirty_paths=subprocess.check_output(['git','-C',str(ROOT),'status','--short'],text=True).splitlines(),
        config=asdict(config),config_revision=config.revision,source_hashes=hashes,
        scene_source='ideal_sim_fixture',capture_complete=False)
    save(output/'manifest.json',manifest)
    try:
        assets=load(ROOT)
        snapshot,box=capture_fixture(args.scene_dir,assets,config)
        save(output/'inputs.json',dict(snapshot=asdict(snapshot),box=asdict(box),
                                     tool=[b.record() for b in assets['tool']],asset_hashes=assets['hashes']))
        actions,report=generate_actions(assets,snapshot,box,config)
        save(output/'candidates.json',[asdict(a) for a in actions])
        report.update(candidate_count=len(actions),full_arm='not_evaluated',
                      connected_trajectory='not_evaluated',physical_execution='not_evaluated')
        save(output/'summary.json',report)
        manifest['capture_complete']=True
        print(json.dumps(report,indent=2)[:4500])
    except BaseException:
        (output/'error.txt').write_text(traceback.format_exc())
        raise
    finally:
        manifest['ended_at']=datetime.now(timezone.utc).isoformat()
        save(output/'manifest.json',manifest)


if __name__=='__main__':main()
