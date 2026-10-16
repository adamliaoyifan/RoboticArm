#!/usr/bin/env python3
"""Validate and optionally run a frozen captured-consumer plan-only replay.

This entry point does not certify original-target recovery or deployed hardware
configuration. Those gates require their own measured evidence. No motion.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys


FILE_OPTIONS = ('scene_tf', 'robot_poses', 'surface', 'scene',
                'attachment_request', 'attachment_response', 'prior_response')


def prepare(manifest, root, output):
    if manifest.get('schema') != 'placement-prior-functional/1':
        raise ValueError('unsupported functional manifest schema')
    if manifest.get('mode') != 'captured_consumer_plan_only':
        raise ValueError('only captured consumer plan-only replay is supported')
    index, choice = manifest['select_index'], manifest['choice_index']
    if any(type(value) is not int or value < 0 for value in (index, choice)):
        raise ValueError('nonnegative exact Select/choice indices required')
    paths = {key:(root / manifest[key]).resolve() for key in FILE_OPTIONS + ('capture',)}
    boundary = paths['capture'] / 'dumps/typed_service_boundaries'
    ledger = boundary / 'boundaries.jsonl'
    hashes = {(root / name).resolve():digest for name,digest in manifest['artifacts'].items()}

    def verify(path):
        if path not in hashes:
            raise ValueError('required input is not frozen: ' + str(path))
        if hashlib.sha256(path.read_bytes()).hexdigest() != hashes[path]:
            raise ValueError('input hash mismatch: ' + str(path))

    verify(ledger)
    rows = [json.loads(line) for line in ledger.read_text().splitlines() if line.strip()]
    selections = [i for i,row in enumerate(rows)
                  if row['boundary'] == '/trajectory_selector/select_place_path_v2:request']
    selected = selections[index]
    stats = next(row for row in reversed(rows[:selected])
                 if row['boundary'] == '/cargo_map/get_stats:response')
    required = [paths[key] for key in FILE_OPTIONS]
    required += [paths['capture'] / 'dumps/runtime_identity.json',
                 paths['capture'] / ('dumps/box_%02d_paths/candidates.json' % index)]
    for row in (rows[selected], stats):
        path = (boundary / row['file']).resolve()
        if not path.is_relative_to(boundary):
            raise ValueError('boundary file escapes capture directory')
        required.append(path)
    for path in required:
        verify(path)
    for path in hashes:
        verify(path)
    command = [sys.executable, str(Path(__file__).with_name('exact_route_replay.py')),
               '--capture', str(paths['capture']), '--output', str(output / 'native'),
               '--select-index', str(index), '--choice-index', str(choice),
               '--observe-pose', manifest['observe_pose'], '--measure-resources']
    for key in FILE_OPTIONS:
        command.extend(['--' + key.replace('_', '-'), str(paths[key])])
    return command


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--run', action='store_true', help='launch isolated MoveIt, never execution')
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text())
    command = prepare(manifest, args.manifest.resolve().parent, args.output.resolve())
    revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip()
    dirty = subprocess.check_output(['git', 'status', '--porcelain'], text=True)
    if revision != manifest['revision'] or dirty:
        raise ValueError('replay requires the frozen clean runtime revision')
    if args.run and os.environ.get('ROS_DOMAIN_ID') != '8':
        raise ValueError('native replay requires isolated ROS_DOMAIN_ID=8')
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / 'manifest.json').write_bytes(args.manifest.read_bytes())
    (args.output / 'command.json').write_text(json.dumps(command, indent=2))
    result = dict(input_validation='pass', native_replay='not_run',
                  F1='not_evaluated', F2='not_evaluated', revision=revision,
                  scope='frozen captured-consumer replay; no deployment or motion acceptance')
    if args.run:
        with (args.output / 'client.log').open('w') as log:
            process = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
        result['exit_code'] = process.returncode
        report_path = args.output / 'native/result.json'
        native = json.loads(report_path.read_text()) if report_path.exists() else {}
        success = (process.returncode == 0 and native.get('success') is True
                   and native.get('consumer_request_matches_capture') is True
                   and native.get('callback_shutdown_complete') is True)
        result['native_replay'] = 'pass' if success else 'fail'
    (args.output / 'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))
    return 1 if result['native_replay'] == 'fail' else 0


if __name__ == '__main__':
    raise SystemExit(main())
