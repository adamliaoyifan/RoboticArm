"""One production build prefix and an immutable source/build provenance receipt."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

from module_layout import prune_relocated_modules

ROOT = Path(__file__).resolve().parents[2]
TARGETS = ('luggage_planning', 'luggage_packing', 'luggage_perception',
           'uld_occ_observer', 'luggage_gazebo')


def source_hashes(package_paths):
    result = {}
    for package in package_paths:
        for path in sorted(package.rglob('*')):
            if (path.is_file() and '__pycache__' not in path.parts
                    and '.pytest_cache' not in path.parts and path.suffix != '.pyc'):
                result[str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', type=Path, default=ROOT / (
        'docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-PRODUCTION-P0/run_01'))
    args = parser.parse_args()
    evidence = args.evidence.resolve()
    evidence.mkdir(parents=True, exist_ok=False)
    selection = ['--base-paths', 'src', 'st09_contracts', '--packages-up-to', *TARGETS]
    listing = subprocess.check_output(['colcon', 'list', *selection], cwd=ROOT, text=True)
    packages = [ROOT / line.split()[1] for line in listing.splitlines() if line.strip()]
    before = source_hashes(packages)
    command = ['colcon', '--log-base', 'log-production', 'build',
               '--build-base', 'build-production', '--install-base', 'install-production',
               *selection, '--executor', 'sequential', '--event-handlers', 'console_direct+',
               '--cmake-force-configure', '--cmake-args', '-DBUILD_TESTING=OFF',
               '-DAMENT_CMAKE_SYMLINK_INSTALL=OFF', '-DCMAKE_BUILD_TYPE=Release']
    manifest = {
        'stage': 'production_build', 'revision': subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'dirty': subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT,
                                         text=True).splitlines(),
        'started_at': datetime.now().astimezone().isoformat(),
        'command': command, 'packages': listing.splitlines(), 'source_before': before,
        'environment': {name: os.environ.get(name) for name in
                        ('AMENT_PREFIX_PATH', 'PYTHONPATH', 'CMAKE_PREFIX_PATH',
                         'CMAKE_BUILD_PARALLEL_LEVEL')},
        'capture_complete': False, 'replay_possible': False,
        'nodes_started': 0, 'teardown': 'not_needed',
    }
    receipt = evidence / 'manifest.json'
    receipt.write_text(json.dumps(manifest, indent=2) + '\n')
    started = time.monotonic()
    try:
        with (evidence / 'build.log').open('w') as log:
            completed = subprocess.run(command, cwd=ROOT, stdout=log,
                                       stderr=subprocess.STDOUT, timeout=1800)
        result = completed.returncode
    except subprocess.TimeoutExpired:
        result = 124
    after = source_hashes(packages)
    changes = sorted(key for key in before.keys() | after.keys() if before.get(key) != after.get(key))
    if result == 0 and not changes:
        try:
            manifest['removed_relocated_modules'] = prune_relocated_modules(ROOT, ROOT/'install-production')
        except Exception as error:
            manifest['module_cleanup_error'] = type(error).__name__+': '+str(error)
            result = 1
    manifest.update(exit_code=result, elapsed_s=time.monotonic()-started,
                    source_after=after, concurrent_changes=changes,
                    capture_complete=True, replay_possible=not changes,
                    passed=result == 0 and not changes)
    receipt.write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps({'passed': manifest['passed'], 'exit_code': result,
                      'elapsed_s': manifest['elapsed_s'], 'changed_inputs': changes,
                      'evidence': str(evidence)}), flush=True)
    return 0 if manifest['passed'] else (result or 1)


if __name__ == '__main__':
    raise SystemExit(main())
