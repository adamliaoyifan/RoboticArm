#!/usr/bin/env python3
"""Classify the 316 regression node ids for the gen6 frozen-install deployment.

Per node we record a rerun or reuse reason (review ruling #3 of 2026-1030_1539):
rerun when the test depends on ament/IDL/config/service artifacts or loads ROS
runtime from the install; reuse when the product modules it exercises are
byte-identical between the source tree and the frozen install (verified here
against the install manifest, not assumed). Nothing is silently dropped: every
one of the 316 ids lands in exactly one bucket.
"""
import argparse
import hashlib
import json
import re
from pathlib import Path

RERUN_MARKERS = (
    re.compile(r'^\s*(?:import|from)\s+luggage_planning\.planning_contract', re.M),
    re.compile(r'^\s*(?:import|from)\s+luggage_msgs', re.M),
    re.compile(r'get_package_share_directory|ament_index', re.M),
    re.compile(r'rclpy\.(init|create_node)|create_client|create_service', re.M),
    re.compile(r'^\s*(?:import|from)\s+launch', re.M),
    re.compile(r'subprocess', re.M),
)


def file_text(worktree, classname):
    """Resolve a junit classname (module or module.TestClass) to its file text.

    Drops trailing components (the pytest test class) until an existing module
    file is found; returns None only when nothing resolves, which callers must
    treat as a hard error — never as evidence for reuse.
    """
    parts = classname.split('.')
    while parts:
        for base in (worktree, worktree/'src'):
            path = base.joinpath(*parts)
            if path.suffix != '.py':
                path = path.with_suffix('.py')
            if path.exists():
                return path.read_text()
        parts = parts[:-1]
    return None


def install_shas(install_manifest):
    return {line.split('  ', 1)[1].strip(): line.split('  ', 1)[0]
            for line in Path(install_manifest).read_text().splitlines() if line.strip()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--catalog', required=True)
    parser.add_argument('--worktree', required=True)
    parser.add_argument('--install-manifest', required=True)
    parser.add_argument('--out-json', required=True)
    parser.add_argument('--out-args', required=True)
    args = parser.parse_args()

    catalog = json.loads(Path(args.catalog).read_text())
    node_ids = catalog['regression_316']['ids']
    worktree = Path(args.worktree)
    shas = install_shas(args.install_manifest)

    # Global byte-identity verification: every product python module in the
    # worktree src must appear with the same sha256 in the install manifest.
    verified, missing = 0, []
    for package in ('luggage_perception', 'luggage_packing', 'luggage_planning'):
        for source in (worktree/'src'/package/package).rglob('*.py'):
            rel = source.relative_to(worktree/'src'/package/package)
            installed = (f'./{package}/local/lib/python3.10/dist-packages/'
                         f'{package}/{rel}')
            digest = hashlib.sha256(source.read_bytes()).hexdigest()
            if shas.get(installed) == digest:
                verified += 1
            else:
                missing.append(f'{package}/{rel}')
    if missing:
        raise RuntimeError(f'install byte-identity failed for {len(missing)} modules, '
                           f'e.g. {missing[:3]}')

    rerun, reuse, cache = [], [], {}
    for node in node_ids:
        classname = node.split('::')[0]
        if classname not in cache:
            text = file_text(worktree, classname)
            cache[classname] = None if text is None else \
                [marker.pattern for marker in RERUN_MARKERS if marker.search(text)]
        markers = cache[classname]
        if markers is None:
            raise RuntimeError(f'unresolvable test file for node: {node}')
        if markers:
            rerun.append(node)
        else:
            reuse.append(node)
    if len(rerun)+len(reuse) != 316:
        raise RuntimeError(f'classification lost nodes: {len(rerun)}+{len(reuse)}')

    Path(args.out_json).write_text(json.dumps({
        'kind': 'occ_grid_acceptance_g6_regression_deploy_selection',
        'total': 316, 'rerun': len(rerun), 'reuse': len(reuse),
        'byte_identity_verified_modules': verified,
        'rerun_nodes': sorted(rerun),
        'reuse_nodes': sorted(reuse),
        'per_file_markers': {k: v for k, v in cache.items() if v},
        'reuse_reason': 'imports no ament/IDL/config/service artifact; product '
                        'modules byte-identical in the frozen install (verified '
                        'against install_cand_sha256.txt)',
    }, indent=2, sort_keys=True)+'\n')
    Path(args.out_args).write_text(''.join(f'{node}\n' for node in sorted(rerun)))
    print(json.dumps({'rerun': len(rerun), 'reuse': len(reuse),
                      'verified_modules': verified}))


if __name__ == '__main__':
    raise SystemExit(main())
