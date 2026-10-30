#!/usr/bin/env python3
"""Emit the pytest argfile for gen7 regression deploy reruns.

Takes regression_316_disposition.json, keeps every
rerun_from_frozen_install node, and rewrites the junit dotted classname
(module or module.TestClass) into a real file path under the worktree.
"""
import argparse
import json
from pathlib import Path


def to_path(worktree, classname):
    parts = classname.split('.')
    while parts:
        for base in (worktree, worktree/'src'):
            candidate = base.joinpath(*parts)
            if candidate.suffix != '.py':
                candidate = candidate.with_suffix('.py')
            if candidate.exists():
                return candidate
        parts = parts[:-1]
    raise RuntimeError('unresolvable test module for '+classname)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--disposition', required=True)
    parser.add_argument('--worktree', required=True)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()

    data = json.loads(Path(args.disposition).read_text())
    worktree = Path(args.worktree)
    lines = []
    for node, verdict in data['disposition'].items():
        if verdict['action'] != 'rerun_from_frozen_install':
            continue
        classname, name = node.split('::', 1)
        parts = classname.split('.')
        # Junit classnames are either module paths or module.TestClass; keep
        # the pytest class scope so every node id matches exactly one test.
        if parts[-1][:1].isupper():
            module, test_class = '.'.join(parts[:-1]), parts[-1]
            lines.append(f"{to_path(worktree, module)}::{test_class}::{name}")
        else:
            lines.append(f"{to_path(worktree, classname)}::{name}")
    if not lines:
        raise RuntimeError('no rerun nodes selected')
    Path(args.out).write_text('\n'.join(sorted(lines))+'\n')
    print(len(lines))


if __name__ == '__main__':
    raise SystemExit(main())
