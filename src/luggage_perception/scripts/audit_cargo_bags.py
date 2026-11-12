#!/usr/bin/env python3
"""User-data assessment only; no ROS playback, publishers or planning changes."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import traceback

from luggage_perception.eval.cargo_bag_audit import audit_mcap, inventory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    manifest = dict(start=datetime.now(timezone.utc).isoformat(), command=sys.argv,
                    revision=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
                    dirty=subprocess.check_output(['git', 'status', '--porcelain'], text=True).splitlines(),
                    capture_complete=False, replay_possible=False, no_ros_graph=True)
    source_paths = [Path(__file__), Path(__file__).parents[1] / 'luggage_perception/eval/cargo_bag_audit.py',
                    Path(__file__).parents[1] / 'luggage_perception/eval/cargo_acquisition_audit.py']
    source_dir = args.output / 'source_snapshot'
    source_dir.mkdir()
    manifest['source_sha256'] = {}
    for p in source_paths:
        data = p.read_bytes()
        (source_dir / p.name).write_bytes(data)
        manifest['source_sha256'][str(p)] = hashlib.sha256(data).hexdigest()
    try:
        items = inventory(args.root)
        (args.output / 'inventory.json').write_text(json.dumps(items, indent=2))
        target = args.root / '20260916_212503_floor_box_pick/bags/pick/pick_0.mcap'
        expected = next(x for x in items if target.parent == Path(x['path']).parent)
        result = audit_mcap(target, args.output / 'floor_box',
                            {t['name']: t['count'] for t in expected['topics']})
        if len(items) != 10:
            raise ValueError('inventory changed from frozen10 bags; review workload')
        total_bytes = sum(p.stat().st_size for p in args.output.rglob('*') if p.is_file())
        if total_bytes > 128 * 1024**2:
            raise ValueError('evidence size exceeds128MiB')
        manifest.update(capture_complete=True, replay_possible=True,
                        inventory_count=len(items), evidence_bytes=total_bytes,
                        exact_join_count=result['exact_four_source_join_count'])
    except Exception:
        manifest['error'] = traceback.format_exc()
        raise
    finally:
        manifest['end'] = datetime.now(timezone.utc).isoformat()
        (args.output / 'manifest.json').write_text(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
