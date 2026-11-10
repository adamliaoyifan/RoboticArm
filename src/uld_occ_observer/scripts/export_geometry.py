#!/usr/bin/env python3
"""Export normalized authority data once; runtime has no Python geometry dependency.

Use the frozen b33 luggage_description on PYTHONPATH, never duplicate its hash.
"""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene-config', required=True)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    import yaml
    from luggage_description.container_geometry import descriptor_from_scene_config
    geometry = descriptor_from_scene_config(yaml.safe_load(Path(args.scene_config).read_text()))
    descriptor = geometry.descriptor()
    canonical = json.dumps(descriptor, sort_keys=True, separators=(',', ':'), allow_nan=False)
    artifact = dict(schema='uld_geometry_export_v1', descriptor_json=canonical,
                    descriptor_sha256=hashlib.sha256(canonical.encode()).hexdigest())
    Path(args.out).write_text(yaml.safe_dump(artifact, sort_keys=True))


if __name__ == '__main__':
    main()
