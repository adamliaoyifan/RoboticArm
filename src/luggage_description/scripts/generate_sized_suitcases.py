#!/usr/bin/env python3
"""Sized-suitcase asset pipeline (G3 wave 3): one entry for every asset.

Subcommands:

  generate [models_root]     regenerate the pre-scaled model dirs and
                             manifest for every REGISTERED tier
                             (``iter_size_tiers``); deterministic.
  sync-manifest [root]       rebuild ``sized_suitcases.json`` from the
                             on-disk meshes (adds records for tiers whose
                             meshes already exist; rewrites nothing else).
  check [models_root]        read-only validation: every registered tier
                             has STL/DAE extents == its registered size,
                             and the manifest equals the disk state.

Reproducibility: fixed iteration order, no timestamps; running
``sync-manifest`` twice yields byte-identical manifests. Register a new
tier in ``suitcase_visual`` (or the catalog), then run the pipeline;
never hand-commit model dirs (the inch models bypassed this pipeline
once and cost a week of geometry forensics).
"""

from __future__ import division

import json
import os
import sys


def _default_root():
    here = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    return os.path.normpath(os.path.join(
        here, "..", "luggage_gazebo", "models"))


def _generate(models_root):
    from luggage_description.suitcase_visual import (
        write_sized_suitcase_models)
    records = write_sized_suitcase_models(models_root)
    print("wrote %d sized suitcase models under %s" % (
        len(records), models_root))
    return 0


def _sync_manifest(models_root):
    from luggage_description.suitcase_visual import (
        sized_asset_records, sized_suitcases_manifest_path)
    records = sized_asset_records(models_root)
    path = sized_suitcases_manifest_path(models_root)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(records, handle, indent=1, sort_keys=True)
        handle.write("\n")
    print("synced %d tier records -> %s" % (len(records), path))
    for name in sorted(records):
        rec = records[name]
        print("  %s size=%.3fx%.3fx%.3f primitive=%.3fx%.3fx%.3f" % (
            name, rec["size"][0], rec["size"][1], rec["size"][2],
            rec["primitive_collision_size"][0],
            rec["primitive_collision_size"][1],
            rec["primitive_collision_size"][2]))
    return 0


def _check(models_root):
    from luggage_description.suitcase_visual import check_sized_assets
    failures = check_sized_assets(models_root)
    if failures:
        for line in failures:
            print("FAIL %s" % line)
        print("%d failure(s)" % len(failures))
        return 1
    print("sized assets consistent (all registered tiers, manifest in sync)")
    return 0


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    command = argv[0] if argv else "check"
    root = argv[1] if len(argv) > 1 else _default_root()
    if command == "generate":
        return _generate(root)
    if command == "sync-manifest":
        return _sync_manifest(root)
    if command == "check":
        return _check(root)
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main())
