#!/usr/bin/env python3
"""Sized suitcase model tooling.

Subcommands (merged 2026-11-10 from both lineages):

  generate        write the TEXTURED sized models (small/medium/large
                  catalog tiers + textured inch20-26; campaign mesh-only
                  line — suitcase.dae visual + suitcase.stl collision)
  generate-inch   write the STL-SHARED inch models (default tiers
                  inch28/30/32 — the pre-merge master band). WARNING:
                  passing inch20-26 here would OVERWRITE their textured
                  model dirs with STL-shared ones.
  sync-manifest   rewrite sized_suitcases.json from the on-disk meshes
  check           validate every registered tier (extents + manifest)

Usage: generate_sized_suitcases.py <command> [models_root]
"""
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


def _generate_inch(models_root, tiers):
    from luggage_description.suitcase_visual import (
        write_inch_suitcase_models)
    records = write_inch_suitcase_models(models_root, tiers=tiers)
    print("wrote %d inch suitcase models under %s" % (
        len(records), models_root))
    for name in sorted(records):
        rec = records[name]
        m = rec["measure_size"]
        sz = rec["size"]
        print("  %s size=%.3fx%.3fx%.3f measure=%.3fx%.3fx%.3f" % (
            name, sz[0], sz[1], sz[2], m[0], m[1], m[2]))
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
    if command == "generate-inch":
        tiers = tuple(argv[2].split(",")) if len(argv) > 2 else (
            "inch28", "inch30", "inch32")
        return _generate_inch(root, tiers)
    if command == "sync-manifest":
        return _sync_manifest(root)
    if command == "check":
        return _check(root)
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main())
