#!/usr/bin/env python3
"""Generate config/deploy_asset_manifest.yaml (ST09-PD).

Measures every deploy asset on disk -- scene, seven-face geometry,
expanded hardware URDF, SRDF, four atlas pairs, four directed route
graphs -- and writes the versioned identity manifest next to the other
luggage_description configs. Hashes are computed from the actual file
bytes; nothing is copied from launch arguments. Re-running the script
after regenerating assets refreshes the manifest; the strict profile
resolver re-measures it at every hardware launch.
"""

from __future__ import division

import argparse
import datetime
import os
import platform
import subprocess
import sys

import yaml

WS = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
for _root in (os.path.join(WS, "src", "luggage_planning"),
              os.path.join(WS, "src", "luggage_description")):
    if _root not in sys.path:
        sys.path.insert(0, _root)

from luggage_description.deploy_manifest import (  # noqa: E402
    DEPLOY_MANIFEST_SCHEMA,
    DEPLOY_PAYLOAD_ENVELOPES,
    build_deploy_manifest,
)
from luggage_planning.deploy.deploy_profile import default_resolver  # noqa: E402

ATLAS_PROFILES = (
    (None, "s20_container_collision_aware", "s20_deploy_empty"),
) + tuple(
    (list(size),
     "s20_container_collision_aware_payload_%.2fx%.2fx%.2f" % tuple(size),
     "s20_deploy_payload_%.2fx%.2fx%.2f" % tuple(size))
    for size in DEPLOY_PAYLOAD_ENVELOPES
)


def git_revision():
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=WS, text=True).strip()
    except Exception:  # noqa: BLE001 - not a git checkout
        return ""


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir", default=os.path.join(
        WS, "src", "luggage_planning", "data", "reachability_atlas_deploy"))
    parser.add_argument("--scene", default=os.path.join(
        WS, "src", "luggage_description", "config", "scene_tf_deploy.yaml"))
    parser.add_argument("--srdf", default=os.path.join(
        WS, "src", "luggage_description", "config", "S20_with_camera.srdf"))
    parser.add_argument("--output", default=os.path.join(
        WS, "src", "luggage_description", "config",
        "deploy_asset_manifest.yaml"))
    parser.add_argument("--command", default=" ".join(sys.argv))
    args = parser.parse_args()

    data_dir = os.path.abspath(args.data_dir)
    scene = os.path.abspath(args.scene)
    srdf = os.path.abspath(args.srdf)
    urdf = os.path.join(data_dir, "robot_description.xml")

    import json
    graph_summary_path = os.path.join(data_dir, "graphs", "summary.json")
    graph_digests = {}
    if os.path.isfile(graph_summary_path):
        with open(graph_summary_path, "r", encoding="utf-8") as handle:
            summary = json.load(handle)
        for entry in summary.get("graphs", []):
            graph_digests[entry["graph_id"]] = entry["digest"]

    entries = []
    for payload, basename, graph_id in ATLAS_PROFILES:
        npz = os.path.join(data_dir, basename + ".npz")
        meta = os.path.join(data_dir, basename + ".yaml")
        graph_path = os.path.join(data_dir, "graphs", graph_id + ".json")
        for path, what in ((npz, "atlas npz"), (meta, "atlas meta"),
                           (graph_path, "route graph")):
            if not os.path.isfile(path):
                raise SystemExit("missing %s: %s" % (what, path))
        entries.append({
            "payload": payload,
            "basename": basename,
            "npz_path": npz,
            "npz_rel": "data/reachability_atlas_deploy/%s.npz" % basename,
            "meta_path": meta,
            "meta_rel": "data/reachability_atlas_deploy/%s.yaml" % basename,
            "graph": {
                "graph_id": graph_id,
                "path": graph_path,
                "rel": "data/reachability_atlas_deploy/graphs/%s.json"
                       % graph_id,
                "digest": graph_digests.get(graph_id, ""),
            },
        })
        if not graph_digests.get(graph_id):
            raise SystemExit(
                "graph %s digest missing from %s; run "
                "build_deploy_route_graphs.py first" % (graph_id,
                                                        graph_summary_path))

    provenance = {
        "generated_at": datetime.datetime.now().astimezone().isoformat(),
        "command": args.command,
        "workspace_revision": git_revision(),
        "host": platform.node(),
        "python": platform.python_version(),
        "ros_distro": os.environ.get("ROS_DISTRO", ""),
    }
    manifest = build_deploy_manifest(
        scene=(scene, "config/scene_tf_deploy.yaml"),
        urdf=(urdf, "data/reachability_atlas_deploy/robot_description.xml"),
        srdf=(srdf, "config/S20_with_camera.srdf"),
        atlas_entries=entries,
        resolve=default_resolver(),
        provenance=provenance,
    )
    with open(args.output, "w", encoding="utf-8") as handle:
        yaml.safe_dump(manifest, handle, default_flow_style=False,
                       sort_keys=True, allow_unicode=False)
    print("wrote %s (schema %s, %d atlas entries)"
          % (args.output, DEPLOY_MANIFEST_SCHEMA, len(manifest["atlas"])))
    return 0


if __name__ == "__main__":
    sys.exit(main())
