#!/usr/bin/env python3
"""F24 isolation proof: legacy ROS1/ignored bringup is absent everywhere.

Scans the install tree for installed executables/scripts that reference the
legacy modules, asserts the legacy packages were never installed, and (when
a graph snapshot is supplied) that no legacy node appears in a tested graph.
Exit 0 = isolated; anything else prints the violation.
"""
from __future__ import division

import argparse
import json
import re
from pathlib import Path

#: Legacy sources that must never reach a Humble runtime (F24 row).
LEGACY_PACKAGES = ("luggage_bringup",)
LEGACY_PATH_MARKERS = (
    re.compile(r"luggage_bringup"),
    re.compile(r"ros1_reference"),
    re.compile(r"bin_packer_node"),
)
LEGACY_NODE_NAMES = (
    "orchestrator_node", "interior_explore_loop", "bin_packer",
    "manual_loading_gui",
)


def _scan_install(install_root):
    """Installed executable/script files that reference legacy modules."""
    violations = []
    root = Path(install_root)
    if not root.is_dir():
        return ["install root missing: %s" % root], []
    scanned = 0
    for package_dir in sorted(root.iterdir()):
        if not package_dir.is_dir():
            continue
        # The legacy packages themselves must not be installed at all.
        if package_dir.name in LEGACY_PACKAGES:
            violations.append("legacy package installed: %s" % package_dir)
            continue
        for area in ("lib", "share"):
            base = package_dir / area
            if not base.is_dir():
                continue
            for path in sorted(base.rglob("*")):
                if not path.is_file() or path.is_symlink():
                    continue
                if path.suffix not in ("", ".py", ".sh"):
                    continue
                scanned += 1
                try:
                    text = path.read_bytes().decode("utf-8", "ignore")
                except OSError:
                    continue
                for marker in LEGACY_PATH_MARKERS:
                    if marker.search(text):
                        violations.append(
                            "legacy reference in %s (%s)"
                            % (path.relative_to(root), marker.pattern))
                        break
    return violations, [scanned]


def _scan_graph(graph_file):
    """Legacy node names inside a recorded tested graph (node list text)."""
    violations = []
    names = []
    for line in Path(graph_file).read_text(encoding="utf-8").splitlines():
        name = line.strip()
        if not name:
            continue
        names.append(name)
        for legacy in LEGACY_NODE_NAMES:
            if legacy in name:
                violations.append("legacy node in tested graph: %s" % name)
    return violations, names


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--install", required=True)
    parser.add_argument("--graph", action="append", default=[],
                        help="node-list file of a tested graph (repeatable)")
    parser.add_argument("--output", required=True,
                        help="evidence JSON (exclusive create)")
    args = parser.parse_args()

    violations, stats = _scan_install(args.install)
    graphs = {}
    for graph_file in args.graph:
        graph_violations, names = _scan_graph(graph_file)
        graphs[graph_file] = {"nodes": names, "violations": graph_violations}
        violations.extend(graph_violations)

    evidence = {
        "schema": "st09.f24_isolation/1",
        "install_root": str(Path(args.install).resolve()),
        "files_scanned": stats[0],
        "graphs": graphs,
        "violations": violations,
        "isolated": not violations,
    }
    output = Path(args.output)
    with open(output, "x", encoding="utf-8") as stream:
        json.dump(evidence, stream, indent=1, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"isolated": evidence["isolated"],
                      "violations": len(violations),
                      "files_scanned": stats[0],
                      "output": str(output)}))
    return 0 if evidence["isolated"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
