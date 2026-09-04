#!/usr/bin/env python3
"""Standalone: reconstruct container_visual.stl from preprocess output PLY/JSON."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from cargo_mesh_qa import log_stl_aabb  # noqa: E402
from cargo_surface_mesh import build_surface_visual_mesh  # noqa: E402


def load_config(path: Path) -> dict:
    with path.open(encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-c",
        "--config",
        type=Path,
        default=SCRIPT_DIR.parent / "config" / "surface_visual.yaml",
    )
    args = parser.parse_args()

    if not args.config.exists():
        example = args.config.parent / "surface_visual.yaml.example"
        print(f"Config missing: {args.config}\nCopy: cp {example} {args.config}", file=sys.stderr)
        return 1

    cfg = load_config(args.config)
    output_dir = Path(cfg.get("output_dir", "./output")).expanduser()
    if not output_dir.is_absolute():
        output_dir = (SCRIPT_DIR.parent / output_dir).resolve()

    try:
        result = build_surface_visual_mesh(output_dir, cfg)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    print(f"Visual STL: {result.visual_path} ({result.visual_triangles} triangles)")
    if result.debug_path:
        print(f"Debug mesh: {result.debug_path}")
    print(f"Coverage: {result.coverage['ratio']:.1%} (median {result.coverage['median_m']:.4f} m)")
    for line in result.qa.messages:
        print(f"  {line}")
    log_stl_aabb(result.visual_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
