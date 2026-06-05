#!/usr/bin/env python3
"""Standalone geometric completion for body/leg point clouds before surface meshing."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from cargo_cloud_completion import complete_cargo_cloud  # noqa: E402


def load_config(path: Path) -> dict:
    with path.open(encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-c",
        "--config",
        type=Path,
        default=SCRIPT_DIR.parent / "config" / "cloud_completion.yaml",
    )
    args = parser.parse_args()

    if not args.config.exists():
        example = args.config.parent / "cloud_completion.yaml.example"
        print(f"Config missing: {args.config}\nCopy: cp {example} {args.config}", file=sys.stderr)
        return 1

    cfg = load_config(args.config)
    output_dir = Path(cfg.get("output_dir", "./output")).expanduser()
    if not output_dir.is_absolute():
        output_dir = (SCRIPT_DIR.parent / output_dir).resolve()

    try:
        result = complete_cargo_cloud(output_dir, cfg)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    for line in result.messages:
        print(line)
    print(f"Body completed: {result.body_path}")
    print(f"Leg completed: {result.leg_path}")
    if result.debug_path:
        print(f"Debug: {result.debug_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
