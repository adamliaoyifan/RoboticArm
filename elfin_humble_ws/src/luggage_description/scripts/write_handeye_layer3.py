#!/usr/bin/env python3
"""Write Layer 3 camera_mount_origin.xacro from a hand-eye JSON (AX=XB)."""

from __future__ import division

import argparse
import os
import shutil
import sys

_PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

from luggage_description.handeye_layer3 import (  # noqa: E402
    camera_mount_xacro_text,
    freeze_from_handeye_json,
)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input",
        default=os.path.join(
            _PKG, "config", "handeye_cc600_20260911_21-38.json"),
    )
    parser.add_argument(
        "--out",
        default=os.path.join(_PKG, "config", "camera_mount_origin.xacro"),
    )
    parser.add_argument("--backup-dir")
    args = parser.parse_args(argv)
    frozen = freeze_from_handeye_json(args.input)
    note = (
        "Layer 3 from CC600 ChArUco eye-in-hand 2026-09-11 poses 21-38 "
        "(TSAI/PARK/DANIILIDIS mean). Do not edit Layers 1-2."
    )
    text = camera_mount_xacro_text(frozen["xyz"], frozen["rpy"], note)
    if args.backup_dir:
        os.makedirs(args.backup_dir, exist_ok=True)
        if os.path.isfile(args.out):
            shutil.copy2(
                args.out,
                os.path.join(args.backup_dir, os.path.basename(args.out)),
            )
    with open(args.out, "w", encoding="utf-8") as handle:
        handle.write(text)
    print("xyz", ["%.6f" % v for v in frozen["xyz"]])
    print("rpy", ["%.8f" % v for v in frozen["rpy"]])
    print("wrote", args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
