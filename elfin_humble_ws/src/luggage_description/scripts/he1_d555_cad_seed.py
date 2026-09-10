#!/usr/bin/env python3
"""HE-1 generation 2: derive eef_mount_adapter -> D555-mechanical seed."""

from __future__ import division

import argparse
import os
import sys


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--cad-step", required=True)
    parser.add_argument("--mount-stl", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--cad-zip")
    parser.add_argument("--datasheet")
    parser.add_argument("--tessellation-glb", action="append", default=[])
    args = parser.parse_args(argv)
    from luggage_description.he1.pipeline import run_he1
    glbs = [os.path.abspath(p) for p in args.tessellation_glb]
    report, out_dir = run_he1(
        os.path.abspath(args.cad_step),
        os.path.abspath(args.mount_stl),
        os.path.abspath(args.out_dir),
        cad_zip=args.cad_zip,
        datasheet=args.datasheet,
        tessellation_glbs=glbs,
    )
    print("outcome=%s out=%s unique=%s" % (
        report["outcome"], out_dir, report["registration"].get("unique")))
    return 0 if report["outcome"] in ("pass", "blocked") else 1


if __name__ == "__main__":
    sys.exit(main())
