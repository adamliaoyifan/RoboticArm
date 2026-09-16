#!/usr/bin/env python3
"""Dump bag RGB-D + FK poses for the Open3D TSDF samples.

Offline mcap only. Do not ros2 bag play onto ROS_DOMAIN_ID=7 or the arm.

Example:
    export_open3d_rgbd.py \\
      --bag ~/work/robotarm_bags/2026-09-09/pendant_jog_compressed \\
      --out ~/work/robotarm_bags/recon_out/pendant_jog_compressed \\
      --stride 5 --max-frames 40
"""
import argparse
import json
import os
import sys

from luggage_perception.eval.open3d_rgbd_export import export_open3d_rgbd

DEFAULT_OUT = os.path.join(
    os.path.expanduser("~/work/robotarm_bags"), "recon_out")


def _parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--bag", required=True,
                        help="bag directory or .mcap file")
    parser.add_argument("--out", default="",
                        help="output dataset dir (default "
                             "%s/<bag-name>)" % DEFAULT_OUT)
    parser.add_argument("--world-frame", default="world")
    parser.add_argument("--stride", type=int, default=5)
    parser.add_argument("--max-frames", type=int, default=40)
    parser.add_argument("--join-tolerance-ms", type=float, default=30.0)
    return parser.parse_args(argv)


def main(argv=None):
    args = _parse_args(argv)
    bag = os.path.abspath(os.path.expanduser(args.bag))
    if args.out:
        out_dir = os.path.abspath(os.path.expanduser(args.out))
    else:
        name = os.path.basename(os.path.normpath(bag))
        if name.endswith(".mcap"):
            name = name[:-len(".mcap")]
        out_dir = os.path.join(DEFAULT_OUT, name)
    summary = export_open3d_rgbd(
        bag, out_dir,
        world_frame=args.world_frame,
        stride=args.stride,
        max_frames=args.max_frames,
        join_tolerance_ms=args.join_tolerance_ms,
    )
    print(json.dumps({
        "out_dir": summary["out_dir"],
        "n_emitted": summary["n_emitted"],
        "n_skipped": summary["n_skipped"],
        "n_selected": summary["n_selected"],
        "optical_frame": summary["optical_frame"],
        "world_frame": summary["world_frame"],
    }, indent=2))
    if summary["n_emitted"] == 0:
        print("no posed RGB-D frames; see manifest.json skipped[]",
              file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
