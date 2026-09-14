#!/usr/bin/env python3
"""Dump a site pendant bag for Mid-360 vs D555 mount-TF ICP.

Offline mcap only. Do not ros2 bag play onto ROS_DOMAIN_ID=7.

Example:
    export_lidar_camera_calib.py \\
      --bag ~/work/robotarm_bags/record_site_pendant_20260911_220406
"""
import argparse
import json
import os
import sys

from luggage_perception.eval.lidar_camera_calib_export import (
    MOUNTER_FRAME,
    export_lidar_camera_calib,
)

DEFAULT_OUT = os.path.join(
    os.path.expanduser("~/work/robotarm_bags"), "calib_out")


def _parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--bag", required=True)
    parser.add_argument("--out", default="")
    parser.add_argument("--mounter-frame", default=MOUNTER_FRAME)
    parser.add_argument("--camera-stride", type=int, default=1)
    parser.add_argument("--bag-tf-only", action="store_true",
                        help="do not override sensor mounts with current xacro")
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
        suffix = "bag_tf" if args.bag_tf_only else "xacro"
        out_dir = os.path.join(DEFAULT_OUT, name + "_" + suffix)
    summary = export_lidar_camera_calib(
        bag, out_dir,
        mounter_frame=args.mounter_frame,
        camera_stride=args.camera_stride,
        apply_xacro=not args.bag_tf_only,
    )
    print(json.dumps({
        "out_dir": summary["out_dir"],
        "n_camera": summary["n_camera"],
        "n_lidar": summary["n_lidar"],
        "apply_xacro": summary.get("apply_xacro"),
        "xacro": summary.get("xacro"),
        "corner_check": summary.get("corner_check"),
        "overlay": os.path.join(summary["out_dir"], summary["preview"]),
        "extra_frames": summary.get("extra_frames"),
    }, indent=2))
    if summary["n_lidar"] == 0 or summary["n_camera"] == 0:
        print("incomplete parse (need both lidar and camera)", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
