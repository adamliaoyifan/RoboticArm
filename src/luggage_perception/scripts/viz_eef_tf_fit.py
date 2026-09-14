#!/usr/bin/env python3
"""Visualize locked EEF TF tree, URDF meshes, and projected D555/Livox clouds.

Reads an existing calib dump (tf_tree.json + fused PLYs). Offline, no ROS
graph, no Gazebo.

Example:
    viz_eef_tf_fit.py \\
      --dump ~/work/robotarm_bags/calib_out/record_site_pendant_20260911_220406_xy_balanced_urdf
"""
import argparse
import json
import os
import sys

from luggage_perception.eval.eef_tf_fit_viz import dump_eef_tf_fit


def _parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--dump", required=True,
                        help="calib export directory with tf_tree.json")
    parser.add_argument("--out", default="",
                        help="output directory (default: <dump>/tf_fit)")
    parser.add_argument("--cloud-cap", type=int, default=6000)
    return parser.parse_args(argv)


def main(argv=None):
    args = _parse_args(argv)
    dump_dir = os.path.abspath(os.path.expanduser(args.dump))
    out = os.path.abspath(os.path.expanduser(args.out)) if args.out else None
    summary = dump_eef_tf_fit(dump_dir, out_dir=out, cloud_cap=args.cloud_cap)
    print(json.dumps({
        "out_dir": summary["out_dir"],
        "html": summary["html"],
        "n_meshes": summary["n_meshes"],
        "n_camera": summary["n_camera"],
        "n_livox": summary["n_livox"],
        "png_ok": summary["png_ok"],
        "frames": summary["frames"],
    }, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
