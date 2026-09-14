#!/usr/bin/env python3
"""Crop the highest D555 table patch, nearby Livox, then ICP.

Offline PLY only. Does not write URDF or play onto ROS_DOMAIN_ID=7.

Example:
    table_patch_icp.py \\
      --camera-ply ~/work/robotarm_bags/calib_out/.../fused/camera_depth_in_elfin_base_link.ply \\
      --livox-ply  ~/work/robotarm_bags/calib_out/.../fused/livox_in_elfin_base_link.ply
"""
import argparse
import json
import os
import sys

from luggage_perception.eval.table_patch_icp import (
    dump_table_patch_icp, dump_xy_full_edges_mean_z, load_frozen_table_icp)


def _parse_args(argv):
    frozen = load_frozen_table_icp()["icp"]
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--camera-ply", required=True)
    parser.add_argument("--livox-ply", required=True)
    parser.add_argument("--out", default="")
    parser.add_argument("--cell", type=float, default=float(frozen["cell_m"]))
    parser.add_argument("--z-band", type=float, default=float(frozen["z_band_m"]))
    parser.add_argument("--xy-margin", type=float, default=float(frozen["xy_margin_m"]))
    parser.add_argument("--z-margin", type=float, default=float(frozen["z_margin_m"]))
    parser.add_argument("--near-radius", type=float, default=float(frozen["near_radius_m"]))
    parser.add_argument("--inlier-radius", type=float, default=float(frozen["inlier_radius_m"]),
                        help="max NN distance to camera table; farther Livox is outlier")
    parser.add_argument("--hull-band", type=float, default=float(frozen["hull_band_m"]))
    parser.add_argument("--interior-w", type=float, default=float(frozen["interior_w"]))
    parser.add_argument("--edge-w", type=float, default=float(frozen["edge_w"]))
    parser.add_argument("--corner-w", type=float, default=float(frozen["corner_w"]),
                        help="ICP weight for the Livox/camera bottom-right corner")
    parser.add_argument("--anchor-radius", type=float, default=float(frozen["anchor_radius_m"]),
                        help="keep CORNER labels only within this radius of the BR corner")
    parser.add_argument("--corners-only", dest="corners_only",
                        action="store_true", default=bool(frozen["corners_only"]),
                        help="ICP using only the BR corner clusters (no side edges)")
    parser.add_argument("--br-sides", dest="corners_only",
                        action="store_false",
                        help="ICP on BR corner plus bottom and right edges (default)")
    parser.add_argument(
        "--mode", default="xy_full_edges_mean_z",
        choices=("xy_full_edges_mean_z", "br_corner_and_sides"),
        help="xy_full_edges_mean_z is planar full-cloud then balanced "
             "corners+edges + mean z; br_corner_and_sides keeps the old "
             "weighted BR ICP")
    return parser.parse_args(argv)


def main(argv=None):
    args = _parse_args(argv)
    cam = os.path.abspath(os.path.expanduser(args.camera_ply))
    lid = os.path.abspath(os.path.expanduser(args.livox_ply))
    if args.out:
        out_dir = os.path.abspath(os.path.expanduser(args.out))
    else:
        out_dir = os.path.join(os.path.dirname(cam), "..", "table_icp")
        out_dir = os.path.abspath(out_dir)
    if args.mode == "xy_full_edges_mean_z":
        report = dump_xy_full_edges_mean_z(
            cam, lid, out_dir,
            cell=args.cell,
            z_band=args.z_band,
            xy_margin=args.xy_margin,
            z_margin=args.z_margin,
            near_radius=args.near_radius,
            inlier_radius=args.inlier_radius,
            hull_band=args.hull_band,
            interior_w=args.interior_w,
            edge_w=args.edge_w,
            corner_w=args.corner_w,
            anchor_radius=args.anchor_radius,
        )
    else:
        report = dump_table_patch_icp(
            cam, lid, out_dir,
            cell=args.cell,
            z_band=args.z_band,
            xy_margin=args.xy_margin,
            z_margin=args.z_margin,
            near_radius=args.near_radius,
            inlier_radius=args.inlier_radius,
            hull_band=args.hull_band,
            interior_w=args.interior_w,
            edge_w=args.edge_w,
            corner_w=args.corner_w,
            anchor_radius=args.anchor_radius,
            corners_only=args.corners_only,
        )
    printable = dict(report)
    print(json.dumps(printable, indent=2))
    if not report.get("ok"):
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
