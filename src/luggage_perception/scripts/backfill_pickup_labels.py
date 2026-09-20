#!/usr/bin/env python3
"""Convert pixel-space click labels to world-XY pickup labels. Eval-only.

The human judgment happens in pixel space (site_pick_viz ``--label-viz``
mode); this backfill re-derives the world XY through the same channel
the replay used — colour-aligned depth + recorded TF — so labels and
candidates live in one frame. The residual coupling (window-median depth
and recorded TF at the clicked stamp) is the only offline world truth
available and is recorded per label (``depth_mm``, ``tf_mode``) so the
benchmark file says what each label went through.

Example:
    backfill_pickup_labels.py --bag <bag> --clicks clicks.json \
        --out labels.json
"""
from __future__ import division

import argparse
import json
import os
import sys

from luggage_perception.eval.bag_mcap_source import (
    COLOR_INFO_TOPIC,
    DEPTH_COMPRESSED_TOPIC,
    DEPTH_INFO_TOPIC,
    DEPTH_TOPIC,
    decode_depth_message,
    iter_bag_messages,
    scan_bag,
)
from luggage_perception.eval.bag_replay_index import sidecar_path
from luggage_perception.eval.bag_tf import BagTfBuffer
from luggage_perception.eval.pickup_xy_candidates import (
    deproject_pinhole,
    median_depth_window,
)
from luggage_perception.ros_message_adapters import (
    camera_info_frame_from_msg,
)

NS_PER_MS = 1_000_000


def _parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--bag", required=True,
                        help="bag directory or .mcap file")
    parser.add_argument("--clicks", required=True,
                        help="site_pick_viz label-mode output "
                             "({labels: [{frame_id, stamp, "
                             "suction_safe_lid_center_pixel}]})")
    parser.add_argument("--out", required=True, help="labels JSON path")
    parser.add_argument("--tf-interpolate", action="store_true",
                        help="match a candidates run that used "
                             "--tf-interpolate (default off, like the "
                             "replay default)")
    parser.add_argument("--tf-max-gap-ms", type=float, default=50.0)
    return parser.parse_args(argv)


def _load_tf_buffer(bag):
    """BagTfBuffer from the sidecar when present, else the /tf stream."""
    buffer = BagTfBuffer()
    tf_npz = sidecar_path(bag, "tf_edges.npz")
    if os.path.isfile(tf_npz):
        try:
            buffer.load_edges_npz(tf_npz)
            return buffer, "sidecar"
        except (OSError, ValueError, KeyError):
            pass
    for rec in iter_bag_messages(bag, topics=["/tf", "/tf_static"]):
        buffer.add_tf_message(
            rec.message, static=rec.topic == "/tf_static")
    return buffer, "stream"


def main(argv=None):
    args = _parse_args(argv if argv is not None else sys.argv[1:])
    bag = os.path.abspath(os.path.expanduser(args.bag))
    with open(args.clicks, encoding="utf-8") as handle:
        payload = json.load(handle)
    clicks = payload["labels"] if isinstance(payload, dict) else payload

    scan = scan_bag(bag)
    depth_topic = None
    for topic in (DEPTH_TOPIC, DEPTH_COMPRESSED_TOPIC):
        if topic in scan.topics:
            depth_topic = topic
            break
    if depth_topic is None:
        print("error: no depth topic in bag", file=sys.stderr)
        return 2

    frame = None
    for rec in iter_bag_messages(bag, topics=[COLOR_INFO_TOPIC]):
        frame = camera_info_frame_from_msg(rec.message)
        break
    if frame is None:
        for rec in iter_bag_messages(bag, topics=[DEPTH_INFO_TOPIC]):
            frame = camera_info_frame_from_msg(rec.message)
            break
    if frame is None:
        print("error: no camera_info in bag", file=sys.stderr)
        return 2

    tf_buffer, tf_source = _load_tf_buffer(bag)
    interpolate = bool(args.tf_interpolate)
    max_gap_ns = int(args.tf_max_gap_ms * NS_PER_MS)
    optical_frame = str(frame.frame_id or "")
    bag_name = os.path.basename(bag)
    if bag_name.endswith(".mcap"):
        bag_name = bag_name[:-len(".mcap")]

    wanted = {}
    for click in clicks:
        stamp = float(click.get("stamp") or 0.0)
        stamp_ns = int(round(stamp * 1e9))
        wanted.setdefault(stamp_ns, []).append(click)
    if not wanted:
        print("error: no clicks with stamps", file=sys.stderr)
        return 2
    wanted_ns = sorted(wanted)

    def _nearest_wanted(stamp_ns, tol_ns=5_000_000):
        # Click stamps travel through JSON floats: a nanosecond int does
        # not survive float64 (2^53 < 1.8e18), so match the nearest
        # wanted stamp inside 5 ms instead of demanding equality.
        import bisect
        idx = bisect.bisect_left(wanted_ns, stamp_ns)
        best = None
        for cand in (idx - 1, idx):
            if 0 <= cand < len(wanted_ns):
                dt = abs(wanted_ns[cand] - stamp_ns)
                if dt <= tol_ns and (best is None or dt < best[0]):
                    best = (dt, cand)
        return None if best is None else wanted_ns[best[1]]

    labels = []
    matched = set()
    for rec in iter_bag_messages(bag, topics=[depth_topic]):
        stamp_ns = rec.header_stamp_ns
        hit = _nearest_wanted(stamp_ns)
        if hit is None:
            continue
        matched.add(hit)
        depth = decode_depth_message(rec.message)
        if depth is None:
            continue
        for click in wanted[hit]:
            u, v = click["suction_safe_lid_center_pixel"][:2]
            z_m = median_depth_window(depth, u, v)
            record = {
                "frame_id": click.get("frame_id")
                or "%s@%.9f" % (bag_name, stamp_ns / 1e9),
                "bag_path": click.get("bag_path") or bag,
                "stamp": stamp_ns / 1e9,
                "suction_safe_lid_center_pixel": [float(u), float(v)],
                "suction_safe_lid_center_world_xy": None,
                "depth_mm": (round(z_m * 1000.0, 1)
                             if z_m is not None else None),
                "tf_mode": "none",
            }
            if z_m is None:
                labels.append(record)
                continue
            optical = deproject_pinhole(u, v, z_m, frame)
            # Per-label accounting: without the reset, stats accumulate
            # across the whole run and every label after the first
            # interpolated lookup would claim tf_mode "interpolated"
            # even when its own lookup was an exact/nearest hit.
            tf_buffer.reset_stats()
            world = tf_buffer.transform_points(
                optical.reshape(1, 3), "world", optical_frame, stamp_ns,
                interpolate=interpolate, max_gap_ns=max_gap_ns)
            if world is None:
                labels.append(record)
                continue
            record["suction_safe_lid_center_world_xy"] = [
                float(world[0][0]), float(world[0][1])]
            stats = tf_buffer.stats
            record["tf_mode"] = (
                "interpolated" if stats["interpolated_edges"]
                else ("nearest" if stats["lookups"] else "none"))
            labels.append(record)

    missing = sorted(set(wanted) - matched)
    out = {
        "generator": {
            "bag": bag,
            "tf_source": tf_source,
            "tf_interpolate": interpolate,
            "tf_max_gap_ms": float(args.tf_max_gap_ms),
        },
        "labels": labels,
        "unmatched_stamp_count": len(missing),
    }
    out_path = os.path.abspath(os.path.expanduser(args.out))
    parent = os.path.dirname(out_path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as handle:
        json.dump(out, handle, indent=2, sort_keys=True)
        handle.write("\n")
    n_world = sum(1 for row in labels
                  if row["suction_safe_lid_center_world_xy"])
    print("labels: %d clicks, %d with world xy, %d unmatched stamps"
          % (len(labels), n_world, len(missing)))
    return 0 if labels and not missing else (1 if missing else 0)


if __name__ == "__main__":
    sys.exit(main())
