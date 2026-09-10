#!/usr/bin/env python3
"""Offline pendant-bag replay YOLO evaluation (open-loop, no ROS graph).

Reads the teach-pendant mcap bags directly (Humble's rosbag2 cannot open
them — see luggage_perception/eval/bag_mcap_source.py), joins each colour
frame with its aligned depth by exact header stamp, runs the ROS-free
YOLO segmenter per frame and writes a timestamp-keyed evidence tree:
per-frame colour/mask/overlay/detections/joint_states, cargo point
clouds in the optical frame, one camera_info per bag, a join report and
an INDEX.md. See luggage_perception/eval/replay_evaluate.py.

Examples:
    # join report only, no YOLO:
    pendant_bag_replay_eval.py --bag ~/work/robotarm_bags/<bag> --dry-run

    # 5-frame GPU smoke run:
    pendant_bag_replay_eval.py --bag <bag> --max-frames 5

    # full bags with overlay quick-look video:
    pendant_bag_replay_eval.py --bag <bag1> --bag <bag2> --make-video
"""
import argparse
import os
import sys

from luggage_perception.eval.replay_evaluate import (
    REAL_SITE_CLASS_MAPPING_LABELS,
    REAL_SITE_PROMPTS,
    ReplayEvalConfig,
    evaluate_bag,
)

DEFAULT_OUT = os.path.join(
    "docs", "status", "evidence", "pendant_replay")


def _parse_args(argv):
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0])
    parser.add_argument("--bag", action="append", required=True,
                        help="bag directory or .mcap file (repeatable)")
    parser.add_argument("--out", default=DEFAULT_OUT,
                        help="output root (default %(default)s)")
    parser.add_argument("--backend", default="yolo_world",
                        choices=["yolo_world", "bbox_fill",
                                 "yolo_world_sam2", "stub"],
                        help="semantic backend (default %(default)s)")
    parser.add_argument("--device", default="cuda",
                        help="torch device (default %(default)s)")
    parser.add_argument("--model", default="",
                        help="YOLO weights path; bare names resolve "
                             "against the package root/share models dir")
    parser.add_argument("--config", default="",
                        help="semantic_segmenter.yaml override (sim prompt "
                             "set); by default the tuned real-site prompt "
                             "set below is used")
    parser.add_argument("--prompts", default=",".join(REAL_SITE_PROMPTS),
                        help="comma-separated prompts (default: the "
                             "2026-09-10 ablation winner for the pendant "
                             "site: platform-descriptive + suitcase + "
                             "luggage, all cargo)")
    parser.add_argument("--class-mapping-labels",
                        default=",".join(str(v)
                                         for v in REAL_SITE_CLASS_MAPPING_LABELS),
                        help="comma-separated label ids parallel to "
                             "--prompts (default all cargo)")
    parser.add_argument("--confidence", type=float, default=0.3,
                        help="confidence threshold (default %(default)s; "
                             "raw confidences are kept in "
                             "detections.jsonl for post-hoc re-filtering)")
    parser.add_argument("--stride", type=int, default=1,
                        help="process every Nth planned frame")
    parser.add_argument("--max-frames", type=int, default=0,
                        help="cap planned frames (0 = all)")
    parser.add_argument("--join-tolerance-ms", type=float, default=30.0,
                        help="colour/depth rescue tolerance after the "
                             "exact-stamp pass; measured orphan-to-depth"
                             " gaps are p50 11-25 ms (one 33 ms camera "
                             "period), so 30 ms rescues real drops without"
                             " crossing a full frame")
    parser.add_argument("--aux-tolerance-ms", type=float, default=50.0,
                        help="joint_states/tcp_pose nearest-stamp window")
    parser.add_argument("--lidar-tolerance-ms", type=float, default=100.0,
                        help="livox scan nearest-stamp window")
    parser.add_argument("--cargo-select", default="center_conf",
                        choices=["center_conf", "none"],
                        help="keep THE cargo detection per frame "
                             "(center-prior + max conf, one-luggage site "
                             "rule); 'none' keeps every detection")
    parser.add_argument("--center-radius-frac", type=float, default=0.35,
                        help="central-preferring radius as a fraction of "
                             "image width (default %(default)s; the true "
                             "box sits inside this radius in 97-100%% of "
                             "measured pendant frames)")
    parser.add_argument("--with-points", dest="with_points",
                        action="store_true", default=True,
                        help="cargo .ply per frame (default on)")
    parser.add_argument("--no-points", dest="with_points",
                        action="store_false")
    parser.add_argument("--pixel-stride", type=int, default=2,
                        help="deproject pixel decimation")
    parser.add_argument("--with-lidar", action="store_true",
                        help="additionally copy the nearest livox scan "
                             "into each frame dir (the full archive is "
                             "always written unless --no-lidar-archive)")
    parser.add_argument("--no-lidar-archive", dest="archive_lidar",
                        action="store_false", default=True,
                        help="skip the full /livox/lidar archive "
                             "(lidar/<stamp>/points.npy [x,y,z,intensity]"
                             " + lidar.ply + lidar_index.jsonl)")
    parser.add_argument("--save-depth-npy", dest="save_depth_npy",
                        action="store_true", default=True)
    parser.add_argument("--no-depth-npy", dest="save_depth_npy",
                        action="store_false",
                        help="skip per-frame depth.npy (~0.9 MB each)")
    parser.add_argument("--no-depth-vis", dest="depth_vis",
                        action="store_false", default=True)
    parser.add_argument("--make-video", action="store_true",
                        help="replay.mp4 quick-look from the overlays "
                             "(requires ffmpeg)")
    parser.add_argument("--allow-stub", dest="require_backend",
                        action="store_false", default=True,
                        help="do not fail when the backend falls back to "
                             "stub (testing only)")
    parser.add_argument("--dry-run", action="store_true",
                        help="join report only, no YOLO")
    return parser.parse_args(argv)


def main(argv=None):
    args = _parse_args(argv if argv is not None else sys.argv[1:])
    cfg = ReplayEvalConfig(
        backend=args.backend,
        device=args.device,
        model_path=args.model,
        config_yaml=args.config,
        prompts=([p.strip() for p in args.prompts.split(",")
                  if p.strip()] or None),
        class_mapping_labels=([int(v) for v in
                               args.class_mapping_labels.split(",")
                               if v.strip()] or None),
        confidence=args.confidence,
        join_tolerance_ms=args.join_tolerance_ms,
        aux_tolerance_ms=args.aux_tolerance_ms,
        lidar_tolerance_ms=args.lidar_tolerance_ms,
        stride=args.stride,
        max_frames=args.max_frames,
        with_points=args.with_points,
        pixel_stride=args.pixel_stride,
        cargo_select=args.cargo_select,
        center_radius_frac=args.center_radius_frac,
        archive_lidar=args.archive_lidar,
        with_lidar=args.with_lidar,
        save_depth_npy=args.save_depth_npy,
        depth_vis=args.depth_vis,
        make_video=args.make_video,
        require_backend=args.require_backend,
        dry_run=args.dry_run,
    )
    failures = []
    for bag in args.bag:
        print("== replay %s" % bag, flush=True)
        try:
            summary = evaluate_bag(bag, args.out, cfg)
        except Exception as exc:  # noqa: BLE001 report per bag, continue
            failures.append((bag, exc))
            print("   FAILED: %r" % (exc,), flush=True)
            continue
        print("   %s frames" % summary.get("frames_processed",
                                           summary.get("frames_planned", 0)),
              "join:", summary.get("join"), flush=True)
    if failures:
        for bag, exc in failures:
            print("FAILED %s: %r" % (bag, exc), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
