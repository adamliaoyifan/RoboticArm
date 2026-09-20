#!/usr/bin/env python3
"""Offline site-bag pick visualizer (no ROS graph for perception).

Replays a pendant/real mcap through the current YOLO + platform-free top
estimator, overlays estimated pick / Cartesian waypoints on recorded TCP
and joint angles, and writes a self-contained HTML page.

Perception never joins a ROS domain. Do not ``ros2 bag play`` onto the
live sim (ROS_DOMAIN_ID=7) or the real arm. Optional ``--plan-moveit``
starts a plan-only MoveIt stack on an isolated domain (default 42).

Examples:
    site_pick_replay_viz.py \\
      --bag ~/work/robotarm_bags/2026-09-09/pendant_jog_vaccum3

    site_pick_replay_viz.py --bag <bag> --plan-moveit --ros-domain-id 42
"""
import argparse
import os
import sys

from luggage_perception.eval.isolated_domain import (
    DEFAULT_REPLAY_DOMAIN_ID,
    IsolatedDomainError,
    LIVE_SIM_DOMAIN_ID,
    assert_isolated_domain,
)
from luggage_perception.eval.replay_evaluate import (
    REAL_SITE_CLASS_MAPPING_LABELS,
    REAL_SITE_PROMPTS,
)
from luggage_perception.eval.site_pick_replay import SitePickConfig, replay_site_pick
from luggage_perception.eval.run_provenance import (
    check_install_staleness,
    staleness_warning,
)

DEFAULT_OUT = os.path.join(
    os.path.expanduser("~/work/robotarm_bags"), "pick_replay_out")


def _parse_args(argv):
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0])
    parser.add_argument("--bag", default="",
                        help="bag directory or .mcap file")
    parser.add_argument("--out", default="",
                        help="output directory (default "
                             "%s/<bag-name>)" % DEFAULT_OUT)
    parser.add_argument("--backend", default="yolo_world",
                        choices=["yolo_world", "bbox_fill",
                                 "yolo_world_sam2", "stub"])
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--model", default="")
    parser.add_argument("--config", default="")
    parser.add_argument("--prompts", default=",".join(REAL_SITE_PROMPTS))
    parser.add_argument("--class-mapping-labels",
                        default=",".join(str(v) for v in
                                         REAL_SITE_CLASS_MAPPING_LABELS))
    parser.add_argument("--confidence", type=float, default=0.3)
    parser.add_argument("--stride", type=int, default=10)
    parser.add_argument("--max-frames", type=int, default=24)
    parser.add_argument("--pixel-stride", type=int, default=2)
    parser.add_argument("--scene-tf", default="",
                        help="scene_tf.yaml (workspace XY); default share example")
    parser.add_argument("--support-mode", default="top_only",
                        choices=["top_only", "auto", "configured",
                                 "auto_then_configured"],
                        help="bags have no preprocessor status; default "
                             "top_only (pick Z from the measured top)")
    parser.add_argument("--plan-moveit", action="store_true",
                        help="plan-only MoveIt on --ros-domain-id "
                             "(never executes, never uses domain %s)"
                             % LIVE_SIM_DOMAIN_ID)
    parser.add_argument("--moveit-scene", default="cargo",
                        choices=["cargo", "cargo_ground", "none"],
                        help="collision objects applied to the isolated "
                             "plan-only scene: the detected cargo box "
                             "(default), cargo + ground slab, or the "
                             "empty scene (explicit)")
    parser.add_argument("--ros-domain-id", type=int,
                        default=DEFAULT_REPLAY_DOMAIN_ID,
                        help="isolated domain for optional MoveIt "
                             "(default %(default)s; refusing 7)")
    parser.add_argument("--allow-stub", dest="require_backend",
                        action="store_false", default=True)
    parser.add_argument("--tf-interpolate", dest="tf_interpolate",
                        action="store_true", default=False,
                        help="lerp/slerp TF edges between bracketing "
                             "samples within --tf-max-gap-ms (nearest-"
                             "stamp lookups otherwise; unbracketed "
                             "stamps still fall back to nearest, so "
                             "this only refines numbers)")
    parser.add_argument("--tf-no-interpolate", dest="tf_interpolate",
                        action="store_false")
    parser.add_argument("--tf-max-gap-ms", type=float, default=50.0,
                        help="max bracket span for TF interpolation "
                             "(default %(default)s, one 50 Hz TF period)")
    parser.add_argument("--emit-candidates", default="",
                        help="write the per-frame pickup XY candidates "
                             "JSON for pickup_xy_benchmark to this path")
    parser.add_argument("--label-viz", action="store_true",
                        help="label mode: click the overlay in the HTML "
                             "to record suction_safe_lid_center_pixel "
                             "rows, then backfill to world XY with "
                             "backfill_pickup_labels.py")
    parser.add_argument("--check-install", action="store_true",
                        help="verify the installed library matches the "
                             "source tree and exit (2 when stale); run "
                             "this before trusting any measured numbers")
    return parser.parse_args(argv)


def main(argv=None):
    args = _parse_args(argv if argv is not None else sys.argv[1:])
    staleness = check_install_staleness()
    if args.check_install:
        print("install check: mode=%s stale=%s checked=%d" % (
            staleness["mode"], staleness["stale"],
            staleness["checked"]))
        for name in staleness["newer_files"]:
            print("  newer-or-missing: %s" % name)
        if staleness["stale"]:
            print(staleness_warning(staleness), file=sys.stderr)
            return 2
        return 0
    if not args.bag:
        # --bag is declared optional so --check-install can run alone.
        print("error: the following arguments are required: --bag",
              file=sys.stderr)
        return 2
    warning = staleness_warning(staleness)
    if warning:
        print(warning, file=sys.stderr)
    try:
        domain = assert_isolated_domain(args.ros_domain_id)
    except IsolatedDomainError as exc:
        print("error: %s" % exc, file=sys.stderr)
        return 2
    print("site pick replay: perception=offline-mcap "
          "(no ROS graph, not domain %s)" % LIVE_SIM_DOMAIN_ID)
    if args.plan_moveit:
        print("MoveIt plan-only on ROS_DOMAIN_ID=%s "
              "ROS_LOCALHOST_ONLY=1 (no execution)" % domain)
    else:
        print("MoveIt off; pass --plan-moveit to replan on domain %s"
              % domain)
    bag = os.path.abspath(os.path.expanduser(args.bag))
    bag_name = os.path.basename(os.path.normpath(bag))
    if bag_name.endswith(".mcap"):
        bag_name = bag_name[:-len(".mcap")]
    out_dir = args.out or os.path.join(DEFAULT_OUT, bag_name)
    out_dir = os.path.abspath(os.path.expanduser(out_dir))
    cfg = SitePickConfig(
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
        stride=args.stride,
        max_frames=args.max_frames,
        pixel_stride=args.pixel_stride,
        scene_tf_config=args.scene_tf,
        support_mode=args.support_mode,
        require_backend=args.require_backend,
        plan_moveit=bool(args.plan_moveit),
        moveit_scene=args.moveit_scene,
        ros_domain_id=domain,
        tf_interpolate=bool(args.tf_interpolate),
        tf_max_gap_ms=args.tf_max_gap_ms,
        emit_candidates=args.emit_candidates,
        label_viz=bool(args.label_viz),
    )
    summary = replay_site_pick(bag, out_dir, cfg)
    print("wrote", summary.get("html"))
    print("frames", summary.get("frames_processed"),
          "picks", summary.get("frames_with_pick"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
