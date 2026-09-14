#!/usr/bin/env python3
"""Shared PF-R7-H1 eval records. Not a test module."""

from __future__ import division

GT_BBOX = [170, 105, 419, 297]


def health_ok():
    return {"capture_complete": True, "replay_possible": True, "missing": []}


def det(bbox, conf, accepted=False, reason="cargo"):
    return {
        "label": 2,
        "prompt": "suitcase",
        "confidence": conf,
        "bbox": list(bbox),
        "accepted": bool(accepted),
        "accept_reason": reason,
    }


def settled_ok(n=40):
    return [{
        "top_surface_valid": True,
        "height_valid": True,
        "geometry_level": 1,
        "height_source": 1,
        "err_top_m": 0.004,
        "err_support_m": 0.004,
        "err_height_m": 0.006,
        "err_xy_m": 0.008,
        "err_width_m": 0.010,
        "err_depth_m": 0.009,
        "false_measured_height": False,
        "stamp_sec": 10.0 + 0.05 * i,
        "monotonic_sec": 10.0 + 0.05 * i,
        "n_cargo_points": 8000,
    } for i in range(n)]


def healthy_base():
    return {
        "box_stable": True,
        "gt_in_frame": True,
        "gt_in_workspace": True,
        "gt_bbox": list(GT_BBOX),
        "image_wh": (640, 480),
        "n_clock_publishers": 1,
        "camera_rate_ok": True,
        "tf_ok": True,
        "yolo_rate_ok": True,
        "inference_rate_ok": True,
        "controller_ok": True,
        "production_confidence_floor": 0.20,
        "dump_health": health_ok(),
        "edge_fp_accepted": False,
        "cabinet_fp_accepted": False,
        "platform_fp_accepted": False,
        "fake_cargo_cloud": False,
        "online_gt_read": 0,
        "stale_cross_epoch": 0,
        "stale_instance_frames": 0,
        "stale_pre_barrier_observed": 0,
        "stale_post_barrier_dropped": 0,
        "stale_scored_or_fused": 0,
    }


def pass_record(**over):
    record = healthy_base()
    record.update({
        "t_proposal": 0.0,
        "n_accepted_matching": 1,
        "n_accepted_cargo": 1,
        "n_cargo_points": 8000,
        "n_geometry_requests": 1,
        "entered_cargo_cloud": True,
        "detections": [det(GT_BBOX, 0.85, accepted=True)],
        "t_first_valid_sec": 0.4,
        "t_first_full3d_sec": 0.5,
        "n_settled": 40,
        "recovery": {
            "spawn_ok": True,
            "n_settled": 40,
            "t_first_valid_sec": 0.4,
            "t_first_full3d_sec": 0.5,
        },
        "settled": settled_ok(),
        "c2": {
            "output_hz": 12.0,
            "executor_lag_q4_sec": 0.09,
            "executor_lag_q1_sec": 0.08,
            "rss_beta": {"luggage_detector": 1.2},
            "buffer_unbounded": False,
            "residual_processes": 0,
        },
    })
    record.update(over)
    return record


def miss_record(**over):
    record = healthy_base()
    record.update({
        "t_proposal": None,
        "n_accepted_matching": 0,
        "n_accepted_cargo": 0,
        "n_cargo_points": 0,
        "n_geometry_requests": 0,
        "entered_cargo_cloud": False,
        "detections": [det(GT_BBOX, 0.07, accepted=False,
                           reason="eval_low_conf")],
        "recovery": {
            "spawn_ok": True,
            "n_settled": 0,
            "t_first_valid_sec": None,
            "t_first_full3d_sec": None,
        },
        "settled": [],
    })
    record.update(over)
    return record
