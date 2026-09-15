#!/usr/bin/env python3
"""PF-R7 generation-3 fail-closed attempt classifier. Eval-only.

Not imported by online nodes. A record is classified into exactly one
terminal class. ``known_detector_miss`` requires every conjunct in the
generation-3 plan; anything weaker is some other class. An accepted
proposal followed by mask/cloud/TF/geometry/C2 failure is never a miss.
"""

from __future__ import division

import math

from luggage_perception.eval import gate4_scoring as scoring
from luggage_perception.eval.sim_texture import (
    IMAGE_HEIGHT,
    IMAGE_WIDTH,
    aabb_wholly_inside,
    accepted_cargo,
    best_gt_iou_proposal,
    edge_strip_accepted,
    spawn_is_flip,
)
from luggage_perception.detection_temporal_gate import bbox_iou

CLASS_ELIGIBLE_PASS = "eligible_pass"
CLASS_ELIGIBLE_FAIL = "eligible_fail"
CLASS_KNOWN_MISS = "known_detector_miss"
CLASS_INFRA = "infrastructure_invalid"
CLASS_FIXTURE = "fixture_invalid"
CLASS_EVIDENCE = "evidence_invalid"

SCAN_PROPOSAL_AVAILABLE = "proposal_available"
SCAN_KNOWN_MISS_VALID = "known_detector_miss_valid"
SCAN_FIXTURE = "fixture_invalid"
SCAN_INFRA = "infrastructure_invalid"
SCAN_EVIDENCE = "evidence_invalid"
SCAN_CLASSES = (
    SCAN_PROPOSAL_AVAILABLE,
    SCAN_KNOWN_MISS_VALID,
    SCAN_FIXTURE,
    SCAN_INFRA,
    SCAN_EVIDENCE,
)

TERMINAL_CLASSES = (
    CLASS_ELIGIBLE_PASS,
    CLASS_ELIGIBLE_FAIL,
    CLASS_KNOWN_MISS,
    CLASS_INFRA,
    CLASS_FIXTURE,
    CLASS_EVIDENCE,
)

PRODUCTION_CONFIDENCE_FLOOR = 0.20
PROPOSAL_DEADLINE_SEC = 1.4
MATCH_IOU_MIN = 0.50
OUTPUT_HZ_MIN = 4.0
LAG_Q4_MAX_SEC = 0.20
LAG_Q4_OVER_Q1_MAX = 1.25
RSS_BETA_MAX_MIB_PER_MIN = 2.0

_MISS_FIXTURE_MARKERS = (
    "box_not_stable",
    "gt_bbox_not_in_frame",
    "gt_not_in_workspace",
)
_MISS_INFRA_MARKERS = (
    "clock_publishers_not_one",
    "camera_rate_unhealthy",
    "tf_unhealthy",
    "yolo_rate_unhealthy",
    "inference_rate_unhealthy",
    "controller_unhealthy",
)
_MISS_EVIDENCE_MARKERS = (
    "dump_incomplete",
)
_MISS_FAIL_MARKERS = (
    "production_floor_not_0.20",
    "accepted_proposal_within_deadline",
    "accepted_matching_proposal",
    "matching_proposal_at_or_above_floor",
    "entered_cargo_cloud",
    "geometry_stage_entered",
    "false_cargo_accepted",
)


def scored_class(attempt_class):
    return attempt_class in (CLASS_ELIGIBLE_PASS, CLASS_ELIGIBLE_FAIL)


def advances_slot(attempt_class):
    return attempt_class == CLASS_ELIGIBLE_PASS


def stops_campaign(attempt_class):
    return attempt_class == CLASS_ELIGIBLE_FAIL


def is_exclusion(attempt_class):
    return attempt_class in (
        CLASS_KNOWN_MISS, CLASS_INFRA, CLASS_FIXTURE, CLASS_EVIDENCE)


def scan_availability_class(record, classified=None):
    """Map a live/scripted record to one G5 detector-availability class.

    Geometry eligible_fail after a production-accepted proposal is still
    ``proposal_available``. Incomplete dumps are always ``evidence_invalid``.
    """
    classified = classified or classify_attempt(record)
    attempt_class = classified.get("attempt_class")
    if not dump_is_complete(record) or attempt_class == CLASS_EVIDENCE:
        return SCAN_EVIDENCE, classified
    if attempt_class == CLASS_INFRA:
        return SCAN_INFRA, classified
    if attempt_class == CLASS_FIXTURE:
        return SCAN_FIXTURE, classified
    if matching_accepted_proposal(record):
        return SCAN_PROPOSAL_AVAILABLE, classified
    if attempt_class == CLASS_KNOWN_MISS:
        return SCAN_KNOWN_MISS_VALID, classified
    return SCAN_EVIDENCE, classified


def _as_int(value, default=0):
    try:
        if value is None:
            return default
        return int(value)
    except (TypeError, ValueError):
        return default


def _as_float(value):
    try:
        if value is None:
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


def _truth(record, key):
    return bool(record.get(key))


def _image_wh(record):
    image_wh = record.get("image_wh") or (IMAGE_WIDTH, IMAGE_HEIGHT)
    return int(image_wh[0]), int(image_wh[1])


def _image_shape(record):
    width, height = _image_wh(record)
    return height, width


def dump_is_complete(record):
    health = record.get("dump_health") or {}
    return bool(health.get("capture_complete")) and bool(
        health.get("replay_possible"))


def production_floor_ok(record):
    floor = _as_float(record.get("production_confidence_floor"))
    if floor is None:
        return False
    return abs(floor - PRODUCTION_CONFIDENCE_FLOOR) <= 1e-9


def matching_accepted_proposal(record):
    """True when a production-accepted cargo proposal matches the GT box."""
    if _as_int(record.get("n_accepted_matching")) > 0:
        return True
    if record.get("t_proposal") is not None:
        return True
    gt_bbox = record.get("gt_bbox")
    min_iou = _as_float(record.get("min_iou")) or MATCH_IOU_MIN
    for det in accepted_cargo(record.get("detections")):
        if gt_bbox is None:
            return True
        if float(bbox_iou(det.get("bbox"), gt_bbox)) >= min_iou:
            return True
    return False


def false_cargo_accepted(record):
    if (record.get("edge_fp_accepted")
            or record.get("cabinet_fp_accepted")
            or record.get("platform_fp_accepted")
            or record.get("fake_cargo_cloud")):
        return True
    if edge_strip_accepted(
            record.get("detections"), image_shape=_image_shape(record)):
        return True
    accepted = accepted_cargo(record.get("detections"))
    if not accepted:
        n_acc = record.get("n_accepted_cargo")
        if n_acc is None:
            return False
        accepted_n = _as_int(n_acc)
        return accepted_n > 0 and not matching_accepted_proposal(record)
    if matching_accepted_proposal(record):
        gt_bbox = record.get("gt_bbox")
        min_iou = _as_float(record.get("min_iou")) or MATCH_IOU_MIN
        for det in accepted:
            if gt_bbox is None:
                continue
            if float(bbox_iou(det.get("bbox"), gt_bbox)) < min_iou:
                if not edge_strip_accepted(
                        [det], image_shape=_image_shape(record)):
                    return True
        return False
    return True


def entered_geometry_stage(record):
    if record.get("entered_cargo_cloud"):
        return True
    if _as_int(record.get("n_cargo_points")) > 0:
        return True
    if _as_int(record.get("n_geometry_requests")) > 0:
        return True
    return False


def infrastructure_reasons(record):
    reasons = []
    if record.get("infrastructure_invalid"):
        reasons.append("infrastructure_flag")
    if record.get("controller_ok") is False or record.get(
            "controller_unreachable"):
        reasons.append("controller_unreachable")
    if record.get("spawn_hang"):
        reasons.append("spawn_hang")
    if record.get("node_crashed"):
        reasons.append("node_crashed")
    if record.get("case_reset_timeout"):
        reasons.append("case_reset_timeout")
    if record.get("duplicate_clock") or record.get("dual_clock"):
        reasons.append("duplicate_clock")
    n_clock = record.get("n_clock_publishers")
    if n_clock is not None and _as_int(n_clock) != 1:
        reasons.append("clock_publishers_%s" % n_clock)
    if record.get("camera_rate_collapsed") or record.get(
            "camera_rate_ok") is False:
        reasons.append("camera_rate_collapsed")
    if record.get("inference_rate_collapsed") or record.get(
            "inference_rate_ok") is False:
        reasons.append("inference_rate_collapsed")
    if record.get("tf_ok") is False:
        reasons.append("tf_unhealthy")
    if record.get("yolo_rate_ok") is False:
        reasons.append("yolo_rate_collapsed")
    return reasons


def fixture_reasons(record):
    reasons = []
    message = record.get("spawn_message") or ""
    if record.get("fixture_invalid") or spawn_is_flip(message):
        reasons.append("spawn_flip_or_fixture")
    if record.get("box_stable") is False:
        reasons.append("box_not_stable")
    if record.get("gt_in_workspace") is False:
        reasons.append("gt_not_in_workspace")
    gt_bbox = record.get("gt_bbox")
    width, height = _image_wh(record)
    if record.get("gt_in_frame") is False:
        reasons.append("gt_bbox_not_in_frame")
    elif gt_bbox is not None and not aabb_wholly_inside(
            gt_bbox, width, height):
        reasons.append("gt_bbox_not_in_frame")
    if record.get("spawn_ok") is False:
        reasons.append("spawn_not_ok")
    return reasons


def safety_fail_reasons(record):
    reasons = []
    if false_cargo_accepted(record):
        reasons.append("false_cargo_accepted")
    if _as_int(record.get("false_measured_height")) > 0:
        reasons.append("false_measured_height")
    if _as_int(record.get("stale_cross_epoch")) > 0:
        reasons.append("stale_cross_epoch")
    # Pre-barrier observations and correctly dropped post-barrier frames
    # are diagnostics. Only stale data admitted into the scored set or
    # fused into a consumer is a safety failure.
    if "stale_scored_or_fused" in record:
        if _as_int(record.get("stale_scored_or_fused")) > 0:
            reasons.append("stale_scored_or_fused")
    elif _as_int(record.get("stale_instance_frames")) > 0:
        reasons.append("stale_instance_frames")
    if _as_int(record.get("online_gt_read")) > 0:
        reasons.append("online_gt_read")
    if record.get("cross_epoch_fusion"):
        reasons.append("cross_epoch_fusion")
    return reasons


def c2_fail_reasons(record):
    c2 = record.get("c2")
    if not c2:
        return []
    reasons = []
    hz = _as_float(c2.get("output_hz"))
    if hz is not None and hz < OUTPUT_HZ_MIN:
        reasons.append("output_hz %s < %.1f" % (hz, OUTPUT_HZ_MIN))
    q4 = _as_float(c2.get("executor_lag_q4_sec"))
    q1 = _as_float(c2.get("executor_lag_q1_sec"))
    if q4 is not None and q4 > LAG_Q4_MAX_SEC:
        reasons.append("executor_lag_q4 %s > %.2f" % (q4, LAG_Q4_MAX_SEC))
    if (q4 is not None and q1 is not None and q1 > 0.0
            and q4 > LAG_Q4_OVER_Q1_MAX * q1):
        reasons.append("executor_lag_q4_over_q1 %s/%s > %.2f" % (
            q4, q1, LAG_Q4_OVER_Q1_MAX))
    for name, beta in (c2.get("rss_beta") or {}).items():
        value = _as_float(beta)
        if value is not None and value > RSS_BETA_MAX_MIB_PER_MIN:
            reasons.append("rss_beta_%s %s > %.1f" % (
                name, value, RSS_BETA_MAX_MIB_PER_MIN))
    if c2.get("buffer_unbounded"):
        reasons.append("buffer_unbounded")
    for name, item in (c2.get("buffer_peaks") or {}).items():
        if not isinstance(item, dict):
            continue
        peak = _as_float(item.get("peak"))
        maxlen = _as_float(item.get("maxlen"))
        if peak is None or maxlen is None:
            continue
        if peak > maxlen:
            reasons.append("buffer_%s peak %s > maxlen %s" % (
                name, peak, maxlen))
        pending = item.get("pending_work")
        if pending and maxlen > 0 and peak > 0.5 * maxlen:
            q4_mean = _as_float(item.get("q4_mean"))
            if q4_mean is not None and q4_mean > 0.5 * maxlen:
                reasons.append("buffer_%s q4_mean %s > 0.5*maxlen" % (
                    name, q4_mean))
    if _as_int(c2.get("residual_processes")) > 0:
        reasons.append("residual_processes %s" % c2.get("residual_processes"))
    return reasons


def g4_evidence_reasons(record):
    """Incomplete G4 capture cannot close PF-R7."""
    if not _uses_g4_window(record):
        return []
    reasons = []
    win = record.get("steady_window") or {}
    settled = list(record.get("settled") or [])
    for row in settled:
        stamp = scoring.row_time_sec(row)
        if stamp is None or not math.isfinite(float(stamp)):
            reasons.append("missing_or_nonfinite_stamp")
            break
    if win.get("missing_stamp") or record.get("missing_stamp"):
        if "missing_or_nonfinite_stamp" not in reasons:
            reasons.append("missing_or_nonfinite_stamp")
    t_steady = _as_float(
        win.get("t_steady") if "t_steady" in win else record.get("t_steady"))
    clock_end = _as_float(win.get("clock_end_sec") or record.get("clock_end_sec"))
    window_sec = _as_float(win.get("window_sec")) or scoring.STEADY_WINDOW_SEC
    # Incomplete capture is evidence only after t_steady is known. Missing
    # readiness by 1.4 s is an eligible geometry failure, not an exclusion.
    if t_steady is not None:
        complete = win.get("window_complete")
        if complete is None:
            complete = record.get("window_complete")
        if complete is False:
            reasons.append("window_complete=false")
        if (
            clock_end is None
            or not math.isfinite(float(clock_end))
            or float(clock_end) + scoring.STAMP_EPS_SEC < t_steady + window_sec
        ):
            if "window_complete=false" not in reasons:
                reasons.append("clock_did_not_reach_window_end")
    return reasons


def _uses_g4_window(record):
    return (
        record.get("score_mode") == scoring.STEADY_START_SUPPORT_READY
        or record.get("steady_start") == scoring.STEADY_START_SUPPORT_READY
    )


def _recovery_times(record):
    t_valid = _as_float(record.get("t_first_valid_sec"))
    t_full = _as_float(record.get("t_first_full3d_sec"))
    if t_valid is not None and t_full is not None:
        return t_valid, t_full
    t_prop = _as_float(record.get("t_proposal"))
    if t_prop is None:
        return t_valid, t_full
    settled = list(record.get("settled") or [])
    warmup = list(record.get("warmup") or [])
    owned = warmup + settled
    first_valid = t_valid
    first_full = t_full
    for row in owned:
        stamp = _as_float(row.get("monotonic_sec"))
        if stamp is None:
            stamp = _as_float(row.get("stamp_sec"))
        if stamp is None:
            continue
        dt = max(0.0, stamp - t_prop)
        if first_valid is None and row.get("top_surface_valid"):
            first_valid = dt
        level = row.get("geometry_level")
        if first_full is None and row.get("height_valid") and (
                level == scoring.GEOMETRY_FULL_3D or level == 1):
            first_full = dt
    return first_valid, first_full


def geometry_fail_reasons(record):
    reasons = []
    recovery = record.get("recovery") or {}
    n_settled = recovery.get("n_settled")
    if n_settled is None:
        n_settled = record.get("n_settled")
    if n_settled is None:
        n_settled = len(record.get("settled") or ())
    n_settled = _as_int(n_settled)
    if n_settled < scoring.MIN_SETTLED_PER_TRIAL:
        reasons.append("settled %d < %d" % (
            n_settled, scoring.MIN_SETTLED_PER_TRIAL))
    t_valid, t_full = _recovery_times(record)
    if recovery.get("t_first_valid_sec") is not None:
        t_valid = _as_float(recovery.get("t_first_valid_sec"))
    if recovery.get("t_first_full3d_sec") is not None:
        t_full = _as_float(recovery.get("t_first_full3d_sec"))
    if t_valid is None or t_valid > PROPOSAL_DEADLINE_SEC:
        reasons.append("t_first_valid %s > %.1f" % (
            t_valid, PROPOSAL_DEADLINE_SEC))
    if t_full is None or t_full > PROPOSAL_DEADLINE_SEC:
        reasons.append("t_first_full3d %s > %.1f" % (
            t_full, PROPOSAL_DEADLINE_SEC))
    settled = list(record.get("settled") or [])
    if _uses_g4_window(record):
        win = record.get("steady_window") or {}
        t_steady_dt = _as_float(record.get("t_steady_sec"))
        t_steady = _as_float(win.get("t_steady") or record.get("t_steady"))
        t_prop_stamp = _as_float(record.get("t_proposal_stamp"))
        if t_steady_dt is None and t_steady is not None and t_prop_stamp is not None:
            t_steady_dt = max(0.0, float(t_steady) - float(t_prop_stamp))
        if t_steady is None or t_steady_dt is None or t_steady_dt > PROPOSAL_DEADLINE_SEC:
            reasons.append("t_steady %s > %.1f" % (
                t_steady_dt, PROPOSAL_DEADLINE_SEC))
        window_sec = _as_float(win.get("window_sec")) or scoring.STEADY_WINDOW_SEC
        if abs(float(window_sec) - scoring.STEADY_WINDOW_SEC) > scoring.STAMP_EPS_SEC:
            reasons.append("steady_window_sec %s != %.3f" % (
                window_sec, scoring.STEADY_WINDOW_SEC))
        hz_n = len(settled)
        min_hz_n = int(math.ceil(OUTPUT_HZ_MIN * scoring.STEADY_WINDOW_SEC))
        if hz_n < min_hz_n:
            reasons.append("output_hz %s < %.1f" % (
                (hz_n / scoring.STEADY_WINDOW_SEC),
                OUTPUT_HZ_MIN))
    if not settled:
        if n_settled < scoring.MIN_SETTLED_PER_TRIAL:
            return reasons
        reasons.append("settled_rows_missing")
        return reasons
    summary = scoring.gate_pass(scoring.aggregate([{
        "warmup": list(record.get("warmup") or []),
        "settled": settled,
        "instance_id": record.get("instance_id") or record.get("box_id"),
    }]))
    if not summary.get("gate4_pass"):
        reasons.extend(summary.get("gate4_failures") or ["geometry"])
    extra = scoring.placement_recovery_gate(
        0,
        [{
            "trial": record.get("attempt") or record.get("trial"),
            "n_settled": n_settled,
            "t_first_valid_sec": t_valid,
            "t_first_full3d_sec": t_full,
        }],
        failed_count=summary.get("categories", {}).get("failed") or 0)
    reasons.extend(extra)
    # Deduplicate while preserving order.
    seen = set()
    out = []
    for item in reasons:
        if item in seen:
            continue
        seen.add(item)
        out.append(item)
    return out


def known_detector_miss_proof(record):
    """Conjunctive proof. Any missing item forbids ``known_detector_miss``."""
    missing = []
    proof = {
        "box_stable": False,
        "gt_in_frame": False,
        "gt_in_workspace": False,
        "sensor_tf_yolo_healthy": False,
        "single_clock": False,
        "production_floor_0_20": False,
        "no_accepted_proposal_by_deadline": False,
        "eval_trace_below_floor_or_absent": False,
        "no_cargo_cloud_or_geometry": False,
        "no_false_cargo": False,
        "dump_complete": False,
        "best_confidence": None,
        "best_iou": None,
        "missing": missing,
        "ok": False,
    }
    if _truth(record, "box_stable"):
        proof["box_stable"] = True
    else:
        missing.append("box_not_stable")

    width, height = _image_wh(record)
    gt_bbox = record.get("gt_bbox")
    in_frame = _truth(record, "gt_in_frame") or (
        gt_bbox is not None and aabb_wholly_inside(gt_bbox, width, height))
    if in_frame:
        proof["gt_in_frame"] = True
    else:
        missing.append("gt_bbox_not_in_frame")

    if _truth(record, "gt_in_workspace"):
        proof["gt_in_workspace"] = True
    else:
        missing.append("gt_not_in_workspace")

    n_clock = record.get("n_clock_publishers")
    if n_clock is not None and _as_int(n_clock) == 1 and not record.get(
            "duplicate_clock") and not record.get("dual_clock"):
        proof["single_clock"] = True
    else:
        missing.append("clock_publishers_not_one")

    health_ok = (
        record.get("camera_rate_ok") is True
        and record.get("tf_ok") is True
        and record.get("yolo_rate_ok") is True
        and record.get("inference_rate_ok") is True
        and record.get("controller_ok") is True
        and not record.get("camera_rate_collapsed")
        and not record.get("inference_rate_collapsed")
        and not record.get("node_crashed"))
    if health_ok:
        proof["sensor_tf_yolo_healthy"] = True
    else:
        if record.get("camera_rate_ok") is not True:
            missing.append("camera_rate_unhealthy")
        if record.get("tf_ok") is not True:
            missing.append("tf_unhealthy")
        if record.get("yolo_rate_ok") is not True:
            missing.append("yolo_rate_unhealthy")
        if record.get("inference_rate_ok") is not True:
            missing.append("inference_rate_unhealthy")
        if record.get("controller_ok") is not True:
            missing.append("controller_unhealthy")

    if production_floor_ok(record):
        proof["production_floor_0_20"] = True
    else:
        missing.append("production_floor_not_0.20")

    t_proposal = _as_float(record.get("t_proposal"))
    accepted = matching_accepted_proposal(record)
    if accepted and t_proposal is not None and t_proposal <= PROPOSAL_DEADLINE_SEC:
        missing.append("accepted_proposal_within_deadline")
    elif accepted:
        missing.append("accepted_matching_proposal")
    else:
        proof["no_accepted_proposal_by_deadline"] = True

    match = best_gt_iou_proposal(
        record.get("detections"), gt_bbox,
        min_iou=_as_float(record.get("min_iou")) or MATCH_IOU_MIN)
    if match is None:
        proof["eval_trace_below_floor_or_absent"] = True
        proof["best_confidence"] = None
        proof["best_iou"] = None
    else:
        proof["best_confidence"] = match["confidence"]
        proof["best_iou"] = match["iou"]
        below = (
            match["confidence"] < PRODUCTION_CONFIDENCE_FLOOR
            or not match.get("matches"))
        if below:
            proof["eval_trace_below_floor_or_absent"] = True
        else:
            missing.append("matching_proposal_at_or_above_floor")

    if entered_geometry_stage(record):
        missing.append("entered_cargo_cloud")
        if _as_int(record.get("n_geometry_requests")) > 0:
            missing.append("geometry_stage_entered")
    else:
        proof["no_cargo_cloud_or_geometry"] = True

    if false_cargo_accepted(record):
        missing.append("false_cargo_accepted")
    else:
        proof["no_false_cargo"] = True

    if dump_is_complete(record):
        proof["dump_complete"] = True
    else:
        missing.append("dump_incomplete")

    # Unique missing labels, stable order.
    seen = set()
    unique = []
    for item in missing:
        if item in seen:
            continue
        seen.add(item)
        unique.append(item)
    proof["missing"] = unique
    proof["ok"] = not unique
    return proof


def _result(attempt_class, reasons, extra=None):
    payload = {
        "attempt_class": attempt_class,
        "reasons": list(reasons or []),
        "scored": scored_class(attempt_class),
        "advances_slot": advances_slot(attempt_class),
        "stops_campaign": stops_campaign(attempt_class),
        "exclusion": is_exclusion(attempt_class),
        "counts_as_identity_slot": attempt_class == CLASS_ELIGIBLE_PASS,
    }
    if extra:
        payload.update(extra)
    return payload


def _class_from_miss_blockers(missing, dump_ok):
    if any(item in _MISS_FAIL_MARKERS for item in missing):
        if not dump_ok:
            return CLASS_EVIDENCE, ["dump_incomplete"] + list(missing)
        return CLASS_ELIGIBLE_FAIL, list(missing)
    if any(item in _MISS_INFRA_MARKERS for item in missing):
        return CLASS_INFRA, list(missing)
    if any(item in _MISS_FIXTURE_MARKERS for item in missing):
        return CLASS_FIXTURE, list(missing)
    if any(item in _MISS_EVIDENCE_MARKERS for item in missing) or not dump_ok:
        return CLASS_EVIDENCE, list(missing) or ["dump_incomplete"]
    return CLASS_EVIDENCE, list(missing) or ["unproved_miss"]


def classify_attempt(record):
    """Return one terminal class. Fail-closed: missing proof is not a miss."""
    record = dict(record or {})
    dump_ok = dump_is_complete(record)
    safety = safety_fail_reasons(record)
    infra = infrastructure_reasons(record)
    fixture = fixture_reasons(record)
    extra = {}

    if safety:
        reasons = list(safety)
        if matching_accepted_proposal(record):
            reasons.extend(geometry_fail_reasons(record))
            reasons.extend(c2_fail_reasons(record))
        if not dump_ok:
            return _result(CLASS_EVIDENCE, ["dump_incomplete"] + reasons, extra)
        return _result(CLASS_ELIGIBLE_FAIL, reasons, extra)

    if matching_accepted_proposal(record):
        evidence = g4_evidence_reasons(record)
        geom = geometry_fail_reasons(record)
        c2 = c2_fail_reasons(record)
        downstream = []
        if record.get("mask_join_failed") or record.get("cloud_join_failed"):
            downstream.append("mask_or_cloud_join_failed")
        if record.get("tf_join_failed"):
            downstream.append("tf_join_failed")
        if entered_geometry_stage(record) is False and (
                record.get("expect_geometry") is not False):
            downstream.append("accepted_proposal_without_cargo_cloud")
        reasons = downstream + geom + c2
        extra["t_proposal"] = record.get("t_proposal")
        extra["t_first_valid_sec"], extra["t_first_full3d_sec"] = (
            _recovery_times(record))
        extra["t_steady"] = (record.get("steady_window") or {}).get("t_steady")
        extra["window_complete"] = (record.get("steady_window") or {}).get(
            "window_complete")
        if evidence:
            if not dump_ok:
                return _result(
                    CLASS_EVIDENCE, ["dump_incomplete"] + evidence + reasons,
                    extra)
            return _result(CLASS_EVIDENCE, evidence + reasons, extra)
        if reasons:
            if not dump_ok:
                return _result(
                    CLASS_EVIDENCE, ["dump_incomplete"] + reasons, extra)
            return _result(CLASS_ELIGIBLE_FAIL, reasons, extra)
        if not dump_ok:
            return _result(CLASS_EVIDENCE, ["dump_incomplete"], extra)
        if infra:
            return _result(CLASS_INFRA, infra, extra)
        return _result(CLASS_ELIGIBLE_PASS, [], extra)

    if infra:
        return _result(CLASS_INFRA, infra, extra)
    if fixture:
        return _result(CLASS_FIXTURE, fixture, extra)

    proof = known_detector_miss_proof(record)
    extra["miss_proof"] = proof
    extra["best_confidence"] = proof.get("best_confidence")
    extra["best_iou"] = proof.get("best_iou")
    if proof.get("ok"):
        return _result(CLASS_KNOWN_MISS, [], extra)
    attempt_class, reasons = _class_from_miss_blockers(
        proof.get("missing") or [], dump_ok)
    extra["miss_proof"] = proof
    return _result(attempt_class, reasons, extra)
