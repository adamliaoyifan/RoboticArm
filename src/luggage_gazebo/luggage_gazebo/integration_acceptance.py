"""Finite two-box integration acceptance over joined, measured boundary events.

This extractor never advances a campaign or labels synthetic fixtures live
coverage. GT placement checks are separate from measured map admission.
"""
import math

EVENT_ORDER = (
    "observation", "slot", "selected_path", "probe", "execution", "attach",
    "release", "measured_verification", "measured_integration", "map_update", "return_complete",
)
FROZEN_HASHES = ("runner", "extractor", "resolved_profile", "scene", "model", "atlas", "urdf", "srdf", "messages", "hull")
GT_LIMITS = {"xy_error_m": 0.04, "z_error_m": 0.03, "drift_m": 0.02,
             "roll_deg": 2.0, "pitch_deg": 2.0}


def extract_acceptance(evidence, frozen):
    """Missing or malformed data is not_evaluated, never a vacuous pass."""
    missing, failures = [], []
    expected_hashes = frozen.get("hashes")
    if not isinstance(expected_hashes, dict):
        missing.append("frozen.hashes")
        expected_hashes = {}
    for key in FROZEN_HASHES:
        digest = expected_hashes.get(key)
        if not isinstance(digest, str) or len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest):
            missing.append("frozen.hashes." + key)
    for key in ("release_sample_interval_s", "drift_sample_interval_s"):
        value = frozen.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            missing.append("frozen." + key)
    if frozen.get("event_order") != list(EVENT_ORDER):
        missing.append("frozen.event_order")
    replay = frozen.get("replay_command")
    if not isinstance(replay, list) or not replay or any(not isinstance(arg, str) or not arg for arg in replay):
        missing.append("frozen.replay_command")
    if not expected_hashes:
        missing.append("frozen.hashes")
    elif evidence.get("hashes") != expected_hashes:
        failures.append("FROZEN_INPUT_CHANGED")
    if evidence.get("complete") is not True:
        missing.append("complete evidence trailer")
    cases = evidence.get("cases", [])
    if not isinstance(cases, list) or len(cases) != 2:
        missing.append("two cases")
        cases = []
    previous_revision = None
    seen_requests = set()
    for index, case in enumerate(cases):
        prefix = f"case[{index}]"
        if case.get("box_id") != ("carryon", "standard")[index]:
            failures.append(prefix + ":BOX_ORDER")
        request_id = case.get("request_id")
        if not isinstance(request_id, str) or not request_id:
            missing.append(prefix + ".request_id")
        elif request_id in seen_requests:
            failures.append(prefix + ":DUPLICATE_REQUEST")
        if isinstance(request_id, str):
            seen_requests.add(request_id)
        events = case.get("events", [])
        if not isinstance(events, list):
            missing.append(prefix + ".events")
            events = []
        stages = [e.get("stage") for e in events]
        if stages != list(EVENT_ORDER):
            missing.append(prefix + ".ordered boundary events")
        last_stamp = -1
        for event in events:
            if event.get("request_id") != request_id:
                failures.append(prefix + ":UNJOINED_EVENT")
            for field, frozen_key in (("scene_revision", "scene"), ("geometry_hash", "hull")):
                if event.get(field) != expected_hashes.get(frozen_key) or field not in event:
                    failures.append(prefix + ":BOUNDARY_IDENTITY:" + field)
            stamp = event.get("stamp_ns")
            if type(stamp) is not int or stamp < last_stamp:
                failures.append(prefix + ":EVENT_TIME_ORDER")
            else:
                last_stamp = stamp
            if event.get("success") is not True:
                failures.append(prefix + ":BOUNDARY_FAILURE:" + str(event.get("stage")))
        measured = case.get("measured", {})
        for key in ("integration_success", "return_success", "payload_retained"):
            if key not in measured:
                missing.append(prefix + ".measured." + key)
            elif measured[key] is not True:
                failures.append(prefix + ":" + key.upper())
        for key in ("input_map_revision", "output_map_revision", "sensor_occupied_cells", "floor_through_candidates"):
            if type(measured.get(key)) is not int or measured[key] < 0:
                missing.append(prefix + ".measured." + key)
        before, after = measured.get("input_map_revision"), measured.get("output_map_revision")
        if type(before) is int and type(after) is int:
            if after <= before or previous_revision is not None and before != previous_revision:
                failures.append(prefix + ":STALE_MAP_REVISION")
            previous_revision = after
        if type(measured.get("sensor_occupied_cells")) is int and measured["sensor_occupied_cells"] < 1:
            failures.append(prefix + ":NO_SENSOR_OCCUPANCY")
        if type(measured.get("floor_through_candidates")) is int and measured["floor_through_candidates"] != 0:
            failures.append(prefix + ":FLOOR_THROUGH")
        gt = case.get("gt_reference", {})
        for metric, limit in GT_LIMITS.items():
            value = gt.get(metric)
            if type(value) not in (int, float) or not math.isfinite(value):
                missing.append(prefix + ".gt_reference." + metric)
            elif abs(value) > limit or metric in ("xy_error_m", "drift_m") and value < 0:
                failures.append(prefix + ":GT_" + metric.upper())
        if "center_inside_hull" not in gt:
            missing.append(prefix + ".gt_reference.center_inside_hull")
        elif gt["center_inside_hull"] is not True:
            failures.append(prefix + ":GT_CENTER_OUTSIDE_HULL")
        denominator, numerator = gt.get("footprint_reference_cells"), gt.get("footprint_sensor_cells")
        if (type(denominator) is not int or denominator <= 0 or type(numerator) is not int
                or numerator < 0 or numerator > denominator):
            missing.append(prefix + ".gt_reference.positive_footprint_denominator")
        elif numerator / denominator < 0.40:
            failures.append(prefix + ":GT_SENSOR_COVERAGE")
    return {"status": "not_evaluated" if missing else "fail" if failures else "pass",
            "missing": missing, "failures": failures, "sample_count": len(cases),
            "claim": "finite two-box coverage; no population reliability claim"}


def capture_window(upstream_seconds, downstream_seconds):
    """Freeze windows only from positive, finite observations."""
    for name, samples in (("upstream", upstream_seconds), ("downstream", downstream_seconds)):
        if not samples or any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in samples):
            raise ValueError("not_evaluated: missing positive " + name + " latency samples")
    return {"pre_seconds": math.ceil(2 * max(upstream_seconds)),
            "post_seconds": math.ceil(max(downstream_seconds))}
