"""Derive two-box rehearsal evidence from motion-occ trial artifacts.

ROS-free. The scored campaign is not advanced: callers write this document
under an explicit rehearsal directory and never into the scoring suite.

The eleven extractor stages keep their acquisition time in ``acquisition_ns``.
``stamp_ns`` is that time projected onto a strictly increasing join order,
because attach is physically earlier than place execution while the frozen
event order lists it later. The live MD5 scene revision stays on the case;
extractor identity fields are filled later by ``bind_evidence`` from the
frozen file digests (those digests are sha256, and the live scene revision
is MD5).
"""
from __future__ import annotations

import hashlib
import math
from pathlib import Path

from luggage_gazebo.integration_acceptance import EVENT_ORDER, FROZEN_HASHES

BOX_ORDER = ("carryon", "standard")

# Post-release camera sample in place_smoke_driver._capture_success_debug.
RELEASE_SAMPLE_INTERVAL_S = 0.35

_MOTION_STATES = ("DESCEND", "INSERT", "TRAVERSE", "TRANSIT")
_RETURN_STUCK = ("GOTO_FAILED", "EXIT_FAILED_STUCK")
_REFUSAL_NOT_A_FAULT = ("REHEARSAL_FAULT_NOT_REFUSED", "REHEARSAL_FAULT_NO_RUNTIME")


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def hash_inputs(groups):
    """sha256 for one file; sha256 of per-file digests when a key has several.

    Every ``FROZEN_HASHES`` key is required. A missing path raises rather
    than producing a partial manifest.
    """
    if not isinstance(groups, dict):
        raise ValueError("frozen inputs must be a mapping of hash key to files")
    missing = [key for key in FROZEN_HASHES if not groups.get(key)]
    if missing:
        raise ValueError("missing frozen inputs: " + ",".join(missing))
    hashes = {}
    for key in FROZEN_HASHES:
        paths = [Path(item) for item in groups[key]]
        for path in paths:
            if not path.is_file():
                raise ValueError("missing frozen file %s: %s" % (key, path))
        if len(paths) == 1:
            hashes[key] = sha256_file(paths[0])
        else:
            digest = hashlib.sha256()
            for path in paths:
                digest.update(bytes.fromhex(sha256_file(path)))
            hashes[key] = digest.hexdigest()
    return hashes


def frozen_document(hashes, release_sample_interval_s, drift_sample_interval_s,
                    replay_command, capture_window=None):
    """Manifest consumed by ``extract_acceptance``."""
    if set(hashes) != set(FROZEN_HASHES):
        raise ValueError("frozen hashes must be exactly %s" % (FROZEN_HASHES,))
    document = {
        "hashes": {key: hashes[key] for key in FROZEN_HASHES},
        "release_sample_interval_s": float(release_sample_interval_s),
        "drift_sample_interval_s": float(drift_sample_interval_s),
        "event_order": list(EVENT_ORDER),
        "replay_command": list(replay_command),
    }
    if capture_window is not None:
        document["capture_window"] = dict(capture_window)
    return document


def _finite(value):
    return type(value) in (int, float) and math.isfinite(float(value))


def _stamp_ns(t_ros):
    if not _finite(t_ros) or float(t_ros) < 0:
        return None
    return int(round(float(t_ros) * 1_000_000_000))


def _state_time(timeline, names):
    found = None
    for row in timeline or []:
        if isinstance(row, dict) and row.get("to") in names:
            found = row.get("t_ros")
    return found


def _marks(trial):
    extras = trial.get("extras") or {}
    found = {}
    for mark in extras.get("rehearsal_marks") or []:
        if isinstance(mark, dict) and mark.get("name"):
            found[str(mark["name"])] = mark.get("t_ros")
    return found


def _enforce_increasing(events):
    """Project acquisition times onto a strictly increasing join order."""
    last = -1
    for event in events:
        acquisition = event.get("acquisition_ns")
        stamp = acquisition if type(acquisition) is int else None
        if stamp is None or stamp <= last:
            if last < 0:
                return False
            stamp = last + 1
        event["stamp_ns"] = stamp
        last = stamp
    return True


def _probe_ok(selector):
    probe = (selector or {}).get("probe") or {}
    if str(probe.get("mode") or "") != "on":
        return False
    if type(probe.get("probed")) is not int or probe["probed"] < 1:
        return False
    if type(probe.get("unavailable")) is not int or probe["unavailable"] != 0:
        return False
    return bool((selector or {}).get("winner_method"))


def _success(stage, trial, timeline):
    extras = trial.get("extras") or {}
    fail = str(trial.get("fail_code") or "")
    states = {row.get("to") for row in timeline if isinstance(row, dict)}
    integrate = extras.get("integrate") or {}
    if stage == "observation":
        return "CARRY_READY" in states and bool(trial.get("catalog_id"))
    if stage == "slot":
        return "SLOT_RESOLVED" in states and fail != "PLACE_SLOT_INVALID"
    if stage == "selected_path":
        return "SEQUENCE_BUILT" in states and bool((extras.get("selector") or {}).get("winner_method"))
    if stage == "probe":
        return _probe_ok(extras.get("selector"))
    if stage == "execution":
        return "DESCEND" in states and not fail.startswith("PLACE_PLAN_")
    if stage == "attach":
        return trial.get("vac_attach") is True
    if stage == "release":
        return "RELEASED" in states and fail != "VACUUM_DETACH"
    if stage == "measured_verification":
        return "VERIFIED" in states and not fail.startswith("PLACE_VERIFY")
    if stage == "measured_integration":
        return integrate.get("success") is True
    if stage == "map_update":
        before, after = integrate.get("expected_map_revision"), integrate.get("resulting_map_revision")
        return type(before) is int and type(after) is int and after > before
    if stage == "return_complete":
        return trial.get("place_state") == "HOME" and fail not in _RETURN_STUCK
    return False


def _acquisitions(trial, timeline, marks):
    selected = _state_time(timeline, ("SEQUENCE_BUILT",))
    execution = _state_time(timeline, _MOTION_STATES)
    verified = _state_time(timeline, ("VERIFIED",))
    integrated = marks.get("integration")
    if integrated is None:
        integrated = verified
    return {
        "observation": marks.get("observation", _state_time(timeline, ("CARRY_READY",))),
        "slot": _state_time(timeline, ("SLOT_RESOLVED",)),
        "selected_path": selected,
        "probe": selected,
        "execution": execution,
        "attach": execution,
        "release": _state_time(timeline, ("RELEASED",)),
        "measured_verification": verified,
        "measured_integration": integrated,
        "map_update": integrated,
        "return_complete": _state_time(timeline, ("HOME",)),
    }


def case_from_artifacts(trial):
    """One extractor case. Missing anchors are omitted, never invented."""
    trial = dict(trial or {})
    extras = dict(trial.get("extras") or {})
    timeline = list(extras.get("state_timeline") or [])
    marks = _marks(trial)
    box_id = str(trial.get("catalog_id") or "")
    request_id = str(extras.get("rehearsal_request_id") or "")
    if not request_id:
        request_id = "%s-%s" % (box_id or "box", trial.get("index"))
    times = _acquisitions(trial, timeline, marks)
    events = []
    for stage in EVENT_ORDER:
        acquisition = _stamp_ns(times.get(stage))
        if acquisition is None:
            continue
        events.append({
            "stage": stage,
            "request_id": request_id,
            "acquisition_ns": acquisition,
            "success": _success(stage, trial, timeline),
        })
    if not _enforce_increasing(events):
        events = []
    integrate = extras.get("integrate") or {}
    measured = {
        "integration_success": integrate.get("success") is True,
        "return_success": _success("return_complete", trial, timeline),
        "payload_retained": trial.get("vac_attach") is True and trial.get("lost_payload") is False,
    }
    for source, dest in (
            ("expected_map_revision", "input_map_revision"),
            ("resulting_map_revision", "output_map_revision"),
            ("n_sensor_occupied", "sensor_occupied_cells"),
            ("floor_through", "floor_through_candidates")):
        if type(integrate.get(source)) is int and integrate[source] >= 0:
            measured[dest] = integrate[source]
    gt = {}
    for source, dest in (("err_xy", "xy_error_m"), ("err_z", "z_error_m"), ("drift", "drift_m")):
        if _finite(trial.get(source)):
            gt[dest] = float(trial[source])
    for source, dest in (("roll", "roll_deg"), ("pitch", "pitch_deg")):
        if _finite(trial.get(source)):
            gt[dest] = math.degrees(float(trial[source]))
    if type(trial.get("inside_inner_box")) is bool:
        gt["center_inside_hull"] = trial["inside_inner_box"]
    if type(integrate.get("total_cells")) is int and type(integrate.get("hit_cells")) is int:
        gt["footprint_reference_cells"] = integrate["total_cells"]
        gt["footprint_sensor_cells"] = integrate["hit_cells"]
    return {
        "box_id": box_id,
        "request_id": request_id,
        "events": events,
        "measured": measured,
        "gt_reference": gt,
        "latency_marks": [
            {"name": name, "t_ros": float(value)}
            for name, value in marks.items() if _finite(value)
        ],
        "live_scene_revision": str(extras.get("live_scene_revision") or ""),
        "live_geometry_hash": str(extras.get("live_geometry_hash") or ""),
    }


def _cases_complete(cases):
    if not isinstance(cases, list) or len(cases) != 2:
        return False
    if [case.get("box_id") for case in cases] != list(BOX_ORDER):
        return False
    for case in cases:
        stages = [event.get("stage") for event in case.get("events") or []]
        if stages != list(EVENT_ORDER):
            return False
    return True


def build_evidence(cases):
    """Unbound evidence. ``bind_evidence`` adds the frozen digests."""
    return {
        "namespace": "rehearsal",
        "scoring_writes": False,
        "complete": _cases_complete(cases),
        "cases": list(cases),
        "latency": latency_samples(cases),
    }


def bind_evidence(evidence, hashes):
    """Stamp every event with the frozen scene and hull digests."""
    if set(hashes) != set(FROZEN_HASHES):
        raise ValueError("frozen hashes must be exactly %s" % (FROZEN_HASHES,))
    bound = {
        "namespace": evidence.get("namespace", "rehearsal"),
        "scoring_writes": False,
        "complete": evidence.get("complete") is True,
        "hashes": {key: hashes[key] for key in FROZEN_HASHES},
        "cases": [],
        "latency": evidence.get("latency") or latency_samples(evidence.get("cases") or []),
    }
    for case in evidence.get("cases") or []:
        copied = {
            "box_id": case.get("box_id"),
            "request_id": case.get("request_id"),
            "measured": dict(case.get("measured") or {}),
            "gt_reference": dict(case.get("gt_reference") or {}),
            "latency_marks": list(case.get("latency_marks") or []),
            "live_scene_revision": case.get("live_scene_revision") or "",
            "live_geometry_hash": case.get("live_geometry_hash") or "",
            "events": [],
        }
        for event in case.get("events") or []:
            stamped = dict(event)
            stamped["scene_revision"] = hashes["scene"]
            stamped["geometry_hash"] = hashes["hull"]
            copied["events"].append(stamped)
        bound["cases"].append(copied)
    bound["complete"] = _cases_complete(bound["cases"]) and evidence.get("complete") is True
    return bound


def assert_live_identity(evidence, scene_md5, hull_digest):
    """The run's live scene revision and hull must be the files just hashed."""
    cases = evidence.get("cases") or []
    if len(cases) != 2:
        raise ValueError("live identity requires two cases")
    for index, case in enumerate(cases):
        if case.get("live_geometry_hash") != hull_digest:
            raise ValueError("case[%d] live hull does not match frozen hull" % index)
        if case.get("live_scene_revision") != scene_md5:
            raise ValueError("case[%d] live scene revision does not match scene file" % index)


def latency_samples(cases):
    """Upstream is detect-to-placement-response; downstream is release-to-integration."""
    upstream, downstream = [], []
    for case in cases or []:
        marks = {}
        for mark in case.get("latency_marks") or []:
            if isinstance(mark, dict) and _finite(mark.get("t_ros")):
                marks[mark.get("name")] = float(mark["t_ros"])
        if "observation" in marks and "placement_response" in marks:
            delta = marks["placement_response"] - marks["observation"]
            if delta > 0:
                upstream.append(delta)
        release = _event_acquisition_s(case, "release")
        integrated = _event_acquisition_s(case, "measured_integration")
        if release is not None and integrated is not None:
            delta = integrated - release
            if delta > 0:
                downstream.append(delta)
    return {"upstream_seconds": upstream, "downstream_seconds": downstream}


def _event_acquisition_s(case, stage):
    for event in case.get("events") or []:
        if event.get("stage") == stage and type(event.get("acquisition_ns")) is int:
            return event["acquisition_ns"] / 1_000_000_000
    return None


def fault_window_ok(fault, refusal_code, before, after, after_recovery, recovered_ready):
    """Zero new motion or commit, and carrying state preserved across recovery."""
    if fault not in ("pre_pick", "pre_place"):
        return False
    if (not isinstance(refusal_code, str) or not refusal_code
            or refusal_code in _REFUSAL_NOT_A_FAULT):
        return False
    if recovered_ready is not True:
        return False
    for window in (after, after_recovery):
        if not isinstance(window, dict) or not isinstance(before, dict):
            return False
        if window.get("segments") != before.get("segments"):
            return False
        if window.get("commits") != before.get("commits"):
            return False
    if fault == "pre_place":
        return bool(before.get("attached") and after.get("attached")
                    and after_recovery.get("attached"))
    return not after.get("attached") and not after_recovery.get("attached")


def throughput_degradation(live_rate, replay_rate):
    """Fractional slowdown of replay throughput against the live record rate."""
    for name, value in (("live", live_rate), ("replay", replay_rate)):
        if not _finite(value) or float(value) <= 0:
            raise ValueError("not_evaluated: missing positive %s rate" % name)
    return (float(live_rate) - float(replay_rate)) / float(live_rate)


def median_degradation(samples):
    if not samples:
        raise ValueError("not_evaluated: no replay samples")
    ordered = sorted(float(value) for value in samples)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


RUN_BUDGET_BYTES = 1024 ** 3
TOTAL_BUDGET_BYTES = 4 * 1024 ** 3


def budget_allows(run_bytes, total_bytes, run_limit=RUN_BUDGET_BYTES,
                  total_limit=TOTAL_BUDGET_BYTES):
    """False once the next retained byte would reach a budget."""
    if type(run_bytes) is not int or type(total_bytes) is not int:
        raise ValueError("not_evaluated: byte counts must be ints")
    if run_bytes < 0 or total_bytes < 0:
        raise ValueError("not_evaluated: negative byte count")
    return run_bytes < run_limit and total_bytes < total_limit


def topic_counts_match(live_counts, replay_counts):
    """Per-topic message counts. A missing or short replay stream fails."""
    if not isinstance(live_counts, dict) or not live_counts:
        raise ValueError("not_evaluated: live topic counts missing")
    if not isinstance(replay_counts, dict):
        raise ValueError("not_evaluated: replay topic counts missing")
    mismatches = []
    for topic, count in sorted(live_counts.items()):
        if type(count) is not int or count < 0:
            raise ValueError("not_evaluated: bad live count for %s" % topic)
        other = replay_counts.get(topic)
        if other != count:
            mismatches.append({"topic": topic, "live": count, "replay": other})
    return mismatches
