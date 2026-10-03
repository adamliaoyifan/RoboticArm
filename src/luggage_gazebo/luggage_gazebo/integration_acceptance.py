"""Finite two- or three-box integration acceptance over joined, measured boundary events.

This extractor never advances a campaign or labels synthetic fixtures live
coverage. GT placement checks are separate from measured map admission.

[ST09-I S5] Unified N-box form: a three-box session is judged by the SAME
11-stage boundary completeness and physical gates as two boxes, plus the
session chain (box sequence, revision handoffs, per-event revision
identity). The shallow chain-only check
(``frame_migration_st09_extract.extract_acceptance_variable``) is NOT a
formal M3 judge.

Ordering semantics: event order is expressed by ``event_seq`` (strictly
increasing within a case). ``acquisition_stamp_ns`` is the RAW measured
value — a stamp that contradicts the sequence is REPORTED
(``EVENT_CAUSAL_ORDER``), never rewritten; ``receiver_stamp_ns`` is the
driver's monotonic receipt time alongside it. Two-box evidence without
``event_seq`` falls back to the legacy stamp-monotonicity check.

Revision semantics (mandatory for box_count >= 3, validated whenever
present for two boxes): every boundary event carries
``input_map_revision`` — the revision the cycle CONSUMED, identical
within the case and equal to ``measured.input_map_revision``; the commit
stages additionally carry ``output_map_revision`` equal to
``measured.output_map_revision``. Events are never required to share ONE
revision across the cycle: integration legitimately changes it, and the
next box must consume the previous box's output (``input(k) ==
output(k-1)``, asserted, never repaired).
"""
import math

EVENT_ORDER = (
    "observation", "slot", "selected_path", "probe", "execution", "attach",
    "release", "measured_verification", "measured_integration", "map_update", "return_complete",
)
FROZEN_HASHES = ("runner", "extractor", "resolved_profile", "scene", "model", "atlas", "urdf", "srdf", "messages", "hull")
GT_LIMITS = {"xy_error_m": 0.04, "z_error_m": 0.03, "drift_m": 0.02,
             "roll_deg": 2.0, "pitch_deg": 2.0}
#: Stages that commit the cycle's map work; they carry the output revision.
COMMIT_STAGES = ("measured_integration", "map_update")
#: Session-level handoff rows may live under evidence["handoffs"] or ride
#: the flat event stream with one of these types.
HANDOFF_TYPES = ("revision_handoff", "map_revision_handoff")


def _handoff_rows(evidence):
    rows = evidence.get("handoffs")
    if isinstance(rows, list):
        return [row for row in rows if isinstance(row, dict)]
    events = evidence.get("events")
    if isinstance(events, list):
        return [event for event in events
                if isinstance(event, dict)
                and event.get("type") in HANDOFF_TYPES]
    return []


def _revision_ints(values):
    """(all_int, unique_value) over a raw collection of revision values."""
    ints = [value for value in values if type(value) is int]
    if len(ints) != len(list(values)) or not ints:
        return False, None
    return len(set(ints)) == 1, ints[0]


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
    # -- workload shape (S5: box_count generalizes the two-box form) -----
    box_count = frozen.get("box_count", 2)
    if type(box_count) is not int or box_count not in (2, 3):
        missing.append("frozen.box_count")
        box_count = 2
    sequence = frozen.get("sequence_ids")
    if sequence is None:
        sequence = ["carryon", "standard"][:box_count]
    if (not isinstance(sequence, list) or len(sequence) != box_count
            or any(not isinstance(name, str) or not name for name in sequence)):
        missing.append("frozen.sequence_ids")
        sequence = []
    strict_revisions = box_count >= 3
    if not expected_hashes:
        missing.append("frozen.hashes")
    elif evidence.get("hashes") != expected_hashes:
        failures.append("FROZEN_INPUT_CHANGED")
    if evidence.get("complete") is not True:
        missing.append("complete evidence trailer")
    cases = evidence.get("cases", [])
    if not isinstance(cases, list) or len(cases) != box_count:
        missing.append("%d cases" % box_count)
        cases = []
    previous_revision = None
    seen_requests = set()
    seen_box_ids = set()
    case_revisions = []
    for index, case in enumerate(cases):
        prefix = f"case[{index}]"
        if not isinstance(case, dict):
            missing.append(prefix)
            continue
        box_id = case.get("box_id")
        if sequence:
            # box_id is "<sequence_id>" or "<sequence_id>@<ordinal>"; the
            # ordinal form is mandatory-unique for three boxes (two raw
            # "carryon" ids would otherwise merge indistinguishably).
            expected_box = sequence[index]
            text = str(box_id or "")
            if text.split("@", 1)[0] != expected_box:
                failures.append(prefix + ":BOX_ORDER")
            if "@" in text:
                try:
                    ordinal = int(text.split("@", 1)[1])
                except ValueError:
                    ordinal = -1
                if ordinal != index + 1:
                    failures.append(prefix + ":BOX_ORDINAL")
        if isinstance(box_id, str) and box_id:
            if box_id in seen_box_ids:
                failures.append(prefix + ":DUPLICATE_BOX_ID")
            seen_box_ids.add(box_id)
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
        has_seq = any(type(e.get("event_seq")) is int for e in events)
        last_seq = None
        last_acquisition = None
        last_receiver = None
        for event in events:
            if event.get("request_id") != request_id:
                failures.append(prefix + ":UNJOINED_EVENT")
            for field, frozen_key in (("scene_revision", "scene"), ("geometry_hash", "hull")):
                if event.get(field) != expected_hashes.get(frozen_key) or field not in event:
                    failures.append(prefix + ":BOUNDARY_IDENTITY:" + field)
            if has_seq:
                seq = event.get("event_seq")
                if type(seq) is not int or (last_seq is not None and seq <= last_seq):
                    failures.append(prefix + ":EVENT_SEQ_ORDER")
                else:
                    last_seq = seq
            acquisition = event.get("acquisition_stamp_ns", event.get("stamp_ns"))
            receiver = event.get("receiver_stamp_ns")
            if type(acquisition) is not int:
                missing.append(prefix + ".acquisition_stamp_ns")
            else:
                if last_acquisition is not None and acquisition < last_acquisition:
                    # RAW measured value preserved; the sequence disagrees.
                    failures.append(prefix + ":EVENT_CAUSAL_ORDER")
                last_acquisition = acquisition
            if receiver is not None:
                if type(receiver) is not int:
                    missing.append(prefix + ".receiver_stamp_ns")
                elif last_receiver is not None and receiver < last_receiver:
                    failures.append(prefix + ":EVENT_CAUSAL_ORDER")
                else:
                    last_receiver = receiver
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
        case_revisions.append((before, after))
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
        # -- per-event revision identity (never one disguised revision) --
        inputs = [event.get("input_map_revision") for event in events]
        outputs = [event.get("output_map_revision") for event in events
                   if event.get("stage") in COMMIT_STAGES]
        if strict_revisions or any(value is not None for value in inputs):
            unique, value = _revision_ints(inputs)
            if not unique:
                failures.append(prefix + ":EVENT_REVISION_IDENTITY")
            elif type(before) is int and value != before:
                failures.append(prefix + ":EVENT_REVISION_IDENTITY")
            commit_unique, commit_value = _revision_ints(outputs)
            if strict_revisions:
                if not commit_unique:
                    failures.append(prefix + ":EVENT_REVISION_OUTPUT")
                elif type(after) is int and commit_value != after:
                    failures.append(prefix + ":EVENT_REVISION_OUTPUT")
            elif outputs and not commit_unique:
                failures.append(prefix + ":EVENT_REVISION_OUTPUT")
    # -- session revision handoffs (mandatory for three boxes) ----------
    handoffs = _handoff_rows(evidence)
    if box_count >= 3 and len(handoffs) != box_count - 1:
        missing.append("revision handoffs: observed %d expected %d"
                       % (len(handoffs), box_count - 1))
    expected_pairs = [(index, index + 1) for index in range(1, box_count)]
    observed_pairs = [(row.get("from"), row.get("to")) for row in handoffs]
    chain_complete = (len(case_revisions) == box_count
                      and all(type(pair[0]) is int and type(pair[1]) is int
                              for pair in case_revisions))
    if handoffs and observed_pairs != expected_pairs[:len(observed_pairs)]:
        failures.append("revision handoffs must be %s, observed %s"
                        % (expected_pairs, observed_pairs))
    seen = set()
    for position, row in enumerate(handoffs):
        key = (row.get("box_id"), row.get("request_id"), row.get("from"), row.get("to"))
        if key in seen:
            failures.append("duplicate revision handoff %s" % (key,))
        seen.add(key)
        if not chain_complete or position >= len(case_revisions) - 1:
            continue
        output_revision = case_revisions[position][1]
        input_revision = case_revisions[position + 1][0]
        if row.get("from_revision") != output_revision \
                or row.get("to_revision") != input_revision:
            failures.append("revision handoff %s->%s does not join the "
                            "committed chain" % (row.get("from"), row.get("to")))
    return {"status": "not_evaluated" if missing else "fail" if failures else "pass",
            "missing": missing, "failures": failures, "sample_count": len(cases),
            "box_count": box_count, "revision_handoff_count": len(handoffs),
            "claim": "finite %d-box coverage; no population reliability claim" % box_count,
    }


def capture_window(upstream_seconds, downstream_seconds):
    """Freeze windows only from positive, finite observations."""
    for name, samples in (("upstream", upstream_seconds), ("downstream", downstream_seconds)):
        if not samples or any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in samples):
            raise ValueError("not_evaluated: missing positive " + name + " latency samples")
    return {"pre_seconds": math.ceil(2 * max(upstream_seconds)),
            "post_seconds": math.ceil(max(downstream_seconds))}
