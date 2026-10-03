#!/usr/bin/env python3
"""Evaluate joined two- or three-box evidence against a frozen manifest."""
import argparse
import json
from pathlib import Path

try:
    from luggage_gazebo.integration_acceptance import extract_acceptance
except Exception:  # pragma: no cover - exercised when package is absent
    extract_acceptance = None


def extract_box_events(evidence):
    """Return ordered box event groups without assuming exactly two boxes."""
    events = evidence.get("events") or evidence.get("timeline") or []
    boxes = {}
    handoffs = []
    for event in events:
        box_id = event.get("box_id") or event.get("sequence_id")
        if box_id:
            boxes.setdefault(str(box_id), []).append(event)
        if event.get("type") in ("revision_handoff", "map_revision_handoff"):
            handoffs.append(event)
    if not boxes and isinstance(evidence.get("boxes"), list):
        for index, box in enumerate(evidence["boxes"]):
            box_id = str(box.get("box_id") or box.get("sequence_id") or index)
            boxes[box_id] = list(box.get("events") or [box])
    ordered = [{"box_id": key, "events": value} for key, value in sorted(boxes.items())]
    return ordered, handoffs


REQUIRED_EVENT_ORDER = ("pick", "place", "release", "integration", "map_publish")


def _event_identity(event):
    return (
        event.get("request_id"),
        event.get("cycle_id"),
        event.get("map_revision"),
    )


def validate_box_chain(box):
    events = box["events"]
    failures = []
    types = [event.get("type") for event in events]
    for required in REQUIRED_EVENT_ORDER:
        if types.count(required) != 1:
            failures.append("%s expected exactly one %s event" % (box["box_id"], required))
    positions = [types.index(required) for required in REQUIRED_EVENT_ORDER if required in types]
    if positions != sorted(positions):
        failures.append("%s event order must be pick->place->release->integration->map_publish" % box["box_id"])
    identities = {_event_identity(event) for event in events if event.get("type") in REQUIRED_EVENT_ORDER}
    if len(identities) != 1 or any(value is None for value in next(iter(identities), ())):
        failures.append("%s request/cycle/map identity mismatch" % box["box_id"])
    causal_ids = {event.get("causal_id") for event in events if event.get("type") in REQUIRED_EVENT_ORDER}
    if len(causal_ids) != 1 or None in causal_ids:
        failures.append("%s causal identity mismatch" % box["box_id"])
    return failures


def validate_handoffs(handoffs, expected_count):
    failures = []
    required = max(0, expected_count - 1)
    if len(handoffs) != required:
        failures.append("revision handoffs: observed %d expected exactly %d" % (len(handoffs), required))
        return failures
    expected_pairs = [(index, index + 1) for index in range(1, expected_count)]
    observed_pairs = [(event.get("from"), event.get("to")) for event in handoffs]
    if observed_pairs != expected_pairs:
        failures.append("revision handoffs must be %s, observed %s" % (expected_pairs, observed_pairs))
    seen = set()
    for event in handoffs:
        key = (event.get("box_id"), event.get("request_id"), event.get("from"), event.get("to"))
        if key in seen:
            failures.append("duplicate revision handoff %s" % (key,))
        seen.add(key)
        if event.get("box_id") and event.get("box_id") not in event.get("to_box_id", event.get("box_id")):
            failures.append("cross-box handoff mismatch for %s" % event.get("box_id"))
    return failures


def extract_acceptance_variable(evidence, frozen):
    """LEGACY shallow chain check — NOT the formal M3 judge (ST09-I S5).

    Verifies only the five-event chain, causal identity, and handoff
    count; physical gates, 11-stage completeness, revision identity, and
    frozen-hash discipline are all outside its scope, so a pass here says
    nothing about placement quality. main() never routes formal judgment
    here; retained for triage of partial/legacy evidence only.
    """
    boxes, handoffs = extract_box_events(evidence)
    expected_count = int(frozen.get("box_count") or len(frozen.get("sequence_ids") or []) or len(boxes))
    missing = []
    failures = []
    if expected_count not in (2, 3):
        failures.append("expected box_count must be 2 or 3")
    if len(boxes) != expected_count:
        missing.append("box events: observed %d expected %d" % (len(boxes), expected_count))
    failures.extend(validate_handoffs(handoffs, expected_count))
    for box in boxes:
        chain_failures = validate_box_chain(box)
        if chain_failures:
            failures.extend(chain_failures)
    status = "pass" if not missing and not failures else ("not_evaluated" if missing else "fail")
    return {
        "status": status,
        "missing": missing,
        "failures": failures,
        "sample_count": len(boxes),
        "box_count": len(boxes),
        "revision_handoff_count": len(handoffs),
        "boxes": boxes,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence", required=True, type=Path)
    parser.add_argument("--frozen", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    # Reserve output before parsing so malformed/truncated inputs leave evidence.
    with args.output.open("x", encoding="utf-8") as stream:
        try:
            evidence = json.loads(args.evidence.read_text())
            frozen = json.loads(args.frozen.read_text())
            # [ST09-I S5] formal judgment is the unified strict judge for
            # BOTH two and three boxes (same 11-stage completeness and
            # physical gates); an unknown workload size is refused, never
            # silently downgraded to the shallow legacy chain check.
            box_count = frozen.get("box_count", 2)
            if extract_acceptance is not None and type(box_count) is int \
                    and box_count in (2, 3):
                result = extract_acceptance(evidence, frozen)
            else:
                result = {"status": "not_evaluated",
                          "missing": ["frozen.box_count must be 2 or 3 for "
                                      "formal judgment"],
                          "failures": [], "sample_count": 0}
        except (ValueError, KeyError, TypeError, AttributeError, OSError) as error:
            result = {"status": "not_evaluated", "missing": [str(error)], "failures": [], "sample_count": 0}
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    return 0 if result["status"] == "pass" else 2


if __name__ == "__main__":
    raise SystemExit(main())
