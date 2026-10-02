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


def extract_acceptance_variable(evidence, frozen):
    boxes, handoffs = extract_box_events(evidence)
    expected_count = int(frozen.get("box_count") or len(frozen.get("sequence_ids") or []) or len(boxes))
    missing = []
    failures = []
    if expected_count not in (2, 3):
        failures.append("expected box_count must be 2 or 3")
    if len(boxes) != expected_count:
        missing.append("box events: observed %d expected %d" % (len(boxes), expected_count))
    required_handoffs = max(0, expected_count - 1)
    if len(handoffs) < required_handoffs:
        missing.append("revision handoffs: observed %d expected %d" % (len(handoffs), required_handoffs))
    for box in boxes:
        types = {event.get("type") for event in box["events"]}
        for required in ("pick", "attach", "release", "map_publish"):
            if required not in types:
                missing.append("%s missing %s" % (box["box_id"], required))
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
            if extract_acceptance is not None and int(frozen.get("box_count") or 2) == 2:
                result = extract_acceptance(evidence, frozen)
            else:
                result = extract_acceptance_variable(evidence, frozen)
        except (ValueError, KeyError, TypeError, AttributeError, OSError) as error:
            result = {"status": "not_evaluated", "missing": [str(error)], "failures": [], "sample_count": 0}
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    return 0 if result["status"] == "pass" else 2


if __name__ == "__main__":
    raise SystemExit(main())
