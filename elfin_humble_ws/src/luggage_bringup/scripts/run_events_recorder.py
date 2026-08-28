#!/usr/bin/env python3
"""Structured run events for the active-loading pipeline.

``active_loading_bag_harness.py`` has always consumed six record kinds and
evaluated more than a dozen hard gates against them, but nothing in the tree
produced those records -- the only fixtures were hand written in the unit test.
This
module is that producer. The orchestrator writes one file per run, so a manual
(stepped) run and an automated matrix run are judged by the same tooling
against the same schema.

Two properties matter and are enforced here rather than at the call sites:

* Records carry no wall-clock state beyond ``t`` and a few duration fields, all
  listed in ``VOLATILE_FIELDS``. Dropping those makes two runs of the same seed
  directly comparable, which is how the manual front-end is proven not to have
  drifted from the automated path.
* A session touched by out-of-band service calls is marked, permanently, with a
  ``taint`` record. The harness refuses such a file, so a hand-poked session
  cannot be presented as acceptance evidence.

No ROS imports, so the schema is unit testable without a roscore.
"""

from __future__ import division

import json
import os
import re
import time

SCHEMA_VERSION = 1

KIND_SESSION = "session"
KIND_DETECTION = "detection"
KIND_PLACEMENT = "placement"
KIND_RELEASE = "release"
KIND_MAP = "map"
KIND_STATUS = "status"
KIND_FAILURE = "failure"
KIND_TAINT = "taint"

# Fields that legitimately differ between two runs of the same configuration:
# wall clock, durations, and how the run was driven. Excluded when comparing
# two runs for equality -- notably ``run_mode``, since proving the manual and
# automated paths agree is the whole point of that comparison.
VOLATILE_FIELDS = (
    "t", "cycle_sec", "started_at", "ended_at", "elapsed_sec",
    "run_mode", "events_path",
)

# A failure message starts with the machine-readable class the orchestrator
# assigned it, e.g. "RELEASE_FAILED_AT_CONTACT: vacuum ...".
_FAILURE_TOKEN = re.compile(r"^([A-Z][A-Z0-9_]{2,})\b")

UNCLASSIFIED = "UNCLASSIFIED"

# Idle is both the failure sink and the normal end of a run. These are the
# messages that mean the run ended on purpose; everything else that lands in
# Idle is a failure and is recorded as one.
TERMINAL_SUCCESS_PREFIXES = (
    "max placed reached",
    "orchestrator finished",
    "no luggage",
)


def failure_class(message):
    """Machine-readable class for an Idle-state message."""
    match = _FAILURE_TOKEN.match(str(message or "").strip())
    if match:
        return match.group(1)
    return UNCLASSIFIED


def is_terminal_success(message):
    text = str(message or "").strip().lower()
    return any(
        text.startswith(prefix) for prefix in TERMINAL_SUCCESS_PREFIXES)


def canonical_record(record):
    """Record with volatile fields removed, for run-to-run comparison."""
    return {
        key: value for key, value in record.items()
        if key not in VOLATILE_FIELDS
    }


def canonical_records(records):
    return [canonical_record(record) for record in records]


def load_events(path):
    records = []
    with open(path, "r") as stream:
        for line in stream:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def diff_events(left, right):
    """Human-readable differences between two runs' canonical records.

    Returns a list of strings; empty means the two runs are identical modulo
    timestamps and durations.
    """
    left_c = canonical_records(left)
    right_c = canonical_records(right)
    problems = []
    for index in range(max(len(left_c), len(right_c))):
        if index >= len(left_c):
            problems.append(
                "%d: only in right: %s"
                % (index, json.dumps(right_c[index], sort_keys=True)))
            continue
        if index >= len(right_c):
            problems.append(
                "%d: only in left: %s"
                % (index, json.dumps(left_c[index], sort_keys=True)))
            continue
        if left_c[index] != right_c[index]:
            problems.append(
                "%d: left=%s right=%s"
                % (
                    index,
                    json.dumps(left_c[index], sort_keys=True),
                    json.dumps(right_c[index], sort_keys=True),
                ))
    return problems


class RunEventsRecorder(object):
    """Appends one JSON object per line. An empty path disables recording."""

    def __init__(self, path, clock=None):
        self.path = str(path or "")
        self._clock = clock or time.time
        self._stream = None
        self._count = 0
        if not self.path:
            return
        directory = os.path.dirname(os.path.abspath(self.path))
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)
        self._stream = open(self.path, "w")

    @property
    def enabled(self):
        return self._stream is not None

    @property
    def count(self):
        return self._count

    def write(self, kind, **fields):
        if self._stream is None:
            return None
        record = {"kind": kind, "t": round(float(self._clock()), 3)}
        record.update(fields)
        self._stream.write(json.dumps(record, sort_keys=True) + "\n")
        # Flushed per record so a killed run keeps everything up to the kill,
        # and so a front-end can tail the file live.
        self._stream.flush()
        self._count += 1
        return record

    def session(self, **fields):
        fields.setdefault("schema_version", SCHEMA_VERSION)
        fields.setdefault("probe_touched", False)
        return self.write(KIND_SESSION, **fields)

    def detection(self, success, source, message=""):
        return self.write(
            KIND_DETECTION, success=bool(success), source=str(source),
            message=str(message))

    def placement(self, record, cycle_sec=None):
        fields = dict(record)
        fields.pop("kind", None)
        if cycle_sec is not None:
            fields["cycle_sec"] = round(float(cycle_sec), 3)
        return self.write(KIND_PLACEMENT, **fields)

    def release(self, released_at_contact, retreat_after_release):
        return self.write(
            KIND_RELEASE,
            released_at_contact=bool(released_at_contact),
            retreat_after_release=bool(retreat_after_release))

    def map_event(self, event, map_revision):
        return self.write(
            KIND_MAP, event=str(event), map_revision=int(map_revision))

    def status(self, placed_count, state=""):
        return self.write(
            KIND_STATUS, placed_count=int(placed_count), state=str(state))

    def failure(self, message, state=""):
        return self.write(
            KIND_FAILURE,
            failure_class=failure_class(message),
            message=str(message),
            state=str(state))

    def taint(self, reason=""):
        return self.write(KIND_TAINT, reason=str(reason))

    def close(self):
        if self._stream is not None:
            self._stream.close()
            self._stream = None


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="Compare two active-loading run event files.")
    parser.add_argument("left")
    parser.add_argument("right")
    parser.add_argument("--output", default="",
                        help="write the diff here as well as to stdout")
    args = parser.parse_args()

    problems = diff_events(load_events(args.left), load_events(args.right))
    header = "identical (modulo %s)" % ", ".join(VOLATILE_FIELDS)
    text = "\n".join(problems) if problems else header
    print(text)
    if args.output:
        with open(args.output, "w") as stream:
            stream.write(text + "\n")
    raise SystemExit(1 if problems else 0)


if __name__ == "__main__":
    main()
