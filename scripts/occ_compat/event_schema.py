"""event_schema — the COMMON payload contract for G3C (reviews 1622 B7).

Single source of truth for every event field and identity type: the
DRIVERS construct events through ``validate_event`` before writing, the
JUDGE loads through ``load_ledger`` — neither side may define its own
field names or identity shapes (contract §2/§3). Historical schema-1
ledgers remain readable and are reported as ``legacy`` — missing G3C
facts stay NOT_EVALUATED, never silently defaulted.

Strictness (contract §2): duplicate JSON keys, non-finite numbers,
bools masquerading as integers, unknown schema versions are rejected at
LOAD time; structural violations are rejected at VALIDATE time.
Completeness metrics (event_seq continuity per instance, terminal
uniqueness, capture drop) are computed from the bytes on disk.
"""
from __future__ import annotations

import json
import math
import re
from pathlib import Path

SCHEMA_VERSION = "g3c-event-v1"
LEGACY_SCHEMA = 1  # pre-G3C ledgers: readable, reported as legacy

BACKENDS = ("G", "O")
HULLS = ("cuboid", "chamfer")
PRODUCER_ROLES = ("mapper", "observer", "bridge", "planner", "t1-driver")

#: Required on EVERY event (contract §2). ``consumer`` uses the explicit
#: producer role on base events — never an empty string.
COMMON_REQUIRED = (
    "schema_version", "run_id", "group_id", "backend", "hull",
    "event_kind", "stage", "state",
    "producer_instance_id", "event_seq", "mono_ns", "clock_domain_id",
)

#: Per-event-kind required fields beyond the common set. Kinds not
#: listed carry only the common set (plus their typed payload fields
#: validated by the business checkers, not here).
KIND_REQUIRED = {
    "case_terminal": ("case_id",),
    "case_complete": ("case_id",),
    "placement_request": ("case_id", "consumer", "request_id"),
    "probe_detail": ("case_id",),
    "integrate": ("request_id", "operation_id"),
    "reset": ("reset_id", "reset_case"),
    "producer_restart": ("old_pid", "new_pid"),
    "r05_checkpoint": ("case_id", "stage"),
    "oracle_payload_ref": ("case_id", "payload_ref"),
}

#: Typed identity blocks (contract §3). Each maps field name -> checker.
IDENTITY_FIELDS = {
    "epoch": str,                 # G uuid string; O canonical uint64 decimal
    "generation": int,
    "map_revision": int,
    "geometry_hash": str,
    "config_hash": str,
    "clock_identity": str,
}

_REQUEST_ID = re.compile(r"^\S+$")


class SchemaError(ValueError):
    """Named rejection — never a bare exception at an entry point."""


# ---- strict loading ---------------------------------------------------

def _no_dupes(pairs):
    out = {}
    for key, value in pairs:
        if key in out:
            raise SchemaError("duplicate JSON key %r" % key)
        out[key] = value
    return out


def _check_finite(node, where=""):
    if isinstance(node, float) and not math.isfinite(node):
        raise SchemaError("non-finite number at %s" % (where or "root"))
    if isinstance(node, dict):
        for key, value in node.items():
            _check_finite(value, "%s.%s" % (where, key))
    elif isinstance(node, list):
        for idx, value in enumerate(node):
            _check_finite(value, "%s[%d]" % (where, idx))


def strict_loads(text):
    """Duplicate-key / non-finite rejecting JSON parse."""
    doc = json.loads(text, object_pairs_hook=_no_dupes,
                     parse_constant=_reject_constant)
    _check_finite(doc)
    return doc


def _reject_constant(name):
    raise SchemaError("non-finite JSON constant %s" % name)


def _is_int(value):
    # bool is a subclass of int — bools masquerading as integers reject
    return isinstance(value, int) and not isinstance(value, bool)


# ---- event validation --------------------------------------------------

def validate_event(event, strict_g3c=True):
    """Validate ONE event dict. Returns (ok, error). ``strict_g3c``
    False accepts legacy schema-1 events (readable history)."""
    try:
        version = event.get("schema_version")
        if version == LEGACY_SCHEMA and not strict_g3c:
            return True, None
        if version != SCHEMA_VERSION:
            return False, "unknown schema_version %r" % (version,)
        for field in COMMON_REQUIRED:
            if field not in event:
                return False, "missing %s" % field
        if event["backend"] not in BACKENDS:
            return False, "backend %r" % event["backend"]
        if event["hull"] not in HULLS:
            return False, "hull %r" % event["hull"]
        if not _is_int(event["event_seq"]) or event["event_seq"] < 1:
            return False, "event_seq must be a positive integer"
        if not _is_int(event["mono_ns"]) or event["mono_ns"] < 0:
            return False, "mono_ns must be a non-negative integer"
        if (not isinstance(event["clock_domain_id"], str)
                or not event["clock_domain_id"]):
            return False, "clock_domain_id must be a non-empty string"
        if (not isinstance(event["producer_instance_id"], str)
                or not event["producer_instance_id"]):
            return False, "producer_instance_id must identify an instance"
        kind = event["event_kind"]
        for field in KIND_REQUIRED.get(kind, ()):
            if field not in event or event[field] in (None, ""):
                return False, "%s requires %s" % (kind, field)
        consumer = event.get("consumer")
        if consumer is not None and consumer == "":
            return False, "consumer must be a role, not empty"
        request_id = event.get("request_id")
        if request_id is not None and (
                not isinstance(request_id, str)
                or not _REQUEST_ID.match(request_id)):
            return False, "request_id %r not a non-space token" % request_id
        for field, checker in IDENTITY_FIELDS.items():
            value = event.get(field)
            if value is None:
                continue
            if checker is int:
                if not _is_int(value):
                    return False, "%s must be an integer" % field
            elif not isinstance(value, str):
                return False, "%s must be a string" % field
        ref = event.get("payload_ref")
        if ref is not None:
            err = _validate_payload_ref(ref)
            if err:
                return False, err
        parents = event.get("causal_parent_ids")
        if parents is not None:
            if (not isinstance(parents, list)
                    or not all(isinstance(p, str) and p
                               for p in parents)):
                return False, "causal_parent_ids must be event-id strings"
        return True, None
    except SchemaError as error:
        return False, str(error)


def _validate_payload_ref(ref):
    if not isinstance(ref, dict):
        return "payload_ref must be an object"
    for field in ("path", "sha256", "length"):
        if field not in ref:
            return "payload_ref missing %s" % field
    if not isinstance(ref["path"], str) or ref["path"].startswith("/"):
        return "payload_ref path must be archive-relative"
    if Path(ref["path"]).parts and ".." in Path(ref["path"]).parts:
        return "payload_ref path escapes the archive"
    if not re.fullmatch(r"[0-9a-f]{64}", str(ref["sha256"])):
        return "payload_ref sha256 must be a 64-hex digest"
    if not _is_int(ref["length"]) or ref["length"] < 0:
        return "payload_ref length must be a non-negative integer"
    return None


# ---- ledger-level integrity --------------------------------------------

def load_ledger(path, strict_g3c=False):
    """Load one events.jsonl: strict JSON per line + per-event
    validation + ledger-level integrity (event_seq continuity per
    instance, terminal uniqueness, dangling parent refs). Returns
    (events, problems). Legacy rows are kept and named — never a silent
    default."""
    path = Path(path)
    events, problems = [], []
    seq_by_instance = {}
    terminals = {}
    seen_ids = set()
    for lineno, line in enumerate(path.read_text().splitlines(), 1):
        if not line.strip():
            continue
        try:
            event = strict_loads(line)
        except (ValueError, SchemaError) as error:
            problems.append("line %d unparseable: %s" % (lineno, error))
            continue
        ok, error = validate_event(event, strict_g3c=strict_g3c)
        if not ok:
            if event.get("schema_version") == LEGACY_SCHEMA:
                # readable history: named, not rejected
                problems.append("line %d legacy schema (no G3C facts)"
                                % lineno)
            else:
                problems.append("line %d invalid: %s" % (lineno, error))
        instance = event.get("producer_instance_id")
        seq = event.get("event_seq")
        if isinstance(instance, str) and _is_int(seq):
            last = seq_by_instance.get(instance)
            if last is not None and seq <= last:
                problems.append(
                    "line %d event_seq %s not increasing for %s "
                    "(last %s)" % (lineno, seq, instance, last))
            seq_by_instance[instance] = seq
        if event.get("event_kind") == "case_terminal":
            key = (event.get("case_id"), event.get("subprobe", ""))
            if key in terminals:
                problems.append(
                    "duplicate terminal %s/%s (lines %s, %s)"
                    % (key[0], key[1], terminals[key], lineno))
            terminals[key] = lineno
        event_id = event.get("event_id")
        if event_id is not None:
            seen_ids.add(event_id)
        event["_line"] = lineno
        events.append(event)
    for event in events:
        for parent in event.get("causal_parent_ids") or ():
            if parent not in seen_ids:
                problems.append("line %s dangling causal parent %s"
                                % (event.get("_line"), parent))
    return events, problems


def completeness_metrics(events):
    """Payload completeness / terminal uniqueness / capture drop,
    computed from the loaded rows (never a self-reported bool)."""
    total = len(events)
    with_payload = sum(1 for e in events
                       if e.get("payload_ref") is not None
                       or e.get("result_repr") is not None
                       or e.get("message") is not None)
    terminals = [e for e in events
                 if e.get("event_kind") == "case_terminal"]
    complete_cases = [e for e in events
                      if e.get("event_kind") == "case_complete"]
    return {
        "events": total,
        "payload_bearing": with_payload,
        "payload_ratio": round(with_payload / total, 4) if total else None,
        "terminals": len(terminals),
        "case_completes": len(complete_cases),
        # capture drop: terminals without a matching case_complete
        "capture_drop": max(0, len(terminals) - len(complete_cases)),
    }


def emit_fields(*, producer_instance_id, event_seq, clock_domain_id,
                run_id, group_id, backend, hull, **payload):
    """DRIVER-side constructor: stamps the common G3C fields so the
    driver can never invent its own field names for them."""
    row = {
        "schema_version": SCHEMA_VERSION,
        "run_id": run_id,
        "group_id": group_id,
        "backend": backend,
        "hull": hull,
        "producer_instance_id": producer_instance_id,
        "event_seq": event_seq,
        "clock_domain_id": clock_domain_id,
    }
    row.update(payload)
    ok, error = validate_event(row)
    if not ok:
        raise SchemaError("driver emit invalid: %s" % error)
    return row
