"""Replay a segmented ledger and reproduce the judge verdict.

A missing segment field is incomplete replay. It cannot pass, and it is not
filled in from another record.
"""
from __future__ import annotations

import json

import repair_s0_contract as contract
import repair_s0_judge as judge


def _present(row, fields):
    """Copy only fields that are already stored. Do not invent a missing one."""
    return {key: row[key] for key in fields if key in row}


def _ledger_schema(inputs):
    """The ledger version is what the rows carry, never an upgrade of them."""
    if inputs and all(
            key in inputs[0] for key in ("publish_planned_ns", "input_stamp_ns", "clock_domain")):
        return contract.SCHEMA_V3
    return contract.SCHEMA


def segment_view(case):
    """Project a case into the four acceptance boundaries."""
    inputs = case.get("inputs") or []
    schema = _ledger_schema(inputs)
    publish_fields = contract.PUBLISH_FIELDS_V3 if schema == contract.SCHEMA_V3 else contract.PUBLISH_FIELDS
    integrate_fields = (contract.INTEGRATE_FIELDS_V3 if schema == contract.SCHEMA_V3
                        else contract.INTEGRATE_FIELDS)
    query_fields = contract.QUERY_FIELDS_V3 if schema == contract.SCHEMA_V3 else contract.QUERY_FIELDS
    return {
        "schema": schema,
        "gate": case["gate"],
        "publish_to_callback": [_present(row, publish_fields) for row in inputs],
        "integrate_to_receipt": [_present(row, integrate_fields) for row in inputs],
        "query_chain": [_present(row, query_fields) for row in case.get("queries") or []],
        "resource_and_recovery": {
            "health_times_s": case.get("health_times_s"),
            "resources": case.get("resources"),
            "in_flight_after_drain": case.get("in_flight_after_drain"),
            "legacy_immediate_first": case.get("legacy_immediate_first"),
            "planned_input_ids": case.get("planned_input_ids"),
            "planned_query_ids": case.get("planned_query_ids"),
            "planned_groups": case.get("planned_groups"),
            "planned_window_ids": case.get("planned_window_ids"),
            "runs": case.get("runs"),
            "holds": case.get("holds"),
            "operations": case.get("operations"),
            "windows": case.get("windows"),
            "groups": case.get("groups"),
            "id": case.get("id"),
            "cloud": case.get("cloud"),
            "integrate_ms": case.get("integrate_ms"),
            "init_s": case.get("init_s"),
            "warm_init_s": case.get("warm_init_s"),
            "rss_kib": case.get("rss_kib"),
            **{metric: case.get(metric) for metric in contract.QUERY_METRICS},
        },
    }


def _missing_fields(rows, fields):
    missing = []
    for index, row in enumerate(rows):
        if not isinstance(row, dict):
            missing.append("%d" % index)
            continue
        for field in fields:
            if field not in row:
                missing.append("%d.%s" % (index, field))
    return missing


def case_from_segments(segments):
    missing = []
    for name in ("publish_to_callback", "integrate_to_receipt", "query_chain", "resource_and_recovery"):
        if name not in segments:
            missing.append(name)
    schema = segments.get("schema")
    if missing or "gate" not in segments or schema not in (
            contract.SCHEMA_V3, contract.SCHEMA, contract.SCHEMA_V1):
        return None, missing or ["schema"]
    # Field requirements follow the ledger's own version; a v2 file that
    # predates the monotonic boundary fields is not failed for lacking them.
    publish_fields = contract.PUBLISH_FIELDS_V3 if schema == contract.SCHEMA_V3 else contract.PUBLISH_FIELDS
    integrate_fields = (contract.INTEGRATE_FIELDS_V3 if schema == contract.SCHEMA_V3
                        else contract.INTEGRATE_FIELDS)
    publish = segments["publish_to_callback"]
    integrate = segments["integrate_to_receipt"]
    queries = segments["query_chain"]
    extra = _missing_fields(publish, publish_fields)
    extra += _missing_fields(integrate, integrate_fields)
    extra += _missing_fields(queries, ("query_id",))
    if extra or not isinstance(publish, list) or not isinstance(integrate, list) or not isinstance(queries, list):
        return None, extra or ["segment_type"]
    if [row["input_id"] for row in publish] != [row["input_id"] for row in integrate]:
        return None, ["segment_mismatch"]
    if len(publish) != len({row["input_id"] for row in publish}):
        return None, ["segment_mismatch"]
    inputs = []
    for left, right in zip(publish, integrate):
        row = dict(left)
        row.update(right)
        inputs.append(row)
    resource = segments["resource_and_recovery"]
    if not isinstance(resource, dict):
        return None, ["resource_and_recovery"]
    case = {"gate": segments["gate"], "inputs": inputs, "queries": queries}
    for key, value in resource.items():
        if value is not None:
            case[key] = value
    return case, []


def dump_bytes(case):
    return json.dumps(segment_view(case), sort_keys=True, separators=(",", ":")).encode()


def replay_bytes(payload):
    try:
        segments = json.loads(payload.decode())
    except (UnicodeError, json.JSONDecodeError):
        return {"pass": False, "status": "fail", "reasons": ["replay_incomplete"],
                "replay_possible": False, "capture_certified": False}
    case, missing = case_from_segments(segments)
    if missing:
        return {"pass": False, "status": "fail", "reasons": ["replay_incomplete"],
                "replay_possible": False, "missing": missing, "capture_certified": False}
    result = judge.judge_case(case)
    direct = judge.judge_case(case)
    result["replay_possible"] = result["status"] == direct["status"] and result["reasons"] == direct["reasons"]
    result["capture_certified"] = False
    return result


def verify_replay(case):
    replayed = replay_bytes(dump_bytes(case))
    direct = judge.judge_case(case)
    return {
        "replay_possible": replayed.get("replay_possible") is True and replayed["reasons"] == direct["reasons"]
        and replayed["status"] == direct["status"],
        "status": replayed["status"],
        "reasons": replayed["reasons"],
    }
