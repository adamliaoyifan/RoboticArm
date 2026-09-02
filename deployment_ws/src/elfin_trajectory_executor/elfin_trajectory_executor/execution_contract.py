"""Shared names and the execution-complete handshake.

The sequencing signal is the FollowJointTrajectory **action result**.
The events topic is for observers that are not holding the goal handle.
Do not start the next motion because `/trajectory_executor/status` became
`idle`: that also happens at startup and after cancel.
"""

from __future__ import annotations

import json
from typing import Any, Mapping, Optional

# Matches MoveIt simple controller manager + luggage motion_planner FJT client.
DEFAULT_ACTION_NAME = "/elfin_arm_controller/follow_joint_trajectory"

STATUS_TOPIC = "/trajectory_executor/status"
EVENTS_TOPIC = "/trajectory_executor/events"

SCHEMA = "elfin_executor_event/v1"

EVENT_IDLE = "idle"
EVENT_ACCEPTED = "accepted"
EVENT_EXECUTING = "executing"
EVENT_SUCCEEDED = "succeeded"
EVENT_ABORTED = "aborted"
EVENT_CANCELED = "canceled"
EVENT_REJECTED = "rejected"

TERMINAL_EVENTS = frozenset(
    {EVENT_SUCCEEDED, EVENT_ABORTED, EVENT_CANCELED, EVENT_REJECTED}
)


def make_event(
    event: str,
    *,
    goal_id: str = "",
    error_code: int = 0,
    error_string: str = "",
    stamp_ns: int = 0,
) -> dict[str, Any]:
    return {
        "schema": SCHEMA,
        "event": str(event),
        "goal_id": str(goal_id or ""),
        "error_code": int(error_code),
        "error_string": str(error_string or ""),
        "stamp_ns": int(stamp_ns),
        "ready_for_next": str(event) == EVENT_SUCCEEDED,
    }


def event_to_json(payload: Mapping[str, Any]) -> str:
    return json.dumps(dict(payload), sort_keys=True, separators=(",", ":"))


def parse_event(data: str) -> Optional[dict[str, Any]]:
    try:
        payload = json.loads(data)
    except (TypeError, ValueError, json.JSONDecodeError):
        return None
    if not isinstance(payload, dict):
        return None
    if payload.get("schema") != SCHEMA:
        return None
    if "event" not in payload:
        return None
    payload.setdefault("ready_for_next", payload.get("event") == EVENT_SUCCEEDED)
    return payload
