"""Plain record of one SelectPlacePath request, for offline replay.

The live driver writes this next to ``candidates.json``. A bundle is
replayable only when every field below is present. A historical dump that
lacks them is ``not_replayable``; that is a missing artifact, not a failed
replay command.
"""

from __future__ import division

from luggage_planning.planning_capture import candidate_complete


REQUIRED_FIELDS = (
    "request_id",
    "start_joints",
    "attachment_transform",
    "container_transform",
    "map_revision",
    "map_source",
    "geometry_hash",
    "candidate_targets",
    "candidates",
    "map_snapshot",
    "atlas_identity",
)


def _vec(value, length):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        return None
    try:
        return [float(v) for v in value]
    except (TypeError, ValueError):
        return None


def _transform(value):
    if not isinstance(value, dict):
        return None
    translation = _vec(value.get("translation"), 3)
    quaternion = _vec(value.get("quaternion_xyzw"), 4)
    if translation is None or quaternion is None:
        return None
    return {"translation": translation, "quaternion_xyzw": quaternion}


def input_slots_from_messages(slots):
    """Every placement candidate the select was given, tried or not.

    Attempt rows stay a separate record. A slot the search never reached
    is still part of the request.
    """
    records = []
    for index, slot in enumerate(slots or []):
        pose = getattr(slot, "place_pose", None)
        if pose is None and isinstance(slot, dict):
            pose = slot.get("place_pose") or slot.get("slot_pose")
        if pose is None:
            continue
        if isinstance(pose, dict):
            position = pose.get("position")
            quaternion = pose.get("quaternion_xyzw") or pose.get(
                "orientation")
        else:
            position = getattr(pose, "position", None)
            quaternion = getattr(pose, "orientation", None)
        if position is None or quaternion is None:
            continue
        if hasattr(position, "x"):
            position = [position.x, position.y, position.z]
        if hasattr(quaternion, "x"):
            quaternion = [
                quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        if len(position) != 3 or len(quaternion) != 4:
            continue
        records.append({
            "index": index,
            "position": [float(v) for v in position],
            "quaternion_xyzw": [float(v) for v in quaternion],
        })
    return records


def candidate_targets_from_rows(rows):
    """World TCP targets already stored on selector rows."""
    targets = []
    for row in rows or []:
        if not isinstance(row, dict):
            continue
        raw = row.get("targets")
        if isinstance(raw, str):
            continue
        if isinstance(raw, list):
            for item in raw:
                if (isinstance(item, list) and len(item) == 2
                        and _vec(item[0], 3) and _vec(item[1], 4)):
                    targets.append([_vec(item[0], 3), _vec(item[1], 4)])
        refine = row.get("ik_refine")
        if isinstance(refine, list):
            for item in refine:
                world = (item or {}).get("target_world") if isinstance(
                    item, dict) else None
                if (isinstance(world, list) and len(world) == 2
                        and _vec(world[0], 3) and _vec(world[1], 4)):
                    targets.append([_vec(world[0], 3), _vec(world[1], 4)])
    return targets


def build_planning_request(fields):
    """Copy the request fields the offline replay command consumes.

    ``fields`` is a plain dict. Missing pieces stay absent so the audit
    can name them; this function does not invent defaults.
    """
    fields = fields or {}
    document = {
        "request_id": str(fields.get("request_id") or ""),
        "box_wdh": _vec(fields.get("box_wdh"), 3),
        "start_joints": _vec(fields.get("start_joints"), 6),
        "attachment_transform": _transform(fields.get("attachment_transform")),
        "container_transform": _transform(fields.get("container_transform")),
        "map_revision": fields.get("map_revision"),
        "map_source": str(fields.get("map_source") or ""),
        "geometry_hash": str(fields.get("geometry_hash") or ""),
        "candidate_targets": list(fields.get("candidate_targets") or []),
        "candidates": list(fields.get("candidates") or []),
        "input_slots": list(fields.get("input_slots") or []),
        "atlas_identity": dict(fields.get("atlas_identity") or {}),
        "stage_timings": dict(fields.get("stage_timings") or {}),
    }
    snapshot = fields.get("map_snapshot")
    if isinstance(snapshot, dict):
        document["map_snapshot"] = snapshot
    return document


def _candidates_replayable(value):
    if not isinstance(value, list) or not value:
        return False
    if not any(candidate_complete(item) for item in value):
        return False
    for item in value:
        if candidate_complete(item):
            continue
        pose = (item.get("slot_pose") or {}) if isinstance(item, dict) else {}
        if (isinstance(item, dict)
                and item.get("reachability") == "not_synthesized"
                and item.get("targets") == []
                and isinstance(item.get("seed_sources"), list)
                and isinstance(pose.get("position"), list)
                and len(pose.get("position")) == 3):
            continue
        return False
    return True


def _present(document, name):
    value = document.get(name)
    if name == "start_joints":
        return _vec(value, 6) is not None
    if name in ("attachment_transform", "container_transform"):
        return _transform(value) is not None
    if name == "map_revision":
        return isinstance(value, int) and not isinstance(value, bool)
    if name in ("request_id", "map_source", "geometry_hash"):
        return isinstance(value, str) and bool(value)
    if name == "candidate_targets":
        return isinstance(value, list) and len(value) > 0
    if name == "candidates":
        return _candidates_replayable(value)
    if name == "map_snapshot":
        return isinstance(value, dict) and bool(value)
    if name == "atlas_identity":
        return isinstance(value, dict) and bool(value.get("identity"))
    return value not in (None, "", [], {})


def audit_replay_bundle(document):
    """Return replayability. Do not claim a replay that the fields cannot support."""
    document = document if isinstance(document, dict) else {}
    missing = [name for name in REQUIRED_FIELDS if not _present(document, name)]
    if missing:
        return {
            "status": "not_replayable",
            "replay_possible": False,
            "missing": missing,
        }
    return {
        "status": "replayable",
        "replay_possible": True,
        "missing": [],
    }
