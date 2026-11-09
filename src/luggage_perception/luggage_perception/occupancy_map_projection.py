"""Stateless snapshots-to-dict projections.

These functions receive the frozen snapshot and any extra publication
fields explicitly. They do not read an OccupancyMapStore.
"""

from copy import deepcopy


def _thaw_json(value):
    """Fresh mutable JSON-shaped copy of a freeze-tagged tree."""
    if isinstance(value, tuple) and len(value) == 2 and value[0] in ("\x00dict", "\x00list"):
        tag, items = value
        if tag == "\x00dict":
            return {key: _thaw_json(item) for key, item in items}
        return [_thaw_json(item) for item in items]
    return value


def v1_surface_dict(snapshot, geometry_descriptor):
    """v1 JSON surface. Nested lists are new objects."""
    identity = snapshot.identity
    return {
        "frame": "container_local",
        "map_revision": identity.revision,
        "geometry_hash": snapshot.geometry_hash,
        "geometry_descriptor": deepcopy(geometry_descriptor),
        "resolution": snapshot.resolution,
        "nx": snapshot.nx,
        "ny": snapshot.ny,
        "inner_size": list(snapshot.inner_size),
        "floor_z": 0.0,
        "origin_local": list(snapshot.origin_local),
        "center_base": list(snapshot.center_base),
        "yaw": snapshot.yaw,
        "height": [list(row) for row in snapshot.height],
        "clearance": [list(row) for row in snapshot.clearance],
        "state": [list(row) for row in snapshot.state],
        "known_ratio": list(snapshot.known_ratio),
        "confidence": [list(row) for row in snapshot.confidence],
        "semantic_label": [list(row) for row in snapshot.semantic_label],
        "epoch": identity.epoch,
        "content_digest": identity.content_digest,
        "commit_revision": identity.revision,
        "acquisition_stamp_ns": snapshot.acquisition_stamp_ns,
        "map_source": snapshot.map_source,
        "placed_count": snapshot.placed_count,
        "cargo_observed": snapshot.cargo_observed,
        "occupancy_confirmed": snapshot.occupancy_confirmed,
        "change_event": snapshot.change_event,
        "change_token": snapshot.change_token,
        "last_settle": _thaw_json(snapshot.last_settle),
        "pending_changes": [
            {"token": token, "event": event, "stamp_ns": stamp_ns,
             "confirmable": confirmable, "box_id": box_id}
            for token, event, stamp_ns, confirmable, box_id
            in snapshot.pending_details],
    }


def v2_record(snapshot, scene_revision, request_id):
    """SurfaceContractV2 record. Nested lists are new objects."""
    identity = snapshot.identity
    record = {
        "contract_version": 2,
        "frame_id": "container_link",
        "length_unit": "m",
        "stamp_ns": 0,
        "time_basis": "scene",
        "scene_revision": str(scene_revision),
        "request_id": str(request_id),
        "kind": "surface",
        "map_revision": identity.revision,
        "geometry_hash": snapshot.geometry_hash,
        "epoch": identity.epoch,
        "content_digest": identity.content_digest,
        "occupancy_confirmed": snapshot.occupancy_confirmed,
        "change_event": snapshot.change_event,
        "height_datum": "container_origin",
        "origin_xy_container": list(snapshot.origin_local),
        "resolution_m": snapshot.resolution,
        "floor_z_container": snapshot.floor_z_container,
        "ceiling_z_container": snapshot.ceiling_z_container,
        "surface_z_container": [list(row) for row in snapshot.surface_z_container],
        "state": [list(row) for row in snapshot.state],
        "confidence": [list(row) for row in snapshot.contract_confidence],
    }
    if snapshot.height_above_floor:
        record["height_above_floor"] = [
            list(row) for row in snapshot.height_above_floor]
    return record


def committed_dict(snapshot, frame, container_in_world):
    """Committed-ledger JSON. Boxes are new dicts."""
    identity = snapshot.identity
    boxes = []
    for center, size, yaw, source, anchor in snapshot.ledger_details:
        box = {"center": list(center), "size": list(size), "yaw": yaw}
        if source:
            box["measured_source"] = source
        if anchor is not None:
            box["verify_anchor"] = {
                "request_id": anchor[0],
                "delta_m": anchor[1],
                "tolerance_m": anchor[2],
                "verify_source": anchor[3],
            }
        boxes.append(box)
    return {
        "frame": frame,
        "container_in_world": container_in_world,
        "boxes": boxes,
        "map_revision": identity.revision,
        "geometry_hash": snapshot.geometry_hash,
        "epoch": identity.epoch,
        "content_digest": identity.content_digest,
    }
