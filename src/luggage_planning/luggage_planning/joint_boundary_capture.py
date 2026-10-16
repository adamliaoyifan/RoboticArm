"""Lossless bounded MoveGroup boundary capture for offline scene comparison."""

import hashlib
import json
from pathlib import Path


def capture_boundary(directory, index, scene, goal, result):
    """Persist pre-call scene and raw action messages before trajectory retiming."""
    return _capture(directory, index, "carrying_joint", {
        "scene": scene, "goal": goal, "result": result})


def capture_insert_boundary(directory, index, scene, request, response):
    """At most four exact INSERT service calls, including failed fractions."""
    return _capture(directory, index, "carrying_insert", {
        "scene": scene, "request": request, "response": response})


def _capture(directory, index, prefix, records):
    from rclpy.serialization import serialize_message

    if not directory or index >= 4:
        return False
    folder = Path(directory) / ("%s_%02d" % (prefix, index))
    folder.mkdir(parents=True, exist_ok=False)
    manifest = {"when": "scene queried before action; result before retiming",
                "scope": "planning-scene query snapshot, not server-internal acknowledgement",
                "artifacts": {}, "capture_complete": all(v is not None for v in records.values())}
    for name, message in records.items():
        if message is None:
            continue
        data = serialize_message(message)
        (folder / (name + ".cdr")).write_bytes(data)
        manifest["artifacts"][name] = {"sha256": hashlib.sha256(data).hexdigest(),
                                       "bytes": len(data)}
    (folder / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return True
