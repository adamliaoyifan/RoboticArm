"""Opt-in lossless scene capture before the eval driver calls Select."""
import hashlib
import json
from pathlib import Path
import time

MAX_CASE_BYTES = 100 * 1024**2


def capture_select_scene(directory, request_id, request_data, read_scene):
    """Capture the first request's scene; publish its binding last.

    The snapshot is from the client just before the call, not a claim that
    the server processed an identical scene. The typed recorder separately
    persists the exact Select request and response.
    """
    from rclpy.serialization import serialize_message

    folder = Path(directory)
    folder.mkdir(parents=True, exist_ok=True)
    binding = folder / "identity.json"
    if binding.exists():
        return
    if not isinstance(request_data, bytes) or not request_data:
        raise RuntimeError("SELECT_REPLAY_REQUEST_CDR_UNAVAILABLE")
    begin = time.monotonic_ns()
    scene = read_scene()
    if scene is None:
        raise RuntimeError("SELECT_REPLAY_SCENE_UNAVAILABLE")
    scene_data = serialize_message(scene)
    if len(request_data) + len(scene_data) > MAX_CASE_BYTES:
        raise RuntimeError("SELECT_REPLAY_CAPTURE_BUDGET_EXHAUSTED")
    with (folder / "planning_scene.cdr").open("xb") as stream:
        stream.write(scene_data)
    with binding.open("x") as stream:
        json.dump({
            "request_id": request_id,
            "select_request_sha256": hashlib.sha256(request_data).hexdigest(),
            "scene_sha256": hashlib.sha256(scene_data).hexdigest(),
            "scene_read_begin_ns": begin, "scene_read_end_ns": time.monotonic_ns(),
            "scope": "client pre-call scene",
        }, stream, indent=2)
        stream.write("\n")
