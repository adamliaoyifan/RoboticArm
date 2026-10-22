"""Replayable eval-side selection boundary; never a planner input."""
import json
from pathlib import Path


def capture_selection(root, boundary, response=None):
    if not root:
        return None
    directory = Path(root) / str(boundary["request_id"])
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "request.json").write_text(json.dumps(boundary, indent=2)+"\n")
    if response is not None:
        from rclpy.serialization import serialize_message
        (directory / "response.cdr").write_bytes(serialize_message(response))
    return str(directory)
