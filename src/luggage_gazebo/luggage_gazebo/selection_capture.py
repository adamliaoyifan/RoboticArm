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


def preserve_failed_selection(directory, replay_directory):
    """A refused Select has no trajectory: preserve its actual service boundary."""
    if not directory:
        return None
    from rclpy.serialization import deserialize_message
    from luggage_msgs.srv import SelectPlacePathV2
    import shutil
    source = Path(directory)
    response_path = source/'response.cdr'
    if not response_path.is_file():
        return None
    response = deserialize_message(response_path.read_bytes(), SelectPlacePathV2.Response)
    if response.success:
        return None
    target = Path(replay_directory)/'selection'
    target.mkdir(parents=True, exist_ok=True)
    artifacts, missing = {}, []
    for name in ('request.json', 'response.cdr', 'planning_scene.cdr'):
        path = source/name
        if not path.is_file() or path.stat().st_size == 0:
            missing.append('selection/'+name)
        else:
            shutil.copyfile(path, target/name)
            artifacts['selection_'+name] = 'replay/selection/'+name
    return artifacts, missing
