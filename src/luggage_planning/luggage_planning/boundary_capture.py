"""Request-boundary capture for the typed place-route chain.

Contract
--------
``capture_service_request`` writes one lossless serialized ROS request and a
small JSON index entry. It is opt-in: the caller passes the directory from
``ST09_SELECT_REQUEST_CAPTURE_DIR``; an empty directory disables it. It never
raises into the service callback and never changes the request.

Replay: ``load_service_request(path, message_type)`` returns the exact message.
The sha256 in the index entry is of the serialized bytes.
"""

import hashlib
import json
import os
import time


def capture_service_request(request, directory, label):
    """Persist ``request``; return the index entry or None when disabled."""
    if not directory:
        return None
    try:
        from rclpy.serialization import serialize_message

        os.makedirs(directory, exist_ok=True)
        payload = bytes(serialize_message(request))
        index = len([name for name in os.listdir(directory)
                     if name.startswith(label + "_")
                     and name.endswith(".bin")])
        stem = "%s_%02d" % (label, index)
        bin_path = os.path.join(directory, stem + ".bin")
        with open(bin_path, "wb") as handle:
            handle.write(payload)
        entry = {
            "label": label,
            "bin": os.path.basename(bin_path),
            "sha256": hashlib.sha256(payload).hexdigest(),
            "size_bytes": len(payload),
            "message_type": "%s.%s" % (
                type(request).__module__, type(request).__name__),
            "request_id": str(getattr(request, "request_id", "")),
            "wall_time": time.time(),
        }
        with open(os.path.join(directory, stem + ".json"), "w",
                  encoding="utf-8") as handle:
            json.dump(entry, handle, indent=1)
            handle.write("\n")
        return entry
    except Exception as error:  # noqa: BLE001 - diagnostics must not break the service
        try:
            os.makedirs(directory, exist_ok=True)
            with open(os.path.join(directory, label + "_capture_error.txt"),
                      "a", encoding="utf-8") as handle:
                handle.write("%r\n" % (error,))
        except Exception:  # noqa: BLE001
            pass
        return None


def load_service_request(path, message_type):
    """Deserialize a captured request exactly as it was received."""
    from rclpy.serialization import deserialize_message

    with open(path, "rb") as handle:
        return deserialize_message(handle.read(), message_type)
