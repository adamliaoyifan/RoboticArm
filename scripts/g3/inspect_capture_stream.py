#!/usr/bin/env python3
"""Decode a bounded set of complete CDR records; no ROS graph or motion."""
import argparse
import importlib
import json
from rosidl_runtime_py.convert import message_to_ordereddict
from rclpy.serialization import deserialize_message
from luggage_planning.capture.async_boundary_capture import read_stream
from luggage_planning.capture.ros_boundary_capture import _json_fields


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('directory')
    parser.add_argument('--boundary')
    parser.add_argument('--limit', type=int, default=1)
    args = parser.parse_args()
    emitted = 0
    for metadata, payload in read_stream(args.directory):
        if args.boundary and metadata['boundary'] != args.boundary:
            continue
        module, name = metadata['type'].rsplit('.', 1)
        kind = getattr(importlib.import_module(module), name)
        message = deserialize_message(payload, kind)
        print(json.dumps(dict(metadata=metadata,
            fields=_json_fields(message_to_ordereddict(message))), ensure_ascii=False))
        emitted += 1
        if emitted >= args.limit:
            break
    if not emitted:
        raise SystemExit('No matching captured boundary')


if __name__ == '__main__':
    main()
