#!/usr/bin/env python3
"""One persistent, wall-clock probe before a G4 functional case starts."""
import json
from pathlib import Path
import sys
import time

import rclpy
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data


def main():
    output = Path(sys.argv[1])
    rclpy.init()
    node = rclpy.create_node('attachment_geometry_preflight')
    images = []
    subscription = node.create_subscription(Image, '/camera/color/image_raw',
        lambda message: images.append(time.monotonic()), qos_profile_sensor_data)
    client = node.create_client(GetParameters, '/robot_state_publisher/get_parameters')
    future = None
    requested_at = None
    request_attempts = []
    started = time.monotonic()
    result = dict(success=False, stage='startup', clock_publishers=None,
                  camera_samples=0, urdf_saved=False)
    try:
        while time.monotonic() - started < 90:
            rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()
            if future is not None and not future.done() and now-requested_at >= 5:
                # A lost DDS response must not consume the entire startup window.
                client.remove_pending_request(future)
                future.cancel()
                request_attempts[-1]['outcome'] = 'response_timeout'
                future = None
            if future is None and not result['urdf_saved'] and client.service_is_ready():
                future = client.call_async(GetParameters.Request(names=['robot_description']))
                requested_at = now
                request_attempts.append(dict(request_wall_s=now-started, outcome='pending'))
            if future is not None and future.done() and not result['urdf_saved']:
                response = future.result()
                xml = response.values[0].string_value if response and response.values else ''
                if '<robot' in xml:
                    (output/'robot_expanded.urdf').write_text(xml)
                    result['urdf_saved'] = True
                    request_attempts[-1]['outcome'] = 'saved'
                else:
                    request_attempts[-1]['outcome'] = 'invalid_description'
                    future = None
            result['clock_publishers'] = node.count_publishers('/clock')
            if (len(images) >= 30 and images[-1]-images[0] >= 5
                    and result['urdf_saved'] and result['clock_publishers'] == 1):
                result['success'] = True
                break
        result['wall_s'] = time.monotonic()-started
        result['urdf_requests'] = request_attempts
        result['camera_samples'] = len(images)
        result['camera_hz'] = ((len(images)-1)/(images[-1]-images[0])
                               if len(images)>1 else None)
        result['success'] = bool(result['success'] and result['camera_hz'] >= 3)
        result['criteria'] = 'one clock publisher; saved URDF; >=30 camera samples over >=5s; camera >=3Hz'
        (output/'startup_preflight.json').write_text(json.dumps(result,indent=2)+'\n')
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if result['success'] else 7


if __name__ == '__main__':
    raise SystemExit(main())
