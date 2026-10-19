#!/usr/bin/env python3
"""Bridge /obs/box_contact (gz.msgs.Contacts) to ROS for the A/B runs.

The in-process recorder (record_box_stream.py) subscribes to the ROS
side; the unfiltered gz echo that previously captured contacts produced
4.8 MB/s of pipe traffic whose disk contention reordered the pose echo
(run-01 invalidation)."""
import os
import subprocess
import time

ros_bin = "/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge"
if not os.path.isfile(ros_bin):
    raise SystemExit("parameter_bridge not found at %s" % ros_bin)
proc = subprocess.Popen([
    ros_bin,
    "/obs/box_contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
    "--ros-args", "-r", "__node:=ab_contact_bridge"],
    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
try:
    while True:
        time.sleep(3600)
except KeyboardInterrupt:
    pass
finally:
    proc.terminate()
