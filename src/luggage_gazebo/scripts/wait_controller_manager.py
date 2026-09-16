#!/usr/bin/env python3
"""Bounded watchdog for the gz_ros2_control controller_manager.

``gz_ros2_control`` 0.7.21 fetches the model URDF with a single
``AsyncParametersClient::get_parameters`` call whose ``wait_for`` result is
discarded before ``get()`` is called. When that ~22 KB reply is lost the
plugin blocks forever: it logs ``connected to service!!`` and then neither
``Received URDF from param server`` nor its own retry message, and the
controller manager is never constructed. Measured on this workspace at roughly
one launch in ten (see
``docs/status/evidence/platform_free_height/2026-09-16_startup_rehearsal/``).

The plugin cannot be made to retry from here, so this watchdog makes the hang
*fast and named* instead of silent. Without it the two controller spawners sit
on their 60 s ``--controller-manager-timeout`` and the whole launch looks alive
for minutes while nothing will ever happen.

Exits 0 once ``/controller_manager/list_controllers`` is advertised, 1 on
timeout. One persistent node; no ``ros2`` CLI subprocess, because per-poll node
construction adds discovery churn to the very window the plugin is racing in.
"""
from __future__ import annotations

import argparse
import sys
import time

from luggage_gazebo.startup_probe import wait_for_service

SERVICE_SUFFIX = "/list_controllers"

READY_MARKER = "wait_controller_manager: ok"
FAIL_MARKER = "startup_failed: plugin_urdf_not_received"


def service_available(node, service_name):
    for name, _types in node.get_service_names_and_types():
        if name == service_name:
            return True
    return False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller-manager", default="/controller_manager")
    parser.add_argument("--timeout-sec", type=float, default=45.0)
    args = parser.parse_args(argv)

    service_name = args.controller_manager.rstrip("/") + SERVICE_SUFFIX

    import rclpy

    rclpy.init(args=None)
    node = rclpy.create_node("wait_controller_manager")
    t0 = time.monotonic()
    try:
        ok = wait_for_service(
            lambda: service_available(node, service_name), args.timeout_sec)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    elapsed = time.monotonic() - t0
    if ok:
        print("%s %s after %.2fs" % (READY_MARKER, service_name, elapsed))
        return 0
    print("%s: %s absent after %.2fs; the gz_ros2_control plugin never built "
          "the controller manager" % (FAIL_MARKER, service_name, elapsed),
          file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
