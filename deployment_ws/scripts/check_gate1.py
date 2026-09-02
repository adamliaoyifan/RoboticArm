#!/usr/bin/env python3
"""Gate 1 software checks: CPS import, FJT action name, optional TCP connect.

Does not command motion. After this passes, run:
  ros2 launch elfin_trajectory_executor jazzy_real.launch.py robot_ip:=$ROBOT_IP
  ros2 run elfin_trajectory_executor send_joint_trajectory --delta-deg 2 --and-back
and wait for READY_FOR_NEXT.
"""

from __future__ import annotations

import os
import socket
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SDK = ROOT.parent / "third_party" / "huayan_python_sdk"
DEFAULT_ACTION = "/elfin_arm_controller/follow_joint_trajectory"


def _ensure_sdk_path() -> None:
    path = str(SDK)
    if path not in sys.path:
        sys.path.insert(0, path)
    os.environ["PYTHONPATH"] = path + os.pathsep + os.environ.get("PYTHONPATH", "")


def check_cps() -> str:
    _ensure_sdk_path()
    from CPS import CPSClient  # noqa: PLC0415
    client = CPSClient()
    return type(client).__name__


def check_action_name() -> str:
    sys.path.insert(
        0,
        str(ROOT / "src" / "elfin_trajectory_executor"),
    )
    from elfin_trajectory_executor.execution_contract import (  # noqa: PLC0415
        DEFAULT_ACTION_NAME,
    )
    if DEFAULT_ACTION_NAME != DEFAULT_ACTION:
        raise RuntimeError(
            "action name %s != %s" % (DEFAULT_ACTION_NAME, DEFAULT_ACTION)
        )
    return DEFAULT_ACTION_NAME


def check_tcp(ip: str, port: int, timeout=2.0) -> str:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    try:
        sock.connect((ip, port))
        return "open"
    finally:
        sock.close()


def main() -> int:
    ip = os.environ.get("ROBOT_IP", "192.168.0.10")
    port = int(os.environ.get("ROBOT_PORT", "10003"))
    failures = []

    try:
        name = check_cps()
        print("cps_import: ok (%s) sdk=%s" % (name, SDK))
    except Exception as exc:
        print("cps_import: FAIL %s" % exc)
        failures.append("cps")

    try:
        action = check_action_name()
        print("fjt_action: %s" % action)
    except Exception as exc:
        print("fjt_action: FAIL %s" % exc)
        failures.append("action")

    try:
        state = check_tcp(ip, port)
        print("tcp %s:%s: %s" % (ip, port, state))
    except OSError as exc:
        print("tcp %s:%s: FAIL %s" % (ip, port, exc))
        failures.append("tcp")

    ping = subprocess.run(
        ["ping", "-c1", "-W1", ip],
        capture_output=True,
        timeout=8,
        check=False,
    )
    if ping.returncode != 0:
        print("ping %s: FAIL (do not send FJT to hardware until ICMP works)" % ip)
    else:
        print("ping %s: ok" % ip)

    if failures:
        print("gate1 software: FAIL (%s)" % ",".join(failures))
        print("motion test skipped")
        return 1
    print("gate1 software: PASS")
    print("next: jazzy_real.launch.py then send_joint_trajectory --delta-deg 2 --and-back")
    return 0


if __name__ == "__main__":
    sys.exit(main())
