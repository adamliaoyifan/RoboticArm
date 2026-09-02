#!/usr/bin/env python3
"""Gate 0: load the site worksheet, probe the host, ping the controller.

Exit 0 only when ROS is Jazzy and the controller IP answers ping.
A filled worksheet is always written to config/site_check_report.yaml.
"""

from __future__ import annotations

import argparse
import os
import shutil
import socket
import subprocess
import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    yaml = None


ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT / "config" / "site_vars.yaml.example"
SITE = ROOT / "config" / "site_vars.yaml"
REPORT = ROOT / "config" / "site_check_report.yaml"
SDK = ROOT.parent / "third_party" / "huayan_python_sdk"


def _load_yaml(path: Path) -> dict:
    if yaml is None:
        raise SystemExit("PyYAML is required (python3-yaml)")
    with path.open(encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise SystemExit("%s is not a mapping" % path)
    return data


def _dump_yaml(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(data, handle, sort_keys=False, allow_unicode=True)


def _run(cmd, timeout=5, env=None):
    try:
        proc = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout, check=False,
            env=env,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return 1, "", str(exc)
    return proc.returncode, proc.stdout.strip(), proc.stderr.strip()


def _nics():
    code, out, _ = _run(["ip", "-br", "addr"])
    return out if code == 0 else ""


def _ping(ip: str) -> bool:
    if not ip:
        return False
    code, _, _ = _run(["ping", "-c1", "-W1", ip], timeout=8)
    return code == 0


def _usb_d435() -> bool:
    code, out, _ = _run(["lsusb"])
    if code != 0:
        return False
    text = out.lower()
    return "intel" in text and ("realsense" in text or "8086:0b07" in text
                                or "8086:0ad" in text or "depth" in text)


def _nvidia() -> str:
    code, out, err = _run(["nvidia-smi", "--query-gpu=name,driver_version",
                           "--format=csv,noheader"], timeout=8)
    if code != 0:
        return err or "no nvidia-smi"
    return out.splitlines()[0] if out else "none"


def _cps_import() -> str:
    env = os.environ.copy()
    path = str(SDK)
    env["PYTHONPATH"] = path + os.pathsep + env.get("PYTHONPATH", "")
    code, out, err = _run(
        [sys.executable, "-c",
         "from CPS import CPSClient; print(CPSClient.__name__)"],
        timeout=10,
        env=env,
    )
    if code == 0:
        return "ok " + out
    return "FAIL " + (err or out or "import failed")


def probe(site: dict) -> dict:
    robot_ip = str(site.get("robot", {}).get("ip") or "")
    ros = os.environ.get("ROS_DISTRO", "")
    nics = _nics()
    ping_ok = _ping(robot_ip)
    return {
        "ros_distro": ros or "unset",
        "python": sys.version.split()[0],
        "nics": nics,
        "robot_ip": robot_ip,
        "ping_robot": ping_ok,
        "d435_usb": _usb_d435(),
        "gpu": _nvidia(),
        "cps_import": _cps_import(),
        "sdk_path": str(SDK),
        "robot_nic_up": _nic_state(nics, "enp0s31f6") == "UP",
    }


def _nic_state(nics: str, name: str) -> str:
    for line in nics.splitlines():
        parts = line.split()
        if parts and parts[0] == name:
            return parts[1] if len(parts) > 1 else "UNKNOWN"
    return "MISSING"


def gate0_ok(probed: dict) -> tuple[bool, str]:
    if probed.get("ros_distro") != "jazzy":
        return False, "ROS_DISTRO is %r (need jazzy)" % probed.get("ros_distro")
    if not probed.get("ping_robot"):
        return False, "controller %s did not answer ping" % probed.get("robot_ip")
    return True, "jazzy + ping ok"


def export_env(site: dict) -> str:
    robot = site.get("robot") or {}
    lines = [
        "export ROS_DISTRO=jazzy",
        "export ROS_DOMAIN_ID=%s" % site.get("ros_domain_id", 7),
        "export ROBOT_IP=%s" % robot.get("ip", "192.168.0.10"),
        "export ROBOT_PORT=%s" % robot.get("port", 10003),
        "export PYTHONPATH=%s:${PYTHONPATH:-}" % SDK,
        "export FJT_ACTION=%s"
        % robot.get("fjt_action", "/elfin_arm_controller/follow_joint_trajectory"),
    ]
    return "\n".join(lines) + "\n"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Gate 0 site check")
    parser.add_argument("--export", action="store_true",
                        help="print shell exports only")
    parser.add_argument("--init", action="store_true",
                        help="copy example to site_vars.yaml if missing")
    args = parser.parse_args(argv)

    if not SITE.exists() or args.init:
        if not EXAMPLE.exists():
            raise SystemExit("missing %s" % EXAMPLE)
        if not SITE.exists():
            shutil.copy(EXAMPLE, SITE)
            print("created %s from example" % SITE, file=sys.stderr)

    site = _load_yaml(SITE if SITE.exists() else EXAMPLE)
    if args.export:
        sys.stdout.write(export_env(site))
        return 0

    probed = probe(site)
    ok, reason = gate0_ok(probed)
    report = {
        "schema": "robotarm_site_check/v1",
        "ok": ok,
        "reason": reason,
        "site_file": str(SITE),
        "site": site,
        "probed": probed,
    }
    _dump_yaml(REPORT, report)

    print("site file: %s" % SITE)
    print("report:    %s" % REPORT)
    print("ROS_DISTRO=%s  python=%s" % (probed["ros_distro"], probed["python"]))
    print("robot_ip=%s  ping=%s" % (probed["robot_ip"], probed["ping_robot"]))
    print("d435_usb=%s  gpu=%s" % (probed["d435_usb"], probed["gpu"]))
    print("cps_import=%s" % probed["cps_import"])
    print("nics:\n%s" % probed["nics"])
    print("gate0: %s (%s)" % ("PASS" if ok else "FAIL", reason))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
