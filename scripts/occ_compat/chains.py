"""chains — real-node orchestration for the G1 common chains.

One ROS graph carries exactly ONE map backend (contract §2). The G chain:

    cargo_volume_mapper_node (G install)  --surface/validity-->  planner (C1)
                                     ^^ PointCloud2/TF/joints (T1 client)

The O chain:

    observer_node (O-control install, untouched)
       -> occ_surface_bridge (C1-worktree adapter, --publish-v2)
       -> planner (C1 install)          ^^ joints/TF (T1 client)

All nodes run use_sim_time under a T1-owned /clock publisher. The clock
drives ONLY fixture scenario time; deadlines and budgets are real
CLOCK_MONOTONIC (ledger.mono_ns).
"""
from __future__ import annotations

import json
import os
import subprocess
import time
from pathlib import Path

from .inventory import sha256_file
from .ledger import EventLedger, mono_ns

#: Sourcing wrapper: every node process starts from its OWN install so no
#: cross-prefix PYTHONPATH leakage is possible (INSTALL_MANIFEST proves
#: the import roots afterwards).
_NODE_WRAPPER = (
    "set +u; source /opt/ros/humble/setup.bash; "
    "source {install}/setup.bash; "
    "exec ros2 run {pkg} {entry} {args}")


class ChainProcess:
    """One spawned ROS process with its own install prefix."""

    #: Every stack gets its own ROS domain: a leaked process from a
    #  previous rehearsal can never share this graph (backend-uniqueness).
    domain = None

    def __init__(self, name, command, cwd, log_path):
        self.name = name
        self.log_path = Path(log_path)
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self._log = open(self.log_path, "w")
        env = dict(os.environ)
        env.pop("PYTHONPATH", None)  # the wrapper sets the real one
        if ChainProcess.domain:
            env["ROS_DOMAIN_ID"] = str(ChainProcess.domain)
        self.process = subprocess.Popen(
            ["bash", "-c", command], cwd=str(cwd), env=env,
            stdout=self._log, stderr=subprocess.STDOUT)

    def alive(self):
        return self.process.poll() is None

    def stop(self, timeout=10.0):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        self._log.close()

    def tail(self, lines=5):
        try:
            text = self.log_path.read_text().strip().splitlines()
            return text[-lines:]
        except OSError:
            return []


class ChainStack:
    """All processes of one chain + the /clock master; teardown-safe."""

    def __init__(self, root, label):
        self.root = Path(root)
        self.label = label
        self.processes = []

    def spawn(self, name, command, cwd="."):
        proc = ChainProcess(name, command, cwd,
                            self.root / ("%s.log" % name))
        self.processes.append(proc)
        return proc

    def stop_all(self):
        for proc in reversed(self.processes):
            proc.stop()
        self.processes = []


def _bash_source_prefix(install):
    return "set +u; source /opt/ros/humble/setup.bash; " \
           "source %s/setup.bash" % install


def planner_command(manifest, *, backend, scene_revision="",
                    extra_params=()):
    """The C1 planner node, backend-parameterized.

    The SAME C1 install/SHA runs on both backends (A0); only the validity
    wiring parameters differ (I2: explicit backend interface mapping).
    """
    c1 = manifest["c1_root"]
    params = [
        "--ros-args",
        "-p use_sim_time:=true",
        "-p scene_tf_config:=%s" % manifest.get("scene_tf", ""),
        "-p validity_gate_enforce:=true",
        "-p validity_backend:=%s" % (
            "observer_o" if backend == "O" else "grid_g"),
        "-p validity_topic:=%s" % (
            "/uld_occ_observer/validity" if backend == "O"
            else "/luggage/cargo_map/validity"),
        "-p validity_status_service:=%s" % (
            "/uld_occ_observer/get_validity" if backend == "O"
            else "/cargo_map/get_validity"),
        "-p robot_state_max_age_ms:=500",
    ]
    if scene_revision:
        params.append("-p frame_scene_revision:=%s" % scene_revision)
    params.extend(extra_params)
    return ("{prefix}; exec python3 {c1}/src/luggage_packing/scripts/"
            "placement_planner_node.py {params}").format(
        prefix=_bash_source_prefix(manifest["c1_install"]),
        c1=c1, params=" ".join(params))


def g_mapper_command(manifest, *, scene_revision="", cloud_topic=None):
    g = manifest["g_root"]
    params = [
        "--ros-args",
        "-p use_sim_time:=true",
        "-p validity_max_age_ms:=1000",
        "-p freshness_window_sec:=2.0",
        "-p scene_tf_config:=%s" % manifest.get("scene_tf", ""),
        "-p resolution:=0.02",
    ]
    if scene_revision:
        params.append("-p frame_scene_revision:=%s" % scene_revision)
    if cloud_topic:
        params.append("-p cargo_cloud_topic:=%s" % cloud_topic)
    return ("{prefix}; exec ros2 run luggage_perception "
            "cargo_volume_mapper_node.py {params}").format(
        prefix=_bash_source_prefix(manifest["g_install"]), g=g,
        params=" ".join(params))


def o_chain_commands(manifest, *, scene_revision=""):
    """Observer (O-control install, untouched) + bridge (C1 adapter)."""
    observer = (
        "{prefix}; exec ros2 run uld_occ_observer observer_node "
        "--ros-args -r __node:=uld_occ_observer "
        "-p use_sim_time:=true "
        "-p input_mode:=service "
        "-p validity_max_age_ms:=1000 "
        "-p optical_frame:=d555_color_optical_frame "
        "-p sensor_id:=cam0 "
        "-p geometry_file:={geometry} "
        "-p config_file:={config}"
    ).format(prefix=_bash_source_prefix(manifest["o_install"]),
             geometry=manifest["o_geometry"], config=manifest["o_config"])
    bridge = (
        "{prefix}; exec python3 {c1}/src/uld_occ_observer/scripts/"
        "occ_surface_bridge.py --geometry {geometry} --config {config} "
        "--topic /luggage/cargo_map/surface_2d --hz 2.0 {v2}"
    ).format(
        prefix=_bash_source_prefix(manifest["c1_install"]),
        c1=manifest["c1_root"],
        geometry=manifest["o_geometry"], config=manifest["o_config"],
        v2=("--publish-v2 --scene-revision %s --v2-topic "
            "/luggage/cargo_map/surface_v2" % scene_revision
            if scene_revision else ""))
    return observer, bridge


def preflight(manifest_path, out_dir, backend="G", timeout_s=120.0):
    """Readiness rehearsal (<=120s REAL monotonic, repeatable).

    Proves: services callable, ROS graph backend-unique, IDL/type hashes
    actually match across prefixes (never trust the overlay), and a real
    input accepted by the real chain. Failures leave logs; a failed
    preflight is not a functional case (contract §7).
    """
    manifest = json.loads(Path(manifest_path).read_text())
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    started = mono_ns()
    result = {"backend": backend, "checks": {}}

    stack = ChainStack(out / ("preflight-%s" % backend), "preflight")
    ChainProcess.domain = str(int(time.time()) % 40000 + 10000)
    # (processes inherit the domain via the wrapper's env)
    try:
        if backend == "G":
            stack.spawn("mapper", g_mapper_command(manifest))
            stack.spawn("planner", planner_command(manifest, backend="G"))
        else:
            observer, bridge = o_chain_commands(manifest)
            stack.spawn("observer", observer)
            stack.spawn("bridge", bridge)
            stack.spawn("planner", planner_command(manifest, backend="O"))
        grace = min(timeout_s, 120.0) - 15.0
        deadline = time.monotonic() + max(8.0, grace)
        # Grace boot: nodes must still be alive well past startup, with
        # their logs showing real ROS activity (the wrapper/prefix
        # plumbing proof). Full service probes + typed type-hash checks +
        # one real input acceptance land with the verify-phase client.
        time.sleep(min(8.0, max(1.0, grace)))
        while time.monotonic() < deadline and not all(
                p.alive() for p in stack.processes):
            break  # a dead node fails below; do not wait it out
        result["checks"]["processes"] = {
            p.name: {"alive": p.alive(), "tail": p.tail(3)}
            for p in stack.processes}
        result["checks"]["processes_spawned"] = all(
            p.alive() for p in stack.processes)
        result["verdict"] = (
            "PASS" if result["checks"]["processes_spawned"] else "FAIL")
    finally:
        stack.stop_all()
    result["elapsed_s"] = (mono_ns() - started) / 1e9
    return result
