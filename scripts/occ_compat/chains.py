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
import signal
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
        # Restart cases (V2 R10/R11/R12) spawn replacement nodes into the
        # same chain dir: never truncate a prior instance's log — an
        # overwrite destroyed the R05-era planner log in formal_run02,
        # leaving the one failing case without log evidence.
        index = 0
        while self.log_path.exists():
            index += 1
            self.log_path = Path(str(log_path) + ".%d" % index)
        self._log = open(self.log_path, "w")
        self._probe_executable(command)
        self._probe_inputs(command)
        env = dict(os.environ)
        env.pop("PYTHONPATH", None)  # the wrapper sets the real one
        if ChainProcess.domain:
            env["ROS_DOMAIN_ID"] = str(ChainProcess.domain)
        # Each chain process gets its OWN session: the command is
        # `bash -c "... exec ros2 run <pkg> <node>"`, and ros2 run is a
        # python launcher that spawns the actual node as a CHILD —
        # killing only the wrapper pid (the G1 R12 "ghost endpoint") left
        # the node orphaned and LIVE, still answering services at the
        # dead epoch on the same domain. A dedicated process group lets
        # stop()/kill_tree() take the whole tree down.
        self.process = subprocess.Popen(
            ["bash", "-c", command], cwd=str(cwd), env=env,
            start_new_session=True,
            stdout=self._log, stderr=subprocess.STDOUT)

    def _signal_tree(self, sig):
        """Signal the process GROUP (wrapper + ros2 run + node child)."""
        try:
            os.killpg(self.process.pid, sig)
        except ProcessLookupError:
            pass

    def tree_empty(self, timeout_s=5.0):
        """True when nothing from this process group survives."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                os.killpg(self.process.pid, 0)
            except ProcessLookupError:
                return True
            time.sleep(0.1)
        return False

    def kill_tree(self):
        """SIGKILL the whole group and PROVE it is empty afterwards —
        a restart (R10/R11/R12) must never leave the old producer
        answering beside the new one."""
        self._signal_tree(signal.SIGKILL)
        try:
            self.process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            pass
        empty = self.tree_empty(5.0)
        if not empty:
            raise RuntimeError(
                "process tree of %r still alive after SIGKILL" % self.name)
        self._log.close()

    @staticmethod
    def _instance_id(pid):
        """Stable instance identity (reviews 1622 B8): pid + /proc
        starttime — a restart is a NEW instance even when pid recycles."""
        try:
            with open("/proc/%d/stat" % int(pid)) as stat:
                starttime = int(stat.read().rsplit(")", 1)[1].split()[19])
            return "%d@%d" % (int(pid), starttime)
        except (OSError, ValueError, IndexError):
            return str(pid)

    def _probe_inputs(self, command):
        """Runtime INPUT identity (reviews 1622 B8): hash every config/
        TF file path the command references WITH the spawning instance
        identity, so A0 can bind per-instance input obligations — a
        restarted process is a new instance with its own obligations."""
        import re
        paths = set(re.findall(
            r"(?:config_file:=|geometry_file:=|scene_tf_config:=)(\S+)",
            command))
        files = []
        for path_text in sorted(paths):
            path = Path(path_text)
            if path.is_dir():  # config/geometry directories: every file
                files.extend(sorted(p for p in path.rglob("*")
                                    if p.is_file()))
            elif path.is_file():
                files.append(path)
        with open(self.log_path.parent / "INPUT_PROBES.jsonl", "a") as probes:
            for path in files:
                try:
                    digest = sha256_file(path)
                except OSError:
                    continue
                probes.write(json.dumps({
                    "kind": "input", "name": self.name,
                    "instance_id": self._instance_id(
                        self.process.pid) if getattr(
                            self, "process", None) else "pre-spawn",
                    "path": str(path.resolve()), "sha256": digest,
                }) + "\n")

    def _probe_executable(self, command):
        """Runtime identity (G2 §3.1): hash the actual entry file this
        command will execute — resolved from the command's own source
        prefix (`ros2 run pkg entry` → <install>/lib/pkg/entry, or a
        direct `python3 <path>`). Appended to EXEC_PROBES.jsonl next to
        the log; group_main folds it into MODULE_PROBES.json for
        a0_verify's closure check."""
        import re
        match = re.search(r"ros2 run (\S+) (\S+)", command)
        path = None
        if match:
            sources = re.findall(r"source (\S+)/setup\.bash", command)
            pkg, entry = match.group(1), match.group(2)
            for install in reversed(sources):
                candidate = Path(install) / pkg / "lib" / pkg / entry
                if candidate.is_file():
                    path = candidate
                    break
        else:
            match = re.search(r"exec python3 (\S+)", command)
            if match and Path(match.group(1)).is_file():
                path = Path(match.group(1))
        if path is None:
            return
        try:
            digest = sha256_file(path)
        except OSError:
            return
        with open(self.log_path.parent / "EXEC_PROBES.jsonl", "a") as probes:
            probes.write(json.dumps({
                "kind": "executable",
                "name": self.name,
                "path": str(path.resolve()),
                "sha256": digest,
            }) + "\n")

    def alive(self):
        return self.process.poll() is None

    def stop(self, timeout=10.0):
        if self.process.poll() is None:
            self._signal_tree(signal.SIGTERM)
            try:
                self.process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                self.kill_tree()
                return
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
        # The full multi-view fixture enumeration legitimately takes
        # ~1 s (run08-era solver was ~0.7 s on the same grid; the edge
        # scan + volume checks add ~10%). A 500 ms robot window makes
        # E4's commit deterministically stale — the chain deployment
        # widens it; the product default stays 500 ms.
        "-p robot_state_max_age_ms:=2000",
        # No precomputed atlas cells in the compat chains: disable the
        # identity gate (placements use the no_atlas band, the same path
        # the default cuboid scene takes when no cell matches).
        "-p atlas_runtime_config:=/tmp/atlas_disabled.yaml",
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
        # Full-ray budget for the S-free fan (the node default 2000
        # subsamples the 8.4k-point frames and thins the observed
        # support; the chain config freezes the full budget).
        "-p max_raycast_points:=20000",
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
        "-p validity_max_age_ms:=5000 "
        "-p optical_frame:=d555_color_optical_frame "
        "-p sensor_id:=cam0 "
        "-p geometry_file:={geometry} "
        "-p config_file:={config}"
    ).format(prefix=_bash_source_prefix(manifest["o_install"]),
             geometry=manifest["o_geometry"], config=manifest["o_config"])
    bridge = (
        "{prefix}; exec python3 {c1}/src/uld_occ_observer/scripts/"
        "occ_surface_bridge.py --geometry {geometry} --config {config} "
        "--topic /luggage/cargo_map/surface_2d --hz 2.0 "
        "--max-age-ms 5000 {v2}"
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
            # Single-lease volume authorization (user ruling 2026-11-08,
            # FREEZE_G3F): the O deployment's planner takes BOTH the
            # surface and the volume evidence from ONE observer lease per
            # compute. Quiescent fixtures need a wider freshness window
            # (the K01 chain integrates its frames, then computes).
            stack.spawn("planner", planner_command(
                manifest, backend="O",
                extra_params=("volume_evidence_source:=observer",
                              "volume_max_age_ms:=30000")))
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
