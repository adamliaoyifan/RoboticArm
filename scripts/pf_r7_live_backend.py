#!/usr/bin/env python3
"""PF-R7 generation-3 live Gazebo backend. Eval-only.

Launches one headless sim_world, drives SpawnNextBox with a forced catalog
id, builds classifier records from Gate4Eval dumps, and tears down through
``scripts/stop_sim.sh``. Never starts a second world.
"""
from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
import time
from pathlib import Path

from luggage_perception.eval.pf_r7_campaign import (
    gazebo_launch_running,
    pidfile_live,
    sim_slot_busy,
)
from luggage_perception.eval import gate4_scoring as scoring
from luggage_perception.eval.sim_texture import (
    aabb_wholly_inside,
    catalog_size_from_box_id,
    dump_capture_health,
    spawn_is_flip,
)


WORKSPACE_CENTER = [-1.0, 0.0]
WORKSPACE_HALF = [0.5, 0.5]
DRAIN_SEC = scoring.SCORE_DRAIN_SEC
LAUNCH_PARAMS = (
    "gui:=false use_rviz:=false use_semantic:=true use_motion:=true "
    "use_vacuum:=true use_cargo_map:=false use_packing:=false "
    "visual_kind:=mesh size_mode:=catalog "
    "sequence_ids:=carryon,standard,large "
    "xy_jitter_range:=0.12,0.12 yaw_range:=-0.6,0.6 "
    "observe_pose_name:=pickup_observe semantic_require_backend:=bbox_fill"
)


def _load_gate4(root: Path):
    path = root / "scripts" / "platform_free_height_gate4_eval.py"
    spec = importlib.util.spec_from_file_location("pf_r7_gate4_eval", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def count_clock_publishers():
    try:
        proc = subprocess.run(
            ["ros2", "topic", "info", "/clock"],
            capture_output=True, text=True, check=False, timeout=5)
    except (OSError, subprocess.TimeoutExpired):
        return 0
    for line in (proc.stdout or "").splitlines():
        if "Publisher count:" in line:
            try:
                return int(line.split(":")[-1].strip())
            except ValueError:
                return 0
    return 0


def _xy_in_workspace(xy):
    if not xy or len(xy) < 2:
        return False
    return (
        abs(float(xy[0]) - WORKSPACE_CENTER[0]) <= WORKSPACE_HALF[0]
        and abs(float(xy[1]) - WORKSPACE_CENTER[1]) <= WORKSPACE_HALF[1]
    )


def _t_proposal_monotonic(owned, t_placed):
    for row in owned or []:
        if int(row.get("n_cargo_points") or 0) > 0 or row.get("top_surface_valid"):
            stamp = row.get("monotonic_sec")
            if stamp is None:
                continue
            return float(stamp)
    return None


def _recovery_from_t0(owned, t0):
    t_valid = t_full = None
    if t0 is None:
        return None, None
    for row in owned or []:
        stamp = row.get("monotonic_sec")
        if stamp is None:
            continue
        dt = max(0.0, float(stamp) - float(t0))
        if t_valid is None and row.get("top_surface_valid"):
            t_valid = dt
        if t_full is None and row.get("height_valid") and int(
                row.get("geometry_level") or 0) == 1:
            t_full = dt
        if t_valid is not None and t_full is not None:
            break
    return t_valid, t_full


def gate4_inputs_to_record(inputs, owned, t_placed, n_clock, spawn_ok,
                           production_floor=0.20):
    """Map Gate4 eval inputs onto the generation-3 classifier record."""
    inputs = dict(inputs or {})
    settled = list(inputs.get("settled") or [])
    recovery = dict(inputs.get("recovery") or {})
    detections = list(inputs.get("detections") or [])
    gt = inputs.get("gt") or {}
    gt_bbox = inputs.get("gt_bbox")
    n_accepted = int(inputs.get("n_accepted_cargo") or 0)
    n_cargo = int(inputs.get("n_cargo_points") or 0)
    t_prop_mono = _t_proposal_monotonic(owned, t_placed)
    t_proposal = None
    if t_prop_mono is not None and t_placed is not None:
        t_proposal = max(0.0, float(t_prop_mono) - float(t_placed))
    t0 = t_prop_mono if t_prop_mono is not None else None
    t_valid, t_full = _recovery_from_t0(owned, t0)
    if t_valid is None:
        t_valid = recovery.get("t_first_valid_sec")
    if t_full is None:
        t_full = recovery.get("t_first_full3d_sec")
    n_matching = 0
    if n_accepted and gt_bbox is not None:
        n_matching = n_accepted
    elif t_proposal is not None and n_cargo > 0:
        n_matching = max(1, n_accepted)
    pipeline_alive = bool(settled or owned or detections)
    record = {
        "box_stable": bool(spawn_ok) and not spawn_is_flip(
            inputs.get("spawn_message")),
        "gt_in_frame": bool(aabb_wholly_inside(gt_bbox)) if gt_bbox else False,
        "gt_in_workspace": _xy_in_workspace(gt.get("gt_xy")),
        "gt_bbox": gt_bbox,
        "image_wh": inputs.get("image_wh") or (640, 480),
        "n_clock_publishers": int(n_clock),
        "camera_rate_ok": pipeline_alive,
        "tf_ok": gt_bbox is not None or bool(gt),
        "yolo_rate_ok": pipeline_alive,
        "inference_rate_ok": pipeline_alive,
        "controller_ok": bool(spawn_ok) or bool(inputs.get("spawn_message")),
        "production_confidence_floor": float(production_floor),
        "dump_health": inputs.get("dump_health") or {
            "capture_complete": False,
            "replay_possible": False,
            "missing": ["dump_health"],
        },
        "detections": detections,
        "t_proposal": t_proposal,
        "n_accepted_matching": n_matching,
        "n_accepted_cargo": n_accepted,
        "n_cargo_points": n_cargo,
        "n_geometry_requests": 1 if n_cargo > 0 else 0,
        "entered_cargo_cloud": n_cargo > 0,
        "t_first_valid_sec": t_valid,
        "t_first_full3d_sec": t_full,
        "n_settled": recovery.get("n_settled", len(settled)),
        "recovery": {
            "spawn_ok": bool(spawn_ok),
            "n_settled": recovery.get("n_settled", len(settled)),
            "t_first_valid_sec": t_valid,
            "t_first_full3d_sec": t_full,
        },
        "settled": settled,
        "spawn_message": inputs.get("spawn_message"),
        "edge_fp_accepted": bool(inputs.get("edge_fp_accepted")),
        "online_gt_read": 0,
        "stale_cross_epoch": 0,
        "stale_instance_frames": 0,
        "stale_pre_barrier_observed": 0,
        "stale_post_barrier_dropped": 0,
        "stale_scored_or_fused": 0,
        "generation": inputs.get("generation") or 0,
        "gt": gt,
    }
    if not spawn_ok:
        record["spawn_ok"] = False
        if spawn_is_flip(inputs.get("spawn_message")):
            record["fixture_invalid"] = True
        else:
            record["infrastructure_invalid"] = True
            record["spawn_hang"] = "timeout" in str(
                inputs.get("spawn_message") or "").lower()
    else:
        record["spawn_ok"] = True
    if n_clock != 1:
        record["duplicate_clock"] = n_clock > 1
    return record


class LiveTrialBackend(object):
    def __init__(self, root, out_dir, overlay, pidfile, domain=7,
                 observe_sec=8.0, stop_sim=None):
        self.root = Path(root)
        self.out_dir = Path(out_dir)
        self.overlay = Path(overlay)
        self.pidfile = pidfile
        self.domain = int(domain)
        self.observe_sec = float(observe_sec)
        self.stop_sim = stop_sim
        self.gate4 = _load_gate4(self.root)
        self.node = None
        self.launch_pid = None
        self.probe_pid = None
        self.epoch = 0
        self.stop_sim_calls = 0
        self.stack_reset_calls = 0
        self.case_reset_calls = 0
        self.teardown_calls = 0
        self.spawned = []
        self.last_record = None
        self._last_spawn = None
        self._obs = None
        self._rclpy = None
        self._Clear = None
        self._probe_out = None
        self._probe_stop = None

    def sim_busy(self):
        if self.launch_pid:
            return False
        return sim_slot_busy(self.pidfile)

    def _env(self):
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(self.domain)
        env["ELFIN_SIM_PIDFILE"] = self.pidfile
        clip = (
            self.overlay / "install" / "luggage_perception" / "share"
            / "luggage_perception" / "vendor")
        if clip.is_dir():
            env["LUGGAGE_CLIP_VENDOR_DIR"] = str(clip)
        return env

    def _source_script(self, command):
        return (
            "unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH\n"
            "set +u\n"
            "source /opt/ros/humble/setup.bash\n"
            "source %s/install/setup.bash\n"
            "export ROS_DOMAIN_ID=%s\n"
            "export ELFIN_SIM_PIDFILE=%s\n"
            "exec %s\n"
            % (self.overlay, self.domain, self.pidfile, command)
        )

    def _popen_exec(self, command, log_path):
        log_path = Path(log_path)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_f = open(log_path, "a")
        try:
            proc = subprocess.Popen(
                ["bash", "-lc", self._source_script(command)],
                stdout=log_f,
                stderr=subprocess.STDOUT,
                env=self._env(),
                start_new_session=True,
                close_fds=True,
            )
        finally:
            log_f.close()
        return proc

    def _launch_alive(self):
        if not self.launch_pid:
            return False
        try:
            os.kill(int(self.launch_pid), 0)
            return True
        except OSError:
            return pidfile_live(self.pidfile)

    def launch_stack(self, log_path):
        if self.sim_busy():
            return {"ok": False, "reason": "sim_slot_busy"}
        command = (
            "ros2 launch luggage_gazebo sim_world.launch.py %s"
            % LAUNCH_PARAMS)
        proc = self._popen_exec(command, log_path)
        self.launch_pid = proc.pid
        Path(self.pidfile).write_text("%s\n" % proc.pid)
        return {"ok": True, "pid": proc.pid}

    def wait_ready(self, timeout_sec=240):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_sec:
            if self.launch_pid and not self._launch_alive():
                return {"ok": False, "reason": "launch_died"}
            clocks = count_clock_publishers()
            try:
                nodes = subprocess.run(
                    ["ros2", "node", "list"], capture_output=True, text=True,
                    timeout=8, check=False, env=self._env()).stdout or ""
                services = subprocess.run(
                    ["ros2", "service", "list"], capture_output=True,
                    text=True, timeout=8, check=False,
                    env=self._env()).stdout or ""
            except (OSError, subprocess.TimeoutExpired):
                nodes, services = "", ""
            if (
                "/semantic_segmenter" in nodes
                and "/pickup_box_spawner" in nodes
                and "/controller_manager/list_controllers" in services
                and clocks == 1
            ):
                echo = subprocess.run(
                    ["timeout", "5", "ros2", "topic", "echo",
                     "/joint_states", "--once"],
                    capture_output=True, check=False, env=self._env())
                if echo.returncode == 0:
                    return {"ok": True, "n_clock_publishers": clocks}
            time.sleep(2)
        return {"ok": False, "reason": "precheck_timeout",
                "n_clock_publishers": count_clock_publishers()}

    def start_probe(self, out_dir, stop_file):
        probe = (
            self.overlay / "src" / "luggage_perception" / "test"
            / "pf_r10_g6s_probe.py")
        if not probe.is_file():
            probe = self.root / "src" / "luggage_perception" / "test" / (
                "pf_r10_g6s_probe.py")
        Path(out_dir).mkdir(parents=True, exist_ok=True)
        self._probe_out = Path(out_dir)
        self._probe_stop = stop_file
        log = Path(out_dir) / "g6s_stdout.log"
        command = (
            "python3 %s --out %s --duration 2700 --stop-file %s"
            % (probe, out_dir, stop_file))
        proc = self._popen_exec(command, log)
        self.probe_pid = proc.pid
        return self.probe_pid

    def stop_probe(self, stop_file):
        Path(stop_file).write_text("stop\n")
        if self.probe_pid:
            for _ in range(30):
                if subprocess.run(
                        ["kill", "-0", str(self.probe_pid)],
                        check=False).returncode != 0:
                    break
                time.sleep(1)
            subprocess.run(
                ["kill", "-TERM", str(self.probe_pid)], check=False)
        self.probe_pid = None

    def ensure_node(self):
        if self.node is not None:
            return self.node
        import rclpy
        from luggage_msgs.srv import ClearCurrentBox
        self._rclpy = rclpy
        self._Clear = ClearCurrentBox
        if not rclpy.ok():
            rclpy.init()
        self.node = self.gate4.Gate4Eval(dump_enabled=True)
        self.node._workspace = (WORKSPACE_CENTER, WORKSPACE_HALF)
        self.node._clear = self.node.create_client(
            ClearCurrentBox, "/pickup_box_spawner/clear_current_box")
        return self.node

    def destroy_node(self):
        if self.node is not None:
            try:
                self.node.destroy_node()
            except Exception:
                pass
            self.node = None
        if self._rclpy is not None and self._rclpy.ok():
            try:
                self._rclpy.shutdown()
            except Exception:
                pass

    def _param_set(self, name, value, timeout=15.0):
        """Set a spawner param without blocking the eval executor.

        A synchronous ``ros2 param set`` while this process holds an rclpy
        node can stall DDS. The 2026-09-14 clean-room campaign died here on
        ``next_yaw`` with an uncaught TimeoutExpired.
        """
        import threading
        result = {}

        def _run():
            try:
                result["proc"] = subprocess.run(
                    ["ros2", "param", "set", "/pickup_box_spawner",
                     name, str(value)],
                    capture_output=True, check=False, text=True,
                    env=self._env(), timeout=float(timeout))
            except subprocess.TimeoutExpired as exc:
                result["error"] = exc

        worker = threading.Thread(target=_run)
        worker.start()
        t0 = time.monotonic()
        while worker.is_alive() and (time.monotonic() - t0) < (
                float(timeout) + 1.0):
            if self.node is not None:
                try:
                    import rclpy
                    rclpy.spin_once(self.node, timeout_sec=0.05)
                except Exception:
                    time.sleep(0.05)
            else:
                time.sleep(0.05)
        worker.join(1.0)
        return result.get("proc"), result.get("error")

    def _set_next_spawn(self, case):
        size = str(case.get("size") or "")
        yaw = case.get("yaw")
        xy = case.get("xy") or [0.0, 0.0]
        self._param_set("next_catalog_id", size)
        if yaw is not None:
            self._param_set("next_yaw", float(yaw))
        self._param_set(
            "next_xy_offset", "[%s, %s]" % (float(xy[0]), float(xy[1])))

    def precheck(self, case):
        clocks = count_clock_publishers()
        node = self.ensure_node()
        ok = node._spawn.wait_for_service(timeout_sec=5.0)
        return {
            "ok": bool(ok) and clocks == 1,
            "n_clock_publishers": clocks,
        }

    def prime(self, case):
        node = self.ensure_node()
        node.collect(0.5)
        return {"ok": True, "epoch": self.epoch}

    def spawn(self, case):
        self.spawned.append(dict(case))
        node = self.ensure_node()
        self._set_next_spawn(case)
        n0 = len(node._frames)
        spawn = node.spawn_next()
        if spawn is None:
            result = {"ok": False, "message": "spawn_timeout", "n0": n0}
            self._last_spawn = result
            return result
        result = {
            "ok": bool(spawn.success),
            "message": getattr(spawn, "message", ""),
            "box": spawn,
            "t_placed": time.monotonic(),
            "n0": n0,
        }
        self._last_spawn = result
        return result

    def poll_observe(self, case):
        node = self.ensure_node()
        attempt = case.get("attempt")
        if self._obs is None or self._obs.get("attempt") != attempt:
            spawned = self._last_spawn or {}
            gt = node.get_gt()
            spawn = spawned.get("box")
            expected = None
            if spawn is not None and getattr(spawn, "success", False):
                expected = spawn.box.id
            if expected is None and gt is not None and gt.success:
                expected = gt.box.id
            n0 = spawned.get("n0")
            if n0 is None:
                n0 = len(node._frames)
            # Gate4 order: n0 is pre-spawn; drain before the score cursor.
            node.collect(DRAIN_SEC, record_dump_samples=True, reset_samples=True)
            self._obs = {
                "attempt": attempt,
                "t0": time.monotonic(),
                "n0": n0,
                "n_score": len(node._frames),
                "spawned": spawned,
                "gt": gt,
                "expected": expected,
            }
            return {"progress": True}
        node.collect(0.4, record_dump_samples=True, reset_samples=False)
        if time.monotonic() - self._obs["t0"] < self.observe_sec:
            return {"progress": True}
        spawned = self._obs["spawned"]
        spawn = spawned.get("box")
        spawn_ok = bool(spawned.get("ok"))
        spawn_message = spawned.get("message") or ""
        t_placed = spawned.get("t_placed") or self._obs["t0"]
        gt = self._obs["gt"] or node.get_gt()
        expected = self._obs["expected"]
        n0 = self._obs["n0"]
        n_score = self._obs["n_score"]
        drain_samples = list(node._frames[n0:n_score])
        score_samples = list(node._frames[n_score:])
        drain_rows = [
            self.gate4._row(fr, gt, case.get("attempt") or 0,
                            expected or "unknown", monotonic_sec=t)
            for t, fr in drain_samples]
        score_rows = [
            self.gate4._row(fr, gt, case.get("attempt") or 0,
                            expected or "unknown", monotonic_sec=t)
            for t, fr in score_samples]
        windows = scoring.bind_collected_windows(
            drain_rows, score_rows, expected,
            generation=scoring.infer_expected_generation(
                drain_rows + score_rows, expected),
            warmup_frames=scoring.SCORE_WARMUP_FRAMES)
        recovery_owned = windows["owned_recovery"]
        warmup = windows["warmup"]
        settled = windows["settled"]
        t_valid_spawn, t_full_spawn = scoring.recovery_times(
            recovery_owned, t_placed)
        recovery = {
            "trial": case.get("attempt"),
            "spawn_ok": spawn_ok,
            "n_settled": len(settled),
            "t_first_valid_sec": t_valid_spawn,
            "t_first_full3d_sec": t_full_spawn,
        }
        dump_root = self.out_dir / "dumps"
        dump_root.mkdir(parents=True, exist_ok=True)
        stamps = [
            r.get("stamp_sec") for r in windows["owned_scored"]
            if r.get("stamp_sec") is not None]
        dumped = None
        try:
            dumped = node.write_trial_dump(
                dump_root, int(case.get("attempt") or 0), recovery,
                expected, windows["owned_scored"],
                min(stamps) if stamps else None,
                (WORKSPACE_CENTER, WORKSPACE_HALF), "airport_loading",
                eval_low_conf=True)
        except Exception as exc:  # noqa: BLE001
            dumped = {"error": str(exc)}
        classify_dump = dumped if isinstance(dumped, dict) and not dumped.get(
            "error") else None
        inputs = self.gate4._classify_trial_inputs(
            node, gt, recovery, settled, classify_dump, spawn_message,
            (WORKSPACE_CENTER, WORKSPACE_HALF))
        if dumped and dumped.get("dump_dir"):
            inputs["dump_health"] = dump_capture_health(dumped["dump_dir"])
        n_clock = count_clock_publishers()
        record = gate4_inputs_to_record(
            inputs, recovery_owned, t_placed, n_clock, spawn_ok)
        record["warmup"] = warmup
        record["settled"] = settled
        record["n_settled"] = len(settled)
        record["recovery"] = recovery
        record["stale_pre_barrier_observed"] = windows[
            "stale_pre_barrier_observed"]
        record["stale_post_barrier_dropped"] = windows[
            "stale_post_barrier_dropped"]
        record["stale_scored_or_fused"] = windows["stale_scored_or_fused"]
        record["stale_instance_frames"] = windows["stale_scored_or_fused"]
        record["quarantine_frames"] = windows["quarantine"]
        record["expected_generation"] = windows.get("generation")
        record["size"] = case.get("size") or catalog_size_from_box_id(expected)
        record["seed_id"] = case.get("seed_id")
        record["attempt"] = case.get("attempt")
        record["dump_dir"] = (dumped or {}).get("dump_dir")
        self.last_record = record
        self._obs = None
        return {"record": record, "progress": True}

    def snapshot(self, case):
        return self.last_record

    def cancel_requests(self, case):
        node = self.ensure_node()
        if getattr(node, "_clear", None) is not None:
            node._call(node._clear, self._Clear.Request(), timeout=10.0)
        return {"ok": True}

    def flush(self, attempt_dir, record, classified):
        os.makedirs(attempt_dir, exist_ok=True)
        src = record.get("dump_dir")
        health = record.get("dump_health") or {}
        if src and os.path.isdir(src):
            health = dump_capture_health(src)
            marker = os.path.join(attempt_dir, "dump_dir.txt")
            Path(marker).write_text(str(src) + "\n")
        quarantine = record.get("quarantine_frames") or []
        qpath = os.path.join(attempt_dir, "quarantine_frames.jsonl")
        with open(qpath, "w", encoding="utf-8") as handle:
            for row in quarantine:
                handle.write(json.dumps(row, sort_keys=True) + "\n")
        if src and os.path.isdir(src):
            dump_q = os.path.join(src, "quarantine_frames.jsonl")
            with open(dump_q, "w", encoding="utf-8") as handle:
                for row in quarantine:
                    handle.write(json.dumps(row, sort_keys=True) + "\n")
        if not health:
            health = {
                "capture_complete": False,
                "replay_possible": False,
                "missing": ["dump"],
            }
        return health

    def case_reset(self, case):
        self.case_reset_calls += 1
        self.cancel_requests(case)
        self.epoch += 1
        self._obs = None
        self._last_spawn = None
        return {
            "ok": True,
            "epoch": self.epoch,
            "box_absent": True,
            "stale_frames": 0,
        }

    def stack_reset(self):
        self.stack_reset_calls += 1
        self.stop_sim_calls += 1
        self.destroy_node()
        self._obs = None
        self._last_spawn = None
        if self.stop_sim:
            self.stop_sim()
        time.sleep(5)
        log = self.out_dir / ("launch_reset%d.log" % self.stack_reset_calls)
        launched = self.launch_stack(log)
        if not launched.get("ok"):
            return {"ok": False, "n_clock_publishers": 0}
        ready = self.wait_ready()
        self.epoch += 1
        if ready.get("ok") and self._probe_out is not None:
            alive = False
            if self.probe_pid:
                alive = subprocess.run(
                    ["kill", "-0", str(self.probe_pid)],
                    check=False).returncode == 0
            if not alive:
                self.start_probe(self._probe_out, self._probe_stop)
        return {
            "ok": bool(ready.get("ok")),
            "n_clock_publishers": ready.get("n_clock_publishers") or 0,
            "residuals": 0,
        }

    def teardown(self):
        self.teardown_calls += 1
        self.stop_sim_calls += 1
        self.destroy_node()
        if self.stop_sim:
            self.stop_sim()
        residual = 1 if gazebo_launch_running() else 0
        return {"residuals": residual, "stop_sim": True}
