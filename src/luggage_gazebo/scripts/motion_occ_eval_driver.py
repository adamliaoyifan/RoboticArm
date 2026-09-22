#!/usr/bin/env python3
"""MOTION-OCC A1: occupancy-aware multi-slot place paths.

n=2 carryon,standard. Occupancy from IntegrateCargoView (no AddPlacedBox).
Second carry goes through PlacePathPlanner; cartesian/OMPL see occupancy
via motion_planner_node injection plus executor re-sweep.

Selection-stage MoveIt probe: rows that pass the occupancy sweep are
probed via /motion_planner/probe_motion_segment (traverse-equivalent
cartesian carry segment) before select_trajectory picks the winner.
``--place-probe off`` restores occupancy-only selection; a missing or
timing-out service fails open (rows stay unprobed, never rejected).
"""

from __future__ import division

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Pose
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

_SCRIPTS = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from cargo_map_integrate_eval_driver import (  # noqa: E402
    CargoMapIntegrateDriver,
    parse_args as parse_integrate_args,
)
from luggage_description.scene_tf_config_utils import (  # noqa: E402
    _point_in_container_link,
    container_opening_normal_in_world,
    container_opening_target_point_in_world,
    origin_in_world,
    xyz_world_to_base_link,
)
from luggage_gazebo.place_metrics import (  # noqa: E402
    RETURN_FAIL_CODES,
    place_ok,
    summarize,
    trial_to_dict,
)
from luggage_msgs.msg import MotionSegment  # noqa: E402
from luggage_msgs.srv import ComputePlacement, ProbeMotionSegment  # noqa: E402
from luggage_planning.occupancy_place_paths import (  # noqa: E402
    PLACE_PATH_INFEASIBLE,
    PAYLOAD_SOURCE_MEASURED,
    dump_rows,
)
from luggage_planning.place_path_planner import (  # noqa: E402
    PlacePathPlanner,
    apply_variant_to_segment,
)
from pick_retreat_eval_driver import _git_meta  # noqa: E402


DEFAULT_OUT = os.path.normpath(os.path.join(
    _SCRIPTS, "..", "..", "..", "docs", "status", "evidence",
    "motion_occ", "latest"))

# One row probe = 1 IK + 1 plan-only cartesian call; generous ceiling for a
# single row, not for the whole sweep (a circuit breaker stops the sweep
# after PLACE_PROBE_MAX_UNAVAILABLE consecutive fail-opens).
PLACE_PROBE_TIMEOUT_S = 10.0
PLACE_PROBE_MAX_UNAVAILABLE = 3


def _map_to_world(scene, xyz):
    origin, rpy = origin_in_world(scene)
    yaw = float(rpy[2])
    c, s = math.cos(yaw), math.sin(yaw)
    x, y, z = [float(v) for v in xyz]
    return [
        origin[0] + c * x - s * y,
        origin[1] + s * x + c * y,
        origin[2] + z,
    ]


def _world_to_map(scene, xyz):
    base = xyz_world_to_base_link(scene, xyz)
    return _point_in_container_link(base, scene)


class MotionOccDriver(CargoMapIntegrateDriver):
    def __init__(self, args):
        super().__init__(args)
        self._planner = PlacePathPlanner.from_yaml()
        self._selected_path = None
        self._selector_rows = []
        self._last_boundary = None
        self._probe_place = self.create_client(
            ProbeMotionSegment, "/motion_planner/probe_motion_segment",
            callback_group=self._group)
        boundary_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, "/motion_planner/last_boundary", self._on_boundary,
            boundary_qos, callback_group=self._group)

    def _on_boundary(self, msg):
        try:
            boundary = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(boundary, dict) and "t_wall" in boundary:
            self._last_boundary = boundary

    def _wait_boundary(self, t0, timeout=1.5):
        """Latest node boundary record at/after wall time ``t0``."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            boundary = self._last_boundary
            if boundary is not None and boundary.get("t_wall", 0.0) >= t0:
                return boundary
            time.sleep(0.02)
        return self._last_boundary

    def _slot_target_map(self, slot):
        """Map-frame suction contact (box top). Carry Z is not a fixed lift.

        PlacePathPlanner raises from occupied tops and clamps to the lid.
        """
        pos = slot.place_pose.position
        local = _point_in_container_link(
            [pos.x, pos.y, pos.z], self._scene_config)
        height = float(slot.height)
        return (local[0], local[1], local[2] + 0.5 * height)

    def _portal_map(self):
        world = container_opening_target_point_in_world(self._scene_config)
        local = _world_to_map(self._scene_config, world)
        return (local[0], local[1], max(local[2] + 0.35, 0.80))

    def _opening_tangent_map(self):
        normal = container_opening_normal_in_world(self._scene_config)
        return (-float(normal[1]), float(normal[0]))

    def _place_probe_fn(self, slots):
        """Row probe for ``PlacePathPlanner.plan``: one cartesian carry
        segment per occupancy-feasible row, mirroring the traverse segment
        the winner will execute (tool-down at the slot's place yaw,
        OMPL fallback allowed). Fail-open: unavailable/timeout leaves the
        row unprobed (occupancy-only gating), never marks it infeasible.

        Returns ``(probe_fn, stats)``; ``probe_fn`` is ``None`` when the
        service is not up at selection time (stats carries the mode so
        dumps stay attributable).
        """
        stats = {"mode": "on", "probed": 0, "ik_reject": 0,
                 "fraction_reject": 0, "unavailable": 0, "sec": 0.0}
        min_fraction = float(self._planner.cartesian_min_fraction)
        if not self._probe_place.wait_for_service(timeout_sec=5.0):
            stats["mode"] = "service_unavailable"
            return None, stats
        unavailable = [0]

        def probe(row):
            if unavailable[0] >= PLACE_PROBE_MAX_UNAVAILABLE:
                return
            slot = slots[int(row["slot_index"])]
            segment = MotionSegment()
            segment.name = "place_probe"
            segment.type = "cartesian"
            segment.keep_tool_down = True
            segment.allow_ompl_fallback = True
            orientation = slot.place_pose.orientation
            world = [_map_to_world(self._scene_config, xyz)
                     for xyz in (row.get("waypoints") or [])]
            if not world:
                return
            dest = world[-1]
            segment.target_pose.position.x = float(dest[0])
            segment.target_pose.position.y = float(dest[1])
            segment.target_pose.position.z = float(dest[2])
            segment.target_pose.orientation = orientation
            for xyz in world[:-1]:
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = (
                    float(xyz[0]), float(xyz[1]), float(xyz[2]))
                pose.orientation = orientation
                segment.waypoints.append(pose)
            request = ProbeMotionSegment.Request()
            request.segment = segment
            t0 = time.time()
            response = self.call_srv(
                self._probe_place, request, PLACE_PROBE_TIMEOUT_S)
            stats["sec"] += time.time() - t0
            if response is None:
                stats["unavailable"] += 1
                unavailable[0] += 1
                return
            unavailable[0] = 0
            stats["probed"] += 1
            row["probe_s"] = round(time.time() - t0, 3)
            row["ik_ok"] = bool(response.ik_ok)
            # fraction stays -1.0 (no cartesian solution) on purpose: the
            # plan() gate rejects it as a sub-threshold fraction.
            row["cartesian_fraction"] = float(response.fraction)
            if not row["ik_ok"]:
                stats["ik_reject"] += 1
            elif float(response.fraction) < min_fraction:
                stats["fraction_reject"] += 1

        return probe, stats

    def _dump_selector(self, trial, candidates, rows, winner, reason, surface,
                       probe=None):
        folder = os.path.join(self._args.dump_dir, "box_%02d_paths" % trial.index)
        os.makedirs(folder, exist_ok=True)
        payload = {
            "geometry_hash": (surface or {}).get("geometry_hash"),
            "map_revision": (surface or {}).get("map_revision"),
            "reason": reason,
            "n_slots": len(candidates),
            "n_rows": len(rows),
            "winner": winner,
            "rows": dump_rows(rows),
            "candidates": candidates,
            "probe": probe,
        }
        with open(os.path.join(folder, "candidates.json"), "w",
                  encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True, default=str)
            handle.write("\n")
        trial.extras["selector"] = {
            "reason": reason,
            "n_slots": len(candidates),
            "n_rows": len(rows),
            "winner_method": None if not winner else winner.get("method"),
            "winner_slot": None if not winner else winner.get("slot_index"),
            "probe": probe,
            "dump": folder,
        }
        return folder

    def _place_slot_for(self, pick_msg, box_msg, trial):
        del box_msg
        if not bool(getattr(pick_msg, "height_valid", False)):
            trial.fail_code = trial.fail_code or "DETECT_FULL_GEOMETRY_REQUIRED"
            return None, None
        from luggage_msgs.srv import GetCargoMapStats
        stats = self.call_srv(self._stats, GetCargoMapStats.Request(), timeout=5.0)
        request = ComputePlacement.Request()
        request.box = pick_msg
        request.max_candidates = 5
        request.geometry_hash = str(
            (stats.geometry_hash if stats else "")
            or (self._surface_2d or {}).get("geometry_hash") or "")
        placement = self.call_srv(self._compute, request, timeout=30.0)
        if placement is None or not placement.success:
            trial.fail_code = trial.fail_code or (
                getattr(placement, "reason_code", None) or "PLACEMENT_FAILED")
            trial.extras["compute_placement"] = (
                placement.message if placement else "timeout")
            return None, None
        trial.extras["compute_placement"] = placement.message
        slots = list(placement.candidates) or [placement.slot]
        scores = list(placement.candidate_scores) or [0.0]
        cand_dump = []
        planner_slots = []
        for i, slot in enumerate(slots):
            score = float(scores[i]) if i < len(scores) else 0.0
            cand_dump.append({
                "index": i,
                "score": score,
                "width": float(slot.width),
                "height": float(slot.height),
                "depth": float(slot.depth),
            })
            planner_slots.append({
                "index": i,
                "score": score,
                "target": self._slot_target_map(slot),
            })
        surface = dict(self._surface_2d or {})
        probe_fn = None
        probe_stats = {"mode": "off"}
        if getattr(self._args, "place_probe", "on") == "on":
            probe_fn, probe_stats = self._place_probe_fn(slots)
        try:
            winner, rows, reason, _snap = self._planner.plan(
                surface, planner_slots, self._portal_map(),
                payload_wdh=(
                    float(pick_msg.width), float(pick_msg.depth),
                    float(pick_msg.height)),
                expected_hash=str(surface.get("geometry_hash") or "") or None,
                expected_revision=surface.get("map_revision"),
                opening_tangent=self._opening_tangent_map(),
                probe_fn=probe_fn,
            )
        except Exception as exc:  # noqa: BLE001 - eval boundary
            trial.fail_code = PLACE_PATH_INFEASIBLE
            trial.extras["selector_error"] = str(exc)
            self._dump_selector(trial, cand_dump, [], None, str(exc), surface,
                                probe=probe_stats)
            return None, None
        self._selector_rows = rows
        self._dump_selector(trial, cand_dump, rows, winner, reason, surface,
                            probe=probe_stats)
        if winner is None:
            trial.fail_code = PLACE_PATH_INFEASIBLE
            return None, None
        self._selected_path = winner
        slot = slots[int(winner["slot_index"])]
        return slot, self._slot_meta(slot)

    def _execute_segment(self, segment, trial):
        if (str(segment.name) in ("transit", "traverse")
                and self._selected_path
                and self._selected_path.get("waypoints")):
            # Selected carry variant (map frame) onto the segment, in the
            # world frame the planner runs in — same helper the production
            # chain will use when the selector moves into the node layer.
            apply_variant_to_segment(
                segment, list(self._selected_path["waypoints"]),
                frame_convert=lambda xyz: _map_to_world(
                    self._scene_config, xyz))
        t0 = time.time()
        ok, code, rec = super()._execute_segment(segment, trial)
        if rec is not None:
            rec["occupancy_checked"] = bool(self._surface_2d)
            rec["selector_method"] = (
                None if not self._selected_path
                else self._selected_path.get("method"))
            if not ok and PLACE_PATH_INFEASIBLE in str(
                    rec.get("message") or ""):
                code = PLACE_PATH_INFEASIBLE
        if str(segment.name) in ("transit", "traverse"):
            boundary = self._wait_boundary(t0)
            sources = trial.extras.setdefault("payload_sources", {})
            sources[str(segment.name)] = (
                None if not boundary else boundary.get("payload_source"))
            if rec is not None and boundary is not None:
                rec["payload_source"] = boundary.get("payload_source")
                rec["payload_wdh"] = boundary.get("payload_wdh")
        return ok, code, rec


def parse_args(argv=None):
    argv = list(argv or [])
    # This driver's own flags; strip them so the shared place parser (and
    # every other driver reusing it) never sees unknown options.
    local = argparse.ArgumentParser(add_help=False)
    local.add_argument("--place-probe", choices=("on", "off"), default="on")
    local_ns, remaining = local.parse_known_args(argv)
    # MOTION-OCC is an n=2 eval; default it here once. (The old
    # n==3 post-normalization never fired: main() had already appended
    # --n 2, and an explicit --n 3 suppressed it on purpose.)
    if "--n" not in remaining:
        remaining.extend(["--n", "2"])
    args = parse_integrate_args(remaining)
    args.place_probe = local_ns.place_probe
    return args


def _place_reached(record):
    if place_ok(record):
        return True
    if record.fail_code in ("", None, "COVERAGE_LOW"):
        state = str(getattr(record, "place_state", "") or "")
        return state in ("VERIFIED", "HOME", "COMMITTED", "RELEASED")
    return False


def _write_suite(out, records, driver):
    dumps = driver._args.dump_dir
    complete = True
    occupancy_blind = False
    # Privilege gate: every carry-segment occupancy sweep from trial 1 on
    # must have used the perception-measured payload, never the static
    # fallback envelope.
    payload_measured = True
    for rec in records:
        if rec.index < 1:
            continue
        sources = rec.extras.get("payload_sources") or {}
        carry = [v for k, v in sources.items() if k in ("transit", "traverse")]
        if not carry or any(s != PAYLOAD_SOURCE_MEASURED for s in carry):
            payload_measured = False
    for rec in records:
        folder = os.path.join(dumps, "box_%02d_paths" % rec.index)
        if rec.index >= 1 or rec.fail_code == PLACE_PATH_INFEASIBLE:
            if not os.path.isfile(os.path.join(folder, "candidates.json")):
                complete = False
        if rec.fail_code in RETURN_FAIL_CODES + (PLACE_PATH_INFEASIBLE,):
            replay = os.path.join(
                dumps, "trial_%02d_%s" % (
                    rec.index, rec.fail_code), "replay", "manifest.json")
            if not os.path.isfile(replay):
                complete = False
            else:
                try:
                    with open(replay, "r", encoding="utf-8") as handle:
                        manifest = json.load(handle)
                    if not manifest.get("replay_possible"):
                        complete = False
                except (OSError, ValueError):
                    complete = False
        if rec.fail_code and rec.fail_code.startswith("PLACE_PLAN_"):
            extra = rec.extras.get("selector") or {}
            if int(extra.get("n_rows") or 0) < 2:
                occupancy_blind = True
    probe_stats = [
        (r.extras.get("selector") or {}).get("probe") for r in records]
    probe_stats = [s for s in probe_stats if s]
    payload = {
        "n": len(records),
        "place_ok": sum(1 for r in records if _place_reached(r)),
        "fail_codes": [r.fail_code for r in records],
        "clock_publishers": driver.clock_publishers(),
        "capture_complete": complete,
        "occupancy_blind": occupancy_blind,
        "payload_measured": payload_measured,
        "place_probe": {
            "mode": driver._args.place_probe,
            "modes": [s.get("mode") for s in probe_stats],
            "probed": sum(int(s.get("probed") or 0) for s in probe_stats),
            "ik_reject": sum(int(s.get("ik_reject") or 0) for s in probe_stats),
            "fraction_reject": sum(
                int(s.get("fraction_reject") or 0) for s in probe_stats),
            "unavailable": sum(
                int(s.get("unavailable") or 0) for s in probe_stats),
            "sec": round(sum(float(s.get("sec") or 0.0)
                             for s in probe_stats), 3),
        },
        "selectors": [r.extras.get("selector") for r in records],
    }
    with open(os.path.join(out, "suite.json"), "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True, default=str)
        handle.write("\n")
    return payload


def _write_t0(out, extra, args, records, suite, started_at):
    payload = {
        "case_ids": [
            {"index": r.index, "catalog_id": r.catalog_id,
             "fail_code": r.fail_code}
            for r in records],
        "commit": extra.get("commit"),
        "dirty": extra.get("dirty"),
        "n": args.n,
        "driver": "motion_occ_eval_driver.py",
        "ros_domain_id": extra.get("ros_domain_id"),
        "started_at": started_at,
        "ended_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "clock_publishers": suite.get("clock_publishers"),
        "suite": suite,
        "dump_matrix": {
            "T0": "t0_manifest.json",
            "T1": "dumps/box_NN_paths/candidates.json",
            "T2": "replay/last_boundary.json + planning_scene.json on fail",
        },
    }
    path = os.path.join(out, "t0_manifest.json")
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True, default=str)
        handle.write("\n")
    return path


def main(argv=None):
    argv = list(argv or sys.argv[1:])
    if "--out" not in argv:
        argv.extend(["--out", DEFAULT_OUT])
    # --n defaults to 2 inside parse_args (single source of that default).
    args = parse_args(argv)
    os.makedirs(args.out, exist_ok=True)
    args.dump_dir = args.dump_dir or os.path.join(args.out, "dumps")
    os.makedirs(args.dump_dir, exist_ok=True)
    jsonl = os.path.join(args.out, "trials.jsonl")
    with open(jsonl, "w", encoding="utf-8"):
        pass
    rclpy.init()
    driver = MotionOccDriver(args)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(driver)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    records = []
    stop = {"flag": False}

    def _on_sigint(_signum, _frame):
        stop["flag"] = True

    signal.signal(signal.SIGINT, _on_sigint)
    extra = _git_meta(os.path.normpath(os.path.join(_SCRIPTS, "..", "..", "..")))
    started_at = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    try:
        if not args.skip_graph_check:
            err = driver.wait_graph(timeout=180.0)
            if err:
                print("graph: %s" % err)
                return 2
        print("graph ok; MOTION-OCC n=%d" % args.n, flush=True)
        time.sleep(8.0)
        for index in range(args.n):
            if stop["flag"] or not rclpy.ok():
                break
            rec = driver.run_trial(index)
            records.append(rec)
            with open(jsonl, "a", encoding="utf-8") as handle:
                handle.write(json.dumps(trial_to_dict(rec), sort_keys=True) + "\n")
            print(json.dumps({
                "index": rec.index, "fail_code": rec.fail_code,
                "selector": rec.extras.get("selector"),
            }, sort_keys=True), flush=True)
            if rec.fail_code and args.on_place_fail == "stop":
                if rec.fail_code not in RETURN_FAIL_CODES + ("COVERAGE_LOW",):
                    break
        ws = os.path.normpath(os.path.join(_SCRIPTS, "..", "..", ".."))
        extra.update({
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
            "dirty": subprocess.check_output(
                ["git", "status", "--porcelain"], cwd=ws, text=True).count("\n"),
        })
        suite = _write_suite(args.out, records, driver)
        _write_t0(args.out, extra, args, records, suite, started_at)
        summary = summarize(records)
        with open(os.path.join(args.out, "summary.json"), "w",
                  encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2, sort_keys=True)
            handle.write("\n")
        print(json.dumps(suite, sort_keys=True, default=str), flush=True)
        if (suite.get("occupancy_blind") or not suite.get("capture_complete")
                or not suite.get("payload_measured")):
            return 1
        if suite.get("clock_publishers") not in (1, -1):
            return 1
        if any(r.fail_code == "BIN_FULL" for r in records):
            return 1
        # Trial 1 (index 1) placed, or PLACE_PATH_INFEASIBLE with dumped set.
        if len(records) >= 2:
            second = records[1]
            extra = second.extras.get("selector") or {}
            if _place_reached(second):
                return 0
            if (second.fail_code == PLACE_PATH_INFEASIBLE
                    and int(extra.get("n_slots") or 0) >= 2
                    and int(extra.get("n_rows") or 0) >= 4):
                return 0
            return 1
        return 0 if records and _place_reached(records[-1]) else 1
    finally:
        executor.remove_node(driver)
        driver.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
