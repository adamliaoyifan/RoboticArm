#!/usr/bin/env python3
"""Pack-to-full eval driver (Todo 5 slice D).

Loops spawn -> pick(vacuum) -> ComputePlacement -> place -> commit until a
stop condition (BIN_FULL / SPAWN_EXHAUSTED / MAX_BOXES / ABORT / TIMEOUT),
writing the ledger-first evidence contract from
docs/plans/packing_eval_metrics.md.

Subclasses PlaceSmokeDriver for the pick+place state machine. This file owns
the multi-box loop, placement service, cargo_map commit, FinalizeCurrentBox
(keep the Gazebo model), and per-box dumps. Does NOT modify
pick_retreat_eval_driver.py beyond write_trial_dump(exact_dir=).
"""

from __future__ import division

import argparse
import copy
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))

from place_smoke_driver import (  # noqa: E402
    PlaceSmokeDriver,
    parse_args as parse_place_args,
    _yaw_quat,
)

from luggage_gazebo.place_metrics import place_ok  # noqa: E402
from luggage_gazebo.place_gt_dump import write_pack_layout_dump  # noqa: E402
from luggage_description.scene_tf_config_utils import (  # noqa: E402
    xyz_base_link_to_world,
    yaw_base_link_to_world,
)
from luggage_msgs.msg import SlotSpec
from luggage_msgs.srv import (
    AddPlacedBox,
    ClearCurrentBox,
    ComputePlacement,
    FinalizeCurrentBox,
    GetCurrentBox,
    SpawnNextBox,
    RemovePlacedBox,
)
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

MAX_CONSECUTIVE_FAILURES = 3
INNER_FLOOR_XY = 1.49 * 1.97
INNER_VOLUME = 1.49 * 1.97 * (2.01 - 0.53)


class PackEvalDriver(PlaceSmokeDriver):

    def __init__(self, args):
        super().__init__(args)
        self._compute = self.create_client(
            ComputePlacement, "/placement_planner/compute_placement")
        self._add_map = self.create_client(
            AddPlacedBox, "/cargo_map/add_placed_box")
        self._remove_box = self.create_client(
            RemovePlacedBox, "/cargo_map/remove_placed_box")
        self._finalize = self.create_client(
            FinalizeCurrentBox, "/pickup_box_spawner/finalize_current_box")
        latch = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._surface_2d = {}
        self._last_result = {}
        self.create_subscription(
            String, "/luggage/cargo_map/surface_2d",
            self._on_surface, latch, callback_group=self._group)
        self.create_subscription(
            String, "/placement_planner/last_result",
            self._on_last_result, latch, callback_group=self._group)
        self._ledger = []
        self._commit_index = 0
        self._consecutive_failures = 0
        self._committed_ledger_boxes = []
        self._placed_slots = []
        self._placed_records = []
        self._out = args.out
        self._dumps = os.path.join(self._out, "dumps")
        os.makedirs(self._dumps, exist_ok=True)
        self._suite_t0 = time.time()
        self._last_rejected = None

    def graph_error(self):
        err = super().graph_error()
        if err:
            return err
        names = set(self.get_node_names())
        missing = [n for n in ("cargo_volume_mapper", "placement_planner")
                   if n not in names]
        if missing:
            return "missing nodes: %s" % ", ".join(missing)
        return ""

    def _on_surface(self, msg):
        try:
            self._surface_2d = json.loads(msg.data)
        except ValueError:
            pass

    def _on_last_result(self, msg):
        try:
            self._last_result = json.loads(msg.data)
        except ValueError:
            pass

    def _dump_json(self, folder, name, payload):
        os.makedirs(folder, exist_ok=True)
        path = os.path.join(folder, name)
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True, default=str)
            handle.write("\n")
        return path

    def _ledger_line(self, **fields):
        self._ledger.append(fields)
        with open(os.path.join(self._out, "ledger.jsonl"), "a",
                  encoding="utf-8") as handle:
            handle.write(json.dumps(fields, sort_keys=True, default=str) + "\n")
        with open(os.path.join(self._out, "trials.jsonl"), "a",
                  encoding="utf-8") as handle:
            handle.write(json.dumps(fields, sort_keys=True, default=str) + "\n")

    def _box_dump_dir(self, seq, slug):
        path = os.path.join(self._dumps, "box_%02d_%s" % (seq, slug))
        os.makedirs(path, exist_ok=True)
        return path

    def _write_sequence(self, sequence_ids, max_boxes, skip, seed):
        ids = [s.strip() for s in str(sequence_ids).split(",") if s.strip()]
        payload = {
            "sequence_ids": ids,
            "visual_kind": getattr(self._args, "visual_kind", "mesh"),
            "size_mode": "catalog",
            "max_boxes": max_boxes,
            "skip_unplaceable": skip,
            "seed": seed,
            "planned": [
                {"seq": None, "catalog_id": catalog, "template": True}
                for catalog in ids
            ],
        }
        with open(os.path.join(self._out, "sequence.json"), "w",
                  encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)

    def _write_rtf(self, seq, rtf_start, rtf_end):
        with open(os.path.join(self._out, "rtf.jsonl"), "a",
                  encoding="utf-8") as handle:
            handle.write(json.dumps({
                "seq": seq,
                "rtf_start": rtf_start,
                "rtf_end": rtf_end,
            }, sort_keys=True) + "\n")

    def _call(self, client, request, timeout=30.0):
        import threading
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        event = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout):
            return None
        return future.result()

    def _sample_rtf(self):
        try:
            import subprocess
            result = subprocess.run(
                ["ign", "topic", "-e", "-t",
                 "/world/airport_loading/stats", "--num", "1"],
                capture_output=True, text=True, timeout=5)
            for line in result.stdout.splitlines():
                if "real_time_factor" in line:
                    return float(line.split(":")[1])
        except (OSError, ValueError, subprocess.TimeoutExpired):
            pass
        return None

    def _corridor_audit(self, slot):
        from luggage_perception.corridor_audit import audit_corridor
        from luggage_description.scene_tf_config_utils import (
            _point_in_container_link,
        )
        pos = slot.place_pose.position
        center_local = _point_in_container_link(
            [pos.x, pos.y, pos.z], self._scene_config)
        return audit_corridor(
            center_local, [slot.width, slot.depth, slot.height],
            self._committed_ledger_boxes,
            [1.49, 1.97, 1.48], [0.55, 0.40, 0.25],
            opening_side="negative_x"), center_local

    def _slot_meta(self, slot):
        pos = slot.place_pose.position
        base_xyz = [pos.x, pos.y, pos.z]
        world_xyz = xyz_base_link_to_world(self._scene_config, base_xyz)
        yaw_base = 2.0 * math.atan2(
            slot.place_pose.orientation.z, slot.place_pose.orientation.w)
        yaw_world = yaw_base_link_to_world(self._scene_config, yaw_base)
        return {
            "planning_frame": "world",
            "pose_world": {"position": world_xyz, "yaw": yaw_world},
            "pose_base_link": {"position": base_xyz, "yaw": yaw_base},
            "source": "compute_placement",
        }

    def _slot_world_msg(self, slot, slot_meta=None):
        if slot_meta is None:
            slot_meta = self._slot_meta(slot)
        out = SlotSpec()
        out.layer, out.row, out.col = slot.layer, slot.row, slot.col
        out.width, out.depth, out.height = slot.width, slot.depth, slot.height
        pos = slot_meta["pose_world"]["position"]
        out.place_pose.position.x = float(pos[0])
        out.place_pose.position.y = float(pos[1])
        out.place_pose.position.z = float(pos[2])
        out.place_pose.orientation = _yaw_quat(slot_meta["pose_world"]["yaw"])
        return out

    def _dump_placement(self, dump_dir, box, size, spawn, extra=None):
        last = dict(self._last_result or {})
        self._dump_json(dump_dir, "compute_placement.json", last)
        histogram = last.get("reject_histogram") or {}
        self._dump_json(dump_dir, "reject_histogram.json", histogram)
        if self._surface_2d:
            self._dump_json(dump_dir, "surface_2d.json", self._surface_2d)
        self._dump_json(dump_dir, "box.json", {
            "catalog_id": self._catalog_id(spawn),
            "size_wdh": size,
            "mass_kg": self._mass(spawn),
            "spawn_id": getattr(getattr(spawn, "box", None), "id", None),
            "detection": extra or {},
        })
        return histogram

    def run(self):
        args = self._args
        self._write_sequence(
            args.sequence_ids, args.max_boxes,
            args.skip_unplaceable, args.seed)
        for name in ("ledger.jsonl", "trials.jsonl", "rtf.jsonl"):
            with open(os.path.join(self._out, name), "w", encoding="utf-8"):
                pass
        if not getattr(args, "skip_graph_check", False):
            err = self.wait_graph(timeout=90.0)
            if err:
                print("graph: %s" % err)
                self._write_suite("ABORT", 0, self._sample_rtf())
                return
        termination = "MAX_BOXES"
        seq = 0
        rtf_start = self._sample_rtf()

        while seq < args.max_boxes:
            rtf_end = self._sample_rtf()
            self._write_rtf(seq, rtf_start, rtf_end)
            rtf_start = rtf_end

            goto_ok, goto_msg, _ = self._home_arm()
            if not goto_ok:
                termination = "ABORT"
                self._ledger_line(
                    seq=seq, committed=False, fail_code="GOTO_FAILED",
                    spawn_id=None, catalog_id=None, message=goto_msg)
                break

            t_spawn = time.time()
            spawn = self._call(
                self._spawn, SpawnNextBox.Request(), timeout=20.0)
            if spawn is None or not spawn.success:
                termination = "SPAWN_EXHAUSTED"
                self._ledger_line(
                    seq=seq, committed=False, fail_code="SPAWN_EXHAUSTED",
                    spawn_id=None, catalog_id=None,
                    t_ros_spawn=t_spawn, volume_m3=0.0)
                break

            current = self._call(
                self._current, GetCurrentBox.Request(), timeout=10.0)
            box = current.box if current and current.success else spawn.box
            size = [box.width, box.depth, box.height]
            volume = size[0] * size[1] * size[2]
            dump_slug = "pending"

            request = self._compute_request(box)
            placement = self._call(self._compute, request, timeout=30.0)
            if placement is None or not placement.success:
                reason = (placement.message
                          if placement else "COMPUTE_TIMEOUT")
                last = dict(self._last_result or {})
                self._last_rejected = {
                    "seq": seq,
                    "catalog_id": self._catalog_id(spawn),
                    "size_wdh": size,
                    "n_candidates_total": last.get("n_candidates_total", -1),
                    "n_feasible": last.get("n_feasible", 0),
                    "reject_histogram": last.get("reject_histogram") or
                    self._parse_histogram(reason),
                }
                if args.skip_unplaceable and "no_candidate" in reason:
                    dump_slug = "SKIP_NO_SLOT"
                    dump_path = self._box_dump_dir(seq, dump_slug)
                    self._dump_placement(dump_path, box, size, spawn)
                    self._ledger_line(
                        seq=seq, committed=False, fail_code=dump_slug,
                        spawn_id=spawn.box.id,
                        catalog_id=self._catalog_id(spawn),
                        size_wdh=size, mass_kg=self._mass(spawn),
                        t_ros_spawn=t_spawn, volume_m3=volume,
                        message=reason, dump=dump_path)
                    self._call(self._clear, ClearCurrentBox.Request(),
                               timeout=15.0)
                    seq += 1
                    continue
                termination = "BIN_FULL"
                dump_slug = "BIN_FULL"
                dump_path = self._box_dump_dir(seq, dump_slug)
                self._dump_placement(dump_path, box, size, spawn)
                self._dump_json(dump_path, "commit.json", {"committed": False})
                self._dump_json(dump_path, "final_layout.json", {
                    "path": "../../final_layout",
                    "note": "suite writes final_layout/ on stop",
                })
                self._ledger_line(
                    seq=seq, committed=False, fail_code="BIN_FULL",
                    spawn_id=spawn.box.id,
                    catalog_id=self._catalog_id(spawn),
                    size_wdh=size, mass_kg=self._mass(spawn),
                    t_ros_spawn=t_spawn, volume_m3=volume,
                    message=reason, dump=dump_path)
                self._call(self._clear, ClearCurrentBox.Request(),
                           timeout=15.0)
                break

            slot = placement.slot
            audit, center_local = self._corridor_audit(slot)
            slot_meta = self._slot_meta(slot)
            dump_slug = "ok"
            args.dump_dir = self._box_dump_dir(seq, "pending")
            args.dump_exact = True
            try:
                record = self.run_trial(
                    seq, slot=slot, slot_meta=slot_meta,
                    already_spawned=True, keep_placed=True)
                commit_ok = place_ok(record)
                fail = getattr(record, "fail_code", "") or ""
                if fail and fail != "GOTO_FAILED":
                    dump_slug = fail
                    commit_ok = False
            except Exception as exc:  # noqa: BLE001 - ledger boundary
                dump_slug = "DRIVER_EXC_%s" % type(exc).__name__
                commit_ok = False
                record = None
                fail = dump_slug
                self.get_logger().error("trial %d raised %s" % (seq, exc))

            t_commit = time.time()
            cycle = t_commit - t_spawn
            if commit_ok:
                add = self._call(
                    self._add_map, self._add_request(self._slot_world_msg(slot)),
                    timeout=15.0)
                commit_ok = bool(add and add.success)
                if commit_ok:
                    self._committed_ledger_boxes.append(
                        (center_local, [slot.width, slot.depth, slot.height]))
                    self._placed_slots.append(copy.deepcopy(slot))
                    self._placed_records.append({
                        "seq": seq,
                        "commit_index": self._commit_index,
                        "catalog_id": self._catalog_id(spawn),
                        "spawn_id": spawn.box.id,
                        "gz_model": spawn.box.id,
                        "size_wdh": size,
                        "mass_kg": self._mass(spawn),
                        "volume_m3": volume,
                        "pose_world": {
                            "position": slot_meta["pose_world"]["position"],
                            "yaw": slot_meta["pose_world"]["yaw"],
                            "rpy": [0.0, 0.0, slot_meta["pose_world"]["yaw"]],
                        },
                        "pose_base_link": {
                            "position": slot_meta["pose_base_link"]["position"],
                            "yaw": slot_meta["pose_base_link"]["yaw"],
                        },
                    })
                    dump_slug = "ok"
                    self._call(self._finalize, FinalizeCurrentBox.Request(),
                               timeout=15.0)
                else:
                    dump_slug = "COMMIT_FAILED"
                    self._call(self._clear, self._clear_request(), timeout=15.0)
            else:
                self._consecutive_failures += 1
                if fail != "GOTO_FAILED":
                    self._call(self._clear, self._clear_request(), timeout=15.0)

            if commit_ok:
                self._consecutive_failures = 0
                commit_index = self._commit_index
                self._commit_index += 1
            else:
                commit_index = None
                if self._consecutive_failures >= MAX_CONSECUTIVE_FAILURES:
                    termination = "ABORT"

            dump_path = self._rename_dump_dir(seq, dump_slug)
            self._args.dump_dir = dump_path
            extras = getattr(record, "extras", {}) if record is not None else {}
            self._dump_placement(dump_path, box, size, spawn, extra=extras)
            self._dump_json(dump_path, "slot.json", slot_meta)
            self._dump_json(dump_path, "corridor_audit.json", audit)
            self._dump_json(dump_path, "commit.json", {
                "committed": bool(commit_ok),
                "commit_index": commit_index,
                "map_add": extras.get("add_placed"),
            })
            self._ledger_line(
                seq=seq, commit_index=commit_index,
                committed=commit_ok,
                fail_code="" if commit_ok else dump_slug,
                spawn_id=spawn.box.id,
                catalog_id=self._catalog_id(spawn),
                size_wdh=size, mass_kg=self._mass(spawn),
                yaw_selected=math.degrees(slot_meta["pose_world"]["yaw"]),
                slot_rank=0,
                pose_planned_world={
                    "position": slot_meta["pose_world"]["position"],
                    "rpy": [0.0, 0.0, slot_meta["pose_world"]["yaw"]],
                },
                pose_planned_base={
                    "position": slot_meta["pose_base_link"]["position"],
                    "rpy": [0.0, 0.0, slot_meta["pose_base_link"]["yaw"]],
                },
                corridor_audit=audit,
                t_ros_spawn=t_spawn, t_ros_commit=t_commit,
                cycle_wall_sec=round(cycle, 2),
                volume_m3=round(volume, 5),
                dump=dump_path,
            )
            seq += 1
            if termination == "ABORT":
                break

        self._write_suite(termination, seq, rtf_start)

    def _rename_dump_dir(self, seq, slug):
        pending = self._box_dump_dir(seq, "pending")
        dest = os.path.join(self._dumps, "box_%02d_%s" % (seq, slug))
        if pending == dest:
            return dest
        if os.path.isdir(dest):
            return dest
        if os.path.isdir(pending):
            os.rename(pending, dest)
            return dest
        os.makedirs(dest, exist_ok=True)
        return dest

    def _clear_request(self):
        return ClearCurrentBox.Request()

    def _compute_request(self, box):
        request = ComputePlacement.Request()
        request.box = box
        request.placed = list(self._placed_slots)
        return request

    @staticmethod
    def _add_request(slot):
        request = AddPlacedBox.Request()
        request.slot = slot
        return request

    @staticmethod
    def _catalog_id(spawn):
        model = getattr(getattr(spawn, "box", None), "id", "") or ""
        for catalog in ("carryon", "standard", "large"):
            if catalog in model:
                return catalog
        return "unknown"

    def _mass(self, spawn):
        payload = {}
        try:
            raw = self._current_box_topic.get("payload")
            if raw:
                payload = json.loads(raw)
        except (TypeError, ValueError):
            payload = {}
        try:
            return float(payload.get("mass_kg") or 0.0)
        except (TypeError, ValueError):
            return 0.0

    @staticmethod
    def _parse_histogram(message):
        histogram = {}
        for token in str(message or "").split():
            if "=" in token and not token[0].isdigit():
                key, _, value = token.partition("=")
                try:
                    histogram[key] = int(value)
                except ValueError:
                    continue
        return histogram

    def _write_suite(self, termination, attempted, last_rtf):
        committed = [entry for entry in self._ledger
                     if entry.get("committed")]
        volume_sum = sum(entry.get("volume_m3", 0.0) for entry in committed)
        footprint = 0.0
        for entry in committed:
            size = entry.get("size_wdh") or [0.0, 0.0, 0.0]
            footprint += float(size[0]) * float(size[1])
        catalog_counts = {}
        for entry in committed:
            catalog = entry.get("catalog_id", "unknown")
            catalog_counts[catalog] = catalog_counts.get(catalog, 0) + 1
        ids = [s.strip() for s in str(self._args.sequence_ids).split(",")
               if s.strip()]
        wall_total = time.time() - self._suite_t0
        layout_dir = os.path.join(self._out, "final_layout")
        layout_meta = write_pack_layout_dump(
            layout_dir, self._scene_config, self._placed_records,
            termination=termination,
            extra_meta={
                "last_rejected": self._last_rejected,
                "surface_2d": bool(self._surface_2d),
            })
        if self._surface_2d:
            self._dump_json(layout_dir, "surface_2d.json", self._surface_2d)
        cycles = [entry.get("cycle_wall_sec") for entry in committed
                  if entry.get("cycle_wall_sec") is not None]
        import statistics
        suite = {
            "termination_reason": termination,
            "capacity_claim_valid": (
                termination == "BIN_FULL"
                and not self._args.skip_unplaceable
                and len(set(ids)) == 1),
            "boxes_packed": len(committed),
            "boxes_attempted": attempted,
            "volume_fraction": round(volume_sum / INNER_VOLUME, 4),
            "floor_coverage": round(footprint / INNER_FLOOR_XY, 4),
            "inner_volume_m3": round(INNER_VOLUME, 3),
            "packed_volume_m3": round(volume_sum, 4),
            "catalog_counts": catalog_counts,
            "cycle_sec": {
                "n": len(cycles),
                "mean": round(statistics.mean(cycles), 2) if cycles else None,
                "p50": round(statistics.median(cycles), 2) if cycles
                       else None,
                "max": round(max(cycles), 2) if cycles else None,
            },
            "wall_total_sec": round(wall_total, 1),
            "boxes_per_min": round(
                len(committed) / (wall_total / 60.0), 3) if wall_total
            else 0.0,
            "last_rejected": self._last_rejected,
            "final_layout": "final_layout",
            "final_layout_meta": layout_meta,
            "goto_failed_count": sum(
                1 for entry in self._ledger
                if "GOTO" in str(entry.get("fail_code", ""))),
            "rtf_last": last_rtf,
            "skip_unplaceable": self._args.skip_unplaceable,
        }
        with open(os.path.join(self._out, "suite.json"), "w",
                  encoding="utf-8") as handle:
            json.dump(suite, handle, indent=2, sort_keys=True)
        print(json.dumps(suite, indent=2, sort_keys=True))


def merge_args(argv=None):
    pack = argparse.ArgumentParser(description=__doc__)
    pack.add_argument("--out", required=True)
    pack.add_argument("--sequence-ids", default="carryon")
    pack.add_argument("--max-boxes", type=int, default=50)
    pack.add_argument("--skip-unplaceable", action="store_true")
    pack.add_argument("--seed", type=int, default=0)
    pack.add_argument("--n", type=int, default=None,
                      help="alias for --max-boxes")
    pack.add_argument("--observe-pose", default="pickup_observe")
    pack.add_argument("--goto-timeout", type=float, default=60.0)
    pack.add_argument("--skip-graph-check", action="store_true")
    known, rest = pack.parse_known_args(argv)
    smoke = parse_place_args([
        "--out", known.out,
        "--n", str(known.n if known.n is not None else known.max_boxes),
        "--goto-timeout", str(known.goto_timeout),
        "--observe-pose", known.observe_pose,
        "--payload", "vacuum",
        "--geometry-timeout", "25",
    ] + rest)
    for key, value in vars(known).items():
        setattr(smoke, key, value)
    if smoke.n is not None and known.n is None:
        pass
    if known.n is not None:
        smoke.max_boxes = known.n
    smoke.dump_dir = os.path.join(known.out, "dumps")
    smoke.dump_exact = True
    smoke.use_vacuum = True
    smoke.payload = "vacuum"
    smoke.visual_kind = getattr(smoke, "visual_kind", "mesh")
    return smoke


def main(argv=None):
    args = merge_args(argv)
    os.makedirs(args.out, exist_ok=True)
    os.makedirs(args.dump_dir, exist_ok=True)
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init()
    driver = PackEvalDriver(args)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(driver)
    import threading
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        driver.run()
    finally:
        driver.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
