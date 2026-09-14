#!/usr/bin/env python3
"""Place-only perfect-geometry eval driver (POS-1).

Drives the production place/map chain from a deterministic carry start:
no perception, no DetectLuggage, no pick segments. The eval harness spawns
the catalog cargo at the pickup fixture, positions the suction frame on the
box top centre (<=5 mm/axis), enables vacuum, then hands an exact
``DetectedLuggage`` (``input_source=eval_perfect_geometry``) to the ordinary
``ComputePlacement`` and ``BuildMotionSequence`` service contracts and
executes the production place sequence through ``PlanMotion``.

Eval-only: nothing in this file is imported by production nodes. Fixtures
and independent cross-checks live in ``luggage_gazebo.place_only_fixture``.

Case matrix, acceptance and dump contract:
``docs/plans/place_only_perfect_geometry_sim.md``.
"""

from __future__ import division

import argparse
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time

_SCRIPTS = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)
_SRC_ROOT = os.path.normpath(os.path.join(_SCRIPTS, "..", "..", ".."))
if _SRC_ROOT not in sys.path:
    sys.path.insert(0, _SRC_ROOT)

from place_smoke_driver import (  # noqa: E402
    PlaceSmokeDriver,
    _yaw_quat,
    parse_args as parse_place_args,
)
from luggage_gazebo import place_only_fixture as fx  # noqa: E402
from luggage_gazebo.place_metrics import (  # noqa: E402
    PlaceTrial,
    place_ok,
    trial_to_dict,
)

from geometry_msgs.msg import Pose as PoseMsg  # noqa: E402
from luggage_msgs.msg import DetectedLuggage, MotionSegment, SlotSpec  # noqa: E402
from luggage_msgs.action import PlanMotion  # noqa: E402
from luggage_msgs.srv import (  # noqa: E402
    AddPlacedBox,
    ClearCurrentBox,
    ComputePlacement,
    DetectLuggage,
    FinalizeCurrentBox,
    GetCargoMapStats,
    GetCurrentBox,
    RemovePlacedBox,
    ResetCargoMap,
    SpawnNextBox,
)
from ros_gz_interfaces.msg import Entity  # noqa: E402
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from luggage_description.box_catalog_utils import (  # noqa: E402
    box_catalog_entries,
    box_catalog_path_from_scene,
    load_box_catalog,
)
from luggage_description.scene_tf_config_utils import (  # noqa: E402
    _local_point_to_base_link,
    _point_in_container_link,
    container_inner_floor_z,
    origin_in_world,
    xyz_base_link_to_world,
    xyz_world_to_base_link,
    yaw_base_link_to_world,
    yaw_world_to_base_link,
)

WORLD_NAME = "airport_loading"

# Scored place-chain segment names (waypoint_generator phase="place").
PLACE_SEGMENT_NAMES = (
    "transit", "traverse", "insert", "descend", "retreat",
    "stage", "stage_mid", "stage_late",
)
# Fixture-setup segments: not scored as pick or place.
SETUP_SEGMENT_NAMES = ("setup_pre_over_box", "setup_attach")

PERCEPTION_NODES = (
    "luggage_detector", "semantic_segmenter", "semantic_point_filter",
    "sensor_preprocessor", "depth_image_republisher",
)
REQUIRED_NODES = (
    "pickup_box_spawner", "waypoint_generator", "motion_planner",
    "move_group", "scene_manager", "cargo_volume_mapper",
    "placement_planner", "vacuum_controller",
    # observe_pose_hold is a one-shot: it publishes the initial arm pose and
    # exits, so it must not be a graph-presence requirement.
)

# Dedicated model prefix so fixture teardown never touches pickup cargo.
FIXTURE_MODEL_PREFIX = "pos_fixture_"


def _ign_model_bin():
    for name in ("ign", "gz"):
        path = shutil.which(name)
        if path and os.path.basename(path) in ("ign", "gz"):
            return path
    return "ign"


def _fixture_box_sdf(model_name, size, mass_kg=20.0):
    """Primitive-box SDF for declared fixture obstacles (eval-only).

    Fixture boxes are never perceived or picked; the catalog mesh-only rule
    applies to pickup cargo only.
    """
    w, d, h = (float(v) for v in size)
    return (
        '<?xml version="1.0" ?>'
        '<sdf version="1.8">'
        '<model name="%s"><static>false</static>'
        '<link name="link">'
        '<inertial><mass>%.3f</mass><inertia>'
        '<ixx>%.6f</ixx><iyy>%.6f</iyy><izz>%.6f</izz>'
        '<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>'
        '</inertia></inertial>'
        '<collision name="col"><geometry><box><size>%.4f %.4f %.4f</size>'
        '</box></geometry></collision>'
        '<visual name="vis"><geometry><box><size>%.4f %.4f %.4f</size>'
        '</box></geometry>'
        '<material><ambient>0.75 0.2 0.2 1</ambient>'
        '<diffuse>0.75 0.2 0.2 1</diffuse></material></visual>'
        '</link></model></sdf>'
    ) % (
        model_name, mass_kg,
        mass_kg * (d * d + h * h) / 12.0,
        mass_kg * (w * w + h * h) / 12.0,
        mass_kg * (w * w + d * d) / 12.0,
        w, d, h, w, d, h,
    )


def _rtf_violation_windows(trace, threshold=0.8, max_gap_sec=5.0):
    """Windows where RTF stayed below threshold for > max_gap wall seconds."""
    windows = []
    start = None
    last = None
    for sample in trace:
        below = float(sample["rtf"]) < threshold
        t = float(sample["t_wall"])
        if below:
            if start is None:
                start = t
            last = t
        else:
            if start is not None and last - start > max_gap_sec:
                windows.append(
                    {"start": start, "end": last,
                     "duration_sec": round(last - start, 2)})
            start = None
    if start is not None and last - start > max_gap_sec:
        windows.append(
            {"start": start, "end": last,
             "duration_sec": round(last - start, 2)})
    return windows


def box_size_of(box_msg):
    return (float(box_msg.width), float(box_msg.depth), float(box_msg.height))


def _git_head():
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True,
            cwd=_SRC_ROOT, timeout=5.0).strip()
    except (OSError, subprocess.SubprocessError):
        return ""


class PlaceOnlyDriver(PlaceSmokeDriver):

    def __init__(self, args):
        super().__init__(args)
        self._streak_out = args.out
        self._dumps = os.path.join(args.out, "dumps")
        os.makedirs(self._dumps, exist_ok=True)

        self._compute_cli = self.create_client(
            ComputePlacement, "/placement_planner/compute_placement",
            callback_group=self._group)
        self._finalize = self.create_client(
            FinalizeCurrentBox, "/pickup_box_spawner/finalize_current_box",
            callback_group=self._group)
        self._map_add = self.create_client(
            AddPlacedBox, "/cargo_map/add_placed_box",
            callback_group=self._group)
        self._map_remove = self.create_client(
            RemovePlacedBox, "/cargo_map/remove_placed_box",
            callback_group=self._group)
        self._map_reset = self.create_client(
            ResetCargoMap, "/cargo_map/reset", callback_group=self._group)
        self._map_stats_cli = self.create_client(
            GetCargoMapStats, "/cargo_map/get_stats",
            callback_group=self._group)
        self._gz_create = self.create_client(
            SpawnEntity, "/world/%s/create" % WORLD_NAME,
            callback_group=self._group)
        self._gz_delete = self.create_client(
            DeleteEntity, "/world/%s/remove" % WORLD_NAME,
            callback_group=self._group)

        latch = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._surface_2d = {}
        self._committed_topic = {}
        self._last_result = {}
        self._finalized_models = []
        self.create_subscription(
            String, "/luggage/cargo_map/surface_2d", self._on_surface, latch,
            callback_group=self._group)
        self.create_subscription(
            String, "/luggage/finalized_models", self._on_finalized, latch,
            callback_group=self._group)
        self.create_subscription(
            String, "/luggage/cargo_map/committed", self._on_committed, latch,
            callback_group=self._group)
        self.create_subscription(
            String, "/placement_planner/last_result", self._on_last_result,
            latch, callback_group=self._group)

        # Fixture geometry context (pure kernel + scene config).
        scene = self._scene_config
        self._hull = fx.hull_from_scene_config(scene)
        self._floor_z = container_inner_floor_z(scene)
        self._ctx = fx.hull_context(self._hull, self._floor_z)
        # Match the place-only profile's planner hull_margin (lateral only).
        self._ctx["lateral_margin"] = 0.010
        catalog = load_box_catalog(box_catalog_path_from_scene(scene))
        self._catalog_sizes = {
            str(entry["id"]): tuple(float(v) for v in entry["size"])
            for entry in box_catalog_entries(catalog)}
        self._cases = {
            case.case_id: case for case in fx.case_matrix(self._catalog_sizes)}

        # Guards and bookkeeping.
        self._pos_guards = {
            "detect_calls": 0,
            "setup_goals": [],
            "executed_goals": [],
            "clock_publisher_samples": [],
            "rtf_trace": [],
            "rss_mib_trace": [],
        }
        self._commit_seq = 0
        self._placed_slots = []        # base_link SlotSpec list for requests
        self._placed_local_boxes = []  # map-committed (local, floor-relative)
        self._blind_obstacles = []     # planner-blind fixture obstacles
        self._fixture_scene_slots = []  # every fixture scene object, teardown
        self._fixture_models = []      # gz model names installed this streak
        self._deleted_finalized = set()  # finalized models already removed
        self._case_t1 = []
        self._segment_sink = {}
        self._segment_samples = []
        self._case_dir = ""
        self._current_case_id = ""
        self._scoring_active = False

    # ------------------------------------------------------------------
    # Subscriptions / low-level helpers

    def _on_surface(self, msg):
        try:
            self._surface_2d = json.loads(msg.data)
        except ValueError:
            pass

    def _on_committed(self, msg):
        try:
            self._committed_topic = json.loads(msg.data)
        except ValueError:
            pass

    def _on_last_result(self, msg):
        try:
            self._last_result = json.loads(msg.data)
        except ValueError:
            pass

    def _on_finalized(self, msg):
        try:
            payload = json.loads(msg.data)
            if isinstance(payload, list):
                self._finalized_models = [
                    str(entry.get("model_name") or "")
                    for entry in payload if entry.get("model_name")]
        except (ValueError, AttributeError):
            pass

    def call_srv(self, client, request, timeout):
        # Detect guard: any DetectLuggage call is an isolation violation.
        if isinstance(request, DetectLuggage.Request):
            self._pos_guards["detect_calls"] += 1
        return super().call_srv(client, request, timeout)

    def _segment_profile(self, segment_name, sink, stop_flag):
        """10 Hz in-segment sampler: panel TF, box pose, joint states.

        Before/after sampling cannot localize a mid-path stall; this
        records the divergence profile (which joints freeze, where the
        tool stops) for the boundary dump."""
        import rclpy
        from sensor_msgs.msg import JointState
        sub = self.create_subscription(
            JointState, "/joint_states",
            lambda m: sink.__setitem__("joints", list(m.position)),
            10, callback_group=self._group)
        try:
            while not stop_flag["flag"] and rclpy.ok():
                suction, _err = self.suction_xyz()
                sample = {
                    "t": round(self.ros_now_sec(), 3),
                    "suction": [round(v, 4) for v in suction]
                    if suction else None,
                    # No gz CLI queries here: each `ign model --pose` is a
                    # subprocess whose cost at 10 Hz starves the vacuum
                    # follow's set_pose service (observed: attach lost to
                    # "set_pose timeout"). Box poses stay at the
                    # before/after segment samples.
                    "joints": [round(v, 4) for v in (sink.get("joints") or [])],
                }
                self._segment_samples.append(sample)
                time.sleep(0.1)
        finally:
            self.destroy_subscription(sub)

    def _execute_segment(self, segment, trial):
        # Motion guard: record every executed segment name; pick-phase
        # names must never appear.
        self._pos_guards["executed_goals"].append(
            {"case": self._current_case_id, "name": str(segment.name),
             "scored": bool(self._scoring_active)})
        self._segment_samples = []
        stop_flag = {"flag": False}
        sampler = threading.Thread(
            target=self._segment_profile,
            args=(str(segment.name), self._segment_sink, stop_flag),
            daemon=True)
        sampler.start()
        try:
            result = super()._execute_segment(segment, trial)
        finally:
            stop_flag["flag"] = True
            sampler.join(timeout=1.0)
            if self._segment_samples:
                path = os.path.join(
                    self._case_dir or ".",
                    "segment_%s_profile.jsonl" % segment.name)
                try:
                    with open(path, "w", encoding="utf-8") as handle:
                        for row in self._segment_samples:
                            handle.write(json.dumps(
                                row, sort_keys=True, default=str) + "\n")
                except OSError:
                    pass
        ok, code = result[0], result[1]
        if ok or code in ("RELEASE_SETTLE_FAILED", "PLACE_LOST_PAYLOAD"):
            return result
        # One replanned retry from the current state: a controller goal-time
        # abort can leave the arm mid-path at a valid pose (observed: joint1
        # stall with the tool 0.25 m short of the traverse target). A hard
        # block fails the retry identically and the case aborts with
        # evidence; the retry is recorded in the segments log.
        retry_ok, retry_code, retry_rec = super()._execute_segment(
            segment, trial)
        if retry_rec is not None:
            retry_rec = dict(retry_rec)
            retry_rec["retry_of"] = str(segment.name)
            self._segments_log.append(retry_rec)
        self._t1("segment_retry", segment=str(segment.name),
                 first_code=code, retry_ok=bool(retry_ok),
                 retry_code=retry_code)
        if retry_ok:
            return True, "", retry_rec
        return False, retry_code, retry_rec

    def _check_i1(self):
        """Tolerant payload check: a single failed follow tick (gz set_pose
        timeout under load) leaves a sticky fail_reason in the vacuum state
        while the attachment itself is intact. Only a real detach, or a
        fail_reason that persists across retries, is a lost payload."""
        if not getattr(self._args, "use_vacuum", False):
            return True, ""
        last = {}
        for _ in range(3):
            last = dict(self._vacuum_state or {})
            if last.get("attached") and not last.get("fail_reason"):
                return True, ""
            if not last.get("attached"):
                return False, "PLACE_LOST_PAYLOAD"
            time.sleep(0.5)
        if last.get("attached") and last.get("fail_reason"):
            return False, "PLACE_LOST_PAYLOAD:%s" % last.get("fail_reason")
        return False, "PLACE_LOST_PAYLOAD"

    def graph_error(self):
        for label, pattern in (
                ("move_group", "moveit_ros_move_group/move_group"),
                ("pickup_box_spawner",
                 "lib/luggage_gazebo/pickup_box_spawner_node.py"),
                ("motion_planner",
                 "lib/luggage_planning/motion_planner_node.py")):
            if self._proc_count(pattern) > 1:
                return "duplicate process %s; residual graph" % label
        names = set(self.get_node_names())
        missing = [n for n in REQUIRED_NODES if n not in names]
        if missing:
            return "missing nodes: %s" % ", ".join(missing)
        present = [n for n in PERCEPTION_NODES if n in names]
        if present:
            return "perception nodes must be absent: %s" % ", ".join(present)
        if self._proc_count("ros_gz_bridge/parameter_bridge") > 3:
            return "duplicate parameter_bridge; residual graph"
        return ""

    def _t1(self, event, **fields):
        row = dict(fields)
        row.update({
            "event": event,
            "t_wall": time.time(),
            "t_ros": self.ros_now_sec(),
        })
        self._case_t1.append(row)
        return row

    def _clock_publishers(self):
        try:
            out = subprocess.check_output(
                ["ros2", "topic", "info", "/clock"],
                text=True, timeout=8.0, stderr=subprocess.DEVNULL)
            head = out.split("Publisher count:")[1].split("\n", 1)[0]
            return int(head.strip())
        except (OSError, subprocess.SubprocessError, ValueError, IndexError):
            return -1

    def _wait_surface_revision(self, target_revision, timeout=10.0):
        """Block until the latched surface topic reflects the given revision
        (the planner consumes the same topic, so ComputePlacement must not
        race ahead of it)."""
        deadline = time.time() + float(timeout)
        last = None
        while time.time() < deadline:
            surface = self._surface_2d or {}
            last = surface.get("map_revision")
            if last is not None and int(last) == int(target_revision):
                return True
            time.sleep(0.05)
        self.get_logger().warning(
            "surface revision did not reach %s (last=%s)"
            % (target_revision, last))
        return False

    def _sample_clock_guard(self):
        count = self._clock_publishers()
        self._pos_guards["clock_publisher_samples"].append(
            {"t_wall": time.time(), "count": count})
        return count

    def _sample_rtf(self):
        try:
            result = subprocess.run(
                [_ign_model_bin(), "topic", "-e", "-t",
                 "/world/%s/stats" % WORLD_NAME, "--num", "1"],
                capture_output=True, text=True, timeout=5)
            for line in result.stdout.splitlines():
                if "real_time_factor" in line:
                    return float(line.split(":")[1])
        except (OSError, ValueError, subprocess.TimeoutExpired):
            pass
        return None

    def _sample_resources(self):
        sample = {"t_wall": time.time(), "rss_mib": {}}
        for label, pattern in (
                ("driver", "place_only_eval_driver.py"),
                ("gz", "ign gazebo"),
                ("move_group", "moveit_ros_move_group/move_group")):
            try:
                out = subprocess.check_output(
                    ["pgrep", "-f", pattern], text=True, timeout=4.0)
                pids = [int(v) for v in out.split()]
            except (OSError, subprocess.CalledProcessError, ValueError):
                continue
            total = 0.0
            for pid in pids[:4]:
                try:
                    with open("/proc/%d/status" % pid, "r",
                              encoding="utf-8") as handle:
                        for line in handle:
                            if line.startswith("VmRSS:"):
                                total += float(line.split()[1]) / 1024.0
                                break
                except OSError:
                    continue
            sample["rss_mib"][label] = round(total, 1)
        self._pos_guards["rss_mib_trace"].append(sample)
        return sample

    def _rtf_tick(self, stage):
        rtf = self._sample_rtf()
        if rtf is not None:
            self._pos_guards["rtf_trace"].append(
                {"t_wall": time.time(), "stage": stage, "rtf": rtf})
        return rtf

    def _call(self, client, request, timeout=30.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        return self.call_srv(client, request, timeout=timeout)

    # ------------------------------------------------------------------
    # Frame conversions (container_link, floor-relative <-> world/base_link)

    def _container_yaw(self):
        _origin, rpy = origin_in_world(self._scene_config)
        return float(rpy[2])

    def _world_to_local(self, xyz_world):
        base = xyz_world_to_base_link(self._scene_config, list(xyz_world))
        local = _point_in_container_link(base, self._scene_config)
        return [float(local[0]), float(local[1]),
                float(local[2]) - self._floor_z]

    def _local_to_world(self, xyz_local):
        absolute = [float(xyz_local[0]), float(xyz_local[1]),
                    float(xyz_local[2]) + self._floor_z]
        base = _local_point_to_base_link(absolute, self._scene_config)
        return xyz_base_link_to_world(self._scene_config, base)

    def _local_to_base(self, xyz_local, yaw_local):
        absolute = [float(xyz_local[0]), float(xyz_local[1]),
                    float(xyz_local[2]) + self._floor_z]
        base = _local_point_to_base_link(absolute, self._scene_config)
        return base, yaw_world_to_base_link(
            self._scene_config, yaw_local + self._container_yaw())

    def _slot_local(self, slot):
        """(center_local_floor_rel, yaw_local, size) of a base_link slot."""
        pos = slot.place_pose.position
        base_xyz = [float(pos.x), float(pos.y), float(pos.z)]
        yaw_base = 2.0 * math.atan2(
            slot.place_pose.orientation.z, slot.place_pose.orientation.w)
        yaw_world = yaw_base_link_to_world(self._scene_config, yaw_base)
        yaw_local = yaw_world - self._container_yaw()
        local = _point_in_container_link(base_xyz, self._scene_config)
        center = [float(local[0]), float(local[1]),
                  float(local[2]) - self._floor_z]
        return center, yaw_local, [
            float(slot.width), float(slot.depth), float(slot.height)]

    def _slot_meta(self, slot):
        center, yaw_local, size = self._slot_local(slot)
        base_xyz, yaw_base = self._local_to_base(center, yaw_local)
        return {
            "planning_frame": "world",
            "pose_world": {
                "position": self._local_to_world(center),
                "yaw": yaw_local + self._container_yaw(),
            },
            "pose_base_link": {
                "position": base_xyz, "yaw": yaw_base,
            },
            "center_local_floor_relative": center,
            "yaw_local": yaw_local,
            "size_wdh": size,
            "source": "compute_placement",
        }

    def _slot_world_msg(self, slot, slot_meta, unique_col=None):
        if unique_col is None:
            # The base place chain calls this without a column; give each
            # committed box a unique scene-object id (ComputePlacement
            # returns 0/0/0 for every slot, which would make later boxes
            # overwrite earlier collision objects).
            unique_col = self._commit_seq + 1
        out = SlotSpec()
        out.layer, out.row, out.col = 0, 0, int(unique_col)
        out.width, out.depth, out.height = (
            float(slot.width), float(slot.depth), float(slot.height))
        pos = slot_meta["pose_world"]["position"]
        out.place_pose.position.x = float(pos[0])
        out.place_pose.position.y = float(pos[1])
        out.place_pose.position.z = float(pos[2])
        out.place_pose.orientation = _yaw_quat(slot_meta["pose_world"]["yaw"])
        return out

    # ------------------------------------------------------------------
    # Cargo-map state helpers

    def _map_stats(self):
        resp = self._call(
            self._map_stats_cli, GetCargoMapStats.Request(), timeout=10.0)
        if resp is None:
            return None
        committed = 0
        try:
            committed = int(str(resp.message).split("committed=")[1])
        except (IndexError, ValueError):
            committed = len((self._committed_topic or {}).get("boxes") or [])
        return {
            "map_revision": int(resp.map_revision),
            "committed_box_count": committed,
            "occupied_count": int(resp.occupied_count),
            "unknown_ratio": float(resp.unknown_ratio),
        }

    def _map_snapshot(self):
        stats = self._map_stats()
        surface = dict(self._surface_2d or {})
        return {
            "stats": stats,
            "surface": surface,
            "digest": fx.surface_digest(surface) if surface else "",
            "ledger": list((self._committed_topic or {}).get("boxes") or []),
        }

    # ------------------------------------------------------------------
    # Evidence dumping

    def _dump_json(self, name, payload):
        if not self._case_dir:
            return
        path = os.path.join(self._case_dir, name)
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True,
                      default=str)
            handle.write("\n")

    def _flush_t1(self):
        if not self._case_dir:
            return
        path = os.path.join(self._case_dir, "t1_trace.jsonl")
        with open(path, "a", encoding="utf-8") as handle:
            for row in self._case_t1:
                handle.write(
                    json.dumps(row, sort_keys=True, default=str) + "\n")
        self._case_t1 = []

    # ------------------------------------------------------------------
    # Fixture installation / teardown

    def _delete_gz_model(self, name):
        req = DeleteEntity.Request()
        req.entity.name = str(name)
        req.entity.type = Entity.MODEL
        resp = self._call(self._gz_delete, req, timeout=10.0)
        return bool(resp and resp.success)

    def _teardown_world_to_empty(self, reason):
        """Restore the declared empty-container fixture: remove placed cargo
        models (finalized suitcases), fixture models, scene placed objects,
        and reset the cargo map. Never touches the robot."""
        removed = []
        targets = list(self._fixture_models)
        for name in self._finalized_models:
            if name not in self._deleted_finalized:
                targets.append(name)
                self._deleted_finalized.add(name)
        for name in targets:
            ok = self._delete_gz_model(name)
            removed.append({"model": name, "ok": ok})
        self._fixture_models = []
        for slot in list(self._placed_slots) + list(self._fixture_scene_slots):
            self._call(self._remove_placed, RemovePlacedBox.Request(slot=slot),
                       timeout=10.0)
        self._placed_slots = []
        self._placed_local_boxes = []
        self._blind_obstacles = []
        self._fixture_scene_slots = []
        reset = self._call(self._map_reset, ResetCargoMap.Request(),
                           timeout=10.0)
        self._commit_seq = 0
        self._t1("fixture_teardown", reason=reason, removed=removed,
                 reset=bool(reset and reset.success))

    def _fixture_base_slot(self, box, col):
        """base_link SlotSpec for a declared fixture box (for requests)."""
        base_xyz, yaw_base = self._local_to_base(box.center, box.yaw)
        slot = SlotSpec()
        slot.layer, slot.row, slot.col = 0, 0, int(col)
        slot.width, slot.depth, slot.height = (
            float(box.size[0]), float(box.size[1]), float(box.size[2]))
        slot.place_pose.position.x = float(base_xyz[0])
        slot.place_pose.position.y = float(base_xyz[1])
        slot.place_pose.position.z = float(base_xyz[2])
        slot.place_pose.orientation = _yaw_quat(yaw_base)
        return slot

    def _fixture_world_slot(self, box, col):
        world_xyz = self._local_to_world(box.center)
        slot = SlotSpec()
        slot.layer, slot.row, slot.col = 0, 0, int(col)
        slot.width, slot.depth, slot.height = (
            float(box.size[0]), float(box.size[1]), float(box.size[2]))
        slot.place_pose.position.x = float(world_xyz[0])
        slot.place_pose.position.y = float(world_xyz[1])
        slot.place_pose.position.z = float(world_xyz[2])
        slot.place_pose.orientation = _yaw_quat(
            box.yaw + self._container_yaw())
        return slot

    def _install_fixture_boxes(self, boxes, spawn_physical=True):
        """Install the declared serialized fixture: production cargo-map
        commits (for in_map boxes) + scene_manager collision objects for
        all declared boxes (+ physical models in scored mode so Gazebo
        matches the fixture). Planner-blind obstacles skip the map."""
        installed = []
        for index, box in enumerate(boxes):
            slot = self._fixture_world_slot(box, 900 + index)
            map_resp = None
            if box.in_map:
                map_resp = self._call(
                    self._map_add, AddPlacedBox.Request(slot=slot),
                    timeout=15.0)
            scene_resp = self._call(
                self._add_placed, AddPlacedBox.Request(slot=slot),
                timeout=15.0)
            ok = bool(scene_resp and scene_resp.success
                      and (not box.in_map
                           or (map_resp and map_resp.success)))
            record = {
                "role": box.role,
                "record": box.record(),
                "map_message": (map_resp.message if map_resp
                                else ("skipped: planner-blind obstacle"
                                      if not box.in_map else "timeout")),
                "scene_message": scene_resp.message if scene_resp
                else "timeout",
                "ok": ok,
            }
            if ok and spawn_physical:
                model = "%s%d" % (FIXTURE_MODEL_PREFIX, index)
                spawn_req = SpawnEntity.Request()
                spawn_req.entity_factory.name = model
                spawn_req.entity_factory.sdf = _fixture_box_sdf(
                    model, box.size)
                pose = PoseMsg()
                pose.position.x = float(slot.place_pose.position.x)
                pose.position.y = float(slot.place_pose.position.y)
                pose.position.z = float(slot.place_pose.position.z)
                pose.orientation = slot.place_pose.orientation
                spawn_req.entity_factory.pose = pose
                gz_resp = self._call(self._gz_create, spawn_req, timeout=15.0)
                record["gz_model"] = model
                # SpawnEntity.Response carries only `success`.
                record["gz_message"] = (
                    "spawned" if (gz_resp and gz_resp.success)
                    else ("timeout" if gz_resp is None else "spawn_failed"))
                record["ok"] = bool(gz_resp and gz_resp.success)
                if record["ok"]:
                    self._fixture_models.append(model)
            installed.append(record)
            self._t1("fixture_install", index=index, role=box.role,
                     in_map=box.in_map, ok=record["ok"])
            if not record["ok"]:
                return False, installed
        time.sleep(0.5)
        for index, box in enumerate(boxes):
            if box.in_map:
                self._placed_local_boxes.append(box)
                self._placed_slots.append(
                    self._fixture_base_slot(box, 900 + index))
            else:
                self._blind_obstacles.append(box)
            self._fixture_scene_slots.append(
                self._fixture_base_slot(box, 900 + index))
        return True, installed

    def _fixture_for(self, case):
        if case.initial == "saturated":
            return fx.saturated_fixture_boxes(
                self._ctx, self._catalog_sizes[case.cargo_id])
        if case.initial == "obstacle":
            return fx.obstacle_fixture_boxes(
                self._ctx, self._catalog_sizes[case.cargo_id])
        return []

    # ------------------------------------------------------------------
    # Carry setup (eval fixture; not scored as pick)

    def _perfect_luggage_msg(self, case, box_msg):
        size = self._catalog_sizes[case.cargo_id]
        yaw = math.atan2(
            2.0 * (box_msg.pose.orientation.w * box_msg.pose.orientation.z
                   + box_msg.pose.orientation.x * box_msg.pose.orientation.y),
            1.0 - 2.0 * (box_msg.pose.orientation.y ** 2
                         + box_msg.pose.orientation.z ** 2))
        position = [box_msg.pose.position.x, box_msg.pose.position.y,
                    box_msg.pose.position.z]
        fields = fx.perfect_descriptor_fields(
            case.cargo_id, size, position, yaw)
        msg = DetectedLuggage()
        msg.id = fields["id"]
        msg.pose = PoseMsg()
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        msg.pose.orientation = box_msg.pose.orientation
        msg.width, msg.depth, msg.height = (
            size[0], size[1], size[2])
        msg.yaw_valid = True
        msg.aspect_ratio = fields["aspect_ratio"]
        msg.height_valid = True
        msg.height_confidence = fields["height_confidence"]
        msg.height_source = fields["height_source"]
        msg.top_surface_valid = True
        msg.top_surface_confidence = fields["top_surface_confidence"]
        msg.top_surface_pose = PoseMsg()
        msg.top_surface_pose.position.x = position[0]
        msg.top_surface_pose.position.y = position[1]
        msg.top_surface_pose.position.z = fields["top_surface_z"]
        msg.top_surface_pose.orientation = box_msg.pose.orientation
        return msg, fields

    def _tool_down_pose(self, xyz, yaw):
        pose = PoseMsg()
        pose.position.x, pose.position.y, pose.position.z = (
            float(xyz[0]), float(xyz[1]), float(xyz[2]))
        half = float(yaw) * 0.5
        pose.orientation.x = math.cos(half)
        pose.orientation.y = math.sin(half)
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
        return pose

    def _send_setup_segment(self, name, target_pose, cartesian):
        segment = MotionSegment()
        segment.name = name
        segment.type = "cartesian" if cartesian else "pose_target"
        segment.target_pose = target_pose
        segment.allow_ompl_fallback = True
        self._pos_guards["setup_goals"].append(
            {"case": self._current_case_id, "name": name})
        ok, message, _result = self.send_action(
            self._plan, PlanMotion.Goal(segment=segment),
            timeout=self._args.plan_timeout, name="PlanMotion:%s" % name)
        self._pos_guards["setup_goals"][-1].update(
            {"ok": bool(ok), "message": message})
        self._t1("setup_segment", name=name, ok=bool(ok), message=message)
        return ok, message

    def _setup_carry(self, case):
        """Spawn cargo, park the suction frame on the box top centre
        (<=5 mm per axis), enable vacuum, verify simulator attachment plus
        exactly one matching PlanningScene attached object."""
        spawn = self._call(self._spawn, SpawnNextBox.Request(), timeout=30.0)
        if spawn is None or not spawn.success:
            return None, "FIXTURE_SETUP_FAILED:spawn:%s" % (
                spawn.message if spawn else "timeout")
        current = self._call(
            self._current, GetCurrentBox.Request(), timeout=10.0)
        if current is None or not current.success:
            return None, "FIXTURE_SETUP_FAILED:current_box_unavailable"
        box_msg = current.box
        size = self._catalog_sizes[case.cargo_id]
        # GetCurrentBox reports the mesh's OBSERVABLE geometry (lid-plane
        # reference for perception evals), not the physical AABB. The
        # physics/collision AABB equals the catalog size, so the manifest
        # and the perfect descriptor keep the catalog size; here we verify
        # the spawned catalog identity and record the observable size.
        box_identity = str(getattr(box_msg, "id", "") or "")
        if case.cargo_id not in box_identity:
            return None, (
                "FIXTURE_SETUP_FAILED:catalog_mismatch:expected %s got %r"
                % (case.cargo_id, box_identity))
        self._t1("fixture_cargo_sizes",
                 catalog_id=case.cargo_id,
                 manifest_aabb=[round(v, 4) for v in size],
                 observable=[round(v, 4) for v in box_size_of(box_msg)])
        self._box_model = str(box_msg.id or "")
        pick_msg, fields = self._perfect_luggage_msg(case, box_msg)
        self._t1("fixture_cargo_spawned", model=self._box_model,
                 declared=fields["size_wdh"], spawn_id=box_msg.id)

        box_top_z = fields["top_surface_z"]
        box_yaw = fields["yaw"]
        box_xy = (fields["pose_position"][0], fields["pose_position"][1])
        pre = self._tool_down_pose(
            (box_xy[0], box_xy[1], box_top_z + 0.30), box_yaw)
        attach = self._tool_down_pose(
            (box_xy[0], box_xy[1], box_top_z), box_yaw)
        ok, message = self._send_setup_segment(
            "setup_pre_over_box", pre, cartesian=False)
        if not ok:
            return None, "FIXTURE_SETUP_FAILED:setup_pre:%s" % message
        ok, message = self._send_setup_segment(
            "setup_attach", attach, cartesian=True)
        if not ok:
            return None, "FIXTURE_SETUP_FAILED:setup_attach:%s" % message

        suction, err = self.suction_xyz()
        if suction is None:
            return None, "FIXTURE_SETUP_FAILED:suction_tf:%s" % err
        errors = {
            "x": suction[0] - box_xy[0],
            "y": suction[1] - box_xy[1],
            "z": suction[2] - box_top_z,
        }
        self._t1("setup_tool_error_m",
                 **{k: round(v, 5) for k, v in errors.items()})
        if any(abs(v) > float(self._args.tool_tol) for v in errors.values()):
            return None, "FIXTURE_SETUP_FAILED:tool_offset:%s" % json.dumps(
                {k: round(v, 5) for k, v in errors.items()})

        vac_ok, vac_msg = self.vacuum_command(True)
        if not vac_ok:
            return None, "FIXTURE_SETUP_FAILED:vacuum:%s" % vac_msg
        time.sleep(0.5)
        state = dict(self._vacuum_state or {})
        scene = self._scene_snapshot()
        attached_objects = [a for a in scene.get("attached", [])
                            if str(a) == "pickup_box"]
        self._t1("setup_attach_verified", vacuum_state=state,
                 attached_objects=scene.get("attached"),
                 box_gz=self._gz_box_pose())
        if not (state.get("attached") and not state.get("fail_reason")):
            return None, "FIXTURE_SETUP_FAILED:sim_attach_state:%s" % (
                state.get("fail_reason") or "not attached")
        if len(attached_objects) != 1:
            return None, "FIXTURE_SETUP_FAILED:scene_attach:%r" % (
                scene.get("attached"),)
        return (pick_msg, fields, errors), ""

    def _recover_carried(self, note, release_in_place=False):
        """Explicit recovery for a still-attached payload.

        ``release_in_place`` releases while the box still rests on the
        pickup platform (planning failed before any place motion), so the
        box is never lifted and dropped. Otherwise keep vacuum until the arm
        is home, then release and clear.
        """
        recovered = {"note": note, "release_in_place": bool(release_in_place)}
        if release_in_place:
            vac_ok, vac_msg = self.vacuum_command(False)
            recovered["vacuum_off"] = "%s:%s" % (int(bool(vac_ok)), vac_msg)
        try:
            goto_ok, goto_msg, _ = self._home_arm()
            recovered["home"] = goto_msg
        except Exception as exc:  # noqa: BLE001 - recovery boundary
            recovered["home_error"] = str(exc)
        if not release_in_place:
            vac_ok, vac_msg = self.vacuum_command(False)
            recovered["vacuum_off"] = "%s:%s" % (int(bool(vac_ok)), vac_msg)
        cleared = self._call(
            self._clear, ClearCurrentBox.Request(), timeout=15.0)
        recovered["clear"] = cleared.message if cleared else "timeout"
        self._t1("carrying_recovery", **recovered)
        return recovered

    # ------------------------------------------------------------------
    # Case execution

    def _planned_aabbs_local(self):
        """Map-committed boxes only (capacity arbitration + corridor height
        must reason on exactly what the production planner sees)."""
        return [tuple(box.aabb()) for box in self._placed_local_boxes]

    def _all_obstacle_aabbs_local(self):
        """Everything physically present: committed boxes plus declared
        planner-blind obstacles (swept-path validation)."""
        return [tuple(box.aabb()) for box
                in self._placed_local_boxes + self._blind_obstacles]

    def _probe_candidate_chain(self, pick_msg, slot):
        """Pre-execution reachability probe of a candidate's full place
        sequence: BuildMotionSequence + sequential segment probes with
        propagated IK seeds (the same semantics as the plan-only probe the
        driver executes; a single-pose IK probe from the observe seed is
        pessimistic because transit is a free-space plan)."""
        from luggage_msgs.srv import BuildMotionSequence
        request = BuildMotionSequence.Request()
        request.phase = "place"
        request.pick = pick_msg
        request.place_slot = slot
        built = self._call(self._build, request, timeout=15.0)
        if built is None or not built.success:
            return {"ok": False, "reason": "build:%s" % (
                built.message if built else "timeout"), "segments": []}
        joints = None
        segments = []
        for segment in built.segments:
            record = self._probe.probe_segment(segment, start_joints=joints)
            if record.get("ik_joints"):
                joints = record["ik_joints"]
            segments.append({
                "name": record["name"], "ik_ok": record["ik_ok"],
                "fraction": record["fraction"],
                "cartesian_ok": record["cartesian_ok"]})
            if not record["ik_ok"] or record.get("cartesian_ok") is False:
                return {"ok": False, "reason": "segment:%s" % record["name"],
                        "segments": segments}
        return {"ok": True, "reason": "probed", "segments": segments}

    def _validate_selected(self, slot, pick_msg):
        """Independent acceptance-B checks on the selected candidate."""
        center, yaw, size = self._slot_local(slot)
        checks = {}
        checks["inside_hull_10mm"] = self._ctx["contains_floor_box_lateral"](
            center, size, yaw, margin=0.010)
        checks["footprint_overlap_free"] = not any(
            fx.box_overlaps_aabb(center, size, yaw, aabb)
            for aabb in self._all_obstacle_aabbs_local())
        traverse_z = fx.corridor_traverse_z(
            self._ctx, center, size, self._placed_local_boxes)
        carry_center = [center[0], center[1], traverse_z - size[2] * 0.5]
        # Sweep the intra-container portion only: the payload is partially
        # outside the hull while passing the aperture plane, so start where
        # it is fully inside.
        entry_x = -self._ctx["inner_l"] * 0.5 + 0.5 * float(size[0])
        entry_center = [entry_x, center[1], traverse_z - size[2] * 0.5]
        checks["swept_inside_hull"] = bool(
            self._ctx["contains_floor_sweep"](
                entry_center, carry_center, size, yaw, margin=0.0)
            and self._ctx["contains_floor_sweep"](
                carry_center, center, size, yaw, margin=0.0))
        blocked, hits = fx.swept_path_blocked(
            self._ctx, center, size, yaw, self._all_obstacle_aabbs_local(),
            traverse_contact_z=traverse_z)
        checks["swept_collision_free"] = not blocked
        reach = self._probe_candidate_chain(pick_msg, slot)
        checks["chain_reachable"] = bool(reach.get("ok"))
        return {
            "center_local": center,
            "yaw_local": yaw,
            "size_wdh": size,
            "traverse_contact_z": traverse_z,
            "checks": checks,
            "swept_hits": hits,
            "reach_probe": reach,
        }

    def _compute_placement(self, pick_msg):
        request = ComputePlacement.Request()
        request.box = pick_msg
        request.placed = list(self._placed_slots)
        return self._call(self._compute_cli, request, timeout=30.0)

    def run_case(self, case):
        """Run one P0-P4 case end to end. Returns a case record."""
        t0 = time.time()
        slug = "pending"
        self._current_case_id = case.case_id
        manifest = case.manifest(self._catalog_sizes[case.cargo_id])
        manifest["geometry_hash"] = self._ctx["geometry_hash"]
        manifest["seed"] = int(self._args.seed)
        manifest["git_head"] = _git_head()
        manifest["dry_run"] = bool(self._args.dry_run)
        record = {
            "case_id": case.case_id, "outcome": "FAIL", "fail_code": "",
            "checks": {}, "committed": False,
        }
        self._case_dir = os.path.join(
            self._dumps, "%s_pending" % case.case_id)
        os.makedirs(self._case_dir, exist_ok=True)
        self._dump_json("case_manifest.json", manifest)
        self._sample_clock_guard()
        self._sample_resources()

        # --- initial map fixture -------------------------------------
        if case.initial != "carry":
            self._teardown_world_to_empty(
                "case_%s_start" % case.case_id)
        if case.initial in ("saturated", "obstacle"):
            boxes = self._fixture_for(case)
            ok, installed = self._install_fixture_boxes(
                boxes, spawn_physical=not self._args.dry_run)
            self._dump_json("fixture_install.json", installed)
            if not ok:
                record["fail_code"] = "FIXTURE_SETUP_FAILED:fixture_install"
                self._finish_case(record, case, slug, t0)
                return record
        pre_map = self._map_snapshot()
        self._dump_json("surface_pre.json", {
            "stats": pre_map["stats"], "digest": pre_map["digest"],
            "ledger": pre_map["ledger"]})
        if pre_map["stats"] is None:
            record["fail_code"] = "FIXTURE_SETUP_FAILED:map_stats_timeout"
            self._finish_case(record, case, slug, t0)
            return record
        baseline_committed = pre_map["stats"]["committed_box_count"]
        # Do not let ComputePlacement race the latched surface topic.
        self._wait_surface_revision(pre_map["stats"]["map_revision"])
        self._t1("pre_map_snapshot",
                 revision=pre_map["stats"]["map_revision"],
                 committed=baseline_committed, digest=pre_map["digest"])

        # --- carry setup ----------------------------------------------
        if self._args.dry_run:
            pick_msg = DetectedLuggage()
            pick_msg.width, pick_msg.depth, pick_msg.height = (
                self._catalog_sizes[case.cargo_id])
            pick_msg.height_valid = True
            pick_msg.height_source = 2
            pick_msg.yaw_valid = False
            pick_msg.pose.orientation.w = 1.0
            self._dump_json("perfect_descriptor.json", {
                "input_source": fx.INPUT_SOURCE,
                "catalog_id": case.cargo_id,
                "size_wdh": list(self._catalog_sizes[case.cargo_id]),
                "dry_run": True,
            })
        else:
            setup, setup_err = self._setup_carry(case)
            if setup is None:
                record["fail_code"] = setup_err
                self._recover_carried(
                    "setup_failed_%s" % case.case_id)
                self._finish_case(record, case, slug, t0)
                return record
            pick_msg, fields, tool_errors = setup
            self._dump_json("perfect_descriptor.json", fields)
            record["tool_errors_m"] = {
                k: round(v, 5) for k, v in tool_errors.items()}
        self._rtf_tick("case_%s_setup_done" % case.case_id)

        # --- ComputePlacement (first scored operation) ----------------
        placement = self._compute_placement(pick_msg)
        last_dump = dict(self._last_result or {})
        self._dump_json("placement_last_result.json", last_dump)
        self._dump_json("surface_at_request.json", self._surface_2d or {})
        if placement is None:
            record["fail_code"] = "COMPUTE_PLACEMENT_TIMEOUT"
            self._abort_case_recovery(record, case)
            self._finish_case(record, case, slug, t0)
            return record
        self._t1("compute_placement", success=bool(placement.success),
                 message=placement.message,
                 revision_in_dump=last_dump.get("map_revision"),
                 revision_snapshot=pre_map["stats"]["map_revision"])
        if not placement.success:
            self._handle_placement_failure(
                record, case, pre_map, placement, last_dump,
                baseline_committed)
            self._finish_case(
                record, case,
                "fail_closed" if record["outcome"].startswith("PASS")
                else (record["fail_code"] or "planning_failed"),
                t0)
            return record

        # --- selected-candidate validation ----------------------------
        slot = placement.slot
        revision_ok = (last_dump.get("map_revision")
                       == pre_map["stats"]["map_revision"])
        slot_meta = self._slot_meta(slot)
        validation = self._validate_selected(slot, pick_msg)
        validation["request_revision_matches_snapshot"] = bool(revision_ok)
        self._dump_json("slot_validation.json", validation)
        if case.case_id == "P4":
            record["p4_fixture_property"] = self._p4_fixture_property(
                last_dump)
            record["checks"]["p4_fixture_property_proven"] = bool(
                record["p4_fixture_property"]["fixture_property_proven"])
        record["checks"].update({
            "request_revision_match": revision_ok,
            "inside_hull_10mm": validation["checks"]["inside_hull_10mm"],
            "footprint_overlap_free":
                validation["checks"]["footprint_overlap_free"],
            "swept_collision_free":
                validation["checks"]["swept_collision_free"],
        })
        if not revision_ok:
            record["fail_code"] = "PLACEMENT_IDENTITY_MISMATCH"
        elif not validation["checks"]["inside_hull_10mm"]:
            record["fail_code"] = "HULL_CLEARANCE_VIOLATION"
        elif case.case_id == "P4" and not record["checks"].get(
                "p4_fixture_property_proven"):
            record["fail_code"] = "FIXTURE_SETUP_FAILED:p4_property_unproven"
        elif not (validation["checks"]["footprint_overlap_free"]
                  and validation["checks"]["swept_collision_free"]
                  and validation["checks"]["chain_reachable"]):
            # The selected candidate is rejected before execution (blind
            # obstacle, overlap, or unreachable traverse pose). Try the
            # retained next-best candidates; P4 may also close as a
            # zero-motion rejection when nothing validates.
            reason = (
                "FOOTPRINT_OVERLAP"
                if not validation["checks"]["footprint_overlap_free"]
                else "SWEPT_PATH_BLOCKED"
                if not validation["checks"]["swept_collision_free"]
                else "PLACE_SLOT_UNREACHABLE")
            alt = self._try_alternative_candidates(case, last_dump, pick_msg)
            record["alternative_candidates"] = alt
            if alt.get("slot") is not None:
                slot = alt["slot"]
                slot_meta = self._slot_meta(slot)
                record["checks"]["alternative_collision_free"] = True
                record["rejected_first_candidate"] = reason
            elif case.case_id == "P4":
                return self._p4_reject_before_execution(
                    record, case, pre_map, validation, t0)
            else:
                record["fail_code"] = reason

        # --- execute production place chain ----------------------------
        if not record["fail_code"]:
            record.update(self._execute_place_chain(case, pick_msg, slot,
                                                    slot_meta))
        if record["fail_code"]:
            self._abort_case_recovery(record, case)
            slug = record["fail_code"]
            self._finish_case(record, case, slug, t0)
            return record

        # --- cargo-map commit + idempotency + occupancy scoring --------
        commit = self._commit_and_score(case, slot, slot_meta, pre_map,
                                        baseline_committed)
        record["commit"] = commit
        record["committed"] = bool(commit.get("ok"))
        if not record["committed"]:
            record["fail_code"] = commit.get("code", "COMMIT_FAILED")
            slug = record["fail_code"]
        else:
            record["outcome"] = "PASS"
            slug = "ok"
        self._finish_case(record, case, slug, t0)
        return record

    # ------------------------------------------------------------------

    def _p4_reject_before_execution(self, record, case, pre_map, validation,
                                    t0):
        """P4 safe alternative: tempting slot rejected before execution."""
        record["outcome"] = "PASS_REJECTED"
        record["fail_code"] = ""
        record["checks"]["rejected_before_execution"] = True
        if not self._args.dry_run:
            self._recover_carried("p4_rejected_%s" % case.case_id,
                                  release_in_place=True)
        post = self._map_snapshot()
        record["checks"]["map_digest_preserved"] = bool(
            post["digest"] == pre_map["digest"])
        record["checks"]["map_revision_preserved"] = bool(
            post["stats"]
            and post["stats"]["map_revision"]
            == pre_map["stats"]["map_revision"])
        record["checks"]["zero_motion_goals"] = (
            len([g for g in self._pos_guards["executed_goals"]
                 if g["case"] == case.case_id and g["scored"]]) == 0)
        self._dump_json("p4_rejection.json", {
            "validation": validation,
            "post_map": {"digest": post["digest"],
                         "stats": post["stats"]}})
        self._finish_case(record, case, "P4_rejected", t0)
        return record

    def _handle_placement_failure(self, record, case, pre_map, placement,
                                  last_dump, baseline_committed):
        """P3 fail-closed path (and unexpected P0-P2/P4 failures)."""
        placed_aabbs = self._planned_aabbs_local()
        fits, first_fit = fx.geometric_capacity(
            self._ctx, self._catalog_sizes[case.cargo_id], placed_aabbs)
        verdict = fx.classify_placement_failure(
            placement.message, fits, last_dump.get("reject_histogram"))
        record["classification"] = verdict
        record["checks"]["capacity_independently_confirmed"] = bool(
            verdict["capacity_confirmed"])
        record["checks"]["zero_motion_goals"] = (
            len([g for g in self._pos_guards["executed_goals"]
                 if g["case"] == case.case_id and g["scored"]]) == 0)
        post = self._map_snapshot()
        record["checks"]["map_digest_preserved"] = bool(
            post["digest"] == pre_map["digest"])
        record["checks"]["map_revision_preserved"] = bool(
            post["stats"]
            and post["stats"]["map_revision"]
            == pre_map["stats"]["map_revision"])
        record["checks"]["committed_count_preserved"] = bool(
            post["stats"]
            and post["stats"]["committed_box_count"] == baseline_committed)
        self._dump_json("failure_classification.json", {
            "verdict": verdict, "capacity_first_fit": first_fit,
            "post_map": {"digest": post["digest"],
                         "stats": post["stats"]}})
        expected = case.expect_place is False
        accepted = bool(
            verdict["ok"] and case.fail_closed_codes
            and verdict["code"] in case.fail_closed_codes)
        if (expected and accepted and record["checks"]["zero_motion_goals"]
                and record["checks"]["map_digest_preserved"]):
            record["outcome"] = "PASS_FAIL_CLOSED"
            record["fail_code"] = ""
            if not self._args.dry_run:
                # No place motion ran: release in place on the platform.
                self._recover_carried(
                    "planning_fail_closed_%s" % case.case_id,
                    release_in_place=True)
            return
        record["fail_code"] = "PLACEMENT_UNEXPECTED_FAILURE:%s" % (
            placement.message)
        if not self._args.dry_run:
            self._recover_carried("planning_failed_%s" % case.case_id)

    def _candidate_slot(self, cand, size):
        slot = SlotSpec()
        base = cand.get("center_base_link") or [0.0, 0.0, 0.0]
        slot.place_pose.position.x = float(base[0])
        slot.place_pose.position.y = float(base[1])
        slot.place_pose.position.z = float(base[2])
        slot.place_pose.orientation = _yaw_quat(
            float(cand.get("yaw_base_link") or 0.0))
        slot.width, slot.depth, slot.height = (
            float(size[0]), float(size[1]), float(size[2]))
        return slot

    def _try_alternative_candidates(self, case, last_dump, pick_msg):
        """Pick the next retained feasible candidate that is overlap-free,
        sweep-free, and chain-reachable ('different collision-free
        candidate succeeds')."""
        tried = []
        candidates = [c for c in (last_dump.get("candidates") or [])
                      if c.get("feasible")]
        # Score-sorted candidates come in homogeneous (x, yaw) families;
        # probe one representative per family so a single unreachable
        # family cannot exhaust the retry budget.
        families = []
        seen = set()
        for cand in candidates:
            center = cand.get("center_local") or (
                cand.get("center_base") or [0.0, 0.0, 0.0])
            yaw = float(cand.get("yaw") or 0.0)
            key = (round(float(center[0]), 2), round(yaw, 2))
            if key in seen:
                continue
            seen.add(key)
            families.append(cand)
            if len(families) >= 8:
                break
        size_default = self._catalog_sizes[case.cargo_id]
        for cand in families:
            center = cand.get("center_local") or (
                cand.get("center_base") or [0.0, 0.0, 0.0])
            # Solver candidates are volume-centre-relative (floor at
            # -inner_h/2); the fixture geometry is floor-relative.
            center = [float(center[0]), float(center[1]),
                      float(center[2]) + self._ctx["inner_h"] * 0.5]
            yaw = float(cand.get("yaw") or 0.0)
            size = [float(v) for v in (cand.get("size") or size_default)]
            traverse_z = fx.corridor_traverse_z(
                self._ctx, center, size, self._placed_local_boxes)
            overlap = any(
                fx.box_overlaps_aabb(center, size, yaw, aabb)
                for aabb in self._all_obstacle_aabbs_local())
            blocked, hits = fx.swept_path_blocked(
                self._ctx, center, size, yaw, self._all_obstacle_aabbs_local(),
                traverse_contact_z=traverse_z)
            if overlap:
                blocked = True
                hits = [{"stage": "footprint_overlap"}]
            slot = self._candidate_slot(cand, size)
            reach = self._probe_candidate_chain(pick_msg, slot)
            if not reach.get("ok"):
                blocked = True
                hits = hits[:0] + [{"stage": "chain_unreachable:%s" % (
                    reach.get("reason", "?"))}]
            tried.append({
                "center_local": [round(float(v), 4) for v in center],
                "yaw": round(yaw, 4), "blocked": bool(blocked),
                "hits": hits[:4]})
            if blocked:
                continue
            return {"slot": slot, "tried": tried, "reach": reach}
        return {"slot": None, "tried": tried}

    def _execute_place_chain(self, case, pick_msg, slot, slot_meta):
        self._scoring_active = True
        # run_place_from_carry is invoked directly (not via run_trial), so
        # reset the per-trial traces the base class would have cleared.
        self._timeline = []
        self._segments_log = []
        self._tf_trace = []
        self._place_state = "INIT"
        self._probe_joints = None
        args = self._args
        args.dump_dir = self._case_dir
        args.dump_exact = False
        # Unique scene-object id per committed box (layer/row/col only feed
        # placed_object_id; ComputePlacement returns 0/0/0 for every slot,
        # which would make later boxes overwrite earlier collision objects).
        slot.layer, slot.row, slot.col = 0, 0, self._commit_seq + 1
        # Keep placed models/scene state for persistence within the streak.
        self._keep_placed = True
        trial = PlaceTrial(index=int(case.case_id[1:]))
        trial.catalog_id = case.cargo_id
        trial.extras = {"input_source": fx.INPUT_SOURCE}
        try:
            result = self.run_place_from_carry(
                pick_msg, trial, slot, slot_meta)
        except Exception as exc:  # noqa: BLE001 - case boundary
            result = trial
            result.fail_code = "DRIVER_EXC_%s" % type(exc).__name__
            self.get_logger().error("place chain raised %s" % exc)
        self._scoring_active = False
        self._rtf_tick("case_%s_place_done" % case.case_id)
        self._sample_resources()
        fractions = {str(row.get("name")): row.get("fraction")
                     for row in self._segments_log}
        insert_fraction = fractions.get("insert")
        descend_fraction = result.descend_fraction
        record = {
            "trial": trial_to_dict(result),
            "place_ok": bool(place_ok(result)),
            # GOTO_FAILED after HOME is not a place failure (closed-loop
            # plan rule); the placement and commit still count.
            "fail_code": ("" if place_ok(result)
                          else (result.fail_code or "")),
            "checks": {
                "insert_fraction_ge_095": bool(
                    insert_fraction is not None
                    and float(insert_fraction) >= 0.95),
                "descend_fraction_ge_095": bool(
                    descend_fraction is not None
                    and float(descend_fraction) >= 0.95),
                "descend_no_ompl_fallback": bool(
                    result.used_ompl_fallback_descend is not True),
                "place_ok": bool(place_ok(result)),
            },
        }
        self._dump_json("place_trial.json", record["trial"])
        if record["fail_code"]:
            return record
        if not record["checks"]["insert_fraction_ge_095"]:
            record["fail_code"] = "PLACE_FRACTION_insert"
        elif not record["checks"]["descend_fraction_ge_095"]:
            record["fail_code"] = "PLACE_FRACTION_descend"
        elif not record["checks"]["descend_no_ompl_fallback"]:
            record["fail_code"] = "DESCEND_OMPL_FALLBACK"
        return record

    def _commit_order_events(self, commit_t_ros):
        events = []
        for row in self._timeline:
            if row.get("to") == "RELEASED":
                events.append(("release", row["t_ros"]))
            elif row.get("to") == "PLACE_RETREAT":
                events.append(("retreat", row["t_ros"]))
            elif row.get("to") == "VERIFIED":
                events.append(("verify", row["t_ros"]))
        events.append(("commit", float(commit_t_ros)))
        return events

    def _commit_and_score(self, case, slot, slot_meta, pre_map,
                          baseline_committed):
        center, yaw_local, size = self._slot_local(slot)
        self._commit_seq += 1
        world_slot = self._slot_world_msg(slot, slot_meta, self._commit_seq)
        commit_t0 = time.time()
        add = self._call(self._map_add, AddPlacedBox.Request(slot=world_slot),
                         timeout=15.0)
        commit_latency = time.time() - commit_t0
        if add is None or not add.success:
            return {"ok": False, "code": "COMMIT_FAILED",
                    "message": add.message if add else "timeout"}
        order_events = self._commit_order_events(self.ros_now_sec())
        order_ok, order_reason = fx.validate_commit_order(order_events)
        stats_now = self._map_stats() or {}
        # The latched surface topic may lag the service-state commit; wait
        # for it so the post map really contains the committed footprint.
        self._wait_surface_revision(stats_now.get("map_revision", -1))
        post = self._map_snapshot()
        stats_after = post["stats"] or {}
        expected_count = baseline_committed + 1
        count_ok = stats_after.get("committed_box_count") == expected_count
        revision_bumped = bool(
            stats_after.get("map_revision", -1)
            > pre_map["stats"]["map_revision"])

        # Idempotency: repeat the identical commit.
        repeat = self._call(self._map_add, AddPlacedBox.Request(
            slot=world_slot), timeout=15.0)
        stats_repeat = self._map_stats() or {}
        self._wait_surface_revision(stats_repeat.get("map_revision", -1))
        post_repeat = self._map_snapshot()
        idem = fx.idempotency_check(
            stats_after, post_repeat["stats"] or {},
            post["digest"], post_repeat["digest"])

        diff = fx.occupancy_diff(
            pre_map["surface"] or post["surface"], post["surface"],
            center, size, yaw_local)

        # Replay determinism: the saved post map must enumerate identically
        # on repeated replay, and the committed footprint must come back
        # overlap-infeasible for the same cargo.
        placed_before = self._planned_aabbs_local()
        committed_aabb = (
            center[0] - size[0] * 0.5, center[1] - size[1] * 0.5,
            center[2] - size[2] * 0.5,
            center[0] + size[0] * 0.5, center[1] + size[1] * 0.5,
            center[2] + size[2] * 0.5)
        replay_a = fx.replay_candidates(
            post["surface"], size, placed_before + [committed_aabb],
            self._ctx)
        replay_b = fx.replay_candidates(
            post["surface"], size, placed_before + [committed_aabb],
            self._ctx)
        replay_stable = replay_a == replay_b
        commit_cell = None
        res = float((post["surface"] or {}).get("resolution", 0.05))
        cell_x = int((center[0] + self._ctx["inner_l"] * 0.5) / res)
        cell_y = int((center[1] + self._ctx["inner_w"] * 0.5) / res)
        for key, value in replay_a.items():
            parsed = key.strip("()").split(", ")
            if (len(parsed) >= 2 and parsed[0] == str(cell_x)
                    and parsed[1] == str(cell_y)):
                commit_cell = {"key": key, **value}
                break

        # Track committed state for the next case request.
        committed_slot = SlotSpec()
        committed_slot.layer, committed_slot.row, committed_slot.col = (
            0, 0, self._commit_seq)
        committed_slot.width, committed_slot.depth, committed_slot.height = (
            float(slot.width), float(slot.depth), float(slot.height))
        base = slot_meta["pose_base_link"]
        committed_slot.place_pose.position.x = float(base["position"][0])
        committed_slot.place_pose.position.y = float(base["position"][1])
        committed_slot.place_pose.position.z = float(base["position"][2])
        committed_slot.place_pose.orientation = _yaw_quat(base["yaw"])
        self._placed_slots.append(committed_slot)
        self._placed_local_boxes.append(
            fx.FixtureBox(tuple(center), tuple(size), yaw_local,
                          role="committed_%s" % case.case_id))

        self._dump_json("surface_post.json", {
            "stats": post["stats"], "digest": post["digest"],
            "ledger": post["ledger"]})
        checks = {
            "count_ok": bool(count_ok),
            "revision_bumped_once": revision_bumped,
            "commit_order_ok": bool(order_ok),
            "idempotency_ok": bool(idem["ok"]),
            "iou_ok": bool(diff["iou_ok"]),
            "height_ok": bool(diff["height_within_one_cell"]),
            "outside_clean": bool(diff["outside_footprint_changes"] == 0),
            "replay_stable": bool(replay_stable),
        }
        code_map = {
            "count_ok": "COMMIT_COUNT",
            "revision_bumped_once": "COMMIT_REVISION",
            "commit_order_ok": "COMMIT_ORDER",
            "idempotency_ok": "COMMIT_NOT_IDEMPOTENT",
            "iou_ok": "COMMIT_FOOTPRINT_IOU",
            "height_ok": "COMMIT_HEIGHT_ERROR",
            "outside_clean": "COMMIT_OUTSIDE_MUTATION",
            "replay_stable": "COMMIT_REPLAY_UNSTABLE",
        }
        payload = {
            "ok": all(checks.values()),
            "code": "",
            "commit_latency_sec": round(commit_latency, 4),
            "map_message": add.message,
            "repeat_message": repeat.message if repeat else "timeout",
            "checks": checks,
            "idempotency": idem,
            "occupancy_diff": diff,
            "commit_order": {"ok": order_ok, "reason": order_reason,
                             "events": order_events},
            "replay_commit_cell": commit_cell,
            "expected_count": expected_count,
            "actual_count": stats_after.get("committed_box_count"),
        }
        if not payload["ok"]:
            payload["code"] = next(
                code_map[key] for key, value in checks.items() if not value)
        self._dump_json("commit.json", payload)
        self._t1("cargo_map_commit",
                 **{k: v for k, v in payload.items()
                    if k != "occupancy_diff"})
        if payload["ok"] and not self._args.dry_run:
            # Keep the placed Gazebo model but release the spawner's
            # current-box slot so the next case can spawn cleanly.
            finalized = self._call(
                self._finalize, FinalizeCurrentBox.Request(), timeout=15.0)
            self._t1("finalize_current_box",
                     ok=bool(finalized and finalized.success),
                     message=finalized.message if finalized else "timeout")
        return payload

    def _abort_case_recovery(self, record, case):
        if self._args.dry_run:
            return
        vacuum = dict(self._vacuum_state or {})
        if vacuum.get("attached"):
            self._recover_carried(
                "abort_%s_%s" % (case.case_id,
                                 record.get("fail_code") or "unknown"))

    def _finish_case(self, record, case, slug, t0):
        clock = self._sample_clock_guard()
        self._pos_guards_snapshot(case)
        self._flush_t1()
        record["wall_time_sec"] = round(time.time() - t0, 2)
        self._dump_json("case_record.json", record)
        if slug == "pending" and record["fail_code"]:
            slug = record["fail_code"]
        safe_slug = str(slug).replace(":", "_").replace("/", "_")[:60]
        target = os.path.join(self._dumps, "%s_%s" % (case.case_id, safe_slug))
        if self._case_dir and self._case_dir != target \
                and not os.path.isdir(target):
            os.rename(self._case_dir, target)
            self._case_dir = target
        record["dump"] = target
        completeness = fx.dump_artifacts_complete(target)
        with open(os.path.join(target, "dump_completeness.json"), "w",
                  encoding="utf-8") as handle:
            json.dump(completeness, handle, indent=2, sort_keys=True)
            handle.write("\n")
        record["capture_complete"] = completeness["capture_complete"]
        print(json.dumps({
            "case": case.case_id, "outcome": record["outcome"],
            "fail_code": record["fail_code"],
            "capture_complete": completeness["capture_complete"],
            "wall_time_sec": record["wall_time_sec"],
        }, sort_keys=True), flush=True)

    def _pos_guards_snapshot(self, case):
        self._dump_json("guards.json", {
            "detect_calls": self._pos_guards["detect_calls"],
            "executed_goals": [g for g in self._pos_guards["executed_goals"]
                               if g["case"] == case.case_id],
            "setup_goals": [g for g in self._pos_guards["setup_goals"]
                            if g.get("case") == case.case_id],
            "clock_publishers": self._pos_guards["clock_publisher_samples"][-1],
            "rtf_last": self._pos_guards["rtf_trace"][-3:],
            "rss_last": self._pos_guards["rss_mib_trace"][-1],
        })

    # ------------------------------------------------------------------
    # Streak entry point

    def run_streak(self, case_ids):
        results = []
        self._write_streak_header()
        for case_id in case_ids:
            record = self.run_case(self._cases[case_id])
            attempts = 1
            while (record["fail_code"].startswith("FIXTURE_SETUP_FAILED")
                   and attempts < 2):
                # Setup failure: evidence is already frozen; the plan calls
                # for one replacement case before the streak fails.
                attempts += 1
                self._t1("setup_failure_replacement",
                         case=case_id, attempt=attempts)
                record["replacement_for_attempt"] = attempts - 1
                self._archive_setup_failure(case_id, attempts - 1)
                record = self.run_case(self._cases[case_id])
            results.append(record)
            if not record["outcome"].startswith("PASS"):
                break
        self._write_streak_summary(results)
        return results

    def _archive_setup_failure(self, case_id, attempt):
        """Keep the failed setup evidence under an attempt-suffixed name so
        the replacement case can reuse the pending dump directory."""
        import glob as _glob
        pattern = os.path.join(
            self._dumps, "%s_FIXTURE_SETUP_FAILED*" % case_id)
        matches = sorted(_glob.glob(pattern))
        if not matches:
            return
        current = matches[0]
        target = "%s_attempt%d" % (current, attempt)
        index = 2
        while os.path.isdir(target):
            target = "%s_attempt%d_%d" % (current, attempt, index)
            index += 1
        os.rename(current, target)

    def _p4_fixture_property(self, last_dump):
        """Evidence that the tempting deep slots really are swept-blocked.

        Checks the most portal-distant feasible candidates independently;
        the fixture property holds when at least one of them is blocked.
        """
        candidates = [c for c in (last_dump.get("candidates") or [])
                      if c.get("feasible")]
        deep = sorted(
            candidates,
            key=lambda c: -(float((c.get("center_local") or [0.0])[0])))[:3]
        checked = []
        blocked_deep = 0
        for cand in deep:
            center = cand.get("center_local") or [0.0, 0.0, 0.0]
            center = [float(center[0]), float(center[1]),
                      float(center[2]) + self._ctx["inner_h"] * 0.5]
            yaw = float(cand.get("yaw") or 0.0)
            size = [float(v) for v in (cand.get("size")
                                       or fx.CATALOG_SIZES["standard"])]
            traverse_z = fx.corridor_traverse_z(
                self._ctx, center, size, self._placed_local_boxes)
            blocked, hits = fx.swept_path_blocked(
                self._ctx, center, size, yaw,
                self._all_obstacle_aabbs_local(),
                traverse_contact_z=traverse_z)
            checked.append({
                "center_local": [round(float(v), 4) for v in center],
                "blocked": bool(blocked), "n_hits": len(hits)})
            if blocked:
                blocked_deep += 1
        property_proof = {
            "deep_candidates_checked": checked,
            "n_deep_blocked": blocked_deep,
            "fixture_property_proven": bool(blocked_deep >= 1),
        }
        self._dump_json("p4_fixture_property.json", property_proof)
        return property_proof

    def _write_streak_header(self):
        with open(os.path.join(self._streak_out, "streak_manifest.json"),
                  "w", encoding="utf-8") as handle:
            json.dump({
                "streak_index": int(self._args.streak_index),
                "dry_run": bool(self._args.dry_run),
                "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
                "seed": int(self._args.seed),
                "geometry_hash": self._ctx["geometry_hash"],
                "git_head": _git_head(),
                "profile": {
                    "hull_margin": 0.01,
                    "min_support_ratio": 0.55,
                    "last_result_max_candidates": 200,
                    "lateral_margin": self._ctx["lateral_margin"],
                },
                "catalog_sizes": {k: list(v)
                                  for k, v in self._catalog_sizes.items()},
                "started_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            }, handle, indent=2, sort_keys=True)
            handle.write("\n")

    def _write_streak_summary(self, results):
        rtf_trace = self._pos_guards["rtf_trace"]
        rtf_violations = _rtf_violation_windows(rtf_trace)
        unexpected_goals = fx.unexpected_segment_names(
            [g["name"] for g in self._pos_guards["executed_goals"]])
        guards = {
            "detect_calls_total": self._pos_guards["detect_calls"],
            "unexpected_segment_names": unexpected_goals,
            "clock_publisher_counts": sorted({
                s["count"] for s
                in self._pos_guards["clock_publisher_samples"]}),
            "rtf_min": min((s["rtf"] for s in rtf_trace), default=None),
            "rtf_violation_windows": rtf_violations,
            "rss_final_mib": (self._pos_guards["rss_mib_trace"][-1]
                              if self._pos_guards["rss_mib_trace"] else None),
        }
        expected_cases = [c.strip() for c in self._args.cases.split(",")
                          if c.strip()]
        summary = {
            "streak_index": int(self._args.streak_index),
            "dry_run": bool(self._args.dry_run),
            "cases": [
                {"case": r["case_id"], "outcome": r["outcome"],
                 "fail_code": r["fail_code"],
                 "capture_complete": r.get("capture_complete")}
                for r in results],
            "n_pass": sum(1 for r in results
                          if r["outcome"].startswith("PASS")),
            "n_cases": len(results),
            "guards": guards,
        }
        summary["streak_pass"] = bool(
            summary["n_pass"] == summary["n_cases"] == len(expected_cases)
            and guards["detect_calls_total"] == 0
            and not guards["unexpected_segment_names"]
            and set(guards["clock_publisher_counts"]) <= {1}
            and not rtf_violations
            and all(r.get("capture_complete") for r in results))
        with open(os.path.join(self._streak_out, "streak_summary.json"), "w",
                  encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2, sort_keys=True,
                      default=str)
            handle.write("\n")
        with open(os.path.join(self._streak_out, "guards_full.json"), "w",
                  encoding="utf-8") as handle:
            json.dump({
                "rtf_trace": rtf_trace,
                "clock_publisher_samples":
                    self._pos_guards["clock_publisher_samples"],
                "rss_mib_trace": self._pos_guards["rss_mib_trace"],
            }, handle, indent=2, sort_keys=True, default=str)
            handle.write("\n")
        print(json.dumps(summary, indent=2, sort_keys=True), flush=True)


def parse_args(argv):
    base = parse_place_args([
        "--out", "/tmp/place_only_unused",
        "--payload", "vacuum",
        "--plan-timeout", "60.0",
        "--goto-timeout", "60.0",
        "--observe-pose", "pickup_observe",
        "--skip-graph-check",
    ])
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True)
    parser.add_argument("--streak-index", type=int, default=1)
    parser.add_argument("--cases", default="P0,P1,P2,P3,P4")
    parser.add_argument("--seed", type=int, default=20260914)
    parser.add_argument("--dry-run", dest="dry_run", action="store_true")
    parser.add_argument("--tool-tol", type=float, default=0.005)
    known, _rest = parser.parse_known_args(argv)
    for key, value in vars(known).items():
        setattr(base, key, value)
    base.dump_dir = os.path.join(base.out, "dumps")
    base.dump_exact = False
    base.use_vacuum = not base.dry_run
    base.plan_only = bool(base.dry_run)
    base.drift_wait = 1.0
    return base


def main(argv=None):
    args = parse_args(argv or sys.argv[1:])
    if os.environ.get("ROS_DOMAIN_ID") != "7":
        print("refusing to run: ROS_DOMAIN_ID must be 7", flush=True)
        return 2
    os.makedirs(args.out, exist_ok=True)
    os.makedirs(args.dump_dir, exist_ok=True)

    import rclpy
    rclpy.init()
    driver = PlaceOnlyDriver(args)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(driver)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    def _on_sigint(_signum, _frame):
        print("interrupt requested; finishing case", flush=True)

    signal.signal(signal.SIGINT, _on_sigint)
    try:
        err = driver.wait_graph(timeout=120.0)
        if err:
            print("graph: %s" % err, flush=True)
            return 2
        if not driver._probe.wait_ready(
                timeout_sec=30.0 if args.dry_run else 10.0):
            print("move_group unavailable", flush=True)
            return 2
        if driver._sample_clock_guard() != 1:
            print("clock guard: expected exactly one /clock publisher",
                  flush=True)
            return 2
        case_ids = [c.strip() for c in args.cases.split(",") if c.strip()]
        driver.run_streak(case_ids)
        with open(os.path.join(args.out, "streak_summary.json"), "r",
                  encoding="utf-8") as handle:
            summary = json.load(handle)
        return 0 if summary.get("streak_pass") else 1
    finally:
        driver._flush_t1()
        listener = getattr(driver, "_tf_listener", None)
        tf_exec = getattr(listener, "executor", None) if listener else None
        if tf_exec is not None:
            tf_exec.shutdown()
            thread = getattr(listener, "dedicated_listener_thread", None)
            if thread is not None:
                thread.join(timeout=2.0)
        executor.shutdown()
        tf_node = getattr(driver, "_tf_node", None)
        driver.destroy_node()
        if tf_node is not None:
            tf_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
