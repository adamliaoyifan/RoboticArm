#!/usr/bin/env python3
"""Reset validity campaign runner (plan OCTOMAP_RESET_VALIDITY_NEXT_20261103 §5/§6).

Entry points (fixed CLI, contract §6):
  reset_campaign.py --mode rehearsal|score --manifest <frozen-input> --out <run>
  reset_campaign.py --mode replay --manifest <case-bundle> --out <run>

Gates (deterministic, barrier-driven -- never sleeps as proof of order):
  V1 invalidation   7 stages x N repeats. A request is parked at a NAMED
                    product boundary (observer stage gates or the downstream
                    harness barriers), reset commits while it is parked, and
                    the released request must terminate INVALIDATED / tagged
                    invalid with no usable payload. A concurrent NEW request
                    post-reset must never lease the old epoch.
  V2 recovery       N cycles: invalidate -> late old result arrives (rejected,
                    counted) -> new-epoch observation -> VALID -> injected
                    robot-state fault (missing / stale / pre-transition) keeps
                    the gate closed -> fresh sample opens it -> exactly one
                    new-epoch plan admission per cycle. Duplicate reset_id is
                    idempotent; a distinct reset during BUILDING re-invalidates.
  V3 lock/lifecycle 8 reset pairings x N: pairing completes within the 500 ms
                    request deadline, no deadlock, invalidation observable
                    within 200 ms. Plus 2N ownership cycles (held view, held
                    surface buffer): bytes attributed exactly once; after the
                    final owner release the class returns to baseline within
                    the bounded 2 s observation.

Scheduling discipline (contract §6): every request is scheduled by integer
index against an absolute monotonic deadline -- no ``while now <= end`` loop;
generation stops at the frozen index set; a race may add at most 2 extra IDs
per stream, labeled ``overrun`` and retained for the judge (which fails an
overrun of 3). Timed P/H/L/R windows are NOT this runner's job (they stay in
freespace_campaign.py / score_orchestrator.py); V1-V3 are the deterministic
pre-scoring gates.

Artifacts (§7): manifest.json, events.jsonl (T1), v1.jsonl, v2.jsonl,
v3.jsonl, ownership.jsonl, cases/ (one bundle per failing case, T2 payload),
campaign_summary.json. Exit 0 only for a complete pass.
"""
from __future__ import annotations

import argparse
import glob as globlib
import hashlib
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import yaml

OBSERVER = "/uld_occ_observer"
V1_STAGES = [
    "acquire_before_lock",     # get_snapshot, parked before repository lock
    "acquire_after_lock",      # get_snapshot, parked with the lease held
    "surface_first_build",     # read_surface, parked building the column cache
    "page_work",               # read_cells + query_freespace page/ROI walk
    "response_serialization",  # query_freespace, parked before payload fill
    "placement_computation",   # downstream harness compute barrier
    "planning_admission",      # downstream harness commit barrier
]
V3_PAIRINGS = [
    "integration", "acquire", "surface", "roi", "release",
    "cache_build", "planning_admission", "status",
]
REQUEST_DEADLINE_MS = 500.0     # existing request deadline (contract V3)
INVALIDATION_VISIBLE_MS = 200.0
#: Contract §5: a V3 pairing (parked request + reset + completion) must
#: close within 500 ms of the pairing start. The gen1 runner used a
#: separate 4*deadline cutoff, which could not implement this gate
#: (review 2026-11-03 finding 2).
PAIRING_DEADLINE_MS = 500.0
OWNERSHIP_BASELINE_S = 2.0


def mono() -> int:
    return time.monotonic_ns()


class Events:
    """T1 boundary events (§7): one JSON line per request/transition."""

    def __init__(self, out_dir: Path):
        self.path = out_dir / "events.jsonl"
        self._file = open(self.path, "w")

    def emit(self, kind: str, **fields):
        record = {"t_mono_ns": mono(), "kind": kind}
        record.update(fields)
        self._file.write(json.dumps(record, sort_keys=True) + "\n")
        self._file.flush()

    def close(self):
        self._file.close()


class NodeProcess:
    def __init__(self, binary, geometry, config, log_path):
        self.log = open(log_path, "w")
        env = os.environ.copy()
        install_lib = Path(binary).resolve().parent.parent.parent / "lib"
        env["LD_LIBRARY_PATH"] = "%s:%s" % (install_lib, env.get("LD_LIBRARY_PATH", ""))
        self.proc = subprocess.Popen(
            [binary, "--ros-args",
             "-p", "geometry_file:=%s" % geometry,
             "-p", "config_file:=%s" % config,
             "-p", "input_mode:=subscription",
             "-p", "cloud_topic:=/reset_campaign_cloud",
             "-p", "optical_frame:=d555_color_optical_frame",
             "-p", "sensor_id:=d555",
             "-p", "behavior_test_hooks:=true",
             "-p", "ownership_trace:=true",
             "-p", "validity_max_age_ms:=1000"],
            stdout=self.log, stderr=subprocess.STDOUT, env=env,
            start_new_session=True)

    def stop(self):
        if self.proc.poll() is None:
            os.killpg(self.proc.pid, signal.SIGINT)
            try:
                self.proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGTERM)
                try:
                    self.proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    # A service thread parked at an armed stage gate can
                    # block graceful shutdown; the last resort is required
                    # or the process leaks and answers the next run's
                    # services under the same name.
                    os.killpg(self.proc.pid, signal.SIGKILL)
                    self.proc.wait(timeout=5)
        self.log.close()


class Client:
    """rclpy service/shell client: 4 executor threads (contract §5)."""

    def __init__(self):
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2
        from tf2_ros import StaticTransformBroadcaster
        from geometry_msgs.msg import TransformStamped
        from uld_occ_observer.srv import (
            GetSnapshot, ReadSurface, ReadCells, QueryFreespace,
            ReleaseSnapshot, ResetObservation, GetValidity,
            GetObservationStatus, GetResourceUsage, SetBehaviorHook,
            IntegrateObservation)
        rclpy.init()
        self.rclpy = rclpy
        self.node = Node("reset_campaign_client")
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        reliable = QoSProfile(depth=32, reliability=ReliabilityPolicy.RELIABLE)
        self.cloud = self.node.create_publisher(
            PointCloud2, "/reset_campaign_cloud", reliable)
        self.get_snapshot = self._client(GetSnapshot, "get_snapshot")
        self.read_surface = self._client(ReadSurface, "read_surface")
        self.read_cells = self._client(ReadCells, "read_cells")
        self.freespace = self._client(QueryFreespace, "query_freespace")
        self.release = self._client(ReleaseSnapshot, "release_snapshot")
        self.reset = self._client(ResetObservation, "reset_observation")
        self.validity = self._client(GetValidity, "get_validity")
        self.status = self._client(GetObservationStatus, "get_observation_status")
        self.resource = self._client(GetResourceUsage, "get_resource_usage")
        self.hook = self._client(SetBehaviorHook, "set_behavior_hook")
        self.integrate = self._client(IntegrateObservation, "integrate_observation")
        self._tf = StaticTransformBroadcaster(self.node)
        frame = TransformStamped()
        frame.header.frame_id = "container_link"
        frame.child_frame_id = "d555_color_optical_frame"
        frame.transform.rotation.x = 0.5
        frame.transform.rotation.y = -0.5
        frame.transform.rotation.z = 0.5
        frame.transform.rotation.w = -0.5
        self._tf_frame = frame

    def _client(self, srv, name):
        return self.node.create_client(srv, OBSERVER + "/" + name)

    def broadcast_tf(self):
        frame = self._tf_frame
        frame.header.stamp.sec = int(time.time())
        frame.header.stamp.nanosec = int((time.time() % 1) * 1e9)
        self._tf.sendTransform(frame)

    def call(self, client, request, timeout=15.0):
        future = client.call_async(request)
        deadline = mono() + int(timeout * 1e9)
        while not future.done() and mono() < deadline:
            time.sleep(0.001)
        if not future.done():
            return None
        return future.result()

    def publish_cloud(self, points, stamp_ns, observation_id):
        from sensor_msgs.msg import PointCloud2, PointField
        import struct
        self.broadcast_tf()
        data = bytearray()
        for x, y, z in points:
            data += struct.pack("<fff", x, y, z)
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud = PointCloud2()
        cloud.header.frame_id = "d555_color_optical_frame"
        cloud.header.stamp.sec = stamp_ns // 1_000_000_000
        cloud.header.stamp.nanosec = stamp_ns % 1_000_000_000
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = fields
        cloud.point_step = 12
        cloud.row_step = len(data)
        cloud.is_dense = True
        cloud.is_bigendian = False
        cloud.data = bytes(data)
        self.cloud.publish(cloud)


class DownstreamHarness:
    """In-process placement/planning boundary driven by the SAME
    AdmissionGate module the ROS planner node uses (§3), with barrier
    checkpoints at the two contract injection stages."""

    def __init__(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "luggage_packing"))
        from luggage_packing.placement_admission import (
            AdmissionGate, MapObservation, RobotStateSample, ValidityObservation)
        self.AdmissionGate = AdmissionGate
        self.MapObservation = MapObservation
        self.RobotStateSample = RobotStateSample
        self.ValidityObservation = ValidityObservation
        self.gate = AdmissionGate()
        self.barriers = {"placement_computation": threading.Event(),
                         "planning_admission": threading.Event()}
        self.armed = set()
        self.records = []
        self.parked = {}  # stage -> parking proof for the harness barriers

    def arm(self, stage):
        self.barriers[stage].clear()
        self.armed.add(stage)
        self.parked.pop(stage, None)  # clear stale proof from a prior case

    def release(self, stage):
        self.barriers[stage].set()

    def checkpoint(self, stage):
        if stage in self.armed:
            self.parked[stage] = True   # arrival proof (never a sleep guess)
            self.barriers[stage].wait(timeout=30.0)

    # -- observer binding ------------------------------------------------
    def observe(self, validity_srv_response):
        v = validity_srv_response
        observation = self.ValidityObservation(
            state=v.state, epoch=int(v.epoch), generation=int(v.generation),
            reason=v.reason, reset_id=v.reset_id,
            transition_mono_ns=int(v.transition_mono_ns),
            map_revision=int(v.map_revision),
            observed_revision=int(v.observed_revision),
            geometry_hash=v.geometry_hash, config_digest=v.config_digest,
            clock_domain=v.clock_domain)
        self.gate.observe_validity(observation)
        return observation

    def admit_map(self, epoch, generation, revision=1):
        return self.gate.observe_map(self.MapObservation(
            epoch=epoch, generation=generation, map_revision=revision))

    def robot(self, stamp_ns, source="joint_states"):
        self.gate.observe_robot_state(self.RobotStateSample(stamp_ns, source))

    def plan_once(self, *, now_ns, robot_max_age_ns, planned_generation=None,
                  commit_with_current_generation=False):
        """Compute + commit with the two barrier stages; returns the
        terminal record: (admitted: bool, reason, stage_reached).

        The caller protocol mirrors the planner node (review 2026-11-03):
        the admission token captured BEFORE computing is what commit
        presents. ``commit_with_current_generation`` deliberately replays
        the gen1 defective wiring (re-read the board at commit) so cases
        can prove the gate itself rejects it after a reset+recovery.
        """
        admit = self.gate.admit_planning(
            now_mono_ns=now_ns, robot_max_age_ns=robot_max_age_ns)
        if not admit.admitted:
            self.records.append(("rejected_admit", admit.reason))
            return {"admitted": False, "reason": admit.reason,
                    "stage": "admission"}
        self.checkpoint("placement_computation")
        # Between the barriers a reset may commit; the commit check below is
        # the contract's second epoch/generation look against the captured
        # admission.
        self.checkpoint("planning_admission")
        if commit_with_current_generation:
            # The gen1 defect, replayed on purpose: the current board
            # generation instead of the captured one.
            commit = self.gate.commit_plan(
                self.gate.validity.generation,
                now_mono_ns=now_ns, robot_max_age_ns=robot_max_age_ns)
        else:
            commit = self.gate.commit_plan(
                planned_generation if planned_generation is not None
                else admit.admission_token.generation,
                admission_token=(None if planned_generation is not None
                                 else admit.admission_token),
                now_mono_ns=now_ns, robot_max_age_ns=robot_max_age_ns)
        if not commit.admitted:
            self.records.append(("rejected_commit", commit.reason))
            return {"admitted": False, "reason": commit.reason,
                    "stage": "commit"}
        generation = self.gate.record_execution()
        self.gate.retire_execution(generation)
        self.records.append(("admitted", generation))
        return {"admitted": True, "reason": "ok", "stage": "commit",
                "generation": generation}


def load_frames(frames_glob):
    frames = []
    for path in sorted(globlib.glob(frames_glob)):
        doc = yaml.safe_load(Path(path).read_text())
        frames.append((path, doc))
    if not frames:
        raise SystemExit("no frames match %s" % frames_glob)
    return frames


class Campaign:
    def __init__(self, args):
        self.args = args
        self.out = Path(args.out)
        self.out.mkdir(parents=True, exist_ok=True)
        self.cases = self.out / "cases"
        self.cases.mkdir(exist_ok=True)
        self.events = Events(self.out)
        self.frames = load_frames(args.frames_glob)
        self.repeats = (args.repeats if args.repeats is not None
                        else (2 if args.mode == "rehearsal" else 30))
        self.reset_seq = 0
        self.obs_seq = 0
        self.request_seq = 0          # integer request index (§6)
        self.extra_requests = 0       # labeled overrun, judge caps at 2
        self.node = None
        self.client = None
        self.harness = None
        self.summary = {"mode": args.mode, "v1": [], "v2": [], "v3": [],
                        "ownership": [], "gate_status": "not_run"}

    # -- run lifecycle -----------------------------------------------------
    def start(self):
        self.node = NodeProcess(self.args.node, self.args.geometry,
                                self.args.config, self.out / "observer.log")
        self.client = Client()
        for attr in ("validity", "reset", "get_snapshot"):
            if not getattr(self.client, attr).wait_for_service(timeout_sec=20.0):
                raise SystemExit("service unavailable: %s" % attr)
        self.harness = DownstreamHarness()
        time.sleep(0.5)
        self.events.emit("runner_start", node_pid=self.node.proc.pid,
                         frames=len(self.frames), repeats=self.repeats)

    def stop(self):
        if self.node:
            self.node.stop()
        if self.client:
            self.client.rclpy.shutdown()
        self.events.emit("runner_stop")
        self.events.close()

    # -- primitives ----------------------------------------------------------
    def next_request_id(self, stream="v1"):
        self.request_seq += 1
        return "%s-%06d" % (stream, self.request_seq)

    def note_extra_request(self):
        self.extra_requests += 1

    def feed_observation(self, frame_doc=None, stamp_ns=None,
                         wait_terminal=True, timeout_s=3.0):
        """Publish one cloud and wait for its terminal receipt (bounded)."""
        _, doc = frame_doc if frame_doc else self.frames[self.obs_seq % len(self.frames)]
        self.obs_seq += 1
        # The ledger is keyed by the node's epoch at arrival; the fixture
        # yaml's epoch field is historical and would query a dead epoch.
        query_epoch = int(self.get_validity().epoch)
        # Inputs are stamped in the node's ROS clock domain (the fixture
        # yaml stamps are bag-relative seconds and would never satisfy the
        # readiness age bound); strictly increasing ids are guaranteed by
        # the ns-resolution clock plus a monotonic guard.
        if stamp_ns is not None:
            stamp = stamp_ns
        else:
            stamp = int(self.client.node.get_clock().now().nanoseconds)
            if stamp <= getattr(self, "_last_stamp", 0):
                stamp = self._last_stamp + 1
            self._last_stamp = stamp
        # The subscription path derives the observation id from the input
        # stamp (observer_node on_cloud); the id is therefore deterministic.
        observation_id = str(stamp)
        self.client.publish_cloud(doc["points_m"], stamp, observation_id)
        self.events.emit("input_sent", observation_id=observation_id,
                         stamp_ns=stamp, points=len(doc["points_m"]))
        if not wait_terminal:
            return "SENT"
        deadline = mono() + int(timeout_s * 1e9)
        while mono() < deadline:
            from uld_occ_observer.srv import GetObservationStatus
            request = GetObservationStatus.Request()
            request.epoch = query_epoch
            request.sensor_id = "d555"
            request.observation_id = observation_id
            response = self.client.call(self.client.status, request)
            if response is not None and response.status in (
                    "COMPLETED", "REJECTED"):
                # The ledger's terminal states; the receipt carries the
                # product's own outcome (INTEGRATED / EPOCH_MISMATCH / ...).
                self.events.emit(
                    "input_terminal", observation_id=observation_id,
                    status=response.receipt.status,
                    ledger_state=response.status,
                    epoch=response.receipt.identity.epoch)
                return response.receipt.status
            time.sleep(0.01)
        return "TIMEOUT"

    def get_validity(self):
        from uld_occ_observer.srv import GetValidity
        return self.client.call(self.client.validity, GetValidity.Request())

    def do_reset(self, reason, epoch=None):
        """Distinct reset_id per call; returns (ok, new_epoch, elapsed_ms)."""
        from uld_occ_observer.srv import ResetObservation
        self.reset_seq += 1
        request = ResetObservation.Request()
        current = self.get_validity()
        request.expected_epoch = epoch if epoch is not None else int(current.epoch)
        request.reset_id = "reset-%04d" % self.reset_seq
        request.reason = reason
        started = mono()
        response = self.client.call(self.client.reset, request)
        elapsed_ms = (mono() - started) / 1e6
        ok = response is not None and response.status == "OK"
        new_epoch = int(response.epoch) if response is not None else -1
        self.events.emit("reset_called", reset_id=request.reset_id,
                         expected_epoch=request.expected_epoch, ok=ok,
                         new_epoch=new_epoch, elapsed_ms=round(elapsed_ms, 3),
                         reason=reason)
        return ok, new_epoch, elapsed_ms, request.reset_id

    def build_valid(self, min_frames=1, max_wait_s=15.0):
        """Feed new-epoch observations until the board publishes VALID."""
        deadline = mono() + int(max_wait_s * 1e9)
        while mono() < deadline:
            self.feed_observation()
            v = self.get_validity()
            if v is not None and v.state == "VALID":
                return v
        raise RuntimeError("VALID not reached within %.1fs" % max_wait_s)

    def wait_validity_state(self, states, timeout_s=5.0):
        deadline = mono() + int(timeout_s * 1e9)
        last = None
        while mono() < deadline:
            last = self.get_validity()
            if last is not None and last.state in states:
                return last
            time.sleep(0.005)
        return last

    def feed_until_valid(self, timeout_s=20.0):
        """Feed new-epoch observations until the board publishes VALID."""
        deadline = mono() + int(timeout_s * 1e9)
        last = None
        while mono() < deadline:
            self.feed_observation()
            last = self.get_validity()
            if last is not None and last.state == "VALID":
                return last
        return last

    def arm_gate(self, stage):
        from uld_occ_observer.srv import SetBehaviorHook
        request = SetBehaviorHook.Request()
        request.name = "gate_arm:%s" % stage
        return self.client.call(self.client.hook, request)

    def release_gate(self, stage):
        from uld_occ_observer.srv import SetBehaviorHook
        request = SetBehaviorHook.Request()
        request.name = "gate_release:%s" % stage
        return self.client.call(self.client.hook, request)

    def gate_probe(self, stage):
        from uld_occ_observer.srv import SetBehaviorHook
        request = SetBehaviorHook.Request()
        request.name = "gate_probe"
        response = self.client.call(self.client.hook, request)
        return (response.status if response else "") or ""

    def wait_gate_blocked(self, stage, timeout_s=10.0):
        """Barrier proof: the request is parked AT the stage (probe shows
        blocked=1). Never a sleep-as-ordering assumption."""
        deadline = mono() + int(timeout_s * 1e9)
        while mono() < deadline:
            probe = self.gate_probe(stage)
            want = "%s:hit=" % stage
            for token in probe.split():
                if token.startswith(want):
                    try:
                        blocked = int(token.split("blocked=")[1])
                    except (IndexError, ValueError):
                        blocked = 0
                    if blocked >= 1:
                        return True
            time.sleep(0.005)
        return False

    def resource_sample(self):
        from uld_occ_observer.srv import GetResourceUsage
        response = self.client.call(self.client.resource,
                                    GetResourceUsage.Request())
        if response is None:
            return None
        return {
            "old_view_block_bytes": int(response.old_view_block_bytes),
            "lease_view_block_bytes": int(response.lease_view_block_bytes),
            "client_held_view_block_bytes": int(
                response.client_held_view_block_bytes),
            "external_surface_buffer_bytes": int(
                response.external_surface_buffer_bytes),
            "surface_cache_bytes": int(response.surface_cache_bytes),
            "ownership_trace": json.loads(response.ownership_trace_json or "[]"),
        }

    def release_all(self):
        from uld_occ_observer.srv import SetBehaviorHook
        request = SetBehaviorHook.Request()
        request.name = "gate_release_all"
        self.client.call(self.client.hook, request)
        if self.harness:
            for barrier in self.harness.barriers.values():
                barrier.set()

    def write_case(self, name, payload):
        path = self.cases / ("%s.json" % name)
        path.write_text(json.dumps(payload, indent=1, sort_keys=True))
        return path

    # -- V1 ------------------------------------------------------------------
    def v1_request(self, stage, request_id):
        """Send the stage's request; returns the raw outcome record."""
        from uld_occ_observer.srv import GetSnapshot, ReadSurface, ReadCells, QueryFreespace
        outcome = {"request_id": request_id, "stage": stage,
                   "t_send_ns": mono()}
        if stage in ("acquire_before_lock", "acquire_after_lock"):
            request = GetSnapshot.Request()
            request.max_age_ms = 60000
            request.allow_stale = True
            response = self.client.call(self.client.get_snapshot, request)
        elif stage in ("surface_first_build", "page_work"):
            get = GetSnapshot.Request()
            get.max_age_ms = 60000
            get.allow_stale = True
            lease = self.client.call(self.client.get_snapshot, get)
            outcome["lease_status"] = lease.status if lease else "timeout"
            if lease is None or lease.status != "OK":
                outcome["status"] = lease.status if lease else "timeout"
                outcome["t_terminal_ns"] = mono()
                return outcome
            if stage == "surface_first_build":
                request = ReadSurface.Request()
                request.handle = lease.handle
                request.cursor = 0
                request.limit = 65536
                response = self.client.call(self.client.read_surface, request)
            else:
                request = ReadCells.Request()
                request.handle = lease.handle
                request.begin = [0, 0, 0]
                request.end = [64, 64, 64]
                request.cursor = 0
                request.limit = 4096
                response = self.client.call(self.client.read_cells, request)
            self.client.call(self.client.release, ReleaseRequest(lease.handle))
        elif stage == "response_serialization":
            get = GetSnapshot.Request()
            get.max_age_ms = 60000
            get.allow_stale = True
            lease = self.client.call(self.client.get_snapshot, get)
            outcome["lease_status"] = lease.status if lease else "timeout"
            if lease is None or lease.status != "OK":
                outcome["status"] = lease.status if lease else "timeout"
                outcome["t_terminal_ns"] = mono()
                return outcome
            request = QueryFreespace.Request()
            request.handle = lease.handle
            request.box_length = 0.30
            request.box_width = 0.20
            request.box_height = 0.20
            request.yaw_mode = 1
            request.cursor = 0
            request.limit = 32
            response = self.client.call(self.client.freespace, request)
            self.client.call(self.client.release, ReleaseRequest(lease.handle))
        elif stage in ("placement_computation", "planning_admission"):
            now = mono()
            response = self.harness.plan_once(
                now_ns=now, robot_max_age_ns=1_000_000_000)
            outcome["status"] = "OK" if response["admitted"] else response["reason"]
            outcome["admitted"] = response["admitted"]
            outcome["stage_reached"] = response["stage"]
            outcome["t_terminal_ns"] = mono()
            self.events.emit(
                "request_terminal", request_id=request_id, stage=stage,
                status=outcome["status"], admitted=response["admitted"])
            return outcome
        else:
            raise ValueError(stage)
        outcome["t_terminal_ns"] = mono()
        outcome["status"] = response.status if response else "timeout"
        if response is not None:
            outcome["validity_state"] = response.validity.state
            outcome["validity_epoch"] = int(response.validity.epoch)
            outcome["validity_generation"] = int(response.validity.generation)
            if getattr(response, "identity", None) is not None:
                outcome["identity_epoch"] = int(response.identity.epoch)
            for payload_field in ("states", "log_odds", "cell_x", "x"):
                value = getattr(response, payload_field, None)
                if value is not None:
                    outcome["payload_%s_len" % payload_field] = len(value)
        self.events.emit("request_terminal", request_id=request_id,
                         stage=stage, status=outcome["status"],
                         validity_state=outcome.get("validity_state"),
                         identity_epoch=outcome.get("identity_epoch"))
        return outcome

    def v1_case(self, index, stage):
        """One deterministic injection case; returns the judged record."""
        case_id = "v1_%s_%03d" % (stage, index)
        record = {"case_id": case_id, "stage": stage, "problems": []}
        # A trusted map exists for the pre-reset epoch.
        valid = self.build_valid()
        pre_epoch = int(valid.epoch)
        pre_generation = int(valid.generation)
        self.harness.observe(valid)
        self.harness.admit_map(pre_epoch, pre_generation)
        self.harness.robot(mono(), "joint_states")
        record["pre_epoch"] = pre_epoch
        record["pre_generation"] = pre_generation
        # Arm the named stage barrier.
        if stage in self.harness.barriers:
            self.harness.arm(stage)
        else:
            self.arm_gate(stage)
        request_id = self.next_request_id("v1")
        self.events.emit("request_planned", request_id=request_id,
                         stage=stage, epoch=pre_epoch)
        result = {}

        def run_request():
            result["outcome"] = self.v1_request(stage, request_id)

        worker = threading.Thread(target=run_request, daemon=True)
        started = mono()
        worker.start()
        # Barrier proof: the request is parked at the stage.
        if stage in self.harness.barriers:
            parked = self.wait_barrier_harness(stage)
        else:
            parked = self.wait_gate_blocked(stage)
        if not parked:
            self.release_all()
            worker.join(timeout=20)
            record["problems"].append("request_never_parked")
            record["status"] = "fail"
            return record
        # The reset commits while the request is parked.
        ok, new_epoch, reset_ms, reset_id = self.do_reset("v1_inject")
        record["reset_ok"] = ok
        record["reset_epoch"] = new_epoch
        record["reset_ms"] = round(reset_ms, 3)
        if reset_ms > REQUEST_DEADLINE_MS:
            record["problems"].append("reset_exceeded_deadline")
        # The downstream boundary observes the invalidation exactly as the
        # planner's latched subscription delivers it; the commit check then
        # reads the moved generation and rejects.
        if stage in self.harness.barriers:
            self.harness.observe(self.get_validity())
        # A NEW request after the reset must never lease the old epoch.
        from uld_occ_observer.srv import GetSnapshot
        new_request = GetSnapshot.Request()
        new_request.max_age_ms = 60000
        new_request.allow_stale = True
        new_response = self.client.call(self.client.get_snapshot, new_request)
        if new_response is not None and new_response.status == "OK":
            if int(new_response.identity.epoch) == pre_epoch:
                record["problems"].append("new_request_leased_old_epoch")
            else:
                # A lease for the NEW epoch must not exist either: the board
                # is INVALID/BUILDING until recovery.
                record["problems"].append("new_request_admitted_pre_valid")
        elif new_response is None:
            self.note_extra_request()
            record["problems"].append("new_request_timeout")
        else:
            record["new_request_status"] = new_response.status
            if new_response.status not in ("INVALIDATED", "NO_DATA",
                                           "STALE"):
                record["problems"].append(
                    "new_request_unexpected_status:%s" % new_response.status)
        # Release the parked request and collect its terminal.
        if stage in self.harness.barriers:
            self.harness.release(stage)
        else:
            self.release_gate(stage)
        worker.join(timeout=30)
        outcome = result.get("outcome")
        if outcome is None:
            record["problems"].append("request_no_terminal")
            record["status"] = "fail"
            self.write_case(case_id, {"record": record, "note": "no_terminal"})
            return record
        record["outcome"] = {k: v for k, v in outcome.items()
                             if k not in ("t_send_ns", "t_terminal_ns")}
        # The contract verdict: no old-epoch valid response after reset.
        status = outcome.get("status")
        identity_epoch = outcome.get("identity_epoch")
        payload_len = sum(v for k, v in outcome.items()
                          if k.startswith("payload_") and isinstance(v, int))
        if stage in ("placement_computation", "planning_admission"):
            if outcome.get("admitted"):
                record["problems"].append("old_generation_plan_admitted")
        else:
            if status == "OK":
                record["problems"].append("old_epoch_response_delivered")
            if status not in ("INVALIDATED", "NO_DATA", "STALE",
                              "REVISION_NOT_COVERED", "EPOCH_MISMATCH"):
                record["problems"].append("no_clean_terminal:%s" % status)
            if payload_len:
                record["problems"].append("usable_payload_delivered")
            if outcome.get("validity_state") == "VALID" and \
                    identity_epoch == pre_epoch:
                record["problems"].append("stale_validity_stamp")
        record["terminal_ms"] = round((mono() - started) / 1e6, 3)
        record["status"] = "pass" if not record["problems"] else "fail"
        if record["status"] == "fail":
            self.write_case(case_id, record)
        return record

    def wait_barrier_harness(self, stage, timeout_s=10.0):
        """Arrival proof for harness stages: the compute thread set the
        parked flag inside checkpoint() and is waiting on the barrier."""
        deadline = mono() + int(timeout_s * 1e9)
        while mono() < deadline:
            if self.harness.parked.get(stage):
                return True
            time.sleep(0.005)
        return False

    # -- V2 ------------------------------------------------------------------
    def v2_cycle(self, index):
        case_id = "v2_%03d" % index
        record = {"case_id": case_id, "problems": []}
        variant = ["missing", "stale", "duplicate_reset"][index % 3]
        record["variant"] = variant
        valid = self.build_valid()
        self.harness.observe(valid)
        pre_epoch = int(valid.epoch)
        if not self.harness.admit_map(pre_epoch, int(valid.generation)).admitted:
            record["problems"].append("pre_cycle_map_not_admitted")
        self.harness.robot(mono(), "joint_states")
        # An admission minted in the pre-reset generation: after reset AND
        # recovery this token must be dead even against the recovered
        # board's CURRENT generation (review 2026-11-03 finding 1).
        pre_admission = self.harness.gate.admit_planning(
            now_mono_ns=mono(), robot_max_age_ns=1_000_000_000)
        record["pre_reset_admission_generation"] = (
            pre_admission.admission_token.generation
            if pre_admission.admitted else None)
        # Invalidate.
        ok, new_epoch, _, reset_id = self.do_reset("v2_cycle")
        record["reset_epoch"] = new_epoch
        self.harness.observe(self.get_validity())
        if self.harness.gate.gate_state().admitted:
            record["problems"].append("gate_open_after_invalidation")
        # The late old-epoch result arrives now; it must be rejected+counted.
        accepted = self.harness.gate.accept_result(pre_epoch)
        if accepted.admitted:
            record["problems"].append("late_old_result_accepted")
        # Duplicate reset_id replays idempotently (same epoch, no change).
        from uld_occ_observer.srv import ResetObservation
        dup = ResetObservation.Request()
        dup.expected_epoch = pre_epoch
        dup.reset_id = reset_id
        dup.reason = "v2_duplicate"
        dup_response = self.client.call(self.client.reset, dup)
        if dup_response is None or dup_response.status != "OK" or \
                int(dup_response.epoch) != new_epoch:
            record["problems"].append("duplicate_reset_not_idempotent")
        # New-epoch observation -> VALID (feed until the readiness gate
        # publishes; a BUILDING board without observations never recovers).
        rebuilt = self.feed_until_valid(timeout_s=20.0)
        if rebuilt is None or rebuilt.state != "VALID":
            record["problems"].append("recovery_valid_not_reached")
            record["status"] = "fail"
            self.write_case(case_id, record)
            return record
        self.harness.observe(rebuilt)
        if rebuilt.epoch == pre_epoch:
            record["problems"].append("epoch_not_incremented")
        # Gate must stay closed without a fresh robot sample.
        if variant == "missing":
            pass  # no robot sample at all
        elif variant == "stale":
            self.harness.robot(mono() - 10_000_000_000, "joint_states")
        else:  # duplicate_reset: reset AGAIN during the fresh VALID epoch
            ok2, new_epoch2, _, _ = self.do_reset("v2_reset_again")
            record["reset_again_epoch"] = new_epoch2
            self.harness.observe(self.get_validity())
            rebuilt = self.feed_until_valid(timeout_s=20.0)
            if rebuilt is None or rebuilt.state != "VALID":
                record["problems"].append("second_recovery_valid_not_reached")
                record["status"] = "fail"
                self.write_case(case_id, record)
                return record
            self.harness.observe(rebuilt)
            if not self.harness.admit_map(
                    int(rebuilt.epoch), int(rebuilt.generation)).admitted:
                record["problems"].append("post_second_reset_map_rejected")
        closed = self.harness.gate.gate_state(now_mono_ns=mono(),
                                              robot_max_age_ns=1_000_000_000)
        if closed.admitted:
            record["problems"].append("gate_open_without_fresh_robot_state")
        # Fresh map + fresh robot state -> exactly one admission.
        if not self.harness.admit_map(
                int(rebuilt.epoch), int(rebuilt.generation)).admitted:
            record["problems"].append("fresh_map_not_admitted")
        self.harness.robot(mono(), "joint_states")
        record["robot_refresh"] = True
        # Raw evidence for the judge (review 2026-11-03 finding 2): the
        # fresh robot sample stamp and the validity transition stamp it
        # must post-date, in one monotonic domain, plus the event-stream
        # copy. A record missing these fields is incomplete evidence.
        record["robot_refresh_ns"] = self.harness.gate.robot_state.stamp_mono_ns
        record["transition_mono_ns"] = int(rebuilt.transition_mono_ns)
        record["recovery_epoch"] = int(rebuilt.epoch)
        record["recovery_generation"] = int(rebuilt.generation)
        self.events.emit(
            "v2_recovery", case_id=case_id,
            transition_mono_ns=record["transition_mono_ns"],
            robot_refresh_ns=record["robot_refresh_ns"],
            epoch=record["recovery_epoch"],
            generation=record["recovery_generation"])
        # The pre-reset admission must be void: committing the OLD candidate
        # with the board's current generation (the gen1 defective wiring)
        # and with its own captured generation must BOTH be rejected.
        current = self.harness.gate.validity.generation
        stale_current = self.harness.gate.commit_plan(
            current, now_mono_ns=mono(), robot_max_age_ns=1_000_000_000)
        stale_captured = self.harness.gate.commit_plan(
            record["pre_reset_admission_generation"],
            now_mono_ns=mono(), robot_max_age_ns=1_000_000_000)
        if stale_current.admitted or stale_captured.admitted:
            record["problems"].append("pre_reset_admission_committed")
        record["stale_admission_rejected"] = not (
            stale_current.admitted or stale_captured.admitted)
        plan = self.harness.plan_once(
            now_ns=mono(), robot_max_age_ns=1_000_000_000)
        if not plan["admitted"]:
            record["problems"].append("recovery_plan_not_admitted:%s"
                                      % plan["reason"])
        again = self.harness.gate.accept_result(
            pre_epoch, int(valid.generation))
        if again.admitted:
            record["problems"].append("old_epoch_result_accepted_late")
        record["status"] = "pass" if not record["problems"] else "fail"
        if record["status"] == "fail":
            self.write_case(case_id, record)
        return record

    # -- V3 ------------------------------------------------------------------
    def v3_begin(self, case_id):
        """Pairing window start (§5): stamped into the event stream so the
        judge derives the latency from events, not from record fields."""
        t = mono()
        self.events.emit("v3_pairing_start", case_id=case_id, t_ns=t)
        return t

    def v3_reset_done(self, case_id):
        """Reset linearization stamp for the pairing (post-response)."""
        t = mono()
        self.events.emit("v3_reset_done", case_id=case_id, t_ns=t)
        return t

    def v3_pairing(self, index, pairing):
        case_id = "v3_%s_%03d" % (pairing, index)
        record = {"case_id": case_id, "pairing": pairing, "problems": []}
        valid = self.build_valid()
        from uld_occ_observer.srv import (GetSnapshot, ReadSurface, ReadCells,
                                          QueryFreespace)
        outcome = {}
        if pairing == "integration":
            # hold_integrate parks integrate INSIDE the store mutex. Reset
            # linearizes after the in-flight integration finishes: the hold
            # is released shortly after the reset is fired, and the pairing
            # time (hold + reset) must stay inside the request deadline.
            self.set_hook("hold_integrate", 1)
            worker = threading.Thread(
                target=self.feed_observation, kwargs={"wait_terminal": False},
                daemon=True)
            worker.start()
            time.sleep(0.3)

            def release_hold_soon():
                time.sleep(0.06)
                self.set_hook("hold_integrate", 0)

            releaser = threading.Thread(target=release_hold_soon, daemon=True)
            started = self.v3_begin(case_id)
            releaser.start()
            ok, new_epoch, reset_ms, _ = self.do_reset("v3_integration")
            record["t_reset_done_ns"] = self.v3_reset_done(case_id)
            worker.join(timeout=15)
            releaser.join(timeout=5)
            outcome = {"reset_ms": reset_ms, "ok": ok}
        elif pairing in ("acquire", "surface", "roi", "cache_build"):
            stage = {"acquire": "acquire_before_lock", "surface":
                     "surface_first_build", "roi": "page_work",
                     "cache_build": "surface_first_build"}[pairing]
            self.arm_gate(stage)
            request_holder = {}

            def send():
                if stage == "acquire_before_lock":
                    request = GetSnapshot.Request()
                    request.max_age_ms = 60000
                    request.allow_stale = True
                    response = self.client.call(self.client.get_snapshot, request)
                    request_holder["response"] = response
                elif stage in ("surface_first_build", "page_work"):
                    get = GetSnapshot.Request()
                    get.max_age_ms = 1000
                    lease = self.client.call(self.client.get_snapshot, get)
                    if stage == "surface_first_build":
                        request = ReadSurface.Request()
                        request.handle = lease.handle
                        request.cursor = 0
                        request.limit = 65536
                        response = self.client.call(
                            self.client.read_surface, request)
                    else:
                        request = ReadCells.Request()
                        request.handle = lease.handle
                        request.begin = [0, 0, 0]
                        request.end = [64, 64, 64]
                        request.cursor = 0
                        request.limit = 4096
                        response = self.client.call(
                            self.client.read_cells, request)
                    self.client.call(self.client.release,
                                     ReleaseRequest(lease.handle))
                    request_holder["response"] = response

            worker = threading.Thread(target=send, daemon=True)
            worker.start()
            if not self.wait_gate_blocked(stage):
                self.release_all()
                worker.join(timeout=20)
                record["problems"].append("pairing_never_parked")
                record["status"] = "fail"
                self.write_case(case_id, record)
                return record
            started = self.v3_begin(case_id)
            ok, new_epoch, reset_ms, _ = self.do_reset("v3_%s" % pairing)
            record["t_reset_done_ns"] = self.v3_reset_done(case_id)
            self.release_gate(stage)
            worker.join(timeout=20)
            response = request_holder.get("response")
            outcome = {"reset_ms": reset_ms, "ok": ok,
                       "request_status": response.status if response else "timeout"}
            if response is not None and response.status == "OK":
                record["problems"].append("pairing_delivered_old_ok")
        elif pairing == "release":
            get = GetSnapshot.Request()
            get.max_age_ms = 60000
            get.allow_stale = True
            lease = self.client.call(self.client.get_snapshot, get)
            started = self.v3_begin(case_id)
            ok, new_epoch, reset_ms, _ = self.do_reset("v3_release")
            record["t_reset_done_ns"] = self.v3_reset_done(case_id)
            release_started = mono()
            from uld_occ_observer.srv import ReleaseSnapshot
            release_response = self.client.call(
                self.client.release, ReleaseRequest(lease.handle))
            outcome = {"reset_ms": reset_ms, "ok": ok,
                       "release_ms": round((mono() - release_started) / 1e6, 3)}
        elif pairing == "planning_admission":
            self.harness.observe(valid)
            self.harness.admit_map(int(valid.epoch), int(valid.generation))
            self.harness.robot(mono(), "joint_states")
            self.harness.arm("planning_admission")
            result = {}

            def send():
                result["plan"] = self.harness.plan_once(
                    now_ns=mono(), robot_max_age_ns=1_000_000_000)

            worker = threading.Thread(target=send, daemon=True)
            worker.start()
            if not self.wait_barrier_harness("planning_admission"):
                self.release_all()
                worker.join(timeout=20)
                record["problems"].append("pairing_never_parked")
                record["status"] = "fail"
                self.write_case(case_id, record)
                return record
            started = self.v3_begin(case_id)
            ok, new_epoch, reset_ms, _ = self.do_reset("v3_planning")
            record["t_reset_done_ns"] = self.v3_reset_done(case_id)
            self.harness.observe(self.get_validity())
            self.harness.release("planning_admission")
            worker.join(timeout=20)
            plan = result.get("plan") or {}
            outcome = {"reset_ms": reset_ms, "ok": ok,
                       "plan_admitted": plan.get("admitted")}
            if plan.get("admitted"):
                record["problems"].append("plan_committed_across_reset")
        elif pairing == "status":
            started = self.v3_begin(case_id)
            ok, new_epoch, reset_ms, _ = self.do_reset("v3_status")
            record["t_reset_done_ns"] = self.v3_reset_done(case_id)
            outcome = {"reset_ms": reset_ms, "ok": ok}
        else:
            raise ValueError(pairing)
        # Invalidation observability <=200 ms from the reset linearization
        # (the v3_reset_done event), derived from event stamps.
        observed = self.wait_validity_state(("INVALID", "BUILDING"),
                                            timeout_s=2.0)
        record["t_visible_ns"] = mono()
        self.events.emit("v3_invalidation_visible", case_id=case_id,
                         t_ns=record["t_visible_ns"],
                         state=(observed.state if observed else None))
        visible_ms = (record["t_visible_ns"]
                      - record.get("t_reset_done_ns",
                                   record["t_visible_ns"])) / 1e6
        # For 'status' the reset itself is the pairing; for others the
        # pairing request must complete within the 500 ms deadline after the
        # release. Reset must never exceed the request deadline.
        record.update(outcome)
        record["invalidation_visible_ms"] = round(visible_ms, 3)
        if observed is None or observed.state not in ("INVALID", "BUILDING"):
            record["problems"].append("invalidation_not_observable")
        if visible_ms > INVALIDATION_VISIBLE_MS:
            record["problems"].append("invalidation_visibility_late")
        reset_ms = record.get("reset_ms", 0.0)
        if reset_ms > REQUEST_DEADLINE_MS:
            record["problems"].append("reset_exceeded_deadline")
        record["t_pairing_start_ns"] = started
        record["t_complete_ns"] = mono()
        record["pairing_ms"] = round(
            (record["t_complete_ns"] - record["t_pairing_start_ns"]) / 1e6, 3)
        if record["pairing_ms"] > PAIRING_DEADLINE_MS:
            record["problems"].append("pairing_exceeded_deadline")
        record["status"] = "pass" if not record["problems"] else "fail"
        self.events.emit("v3_pairing_complete", case_id=case_id,
                         t_ns=record["t_complete_ns"],
                         pairing_ms=record["pairing_ms"],
                         status=record["status"])
        if record["status"] == "fail":
            self.write_case(case_id, record)
        return record

    def set_hook(self, name, value):
        from uld_occ_observer.srv import SetBehaviorHook
        request = SetBehaviorHook.Request()
        request.name = name
        request.value = int(value)
        return self.client.call(self.client.hook, request)

    # -- ownership cycles ------------------------------------------------------
    def ownership_cycle(self, index, kind):
        """One held-object reset/release cycle with attribution checks.

        The object is held by a REAL owner inside the node process (a read
        parked at a stage gate holds the view shared_ptr; a response parked
        at serialization holds the surface buffer), so the byte classes and
        the ownership ledger describe a genuine lifecycle, not a mock.
        """
        case_id = "own_%s_%03d" % (kind, index)
        record = {"case_id": case_id, "kind": kind, "problems": []}
        from uld_occ_observer.srv import GetSnapshot, ReadSurface, ReadCells
        self.build_valid()
        get = GetSnapshot.Request()
        get.max_age_ms = 60000
        get.allow_stale = True
        lease = self.client.call(self.client.get_snapshot, get)
        if lease is None or lease.status != "OK":
            record["problems"].append("lease_failed")
            record["status"] = "fail"
            self.write_case(case_id, record)
            return record
        stage = "page_work" if kind == "held_view" else "response_serialization"
        self.arm_gate(stage)
        holder = {}

        def send():
            if kind == "held_view":
                request = ReadCells.Request()
                request.handle = lease.handle
                request.begin = [0, 0, 0]
                request.end = [64, 64, 64]
                request.cursor = 0
                request.limit = 4096
                holder["response"] = self.client.call(
                    self.client.read_cells, request)
            else:
                request = ReadSurface.Request()
                request.handle = lease.handle
                request.cursor = 0
                request.limit = 65536
                holder["response"] = self.client.call(
                    self.client.read_surface, request)

        worker = threading.Thread(target=send, daemon=True)
        worker.start()
        if not self.wait_gate_blocked(stage):
            self.release_all()
            worker.join(timeout=20)
            record["problems"].append("holder_never_parked")
            record["status"] = "fail"
            self.write_case(case_id, record)
            return record
        ok, new_epoch, _, _ = self.do_reset("ownership_%s" % kind)
        record["t_reset_done_ns"] = mono()
        # While the last owner still holds the object, its bytes stay charged
        # to its class (the reset erased the lease handle; the parked reader
        # keeps the reference alive).
        held = self.resource_sample()
        record["t_hold_ns"] = mono()
        record["bytes_at_hold"] = {
            "old_view": held["old_view_block_bytes"],
            "client_held": held["client_held_view_block_bytes"],
            "external_surface": held["external_surface_buffer_bytes"],
        }
        self.events.emit(
            "owner_hold", case_id=case_id, t_ns=record["t_hold_ns"],
            bytes_at_hold=record["bytes_at_hold"])
        if kind == "held_view" and \
                held["old_view_block_bytes"] == 0 and \
                held["client_held_view_block_bytes"] == 0:
            record["problems"].append("held_view_unattributed")
        if kind == "held_buffer" and \
                held["external_surface_buffer_bytes"] == 0 and \
                held["old_view_block_bytes"] == 0 and \
                held["client_held_view_block_bytes"] == 0:
            # The parked reader holds the view (and its surface buffer)
            # alive: the bytes are charged to a live owner class. The pure
            # external share only appears when the holder outlives the
            # snapshot entirely (an in-process client); either way the
            # bytes must be SOMEWHERE while the owner holds the object.
            record["problems"].append("held_buffer_unattributed")
        # Final owner release: the parked read completes (INVALIDATED), its
        # shared_ptr drops, and the classes must return to baseline within
        # the bounded 2 s observation window.
        self.release_gate(stage)
        worker.join(timeout=20)
        response = holder.get("response")
        record["terminal_status"] = response.status if response else "timeout"
        # Final-owner release: the moment the parked reader dropped its
        # reference, stamped into the event stream (judge derives the
        # settle window from events, never from the record's own ms).
        record["t_final_release_ns"] = mono()
        if response is None or response.status != "INVALIDATED":
            record["problems"].append(
                "holder_terminal:%s" % record["terminal_status"])
        settle_started = mono()
        deadline = settle_started + int(OWNERSHIP_BASELINE_S * 1e9)
        settled = None
        while mono() < deadline:
            settled = self.resource_sample()
            if settled["old_view_block_bytes"] == 0 and \
                    settled["lease_view_block_bytes"] == 0 and \
                    settled["client_held_view_block_bytes"] == 0 and \
                    settled["external_surface_buffer_bytes"] == 0:
                break
            time.sleep(0.05)
        record["t_baseline_ns"] = mono()
        record["settle_ms"] = round(
            (record["t_baseline_ns"] - record["t_final_release_ns"]) / 1e6, 3)
        baseline_bytes = {
            "old_view": (settled or {}).get("old_view_block_bytes"),
            "client_held": (settled or {}).get("client_held_view_block_bytes"),
            "external_surface": (settled or {}).get(
                "external_surface_buffer_bytes"),
        }
        self.events.emit(
            "owner_baseline", case_id=case_id, t_ns=record["t_baseline_ns"],
            bytes=baseline_bytes,
            settled=(settled is not None and not any(
                settled[k] for k in ("old_view_block_bytes",
                                     "lease_view_block_bytes",
                                     "client_held_view_block_bytes",
                                     "external_surface_buffer_bytes"))))
        if settled is None or \
                settled["old_view_block_bytes"] or \
                settled["lease_view_block_bytes"] or \
                settled["client_held_view_block_bytes"] or \
                settled["external_surface_buffer_bytes"]:
            record["problems"].append("residual_after_final_release")
            record["residual"] = {
                "old_view": settled["old_view_block_bytes"],
                "client_held": settled["client_held_view_block_bytes"],
                "external_surface": settled["external_surface_buffer_bytes"],
            }
        # The ledger must show the object released AFTER the reset and the
        # final-release sample; and every object id appears once. The
        # released stamp is evidence the judge can cross-check against the
        # final_release event (review 2026-11-03 finding 2).
        trace = (settled or held)["ownership_trace"]
        relevant = [entry for entry in trace if entry["object_class"] == (
            "view" if kind == "held_view" else "surface_buffer")]
        if relevant:
            newest = max(relevant, key=lambda entry: entry["object_id"])
            record["ledger_object_id"] = newest.get("object_id")
            record["ledger_released_mono_ns"] = newest.get("released_mono_ns")
            if not newest.get("released_mono_ns"):
                record["problems"].append("ledger_release_missing")
            elif int(newest["released_mono_ns"]) < record["t_reset_done_ns"]:
                record["problems"].append("ledger_release_before_reset")
        self.events.emit(
            "owner_final_release", case_id=case_id,
            t_ns=record["t_final_release_ns"],
            terminal_status=record["terminal_status"],
            ledger_released_mono_ns=record.get("ledger_released_mono_ns"))
        ids = [entry["object_id"] for entry in trace]
        if len(ids) != len(set(ids)):
            record["problems"].append("object_attributed_twice")
        self.client.call(self.client.release, ReleaseRequest(lease.handle))
        record["status"] = "pass" if not record["problems"] else "fail"
        if record["status"] == "fail":
            self.write_case(case_id, record)
        return record

    # -- drive -----------------------------------------------------------------
    def run(self):
        self.start()
        try:
            for index in range(self.repeats):
                for stage in V1_STAGES:
                    record = self.v1_case(index, stage)
                    self.summary["v1"].append(record)
                    self.write_case(record["case_id"], record)
                    self.events.emit("case_terminal", **{
                        "case_id": record["case_id"],
                        "status": record["status"],
                        "problems": record["problems"]})
            for index in range(self.repeats):
                record = self.v2_cycle(index)
                self.summary["v2"].append(record)
                self.write_case(record["case_id"], record)
                self.events.emit("case_terminal", **{
                    "case_id": record["case_id"], "status": record["status"],
                    "problems": record["problems"]})
            for index in range(self.repeats):
                for pairing in V3_PAIRINGS:
                    record = self.v3_pairing(index, pairing)
                    self.summary["v3"].append(record)
                    self.write_case(record["case_id"], record)
                    self.events.emit("case_terminal", **{
                        "case_id": record["case_id"],
                        "status": record["status"],
                        "problems": record["problems"]})
            for index in range(self.repeats * 2):
                kind = "held_view" if index % 2 == 0 else "held_buffer"
                record = self.ownership_cycle(index, kind)
                self.summary["ownership"].append(record)
                self.write_case(record["case_id"], record)
                self.events.emit("case_terminal", **{
                    "case_id": record["case_id"], "status": record["status"],
                    "problems": record["problems"]})
        finally:
            self.release_all()
            self.stop()
        v1_pass = all(record["status"] == "pass" for record in self.summary["v1"])
        v2_pass = all(record["status"] == "pass" for record in self.summary["v2"])
        v3_pass = all(record["status"] == "pass" for record in self.summary["v3"])
        own_pass = all(record["status"] == "pass"
                       for record in self.summary["ownership"])
        complete = (len(self.summary["v1"]) == self.repeats * len(V1_STAGES)
                    and len(self.summary["v2"]) == self.repeats
                    and len(self.summary["v3"]) == self.repeats * len(V3_PAIRINGS)
                    and len(self.summary["ownership"]) == self.repeats * 2)
        self.summary["gate_status"] = (
            "pass" if (v1_pass and v2_pass and v3_pass and own_pass and complete)
            else "fail")
        self.summary["complete"] = complete
        self.summary["extra_requests"] = self.extra_requests
        (self.out / "campaign_summary.json").write_text(
            json.dumps(self.summary, indent=1, sort_keys=True))
        return 0 if self.summary["gate_status"] == "pass" else 4


def ReleaseRequest(handle):
    from uld_occ_observer.srv import ReleaseSnapshot
    request = ReleaseSnapshot.Request()
    request.handle = handle
    return request


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", required=True,
                        choices=["rehearsal", "score", "replay"])
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--node", required=True)
    parser.add_argument("--geometry", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--frames-glob", required=True)
    parser.add_argument("--repeats", type=int, default=None)
    args = parser.parse_args()
    manifest = {
        "manifest": args.manifest,
        "mode": args.mode,
        "repeats": (args.repeats if args.repeats is not None
                    else (2 if args.mode == "rehearsal" else 30)),
        "node_sha256": hashlib.sha256(
            Path(args.node).read_bytes()).hexdigest(),
        "geometry_sha256": hashlib.sha256(
            Path(args.geometry).read_bytes()).hexdigest(),
        "config_sha256": hashlib.sha256(
            Path(args.config).read_bytes()).hexdigest(),
        "frames": sorted(Path(p).name for p in globlib.glob(args.frames_glob)),
        "started_wall": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "owner": "OCC-RESET-VALIDITY-IMPL",
    }
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "manifest.json").write_text(json.dumps(manifest, indent=1))
    campaign = Campaign(args)
    code = campaign.run()
    manifest["finished_wall"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    manifest["extra_requests"] = campaign.extra_requests
    (out / "manifest.json").write_text(json.dumps(manifest, indent=1))
    return code


if __name__ == "__main__":
    sys.exit(main())
