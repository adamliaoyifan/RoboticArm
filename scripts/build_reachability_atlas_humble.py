#!/usr/bin/env python3
"""Build a reachability atlas against the Humble MoveIt 2 planning scene.

The atlas builder speaks rospy. This host has Humble only, so the script
installs a small rospy shim over one rclpy node and then runs the existing
builder. Pose IK stays in offline_ik; collision stays on /check_state_validity.

The Gazebo + move_group + scene_manager stack must already be up.
"""

from __future__ import division

import argparse
import math
import os
import sys
import time
import types

import numpy as np
import yaml

WS = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
PKG = os.path.join(WS, "src", "luggage_planning")
if PKG not in sys.path:
    sys.path.insert(0, PKG)
if os.path.join(WS, "src", "luggage_description") not in sys.path:
    sys.path.insert(0, os.path.join(WS, "src", "luggage_description"))


def _install_rospy(node, params):
    import rclpy
    from moveit_msgs.srv import GetStateValidity

    rospy = types.ModuleType("rospy")

    class ROSException(Exception):
        pass

    class ServiceException(Exception):
        pass

    class Duration(object):
        def __init__(self, secs=0.0):
            self.secs = float(secs)

    class Time(object):
        def __init__(self, secs=0.0):
            self.secs = float(secs)

        @staticmethod
        def now():
            return Time(time.time())

    class ServiceProxy(object):
        def __init__(self, name, srv_type):
            self._name = name
            self._type = srv_type
            self._client = node.create_client(srv_type, name)

        def wait_for_service(self, timeout=60.0):
            if not self._client.wait_for_service(timeout_sec=float(timeout)):
                raise ROSException("%s unavailable" % self._name)

        def __call__(self, req=None):
            if req is None:
                req = self._type.Request()
            self.wait_for_service(60.0)
            future = self._client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
            if future.result() is None:
                raise ServiceException(
                    "%s failed: %s" % (self._name, future.exception()))
            return future.result()

    def get_param(name, default=None):
        if name in params:
            return params[name]
        return default

    def log(level, fmt, args):
        text = fmt % args if args else fmt
        print("[%s] %s" % (level, text), flush=True)

    def wait_for_service(name, timeout=60.0):
        client = node.create_client(GetStateValidity, name)
        if not client.wait_for_service(timeout_sec=float(timeout)):
            raise ROSException("service %s not ready" % name)
        node.destroy_client(client)

    rospy.ROSException = ROSException
    rospy.ServiceException = ServiceException
    rospy.Duration = Duration
    rospy.Time = Time
    rospy.ServiceProxy = ServiceProxy
    rospy.get_param = get_param
    rospy.wait_for_service = wait_for_service
    rospy.init_node = lambda name: None
    rospy.sleep = time.sleep
    rospy.loginfo = lambda fmt, *args: log("INFO", fmt, args)
    rospy.logwarn = lambda fmt, *args: log("WARN", fmt, args)
    rospy.logerr = lambda fmt, *args: log("ERROR", fmt, args)
    rospy.logwarn_throttle = lambda _period, fmt, *args: log("WARN", fmt, args)
    sys.modules["rospy"] = rospy
    return rospy


def _detach_payload(node):
    """Drop a payload left by an earlier build so the next scene is empty-load."""
    import rclpy
    from moveit_msgs.srv import ApplyPlanningScene
    from luggage_planning.motion.planning_scene_client import build_detach_scene
    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not client.wait_for_service(timeout_sec=15.0):
        print("payload detach: apply_planning_scene unavailable", flush=True)
        return
    request = ApplyPlanningScene.Request()
    request.scene = build_detach_scene("atlas_payload", "suction_contact_frame")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
    result = future.result()
    print("payload detach: %s" % (
        getattr(result, "success", None) if result is not None else future.exception()),
          flush=True)


def _sync_scene(node):
    import rclpy
    from luggage_msgs.srv import SyncStaticScene
    client = node.create_client(SyncStaticScene, "/scene_manager/sync_static_scene")
    if not client.wait_for_service(timeout_sec=60.0):
        raise RuntimeError("scene_manager/sync_static_scene unavailable")
    future = client.call_async(SyncStaticScene.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=60.0)
    result = future.result()
    if result is None or not result.success:
        message = getattr(result, "message", None) or future.exception()
        raise RuntimeError("scene sync failed: %s" % message)
    print("scene sync: %s" % result.message, flush=True)


def _robot_description(node):
    from rcl_interfaces.srv import GetParameters
    client = node.create_client(
        GetParameters, "/robot_state_publisher/get_parameters")
    if not client.wait_for_service(timeout_sec=10.0):
        return ""
    request = GetParameters.Request()
    request.names = ["robot_description"]
    import rclpy
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    result = future.result()
    if result is None or not result.values:
        return ""
    return str(result.values[0].string_value or "")


def layer_rows(npz_path, meta_path):
    data = np.load(npz_path)
    with open(meta_path, "r", encoding="utf-8") as stream:
        meta = yaml.safe_load(stream)
    status = data["status"]
    connected = data["opening_connected"]
    origin_z = float(meta["grid"]["origin"][2])
    resolution = float(meta["grid"]["resolution_xyz"])
    rows = []
    for iz in range(status.shape[2]):
        slab = status[:, :, iz, :]
        rows.append({
            "iz": iz,
            "z": origin_z + (iz + 0.5) * resolution,
            "reachable": int(np.count_nonzero(slab == 3)),
            "marginal": int(np.count_nonzero(slab == 2)),
            "unreachable": int(np.count_nonzero(slab == 1)),
            "unknown": int(np.count_nonzero(slab == 0)),
            "connected": int(np.count_nonzero(connected[:, :, iz, :])),
        })
    data.close()
    return rows, meta


def _print_rows(title, rows):
    print(title, flush=True)
    print("iz    z   REACH MARG UNREACH UNKNOWN CONNECTED", flush=True)
    for row in rows:
        print("%2d %6.3f %5d %4d %7d %7d %9d" % (
            row["iz"], row["z"], row["reachable"], row["marginal"],
            row["unreachable"], row["unknown"], row["connected"]), flush=True)


def _gate_yaw_map(old_meta_path, new_meta_path):
    """Old bin index -> new bin index across the v1/v3 yaw schema gap.

    v1 bins are base-frame yaws; v3 bins are DIRECTED container-frame yaws
    (base = container - pi/2 on the sim scene). Comparing cells index-for-
    index would test a different branch. Returns {old_iyaw: new_iyaw};
    unmapped old bins are reported by the caller, never silently skipped.
    """
    with open(old_meta_path, "r", encoding="utf-8") as stream:
        old_meta = yaml.safe_load(stream)
    with open(new_meta_path, "r", encoding="utf-8") as stream:
        new_meta = yaml.safe_load(stream)
    old_bins = [float(v) for v in old_meta["grid"]["yaw_bins"]]
    new_bins = [float(v) for v in new_meta["grid"]["yaw_bins"]]
    offset = 0.0
    old_contract = old_meta.get("contract")
    new_contract = new_meta.get("contract")
    if old_contract is None and new_contract is not None:
        # v1 base-frame bins -> container frame via the scene container yaw.
        rpy = [float(v) for v in new_meta["container"]["base_rpy"]]
        offset = -math.atan2(math.sin(rpy[2]), math.cos(rpy[2]))
    mapping = {}
    for old_i, old_yaw in enumerate(old_bins):
        target = (old_yaw + offset) % (2.0 * math.pi)
        for new_i, new_yaw in enumerate(new_bins):
            delta = abs((target - (new_yaw % (2.0 * math.pi)) + math.pi)
                        % (2.0 * math.pi) - math.pi)
            # Scene configs store rounded yaws (-1.5708 vs -pi/2 ~ 3.7e-6
            # off); 1e-3 still sits far below the pi/2 bin spacing.
            if delta < 1e-3:
                mapping[old_i] = new_i
                break
    return mapping


def unexplained_low_layer_drops(builder, old_npz, new_npz,
                                old_meta_path, new_meta_path):
    """Cells in iz 0-4 that lost a solution while the old seed is still valid."""
    old = np.load(old_npz)
    new = np.load(new_npz)
    old_status = old["status"]
    new_status = new["status"]
    seeds = old["contact_seeds"] if "contact_seeds" in old.files else None
    if seeds is None:
        seeds = old["seed_joints"][..., None, :]
    yaw_map = _gate_yaw_map(old_meta_path, new_meta_path)
    dropped = []
    limit = min(5, old_status.shape[2], new_status.shape[2])
    for iz in range(limit):
        for ix in range(old_status.shape[0]):
            for iy in range(old_status.shape[1]):
                for iyaw in range(old_status.shape[3]):
                    was = int(old_status[ix, iy, iz, iyaw]) in (2, 3)
                    if not was:
                        continue
                    if iyaw not in yaw_map:
                        dropped.append((ix, iy, iz, iyaw, "yaw_bin_unmapped"))
                        continue
                    still = int(
                        new_status[ix, iy, iz, yaw_map[iyaw]]) in (2, 3)
                    if still:
                        continue
                    seed = np.asarray(seeds[ix, iy, iz, iyaw, 0], dtype=float)
                    if not np.all(np.isfinite(seed)):
                        dropped.append((ix, iy, iz, iyaw, "no_old_seed"))
                        continue
                    valid = builder._state_is_valid(seed.tolist())
                    if valid is not False:
                        dropped.append((ix, iy, iz, iyaw, "still_valid" if valid else "validity_unknown"))
    old.close()
    new.close()
    return dropped


def gate(old_base, new_npz, new_meta, builder):
    old_npz = old_base + ".npz"
    old_meta = old_base + ".yaml"
    old_rows, _old = layer_rows(old_npz, old_meta)
    new_rows, new_meta_doc = layer_rows(new_npz, new_meta)
    _print_rows("old", old_rows)
    _print_rows("new", new_rows)
    failures = []
    for old, new in zip(old_rows, new_rows):
        iz = old["iz"]
        # Not-regress: a semantics-preserving directed-bin rebuild keeps the
        # mapped bins identical, so equal counts pass; only a loss fails.
        if iz in (5, 6) and new["connected"] < old["connected"]:
            failures.append(
                "iz%d connected %d -> %d" % (iz, old["connected"], new["connected"]))
        if new["z"] >= 1.70 and new["reachable"] != 0:
            failures.append("iz%d z=%.3f has REACHABLE cells" % (iz, new["z"]))
    drops = unexplained_low_layer_drops(
        builder, old_npz, new_npz,
        old_base + ".yaml", new_meta)
    if drops:
        failures.append("unexplained low-layer drops: %d" % len(drops))
        print("unexplained low-layer drops: %d" % len(drops), flush=True)
        for item in drops[:20]:
            print("  %s" % (item,), flush=True)
    else:
        print("low-layer drops are collision on the stored seed, or there were none",
              flush=True)
    if len(new_rows) > 7:
        row = new_rows[7]
        print("iz7 report only: connected=%d reachable=%d unknown=%d z=%.3f" % (
            row["connected"], row["reachable"], row["unknown"], row["z"]), flush=True)
    fingerprint = new_meta_doc.get("builder", {}).get("deterministic_fingerprint")
    return failures, fingerprint


def build_one(node, output_dir, payload=None, scene_tf=""):
    params = {
        "~output_dir": output_dir,
        "~resolution_xyz": 0.15,
        "~avoid_collisions": True,
        "~tool_down_tolerance": 0.0,
        "~transit_clearance": 0.30,
        "~safe_tool_z": 1.55,
        "~min_lift": 0.06,
        "~atlas_prefix": "s20",
        # ST06/F14: four DIRECTED container-frame concrete-tool bins
        # (0 and pi distinct, 2pi period). The v1 [0, pi/2] pi-folded
        # sampling is not relabelled -- every profile is rebuilt.
        "~yaw_bins": [0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0],
        "~payload_enabled": payload is not None,
    }
    if scene_tf:
        params["~scene_tf_config"] = scene_tf
    description = _robot_description(node)
    if description:
        params["/robot_description"] = description
        # ST06: the exact robot_description bytes the stack built against.
        # The FK certifier md5s this capture and compares it with the
        # artifact contract urdf_hash (model identity, not a relabel).
        capture = os.path.join(output_dir, "robot_description.xml")
        with open(capture, "w", encoding="utf-8") as handle:
            handle.write(description)
    if payload is not None:
        length, width, height = payload
        params["~payload_size"] = [length, width, height]
        params["~payload_offset"] = [0.0, 0.0, height * 0.5]
        params["~payload_attach_link"] = "suction_contact_frame"
    _install_rospy(node, params)
    # Import after the shim so the builder binds this rospy, not a missing one.
    if "reachability_atlas_builder" in sys.modules:
        del sys.modules["reachability_atlas_builder"]
    scripts = os.path.join(PKG, "scripts")
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    from reachability_atlas_builder import ReachabilityAtlasBuilder
    builder = ReachabilityAtlasBuilder()
    builder._rclpy_node = node
    builder._scene_already_synced = True
    builder.build()
    return builder


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    parser.add_argument(
        "--scene-tf", default="",
        help="scene_tf yaml baked into IK and the atlas hash. "
             "Empty keeps the builder default.")
    parser.add_argument(
        "--old-atlas", default="",
        help="Prefix path without extension of the atlas to compare. "
             "Required with --gate.")
    parser.add_argument("--payload", default="",
                        help="l,w,h metres. Empty builds the empty-load atlas.")
    parser.add_argument("--gate", action="store_true")
    args = parser.parse_args()
    if args.gate and not args.old_atlas:
        parser.error("--old-atlas is required with --gate")
    import rclpy
    rclpy.init()
    node = rclpy.create_node("reachability_atlas_build_humble")
    try:
        _sync_scene(node)
        _detach_payload(node)
        payload = None
        if args.payload:
            payload = tuple(float(part) for part in args.payload.split(","))
        os.makedirs(args.output_dir, exist_ok=True)
        builder = build_one(
            node, args.output_dir, payload, scene_tf=args.scene_tf)
        if not args.gate:
            return 0
        # Mirror the builder's file naming so payload builds gate their own
        # artifact, not the stale empty-load one.
        suffix = "collision_aware"
        if payload is not None:
            suffix += "_payload_%.2fx%.2fx%.2f" % tuple(payload)
        npz = os.path.join(
            args.output_dir, "s20_container_%s.npz" % suffix)
        meta = os.path.join(
            args.output_dir, "s20_container_%s.yaml" % suffix)
        failures, fingerprint = gate(args.old_atlas, npz, meta, builder)
        print("fingerprint %s" % fingerprint, flush=True)
        if failures:
            print("GATE FAIL", flush=True)
            for item in failures:
                print("  %s" % item, flush=True)
            return 1
        print("GATE PASS", flush=True)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
