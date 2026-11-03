#!/usr/bin/env python3
"""Observer -> placement surface_map_2d bridge (v1 JSON contract).

Bridges the observer's immutable snapshot surface (read_surface chain) into
the existing placement solver's surface_map_2d v1 dict, published as
std_msgs/String JSON on a latched reliable topic. The placement policy is
NOT changed: this bridge only translates observed state.

Contract notes (docs/architecture/placement.md, frame_contracts.md):
  - height is floor-relative (solver floor_z=0.0 view); the container-frame
    floor/ceiling travel through center_base exactly as the mapper's v1 map.
  - an unobserved column keeps state "unknown" with height 0.0 -- unknown is
    never reported as free floor; the solver's floor-prior policy stays the
    only thing allowed to answer on unknown columns.
  - geometry_hash travels from the snapshot identity; a mismatch against the
    bridge's geometry file fails the map closed (empty dict + error status)
    rather than falling back to a prior.
  - epoch/observed_revision/last_integrated_stamp_ns are ADDITIVE keys: the
    existing solver ignores them, and a downstream epoch-aware consumer can
    distinguish a post-reset revision restart from a stale map (a monotonic
    map_revision check alone would misread a reset).

The pure builder (build_surface_map) has no ROS dependency so the consumer
test can drive it directly; this file only adds the transport shell.
"""
from __future__ import annotations

import json

import yaml

BRIDGE_VERSION = "occ_surface_bridge_v1"


def load_geometry(geometry_path):
    document = yaml.safe_load(open(geometry_path).read())
    return json.loads(document["descriptor_json"])


def build_surface_map(columns, geometry, identity):
    """v1 surface_map_2d dict from read_surface columns and a snapshot identity.

    columns: iterable of dicts with x, y, active_count, known_count,
    has_occupied, top_z (world ULD z). Fail-closed on identity mismatch.
    """
    if identity.get("geometry_hash") != geometry["geometry_hash"]:
        return {"error": "CARGO_MAP_GEOMETRY_MISMATCH",
                "geometry_hash": identity.get("geometry_hash"),
                "expected": geometry["geometry_hash"],
                "bridge_version": BRIDGE_VERSION}
    resolution = geometry.get("_resolution")
    if resolution is None:
        raise ValueError("resolution not pinned for the bridge (config resolution_m)")
    nx = ny = 0
    by_index = {}
    for column in columns:
        x, y = int(column["x"]), int(column["y"])
        by_index[(x, y)] = column
        nx = max(nx, x + 1)
        ny = max(ny, y + 1)
    if not by_index:
        return {"error": "NO_DATA", "bridge_version": BRIDGE_VERSION}
    floor_z = float(geometry["floor_z"])
    ceiling_z = float(geometry["ceiling_z"])
    inner_h = ceiling_z - floor_z
    height = [[0.0] * ny for _ in range(nx)]
    state = [["unknown"] * ny for _ in range(nx)]
    confidence = [["none"] * ny for _ in range(nx)]
    for (x, y), column in by_index.items():
        if not column["known_count"]:
            continue  # unobserved column stays unknown/none/0.0
        if column["has_occupied"]:
            height[x][y] = float(column["top_z"]) - floor_z
            state[x][y] = "occupied"
        else:
            state[x][y] = "free"
        confidence[x][y] = "sensor"
    return {
        "bridge_version": BRIDGE_VERSION,
        "resolution": float(resolution),
        "nx": nx,
        "ny": ny,
        "inner_size": [float(geometry["length"]), float(geometry["width"]), inner_h],
        "floor_z": 0.0,
        "center_base": [0.0, 0.0, floor_z + inner_h / 2.0],
        "yaw": 0.0,  # scalar: generate_candidates does float(surface_map["yaw"])
        "height": height,
        "state": state,
        "confidence": confidence,
        "geometry_hash": identity["geometry_hash"],
        "map_revision": int(identity.get("map_revision", 0)),
        "epoch": int(identity.get("epoch", 0)),
        # Validity generation the whole read chain crossed (reset contract):
        # the planner admission binds map -> generation, not epoch alone.
        "validity_generation": int(identity.get("validity_generation", 0)),
        "observed_revision": int(identity.get("observed_revision", 0)),
        "last_integrated_stamp_ns": int(identity.get("last_integrated_stamp_ns", 0)),
        "clock_domain": identity.get("clock_domain", ""),
    }


def collect_columns(observer_prefix, timeout_s=5.0):
    """read_surface chain via rclpy; returns (columns, identity) or raises."""
    import rclpy
    from rclpy.node import Node
    from uld_occ_observer.srv import GetSnapshot, ReadSurface, ReleaseSnapshot

    node = Node("occ_surface_bridge_fetch")
    try:
        get_snapshot = node.create_client(
            GetSnapshot, observer_prefix + "/get_snapshot")
        read_surface = node.create_client(
            ReadSurface, observer_prefix + "/read_surface")
        release = node.create_client(
            ReleaseSnapshot, observer_prefix + "/release_snapshot")
        for client in (get_snapshot, read_surface, release):
            if not client.wait_for_service(timeout_sec=timeout_s):
                raise RuntimeError("service unavailable: %s" % client.srv_name)
        request = GetSnapshot.Request()
        request.max_age_ms = 1000
        future = get_snapshot.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
        lease = future.result()
        if lease is None or lease.status != "OK":
            raise RuntimeError("get_snapshot: %s" % (lease.status if lease else "timeout"))
        # Reset-contract binding: the published map carries the validity
        # generation the whole read chain crossed (lease + every page's
        # commit check), so the planner admission can reject a page that lost
        # a reset race even if its epoch field looks current.
        lease_generation = lease.validity.generation
        last_page_generation = lease_generation
        columns = []
        try:
            cursor, done = 0, False
            while not done:
                page = ReadSurface.Request()
                page.handle = lease.handle
                page.cursor = cursor
                page.limit = 65536
                future = read_surface.call_async(page)
                rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
                response = future.result()
                if response is None or response.status != "OK":
                    raise RuntimeError("read_surface: %s"
                                       % (response.status if response else "timeout"))
                if response.validity.state != "VALID":
                    raise RuntimeError("read_surface invalidated: %s"
                                       % response.validity.state)
                last_page_generation = response.validity.generation
                identity = {
                    "epoch": response.identity.epoch,
                    "map_revision": response.identity.map_revision,
                    "observed_revision": response.identity.observed_revision,
                    "last_integrated_stamp_ns": response.identity.last_integrated_stamp_ns,
                    "geometry_hash": response.identity.geometry_hash,
                    "config_digest": response.identity.config_digest,
                    "clock_domain": response.identity.clock_domain,
                }
                for i in range(len(response.x)):
                    columns.append({
                        "x": response.x[i], "y": response.y[i],
                        "active_count": response.active_count[i],
                        "known_count": response.known_count[i],
                        "has_occupied": response.has_occupied[i],
                        "top_z": response.top_z[i],
                    })
                cursor, done = response.next_cursor, response.done
        finally:
            release.call_async(ReleaseSnapshot.Request(handle=lease.handle))
        # A page that crossed a generation change mid-walk is not one map.
        if last_page_generation != lease_generation:
            raise RuntimeError("validity generation changed mid-read: %s -> %s"
                               % (lease_generation, last_page_generation))
        identity["validity_generation"] = last_page_generation
        return columns, identity
    finally:
        node.destroy_node()


def main():
    import argparse
    import sys
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--geometry", required=True,
                        help="geometry export yaml (descriptor + hash)")
    parser.add_argument("--config", required=True,
                        help="observer config yaml (resolution_m)")
    parser.add_argument("--observer-prefix", default="/uld_occ_observer")
    parser.add_argument("--topic", default="surface_2d")
    parser.add_argument("--hz", type=float, default=1.0)
    args = parser.parse_args()

    geometry = load_geometry(args.geometry)
    config = yaml.safe_load(open(args.config).read())
    geometry["_resolution"] = float(config["resolution_m"])

    rclpy.init()
    node = rclpy.create_node("occ_surface_bridge")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(String, args.topic, qos)
    import time
    period = 1.0 / max(args.hz, 1e-6)
    next_publish = time.monotonic()
    try:
        while rclpy.ok():
            now = time.monotonic()
            if now < next_publish:
                time.sleep(min(0.05, next_publish - now))
                continue
            next_publish += period
            try:
                columns, identity = collect_columns(args.observer_prefix)
                surface_map = build_surface_map(columns, geometry, identity)
                message = String()
                message.data = json.dumps(surface_map)
                publisher.publish(message)
                node.get_logger().info(
                    "published revision %s epoch %s (%s columns)"
                    % (surface_map.get("map_revision"), surface_map.get("epoch"),
                       len(columns)))
            except Exception as error:  # noqa: BLE001 - bridge stays alive
                node.get_logger().error("surface fetch failed: %s" % error)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
