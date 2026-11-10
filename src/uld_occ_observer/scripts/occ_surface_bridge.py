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
import sys

import yaml

BRIDGE_VERSION = "occ_surface_bridge_v2"  # v2: conservative free (known == active)


def load_geometry(geometry_path):
    document = yaml.safe_load(open(geometry_path).read())
    return json.loads(document["descriptor_json"])


def build_surface_v2_record(v1_map, geometry, identity, scene_revision,
                            request_id):
    """SurfaceContractV2 record equivalent to the v1 view (G1, reviews pt 5).

    Productizes the audited v1->v2 conversion (test_frame_migration_placement
    round-trips): unobserved cells are unknown/unknown with a null top;
    the v1 solver's floor-relative heights become CONTAINER-ORIGIN z
    (floor_z_container = the geometry's container floor, not the v1
    convention's 0.0 — the typed contract's datum is container_link);
    origin_xy is the lower-left corner; identity is carried losslessly
    (epoch int -> the wire's string field), including the validity
    generation the read chain crossed.
    """
    floor = float(geometry.get("floor_z", 0.0) or 0.0)
    inner_l = float(v1_map["inner_size"][0])
    inner_w = float(v1_map["inner_size"][1])
    inner_h = float(v1_map["inner_size"][2])
    surface_z, state, confidence = [], [], []
    for ix in range(v1_map["nx"]):
        z_row, s_row, c_row = [], [], []
        for iy in range(v1_map["ny"]):
            cell_state = v1_map["state"][ix][iy]
            if cell_state == "unknown":
                z_row.append(None)
                s_row.append("unknown")
                c_row.append("unknown")
            else:
                z_row.append(floor + float(v1_map["height"][ix][iy]))
                s_row.append(cell_state)
                c_row.append("verified_geometry"
                             if v1_map["confidence"][ix][iy] == "geometry"
                             else "sensor")
        surface_z.append(z_row)
        state.append(s_row)
        confidence.append(c_row)
    return {
        "contract_version": 2,
        "frame_id": "container_link",
        "length_unit": "m",
        "stamp_ns": 0,
        "time_basis": "scene",
        "scene_revision": str(scene_revision),
        "request_id": str(request_id),
        "kind": "surface",
        "map_revision": int(v1_map.get("map_revision", 0) or 0),
        "geometry_hash": v1_map["geometry_hash"],
        "epoch": str(v1_map.get("epoch", "") or ""),
        "content_digest": "",
        "occupancy_confirmed": False,
        "change_event": "",
        "validity_generation": int(
            v1_map.get("validity_generation", 0) or 0),
        "height_datum": "container_origin",
        "origin_xy_container": [-0.5 * inner_l, -0.5 * inner_w],
        "resolution_m": float(v1_map["resolution"]),
        "floor_z_container": floor,
        "ceiling_z_container": floor + inner_h,
        "surface_z_container": surface_z,
        "state": state,
        "confidence": confidence,
        "height_above_floor": [
            [None if z is None else z - floor for z in row]
            for row in surface_z],
    }


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
            confidence[x][y] = "sensor"
            continue
        # Conservative free (adjudication 2026-11-08): free ONLY when
        # every ACTIVE voxel of the column is observed and none is
        # occupied. A partially-known column still holds unobserved
        # volume; the surface label feeds placement admission as volume
        # evidence and surface-known must never be promoted to
        # volume-free (known_count >= 1 is not enough).
        if (0 < column["known_count"]
                and column["known_count"] == column["active_count"]):
            state[x][y] = "free"
            confidence[x][y] = "sensor"
        # partial knowledge stays unknown/none — fail-closed
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
        # Full hull descriptor VERBATIM (chamfer hulls carry the
        # chamfer block; readers validate the complete geometry).
        "geometry_descriptor": {
            k: v for k, v in geometry.items()
            if not k.startswith("_")},
        "map_revision": int(identity.get("map_revision", 0)),
        "epoch": int(identity.get("epoch", 0)),
        # Validity generation the whole read chain crossed (reset contract):
        # the planner admission binds map -> generation, not epoch alone.
        "validity_generation": int(identity.get("validity_generation", 0)),
        "observed_revision": int(identity.get("observed_revision", 0)),
        "last_integrated_stamp_ns": int(identity.get("last_integrated_stamp_ns", 0)),
        "clock_domain": identity.get("clock_domain", ""),
    }


def collect_columns(observer_prefix, timeout_s=5.0, max_age_ms=1000):
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
        request.max_age_ms = int(max_age_ms)
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
    parser.add_argument("--publish-v2", action="store_true",
                        help="also publish the typed SurfaceContractV2 "
                             "stream (lower-left origin, container-origin "
                             "z datum, lossless epoch/generation)")
    parser.add_argument("--scene-revision", default="",
                        help="scene revision for the v2 contract identity "
                             "(required with --publish-v2)")
    parser.add_argument("--v2-topic", default="/luggage/cargo_map/surface_v2")
    parser.add_argument("--max-age-ms", type=int, default=1000,
                        help="snapshot freshness request bound (get_"
                             "snapshot max_age_ms)")
    args = parser.parse_args()

    geometry = load_geometry(args.geometry)
    config = yaml.safe_load(open(args.config).read())
    geometry["_resolution"] = float(config["resolution_m"])

    rclpy.init()
    node = rclpy.create_node("occ_surface_bridge")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(String, args.topic, qos)
    v2_publisher = None
    if args.publish_v2:
        if not args.scene_revision:
            node.get_logger().error(
                "--publish-v2 requires --scene-revision (the v2 contract "
                "binds the scene identity)")
            return 2
        from luggage_msgs.msg import SurfaceContractV2
        v2_publisher = node.create_publisher(
            SurfaceContractV2, args.v2_topic, qos)
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
                columns, identity = collect_columns(
                    args.observer_prefix, max_age_ms=args.max_age_ms)
                surface_map = build_surface_map(columns, geometry, identity)
                if surface_map.get("error"):
                    node.get_logger().error(
                        "surface map rejected: %s"
                        % surface_map["error"])
                    continue
                message = String()
                message.data = json.dumps(surface_map)
                publisher.publish(message)
                if v2_publisher is not None:
                    from luggage_description.frame_diagnostics import (
                        BoundaryContext)
                    from luggage_description.ros_frame_bindings.surfaces \
                        import surface_to_message
                    revision = int(surface_map.get("map_revision", 0) or 0)
                    record = build_surface_v2_record(
                        surface_map, geometry, identity,
                        args.scene_revision, "map-rev-%d" % revision)
                    context = BoundaryContext(
                        "F06", "occ_surface_bridge", "placement_planner",
                        record["request_id"], args.scene_revision,
                        "container_link", "", "scene",
                        geometry_hash=record["geometry_hash"],
                        map_revision=revision)
                    v2_publisher.publish(
                        surface_to_message(record, context))
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
