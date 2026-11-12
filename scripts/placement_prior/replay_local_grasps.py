#!/usr/bin/env python3
"""Check all measured grasps against one preserved prior target, without motion."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--capture", type=Path, required=True)
    p.add_argument("--request-id", required=True)
    p.add_argument("--scene-tf", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--negative-checks", action="store_true")
    args = p.parse_args()
    from process_guard import dedicated_domain_processes

    if os.environ.get("ROS_DOMAIN_ID") != "8" or dedicated_domain_processes("8"):
        raise RuntimeError("empty isolated domain 8 required")
    args.output.mkdir(parents=True, exist_ok=False)
    import yaml
    import rclpy
    from rclpy.serialization import serialize_message, deserialize_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    from ament_index_python.packages import get_package_share_directory
    from ament_index_python.packages import get_package_prefix
    from luggage_msgs.srv import (
        PreparePlacementPriorV2,
        CheckPlacementToolV2,
        CheckGraspLocalPaths,
    )
    from luggage_description.xacro_robot_with_scene_base import expand_and_patch
    from luggage_description.scene_tf_config_utils import (
        load_scene_tf_config,
        origin_in_world,
    )
    from luggage_description.scene_tf_publisher import rpy_to_quaternion
    from luggage_planning.placement_prior.requests import measured_detection_view
    from luggage_planning.placement_prior.pick import (
        candidates_for_prior,
        transform_from_pose,
    )
    from luggage_planning.placement_prior.contracts import (
        PriorRequest,
        Snapshot,
        Transform,
        PlacementCandidate,
    )
    from luggage_planning.placement_prior.local_paths import local_query
    from geometry_msgs.msg import Pose

    root = args.capture / "prior_boundaries"
    rows = [json.loads(l) for l in (root / "boundaries.jsonl").read_text().splitlines()]
    begin = next(
        i
        for i, r in enumerate(rows)
        if r["boundary"] == "prepare_request" and r["request_id"] == args.request_id
    )
    end = next(
        (
            i
            for i in range(begin + 1, len(rows))
            if rows[i]["boundary"] == "prepare_request"
        ),
        len(rows),
    )
    rows = rows[begin:end]
    inputs = {}

    def read(kind, typ):
        path = root / next(r["file"] for r in rows if r["boundary"] == kind)
        data = path.read_bytes()
        inputs[str(path)] = hashlib.sha256(data).hexdigest()
        return deserialize_message(data, typ)

    original = read("prepare_request", PreparePlacementPriorV2.Request)
    response = read("prepare_response", PreparePlacementPriorV2.Response)
    rigid = read("collision_request", CheckPlacementToolV2.Request)
    if not original.measured or not response.choices:
        raise ValueError("measured prior with target required")
    view = measured_detection_view(original.detection)
    translation, rpy = origin_in_world(load_scene_tf_config(str(args.scene_tf)))
    container = Transform(translation, rpy_to_quaternion(rpy))
    snapshot = Snapshot(
        response.surface.identity.scene_revision,
        response.map_revision,
        response.geometry_hash,
        response.model_sha256,
        container,
        serialize_message(response.surface),
        serialize_message(rigid.scene),
    )
    slot = response.choices[0]
    placement = PlacementCandidate(slot.slot_id, transform_from_pose(slot.slot.pose))
    req = PriorRequest(
        original.request_id,
        snapshot,
        view.box_size,
        (placement,),
        candidates_for_prior(view),
        True,
        world_from_pick_box=Transform(view.box_xyz, view.box_quat),
    )
    share = Path(get_package_share_directory("luggage_description"))
    xml, _ = expand_and_patch(
        str(share / "urdf/elfin_s20_with_camera.urdf.xacro"),
        config_path=str(args.scene_tf),
        xacro_args=[],
    )
    semantic = (
        (share / "config/S20_with_camera.srdf")
        .read_text()
        .replace('name="elfin_s20_with_camera"', 'name="S20"', 1)
    )
    if hashlib.sha256((xml + semantic).encode()).hexdigest() != snapshot.model_hash:
        raise ValueError("robot model differs")
    kin = yaml.safe_load(
        (
            Path(get_package_share_directory("elfin_moveit_config"))
            / "config/kinematics.yaml"
        ).read_text()
    )
    params = args.output / "parameters.yaml"
    params.write_text(
        yaml.safe_dump(
            {
                "/**": {
                    "ros__parameters": dict(
                        robot_description=xml,
                        robot_description_semantic=semantic,
                        robot_description_kinematics=kin,
                    )
                }
            }
        )
    )
    (args.output / "source.diff").write_text(
        subprocess.check_output(["git", "diff", "HEAD"], text=True)
    )
    report = dict(
        scored=False,
        revision=subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True
        ).strip(),
        dirty=subprocess.check_output(["git", "status", "--porcelain"], text=True),
        inputs=inputs,
        request_id=args.request_id,
        slot_id=slot.slot_id,
        scope="local joint paths only; transit and execution not evaluated",
        cases=[],
    )
    process = None
    node = None
    try:
        rclpy.init()
        node = rclpy.create_node("local_grasp_replay")
        log = (args.output / "native.log").open("w")
        binary = (
            Path(get_package_prefix("luggage_planning"))
            / "lib/luggage_planning/grasp_local_check_node"
        )
        process = subprocess.Popen(
            [str(binary), "--ros-args", "--params-file", str(params)],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        report["owned_pid"] = process.pid
        client = node.create_client(
            CheckGraspLocalPaths, "/placement_prior/check_local_paths"
        )
        if not client.wait_for_service(timeout_sec=20):
            raise TimeoutError("native service readiness")
        for i, g in enumerate(req.grasps):
            query = CheckGraspLocalPaths.Request()
            query.request_id = f"{req.request_id}:{slot.slot_id}:{g.candidate_id}"
            query.snapshot_id = hashlib.sha256(
                repr(snapshot.identity).encode()
            ).hexdigest()
            query.model_sha256 = snapshot.model_hash
            query.scene = rigid.scene
            query.box_wdh = list(view.box_size)
            query.budget_s = 2.0
            for name, t in local_query(req, placement, g).items():
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = t.translation
                (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ) = t.quaternion_xyzw
                setattr(query, name, pose)
            (args.output / f"{i:02d}.request.cdr").write_bytes(serialize_message(query))
            future = client.call_async(query)
            rclpy.spin_until_future_complete(node, future, timeout_sec=4.0)
            if not future.done():
                raise TimeoutError("local checker watchdog")
            result = future.result()
            (args.output / f"{i:02d}.response.cdr").write_bytes(
                serialize_message(result)
            )
            (args.output / f"{i:02d}.response.json").write_text(
                json.dumps(message_to_ordereddict(result), indent=2)
            )
            assert (result.request_id, result.snapshot_id, result.model_sha256) == (
                query.request_id,
                query.snapshot_id,
                query.model_sha256,
            )
            for branch in list(result.pick_branches) + list(result.place_branches):
                assert (
                    len(branch.incoming.points) >= 2
                    and len(branch.outgoing.points) >= 2
                )
                assert (
                    branch.incoming.points[-1].positions
                    == branch.outgoing.points[0].positions
                )
            row = dict(
                candidate_id=g.candidate_id,
                G=message_to_ordereddict(query.box_from_tcp),
                complete=result.complete,
                reason=result.reason,
                pick_branches=len(result.pick_branches),
                place_branches=len(result.place_branches),
                elapsed_s=result.elapsed_s,
                diagnostics=result.diagnostics_json,
            )
            report["cases"].append(row)
            print(json.dumps(row), flush=True)
        if args.negative_checks:
            from copy import deepcopy
            from moveit_msgs.msg import CollisionObject
            from shape_msgs.msg import SolidPrimitive

            report["negatives"] = []
            for label in ("model_mismatch", "blocked_scene"):
                bad = deepcopy(query)
                bad.request_id += ":" + label
                if label == "model_mismatch":
                    bad.model_sha256 = "invalid"
                else:
                    obstacle = CollisionObject()
                    obstacle.header.frame_id = "world"
                    obstacle.id = "injected-negative-obstacle"
                    obstacle.operation = CollisionObject.ADD
                    shape = SolidPrimitive(
                        type=SolidPrimitive.BOX, dimensions=[10.0, 10.0, 10.0]
                    )
                    center = Pose()
                    center.orientation.w = 1.0
                    obstacle.primitives = [shape]
                    obstacle.primitive_poses = [center]
                    bad.scene.world.collision_objects.append(obstacle)
                (args.output / (label + ".request.cdr")).write_bytes(
                    serialize_message(bad)
                )
                future = client.call_async(bad)
                rclpy.spin_until_future_complete(node, future, timeout_sec=4.0)
                if not future.done():
                    raise TimeoutError("negative checker watchdog")
                result = future.result()
                (args.output / (label + ".response.cdr")).write_bytes(
                    serialize_message(result)
                )
                assert not result.pick_branches and not result.place_branches
                assert result.reason != "LOCAL_PATHS_VALID"
                if label == "model_mismatch":
                    assert not result.complete and "invalid identity" in result.reason
                else:
                    assert (
                        json.loads(result.diagnostics_json)["collision_rejections"] > 0
                    )
                report["negatives"].append(
                    dict(label=label, reason=result.reason, refused=True)
                )
    finally:
        if process:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
            report["native_exit"] = process.returncode
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        report["domain_remaining"] = dedicated_domain_processes("8")
        report["capture_complete"] = len(report["cases"]) == len(req.grasps)
        (args.output / "result.json").write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
