"""Generated MotionSegmentV2 adapters. v1 ros_message_adapters is unchanged."""

from dataclasses import replace
from types import SimpleNamespace

from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectoryPoint

from luggage_msgs.msg import JointTrajectoryContractV2, MotionSegmentV2, PoseContractV2
from luggage_description.frame_diagnostics import BoundaryContractError
from luggage_description.ros_frame_bindings.diagnostics import diagnostic_to_message
from luggage_description.ros_frame_bindings.identity import (
    identity_from_message, require_message,
)
from luggage_description.ros_frame_bindings.joints import joints_from_message, joints_to_message
from luggage_description.ros_frame_bindings.poses import pose_from_message, pose_to_message
from luggage_planning.motion_frame_contracts import (
    MOTION_FRAME,
    AdmittedSegment,
    admit_motion_segment,
)


def _pose_inactive(message):
    identity = message.identity
    return (int(identity.contract_version) == 0 and not str(identity.frame_id)
            and not str(identity.request_id) and not str(identity.scene_revision))


def _joint_inactive(message):
    trajectory = message.trajectory
    return (int(message.contract_version) == 0 and not list(trajectory.joint_names)
            and not list(trajectory.points))


def _pose_dict(message, context):
    return pose_from_message(message, "motion", context)


def record_from_message(message):
    """Structural dict. Admission is separate so the expected scene is not copied."""
    require_message(message, MotionSegmentV2)
    record = {
        "contract_version": int(message.contract_version),
        "request_id": str(message.request_id),
        "scene_revision": str(message.scene_revision),
        "stamp_ns": int(message.stamp_ns) if message.stamp_valid else None,
        "time_basis": str(message.time_basis),
        "name": str(message.name),
        "segment_type": str(message.segment_type),
        "target_link": str(message.target_link),
        "policy_valid": bool(message.policy_valid),
        "keep_tool_down": bool(message.keep_tool_down),
        "keep_camera_down": bool(message.keep_camera_down),
        "lock_wrist": bool(message.lock_wrist),
        "allow_ompl_fallback": bool(message.allow_ompl_fallback),
        "required_cartesian_fraction": float(message.required_cartesian_fraction),
        "target_pose": None,
        "waypoints": [],
        "joint_trajectory": None,
    }
    if not _pose_inactive(message.target_pose):
        identity = identity_from_message(message.target_pose.identity)
        record["target_pose"] = dict(
            identity,
            kind="pose",
            reference=str(message.target_pose.reference),
            orientation_semantics=str(message.target_pose.orientation_semantics),
            position=[getattr(message.target_pose.pose.position, name) for name in ("x", "y", "z")],
            quaternion_xyzw=[
                getattr(message.target_pose.pose.orientation, name)
                for name in ("x", "y", "z", "w")],
        )
    for waypoint in message.waypoints:
        if _pose_inactive(waypoint):
            record["waypoints"].append({"inactive_default": True})
            continue
        identity = identity_from_message(waypoint.identity)
        record["waypoints"].append(dict(
            identity,
            kind="pose",
            reference=str(waypoint.reference),
            orientation_semantics=str(waypoint.orientation_semantics),
            position=[getattr(waypoint.pose.position, name) for name in ("x", "y", "z")],
            quaternion_xyzw=[
                getattr(waypoint.pose.orientation, name) for name in ("x", "y", "z", "w")],
        ))
    if not _joint_inactive(message.joint_trajectory):
        # Decode structurally; admission runs validate_joint_trajectory.
        joint = message.joint_trajectory
        record["joint_trajectory"] = {
            "contract_version": int(joint.contract_version),
            "kind": "joint_trajectory",
            "request_id": str(joint.request_id),
            "scene_revision": str(joint.scene_revision),
            "stamp_ns": int(joint.stamp_ns) if joint.stamp_valid else None,
            "time_basis": str(joint.time_basis),
            "angle_unit": str(joint.angle_unit),
            "joint_names": list(joint.trajectory.joint_names),
            "points": [],
        }
        for point in joint.trajectory.points:
            record["joint_trajectory"]["points"].append({
                "positions": [float(v) for v in point.positions],
                "velocities": [float(v) for v in point.velocities],
                "accelerations": [float(v) for v in point.accelerations],
                "effort": [float(v) for v in point.effort],
                "time_from_start_ns": (
                    int(point.time_from_start.sec) * 10**9
                    + int(point.time_from_start.nanosec)),
            })
    return record


def message_from_admitted(record, context):
    """Admit, then encode. Inactive payloads stay the default ROS messages."""
    admitted = admit_motion_segment(record, context)
    message = MotionSegmentV2()
    message.contract_version = 2
    message.request_id = admitted.request_id
    message.scene_revision = admitted.scene_revision
    message.stamp_valid = True
    message.stamp_ns = int(admitted.stamp_ns)
    message.time_basis = admitted.time_basis
    message.name = admitted.name
    message.segment_type = admitted.segment_type
    message.target_link = admitted.target_link
    message.policy_valid = True
    message.keep_tool_down = admitted.keep_tool_down
    message.keep_camera_down = admitted.keep_camera_down
    message.lock_wrist = admitted.lock_wrist
    message.allow_ompl_fallback = admitted.allow_ompl_fallback
    message.required_cartesian_fraction = float(admitted.required_cartesian_fraction)
    if admitted.target_pose is not None:
        message.target_pose = pose_to_message(admitted.target_pose, "motion", context.boundary)
        message.waypoints = [
            pose_to_message(pose, "motion", context.boundary) for pose in admitted.waypoints]
    if admitted.joint_points is not None:
        joint_record = dict(record["joint_trajectory"])
        joint_record["joint_names"] = list(admitted.joint_names)
        joint_record["points"] = [dict(point) for point in admitted.joint_points]
        joint_context = replace(
            context.boundary, frame_id="", reference="", time_basis="robot_state")
        message.joint_trajectory = joints_to_message(joint_record, joint_context)
    return message


def diagnostic_message(record, context, error=None):
    boundary = getattr(context, "boundary", context)
    return diagnostic_to_message(record if isinstance(record, dict) else {}, boundary, error)


def pose_namespace(record):
    position = record["position"]
    quat = record["quaternion_xyzw"]
    return SimpleNamespace(
        position=SimpleNamespace(x=float(position[0]), y=float(position[1]), z=float(position[2])),
        orientation=SimpleNamespace(
            x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3])),
    )


def geometry_pose(record):
    pose = Pose()
    pose.position.x = float(record["position"][0])
    pose.position.y = float(record["position"][1])
    pose.position.z = float(record["position"][2])
    pose.orientation.x = float(record["quaternion_xyzw"][0])
    pose.orientation.y = float(record["quaternion_xyzw"][1])
    pose.orientation.z = float(record["quaternion_xyzw"][2])
    pose.orientation.w = float(record["quaternion_xyzw"][3])
    return pose


def segment_namespace(admitted):
    """v1-shaped object. Joint points keep their times; this is not read_joint_knots input."""
    segment = SimpleNamespace(
        type=admitted.segment_type,
        name=admitted.name,
        target_pose=pose_namespace(admitted.target_pose) if admitted.target_pose else None,
        waypoints=[pose_namespace(pose) for pose in admitted.waypoints],
        joint_waypoints=[],
        keep_tool_down=admitted.keep_tool_down,
        keep_camera_down=admitted.keep_camera_down,
        lock_wrist=admitted.lock_wrist,
        allow_ompl_fallback=admitted.allow_ompl_fallback,
        required_cartesian_fraction=admitted.required_cartesian_fraction,
        admitted=admitted,
    )
    return segment


def timed_trajectory(admitted):
    """RobotTrajectory-free joint message pieces: names, positions, and original times."""
    points = []
    for point in admitted.joint_points:
        out = JointTrajectoryPoint()
        out.positions = list(point["positions"])
        out.velocities = list(point["velocities"])
        out.accelerations = list(point["accelerations"])
        out.effort = list(point["effort"])
        out.time_from_start.sec, out.time_from_start.nanosec = divmod(
            int(point["time_from_start_ns"]), 10**9)
        points.append(out)
    return list(admitted.joint_names), points


def apply_diagnostic(response, record, context, error=None):
    response.diagnostic = diagnostic_message(record, context, error)
    if error is not None:
        response.success = False
        response.message = error.code if not isinstance(error, str) else error
        if isinstance(error, BoundaryContractError):
            response.message = error.code
    return response
