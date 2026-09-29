#!/usr/bin/env python3
"""ST03 pick/retry frame-boundary contracts (ROS-free; F09/F10).

The pick entry stores the explicit suction target link, the observation
identity, the actual box-to-tool grasp transform and the scene revision
alongside the recorded approach pose; retry rebuilds only from that
snapshot. Nothing here looks up TF, defaults a frame or repairs input.
"""

import copy
from dataclasses import dataclass
from typing import Optional

from luggage_description.frame_transforms import (
    quaternion_product, rotate_vector,
)
from luggage_planning.pose import MotionSegment
from luggage_planning.suction_candidate_waypoints import (
    REVERSE_MIN_M, candidate_normal,
)

#: The executor's implicit default link, made explicit at the boundary.
SUCTION_TARGET_LINK = "suction_contact_frame"

#: Recovery rejects a changed calibration/scene identity with this code.
STALE_CALIBRATION_CODE = "CALIBRATION_REVISION_MISMATCH"


def _conjugate(quaternion_xyzw):
    return (-quaternion_xyzw[0], -quaternion_xyzw[1], -quaternion_xyzw[2],
            quaternion_xyzw[3])


def grasp_transform(box_center_world, box_quaternion_xyzw, contact_world,
                    contact_quaternion_xyzw, scene_revision):
    """Actual ``T_box_tool`` for the selected grasp.

    ``translation`` is the contact offset expressed in box axes — it
    includes the real lateral offset, not only half the box height.
    ``quaternion_xyzw`` is the tool orientation in box axes. Composing with
    ``frame_transforms.box_to_tool_target`` reproduces the world tool
    target exactly, which is the caller's independent geometric oracle.
    """
    box_quaternion = tuple(float(q) for q in box_quaternion_xyzw)
    contact_quaternion = tuple(float(q) for q in contact_quaternion_xyzw)
    delta = [float(contact_world[i]) - float(box_center_world[i])
             for i in range(3)]
    return {
        "source_frame": "suction_contact_frame",
        "target_frame": "box",
        "translation": list(rotate_vector(_conjugate(box_quaternion), delta)),
        "quaternion_xyzw": list(
            quaternion_product(_conjugate(box_quaternion), contact_quaternion)),
        "scene_revision": str(scene_revision),
    }


@dataclass(frozen=True)
class ApproachSnapshot:
    """F10 retry snapshot: everything retry needs, frozen at record time.

    The stored pose and grasp are private deep copies: a frozen dataclass
    alone does not stop nested mutation of the caller's pose, and the
    recorded target must stay bit-for-bit regardless of what the caller or
    a later retry does to their objects.
    """

    target_pose: object                 # exact approach segment target (world)
    link_name: str = SUCTION_TARGET_LINK
    candidate_id: str = ""
    approach_normal: tuple = ()
    grasp: Optional[dict] = None        # T_box_tool from the selected grasp
    scene_revision: str = ""
    stamp_ns: int = 0

    def as_dict(self):
        return {
            "link_name": self.link_name,
            "candidate_id": self.candidate_id,
            "approach_normal": list(self.approach_normal),
            "grasp": copy.deepcopy(self.grasp) if self.grasp else None,
            "scene_revision": self.scene_revision,
            "stamp_ns": self.stamp_ns,
        }


def approach_snapshot(target_pose, candidate, detection, scene_revision="",
                      link_name=SUCTION_TARGET_LINK):
    """Record the approach snapshot when an approach segment succeeds.

    ``detection`` is the session ``DetectionView``; the grasp transform is
    stored only when the detection actually carried box geometry (measured
    provenance retained, never invented). The pose and grasp are captured
    as independent copies — mutating the caller's pose afterwards cannot
    rewrite history.
    """
    grasp = None
    box_center = getattr(detection, "box_xyz", None)
    box_quat = getattr(detection, "box_quat", None)
    contact = getattr(candidate, "contact", None)
    if (box_center and box_quat and contact is not None
            and any(abs(q) > 1e-12 for q in box_quat)):
        grasp = grasp_transform(
            box_center, box_quat,
            (contact.position.x, contact.position.y, contact.position.z),
            (contact.orientation.x, contact.orientation.y,
             contact.orientation.z, contact.orientation.w),
            scene_revision)
    # Exact stamp preference: a declared integer-nanosecond field wins
    # over the float-seconds transport form (epoch floats cannot carry
    # nanoseconds). Zero stays undeclared so a synthetic float stamp is
    # never folded to 0.
    stamp = 0
    detection_stamp_ns = getattr(detection, "stamp_ns", None)
    if (isinstance(detection_stamp_ns, int)
            and not isinstance(detection_stamp_ns, bool)
            and detection_stamp_ns > 0):
        stamp = detection_stamp_ns
    else:
        detection_stamp = getattr(detection, "stamp", None)
        if detection_stamp is not None:
            stamp = int(round(float(detection_stamp) * 1e9))
    return ApproachSnapshot(
        target_pose=copy.deepcopy(target_pose),
        link_name=str(link_name),
        candidate_id=str(getattr(candidate, "candidate_id", "")),
        approach_normal=tuple(candidate_normal(contact.orientation)),
        grasp=copy.deepcopy(grasp) if grasp is not None else None,
        scene_revision=str(scene_revision),
        stamp_ns=stamp,
    )


def retry_segment_from_snapshot(snapshot, attach_pose=None,
                                reverse_min_m=REVERSE_MIN_M):
    """Retry segment rebuilt strictly from the recorded snapshot.

    The target is the snapshot pose bit-for-bit; no TF query, no latest
    candidate state. The returned segment carries its own copy — mutating
    the retry target cannot rewrite the frozen snapshot either. The
    reverse-distance guard uses the snapshot's frozen normal, so a mutated
    live candidate cannot stretch the retreat.
    """
    if attach_pose is not None and snapshot.approach_normal:
        delta = (snapshot.target_pose.position.x - attach_pose.position.x,
                 snapshot.target_pose.position.y - attach_pose.position.y,
                 snapshot.target_pose.position.z - attach_pose.position.z)
        distance = (delta[0] * snapshot.approach_normal[0]
                    + delta[1] * snapshot.approach_normal[1]
                    + delta[2] * snapshot.approach_normal[2])
        if distance < float(reverse_min_m):
            raise ValueError(
                "retry reverse distance %.3f m below %.3f m"
                % (distance, float(reverse_min_m)))
    return MotionSegment(
        name="retry_reverse", type="cartesian",
        target_pose=copy.deepcopy(snapshot.target_pose),
        allow_ompl_fallback=False, required_cartesian_fraction=1.0,
    )


def stale_calibration_reason(snapshot, current_scene_revision):
    """``STALE_CALIBRATION_CODE`` when recovery's scene identity changed.

    Comparison happens only when both sides declare a nonempty revision:
    an undeclared scene identity is a wiring gap, not a silent pass, and
    callers report it separately.
    """
    recorded = getattr(snapshot, "scene_revision", "")
    current = str(current_scene_revision or "")
    if recorded and current and recorded != current:
        return STALE_CALIBRATION_CODE
    return None
