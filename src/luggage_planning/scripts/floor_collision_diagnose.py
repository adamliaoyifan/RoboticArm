#!/usr/bin/env python3
"""Verify first-box placement on the container floor SLAB top (E12).

The container inner floor is the top of a slab at Z~0.53 (container_link),
NOT Z=0 (exterior base bottom). A box on the slab has contact (box top) at
slab_top + h. This script tests IK + collision at that contact for each box
size, with the box attached, to verify the first-box placement is reachable
and the loop can close.

Usage (atlas_stack.launch running):
    rosrun luggage_planning floor_collision_diagnose.py
"""
from __future__ import division

import math
import os
import sys
import hashlib
from datetime import datetime

import rospkg
import rospy
import yaml

PKG_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_planning"), "scripts")
DESC_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_description"), "scripts")
for _p in (PKG_SCRIPTS, DESC_SCRIPTS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from reachability_atlas_builder import ReachabilityAtlasBuilder, DEFAULT_JOINT_NAMES  # noqa: E402
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest  # noqa: E402

# Slab top (inner floor) height in container_link, from STL cross-section
# (E12): horizontal surface at Z~0.49-0.53, peak area at Z=0.530.
SLAB_TOP_Z = 0.53
BOX_SIZES = [
    ("carryon",  [0.55, 0.40, 0.25]),
    ("standard", [0.70, 0.45, 0.28]),
    ("large",    [0.80, 0.50, 0.32]),
]


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    rospy.init_node("floor_collision_diagnose", anonymous=True)
    rospy.loginfo("Instantiating builder (waits for /compute_ik)...")
    builder = ReachabilityAtlasBuilder()
    try:
        from std_srvs.srv import Trigger
        sync = rospy.ServiceProxy("/scene_manager/sync_static_scene", Trigger)
        sync.wait_for_service(timeout=5.0); sync(); rospy.sleep(2.0)
    except Exception as exc:
        rospy.logwarn("scene sync failed: %s", exc)

    validity = rospy.ServiceProxy("/check_state_validity", GetStateValidity)
    validity.wait_for_service(timeout=5.0)

    res = builder._resolution
    iy_mid = int(math.ceil(builder._inner_w / res)) // 2
    # Test cells across the container depth (ix), at the slab-top contact.
    ix_list = (1, 3, 5, 7)
    output_path = rospy.get_param(
        "~output_path",
        os.path.join(
            rospkg.RosPack().get_path("luggage_planning"),
            "data", "reachability_atlas", "floor_contact_validation.yaml",
        ),
    )
    scene_path = builder._resolve_scene_tf_config_path()
    robot_description = rospy.get_param("/robot_description", "")
    evidence = {
        "schema_version": 1,
        "generated_at": datetime.utcnow().isoformat() + "Z",
        "slab_top_z": SLAB_TOP_Z,
        "frame": "container_link",
        "resolution": res,
        "scene_tf_path": scene_path,
        "scene_tf_sha256": _sha256(scene_path),
        "robot_description_sha256": hashlib.sha256(
            robot_description.encode("utf-8")).hexdigest(),
        "ik_link": builder._ik_link,
        "ik_group": builder._ik_group,
        "avoid_collisions_for_ik": False,
        "state_validity_check": True,
        "results": [],
    }

    rospy.loginfo("=== Slab-top placement (slab Z=%.2f, box attached) ===", SLAB_TOP_Z)
    for name, size in BOX_SIZES:
        h = size[2]
        contact_z = SLAB_TOP_Z + h  # box bottom on slab top
        # Attach box.
        builder._payload_enabled = True
        builder._payload_size = [float(v) for v in size]
        builder._payload_shape = "box"
        builder._payload_attach_link = builder._ik_link
        builder._payload_offset = [0.0, 0.0, h * 0.5]
        builder._payload_touch_links = ["suction_panel"]
        builder._payload_id = "diag_payload_%s" % name
        builder._attach_payload()
        rospy.sleep(1.0)

        rospy.loginfo("[%s] h=%.2f contact_z=%.3f (box bottom on slab)", name, h, contact_z)
        for ix in ix_list:
            x = -builder._inner_l * 0.5 + (ix + 0.5) * res
            y = -builder._inner_w * 0.5 + (iy_mid + 0.5) * res
            contact_pos, contact_quat = builder._construct_base_pose(x, y, contact_z, 0.0)
            builder._avoid_collisions = False
            sol = builder._solve_ik(contact_pos, contact_quat, builder._fixed_seeds[0])
            if sol is None:
                rospy.loginfo("  ix=%d (x=%.2f): IK miss (kinematic)", ix, x)
                evidence["results"].append({
                    "box": name,
                    "size": list(size),
                    "ix": ix,
                    "container_x": round(x, 6),
                    "container_y": round(y, 6),
                    "contact_z": round(contact_z, 6),
                    "ik_solved": False,
                    "state_valid": False,
                    "contacts": [],
                })
                continue
            req = GetStateValidityRequest()
            req.robot_state.joint_state.name = list(DEFAULT_JOINT_NAMES)
            req.robot_state.joint_state.position = list(sol)
            req.group_name = builder._ik_group
            try:
                resp = validity(req)
            except rospy.ServiceException as exc:
                rospy.logwarn("  ix=%d validity failed: %s", ix, exc)
                evidence["results"].append({
                    "box": name,
                    "size": list(size),
                    "ix": ix,
                    "container_x": round(x, 6),
                    "container_y": round(y, 6),
                    "contact_z": round(contact_z, 6),
                    "ik_solved": True,
                    "state_valid": None,
                    "error": str(exc),
                    "contacts": [],
                })
                continue
            bodies = [
                "%s<->%s" % (c.contact_body_1, c.contact_body_2)
                for c in resp.contacts
            ]
            evidence["results"].append({
                "box": name,
                "size": list(size),
                "ix": ix,
                "container_x": round(x, 6),
                "container_y": round(y, 6),
                "contact_z": round(contact_z, 6),
                "ik_solved": True,
                "state_valid": bool(resp.valid),
                "contacts": bodies,
            })
            if resp.valid:
                rospy.loginfo("  ix=%d (x=%.2f): VALID (placeable!)", ix, x)
            else:
                rospy.loginfo(
                    "  ix=%d (x=%.2f): COLLISION -- %s",
                    ix, x, "; ".join(bodies[:4]))

        # Detach.
        try:
            builder._mc_scene.remove_attached_object(builder._payload_id)
            rospy.sleep(0.5)
        except Exception:
            pass
        builder._payload_enabled = False

    evidence["summary"] = {
        "total": len(evidence["results"]),
        "ik_solved": sum(1 for row in evidence["results"] if row["ik_solved"]),
        "valid": sum(1 for row in evidence["results"] if row["state_valid"] is True),
    }
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w") as stream:
        yaml.safe_dump(
            evidence, stream, default_flow_style=False, sort_keys=False)
    rospy.loginfo("Wrote floor-contact evidence to %s", output_path)


if __name__ == "__main__":
    main()
