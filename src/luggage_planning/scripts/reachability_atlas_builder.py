#!/usr/bin/env python3
"""Offline reachability atlas builder.

Pre-computes a (x,y,z,yaw) reachability grid. Pose IK is
offline_ik.refine_to_pose and requires tool-down
(``tool_down_tolerance`` 0; suction normal along base -Z). Collision is
/check_state_validity on the full robot model, so suction_panel,
eef_mount_adapter, the D555 box, and the Mid-360 housing all reject a
cell when they intersect the container.
Saves results to .npz + .yaml for runtime loading.

Usage:
    rosrun luggage_planning reachability_atlas_builder.py \
        _scene_tf_config:=$(rospack find luggage_description)/config/scene_tf.yaml \
        _output_dir:=$(rospack find luggage_planning)/data/reachability_atlas \
        _resolution_xyz:=0.15 _avoid_collisions:=true _max_workers:=8

Payload-aware build (§5.6) -- attach a nominal box to the EE so the reachable
set reflects the carried load, for per-box-size reachability sampling:
    rosrun luggage_planning reachability_atlas_builder.py \
        _output_dir:=.../data/reachability_atlas _avoid_collisions:=true \
        _payload_enabled:=true _payload_size:=[0.55,0.40,0.25] \
        _payload_attach_link:=suction_contact_frame \
        _payload_offset:=[0.0,0.0,0.125]
    # -> s20_container_collision_aware_payload_0.55x0.40x0.25.{npz,yaml}

Vary _payload_size per catalog box type to measure how reachability changes
with the box being placed. _payload_offset default = [0,0,box_h/2] (box hangs
below the suction contact). _payload_touch_links default = ["suction_panel"].
"""

from __future__ import division

import math
import os
import sys
import time
import hashlib

import numpy as np
import yaml

_PKG_ROOT = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), ".."))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from luggage_planning.atlas.wavefront import (  # noqa: E402
    BRANCH_INTERNAL_LIFT,
    BRANCH_NONE,
    BRANCH_VERTICAL,
    MARGINAL as _MARGINAL,
    INTERFERENCE_TOOL_Z,
    MIN_LIFT,
    NOMINAL_LIFT_CLEARANCE,
    contact_below_interference,
    REACHABLE as _REACHABLE,
    SAFE_TOOL_Z,
    UNKNOWN as _UNKNOWN,
    UNREACHABLE as _UNREACHABLE,
    adaptive_lift_clearance,
    classify_cell,
    deterministic_wavefront,
    grid_neighbors,
    joint_interpolation_samples,
    select_distinct_branches,
)
from luggage_planning.atlas.frame_contracts import (  # noqa: E402
    DIRECTED_YAW_BINS,
    atlas_contract_record,
    container_yaw_offset,
    geometry_hash as _contract_geometry_hash,
    payload_hash as _contract_payload_hash,
    tool_target_in_base,
)

try:  # Keep the pure wavefront helpers importable without a ROS installation.
    import rospy
    import rospkg
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetStateValidity
    from sensor_msgs.msg import JointState
    try:
        from moveit_msgs.srv import GetStateValidityRequest
    except ImportError:  # ROS 2: the request type lives on the service class.
        GetStateValidityRequest = GetStateValidity.Request
except ImportError:  # pragma: no cover - exercised by the no-ROS unit tests.
    rospy = None
    rospkg = None


UNKNOWN = np.uint8(_UNKNOWN)
UNREACHABLE = np.uint8(_UNREACHABLE)
MARGINAL = np.uint8(_MARGINAL)
REACHABLE = np.uint8(_REACHABLE)


DEFAULT_JOINT_NAMES = [
    "elfin_joint1", "elfin_joint2", "elfin_joint3",
    "elfin_joint4", "elfin_joint5", "elfin_joint6",
]


def _rpy_to_matrix(rpy):
    cr, sr = math.cos(rpy[0]), math.sin(rpy[0])
    cp, sp = math.cos(rpy[1]), math.sin(rpy[1])
    cy, sy = math.cos(rpy[2]), math.sin(rpy[2])
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _rpy_to_quat(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _rotate_point(R, p):
    return (
        R[0][0] * p[0] + R[0][1] * p[1] + R[0][2] * p[2],
        R[1][0] * p[0] + R[1][1] * p[1] + R[1][2] * p[2],
        R[2][0] * p[0] + R[2][1] * p[1] + R[2][2] * p[2],
    )


def _file_hash(path):
    """MD5 hash of a file's contents (for version verification)."""
    h = hashlib.md5()
    try:
        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(4096), b""):
                h.update(chunk)
    except IOError:
        return ""
    return h.hexdigest()


def _param_bool(value, default=False):
    """Parse a ROS param that may arrive as bool, int, or 'true'/'false' string.

    ROS ``<param value="false"/>`` and command-line ``_x:=false`` both store the
    string "false"; ``bool("false")`` is True in Python, so naive ``bool(...)``
    is wrong. ``<rosparam>`` stores a real bool.
    """
    if isinstance(value, bool):
        return value
    if value is None:
        return default
    if isinstance(value, (int, float)):
        return bool(value)
    s = str(value).strip().lower()
    if s in ("true", "1", "yes", "on"):
        return True
    if s in ("false", "0", "no", "off", ""):
        return False
    return default


def _param_list(value, default, cast=float):
    """Parse a ROS param that may arrive as a list or a '[a,b,c]' string.

    ``<param value="[a,b,c]"/>``, command-line ``_x:=[a,b,c]``, and ``<rosparam>``
    all store different types (string / string / list). Handle all three so the
    builder works regardless of how the param was passed.
    """
    if isinstance(value, (list, tuple)):
        try:
            return [cast(v) for v in value]
        except (TypeError, ValueError):
            return list(default)
    if isinstance(value, str):
        s = value.strip()
        if not s:
            return list(default)
        parsed = None
        try:
            import ast
            parsed = ast.literal_eval(s)
        except (ValueError, SyntaxError):
            try:
                parsed = yaml.safe_load(s)
            except yaml.YAMLError:
                parsed = None
        if not isinstance(parsed, (list, tuple)):
            return list(default)
        try:
            return [cast(v) for v in parsed]
        except (TypeError, ValueError):
            return list(default)
    return list(default)


class ReachabilityAtlasBuilder:
    def __init__(self):
        if rospy is None:
            raise RuntimeError("ReachabilityAtlasBuilder requires ROS Python packages")
        try:
            from luggage_description.scene_tf_config_utils import (  # pylint: disable=import-outside-toplevel
                container_in_base_link,
                container_inner_ceiling_z,
                container_inner_dimensions,
                container_inner_floor_z,
                container_opening_side,
                load_scene_tf_config,
                resolve_scene_tf_config_path,
            )
        except ImportError:
            desc_scripts = os.path.join(
                rospkg.RosPack().get_path("luggage_description"), "scripts")
            if desc_scripts not in sys.path:
                sys.path.insert(0, desc_scripts)
            from scene_tf_config_utils import (  # pylint: disable=import-outside-toplevel
                container_in_base_link,
                container_inner_ceiling_z,
                container_inner_dimensions,
                container_inner_floor_z,
                container_opening_side,
                load_scene_tf_config,
                resolve_scene_tf_config_path,
            )
        self._resolve_scene_tf_config_path = resolve_scene_tf_config_path
        self._scene_tf_path = rospy.get_param(
            "~scene_tf_config", resolve_scene_tf_config_path())
        self._scene_config = load_scene_tf_config(self._scene_tf_path)
        self._resolution = float(rospy.get_param("~resolution_xyz", 0.15))
        # ST06/F14: DIRECTED container-frame concrete-tool bins with 2pi
        # period (0 and pi are distinct branches). The legacy [0, pi/2]
        # pi-folded sampling is v1; it is not relabelled, it is rebuilt.
        self._yaw_bins = [float(y) for y in rospy.get_param(
            "~yaw_bins", list(DIRECTED_YAW_BINS))]
        # Nominal same-column lift. High contacts shrink it via
        # adaptive_lift_clearance so the carry stays at or below safe_tool_z.
        self._transit_clearance = float(rospy.get_param(
            "~transit_clearance", NOMINAL_LIFT_CLEARANCE))
        self._safe_tool_z = float(rospy.get_param("~safe_tool_z", SAFE_TOOL_Z))
        self._min_lift = float(rospy.get_param("~min_lift", MIN_LIFT))
        self._offline_urdf = None
        self._tcp_path = None
        floor_z = container_inner_floor_z(self._scene_config)
        ceiling_z = container_inner_ceiling_z(self._scene_config)
        # Z coordinate of the sampling region's bottom in container_link. The
        # airport_container_real mesh has a floor slab whose TOP is at Z~0.50
        # (container_link Z=0 is the exterior base bottom, NOT the inner floor
        # -- see docs/status/experiment_log.md E12). Sampling must start above
        # the slab top; otherwise iz=0..2 land inside the solid slab/base and
        # report 0% reachable (EE target inside solid material, not a real
        # placement). floor_z-0.03 puts the first 0.15-m cell center safely
        # above the measured slab while preserving backward compatibility.
        self._z_min_offset = float(rospy.get_param(
            "~z_min_offset", max(0.0, floor_z - 0.03)))
        self._z_max_offset = float(rospy.get_param("~z_max_offset", 0.05))
        self._floor_z = float(floor_z)
        self._ceiling_z = float(ceiling_z)
        # Transit-side tool-down tolerance (rad). Mirrors motion_planner's
        # OrientationConstraint path constraint: transit IK may tilt the EE
        # within +/-tolerance (RP) to clear the door sill / openings, while
        # contact IK stays strict tool-down to guarantee ideal flat placement
        # (E1-B: zero tolerance amplifies the door-sill floor block). 0 = strict
        # everywhere (legacy). See docs/status/experiment_log.md E1/E5.
        self._tool_down_tolerance = float(
            rospy.get_param("~tool_down_tolerance", 0.0))
        self._avoid_collisions = _param_bool(rospy.get_param("~avoid_collisions", True))
        self._ik_timeout = float(rospy.get_param("~ik_timeout", 0.05))
        self._max_workers = int(rospy.get_param("~max_workers", 1))
        self._max_branches = max(1, int(rospy.get_param("~max_branches", 3)))
        self._branch_distance = float(rospy.get_param("~branch_distance", 0.25))
        self._joint_interp_step = float(rospy.get_param(
            "~joint_interpolation_step", 0.10))
        self._marginal_margin = float(rospy.get_param(
            "~marginal_joint_margin", 0.10))
        self._neighbor_threshold = float(rospy.get_param(
            "~neighbor_confidence_threshold", 0.50))
        # The default is computed before get_param so a Humble host without
        # rospkg still starts when ~output_dir is set. RosPack() inside the
        # call would run even when the parameter is present.
        default_output = os.path.normpath(os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "..", "data", "reachability_atlas"))
        self._output_dir = rospy.get_param("~output_dir", default_output)
        # Atlas filename prefix. Empty -> derive from /robot_name
        # (elfin_s30_with_camera -> "s30") so S20/S30 atlases coexist as
        # s20_container_*.npz / s30_container_*.npz instead of overwriting.
        self._atlas_prefix = str(rospy.get_param("~atlas_prefix", ""))
        self._ik_group = rospy.get_param("~ik_group", "elfin_arm")
        self._ik_link = rospy.get_param("~ik_link", "suction_contact_frame")
        self._base_frame = rospy.get_param("~base_frame", "elfin_base_link")

        # Payload-aware atlas (§5.6): attach a nominal box to the EE so the
        # reachable set reflects the carried load (empty-load atlas is optimistic).
        # Used for per-box-size reachability sampling research: vary payload_size
        # to measure how reachability changes with the box being placed.
        self._payload_enabled = _param_bool(rospy.get_param("~payload_enabled", False))
        self._payload_size = _param_list(rospy.get_param(
            "~payload_size", [0.80, 0.50, 0.32]), [0.80, 0.50, 0.32])  # [l, w, h]
        self._payload_shape = str(rospy.get_param("~payload_shape", "box"))
        self._payload_attach_link = rospy.get_param(
            "~payload_attach_link", self._ik_link)
        # Box center offset in the attach_link frame. Default hangs the box below
        # the suction contact by box_h/2 (box top at the contact, extending down).
        self._payload_offset = _param_list(
            rospy.get_param("~payload_offset", [0.0, 0.0, self._payload_size[2] * 0.5]),
            [0.0, 0.0, self._payload_size[2] * 0.5])
        self._payload_touch_links = _param_list(
            rospy.get_param("~payload_touch_links", ["suction_panel"]),
            ["suction_panel"], cast=str)
        self._payload_id = str(rospy.get_param("~payload_id", "atlas_payload"))

        # Container geometry.
        inner_l, inner_w, inner_h = container_inner_dimensions(self._scene_config)
        self._inner_l = float(inner_l)
        self._inner_w = float(inner_w)
        self._inner_h = float(inner_h)
        base_xyz, base_rpy = container_in_base_link(self._scene_config)
        self._container_xyz = [float(v) for v in base_xyz]
        self._container_rpy = [float(v) for v in base_rpy]
        self._container_R = _rpy_to_matrix(self._container_rpy)
        # ST06: the yaw bins are container-frame directed tool yaws; IK
        # still solves in base_link, so the mapping needs the constant Z
        # offset of the (horizontal) container in base_link.
        self._container_yaw_offset = container_yaw_offset(self._container_rpy)
        self._opening_axis, self._opening_sign = container_opening_side(
            self._scene_config)

        # Fixed seeds are used only for opening anchors and branch repair.
        self._fixed_seeds = self._load_fixed_seeds()

        # Pose solving is offline and deterministic. Collision stays on
        # check_state_validity; without it every solved cell is UNKNOWN.
        validity_service = rospy.get_param(
            "~state_validity_service", "/check_state_validity")
        rospy.loginfo("Waiting for state validity service %s ...", validity_service)
        rospy.wait_for_service(validity_service, timeout=60.0)
        self._state_validity = rospy.ServiceProxy(
            validity_service, GetStateValidity)
        rospy.loginfo("State validity service ready.")

        # Tool-down quaternion (suction +Z -> base -Z).
        self._tool_down = (1.0, 0.0, 0.0, 0.0)

    def _load_observe_seed(self):
        try:
            import rospkg as _rp
            poses_path = os.path.join(
                _rp.RosPack().get_path("luggage_description"),
                "config", "robot_poses.yaml.example")
            with open(poses_path) as f:
                cfg = yaml.safe_load(f)
            return [float(v) for v in cfg.get("poses", {}).get("observe", {}).get("values", [])]
        except Exception as exc:
            rospy.logwarn("Cannot load observe seed: %s", exc)
            return []

    def _load_fixed_seeds(self):
        configured = rospy.get_param("~fixed_seeds", [])
        seeds = []
        observe = self._load_observe_seed()
        if len(observe) == len(DEFAULT_JOINT_NAMES):
            seeds.append(tuple(observe))
        defaults = configured or [
            [0.0, -1.57, 1.57, 0.0, 1.57, 0.0],
            [0.0, -1.20, -1.20, 0.0, 1.57, 0.0],
            [0.0, -1.57, 1.57, math.pi, 1.57, 0.0],
            # Local refine cannot jump from j1≈0 into these families.
            # Side-reach covers the +Y interior; the last seed is the
            # opening-column pose at iz4 that the j1≈0 seeds miss.
            [1.80, -0.70, 1.66, -2.54, -1.57, 0.22],
            [0.59, 0.14, -1.37, -0.34, -1.57, -0.98],
        ]
        for raw in defaults:
            if len(raw) != len(DEFAULT_JOINT_NAMES):
                rospy.logwarn("Ignoring fixed seed with %d joints", len(raw))
                continue
            seed = tuple(float(value) for value in raw)
            if seed not in seeds:
                seeds.append(seed)
        if not seeds:
            raise ValueError("At least one six-joint fixed seed is required")
        return tuple(seeds)

    def _construct_base_pose(self, x, y, z, yaw):
        """Construct suction_contact_frame target pose in base_link.

        ``yaw`` is the DIRECTED container-frame tool yaw (ST06/F14); it is
        mapped through the scene-config container rotation before composing
        the base-frame tool-down rotation, exactly as
        ``atlas_frame_contracts.tool_target_in_base`` defines the advertised
        target. Container roll/pitch must not tilt the suction normal away
        from base -Z (enforced by ``container_yaw_offset`` at init).
        """
        return tool_target_in_base(
            x, y, z, yaw, self._container_xyz, self._container_rpy,
            self._tool_down)

    def _ensure_offline_ik(self):
        """Load the deterministic FK/IK model once per build."""
        if self._offline_urdf is not None:
            return
        from luggage_planning.offline_ik import load_urdf
        self._offline_urdf = load_urdf(scene_tf_config=self._scene_tf_path)
        # load_urdf keeps the world_base joint, so a full path is in world.
        # Cell targets are elfin_base_link. Start at the first arm joint so
        # FK matches _construct_base_pose.
        full_path = self._offline_urdf.path_to(self._ik_link)
        arm = DEFAULT_JOINT_NAMES[0]
        if arm not in full_path:
            raise RuntimeError(
                "URDF path to %s does not contain %s: %s" % (
                    self._ik_link, arm, full_path))
        self._tcp_path = full_path[full_path.index(arm):]
        if len(self._tcp_path) < len(DEFAULT_JOINT_NAMES):
            raise RuntimeError(
                "URDF path to %s is incomplete: %s" % (
                    self._ik_link, self._tcp_path))

    def _solve_ik(self, pos, quat, seed):
        """Refine ``seed`` onto the pose, then require a validity check.

        ``None`` is a deterministic kinematic or collision miss. A validity
        service failure is raised so the caller marks the cell UNKNOWN
        instead of recording a colliding pose as unreachable or reachable.
        ``refine_to_pose`` does not see the PlanningScene.
        """
        if seed is None or len(seed) != len(DEFAULT_JOINT_NAMES):
            raise ValueError("IK seed must contain exactly six joints")
        self._ensure_offline_ik()
        from luggage_planning.offline_ik import refine_to_pose
        joints, _info = refine_to_pose(
            self._offline_urdf, self._tcp_path, pos, quat, seed)
        if joints is None:
            return None
        if self._state_validity is None:
            raise RuntimeError(
                "check_state_validity unavailable; refusing an unchecked IK solution")
        valid = self._state_is_valid(joints)
        if valid is None:
            raise RuntimeError(
                "check_state_validity failed; cell stays UNKNOWN")
        if not valid:
            return None
        return tuple(float(v) for v in joints)

    def _solve_ik_transit_with_tolerance(self, pos, base_quat, seed):
        """Transit IK with tool-down tolerance (E1-B/E5).

        Mirrors motion_planner's OrientationConstraint path constraint: if
        strict tool-down transit IK fails and ``~tool_down_tolerance > 0``,
        retry with the target orientation tilted by +/-tolerance around base
        X and Y (a cross sample of the tolerance cone). This lets the EE clear
        the door sill / openings during transit. Contact IK stays strict
        tool-down to guarantee ideal flat placement. Returns joint solution or
        None.
        """
        sol = self._solve_ik(pos, base_quat, seed)
        if sol is not None or self._tool_down_tolerance <= 0.0:
            return sol
        tol = self._tool_down_tolerance
        half = tol * 0.5
        s, c = math.sin(half), math.cos(half)
        tilt_axes = [
            (s, 0.0, 0.0, c),    # +X tilt
            (-s, 0.0, 0.0, c),   # -X tilt
            (0.0, s, 0.0, c),    # +Y tilt
            (0.0, -s, 0.0, c),   # -Y tilt
        ]
        for tilt in tilt_axes:
            tilted = _quat_multiply(tilt, base_quat)
            sol = self._solve_ik(pos, tilted, seed)
            if sol is not None:
                return sol
        return None

    @staticmethod
    def _joint_margin(joints):
        """Conservative distance to the Elfin S20 joint limits."""
        return min(
            min(abs(j - (-6.28)), abs(6.28 - j)) if i in (0, 3, 4, 5)
            else min(abs(j - (-3.32)), abs(0.17 - j)) if i == 1
            else min(abs(j - (-2.93)), abs(2.93 - j))
            for i, j in enumerate(joints)
        )

    def _state_is_valid(self, joints):
        """Return True/False, or None when validity cannot be established."""
        if self._state_validity is None:
            return None
        req = GetStateValidityRequest()
        req.group_name = self._ik_group
        req.robot_state = RobotState(joint_state=JointState(
            name=list(DEFAULT_JOINT_NAMES),
            position=list(joints),
        ))
        # A full state would drop the attached payload. A diff keeps it.
        req.robot_state.is_diff = True
        try:
            return bool(self._state_validity(req).valid)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(
                5.0, "check_state_validity failed (fail-closed): %s", exc)
            return None

    def _segment_is_valid(self, start, goal):
        start_valid = self._state_is_valid(start)
        if start_valid is not True:
            return start_valid
        for sample in joint_interpolation_samples(
                start, goal, self._joint_interp_step):
            valid = self._state_is_valid(sample)
            if valid is not True:
                return valid
        return True

    def _cell_target(self, ix, iy, iz, iyaw):
        x = -self._inner_l * 0.5 + (ix + 0.5) * self._resolution
        y = -self._inner_w * 0.5 + (iy + 0.5) * self._resolution
        z = self._z_min_offset + (iz + 0.5) * self._resolution
        yaw = self._yaw_bins[iyaw]
        return x, y, z, yaw

    def _make_branch(self, transit, contact, repair, kind, lift,
                     transit_target):
        target = tuple(float(v) for v in transit_target)
        return {
            "transit": tuple(float(v) for v in transit),
            "contact": tuple(float(v) for v in contact),
            "margin": min(
                self._joint_margin(transit), self._joint_margin(contact)),
            "repair": bool(repair),
            "kind": int(kind),
            "lift_clearance": float(lift),
            # Container-frame pose this transit seed was solved against.
            # Vertical branches store this cell's (x, y, z + lift). Internal
            # branches copy the predecessor's already-recorded target. Never
            # recompute it from the joints being stored.
            "transit_target": target,
        }

    def _try_vertical(self, ix, iy, iz, iyaw, transit_seed, clearance,
                      predecessor=None, repair=False):
        """Same-column carry at ``z + clearance``, then down to contact."""
        x, y, z, yaw = self._cell_target(ix, iy, iz, iyaw)
        if not contact_below_interference(z, INTERFERENCE_TOOL_Z):
            return None, False, False, False
        contact_pos, contact_quat = self._construct_base_pose(x, y, z, yaw)
        transit_pos, transit_quat = self._construct_base_pose(
            x, y, z + clearance, yaw)
        transit = self._solve_ik_transit_with_tolerance(
            transit_pos, transit_quat, transit_seed)
        if transit is None:
            return None, False, False, False
        if predecessor is not None:
            connected = self._segment_is_valid(predecessor, transit)
            if connected is not True:
                return None, True, False, connected is None
        contact = self._solve_ik(contact_pos, contact_quat, transit)
        if contact is None:
            return None, True, False, False
        insertion = self._segment_is_valid(transit, contact)
        if insertion is not True:
            return None, True, True, insertion is None
        return (
            self._make_branch(
                transit, contact, repair, BRANCH_VERTICAL, clearance,
                (x, y, z + float(clearance))),
            True, True, False)

    def _try_internal(self, ix, iy, iz, iyaw, predecessor, repair=False,
                      predecessor_target=None):
        """Rise from the predecessor's safe carry straight to this contact.

        The stored transit seed stays the predecessor carry so the next cell
        is not warm-started from a pose against the lid. Its advertised
        target is the predecessor branch's recorded geometric target, copied
        as-is.
        """
        x, y, z, yaw = self._cell_target(ix, iy, iz, iyaw)
        if not contact_below_interference(z, INTERFERENCE_TOOL_Z):
            return None, False, False, False
        if predecessor_target is None:
            return None, False, False, False
        target = tuple(float(v) for v in predecessor_target)
        if len(target) != 3 or not all(math.isfinite(v) for v in target):
            return None, False, False, False
        contact_pos, contact_quat = self._construct_base_pose(x, y, z, yaw)
        contact = self._solve_ik(contact_pos, contact_quat, predecessor)
        if contact is None:
            return None, False, False, False
        segment = self._segment_is_valid(predecessor, contact)
        if segment is not True:
            return None, False, True, segment is None
        return (
            self._make_branch(
                predecessor, contact, repair, BRANCH_INTERNAL_LIFT, 0.0,
                target),
            False, True, False)

    def compute(self):
        """Compute atlas data without saving or scene sync.

        Returns (data_dict, meta_dict). The caller (e.g. layout_atlas_evaluator)
        is responsible for scene setup before calling this.
        """
        self._compute_only = True
        self._compute_result = None
        try:
            self.build()
        finally:
            self._compute_only = False
        return self._compute_result

    def _init_moveit_commander(self):
        """Lazy-init moveit_commander, safe to call after rospy.init_node()."""
        if getattr(self, '_mc_scene', None) is not None:
            return True
        try:
            import moveit_commander
            from moveit_commander import PlanningSceneInterface
        except ImportError as exc:
            rospy.logwarn(
                "moveit_commander unavailable (%s); payload disabled.", exc)
            return False
        try:
            # roscpp_initialize is a no-op when rospy is already initialized,
            # but calling it here makes the intent explicit.
            moveit_commander.roscpp_initialize([])
            self._mc_scene = PlanningSceneInterface(synchronous=True)
            return True
        except Exception as exc:
            rospy.logwarn(
                "moveit_commander init failed (%s); payload disabled.", exc)
            return False

    def _attach_payload(self):
        """Attach the nominal payload box to the EE planning scene (§5.6).

        Enables payload-aware atlas builds for per-box-size reachability
        sampling. The box is attached to ``~payload_attach_link`` (default the
        IK link) so state-validity checks reflect the carried load.
        No-op when ``~payload_enabled`` is false (empty-load atlas).
        """
        if not self._payload_enabled:
            return
        if self._payload_shape != "box":
            rospy.logwarn(
                "payload_shape=%s unsupported (only 'box'); building empty-load.",
                self._payload_shape)
            return
        if not self._init_moveit_commander():
            if not self._attach_payload_rclpy():
                if getattr(self, "_rclpy_node", None) is not None:
                    raise RuntimeError(
                        "payload requested but the planning scene attach failed")
            return
        try:
            scene = self._mc_scene
            pose = PoseStamped()
            pose.header.frame_id = self._payload_attach_link
            pose.header.stamp = rospy.Time(0)
            pose.pose.position.x = self._payload_offset[0]
            pose.pose.position.y = self._payload_offset[1]
            pose.pose.position.z = self._payload_offset[2]
            pose.pose.orientation.w = 1.0
            scene.attach_box(
                self._payload_attach_link, self._payload_id, pose,
                self._payload_size, touch_links=self._payload_touch_links)
            rospy.sleep(1.0)  # let the attached object propagate to move_group
            rospy.loginfo(
                "Payload attached: shape=box size=%s link=%s offset=%s touch=%s",
                self._payload_size, self._payload_attach_link,
                self._payload_offset, self._payload_touch_links)
        except Exception as exc:
            rospy.logerr("Failed to attach payload (building empty-load): %s", exc)
            if getattr(self, "_rclpy_node", None) is not None:
                raise

    def _attach_payload_rclpy(self):
        """Attach the payload through ApplyPlanningScene when MoveIt Commander is absent."""
        node = getattr(self, "_rclpy_node", None)
        if node is None or not self._payload_enabled:
            return False
        import rclpy
        from moveit_msgs.srv import ApplyPlanningScene
        from luggage_planning.planning_scene_client import build_attach_scene
        client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        if not client.wait_for_service(timeout_sec=30.0):
            rospy.logerr("apply_planning_scene unavailable; payload not attached")
            return False
        scene = build_attach_scene(
            self._payload_id,
            [float(v) for v in self._payload_offset],
            (0.0, 0.0, 0.0, 1.0),
            [float(v) for v in self._payload_size],
            self._payload_attach_link,
            touch_links=self._payload_touch_links,
            frame_id=self._payload_attach_link,
        )
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
        result = future.result()
        if result is None or not result.success:
            rospy.logerr("payload apply failed: %s", future.exception())
            return False
        rospy.sleep(1.0)
        rospy.loginfo(
            "Payload attached via ApplyPlanningScene: size=%s link=%s offset=%s",
            self._payload_size, self._payload_attach_link, self._payload_offset)
        return True

    def build(self):
        """Build the full atlas and save to files."""
        nx = int(math.ceil(self._inner_l / self._resolution))
        ny = int(math.ceil(self._inner_w / self._resolution))
        nz = int(math.ceil((self._ceiling_z - self._z_min_offset - self._z_max_offset)
                            / self._resolution))
        nyaw = len(self._yaw_bins)
        total = nx * ny * nz * nyaw
        rospy.loginfo("Grid: %dx%dx%dx%d = %d cells (resolution=%.2f, avoid_collisions=%s)",
                      nx, ny, nz, nyaw, total, self._resolution, self._avoid_collisions)
        if self._max_workers != 1:
            rospy.logwarn(
                "Opening-connected wavefront uses the deterministic serial "
                "baseline; ignoring max_workers=%d.", self._max_workers)

        # Optionally sync scene for collision-aware IK.
        if (self._avoid_collisions
                and not getattr(self, '_compute_only', False)
                and not getattr(self, '_scene_already_synced', False)):
            try:
                from std_srvs.srv import Trigger
                sync = rospy.ServiceProxy("/scene_manager/sync_static_scene", Trigger)
                sync.wait_for_service(timeout=5.0)
                resp = sync()
                if resp.success:
                    rospy.loginfo("Scene synced for collision-aware atlas.")
                else:
                    rospy.logwarn("Scene sync failed: %s. Atlas may be kinematic-only.", resp.message)
            except Exception as exc:
                rospy.logwarn("Cannot sync scene: %s. Atlas may be kinematic-only.", exc)
            rospy.sleep(2.0)  # Let scene settle.

        # Attach the nominal payload (§5.6) before IK sampling so the reachable
        # set reflects the carried box. No-op when payload_enabled is false.
        self._attach_payload()

        shape4 = (nx, ny, nz, nyaw)
        status = np.full(shape4, UNKNOWN, dtype=np.uint8)
        contact_ik = np.zeros(shape4, dtype=np.bool_)
        transit_ik = np.zeros(shape4, dtype=np.bool_)
        contact_seeds = np.full(
            shape4 + (self._max_branches, 6), np.nan, dtype=np.float64)
        transit_seeds = np.full_like(contact_seeds, np.nan)
        # ST06: per-branch lift the transit seed was solved at (vertical
        # branches) or NaN (internal branches carry the predecessor pose),
        # plus the container-frame target recorded when that branch was
        # solved. Internal branches copy the predecessor's target; they do
        # not store FK of the joints being certified.
        transit_lift = np.full(
            shape4 + (self._max_branches,), np.nan, dtype=np.float32)
        transit_target = np.full(
            shape4 + (self._max_branches, 3), np.nan, dtype=np.float32)
        lift_clearance = np.full(shape4, np.nan, dtype=np.float32)
        branch_kind = np.zeros(shape4, dtype=np.uint8)
        solution_count = np.zeros(shape4, dtype=np.uint8)
        joint_margin = np.zeros(shape4, dtype=np.float32)
        manipulability = np.zeros((nx, ny, nz, nyaw), dtype=np.float32)
        branches_by_yaw = [{} for _ in range(nyaw)]
        t0 = time.time()
        rospy.loginfo(
            "Computing deterministic opening wavefront: fixed_seeds=%d "
            "max_branches=%d workers=1", len(self._fixed_seeds),
            self._max_branches)

        for iyaw in range(nyaw):
            branch_map = branches_by_yaw[iyaw]
            attempted = set()
            indeterminate_cells = set()

            def _expand(cell, predecessors, is_anchor):
                ix, iy, iz = cell
                _x, _y, contact_z, _yaw = self._cell_target(ix, iy, iz, iyaw)
                clearance = adaptive_lift_clearance(
                    contact_z, self._transit_clearance,
                    self._safe_tool_z, self._min_lift)
                candidates = []
                indeterminate = cell in indeterminate_cells
                any_transit = bool(transit_ik[ix, iy, iz, iyaw])
                any_contact = bool(contact_ik[ix, iy, iz, iyaw])
                internal_tried = set()

                propagated = []
                for predecessor_cell in predecessors:
                    for predecessor_branch in branch_map[predecessor_cell]:
                        key = (
                            cell, predecessor_cell,
                            tuple(predecessor_branch["transit"]))
                        if key in attempted:
                            continue
                        attempted.add(key)
                        propagated.append((
                            predecessor_branch["transit"],
                            predecessor_branch["transit"],
                            False,
                            predecessor_branch["transit_target"],
                        ))

                attempts = []
                if is_anchor:
                    attempts.extend((seed, None, False, None)
                                    for seed in self._fixed_seeds)
                attempts.extend(propagated)

                def _consume(result):
                    nonlocal indeterminate, any_transit, any_contact
                    branch, transit_ok, contact_ok, unknown = result
                    any_transit = any_transit or transit_ok
                    any_contact = any_contact or contact_ok
                    indeterminate = indeterminate or unknown
                    if branch is not None:
                        candidates.append(branch)

                def _call(fn):
                    try:
                        return fn()
                    except RuntimeError as exc:
                        rospy.logwarn_throttle(5.0, "%s", exc)
                        return None, False, False, True

                def _run(attempt_list):
                    for seed, predecessor, repair, predecessor_target in (
                            attempt_list):
                        if clearance is not None:
                            result = _call(
                                lambda seed=seed, predecessor=predecessor,
                                repair=repair: self._try_vertical(
                                    ix, iy, iz, iyaw, seed, clearance,
                                    predecessor=predecessor, repair=repair))
                            _consume(result)
                            if result[0] is not None:
                                continue
                        # Opening anchors are not climbed from a lower cell.
                        # The wavefront enters the layer through an interior
                        # neighbor of a lower, already connected cell.
                        # A vertical success above keeps this cell's own
                        # (x, y, z + clearance) target, not the predecessor's.
                        if is_anchor or predecessor is None:
                            continue
                        pred_key = tuple(float(v) for v in predecessor)
                        if pred_key in internal_tried:
                            continue
                        internal_tried.add(pred_key)
                        _consume(_call(
                            lambda predecessor=predecessor, repair=repair,
                            predecessor_target=predecessor_target:
                            self._try_internal(
                                ix, iy, iz, iyaw, predecessor, repair=repair,
                                predecessor_target=predecessor_target)))

                _run(attempts)
                # Fixed seeds are a repair mechanism only after connected
                # warm starts fail; every attempt uses the identical target.
                if not is_anchor and predecessors and not candidates:
                    repairs = []
                    for predecessor_cell in predecessors:
                        for predecessor_branch in branch_map[predecessor_cell]:
                            repairs.extend(
                                (seed, predecessor_branch["transit"], True,
                                 predecessor_branch["transit_target"])
                                for seed in self._fixed_seeds)
                    _run(repairs)

                selected = select_distinct_branches(
                    candidates, self._max_branches, self._branch_distance)
                transit_ik[ix, iy, iz, iyaw] = any_transit
                contact_ik[ix, iy, iz, iyaw] = any_contact
                if indeterminate:
                    indeterminate_cells.add(cell)
                status[ix, iy, iz, iyaw] = classify_cell(
                    selected, indeterminate, self._marginal_margin)
                if selected:
                    lift_clearance[ix, iy, iz, iyaw] = (
                        selected[0]["lift_clearance"])
                    branch_kind[ix, iy, iz, iyaw] = selected[0]["kind"]
                else:
                    branch_kind[ix, iy, iz, iyaw] = BRANCH_NONE
                    lift_clearance[ix, iy, iz, iyaw] = (
                        clearance if clearance is not None else 0.0)
                if not selected:
                    return False

                branch_map[cell] = selected
                solution_count[ix, iy, iz, iyaw] = len(selected)
                joint_margin[ix, iy, iz, iyaw] = min(
                    branch["margin"] for branch in selected)
                for branch_index, branch in enumerate(selected):
                    transit_seeds[ix, iy, iz, iyaw, branch_index] = (
                        branch["transit"])
                    contact_seeds[ix, iy, iz, iyaw, branch_index] = (
                        branch["contact"])
                    transit_lift[ix, iy, iz, iyaw, branch_index] = (
                        branch["lift_clearance"]
                        if branch["kind"] == BRANCH_VERTICAL else np.nan)
                    transit_target[ix, iy, iz, iyaw, branch_index] = (
                        branch["transit_target"])
                return True

            deterministic_wavefront(
                (nx, ny, nz), self._opening_axis, self._opening_sign, _expand)
            rospy.loginfo(
                "Yaw %d/%d complete: connected=%d elapsed=%.1fs",
                iyaw + 1, nyaw, len(branch_map), time.time() - t0)

        opening_connected = solution_count > 0
        neighbor_confidence = np.zeros(shape4, dtype=np.float32)
        status_before_neighbor_check = status.copy()
        for iyaw in range(nyaw):
            for ix in range(nx):
                for iy in range(ny):
                    for iz in range(nz):
                        cell_status = status_before_neighbor_check[
                            ix, iy, iz, iyaw]
                        neighbors = grid_neighbors(
                            (ix, iy, iz), (nx, ny, nz))
                        if not neighbors:
                            continue
                        consistent = sum(
                            status_before_neighbor_check[
                                nx_i, ny_i, nz_i, iyaw] == cell_status
                            for nx_i, ny_i, nz_i in neighbors)
                        confidence = float(consistent) / len(neighbors)
                        neighbor_confidence[ix, iy, iz, iyaw] = confidence
                        if (cell_status == REACHABLE and
                                confidence < self._neighbor_threshold):
                            status[ix, iy, iz, iyaw] = MARGINAL

        elapsed = time.time() - t0
        reachable_count = int(np.count_nonzero(status == REACHABLE))
        marginal_count = int(np.count_nonzero(status == MARGINAL))
        unreachable_count = int(np.count_nonzero(status == UNREACHABLE))
        unknown_count = int(np.count_nonzero(status == UNKNOWN))
        rospy.loginfo(
            "Done in %.1fs: reachable=%d marginal=%d unreachable=%d unknown=%d",
            elapsed, reachable_count, marginal_count, unreachable_count,
            unknown_count)

        # Metadata.
        scene_tf_path = rospy.get_param(
            "~scene_tf_config", self._resolve_scene_tf_config_path())
        urdf = rospy.get_param("/robot_description", "")
        mode = "collision_aware" if self._avoid_collisions else "kinematic"
        opening_side = (
            ("positive_" if self._opening_sign > 0 else "negative_") +
            "xyz"[self._opening_axis])
        deterministic_hash = hashlib.sha256()
        deterministic_hash.update(status.tobytes())
        deterministic_hash.update(
            np.nan_to_num(contact_seeds, nan=0.0).tobytes())
        deterministic_hash.update(
            np.nan_to_num(transit_seeds, nan=0.0).tobytes())
        deterministic_hash.update(str(
            [self._payload_enabled, self._payload_size, self._payload_shape,
             self._payload_offset, self._payload_attach_link]).encode())
        deterministic_hash.update(np.nan_to_num(lift_clearance, nan=-1.0).tobytes())
        deterministic_hash.update(branch_kind.tobytes())
        deterministic_hash.update(np.nan_to_num(
            transit_lift, nan=-1.0).tobytes())
        deterministic_hash.update(np.nan_to_num(
            transit_target, nan=0.0).tobytes())
        deterministic_hash.update(str(
            [self._tool_down_tolerance, self._transit_clearance,
             self._safe_tool_z, self._min_lift, "adaptive_v3"]).encode())
        payload_section = {
            "enabled": self._payload_enabled,
            "shape": self._payload_shape if self._payload_enabled else None,
            "size": list(self._payload_size) if self._payload_enabled else None,
            "attach_link": self._payload_attach_link,
            "offset": list(self._payload_offset) if self._payload_enabled else None,
            "touch_links": list(self._payload_touch_links) if self._payload_enabled else None,
        }
        # ST06/F14 v2 contract block. The identity hashes reproduce from the
        # artifact's own geometry/payload sections, so an existing v1 NPZ
        # cannot be relabelled as v2 by stamping metadata onto it.
        scene_tf_hash = _file_hash(scene_tf_path)
        from luggage_planning.atlas.runtime import (
            compute_reachability_scene_hash,
        )
        reachability_scene_hash = compute_reachability_scene_hash(
            scene_tf_path)
        urdf_hash = hashlib.md5(urdf.encode()).hexdigest()
        contract = atlas_contract_record(
            self._yaw_bins,
            scene_tf_hash,
            urdf_hash,
            _contract_geometry_hash(
                [self._inner_l, self._inner_w, self._inner_h],
                self._floor_z, self._ceiling_z, opening_side),
            _contract_payload_hash(payload_section),
        )
        meta = {
            "atlas_version": "3.0",
            "schema_version": 3,
            "contract": contract,
            "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime()),
            "computation_time_sec": round(elapsed, 2),
            "mode": mode,
            "marginal_joint_margin": self._marginal_margin,
            "query": {
                "yaw_tolerance": float(rospy.get_param(
                    "~query_yaw_tolerance", 0.10)),
                "hard_reject_min_neighbor_confidence": float(
                    rospy.get_param(
                        "~hard_reject_min_neighbor_confidence", 0.90)),
                "hard_reject_interior_fraction": float(rospy.get_param(
                    "~hard_reject_interior_fraction", 0.10)),
            },
            "builder": {
                "algorithm": "opening_connected_wavefront_v3",
                "serial_deterministic": True,
                "max_branches": self._max_branches,
                "branch_distance": self._branch_distance,
                "joint_interpolation_step": self._joint_interp_step,
                "state_validity_available": self._state_validity is not None,
                "seed_policy": "fixed_anchor_connected_warm_start_repair_v1",
                "ik_solver": "offline_ik.refine_to_pose",
                "deterministic_fingerprint": deterministic_hash.hexdigest(),
                "tool_down_tolerance": self._tool_down_tolerance,
                "transit_clearance": self._transit_clearance,
            },
            "dependencies": {
                "robot_model": rospy.get_param("/robot_name", "elfin_s20_with_camera"),
                "scene_tf_hash": scene_tf_hash,
                "reachability_scene_hash": reachability_scene_hash,
                "urdf_hash": urdf_hash,
                "ik_link": self._ik_link,
                "ik_group": self._ik_group,
            },
            "payload": payload_section,
            "grid": {
                "frame": "container_link",
                "resolution_xyz": self._resolution,
                "origin": [-self._inner_l * 0.5, -self._inner_w * 0.5, self._z_min_offset],
                "size": [nx, ny, nz],
                "yaw_bins": self._yaw_bins,
                "yaw_semantics": "directed_container_2pi",
                "transit_clearance": self._transit_clearance,
                "lift_policy": {
                    "nominal": self._transit_clearance,
                    "safe_tool_z": self._safe_tool_z,
                    "min_lift": self._min_lift,
                    "frame": "container_link",
                },
            },
            "container": {
                "inner_dimensions": [self._inner_l, self._inner_w, self._inner_h],
                "floor_z": self._floor_z,
                "ceiling_z": self._ceiling_z,
                "usable_height": self._ceiling_z - self._floor_z,
                "base_xyz": self._container_xyz,
                "base_rpy": self._container_rpy,
                "opening_side": opening_side,
            },
            "stats": {
                "total_cells": total,
                "reachable_cells": reachable_count,
                "marginal_cells": marginal_count,
                "unreachable_cells": unreachable_count,
                "unknown_cells": unknown_count,
                "reachability_rate": round(reachable_count / max(1, total), 4),
            },
        }

        # Compute v1 compatibility aliases.
        reachable = status >= MARGINAL
        legacy_seeds = np.zeros(shape4 + (6,), dtype=np.float64)
        has_solution = solution_count > 0
        legacy_seeds[has_solution] = contact_seeds[..., 0, :][has_solution]

        data = {
            "status": status,
            "opening_connected": opening_connected,
            "contact_ik": contact_ik,
            "transit_ik": transit_ik,
            "contact_seeds": contact_seeds,
            "transit_seeds": transit_seeds,
            "transit_lift": transit_lift,
            "transit_target": transit_target,
            "lift_clearance": lift_clearance,
            "branch_kind": branch_kind,
            "solution_count": solution_count,
            "joint_margin": joint_margin,
            "manipulability": manipulability,
            "neighbor_confidence": neighbor_confidence,
            "reachable": reachable,
            "seed_joints": legacy_seeds,
        }

        # Compute-only mode: return data without saving (for layout evaluator).
        if getattr(self, '_compute_only', False):
            self._compute_result = (data, meta)
            return

        # Save the planned v2 schema plus compatibility aliases for v1 readers.
        os.makedirs(self._output_dir, exist_ok=True)
        suffix = mode
        if self._payload_enabled:
            # Per-box-size sampling: tag the file with payload dims so different
            # box shapes/ sizes produce distinct atlases.
            suffix += "_payload_%.2fx%.2fx%.2f" % (
                self._payload_size[0], self._payload_size[1], self._payload_size[2])
        if self._tool_down_tolerance > 0.0:
            # Tag with transit tolerance so different limits produce distinct files.
            suffix += "_tol%.3f" % self._tool_down_tolerance
        prefix = self._atlas_prefix
        if not prefix:
            robot_model = rospy.get_param("/robot_name", "elfin_s20_with_camera")
            prefix = str(robot_model)
            if prefix.startswith("elfin_"):
                prefix = prefix[len("elfin_"):]
            if prefix.endswith("_with_camera"):
                prefix = prefix[: -len("_with_camera")]
            if not prefix:
                prefix = "s20"
        npz_path = os.path.join(self._output_dir, "%s_container_%s.npz" % (prefix, suffix))
        meta_path = os.path.join(self._output_dir, "%s_container_%s.yaml" % (prefix, suffix))
        np.savez_compressed(npz_path, **data)
        with open(meta_path, "w", encoding="utf-8") as stream:
            yaml.safe_dump(
                meta, stream, default_flow_style=False, sort_keys=False)
        rospy.loginfo("Saved atlas to %s + %s", npz_path, meta_path)


def main():
    rospy.init_node("reachability_atlas_builder")
    builder = ReachabilityAtlasBuilder()
    builder.build()
    rospy.loginfo("Reachability atlas builder done.")


if __name__ == "__main__":
    main()
