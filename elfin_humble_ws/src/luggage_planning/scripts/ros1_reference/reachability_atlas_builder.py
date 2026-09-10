#!/usr/bin/env python3
"""Offline reachability atlas builder.

Pre-computes a (x,y,z,yaw) reachability grid by calling /compute_ik for
each cell, with optional container-collision awareness. Saves results
to .npz + .yaml for runtime loading.

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
from collections import deque

import numpy as np
import yaml

try:  # Keep the pure wavefront helpers importable without a ROS installation.
    import rospy
    import rospkg
    from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
    from moveit_msgs.msg import PositionIKRequest, RobotState
    from moveit_msgs.srv import (
        GetPositionIK,
        GetStateValidity,
        GetStateValidityRequest,
    )
    from sensor_msgs.msg import JointState
except ImportError:  # pragma: no cover - exercised by the no-ROS unit tests.
    rospy = None
    rospkg = None


UNKNOWN = np.uint8(0)
UNREACHABLE = np.uint8(1)
MARGINAL = np.uint8(2)
REACHABLE = np.uint8(3)


def grid_neighbors(cell, shape):
    """Return deterministic 6-connected spatial neighbors."""
    ix, iy, iz = cell
    nx, ny, nz = shape
    result = []
    for axis in range(3):
        for delta in (-1, 1):
            candidate = [ix, iy, iz]
            candidate[axis] += delta
            if (0 <= candidate[0] < nx and
                    0 <= candidate[1] < ny and
                    0 <= candidate[2] < nz):
                result.append(tuple(candidate))
    return tuple(sorted(result))


def opening_boundary_cells(shape, opening_axis, opening_sign):
    """Return sorted grid cells touching the configured opening face."""
    boundary_index = shape[opening_axis] - 1 if opening_sign > 0 else 0
    cells = []
    for ix in range(shape[0]):
        for iy in range(shape[1]):
            for iz in range(shape[2]):
                cell = (ix, iy, iz)
                if cell[opening_axis] == boundary_index:
                    cells.append(cell)
    return tuple(sorted(cells))


def deterministic_wavefront(shape, opening_axis, opening_sign, can_expand):
    """Traverse only cells connected to a successful opening boundary.

    ``can_expand(cell, connected_predecessors, is_anchor)`` performs the
    application-specific work and returns true when the cell may expand the
    frontier.  The queue and predecessor order are stable across runs.
    """
    anchors = opening_boundary_cells(shape, opening_axis, opening_sign)
    anchor_set = set(anchors)
    queue = deque(anchors)
    queued = set(anchors)
    connected = set()
    order = []
    while queue:
        cell = queue.popleft()
        queued.discard(cell)
        predecessors = tuple(
            neighbor for neighbor in grid_neighbors(cell, shape)
            if neighbor in connected
        )
        if can_expand(cell, predecessors, cell in anchor_set):
            if cell not in connected:
                connected.add(cell)
                order.append(cell)
                for neighbor in grid_neighbors(cell, shape):
                    if neighbor not in connected and neighbor not in queued:
                        queue.append(neighbor)
                        queued.add(neighbor)
    return tuple(order)


def joint_interpolation_samples(start, goal, max_step):
    """Return deterministic interior samples for a local joint segment."""
    if len(start) != len(goal) or max_step <= 0.0:
        raise ValueError("joint vectors must match and max_step must be positive")
    max_delta = max(abs(float(b) - float(a)) for a, b in zip(start, goal))
    segments = max(1, int(math.ceil(max_delta / max_step)))
    samples = [
        tuple(float(a) + (float(b) - float(a)) * step / segments
              for a, b in zip(start, goal))
        for step in range(1, segments + 1)
    ]
    samples[-1] = tuple(float(value) for value in goal)
    return tuple(samples)


def joint_branch_distance(left, right):
    """Maximum per-joint distance, accounting for equivalent 2*pi wraps."""
    distances = []
    for a, b in zip(left, right):
        delta = float(a) - float(b)
        distances.append(abs(
            (delta + math.pi) % (2.0 * math.pi) - math.pi))
    return max(distances) if distances else 0.0


def select_distinct_branches(branches, limit, threshold):
    """Sort, deduplicate, and bound branch dictionaries deterministically."""
    ranked = sorted(
        branches,
        key=lambda branch: (
            -float(branch["margin"]),
            tuple(round(float(v), 10) for v in branch["transit"]),
            tuple(round(float(v), 10) for v in branch["contact"]),
        ),
    )
    selected = []
    for branch in ranked:
        duplicate = any(
            joint_branch_distance(branch["transit"], prior["transit"]) < threshold
            and joint_branch_distance(branch["contact"], prior["contact"]) < threshold
            for prior in selected
        )
        if not duplicate:
            selected.append(branch)
        if len(selected) >= limit:
            break
    return selected


def classify_cell(branches, indeterminate=False, marginal_margin=0.10):
    """Classify one cell conservatively from connected branch evidence."""
    if not branches:
        return UNKNOWN if indeterminate else UNREACHABLE
    if indeterminate or any(
            branch.get("repair", False) or
            float(branch["margin"]) < marginal_margin
            for branch in branches):
        return MARGINAL
    return REACHABLE

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
        self._scene_config = load_scene_tf_config(
            rospy.get_param("~scene_tf_config", resolve_scene_tf_config_path())
        )
        self._resolution = float(rospy.get_param("~resolution_xyz", 0.15))
        self._yaw_bins = [float(y) for y in rospy.get_param(
            "~yaw_bins", [0.0, math.pi / 2])]
        self._transit_clearance = float(rospy.get_param("~transit_clearance", 0.30))
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
        self._output_dir = rospy.get_param(
            "~output_dir",
            os.path.join(rospkg.RosPack().get_path("luggage_planning"),
                         "data", "reachability_atlas"),
        )
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
        self._opening_axis, self._opening_sign = container_opening_side(
            self._scene_config)

        # Fixed seeds are used only for opening anchors and branch repair.
        self._fixed_seeds = self._load_fixed_seeds()

        # IK service.
        ik_service = rospy.get_param("~ik_service", "/compute_ik")
        rospy.loginfo("Waiting for IK service %s ...", ik_service)
        rospy.wait_for_service(ik_service, timeout=60.0)
        self._ik = rospy.ServiceProxy(ik_service, GetPositionIK)
        rospy.loginfo("IK service ready.")

        validity_service = rospy.get_param(
            "~state_validity_service", "/check_state_validity")
        self._state_validity = None
        try:
            rospy.wait_for_service(validity_service, timeout=2.0)
            self._state_validity = rospy.ServiceProxy(
                validity_service, GetStateValidity)
            rospy.loginfo("State validity service ready.")
        except rospy.ROSException:
            rospy.logwarn(
                "%s unavailable: interpolation-dependent cells will be UNKNOWN.",
                validity_service)

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
        """Construct suction_contact_frame target pose in base_link."""
        # Position: container_link (x,y,z) -> base_link.
        local_pos = (x, y, z)
        base_pos = (
            self._container_xyz[0] + self._container_R[0][0] * x + self._container_R[0][1] * y + self._container_R[0][2] * z,
            self._container_xyz[1] + self._container_R[1][0] * x + self._container_R[1][1] * y + self._container_R[1][2] * z,
            self._container_xyz[2] + self._container_R[2][0] * x + self._container_R[2][1] * y + self._container_R[2][2] * z,
        )
        # Orientation is defined entirely in base_link and exactly matches
        # placement_motion_filter_node._tool_down_quaternion().  Container
        # roll/pitch/yaw must not tilt the suction normal away from base -Z.
        base_yaw_quat = (
            0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))
        base_quat = _quat_multiply(base_yaw_quat, self._tool_down)
        return base_pos, base_quat

    def _solve_ik(self, pos, quat, seed):
        """Call /compute_ik with an explicit seed and return the joint solution.

        ``None`` means a deterministic IK miss.  Service failures are raised so
        the caller can classify the cell UNKNOWN instead of inventing a
        negative reachability result.
        """
        req = PositionIKRequest()
        req.group_name = self._ik_group
        req.ik_link_name = self._ik_link
        req.pose_stamped = PoseStamped()
        req.pose_stamped.header.frame_id = self._base_frame
        req.pose_stamped.header.stamp = rospy.Time(0)
        req.pose_stamped.pose = Pose(
            position=Point(x=pos[0], y=pos[1], z=pos[2]),
            orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]),
        )
        req.avoid_collisions = self._avoid_collisions
        req.timeout = rospy.Duration(self._ik_timeout)
        if seed is None or len(seed) != len(DEFAULT_JOINT_NAMES):
            raise ValueError("IK seed must contain exactly six joints")
        req.robot_state = RobotState(joint_state=JointState(
            name=list(DEFAULT_JOINT_NAMES),
            position=list(seed),
        ))
        try:
            resp = self._ik(req)
        except rospy.ServiceException as exc:
            raise RuntimeError("compute_ik service failed: %s" % exc)
        if resp.error_code.val != resp.error_code.SUCCESS:
            return None
        sol = resp.solution.joint_state
        joint_map = dict(zip(sol.name, sol.position))
        if any(name not in joint_map for name in DEFAULT_JOINT_NAMES):
            raise RuntimeError("compute_ik returned an incomplete joint state")
        return tuple(float(joint_map[name]) for name in DEFAULT_JOINT_NAMES)

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

    def _try_branch(self, ix, iy, iz, iyaw, transit_seed,
                    predecessor=None, repair=False):
        """Solve transit then contact on one continuous physical branch."""
        x, y, z, yaw = self._cell_target(ix, iy, iz, iyaw)
        contact_pos, contact_quat = self._construct_base_pose(x, y, z, yaw)
        transit_z = z + self._transit_clearance
        # Deliberately solve the real target even when it is above the nominal
        # container height.  Above-container transit is never auto-accepted.
        transit_pos, transit_quat = self._construct_base_pose(
            x, y, transit_z, yaw)
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
        branch = {
            "transit": transit,
            "contact": contact,
            "margin": min(
                self._joint_margin(transit), self._joint_margin(contact)),
            "repair": bool(repair),
        }
        return branch, True, True, False

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
        IK link) so /compute_ik collision checks reflect the carried load.
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
        if self._avoid_collisions and not getattr(self, '_compute_only', False):
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
                candidates = []
                indeterminate = cell in indeterminate_cells
                any_transit = bool(transit_ik[ix, iy, iz, iyaw])
                any_contact = bool(contact_ik[ix, iy, iz, iyaw])

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
                        ))

                attempts = []
                if is_anchor:
                    attempts.extend((seed, None, False)
                                    for seed in self._fixed_seeds)
                attempts.extend(propagated)

                def _run(attempt_list):
                    nonlocal indeterminate, any_transit, any_contact
                    for seed, predecessor, repair in attempt_list:
                        try:
                            branch, transit_ok, contact_ok, unknown = (
                                self._try_branch(
                                    ix, iy, iz, iyaw, seed,
                                    predecessor=predecessor, repair=repair))
                        except RuntimeError as exc:
                            rospy.logwarn_throttle(5.0, "%s", exc)
                            branch, transit_ok, contact_ok, unknown = (
                                None, False, False, True)
                        any_transit = any_transit or transit_ok
                        any_contact = any_contact or contact_ok
                        indeterminate = indeterminate or unknown
                        if branch is not None:
                            candidates.append(branch)

                _run(attempts)
                # Fixed seeds are a repair mechanism only after connected
                # warm starts fail; every attempt uses the identical target.
                if not is_anchor and predecessors and not candidates:
                    repairs = []
                    for predecessor_cell in predecessors:
                        for predecessor_branch in branch_map[predecessor_cell]:
                            repairs.extend(
                                (seed, predecessor_branch["transit"], True)
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
        deterministic_hash.update(str(
            [self._tool_down_tolerance, self._transit_clearance]).encode())
        payload_section = {
            "enabled": self._payload_enabled,
            "shape": self._payload_shape if self._payload_enabled else None,
            "size": list(self._payload_size) if self._payload_enabled else None,
            "attach_link": self._payload_attach_link,
            "offset": list(self._payload_offset) if self._payload_enabled else None,
            "touch_links": list(self._payload_touch_links) if self._payload_enabled else None,
        }
        meta = {
            "atlas_version": "2.0",
            "schema_version": 2,
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
                "algorithm": "opening_connected_wavefront_v2",
                "serial_deterministic": True,
                "max_branches": self._max_branches,
                "branch_distance": self._branch_distance,
                "joint_interpolation_step": self._joint_interp_step,
                "state_validity_available": self._state_validity is not None,
                "seed_policy": "fixed_anchor_connected_warm_start_repair_v1",
                "deterministic_fingerprint": deterministic_hash.hexdigest(),
                "tool_down_tolerance": self._tool_down_tolerance,
                "transit_clearance": self._transit_clearance,
            },
            "dependencies": {
                "robot_model": rospy.get_param("/robot_name", "elfin_s20_with_camera"),
                "scene_tf_hash": _file_hash(scene_tf_path),
                "urdf_hash": hashlib.md5(urdf.encode()).hexdigest(),
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
                "transit_clearance": self._transit_clearance,
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
