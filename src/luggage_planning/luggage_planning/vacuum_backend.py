#!/usr/bin/env python3
"""Vacuum backend abstraction (no ROS): attach state machine + sim follow.

Three layers per the plan:
- ``VacuumBackend`` ABC - the hardware/sim contract. ``HardwareBackend``
  (real GPIO + pressure sensor) is intentionally left unimplemented here;
  the interface is what guarantees the same VacuumCommand works on both.
- ``SimVacuumBackend`` - kinematic follow via an injected ``GzPoseClient``
  (so the state machine is unit-testable with a mock) plus an injected
  planning-scene attacher.
- ``StubVacuumBackend`` - state-only, for environments without Gazebo.

The follow loop itself (timer, TF lookups) lives in the node layer; this
module owns the *decisions*: what attach/detach do to state, how the box
pose is computed from the locked offset.
"""

from __future__ import division

import math


def compose_pose(panel_xyz, panel_quat, offset_xyz, offset_quat):
    """Box pose in world = panel pose * locked relative offset.

    ``offset_*`` is the box pose expressed in the panel frame (captured at
    attach time). Uses the same quaternion math as vacuum_attach_utils.
    """
    from luggage_planning.vacuum_attach_utils import (
        compose_transform,
    )
    return compose_transform(panel_xyz, panel_quat, offset_xyz, offset_quat)


def invert_pose(xyz, quat):
    from luggage_planning.vacuum_attach_utils import invert_transform
    return invert_transform(xyz, quat)


def relative_offset(panel_xyz, panel_quat, box_xyz, box_quat):
    """Offset of the box in the panel frame (captured at attach)."""
    inv_xyz, inv_quat = invert_pose(panel_xyz, panel_quat)
    return compose_pose(inv_xyz, inv_quat, box_xyz, box_quat)


class VacuumBackend(object):
    """Contract for attach/detach implementations."""

    def attach(self, context):
        """Bind the box. ``context`` carries model name, current panel/box
        poses and the planning-scene handle. Returns (ok, message)."""
        raise NotImplementedError

    def detach(self, context):
        """Release the box. Returns (ok, message)."""
        raise NotImplementedError

    def is_attached(self):
        raise NotImplementedError

    def follow_step(self, panel_xyz, panel_quat):
        """Optional kinematic follow. Stub/hardware default is a no-op."""
        del panel_xyz, panel_quat
        return True, ""


class StubVacuumBackend(VacuumBackend):
    """State-only backend (tests, no-Gazebo environments)."""

    def __init__(self):
        self._attached = False

    def attach(self, context):
        self._attached = True
        return True, "stub attached"

    def detach(self, context):
        self._attached = False
        return True, "stub detached"

    def is_attached(self):
        return self._attached


class SimVacuumBackend(VacuumBackend):
    """Simulation backend: gz kinematic follow + planning-scene attach.

    ``gz_client`` must expose ``set_model_pose(model_name, xyz, quat)``
    (the node wraps the bridged /world/<w>/set_pose service).
    ``scene`` must expose ``attach_pickup_box(...)`` /
    ``detach_and_remove(...)`` (planning_scene_client). Both are injected
    so the state machine runs under plain pytest with mocks.
    """

    def __init__(self, gz_client, scene, follow_rate_hz=30.0):
        self._gz = gz_client
        self._scene = scene
        self.follow_rate_hz = float(follow_rate_hz)
        self._attached = False
        self._model_name = None
        self._offset_xyz = None
        self._offset_quat = None
        self.last_error = ""

    def is_attached(self):
        return self._attached

    def attach(self, context):
        if self._attached:
            return True, "already attached"
        try:
            self._model_name = context["model_name"]
            self._offset_xyz, self._offset_quat = relative_offset(
                context["panel_xyz"], context["panel_quat"],
                context["box_xyz"], context["box_quat"])
            scene_ok, scene_msg = self._scene.attach_pickup_box(
                context["model_name"], context["box_xyz"],
                context["box_quat"], context["box_size"],
                context.get("attach_link", "suction_contact_frame"))
            if not scene_ok:
                self.last_error = scene_msg
                return False, "VACUUM_BACKEND_ERROR: scene %s" % scene_msg
        except Exception as exc:  # noqa: BLE001 - backend boundary
            self.last_error = str(exc)
            return False, "VACUUM_BACKEND_ERROR: %s" % exc
        self._attached = True
        return True, "attached %s (follow %.1f Hz)" % (
            self._model_name, self.follow_rate_hz)

    def follow_step(self, panel_xyz, panel_quat):
        """One follow tick: recompute the box pose and push it to gz.

        Called by the node's timer at ``follow_rate_hz``. No-op unless
        attached. Returns (ok, message).
        """
        if not self._attached or self._model_name is None:
            return True, ""
        try:
            xyz, quat = compose_pose(
                panel_xyz, panel_quat, self._offset_xyz, self._offset_quat)
            ok, msg = self._gz.set_model_pose(self._model_name, xyz, quat)
            if not ok:
                return False, msg
        except Exception as exc:  # noqa: BLE001
            return False, "follow error: %s" % exc
        return True, ""

    def detach(self, context):
        if not self._attached:
            return True, "not attached"
        model = self._model_name
        try:
            scene_ok, scene_msg = self._scene.detach_and_remove(model)
        except Exception as exc:  # noqa: BLE001
            self.last_error = str(exc)
            return False, "VACUUM_BACKEND_ERROR: detach %s" % exc
        if not scene_ok:
            self.last_error = scene_msg
            return False, "VACUUM_BACKEND_ERROR: detach %s" % scene_msg
        self._attached = False
        self._model_name = None
        self._offset_xyz = None
        self._offset_quat = None
        # Leave the box where it is; physics takes over.
        return True, "detached %s" % model
