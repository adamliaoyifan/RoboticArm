"""Measured grasp-local reachability transport, separate from rigid screening.

The witness contains untimed joint paths, bound to the exact snapshot and G.
It is diagnostic/adoption evidence, never a controller command.
"""

import hashlib
import json
import math
import threading
import time

from luggage_planning.placement_prior.core import CollisionVerdict
from luggage_planning.placement_prior.contracts import Transform


def local_query(request, placement, grasp):
    """Use production pick clearances, and a 10 cm container-Z place corridor.

    The place corridor is a bounded local feasibility probe; production route
    generation still owns the actual approach and retreat and must certify it.
    """
    if (
        not request.measured
        or request.world_from_pick_box is None
        or len(grasp.pick_references) != 4
    ):
        raise ValueError("measured pickup and production pick references required")
    contact = request.world_from_pick_box.compose(grasp.box_from_tcp)
    if contact != grasp.pick_references[2]:
        # Composition introduces ordinary float roundoff; never round inputs.
        import numpy as np

        if (
            not np.allclose(
                contact.translation,
                grasp.pick_references[2].translation,
                atol=1e-9,
                rtol=0,
            )
            or abs(
                sum(
                    a * b
                    for a, b in zip(
                        contact.quaternion_xyzw,
                        grasp.pick_references[2].quaternion_xyzw,
                    )
                )
            )
            < 1 - 1e-9
        ):
            raise ValueError("pickup contact and G disagree")
    box = request.snapshot.world_from_container.compose(placement.container_from_box)
    p = placement.container_from_box
    raised = Transform(
        (p.translation[0], p.translation[1], p.translation[2] + 0.10), p.quaternion_xyzw
    )
    entry = request.snapshot.world_from_container.compose(raised).compose(
        grasp.box_from_tcp
    )
    return dict(
        pick_box_world=request.world_from_pick_box,
        place_box_world=box,
        box_from_tcp=grasp.box_from_tcp,
        pick_approach_world=grasp.pick_references[0],
        pick_lift_world=grasp.pick_references[3],
        place_entry_world=entry,
        place_retreat_world=entry,
    )


def valid_branch_paths(branches):
    """Reject truncated/nonfinite or disconnected branch evidence at transport."""
    if not 1 <= len(branches) <= 2:
        return False
    for branch in branches:
        incoming, outgoing = branch.incoming, branch.outgoing
        names = incoming.joint_names
        if not names or len(set(names)) != len(names) or names != outgoing.joint_names:
            return False
        for path in (incoming, outgoing):
            if len(path.points) < 2:
                return False
            for point in path.points:
                if len(point.positions) != len(names) or not all(
                    math.isfinite(v) for v in point.positions
                ):
                    return False
        if incoming.points[-1].positions != outgoing.points[0].positions:
            return False
    return True


class NativeLocalPathPort:
    def __init__(self, node, recorder=None):
        from luggage_msgs.srv import CheckGraspLocalPaths

        self.service = CheckGraspLocalPaths
        self.client = node.create_client(
            CheckGraspLocalPaths, "/placement_prior/check_local_paths"
        )
        self.recorder = recorder
        self.inflight = None

    def check(self, request, placement, grasp, *, deadline, cancelled):
        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import PlanningScene
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.convert import message_to_ordereddict

        if cancelled() or time.monotonic() >= deadline:
            return CollisionVerdict(False, "CANCELLED_OR_DEADLINE")
        if not self.client.service_is_ready():
            return CollisionVerdict(False, "LOCAL_PATH_BACKEND_UNAVAILABLE")
        if self.inflight is not None and not self.inflight.done():
            return CollisionVerdict(False, "LOCAL_PATH_BACKEND_BUSY")
        query = self.service.Request()
        query.request_id = (
            f"{request.request_id}:{placement.slot_id}:{grasp.candidate_id}"
        )
        query.snapshot_id = hashlib.sha256(
            repr(request.snapshot.identity).encode()
        ).hexdigest()
        query.model_sha256 = request.snapshot.model_hash
        query.scene = deserialize_message(request.snapshot.scene_cdr, PlanningScene)
        for name, t in local_query(request, placement, grasp).items():
            p = Pose()
            p.position.x, p.position.y, p.position.z = t.translation
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (
                t.quaternion_xyzw
            )
            setattr(query, name, p)
        query.box_wdh = list(request.box_wdh)
        query.budget_s = min(2.0, max(0.000001, deadline - time.monotonic()))
        if self.recorder:
            self.recorder("local_request", query)
        event = threading.Event()
        self.inflight = self.client.call_async(query)
        self.inflight.add_done_callback(lambda _: event.set())
        while not event.wait(0.01):
            if cancelled() or time.monotonic() >= deadline:
                return CollisionVerdict(False, "CANCELLED_OR_DEADLINE")
        response = self.inflight.result()
        if self.recorder:
            self.recorder("local_response", response)
        if cancelled() or time.monotonic() >= deadline:
            return CollisionVerdict(False, "CANCELLED_OR_DEADLINE")
        if (
            response.request_id != query.request_id
            or response.snapshot_id != query.snapshot_id
            or response.model_sha256 != query.model_sha256
        ):
            return CollisionVerdict(False, "LOCAL_PATH_IDENTITY_MISMATCH")
        witness = json.dumps(message_to_ordereddict(response))
        valid = (
            response.complete
            and response.reason == "LOCAL_PATHS_VALID"
            and valid_branch_paths(response.pick_branches)
            and valid_branch_paths(response.place_branches)
        )
        reason = response.reason
        if response.reason == "LOCAL_PATHS_VALID" and not valid:
            reason = "INVALID_LOCAL_PATH_WITNESS"
        return CollisionVerdict(bool(valid), reason, local_paths_json=witness)
