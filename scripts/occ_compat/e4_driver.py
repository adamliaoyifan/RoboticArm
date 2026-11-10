"""e4_driver — the E4 real entry: HardwarePlacePlanner.plan in-process.

The product class runs on the T1 client node with its OWN validity
subscription and gate; compute_placement goes through the REAL
ComputePlacementV2 service. select_place_path is the contract's explicit
stop boundary (planning admission / plan commit — no motion planning),
so the adapter returns a sentinel there AFTER the product's commit
token check has executed. The driver classifies plan() outcomes into
the E4 terminal vocabulary (gate refusals, geometry mismatches, commit
rejections, boundary-reached).
"""
from __future__ import annotations


class T1E4Adapter:
    """compute via the real service; select is the boundary sentinel."""

    def __init__(self, client, e4_node):
        self._client = client
        self._e4_node = e4_node

    def compute_placement(self, *, request_id, box_wdh, height_measured,
                          scene_revision, placed=(), placed_wdh_flat=(),
                          geometry_hash="", expected_map_revision=0,
                          max_candidates=5, conservative_geometry=None):
        from luggage_msgs.srv import ComputePlacementV2
        request = ComputePlacementV2.Request()
        request.box_wdh = [float(v) for v in box_wdh]
        request.height_measured = bool(height_measured)
        request.geometry_hash = str(geometry_hash)
        request.expected_map_revision = int(expected_map_revision)
        request.max_candidates = int(max_candidates)
        # Spin BOTH nodes while the future resolves: the E4 planner's
        # subscriptions must keep delivering (the surface recheck after
        # compute reads the freshest state).
        import time as _time
        future = self._client._chain_clients["placement_v2"].call_async(
            request)
        import rclpy
        deadline = _time.monotonic() + 30.0
        while not future.done() and _time.monotonic() < deadline:
            rclpy.spin_once(self._client, timeout_sec=0.03)
            rclpy.spin_once(self._e4_node, timeout_sec=0.02)
        return future.result() if future.done() else {
            "error": "TIMEOUT"}

    def select_place_path(self, **kwargs):
        # The contract's stop boundary: everything before this return
        # (entry admission, measured geometry checks, the real compute,
        # the surface-identity recheck) has executed; the product's
        # commit token check runs after this returns.
        return {"boundary": "planning_admission_reached",
                "select": "NOT_IN_CHAIN (contract stop boundary)"}


class E4Harness:
    """Owns one product HardwarePlacePlanner bound to the client node."""

    _seq = 0

    def __init__(self, client, backend):
        import rclpy
        E4Harness._seq += 1
        from rclpy.callback_groups import ReentrantCallbackGroup
        from luggage_planning.hardware_place_planning import (
            HardwarePlacePlanner)
        self._client = client
        # A dedicated node that flips to sim time AFTER its
        # subscriptions exist (Node(cli_args=...) breaks the participant
        # on this rmw — zero delivery; the runtime flip is safe once
        # discovery has matched). plan() judges joint age against the
        # node clock, which must be the scenario clock.
        self.node = rclpy.node.Node(
            "t1_e4_%s_%d" % (backend.lower(), E4Harness._seq))
        self._group = ReentrantCallbackGroup()
        self.planner = HardwarePlacePlanner(
            self.node, T1E4Adapter(client, self.node),
            group=self._group,
            # aligned with the chain deployment's robot_state_max_age_ms
            # (the full fixture enumeration legitimately takes ~1 s;
            # the 1.0 s in-process default made the commit stale by
            # ~0.1 s deterministically)
            max_state_age_s=2.0,
            validity_topic="/uld_occ_observer/validity"
            if backend == "O" else "/luggage/cargo_map/validity",
            validity_backend="observer_o" if backend == "O" else "grid_g")
        import time as _t
        import rclpy as _rclpy
        # 1 s of matching with the listener alive, THEN publish the
        # statics (a publish into an already-matched subscriber cannot
        # be lost to discovery races).
        _t.sleep(1.0)
        for _ in range(10):
            _rclpy.spin_once(self.node, timeout_sec=0.05)
        identity_matrix = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        client.broadcast_transform(
            {"child_frame_id": "suction_contact_frame",
             "translation": [0.0, 0.0, 0.1],
             "rotation": identity_matrix},
            client.clock_now_ns(), static=True,
            frame_id="container_link")
        client.broadcast_transform(
            {"child_frame_id": "container_link",
             "translation": [0.0, 0.0, 0.0],
             "rotation": identity_matrix},
            client.clock_now_ns(), static=True, frame_id="world")
        deadline = _t.monotonic() + 10.0
        while _t.monotonic() < deadline:
            _rclpy.spin_once(self.node, timeout_sec=0.05)
            if (self.planner._joints is not None
                    and self.planner._surface is not None
                    and self.planner._geometry is not None):
                break
        # NO sim-time flip: the node stays on the WALL clock and run()
        # publishes its own wall-stamped robot sample, so the joint-age
        # and transition-vs-sample comparisons stay in one (wall) domain.
        # The wall-stamped sample is invisible to later cases' sim gates:
        # every case begins with a producer reset whose INVALID clears
        # the gate's robot sample.

    def run(self, box_wdh, scene_revision, geometry_hash, map_revision):
        """One plan() through the product entry; returns a classification
        dict (never raises past the product's own ValueError surface)."""
        from luggage_planning.hardware_place_planning import MeasuredAttachment
        from luggage_planning.placement_prior_contracts import Transform
        client = self._client
        now = client.clock_now_ns()

        class _Conservative:
            dimensions_wdh = tuple(float(v) for v in box_wdh)

        class _FixtureGeometry:
            # A frozen test-input attachment geometry (contract: robot
            # fixtures are explicit test inputs). It never reaches the
            # wire — the T1 adapter computes on the plain service.
            measured_dimensions_wdh = tuple(float(v) for v in box_wdh)
            conservative = _Conservative()

        # Settle: the planner's own subscriptions (surface/joints/
        # validity) deliver during these spins — plan() reads their
        # latest state synchronously afterwards.
        import rclpy
        # Discovery needs real wall time on a fresh participant; 4 s of
        # spinning lets the latched+live deliveries land.
        import time as _t
        deadline = _t.monotonic() + 6.0
        while _t.monotonic() < deadline:
            rclpy.spin_once(client, timeout_sec=0.03)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if (self.planner._joints is not None
                    and self.planner._surface is not None
                    and self.planner._geometry is not None
                    # the sim clock must have ACTUALLY ticked (an
                    # un-ticked sim clock reads 0 and every joint looks
                    # 'from a different clock')
                    and self.node.get_clock().now().nanoseconds > 0):
                break
        # Post-settle fresh sample: the latched validity is PROCESSED
        # during the settle (its arrival stamp is settle-time), so the
        # robot sample must post-date THAT. Drain the queue FIRST, then
        # stamp-publish-deliver within one age window (max_state_age is
        # 1.0 s — a long post-publish spin would age the sample out).
        for _ in range(30):
            rclpy.spin_once(client, timeout_sec=0.02)
            rclpy.spin_once(self.node, timeout_sec=0.02)
        # Freeze the sim stream for the wall-clock sample below (its
        # ticks would age the sample or sit 'in the future').
        client.stop_robot_streams()
        from sensor_msgs.msg import JointState as _JointState
        # The robot freshness window is 500 ms and the sample must still
        # be inside it when plan() reads it: publish, deliver, and verify
        # the age headroom; re-stamp and retry if the window was eaten by
        # delivery jitter (run02 V1-O K03/E4 lost exactly this race).
        fresh = _JointState()
        fresh.name = ["elfin_joint%d" % i for i in range(1, 7)]
        fresh.position = [0.0] * 6
        for _attempt in range(4):
            node_now = self.node.get_clock().now().nanoseconds
            fresh.header.stamp.sec = int(
                (node_now - 30_000_000) // 10**9)
            fresh.header.stamp.nanosec = int(
                (node_now - 30_000_000) % 10**9)
            client._joint_pub.publish(fresh)
            for _ in range(6):
                rclpy.spin_once(client, timeout_sec=0.01)
                rclpy.spin_once(self.node, timeout_sec=0.01)
            age = self.node.get_clock().now().nanoseconds - (
                fresh.header.stamp.sec * 10**9
                + fresh.header.stamp.nanosec)
            if age < 200_000_000:  # >=300 ms headroom at plan entry
                break
        attachment = MeasuredAttachment(
            box_id="t1-e4-box", generation=1,
            scene_revision=scene_revision, stamp_ns=now,
            box_wdh=tuple(float(v) for v in box_wdh),
            box_from_tcp=Transform([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
            geometry=_FixtureGeometry())
        geometry = dict(client.surface_map or {})
        try:
            result = self.planner.plan(
                attachment, request_id="t1-e4-%d" % now,
                prior=None, selected_choice=None)
        except ValueError as error:
            text = str(error)
            if "validity gate closed" in text:
                return {"outcome": "REFUSED_GATE", "detail": text[:80]}
            if "commit rejected" in text:
                return {"outcome": "COMMIT_REJECTED", "detail": text[:80]}
            if "unavailable" in text or "stale" in text:
                joint = self.planner._joints
                stamp = ((int(joint.header.stamp.sec) * 10**9
                          + int(joint.header.stamp.nanosec))
                         if joint is not None else None)
                return {"outcome": "REFUSED_INPUT",
                        "detail": "%s | joints=%s surface=%s geom=%s "
                                  "stamp=%d clock=%d"
                        % (text[:40],
                           joint is not None,
                           self.planner._surface is not None,
                           self.planner._geometry is not None,
                           stamp or 0,
                           self.node.get_clock().now().nanoseconds)}
            if "mismatch" in text:
                return {"outcome": "REFUSED_IDENTITY", "detail": text[:80]}
            return {"outcome": "REFUSED_OTHER", "detail": text[:80]}
        if isinstance(result, dict) and result.get("boundary"):
            return {"outcome": "BOUNDARY_REACHED",
                    "detail": "entry+compute+recheck+commit passed"}
        return {"outcome": "UNEXPECTED", "detail": str(result)[:80]}
