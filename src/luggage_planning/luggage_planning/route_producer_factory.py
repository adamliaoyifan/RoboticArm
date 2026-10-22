"""Production RouteProducer construction for SelectPlacePathV2.

``trajectory_selector_node`` and the second-box witness replay both call
``build_production_route_producer``. The node path reads its latched
surface, cargo-map stats service, atlas, and parameters. A replay passes
the same callables with captured values. Collaborator types, contract
policies, and the certification budget stay the production ones.
"""

from __future__ import division

import os


def build_production_route_producer(node, scene_tf_path, surface_provider,
                                    stats_provider, *, scene=None,
                                    scene_tf_hash=None, atlas_cfg=None,
                                    urdf_identity=None, runtime_atlas=None,
                                    scene_revision=None, box_count=None,
                                    corridor_planner=None,
                                    vacuum_state_fn=None, site_poses_fn=None,
                                    clock_ns=None, log=None,
                                    static_identity=None, work_counters=None,
                                    route_budget_override_s=None):
    """Build one production route producer, or None when the contract is missing.

    ``surface_provider()`` returns the latched surface dict or empty.
    ``stats_provider()`` returns the cargo-map stats dict or None. A None
    stats result fails revalidation closed, the same as a missing
    ``/cargo_map/get_stats`` response.
    """
    from luggage_description.scene_tf_config_utils import (
        container_opening_normal_in_world,
        container_opening_target_point_in_world,
        pedestal_dimensions,
        pedestal_enabled,
    )
    from luggage_planning.attachment_provider import MeasuredAttachmentProvider
    from luggage_planning.atlas_prior import prior_for_targets
    from luggage_planning.grid_path_search import route_budget_ns
    from luggage_planning.lattice_route import LatticeStateChecker
    from luggage_planning.map_revalidation import MapRevalidator
    from luggage_planning.moveit_edge_adapter import MoveItEdgeAdapter
    from luggage_planning.moveit_state_validity import MoveItStateValidityAdapter
    from luggage_planning.offline_ik import load_urdf, refine_to_pose
    from luggage_planning.planning_scene_client import PlanningSceneClient
    from luggage_planning.reachable_graph import RouteGraph, make_offline_fk
    from luggage_planning.route_producer import (
        RouteProducer,
        certification_policy_from_contract,
        revalidation_policy_from_contract,
    )
    from luggage_planning.trajectory_certifier import TrajectoryCertifier
    import json as _json

    logger = log or (lambda message: node.get_logger().info(message))
    try:
        from ament_index_python.packages import get_package_share_directory

        contract_path = os.path.join(
            get_package_share_directory("st09_contracts"),
            "resolved_planning_contract.json")
        with open(contract_path, encoding="utf-8") as handle:
            contract_document = _json.load(handle)
        cert_policy = certification_policy_from_contract(contract_document)
        revalidation_policy = revalidation_policy_from_contract(
            contract_document)
    except Exception:  # noqa: BLE001 - fail closed to C defaults
        node.get_logger().error(
            "route: resolved contract unavailable; refusing to plan")
        return None

    if scene is None:
        scene = node._scene
    if scene_tf_hash is None:
        scene_tf_hash = node._scene_tf_hash
    if atlas_cfg is None:
        atlas_cfg = node._atlas_cfg
    if urdf_identity is None:
        urdf_identity = node._urdf_identity
    if runtime_atlas is None:
        runtime_atlas = node._runtime_atlas
    if scene_revision is None:
        scene_revision = str(
            node.get_parameter("frame_scene_revision").value or "")
    if box_count is None:
        box_count = int(node.get_parameter("box_count").value or 1)
    if corridor_planner is None:
        corridor_planner = str(
            node.get_parameter("corridor_planner").value or "graph")
    if vacuum_state_fn is None:
        def vacuum_state_fn():  # noqa: WPS430
            state = getattr(node, "_vacuum_state", None)
            return dict(state) if state is not None else None
    if site_poses_fn is None:
        site_poses_fn = node._site_poses
    if clock_ns is None:
        clock_ns = lambda: node.get_clock().now().nanoseconds  # noqa: E731

    urdf = load_urdf(scene_tf_config=scene_tf_path)
    # path_to returns the JOINT sequence to the TCP link (tcp() FK
    # walks joints; the link name itself is not a joint).
    tcp_path = urdf.path_to("suction_contact_frame")
    fk = make_offline_fk(urdf, tcp_path)
    scene_client = PlanningSceneClient(node)
    holder = []

    def latest_snapshot():
        """Fresh mapper state at admission time — never the planned snapshot.

        Stats supply map_source, placed_count, whether a cargo view was
        integrated, and the acquisition stamp. Stamp 0 is legal only for
        static_empty with no placed boxes and no observation. Any
        unavailable piece fails the revalidation closed.
        """
        from luggage_perception.cargo_view_integration import (
            planning_snapshot_from_surface,
        )

        surface = dict(surface_provider() or {})
        if not surface:
            return None
        stats = stats_provider()
        if not stats or not stats.get("success"):
            return None
        surface.update({
            "acquisition_stamp_ns": int(stats["last_acquisition_stamp_ns"]),
            "map_source": str(stats.get("map_source") or ""),
            "placed_count": int(stats.get("placed_count") or 0),
            "cargo_observed": bool(stats.get("cargo_observed")),
            "map_revision": int(stats["map_revision"]),
            "commit_revision": int(stats["map_revision"]),
            "geometry_hash": str(stats["geometry_hash"]),
            # [B1] Snapshot identity: fresh stats win (same-commit), the
            # latched surface's is the fallback for pre-B1 stat shapes.
            "epoch": str(stats.get("epoch") or surface.get("epoch") or ""),
            "content_digest": str(
                stats.get("content_digest")
                or surface.get("content_digest") or ""),
        })
        return planning_snapshot_from_surface(
            surface, scene_revision=str(scene_revision or ""),
            request_id="latest-snapshot")

    def refine(position, quaternion, seed):
        operation = refine_to_pose
        if work_counters is not None:
            operation = lambda *args, **kwargs: work_counters.refine(refine_to_pose, *args, **kwargs)
        return operation(
            urdf, tcp_path, position, quaternion, seed,
            pos_tol=2e-3, rot_tol=2e-2)

    def atlas_prior(targets, request):
        """Reachability prior at the synthesized TCP, not the box pose.

        A candidate is unreachable only when every allowed TCP target is
        sampled-unreachable. A miss stays unknown. Seeds are re-refined
        onto the exact target. MoveIt and certification stay the authority
        for every leg that runs.
        """
        atlas, diag = runtime_atlas()
        producer = holder[0] if holder else None
        if producer is not None:
            diag = diag or {}
            producer.last_atlas_diag = {
                "loaded": bool(diag.get("loaded")),
                "reason": diag.get("reason"),
                "hash_message": diag.get("hash_message"),
                "scene_tf_ok": diag.get("scene_tf_ok"),
                "urdf_ok": diag.get("urdf_ok"),
                "scene_tf_hash": scene_tf_hash,
                "urdf_hash": (
                    urdf_identity.hash if urdf_identity.ready else ""),
                "live_urdf_hash": diag.get("live_urdf_hash") or "",
                "urdf_hash_override": diag.get("urdf_hash_override") or "",
                "query_offset": [
                    float(v) for v in (atlas_cfg.get("query_offset") or [])],
            }
        if atlas is None or not targets:
            return None
        try:
            prior = prior_for_targets(
                targets, request["container_transform"], atlas.query)
        except Exception:  # noqa: BLE001 - prior fails open
            return {"verdict": "unknown", "seeds": []}
        return prior

    def validity_factory(*, scene_states, edges):
        adapter = MoveItStateValidityAdapter(
            node, fk=fk, scene_client=scene_client,
            log=logger)
        adapter.load_edges(edges, scene_states=scene_states)
        return adapter

    def lattice_checker_factory(*, scene_states, start_q, service_budget_s=5.0):
        return LatticeStateChecker(
            node, scene_client, scene_states, start_q, service_budget_s=service_budget_s)

    # [Functional-diagnostic override, 2026-10-22] Phase-1 bounded
    # relaxation ONLY: an explicit per-Select wall budget that reaches the
    # searcher (RouteProducer.budget_ns), not a client-side wait. The
    # frozen n*60s contract stays the default (override <= 0); per-edge
    # budgets, retries, and every safety gate are untouched. Non-scoring
    # diagnostics only — a pass under this budget is never reported as a
    # frozen-budget pass.
    if route_budget_override_s is None:
        route_budget_override_s = float(
            node.get_parameter("diagnostic_route_budget_s").value or 0.0)
    try:
        override_s = float(route_budget_override_s or 0.0)
    except (TypeError, ValueError):
        override_s = 0.0
    if override_s > 0.0:
        budget_ns = int(override_s * 1e9)
        logger("route budget %.0fs for %d box(es) "
               "(DIAGNOSTIC override of frozen %.0fs)" % (
                   budget_ns / 1e9, box_count, route_budget_ns(int(box_count)) / 1e9))
    else:
        budget_ns = route_budget_ns(int(box_count))
        logger("route budget %.0fs for %d box(es)" % (budget_ns / 1e9, box_count))
    from luggage_planning.payload_envelope_config import load_envelope_limits
    envelope_path = str(node.get_parameter("payload_envelope_config").value or "")
    if not envelope_path:
        envelope_path = os.path.join(get_package_share_directory("luggage_planning"),
                                     "config", "payload_envelope.yaml")
    envelope_limits = load_envelope_limits(envelope_path)
    logger("payload envelope diagnostic limits: %s" % envelope_limits)
    skip_slot_envelope = bool(node.get_parameter("diagnostic_skip_slot_envelope_check").value)
    logger("diagnostic_skip_slot_envelope_check=%s; trajectory collision checks retained" % skip_slot_envelope)
    producer = RouteProducer(
        diagnostic_skip_slot_envelope_check=skip_slot_envelope,
        graph=RouteGraph(static_identity=static_identity or (
            "st09-i-route:%s" % scene_tf_hash)),
        edge_builder=MoveItEdgeAdapter(node, scene_client=scene_client),
        certifier=TrajectoryCertifier(),
        cert_policy=cert_policy,
        validity_adapter_factory=validity_factory,
        attachment_provider=MeasuredAttachmentProvider(
            str(scene_revision or "scene-runtime"), "route-selector",
            tcp_frame="suction_contact_frame", box_frame="box",
            max_age_ns=budget_ns, envelope_limits=envelope_limits),
        budget_ns=budget_ns,
        revalidator=MapRevalidator(revalidation_policy),
        latest_snapshot_fn=latest_snapshot,
        fk=fk, refine=refine, urdf=urdf, tcp_path=tcp_path,
        clock_ns=clock_ns,
        log=logger,
        opening_info={
            "point": container_opening_target_point_in_world(scene),
            "normal": container_opening_normal_in_world(scene),
        },
        transit_clearance_z=(
            pedestal_dimensions(scene)[2]
            if pedestal_enabled(scene) else None),
        atlas_prior=atlas_prior,
        vacuum_state_fn=vacuum_state_fn,
        site_poses_fn=site_poses_fn,
        scene_config=scene,
        scene_revision_binding=str(scene_revision or ""),
        corridor_planner=str(corridor_planner or "graph"),
        lattice_checker_factory=lattice_checker_factory)
    holder.append(producer)
    return producer
