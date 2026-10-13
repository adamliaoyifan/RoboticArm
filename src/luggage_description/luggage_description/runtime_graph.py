"""Ownership table of the v2 runtime interface graph (ROS-free).

Every endpoint of the typed chain, who provides it and which peers may call
it. All participating nodes derive the same endpoint registry from the same
peer set, so a heartbeat that disagrees with the local view of the graph is
rejected at decode time (ENDPOINT_PROVIDER_MISMATCH) instead of trusting a
self-declared topology. Peer names are ROS node names.
"""
from .runtime_profile import reject

#: endpoint -> (type, provider, clients). The runtime identity topic itself
#: is appended per peer set by :func:`endpoint_registry`.
OWNED_ENDPOINTS = {
    "/placement_planner/compute_placement_v2": (
        "luggage_msgs/srv/ComputePlacementV2", "placement_planner", ["driver"]),
    "/waypoint_generator/build_motion_sequence_v2": (
        "luggage_msgs/srv/BuildMotionSequenceV2", "waypoint_generator", ["driver"]),
    "/trajectory_selector/select_place_path_v2": (
        "luggage_msgs/srv/SelectPlacePathV2", "trajectory_selector", ["driver"]),
    "/motion_planner/probe_motion_segment_v2": (
        "luggage_msgs/srv/ProbeMotionSegmentV2", "motion_planner", ["driver"]),
    "/motion_planner/plan_motion_v2": (
        "luggage_msgs/action/PlanMotionV2", "motion_planner", ["driver"]),
    "/vacuum/command_v2": (
        "luggage_msgs/srv/VacuumCommandV2", "vacuum_controller", ["driver"]),
    "/scene_manager/add_scene_box_v2": (
        "luggage_msgs/srv/AddSceneBoxV2", "scene_manager", ["driver"]),
    "/scene_manager/remove_scene_box_v2": (
        "luggage_msgs/srv/RemoveSceneBoxV2", "scene_manager", ["driver"]),
    "/cargo_map/add_placed_box_v2": (
        "luggage_msgs/srv/AddPlacedBoxV2", "cargo_volume_mapper", ["driver"]),
    "/cargo_map/remove_placed_box_v2": (
        "luggage_msgs/srv/RemovePlacedBoxV2", "cargo_volume_mapper", ["driver"]),
    "/luggage/cargo_map/surface_v2": (
        "luggage_msgs/msg/SurfaceContractV2", "cargo_volume_mapper",
        ["placement_planner", "driver"]),
}
RUNTIME_IDENTITY_TOPIC = "/frame_contracts/v2/runtime_identity"
RUNTIME_IDENTITY_TYPE = "luggage_msgs/msg/RuntimeIdentityV2"

#: Core stack peers started together whenever the v2 chain is enabled.
#: The orchestrating driver peer joins through the launch setting
#: ``frame_driver_peer`` (default the motion_occ driver).
STACK_PEERS = (
    "placement_planner", "waypoint_generator", "trajectory_selector",
    "motion_planner", "vacuum_controller", "scene_manager", "cargo_volume_mapper",
)
#: Hardware-plan-only cell: pick chain without the placement/map surface
#: (no cargo mapper, placement planner or trajectory selector). The
#: endpoint registry drops their endpoints with the peers.
HARDWARE_STACK_PEERS = (
    "waypoint_generator", "motion_planner", "vacuum_controller", "scene_manager",
)
DEFAULT_DRIVER_PEER = "motion_occ_driver"


def resolve_peers(driver_peer=DEFAULT_DRIVER_PEER, stack=STACK_PEERS):
    peers = [str(name) for name in stack]
    driver = str(driver_peer or "").strip()
    if not driver:
        reject("INVALID_PEER_SET", "frame_driver_peer", "driver node name", driver_peer)
    if driver in peers:
        reject("INVALID_PEER_SET", "frame_driver_peer",
               "non-stack driver node", driver_peer)
    peers.append(driver)
    return peers


def endpoint_registry(peers):
    """Registry for exactly this peer set: {endpoint: {type, provider, clients}}.

    ``driver`` entries in OWNED_ENDPOINTS are rewritten to the session's
    driver peer; endpoints owned or consumed by absent peers are not part of
    the graph. The runtime identity topic is provided and consumed by every
    peer. Every node in the session must derive an identical registry.
    """
    peers = list(peers)
    if not peers or len(set(peers)) != len(peers):
        reject("INVALID_PEER_SET", "peers", "unique peer set", peers)
    known = set(STACK_PEERS) | set(HARDWARE_STACK_PEERS)
    drivers = [peer for peer in peers if peer not in known]
    if len(drivers) > 1:
        reject("INVALID_PEER_SET", "driver", "one driver peer", drivers)
    driver = drivers[0] if drivers else None
    registry = {}
    for name, (type_name, provider, clients) in OWNED_ENDPOINTS.items():
        resolved_clients = []
        for client in clients:
            if client == "driver":
                if driver is None:
                    continue
                resolved_clients.append(driver)
            elif client in peers:
                resolved_clients.append(client)
        if provider not in peers or not resolved_clients:
            continue
        registry[name] = {"type": type_name, "provider": provider,
                          "clients": resolved_clients}
    if not registry:
        reject("INVALID_PEER_SET", "peers", "v2 graph participants", peers)
    registry[RUNTIME_IDENTITY_TOPIC] = {
        "type": RUNTIME_IDENTITY_TYPE, "provider": list(peers),
        "clients": list(peers)}
    return registry


def identity_endpoints(registry):
    """The identity endpoint map (name -> type) for a registry."""
    return {name: entry["type"] for name, entry in registry.items()}


def live_providers(registry, observed_peers):
    """Provider-type list per endpoint, restricted to observed live peers.

    This is the ``providers`` argument of ``ReadinessGate.reason``: an
    endpoint counts only when its owning peer has a current lease, a second
    live owner is a duplicate provider, and an endpoint with no live owner is
    missing.
    """
    providers = {}
    for name, entry in registry.items():
        if name == RUNTIME_IDENTITY_TOPIC:
            # The heartbeat bus is every-peer; readiness counts it as one
            # live provider (an owned endpoint has exactly one owner).
            owners = ([RUNTIME_IDENTITY_TYPE]
                      if any(peer in observed_peers
                             for peer in entry["provider"]) else [])
            providers[name] = owners
            continue
        owners = ([entry["type"]]
                  if entry["provider"] in observed_peers else [])
        providers[name] = owners
    return providers
