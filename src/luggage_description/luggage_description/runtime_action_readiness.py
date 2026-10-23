"""Action dependency scopes; wire identity and lease rules stay unchanged.

Both gates observe from process startup. An unrelated peer's revoked lease
cannot poison unloaded motion, nor can creating a scope erase a relevant gap.
"""
from .frame_contracts import ContractError
from .runtime_profile import reject
from .runtime_readiness import ReadinessGate

UNLOADED_MOTION = "unloaded_motion"


def certified_motion_scope(admission):
    """Only a complete unloaded certificate may request the motion scope.

    This selects dependencies, not authorization: the server must still verify
    certificate identity, exact leg, phase, live vacuum, map and scene.
    """
    if (admission and admission.get("load_state") == "unloaded"
            and all(admission.get(key) for key in (
                "route_digest", "certification_id", "leg_digest"))):
        return UNLOADED_MOTION
    return None


class ActionReadiness:
    def __init__(self, full_gate, registry, local_peer):
        self.full = full_gate
        required = set(full_gate.peers) - {"placement_planner", "waypoint_generator"}
        endpoints = [name for name, entry in registry.items()
                     if (set(entry["provider"] if isinstance(entry["provider"], list)
                             else [entry["provider"]])
                         <= (required | {local_peer}))]
        self.unloaded = ReadinessGate(
            full_gate.identity, sorted(required), full_gate.install_root,
            lease_seconds=full_gate.lease_seconds, clock=full_gate.clock,
            endpoints=endpoints, manifest=full_gate.manifest)

    def observe(self, report):
        if not isinstance(report, dict) or report.get("peer") not in self.full.peers:
            self.full.observe(report)  # preserve invalid/unknown-peer diagnostics
            return
        first_error = None
        for gate in (self.full, self.unloaded):
            if report.get("peer") not in gate.peers:
                continue
            try:
                gate.observe(report)
            except ContractError as error:
                first_error = first_error or error
        if first_error is not None:
            raise first_error

    def gate(self, scope=None):
        if scope is None:
            return self.full
        if scope == UNLOADED_MOTION:
            return self.unloaded
        reject("UNKNOWN_ACTION_SCOPE", "scope", UNLOADED_MOTION, scope)
