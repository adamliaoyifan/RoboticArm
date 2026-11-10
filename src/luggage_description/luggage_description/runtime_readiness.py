"""Consumer-owned readiness: current peer leases and immutable process identity.

Call require_ready immediately before side effects. A heartbeat only refreshes
an unchanged incarnation. A restart latches the gate closed until explicit
re-admission; it must never silently resume a carrying sequence.
"""
from copy import deepcopy
import time
import math
from pathlib import Path

from .runtime_profile import IDENTITY_FIELDS, canonical_hash, reject, validate_identity
from .runtime_manifest import identity_fingerprint, validate_artifact_locations, validate_manifest


class ReadinessGate:
    def __init__(self, expected_identity, peers, install_root, lease_seconds=2.0,
                 clock=time.monotonic, manifest=None, endpoints=None, renewable_peers=()):
        self.identity = validate_identity(expected_identity)
        self.manifest = validate_manifest(manifest) if manifest is not None else None
        if self.manifest and self.manifest["identity"] != self.identity:
            reject("PEER_IDENTITY_MISMATCH", "manifest.identity", self.identity, self.manifest["identity"])
        self.endpoints = tuple(self.identity["endpoints"] if endpoints is None else endpoints)
        if not self.endpoints or not set(self.endpoints).issubset(self.identity["endpoints"]):
            reject("INVALID_ENDPOINT_SCOPE", "endpoints", sorted(self.identity["endpoints"]), self.endpoints)
        self.generation = 1
        if (not isinstance(peers, (list, tuple)) or not peers
                or any(not isinstance(peer, str) or not peer.strip() for peer in peers)
                or len(peers) != len(set(peers))):
            reject("INVALID_PEER_SET", "peers", "unique nonempty peer set", peers)
        if (type(lease_seconds) not in (int, float) or not math.isfinite(lease_seconds)
                or lease_seconds <= 0):
            reject("INVALID_LEASE", "lease_seconds", "positive", lease_seconds)
        self.peers = tuple(peers)
        self.renewable_peers = frozenset(renewable_peers)
        if not self.renewable_peers.issubset(self.peers):
            reject("INVALID_PEER_SET", "renewable_peers", self.peers, renewable_peers)
        self.lease_epochs = {peer: 0 for peer in self.peers}
        self.install_root = Path(install_root).resolve()
        if self.manifest and Path(self.manifest["install_root"]).resolve() != self.install_root:
            reject("WRONG_OVERLAY", "install_root", self.manifest["install_root"], str(self.install_root))
        if self.manifest and not set(peers).issubset(self.manifest["processes"]):
            reject("INVALID_PEER_SET", "peers", sorted(self.manifest["processes"]), peers)
        self.lease_seconds = lease_seconds
        self.clock = clock
        self.reports = {}
        self.revoked = None
        self.transitions = []
        self._state = None

    def _transition(self, reason):
        if reason != self._state:
            self.transitions.append({"at_monotonic": self.clock(), "reason": reason})
            self._state = reason
        return reason

    def observe(self, report):
        if not isinstance(report, dict):
            self.revoked = "INVALID_PROCESS_REPORT"
            reject(self.revoked, "report", "mapping", type(report).__name__)
        peer = report.get("peer")
        if peer not in self.peers:
            reject("UNKNOWN_PEER", "peer", self.peers, peer)
        try:
            identity = validate_identity(report.get("identity"))
            for key in ("contract_version", *IDENTITY_FIELDS):
                if identity[key] != self.identity[key]:
                    code = "STALE_ATLAS" if key.startswith("atlas_") else "PEER_IDENTITY_MISMATCH"
                    reject(code, key, self.identity[key], identity[key])
            for key in ("incarnation", "resolved_config", "artifacts", "package_prefixes"):
                if not report.get(key):
                    reject("MISSING_PROCESS_PROVENANCE", key, "nonempty", report.get(key))
            for key, expected_type in (("incarnation", str), ("resolved_config", dict),
                                       ("artifacts", list), ("package_prefixes", dict)):
                if not isinstance(report[key], expected_type):
                    reject("INVALID_PROCESS_REPORT", key, expected_type.__name__, type(report[key]).__name__)
            validate_artifact_locations(
                report, self.install_root,
                self.manifest["system_root"] if self.manifest else None)
            if self.manifest:
                if report.get("contract_digest") != canonical_hash(self.identity):
                    reject("PEER_IDENTITY_MISMATCH", "contract_digest", canonical_hash(self.identity), report.get("contract_digest"))
                expected = self.manifest["processes"][peer]
                for field in ("resolved_config", "artifacts", "package_prefixes"):
                    if report[field] != expected[field]:
                        reject("STALE_PROCESS_ARTIFACT" if field == "artifacts" else "PEER_IDENTITY_MISMATCH",
                               field, expected[field], report[field])
            prior = self.reports.get(peer)
            # Detect expiration before updating received: a late heartbeat must
            # not conceal a lease gap between request acceptance and effects.
            if prior and self.clock() - prior["received"] > self.lease_seconds:
                if peer in self.renewable_peers:
                    # Idle planning service can renew for a NEW request. An
                    # already-issued token remembers the prior lease epoch.
                    self.lease_epochs[peer] += 1
                else:
                    reject("PEER_LEASE_EXPIRED", peer, "unbroken lease", self.clock() - prior["received"])
            fingerprint = identity_fingerprint(report)
            if prior and prior["fingerprint"] != fingerprint:
                code = ("PEER_RESTART" if prior["report"]["incarnation"] != report["incarnation"]
                        else "PEER_IDENTITY_CHANGED")
                reject(code, peer, prior["fingerprint"], fingerprint)
            self.reports[peer] = {"report": deepcopy(report), "fingerprint": fingerprint,
                                  "received": self.clock()}
        except Exception as error:
            self.revoked = getattr(error, "code", "INVALID_PROCESS_REPORT")
            self._transition(self.revoked)
            raise

    def reason(self, providers=None):
        if self.revoked:
            return self._transition(self.revoked)
        now = self.clock()
        for peer in self.peers:
            if peer not in self.reports:
                return self._transition("MISSING_PEER")
            if now - self.reports[peer]["received"] > self.lease_seconds:
                # An expired admitted peer cannot re-enable an interrupted request.
                if peer not in self.renewable_peers:
                    self.revoked = "PEER_LEASE_EXPIRED"
                return self._transition("PEER_LEASE_EXPIRED")
        if providers is None:
            return self._transition("MISSING_ENDPOINT_GRAPH")
        for endpoint in self.endpoints:
            expected_type = self.identity["endpoints"][endpoint]
            actual = providers.get(endpoint, [])
            if self.manifest:
                registered = self.manifest["endpoints"][endpoint]
                owners = registered["provider"]
                owners = [owners] if isinstance(owners, str) else owners
                seen = []
                for provider in actual:
                    if not isinstance(provider, dict) or provider.get("peer") not in owners:
                        self.revoked = "ENDPOINT_PROVIDER_MISMATCH"
                        return self._transition(self.revoked)
                    if provider["peer"] in seen:
                        self.revoked = "DUPLICATE_PROVIDER"
                        return self._transition(self.revoked)
                    seen.append(provider["peer"])
                    if provider.get("type") != expected_type:
                        self.revoked = "ENDPOINT_TYPE_MISMATCH"
                        return self._transition(self.revoked)
                if set(seen) != set(owners):
                    return self._transition("MISSING_ENDPOINT")
            else:
                if len(actual) > 1:
                    self.revoked = "DUPLICATE_PROVIDER"
                    return self._transition(self.revoked)
                if not actual:
                    return self._transition("MISSING_ENDPOINT")
                if actual[0] != expected_type:
                    self.revoked = "ENDPOINT_TYPE_MISMATCH"
                    return self._transition(self.revoked)
        return self._transition("READY")

    def require_ready(self, providers, wire_version=2):
        if wire_version != 2:
            reject("LEGACY_ENDPOINT_DISABLED", "wire_version", 2, wire_version)
        reason = self.reason(providers)
        if reason != "READY":
            reject(reason, "readiness", "READY", reason)

    def readmit(self):
        """Explicit recovery invalidates every token and requires fresh reports.

        This resets only readiness. Operator authorization and carry recovery
        belong to the caller and are never granted by this operation.
        """
        self.generation += 1
        self.reports.clear()
        self.revoked = None
        self._transition("MISSING_PEER")
        return self.generation

    def admit(self, providers, request_id, *, endpoint=None, client=None):
        if not isinstance(request_id, str) or not request_id.strip():
            reject("MISSING_IDENTITY", "request_id", "nonempty request", request_id)
        self.require_ready(providers)
        if self.manifest:
            registered = self.manifest["endpoints"].get(endpoint)
            if not registered or client not in registered["clients"]:
                reject("ENDPOINT_CLIENT_MISMATCH", "client", endpoint, client)
        return {"generation": self.generation, "request_id": request_id,
                "endpoint": endpoint, "client": client,
                "lease_epochs": dict(self.lease_epochs),
                "peers": {peer: value["fingerprint"] for peer, value in self.reports.items()}}

    def require_admission(self, admission, providers, request_id):
        """Recheck at the effect boundary, after any asynchronous work."""
        self.require_ready(providers)
        if (not isinstance(admission, dict)
                or admission.get("generation") != self.generation
                or admission.get("request_id") != request_id
                or admission.get("lease_epochs") != self.lease_epochs
                or admission.get("peers") != {peer: value["fingerprint"] for peer, value in self.reports.items()}):
            reject("ADMISSION_REVOKED", "admission", "current request and generation", admission)
        if self.manifest:
            registered = self.manifest["endpoints"].get(admission.get("endpoint"))
            if not registered or admission.get("client") not in registered["clients"]:
                reject("ENDPOINT_CLIENT_MISMATCH", "client", "permitted client", admission.get("client"))
