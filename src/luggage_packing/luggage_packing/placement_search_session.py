"""Placement search session with route-failure continuation (G3 §5).

The orchestrator owns route attempts; this session owns candidate
supply. Contract (user item 2, 2026-10-25): when a batch's positions
are all LEGAL but every route attempt fails, the caller asks
``next_batch()`` and receives more candidates -- remaining structural
positions first, then the lazy grid fallback -- until the grid is
exhausted under the session budget. The session NEVER concludes
"container full" while unseen candidates remain; exhaustion and budget
cuts are reported separately (``coverage_complete`` vs
``budget_exhausted``), and coverage completeness is claimed only after
a full grid traversal.

Deterministic, single active session per request, no cross-request
state. Inputs are bound at ``start``; the session is read-only on the
map and payload afterwards.
"""

import time

from luggage_packing.placement_solver import (
    _default_params,
    _depth_datum,
    _surface_hull,
)
from luggage_packing.placement_structural_candidates import (
    DEFAULT_STRUCTURAL_POLICY,
    evaluate_window,
    fallback_batch,
    generate_proposals,
)
from luggage_packing.placement_snug_ranking import snug_rank
from luggage_packing.placement_support_index import (
    SupportIndex, structural_neighbor_records)


class SearchSessionExpired(Exception):
    """Raised when a caller advances a session whose inputs changed or
    whose deadline passed. The caller must re-plan from a new request,
    never silently continue this session."""


class PlacementSearchSession:
    """Batches of ranked candidates for one placement request."""

    def __init__(self, surface_map, payload_size, allowed_yaws=None,
                 params=None, structural_policy=None, payload=None,
                 budget_s=20.0, batch_size=64, candidate_validator=None):
        self._map = surface_map
        cfg = _default_params()
        if params:
            cfg.update(params)
        cfg["top_n"] = 10 ** 9
        cfg["keep_rejected"] = 10 ** 9
        self._cfg = cfg
        self._policy = dict(DEFAULT_STRUCTURAL_POLICY)
        if structural_policy:
            self._policy.update(structural_policy)
        self._deadline = time.monotonic() + float(budget_s)
        self._batch_size = int(batch_size)
        self._payload = payload
        t0 = time.perf_counter()
        self._index = SupportIndex(surface_map, payload_size,
                                   allowed_yaws or [0.0], cfg)
        self._hull = _surface_hull(surface_map)
        self._datum = _depth_datum(surface_map, self._hull)
        proposals, report = generate_proposals(
            self._index, surface_map, structural_policy)
        candidates = []
        for cls, ix0, iy0, _prov in proposals:
            candidates.extend(evaluate_window(
                self._index, cls, ix0, iy0, cfg, self._hull, self._datum,
                candidate_validator=candidate_validator))
        self._structural_pool = [c for c in candidates if c["feasible"]]
        self._neighbors = structural_neighbor_records(surface_map)
        self._report = report
        self._setup_s = time.perf_counter() - t0
        self._served = []          # ranked ids already returned
        self._seen_ids = {c["candidate_id"] for c in candidates}
        self._cursor = None
        self._coverage_complete = False
        self._batches_served = 0
        self._candidate_validator = candidate_validator

    # -- identity ------------------------------------------------------

    def identity(self):
        index_id = self._index.identity
        return {
            "map_geometry_hash": index_id["map_geometry_hash"],
            "policy_digest": index_id["policy_digest"],
            "attachment_digest": (self._payload.attachment_digest
                                  if self._payload is not None else ""),
        }

    # -- batches -------------------------------------------------------

    def _remaining_time(self):
        return self._deadline - time.monotonic()

    def start(self):
        """First batch: the ranked structural head."""
        return self.next_batch()

    def next_batch(self):
        """The next batch of unseen ranked candidates.

        Structural remainder first, then lazy grid fallback in the
        solver's own order. Raises ``SearchSessionExpired`` past the
        deadline. Terminal states are reported, never guessed: an empty
        batch with ``coverage_complete`` means the grid is exhausted;
        with ``budget_exhausted`` it means the budget ran out with
        unseen positions remaining -- which is NOT "container full".
        """
        if self._remaining_time() <= 0.0:
            raise SearchSessionExpired(
                "SEARCH_BUDGET_EXHAUSTED: session deadline passed with "
                "%d served candidates" % len(self._served))
        if self._structural_pool:
            ranked = snug_rank(
                self._structural_pool, self._neighbors, self._policy,
                payload=self._payload)
            take = []
            for cand in ranked:
                if cand["candidate_id"] in self._served:
                    continue
                take.append(cand)
                if len(take) >= self._batch_size:
                    break
            self._structural_pool = [
                c for c in self._structural_pool
                if c["candidate_id"] not in {t["candidate_id"]
                                             for t in take}]
            if take:
                self._served.extend(c["candidate_id"] for c in take)
                self._batches_served += 1
                return self._wrap(take, source="structural")
        # Structural exhausted -> lazy grid fallback.
        batch = fallback_batch(
            self._index, self._cfg, self._hull, self._datum,
            self._seen_ids, cursor=self._cursor,
            batch_size=self._batch_size,
            budget_s=max(0.0, self._remaining_time()),
            candidate_validator=self._candidate_validator)
        self._cursor = batch["cursor"]
        self._coverage_complete = batch["coverage_complete"]
        take = [c for c in batch["candidates"] if c["feasible"]]
        self._served.extend(c["candidate_id"] for c in take)
        self._batches_served += 1
        return self._wrap(take, source="grid_fallback",
                          budget_exhausted=batch["budget_exhausted"])

    def _wrap(self, candidates, source, budget_exhausted=False):
        return {
            "candidates": candidates,
            "source": source,
            "batch_index": self._batches_served - 1,
            "served_total": len(self._served),
            "coverage_complete": (self._coverage_complete
                                  and not self._structural_pool),
            "budget_exhausted": budget_exhausted,
            "remaining_budget_s": round(self._remaining_time(), 3),
            "terminal_reason": (
                "grid_exhausted" if (self._coverage_complete
                                     and not self._structural_pool
                                     and not candidates)
                else ("budget_exhausted" if budget_exhausted
                      else ("batch_served" if candidates
                            else "grid_exhausted"))),
            "proposals": self._report,
            "setup_s": round(self._setup_s, 4),
        }

    def close(self):
        self._structural_pool = []
        self._cursor = None
