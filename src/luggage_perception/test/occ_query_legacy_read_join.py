"""Offline, response-independent association of legacy GetVolume callbacks.

Legacy refusals have no response identity. Requests with equal premises may
overlap, so a callback cannot be assigned separately to each request. A complete
one-to-one association must exist, and every feasible association must give the
same first snapshot identity for a request before its revision is usable.
"""

from collections import deque


def _callbacks(events):
    active = {}
    completed = []
    for event in events:
        stage = event.get("stage")
        thread = event.get("thread")
        if stage == "_on_volume_begin":
            if thread in active:
                return None
            active[thread] = dict(begin=event["monotonic_ns"] / 1e9, identity=None)
        elif stage == "snapshot_read" and thread in active:
            callback = active[thread]
            if "read_seen" not in callback:
                callback["read_seen"] = True
                identity = event.get("snapshot_identity")
                if isinstance(identity, (list, tuple)) and len(identity) == 3:
                    callback["identity"] = tuple(identity)
        elif stage == "_on_volume_end":
            callback = active.pop(thread, None)
            if callback is None:
                return None
            callback.update(
                ended=event["monotonic_ns"] / 1e9,
                premise=event.get("volume_premise"),
            )
            if callback["ended"] < callback["begin"]:
                return None
            completed.append(callback)
    return None if active else completed


def _perfect_matching(edges, spend, fixed=None):
    """Iterative augmenting paths; a forced edge is never reassigned."""
    by_request = {}
    by_callback = {}
    if fixed is not None:
        by_request[fixed[0]] = fixed[1]
        by_callback[fixed[1]] = fixed[0]
    for request in range(len(edges)):
        if request in by_request:
            continue
        queue = deque([request])
        visited = {request}
        parent = {}
        free = None
        while queue and free is None:
            current = queue.popleft()
            for callback in edges[current]:
                spend()
                if callback in parent or (fixed is not None and callback == fixed[1]):
                    continue
                parent[callback] = current
                owner = by_callback.get(callback)
                if owner is None:
                    free = callback
                    break
                if owner not in visited:
                    visited.add(owner)
                    queue.append(owner)
        if free is None:
            return None
        while free is not None:
            owner = parent[free]
            previous = by_request.get(owner)
            by_request[owner] = free
            by_callback[free] = owner
            free = previous
    return by_request


def resolve_legacy_volume_revisions(requests, events, *, max_requests=2048, max_work=20_000_000):
    """Return name->revision/None and diagnostics, without reading responses.

    Descriptors contain only name, premise and same-host monotonic begin/ended
    times. Missing callbacks, identity ambiguity or work exhaustion remain
    explicitly unjoinable. Callback permutations with the same identity are OK.
    The bounded search does not truncate requests or choose favorable matches.
    """
    unknown = {request["name"]: None for request in requests}
    diagnostics = dict(requests=len(requests), work=0, method="global_bijection_first_snapshot")

    def rejected(reason):
        diagnostics["reason"] = reason
        return unknown, diagnostics

    if len(unknown) != len(requests):
        return rejected("DUPLICATE_REQUEST_NAME")
    if len(requests) > max_requests:
        return rejected("JOIN_WORK_BUDGET")
    callbacks = _callbacks(events)
    diagnostics["callbacks"] = None if callbacks is None else len(callbacks)
    if callbacks is None:
        return rejected("INCOMPLETE_CALLBACK_TRACE")
    if len(callbacks) != len(requests):
        return rejected("CALLBACK_CARDINALITY")

    def spend():
        diagnostics["work"] += 1
        if diagnostics["work"] > max_work:
            raise OverflowError("JOIN_WORK_BUDGET")

    try:
        edges = []
        for request in requests:
            candidates = []
            for index, callback in enumerate(callbacks):
                spend()
                if (
                    request["premise"] == callback["premise"]
                    and request["begin"] <= callback["begin"]
                    and callback["ended"] <= request["ended"]
                ):
                    candidates.append(index)
            edges.append(candidates)
        matching = _perfect_matching(edges, spend)
        if matching is None:
            return rejected("NO_PERFECT_MATCHING")
        resolved = {}
        diagnostics["rows"] = []
        for index, request in enumerate(requests):
            identity = callbacks[matching[index]]["identity"]
            ambiguous = False
            for callback in edges[index]:
                if callbacks[callback]["identity"] == identity:
                    continue
                if _perfect_matching(edges, spend, (index, callback)) is not None:
                    ambiguous = True
                    break
            revision = identity[1] if identity is not None and not ambiguous else None
            resolved[request["name"]] = revision
            diagnostics["rows"].append(dict(
                name=request["name"], revision=revision,
                reason="AMBIGUOUS_IDENTITY" if ambiguous else (
                    "MISSING_FIRST_SNAPSHOT" if identity is None else "UNIQUE_IDENTITY"
                ),
                identity=None if revision is None else list(identity),
                candidate_callbacks=edges[index],
            ))
        diagnostics["reason"] = "COMPLETE_ASSOCIATION"
        return resolved, diagnostics
    except OverflowError:
        return rejected("JOIN_WORK_BUDGET")
