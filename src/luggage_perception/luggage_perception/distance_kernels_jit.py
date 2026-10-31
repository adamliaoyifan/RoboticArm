"""Numba-JIT kernel for the exact distance branch-and-bound (W1-D).

Replicates ``cargo_distance_query._nearest`` bit-for-bit:

- heap pop order: CPython heapq always pops the tuple-minimum, and
  tuple comparison over (lower, block, leaf) is total (bounds are
  finite), so the pop sequence equals ascending order. The kernel
  therefore seeds the heap as an ascending sorted array -- itself a
  valid min-heap with identical pop semantics -- and ports heapq's
  _siftup/_siftdown verbatim for later pushes.
- leaf stage: at a block's first pop, every leaf bound is computed with
  the same scalar expressions, survivors (bound <= best_squared AT THAT
  MOMENT) are collected in ascending leaf order, then stably sorted by
  (bound, leaf) -- exactly the reference flatnonzero + stable argsort.
- cursor advance: the next surviving leaf is offered only while its
  bound <= the CURRENT best_squared, and the cursor advances only on a
  successful push, matching push_next_leaf.
- member scan: ascending member order, strict '<' keeps the FIRST
  minimum, tie-break on the smaller flat id -- matching the reference.
- clock half of budget admission: checked at the SAME admission sites
  as the count half, but at a bounded stride so the interpreter
  round-trip stays cheap -- the read happens in an objmode block
  (measured ~255 ns/entry) through ``_clock_ns``, so tests can pin or
  advance it by patching this module attribute. The caller passes its
  OWN absolute deadline through UNCHANGED (both clocks are
  perf_counter_ns in production; with controlled clocks the patched
  caller clock stays authoritative precisely because no second
  re-anchoring read exists); kernel reads compare against it directly,
  so every dispatch-to-check gap (scheduling, first-call compilation,
  initialization) is spent against the budget -- never re-based away.
  Stride schedule: check at the first pop, then geometric backoff 32,
  64, ... capped at 512 pops; the first check covers an
  already-expired dispatch with the first POPPED key as the bound (the
  heap root, i.e. the true minimum lower -- cand_lowers itself is in
  block order, not lower order). Worst-case work past an expired
  budget is bounded by one stride interval -- <= ~80 us of steady leaf
  pops (~150 ns each) or <= ~0.2 ms when the interval is carried by
  block-decode pops (~microseconds each) -- instead of the reference's
  single-pop granularity.

Without numba this module is inert and the original path is used.
"""
import numpy as np

try:
    from numba import njit, objmode, types as _nb_types
    HAS_NUMBA = True
except ImportError:  # pragma: no cover - depends on host
    HAS_NUMBA = False

    def njit(*args, **kwargs):  # type: ignore[misc]
        if args and callable(args[0]):
            return args[0]

        def wrap(fn):
            return fn
        return wrap


def _clock_ns():
    """Monotonic clock for the in-kernel budget check.

    Only ever executed inside an objmode block (the nopython compiler
    cannot type this interpreter build's ``time`` module). Resolved
    from this module's globals at call time, so tests can substitute a
    controlled clock by patching ``distance_kernels_jit._clock_ns``.

    In production this is the SAME clock the caller's deadline is
    expressed in (perf_counter_ns), and the caller passes that absolute
    deadline through UNCHANGED -- there is no re-anchoring read
    anywhere, so no two-read conversion window can re-base the budget
    (reviews reproduced a 10 ms gap between the remaining-read and a
    re-anchoring read stealing the whole gap from a 1 ms budget).
    """
    import time
    return time.perf_counter_ns()


STATUS_MATCH, STATUS_RADIUS_CLEAR, STATUS_BUDGET = 0, 1, 2


@njit(cache=True)
def _less(a_l, a_b, a_f, b_l, b_b, b_f):
    if a_l != b_l:
        return a_l < b_l
    if a_b != b_b:
        return a_b < b_b
    return a_f < b_f


@njit(cache=True)
def _siftdown(hl, hb, hf, startpos, pos):
    newl, newb, newf = hl[pos], hb[pos], hf[pos]
    while pos > startpos:
        parent = (pos - 1) >> 1
        if _less(newl, newb, newf, hl[parent], hb[parent], hf[parent]):
            hl[pos] = hl[parent]; hb[pos] = hb[parent]; hf[pos] = hf[parent]
            pos = parent
        else:
            break
    hl[pos] = newl; hb[pos] = newb; hf[pos] = newf


@njit(cache=True)
def _siftup(hl, hb, hf, endpos, pos):
    startpos = pos
    newl, newb, newf = hl[pos], hb[pos], hf[pos]
    child = 2 * pos + 1
    while child < endpos:
        right = child + 1
        if right < endpos and not _less(hl[child], hb[child], hf[child],
                                        hl[right], hb[right], hf[right]):
            child = right
        hl[pos] = hl[child]; hb[pos] = hb[child]; hf[pos] = hf[child]
        pos = child
        child = 2 * pos + 1
    hl[pos] = newl; hb[pos] = newb; hf[pos] = newf
    _siftdown(hl, hb, hf, startpos, pos)


@njit(cache=True)
def _push(hl, hb, hf, size, l, b, f):
    hl[size] = l; hb[size] = b; hf[size] = f
    _siftdown(hl, hb, hf, 0, size)


@njit(cache=True)
def _pop_root(hl, hb, hf, size):
    rl, rb, rf = hl[0], hb[0], hf[0]
    last = size - 1
    hl[0] = hl[last]; hb[0] = hb[last]; hf[0] = hf[last]
    _siftup(hl, hb, hf, last, 0)
    return rl, rb, rf


@njit(cache=True, inline='always')
def _leaf_bound(low, high, point, lf):
    dx = low[lf, 0] - point[0]
    if point[0] - high[lf, 0] > dx:
        dx = point[0] - high[lf, 0]
    if dx < 0.0:
        dx = 0.0
    dy = low[lf, 1] - point[1]
    if point[1] - high[lf, 1] > dy:
        dy = point[1] - high[lf, 1]
    if dy < 0.0:
        dy = 0.0
    dz = low[lf, 2] - point[2]
    if point[2] - high[lf, 2] > dz:
        dz = point[2] - high[lf, 2]
    if dz < 0.0:
        dz = 0.0
    # numpy einsum's 3-wide row sum accumulates as (x^2 + z^2) + y^2
    # (SIMD 2-lane pattern, verified exhaustively against einsum); keep
    # that exact order so bounds match the reference bit-for-bit.
    return (dx * dx + dz * dz) + dy * dy


@njit(cache=True)
def _clock_check(deadline_ns, pops, check_next, stride):
    """Read the clock through objmode and compare against the ABSOLUTE
    deadline. Returns (expired, next_check_pop); the caller doubles the
    stride (capped) after each passing check."""
    with objmode(now=_nb_types.int64):
        now = _clock_ns()
    if now >= deadline_ns:
        return 1, check_next
    return 0, pops + stride


@njit(cache=True)
def kernel_nearest(point, radius_squared,
                   cand_blocks, cand_lowers,
                   blk_leaf_base,
                   leaf_low, leaf_high, leaf_gstart, leaf_gend,
                   mem_low, mem_high, mem_flat,
                   max_cells, max_blocks, deadline_ns, cells, blocks,
                   scratch_bounds, scratch_order):
    """One (point, state) traversal. cand_blocks/cand_lowers are the
    radius-admitted candidates in ascending BLOCK order (not lower
    order -- the seeded heap below holds the first-pop minimum);
    blk_leaf_base maps block position -> first concat-leaf of that
    block (with an end sentinel). deadline_ns is the caller's ABSOLUTE
    budget deadline, passed through UNCHANGED (same perf_counter_ns
    clock as this module's reads in production): kernel clock reads
    compare against it directly, so every gap between the caller's
    deadline-setting read and the checks (scheduling, first-call
    compilation, initialization) is spent against the budget -- no
    re-anchoring read exists anywhere. cells/blocks are
    1-element carriers of the request-wide counters. scratch_* are
    per-block cursor rows (bounds + survivor order). Returns (status,
    best_sq, bound_sq, best_flat, best_member, best_block_pos)."""
    n = cand_blocks.shape[0]
    heap_cap = 2 * n + 4
    hl = np.empty(heap_cap, np.float64)
    hb = np.empty(heap_cap, np.int64)
    hf = np.empty(heap_cap, np.int64)
    # Seed ascending by (lower, block): a sorted array is a valid
    # min-heap whose pop sequence equals heapq's over these tuples.
    size = 0
    for i in range(n):
        l = cand_lowers[i]
        b = cand_blocks[i]
        j = size
        while j > 0 and _less(l, b, -1, hl[j - 1], hb[j - 1], hf[j - 1]):
            hl[j] = hl[j - 1]; hb[j] = hb[j - 1]; hf[j] = hf[j - 1]
            j -= 1
        hl[j] = l; hb[j] = b; hf[j] = -1
        size += 1

    best_squared = radius_squared
    best_flat = -1
    best_member = -1
    best_bpos = -1

    # No pre-expiry special case: the first stride check at pops=1
    # fails exactly like the reference's first failed admits -- before
    # any charge, with the first POPPED key as the bound, which is the
    # true minimum lower (the heap root), not cand_lowers[0].
    check_next = 0
    stride = 32
    pops = 0

    while size > 0:
        lower, bpos, leaf = _pop_root(hl, hb, hf, size)
        size -= 1
        pops += 1
        if lower > best_squared:
            break
        leaf_base = blk_leaf_base[bpos]
        leaf_end = blk_leaf_base[bpos + 1]
        if leaf < 0:
            # The reference charges every pop (examined=0 here) before
            # decoding: same admission arithmetic, same block increment.
            if cells[0] > max_cells or blocks[0] + 1 > max_blocks:
                return (STATUS_BUDGET, best_squared, min(lower, best_squared),
                        best_flat, best_member, best_bpos)
            if pops >= check_next:
                expired, check_next = _clock_check(deadline_ns,
                                                   pops, check_next, stride)
                if expired:
                    return (STATUS_BUDGET, best_squared,
                            min(lower, best_squared),
                            best_flat, best_member, best_bpos)
                if stride < 512:
                    stride = 2 * stride
            blocks[0] += 1
            # First pop: bounds for every leaf, survivors filtered by the
            # CURRENT best_squared, stably ordered by (bound, leaf).
            n_leaves = leaf_end - leaf_base
            bounds_row = scratch_bounds[bpos]
            order_row = scratch_order[bpos]
            count = 0
            for lf in range(n_leaves):
                bounds_row[lf] = _leaf_bound(leaf_low, leaf_high, point,
                                             leaf_base + lf)
            for lf in range(n_leaves):        # survivors, ascending leaf
                if bounds_row[lf] <= best_squared:
                    # stable insertion into order by bound
                    bound = bounds_row[lf]
                    j = count
                    while j > 0 and bounds_row[order_row[j - 1]] > bound:
                        order_row[j] = order_row[j - 1]
                        j -= 1
                    order_row[j] = lf
                    count += 1
            order_row[n_leaves] = count       # sentinel: survivor count
            order_row[n_leaves + 1] = 0       # cursor position
            # offer the first survivor now (same best as the filter)
            pos, total = 0, count
            if pos < total:
                lf = order_row[pos]
                bound = bounds_row[lf]
                if bound <= best_squared:
                    _push(hl, hb, hf, size, bound, bpos, lf)
                    size += 1
                    order_row[n_leaves + 1] = 1
            continue
        gstart = leaf_gstart[leaf_base + leaf]
        gend = leaf_gend[leaf_base + leaf]
        examined = gend - gstart
        if cells[0] + examined > max_cells or blocks[0] + 1 > max_blocks:
            return (STATUS_BUDGET, best_squared, min(lower, best_squared),
                    best_flat, best_member, best_bpos)
        if pops >= check_next:
            expired, check_next = _clock_check(deadline_ns,
                                               pops, check_next, stride)
            if expired:
                return (STATUS_BUDGET, best_squared, min(lower, best_squared),
                        best_flat, best_member, best_bpos)
            if stride < 512:
                stride = 2 * stride
        cells[0] += examined
        blocks[0] += 1
        minimum = 0.0
        min_member = -1
        for m in range(gstart, gend):
            dx = mem_low[m, 0] - point[0]
            if point[0] - mem_high[m, 0] > dx:
                dx = point[0] - mem_high[m, 0]
            if dx < 0.0:
                dx = 0.0
            dy = mem_low[m, 1] - point[1]
            if point[1] - mem_high[m, 1] > dy:
                dy = point[1] - mem_high[m, 1]
            if dy < 0.0:
                dy = 0.0
            dz = mem_low[m, 2] - point[2]
            if point[2] - mem_high[m, 2] > dz:
                dz = point[2] - mem_high[m, 2]
            if dz < 0.0:
                dz = 0.0
            squared = (dx * dx + dz * dz) + dy * dy
            if min_member < 0 or squared < minimum:
                minimum = squared
                min_member = m
        if min_member >= 0:
            flat = mem_flat[min_member]
            if minimum < best_squared or (minimum == best_squared
                                          and (best_flat < 0 or flat < best_flat)):
                best_squared = minimum
                best_flat = flat
                best_member = min_member
                best_bpos = bpos
        # offer the next survivor AFTER the scan (the reference filters
        # with the updated best_squared); skip silently on budget return
        n_leaves = leaf_end - leaf_base
        bounds_row = scratch_bounds[bpos]
        order_row = scratch_order[bpos]
        total = order_row[n_leaves]
        pos = order_row[n_leaves + 1]
        if pos < total:
            lf = order_row[pos]
            bound = bounds_row[lf]
            if bound <= best_squared:
                _push(hl, hb, hf, size, bound, bpos, lf)
                size += 1
                order_row[n_leaves + 1] = pos + 1

    if best_flat < 0:
        return (STATUS_RADIUS_CLEAR, best_squared, 0.0, -1, -1, -1)
    return (STATUS_MATCH, best_squared, best_squared, best_flat,
            best_member, best_bpos)
