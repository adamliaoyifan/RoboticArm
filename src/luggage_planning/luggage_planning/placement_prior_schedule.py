"""Background-only scheduling on a continuously confirmed snapshot.

The node holds its existing lock while using this state. Readiness here never
authorizes a grasp or bypasses snapshot checks during/after computation.
"""
import math


class SnapshotQuietGate:
    def __init__(self, quiet_s):
        if isinstance(quiet_s, bool) or not math.isfinite(quiet_s) or quiet_s < 0:
            raise ValueError('prefetch quiet interval must be finite and nonnegative')
        self.quiet_s = float(quiet_s)
        self.identity = None
        self.since = None

    def observe(self, identity, now):
        if identity != self.identity:
            self.identity = identity
            self.since = now if identity is not None else None

    def ready(self, identity, now):
        return (identity is not None and identity == self.identity
                and self.since is not None and now - self.since >= self.quiet_s)
