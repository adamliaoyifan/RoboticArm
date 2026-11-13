"""Shared construction-time validation helpers (ROS-free)."""

from __future__ import annotations

import math
from typing import Any, Iterable, Mapping, Sequence


class ContractViolation(ValueError):
    """Raised when a frozen ST09 contract value object cannot be built."""


def finite(value: Any, field: str) -> float:
    """One finite float or :class:`ContractViolation`."""
    try:
        out = float(value)
    except (TypeError, ValueError) as exc:
        raise ContractViolation("%s: not a number (%r)" % (field, value)) from exc
    if not math.isfinite(out):
        raise ContractViolation("%s: non-finite (%r)" % (field, value))
    return out


def finite_tuple(values: Iterable[Any], size: int, field: str) -> tuple:
    """Fixed-length tuple of finite floats."""
    if values is None:
        raise ContractViolation("%s: missing" % field)
    row = tuple(values)
    if len(row) != size:
        raise ContractViolation(
            "%s: expected %d values, got %d" % (field, size, len(row)))
    return tuple(finite(v, "%s[%d]" % (field, i)) for i, v in enumerate(row))


def positive_dimensions(values: Sequence[Any], field: str) -> tuple:
    out = finite_tuple(values, 3, field)
    for i, v in enumerate(out):
        if v <= 0.0:
            raise ContractViolation(
                "%s[%d]: non-positive dimension (%r)" % (field, i, v))
    return out


def unit_quaternion(quat: Sequence[Any], field: str) -> tuple:
    """Unit quaternion as xyzw tuple; sign is a representation choice."""
    q = finite_tuple(quat, 4, field)
    norm = math.sqrt(sum(v * v for v in q))
    if not math.isfinite(norm) or norm == 0.0:
        raise ContractViolation("%s: zero quaternion" % field)
    if abs(norm - 1.0) > 1e-9:
        raise ContractViolation("%s: not unit (norm=%.12f)" % (field, norm))
    return q


def nonempty_str(value: Any, field: str) -> str:
    out = str(value)
    if not out.strip():
        raise ContractViolation("%s: empty string" % field)
    return out


def stamp_ns(value: Any, field: str) -> int:
    """Non-negative integer nanosecond stamp."""
    if isinstance(value, bool) or not isinstance(value, int):
        raise ContractViolation("%s: integer nanoseconds required (%r)" % (field, value))
    if value < 0:
        raise ContractViolation("%s: negative stamp (%r)" % (field, value))
    return value


def digest_str(value: Any, field: str) -> str:
    """Lowercase hex sha256 (64 chars) or a short stable symbolic id."""
    out = nonempty_str(value, field)
    return out


def require_mapping(value: Any, field: str) -> Mapping:
    if not isinstance(value, Mapping):
        raise ContractViolation("%s: mapping required (%r)" % (field, value))
    return value


def digest_of(*parts: Any) -> str:
    """Deterministic sha256 over the given scalar parts (repr-stable)."""
    import hashlib
    hasher = hashlib.sha256()
    for part in parts:
        hasher.update(repr(part).encode("utf-8"))
        hasher.update(b"\x00")
    return hasher.hexdigest()
