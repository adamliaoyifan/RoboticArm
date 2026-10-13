"""ROS-free boundary context and structured failures; no logging or side effects."""
from dataclasses import dataclass

from luggage_description.frame_contracts import ContractError, require_equal


@dataclass(frozen=True)
class BoundaryContext:
    interface_id: str
    producer: str
    consumer: str
    request_id: str
    scene_revision: str
    frame_id: str
    reference: str
    time_basis: str
    geometry_hash: str = ""
    map_revision: object = None

    def __post_init__(self):
        for name in ("interface_id", "producer", "consumer", "request_id", "scene_revision"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ContractError("MISSING_IDENTITY", "context." + name, "nonempty string", value)
        if self.time_basis not in ("acquisition", "robot_state", "scene"):
            raise ContractError("TIME_BASIS_MISMATCH", "context.time_basis", "explicit basis", self.time_basis)


def diagnostic(record, context, error=None):
    """Keep malformed/missing declarations visible without repairing the payload."""
    record = record if isinstance(record, dict) else {}
    result = dict(contract_version=2, interface_id=context.interface_id,
                  request_id=str(record.get("request_id", "")),
                  expected_request_id=context.request_id, producer=context.producer,
                  consumer=context.consumer, accepted=error is None,
                  code="OK" if error is None else error.code,
                  field="" if error is None else error.field,
                  expected="" if error is None else repr(error.expected),
                  received="" if error is None else repr(error.received),
                  declared_stamp_ns=repr(record.get("stamp_ns")))
    for output, key in (("frame", "frame_id"), ("reference", "reference"),
                        ("scene_revision", "scene_revision"), ("map_revision", "map_revision"),
                        ("geometry_hash", "geometry_hash"), ("time_basis", "time_basis")):
        result["declared_" + output] = str(record.get(key, ""))
        value = getattr(context, key)
        result["expected_" + output] = "" if value is None else str(value)
    return result


class BoundaryContractError(ContractError):
    def __init__(self, error, record, context):
        super().__init__(error.code, error.field, error.expected, error.received)
        self.diagnostic = diagnostic(record, context, error)


def at_boundary(record, context, operation):
    """Run a pure conversion; reject before the caller can dispatch its result."""
    try:
        return operation()
    except BoundaryContractError:
        raise
    except ContractError as error:
        raise BoundaryContractError(error, record, context) from error


def validate_context(record, context, *, spatial=True):
    require_equal(record, "request_id", context.request_id, "REQUEST_IDENTITY_MISMATCH")
    require_equal(record, "scene_revision", context.scene_revision, "CALIBRATION_REVISION_MISMATCH")
    require_equal(record, "time_basis", context.time_basis, "TIME_BASIS_MISMATCH")
    if spatial:
        require_equal(record, "frame_id", context.frame_id, "FRAME_CONTRACT_MISMATCH")
        if context.reference:
            require_equal(record, "reference", context.reference, "POSE_REFERENCE_MISMATCH")
