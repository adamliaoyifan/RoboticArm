"""Strict generated-type and identity conversion helpers; no fallback identities."""
from luggage_msgs.msg import FrameIdentityV2
from luggage_description.frame_contracts import ContractError, validate_envelope
from luggage_description.frame_sequence_contracts import unsigned_integer


IDENTITY_FIELDS = ("contract_version", "frame_id", "length_unit", "time_basis", "scene_revision", "request_id")


def require_message(message, cls):
    if type(message) is not cls:
        raise ContractError("MESSAGE_TYPE_MISMATCH", "message", cls.__name__, type(message).__name__)


def identity_to_message(record, scene_revision):
    validate_envelope(record, scene_revision)
    unsigned_integer(record['stamp_ns'], 64, 'stamp_ns')
    message = FrameIdentityV2()
    for name in IDENTITY_FIELDS:
        setattr(message, name, record[name])
    message.stamp_valid = True
    message.stamp_ns = record['stamp_ns']
    return message


def identity_from_message(message):
    require_message(message, FrameIdentityV2)
    record = {name: getattr(message, name) for name in IDENTITY_FIELDS}
    record['stamp_ns'] = message.stamp_ns if message.stamp_valid else None
    return record


def reject_extra_fields(record, allowed):
    if not isinstance(record, dict):
        raise ContractError('INVALID_RECORD', 'record', 'object', record)
    extras = sorted(set(record) - set(allowed), key=str)
    if extras:
        raise ContractError('UNSUPPORTED_RECORD_FIELD', 'record', sorted(allowed), extras)
