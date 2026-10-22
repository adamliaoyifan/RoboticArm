"""Load explicit diagnostic bounds once, before freezing attachments."""
from pathlib import Path
import yaml
from st09_contracts._base import ContractViolation
from st09_contracts.payload_envelope import EnvelopeLimits


def load_envelope_limits(path):
    try:
        data = yaml.safe_load(Path(path).read_text())
        keys = {'yaw_half_range_deg', 'size_margin_per_face_m', 'position_half_range_tool_m'}
        if not isinstance(data, dict) or set(data) != keys:
            raise ValueError('expected exactly ' + ', '.join(sorted(keys)))
        return EnvelopeLimits(**data)
    except (OSError, ValueError, TypeError, yaml.YAMLError) as exc:
        raise ContractViolation('PAYLOAD_ENVELOPE_CONFIG_INVALID: %s' % exc) from exc
