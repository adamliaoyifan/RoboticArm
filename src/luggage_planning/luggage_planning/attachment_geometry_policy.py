"""Attachment boundary policy loading; motion consumers never load margins."""
from pathlib import Path
import yaml

from st09_contracts._base import ContractViolation
from st09_contracts.payload_envelope import EnvelopeLimits


def load_attachment_geometry_policy(path=''):
    if not path:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('luggage_planning')) / 'config' / 'attachment_geometry.yaml'
    try:
        data = yaml.safe_load(Path(path).read_text())
        keys = {'yaw_half_range_deg', 'size_growth_fraction_wdh',
                'position_half_range_tool_xy_m', 'away_from_contact_position_error_m'}
        if not isinstance(data, dict) or set(data) != keys:
            raise ValueError('expected exactly ' + ', '.join(sorted(keys)))
        return EnvelopeLimits(**data)
    except (OSError, ValueError, TypeError, yaml.YAMLError) as exc:
        raise ContractViolation('ATTACHMENT_GEOMETRY_CONFIG_INVALID: %s' % exc) from exc
