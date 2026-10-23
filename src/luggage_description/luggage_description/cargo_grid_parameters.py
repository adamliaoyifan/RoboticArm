"""Node/launch configuration boundary; no independently writable surface grid."""
from pathlib import Path
import yaml
from luggage_description.cargo_grid_contract import deployment_resolution


def default_resolution():
    # Source tests and installed runtime read the same packaged config file.
    source = Path(__file__).resolve().parents[1] / 'config' / 'cargo_grid.yaml'
    if not source.is_file():
        from ament_index_python.packages import get_package_share_directory
        source = Path(get_package_share_directory('luggage_description')) / 'config/cargo_grid.yaml'
    return deployment_resolution(yaml.safe_load(source.read_text())['resolution_m'])


def resolve_resolution(override=''):
    return default_resolution() if override in ('', None) else deployment_resolution(override)


def declare_resolution(node, parameter_name='resolution'):
    from rcl_interfaces.msg import ParameterDescriptor
    parameter = node.declare_parameter(
        parameter_name, default_resolution(),
        ParameterDescriptor(read_only=True, description=(
            'Cargo OCC/surface resolution in metres; startup-only, restart to change.')))
    value = deployment_resolution(parameter.value)
    node.get_logger().info('cargo grid %s=%.6fm (startup-only)' % (parameter_name, value))
    return value
