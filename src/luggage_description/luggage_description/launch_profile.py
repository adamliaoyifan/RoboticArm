"""ROS launch binding for the shared strict profile resolver."""
from .runtime_profile import PROFILE_SENTINEL, load_profile, resolve_profile, launch_text


def with_profile(description):
    """Wrap a launch with contiguous leading declarations, preserving CLI values."""
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
    from launch.substitutions import LaunchConfiguration
    from launch.utilities import perform_substitutions

    entities = list(description.entities)
    declarations = []
    while entities and isinstance(entities[0], DeclareLaunchArgument):
        declarations.append(entities.pop(0))
    defaults = {item.name: item.default_value for item in declarations}

    def resolve(context):
        values = {name: perform_substitutions(context, value) for name, value in defaults.items()}
        overrides = {name: LaunchConfiguration(name).perform(context) for name in defaults}
        path = LaunchConfiguration("profile_config").perform(context)
        resolved = resolve_profile(values, load_profile(path), overrides)
        return [SetLaunchConfiguration(name, launch_text(value)) for name, value in resolved.items()]

    wrapped = [DeclareLaunchArgument("profile_config", default_value="")]
    wrapped.extend(DeclareLaunchArgument(item.name, default_value=PROFILE_SENTINEL,
                                        description=item.description) for item in declarations)
    return LaunchDescription(wrapped + [OpaqueFunction(function=resolve)] + entities)
