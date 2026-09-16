from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _default_project_map_path():
    prefix = Path(get_package_prefix('a2_map_localization')).resolve()
    for candidate in (prefix, *prefix.parents):
        if (candidate / 'src' / 'localization' / 'a2_map_localization').is_dir():
            return str(candidate / 'maps' / 'a2_map.pcd')
    return str(Path.cwd() / 'maps' / 'a2_map.pcd')


def generate_launch_description():
    map_path = LaunchConfiguration('map_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    auto_initialize = LaunchConfiguration('auto_initialize')
    config = PathJoinSubstitution([
        FindPackageShare('a2_map_localization'), 'config', 'relocalization.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_path',
            default_value=_default_project_map_path(),
            description='FAST-LIO PCD map to load'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'auto_initialize', default_value='false',
            description='Use initial_pose_xyzrpy from the parameter file'),
        Node(
            package='a2_map_localization',
            executable='map_localizer',
            output='screen',
            parameters=[config, {
                'map_path': ParameterValue(map_path, value_type=str),
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'auto_initialize': ParameterValue(auto_initialize, value_type=bool),
            }],
        ),
    ])
