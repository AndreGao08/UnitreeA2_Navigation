from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from a2_terrain_nav.pcd_map import project_pcd_to_nav2


def _project_map_path(filename: str) -> str:
    prefix = Path(get_package_prefix('a2_terrain_nav')).resolve()
    for candidate in (prefix, *prefix.parents):
        if (candidate / 'src' / 'a2_terrain_nav').is_dir():
            return str(candidate / 'maps' / filename)
    return str(Path.cwd() / 'maps' / filename)


def _launch_navigation(context, *args, **kwargs):
    pcd_path = Path(LaunchConfiguration('pcd_map').perform(context)).expanduser().resolve()
    yaml_path = Path(LaunchConfiguration('map').perform(context)).expanduser().resolve()
    generate_map = LaunchConfiguration('generate_map').perform(context).lower() in {
        '1', 'true', 'yes', 'on'
    }
    if generate_map and (
        not yaml_path.exists()
        or (pcd_path.exists() and pcd_path.stat().st_mtime > yaml_path.stat().st_mtime)
    ):
        if not pcd_path.exists():
            raise RuntimeError(
                f'Cannot generate Nav2 map: FAST-LIO PCD does not exist: {pcd_path}'
            )
        result = project_pcd_to_nav2(pcd_path, yaml_path)
        print(
            f'[a2_terrain_nav] generated {result.width}x{result.height} map at '
            f'{result.yaml_path} (estimated ground_z={result.ground_height:.3f})'
        )
    if not yaml_path.exists():
        raise RuntimeError(
            f'Nav2 map YAML does not exist: {yaml_path}. '
            'Set generate_map:=true or provide map:=/path/to/map.yaml.'
        )

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': str(yaml_path), 'use_sim_time': use_sim_time}],
    )
    map_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server'],
        }],
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('nav2_bringup')) / 'launch' / 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'use_composition': 'False',
        }.items(),
    )
    return [map_server, map_lifecycle_manager, navigation]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('generate_map', default_value='true'),
        DeclareLaunchArgument('pcd_map', default_value=_project_map_path('a2_map.pcd')),
        DeclareLaunchArgument('map', default_value=_project_map_path('a2_nav2_map.yaml')),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('a2_terrain_nav'), 'config', 'nav2_params.yaml'
            ]),
        ),
        OpaqueFunction(function=_launch_navigation),
    ])
