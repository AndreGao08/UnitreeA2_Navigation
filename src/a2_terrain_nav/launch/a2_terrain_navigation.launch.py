from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _project_map_path(filename: str) -> str:
    prefix = Path(get_package_prefix('a2_terrain_nav')).resolve()
    for candidate in (prefix, *prefix.parents):
        if (candidate / 'src' / 'a2_terrain_nav').is_dir():
            return str(candidate / 'maps' / filename)
    return str(Path.cwd() / 'maps' / filename)


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_pcd = LaunchConfiguration('map_pcd')
    map_yaml = LaunchConfiguration('map_yaml')
    auto_initialize = LaunchConfiguration('auto_initialize')
    locomotion_mode = LaunchConfiguration('locomotion_mode')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    rmw_implementation = LaunchConfiguration('rmw_implementation')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('a2_localization_bringup'),
            'launch',
            'a2_jt128_gazebo.launch.py',
        ])),
        launch_arguments={
            'operation_mode': 'relocalization',
            'map_path': map_pcd,
            'auto_initialize': auto_initialize,
            'trajectory': 'manual',
            'locomotion_mode': locomotion_mode,
            'gui': gui,
            'rviz': 'false',
            'rmw_implementation': rmw_implementation,
        }.items(),
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('a2_terrain_nav'), 'launch', 'terrain_perception.launch.py'
        ])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution([
                FindPackageShare('a2_terrain_nav'), 'config', 'gseg3d_a2.yaml'
            ]),
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('a2_terrain_nav'), 'launch', 'navigation.launch.py'
        ])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'pcd_map': map_pcd,
            'map': map_yaml,
            'generate_map': 'true',
            'autostart': 'true',
            'params_file': PathJoinSubstitution([
                FindPackageShare('a2_terrain_nav'), 'config', 'nav2_params.yaml'
            ]),
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('a2_terrain_nav'), 'rviz', 'a2_terrain_nav.rviz'
        ])],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map_pcd', default_value=_project_map_path('a2_map.pcd')),
        DeclareLaunchArgument('map_yaml', default_value=_project_map_path('a2_nav2_map.yaml')),
        DeclareLaunchArgument('auto_initialize', default_value='false'),
        DeclareLaunchArgument('locomotion_mode', default_value='gait_demo'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rmw_implementation', default_value='rmw_fastrtps_cpp'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', rmw_implementation),
        localization,
        TimerAction(period=3.0, actions=[perception]),
        TimerAction(period=5.0, actions=[navigation]),
        TimerAction(period=6.0, actions=[rviz_node]),
    ])
