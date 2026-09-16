from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _project_map_path(filename: str) -> str:
    prefix = Path(get_package_prefix('a2_terrain_nav')).resolve()
    for candidate in (prefix, *prefix.parents):
        if (candidate / 'src' / 'navigation' / 'a2_terrain_nav').is_dir():
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
    horizontal_samples = LaunchConfiguration('horizontal_samples')
    front_lidar_topic = LaunchConfiguration('front_lidar_topic')
    rear_lidar_topic = LaunchConfiguration('rear_lidar_topic')

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
            'horizontal_samples': horizontal_samples,
            'robot_cmd_vel_topic': '/a2/safe_cmd_vel',
            'relocalization_params_file': PathJoinSubstitution([
                FindPackageShare('a2_terrain_nav'), 'config', 'relocalization_nav.yaml'
            ]),
        }.items(),
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('a2_terrain_nav'), 'launch', 'terrain_perception.launch.py'
        ])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'pointcloud_topic': front_lidar_topic,
            'rear_pointcloud_topic': rear_lidar_topic,
            'front_fov_deg': '180.0',
            'rear_fov_deg': '180.0',
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
            'minimum_points_per_cell': LaunchConfiguration('minimum_points_per_cell'),
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
    safety_monitor = Node(
        package='a2_terrain_nav',
        executable='localization_safety_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_cmd_vel_topic': '/cmd_vel',
            'output_cmd_vel_topic': '/a2/safe_cmd_vel',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map_pcd', default_value=_project_map_path('a2_map.pcd')),
        DeclareLaunchArgument('map_yaml', default_value=_project_map_path('a2_nav2_map.yaml')),
        DeclareLaunchArgument('auto_initialize', default_value='false'),
        DeclareLaunchArgument(
            'minimum_points_per_cell',
            default_value='4',
            description='Minimum PCD returns needed to mark a static-map cell occupied',
        ),
        DeclareLaunchArgument('locomotion_mode', default_value='gait_demo'),
        DeclareLaunchArgument(
            'gui',
            default_value='false',
            description='Show Gazebo GUI; disabled by default to reserve GPU capacity for JT128 and RViz',
        ),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'front_lidar_topic', default_value='/lidar_points',
            description='Front LiDAR cloud; remains the unchanged FAST-LIO input'),
        DeclareLaunchArgument(
            'rear_lidar_topic', default_value='/lidar_points_2',
            description='Rear LiDAR cloud used only by navigation'),
        DeclareLaunchArgument(
            'horizontal_samples',
            default_value='128',
            description='JT128 horizontal samples per scan; increase only with sufficient GPU headroom',
        ),
        DeclareLaunchArgument(
            'rmw_implementation',
            default_value='rmw_cyclonedds_cpp',
            description='ROS middleware used consistently by simulation, localization, and Nav2',
        ),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', rmw_implementation),
        # Keep child launch arguments (notably its rviz=false) from overwriting
        # the combined launch's public rviz argument.
        GroupAction(actions=[localization], scoped=True),
        TimerAction(period=3.0, actions=[perception]),
        # FAST-LIO needs its LiDAR/IMU initialization window before Nav2 can
        # resolve odom -> base_link. Delaying consumers avoids a false TF error
        # during an otherwise healthy startup.
        TimerAction(period=8.5, actions=[safety_monitor]),
        TimerAction(period=9.0, actions=[navigation]),
        TimerAction(period=10.0, actions=[rviz_node]),
    ])
