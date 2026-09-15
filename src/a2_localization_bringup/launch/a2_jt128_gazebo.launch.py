from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _default_project_map_path():
    """Find this colcon workspace and keep the default map inside it."""
    prefix = Path(get_package_prefix('a2_localization_bringup')).resolve()
    for candidate in (prefix, *prefix.parents):
        if (candidate / 'src' / 'a2_localization_bringup').is_dir():
            return str(candidate / 'maps' / 'a2_map.pcd')
    return str(Path.cwd() / 'maps' / 'a2_map.pcd')


def generate_launch_description():
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    trajectory = LaunchConfiguration('trajectory')
    locomotion_mode = LaunchConfiguration('locomotion_mode')
    operation_mode = LaunchConfiguration('operation_mode')
    map_path = LaunchConfiguration('map_path')
    auto_initialize = LaunchConfiguration('auto_initialize')
    lidar_xyz = LaunchConfiguration('lidar_xyz')
    lidar_rpy = LaunchConfiguration('lidar_rpy')
    horizontal_samples = LaunchConfiguration('horizontal_samples')
    eval_directory = LaunchConfiguration('eval_directory')
    rmw_implementation = LaunchConfiguration('rmw_implementation')
    gazebo_verbosity = LaunchConfiguration('gazebo_verbosity')
    robot_cmd_vel_topic = LaunchConfiguration('robot_cmd_vel_topic')

    world = PathJoinSubstitution([
        FindPackageShare('a2_gazebo'), 'worlds', PythonExpression([
            "'a2_localization_dynamic.sdf' if '", locomotion_mode,
            "' == 'dynamic' else ('a2_localization_gait_demo.sdf' if '",
            locomotion_mode,
            "' == 'gait_demo' else 'a2_localization.sdf')",
        ])])
    urdf = PathJoinSubstitution([FindPackageShare('a2_description'), 'urdf', 'a2_jt128.urdf.xacro'])
    fast_lio_config = PathJoinSubstitution([
        FindPackageShare('a2_localization_bringup'), 'config', 'a2_jt128_sim.yaml'])
    gait_config = PathJoinSubstitution([
        FindPackageShare('a2_localization_bringup'), 'config', 'a2_gait.yaml'])
    relocalization_config = LaunchConfiguration('relocalization_params_file')
    rviz_config = PathJoinSubstitution([
        FindPackageShare('a2_localization_bringup'), 'rviz', PythonExpression([
            "'a2_relocalization.rviz' if '", operation_mode,
            "' == 'relocalization' else 'a2_localization.rviz'",
        ])])

    robot_description = ParameterValue(Command([
        'xacro ', urdf,
        ' lidar_xyz:="', lidar_xyz, '"',
        ' lidar_rpy:="', lidar_rpy, '"',
        ' lidar_horizontal_samples:=', horizontal_samples,
        ' lidar_update_rate:=10',
        ' lidar_visualize:=false',
        ' locomotion_mode:=', locomotion_mode,
    ]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            'gz_args': [PythonExpression([
                "'-r -s' if '", gui, "' == 'false' else '-r'"]),
                ' -v ', gazebo_verbosity, ' ', world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen')
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', 'a2_localization', '-name', 'unitree_a2',
                   '-topic', 'robot_description', '-x', '0', '-y', '0', '-z',
                   PythonExpression([
                       # Dynamic URDF joints spawn at zero (fully extended),
                       # so release the feet above the floor.  The controller
                       # folds them into the nominal A2 pose while falling.
                       "'0.65' if '", locomotion_mode,
                       "' == 'dynamic' else ('0.38' if '", locomotion_mode,
                       "' == 'gait_demo' else '0.59')",
                   ])])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/a2/jt128/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/a2/imu_raw@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/a2/ground_truth/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/a2/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/a2/base_velocity_assist@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/model/unitree_a2/joint_trajectory@trajectory_msgs/msg/JointTrajectory]ignition.msgs.JointTrajectory',
        ],
        remappings=[
            ('/a2/joint_states', '/joint_states'),
            # In navigation mode this resolves to /a2/safe_cmd_vel, so neither
            # Nav2 nor teleop can bypass the localization safety gate.
            ('/cmd_vel', robot_cmd_vel_topic),
        ])

    cloud_adapter = Node(
        package='hesai_jt128_sim', executable='cloud_adapter', output='screen',
        parameters=[{'use_sim_time': False, 'input_topic': '/a2/jt128/points',
                     'output_topic': '/lidar_points', 'scan_rate_hz': 10.0,
                     'scan_lines': 128, 'output_frame': 'hesai_jt128_link'}])
    imu_adapter = Node(
        package='hesai_jt128_sim', executable='imu_adapter', output='screen',
        parameters=[{'use_sim_time': False, 'input_topic': '/a2/imu_raw',
                     'output_topic': '/lidar_imu', 'output_frame': 'hesai_jt128_imu_link',
                     'add_gravity': ParameterValue(PythonExpression([
                         "'", locomotion_mode, "' != 'dynamic'"
                     ]), value_type=bool)}])
    input_monitor = Node(
        package='hesai_jt128_sim', executable='input_monitor', output='screen',
        parameters=[{'use_sim_time': False}])

    fast_lio = Node(
        package='fast_lio', executable='fastlio_mapping', name='laser_mapping',
        parameters=[fast_lio_config, {
            'use_sim_time': True,
            'map_file_path': ParameterValue(map_path, value_type=str),
            'pcd_save.pcd_save_en': ParameterValue(PythonExpression([
                "'", operation_mode, "' == 'mapping'"
            ]), value_type=bool),
        }], output='screen')

    odom_adapter = Node(
        package='a2_localization_bringup', executable='odom_tf_adapter', output='screen',
        parameters=[{
            'use_sim_time': False,
            'base_to_imu_xyz_text': ParameterValue(lidar_xyz, value_type=str),
            'base_to_imu_rpy_text': ParameterValue(lidar_rpy, value_type=str),
        }])
    evaluator = Node(
        package='a2_localization_bringup', executable='trajectory_evaluator', output='screen',
        parameters=[{
            'use_sim_time': True,
            'output_directory': eval_directory,
            'estimate_topic': ParameterValue(PythonExpression([
                "'/a2/localization' if '", operation_mode,
                "' == 'relocalization' else '/a2/odometry'",
            ]), value_type=str),
        }])
    commander = Node(
        package='a2_localization_bringup', executable='trajectory_commander', output='screen',
        parameters=[{'use_sim_time': True, 'trajectory': trajectory,
                     'cmd_vel_topic': '/cmd_vel'}])
    gait_controller = Node(
        package='a2_localization_bringup', executable='gait_controller',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", locomotion_mode, "' != 'kinematic'"
        ])),
        parameters=[gait_config, {
            'use_sim_time': True,
            'cmd_vel_topic': robot_cmd_vel_topic,
            'base_motion_assist': ParameterValue(PythonExpression([
                "'", locomotion_mode, "' == 'gait_demo'"
            ]), value_type=bool),
        }])
    map_localizer = Node(
        package='a2_map_localization', executable='map_localizer',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", operation_mode, "' == 'relocalization'"
        ])),
        parameters=[relocalization_config, {
            'use_sim_time': True,
            'map_path': ParameterValue(map_path, value_type=str),
            'auto_initialize': ParameterValue(auto_initialize, value_type=bool),
        }])
    rviz_node = Node(
        package='rviz2', executable='rviz2', arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}], condition=IfCondition(rviz), output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('trajectory', default_value='manual',
                              description='manual, stationary, straight, square, or figure8'),
        DeclareLaunchArgument(
            'operation_mode', default_value='mapping',
            description='mapping saves a PCD; relocalization loads it and publishes map -> odom',
            choices=['mapping', 'relocalization']),
        DeclareLaunchArgument(
            'map_path',
            default_value=_default_project_map_path(),
            description='PCD path; defaults to this project maps/a2_map.pcd'),
        DeclareLaunchArgument(
            'relocalization_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('a2_map_localization'), 'config', 'relocalization.yaml'
            ]),
            description='Parameters for scan-to-map initialization and periodic correction'),
        DeclareLaunchArgument(
            'auto_initialize', default_value='false',
            description='Use the zero xyz/rpy initial guess instead of waiting for RViz /initialpose'),
        DeclareLaunchArgument(
            'locomotion_mode', default_value='kinematic',
            description=(
                'kinematic is rigid-body LIO; gait_demo animates the gait with '
                'base velocity assistance; dynamic is unassisted contact dynamics'
            )),
        DeclareLaunchArgument('lidar_xyz', default_value='0.33767 0.0 0.08134'),
        DeclareLaunchArgument('lidar_rpy', default_value='0.0 0.0 0.0'),
        DeclareLaunchArgument('horizontal_samples', default_value='256'),
        DeclareLaunchArgument('gazebo_verbosity', default_value='3'),
        DeclareLaunchArgument(
            'robot_cmd_vel_topic', default_value='/cmd_vel',
            description='Velocity topic consumed by Gazebo and the gait controller'),
        DeclareLaunchArgument('eval_directory', default_value='/tmp/a2_localization_eval'),
        DeclareLaunchArgument(
            'rmw_implementation',
            default_value='rmw_fastrtps_cpp',
            description=(
                'ROS 2 middleware used by every launched process. Fast DDS is the '
                'simulation default because its local shared-memory transport handles '
                'the large JT128 PointCloud2 stream reliably.'
            ),
        ),
        SetEnvironmentVariable(
            name='RMW_IMPLEMENTATION', value=rmw_implementation),
        gazebo,
        robot_state_publisher,
        bridge,
        TimerAction(period=2.0, actions=[spawn]),
        cloud_adapter,
        imu_adapter,
        input_monitor,
        fast_lio,
        odom_adapter,
        evaluator,
        # Publish zero velocity before the model is spawned so the localization
        # platform starts from a deterministic, stationary state.
        commander,
        gait_controller,
        map_localizer,
        rviz_node,
    ])
