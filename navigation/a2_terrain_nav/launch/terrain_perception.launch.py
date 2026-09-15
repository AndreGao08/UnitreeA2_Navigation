from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    rear_pointcloud_topic = LaunchConfiguration('rear_pointcloud_topic')
    navigation_pointcloud_topic = LaunchConfiguration('navigation_pointcloud_topic')
    front_fov_deg = LaunchConfiguration('front_fov_deg')
    rear_fov_deg = LaunchConfiguration('rear_fov_deg')
    imu_topic = LaunchConfiguration('imu_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')

    dual_lidar_filter = Node(
        package='a2_dual_lidar_nav',
        executable='dual_lidar_nav_filter',
        name='dual_lidar_nav_filter',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'front_topic': pointcloud_topic,
            'rear_topic': rear_pointcloud_topic,
            'output_topic': navigation_pointcloud_topic,
            'target_frame': 'base_link',
            'front_fov_deg': ParameterValue(front_fov_deg, value_type=float),
            'rear_fov_deg': ParameterValue(rear_fov_deg, value_type=float),
        }],
    )

    ground_segmentation = Node(
        package='ground_segmentation_ros2',
        executable='ground_segmentation_ros2_node',
        name='ground_segmentation',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/ground_segmentation/input_pointcloud', navigation_pointcloud_topic),
            ('/ground_segmentation/input_imu', imu_topic),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('a2_terrain_nav'), 'config', 'gseg3d_a2.yaml'
            ]),
        ),
        DeclareLaunchArgument('pointcloud_topic', default_value='/lidar_points'),
        DeclareLaunchArgument('rear_pointcloud_topic', default_value='/lidar_points_2'),
        DeclareLaunchArgument(
            'navigation_pointcloud_topic', default_value='/a2/navigation/lidar_points'),
        DeclareLaunchArgument(
            'front_fov_deg', default_value='180.0',
            description='Horizontal navigation FOV retained from the front LiDAR'),
        DeclareLaunchArgument(
            'rear_fov_deg', default_value='180.0',
            description='Horizontal navigation FOV retained from the rear LiDAR'),
        DeclareLaunchArgument('imu_topic', default_value='/lidar_imu'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        dual_lidar_filter,
        ground_segmentation,
    ])
