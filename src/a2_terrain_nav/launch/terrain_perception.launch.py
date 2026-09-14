from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')

    ground_segmentation = Node(
        package='ground_segmentation_ros2',
        executable='ground_segmentation_ros2_node',
        name='ground_segmentation',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/ground_segmentation/input_pointcloud', pointcloud_topic),
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
        DeclareLaunchArgument('imu_topic', default_value='/lidar_imu'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        ground_segmentation,
    ])
