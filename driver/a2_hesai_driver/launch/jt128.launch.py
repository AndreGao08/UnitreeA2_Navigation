from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = LaunchConfiguration('config_path')
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('a2_hesai_driver'), 'config', 'jt128.yaml'
            ]),
            description='Hesai SDK YAML configuration for the physical JT128',
        ),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Start RViz with the upstream Hesai driver configuration',
        ),
        Node(
            package='hesai_ros_driver',
            executable='hesai_ros_driver_node',
            name='hesai_jt128_driver',
            output='screen',
            parameters=[{'config_path': config_path}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            condition=IfCondition(rviz),
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('hesai_ros_driver'), 'rviz', 'rviz2.rviz'
            ])],
            output='screen',
        ),
    ])
