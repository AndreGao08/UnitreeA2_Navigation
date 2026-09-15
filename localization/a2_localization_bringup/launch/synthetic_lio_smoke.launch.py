from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('a2_localization_bringup'), 'config', 'a2_jt128_sim.yaml'])
    return LaunchDescription([
        Node(package='hesai_jt128_sim', executable='synthetic_sensor', output='screen'),
        Node(package='hesai_jt128_sim', executable='cloud_adapter', output='screen'),
        Node(package='hesai_jt128_sim', executable='imu_adapter', output='screen'),
        Node(package='hesai_jt128_sim', executable='input_monitor', output='screen'),
        Node(package='fast_lio', executable='fastlio_mapping', name='laser_mapping',
             parameters=[config, {'use_sim_time': False}], output='screen'),
        Node(package='a2_localization_bringup', executable='odom_tf_adapter',
             parameters=[{'use_sim_time': False}], output='screen'),
    ])
