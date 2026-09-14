from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'a2_localization_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Unitree A2 Localization',
    maintainer_email='maintainer@example.com',
    description='Integrated Gazebo / FAST-LIO bringup and evaluation for Unitree A2.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gait_controller = a2_localization_bringup.gait_controller:main',
            'odom_tf_adapter = a2_localization_bringup.odom_tf_adapter:main',
            'trajectory_commander = a2_localization_bringup.trajectory_commander:main',
            'trajectory_evaluator = a2_localization_bringup.trajectory_evaluator:main',
        ],
    },
)
