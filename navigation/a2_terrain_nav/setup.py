from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'a2_terrain_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'numpy'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Unitree A2 Navigation',
    maintainer_email='maintainer@example.com',
    description='GSeg3D, Ground Consistency and Nav2 integration for Unitree A2.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'localization_safety_monitor = a2_terrain_nav.localization_safety_monitor:main',
            'pcd_to_nav2_map = a2_terrain_nav.pcd_map:main',
        ],
    },
)
