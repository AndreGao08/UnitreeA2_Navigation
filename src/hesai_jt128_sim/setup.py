from setuptools import find_packages, setup

package_name = 'hesai_jt128_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Unitree A2 Localization',
    maintainer_email='maintainer@example.com',
    description='Gazebo-to-Hesai JT128 PointCloud2 and IMU adapters.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cloud_adapter = hesai_jt128_sim.cloud_adapter:main',
            'imu_adapter = hesai_jt128_sim.imu_adapter:main',
            'input_monitor = hesai_jt128_sim.input_monitor:main',
            'synthetic_sensor = hesai_jt128_sim.synthetic_sensor:main',
        ],
    },
)
