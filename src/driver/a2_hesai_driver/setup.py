from glob import glob
from setuptools import setup


package_name = 'a2_hesai_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Unitree A2 Navigation',
    maintainer_email='maintainer@example.com',
    description='JT128 hardware-driver configuration for Unitree A2 Navigation.',
    license='Apache-2.0',
)
