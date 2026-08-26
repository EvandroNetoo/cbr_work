import os
from glob import glob

from setuptools import setup


package_name = 'imu'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='Driver ROS 2 leve da IMU da Mariola',
    license='GPL-3.0-only',
    entry_points={'console_scripts': [
        'imu_node = imu.imu_node:main',
    ]},
)
