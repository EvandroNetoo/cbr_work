import os
from glob import glob

from setuptools import setup


package_name = 'lidar'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='Driver ROS 2 do LiDAR XV-11 da base CBR',
    license='GPL-3.0-only',
    entry_points={'console_scripts': [
        'lidar_node = lidar.lidar_node:main',
    ]},
)
