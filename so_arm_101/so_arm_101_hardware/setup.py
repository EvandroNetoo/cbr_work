import os
from glob import glob

from setuptools import setup


package_name = 'so_arm_101_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml') + glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='SO-ARM-101 LeRobot/Feetech ROS 2 bridge',
    license='GPL-3.0-only',
    entry_points={
        'console_scripts': [
            'so101_hardware_node = so_arm_101_hardware.hardware_node:main',
        ],
    },
)
