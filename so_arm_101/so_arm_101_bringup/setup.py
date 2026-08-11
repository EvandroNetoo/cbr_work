import os
from glob import glob

from setuptools import setup


package_name = 'so_arm_101_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'wait_for_joint_states = so_arm_101_bringup.wait_for_joint_states:main',
            'wait_for_controllers = so_arm_101_bringup.wait_for_controllers:main',
        ],
    },
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='SO-ARM-101 bringup, controllers and simulation',
    license='GPL-3.0-only',
)
