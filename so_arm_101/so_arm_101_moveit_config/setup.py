import os
from glob import glob

from setuptools import setup


package_name = 'so_arm_101_moveit_config'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', '.setup_assistant']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='MoveIt 2 configuration for SO-ARM-101',
    license='GPL-3.0-only',
)
