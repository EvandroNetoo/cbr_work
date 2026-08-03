import os

from setuptools import setup


package_name = 'so_arm_101_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='SO-ARM-101 keyboard teleoperation',
    license='GPL-3.0-only',
    entry_points={
        'console_scripts': [
            'keyboard_teleop = so_arm_101_teleop.keyboard_teleop:main',
        ],
    },
    tests_require=['pytest'],
)
