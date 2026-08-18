import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'cbr_motor_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='ROS 2 motor control for the CBR mecanum/omnidirectional base.',
    license='GPL-3.0-only',
    entry_points={
        'console_scripts': [
            'motor_node = cbr_motor_control.motor_node:main',
        ],
    },
)

