import os
from glob import glob

from setuptools import setup

package_name = 'base_bringup'
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
    maintainer='author', maintainer_email='todo@todo.com',
    description='Bringup for the CBR mecanum base', license='GPL-3.0-only',
    entry_points={'console_scripts': [
        'wait_for_wheel_states = base_bringup.wait_for_wheel_states:main',
    ]},
)
