import os
from glob import glob
from setuptools import setup

package_name = 'cbr_simulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='CBR Gazebo Sim integration',
    license='GPL-3.0-only',
    entry_points={'console_scripts': [
        'simulated_xv11 = cbr_simulation.scan_filter:main',
    ]},
)
