import os
from glob import glob

from setuptools import setup


package_name = 'base_hardware'

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
    description='Python driver for the CBR Mariola mecanum base',
    license='GPL-3.0-only',
    entry_points={'console_scripts': [
        'base_hardware_node = base_hardware.hardware_node:main',
    ]},
)
