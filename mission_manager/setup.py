import os
from glob import glob

from setuptools import setup


package_name = 'mission_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='Finite, cancelable mission manager for the CBR robot',
    license='GPL-3.0-only',
    entry_points={'console_scripts': [
        'mission_manager = mission_manager.node:main',
    ]},
)
