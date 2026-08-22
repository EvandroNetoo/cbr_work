from glob import glob
from setuptools import setup


package_name = 'apriltag'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='AprilTag poses from a camera mounted on the SO-ARM-101.',
    license='GPL-3.0-only',
    entry_points={
        'console_scripts': [
            'apriltag_detector = apriltag.apriltag_detector:main',
        ],
    },
)
