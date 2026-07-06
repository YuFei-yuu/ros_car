import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'course_design'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@example.com',
    description='Course design wrappers for ROS2 robot mapping and navigation.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_status_node = course_design.map_status_node:main',
            'waypoint_nav_node = course_design.waypoint_nav_node:main',
            'behavior_node = course_design.behavior_node:main',
        ],
    },
)
