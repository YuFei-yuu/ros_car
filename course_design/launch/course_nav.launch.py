import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def package_path(package_name):
    if os.environ.get('need_compile', 'False') == 'True':
        return get_package_share_directory(package_name)
    return f'/home/ubuntu/ros2_ws/src/{package_name}'


def load_course_config(course_design_path):
    config_file = os.path.join(course_design_path, 'config', 'course_design.yaml')
    with open(config_file, 'r', encoding='utf-8') as stream:
        config = yaml.safe_load(stream) or {}
    return config_file, config


def launch_setup(_context):
    course_design_path = package_path('course_design')
    navigation_path = package_path('navigation')
    config_file, config = load_course_config(course_design_path)

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_path, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'map': config.get('map_name', 'map_01'),
            'use_teb': 'true' if config.get('use_teb', True) else 'false',
        }.items(),
    )

    waypoint_node = Node(
        package='course_design',
        executable='waypoint_nav_node',
        output='screen',
        parameters=[{'config_file': config_file}],
    )

    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', os.path.join(
            navigation_path, 'rviz', 'navigation_desktop.rviz')],
        output='screen',
    )

    return [navigation_launch, waypoint_node, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
