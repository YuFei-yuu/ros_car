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


def launch_setup(_context):
    course_design_path = package_path('course_design')
    navigation_path = package_path('navigation')
    example_path = package_path('example')
    config_file = os.path.join(course_design_path, 'config', 'course_design.yaml')
    with open(config_file, 'r', encoding='utf-8') as stream:
        config = yaml.safe_load(stream) or {}

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_path, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'robot_name': '/',
            'master_name': '/',
            'map': config.get('map_name', 'map_01'),
            'use_teb': 'true' if config.get('use_teb', True) else 'false',
        }.items(),
    )
    color_detect_node = Node(
        package='example',
        executable='color_detect',
        output='screen',
        parameters=[
            os.path.join(example_path, 'config', 'roi.yaml'),
            {'enable_display': False, 'enable_roi_display': False},
        ],
    )
    pick_place_node = Node(
        package='course_design',
        executable='pick_place_node',
        output='screen',
        parameters=[{'config_file': config_file}],
    )
    workflow_node = Node(
        package='course_design',
        executable='transport_workflow_node',
        output='screen',
        parameters=[{'config_file': config_file}],
    )
    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', os.path.join(
            navigation_path, 'rviz', 'navigation_desktop.rviz')],
        output='screen',
    )
    return [
        navigation_launch,
        color_detect_node,
        pick_place_node,
        workflow_node,
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
