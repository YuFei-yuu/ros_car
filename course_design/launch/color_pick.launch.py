import os

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
    example_path = package_path('example')
    slam_path = package_path('slam')
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_path, 'launch', 'include', 'robot.launch.py')),
        launch_arguments={
            'sim': 'false',
            'robot_name': '/',
            'master_name': '/',
            'use_joy': 'false',
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
        parameters=[{'config_file': os.path.join(
            course_design_path, 'config', 'course_design.yaml')}],
    )
    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', os.path.join(
            course_design_path, 'rviz', 'color_pick.rviz')],
        output='screen',
    )
    return [robot_launch, color_detect_node, pick_place_node, rviz_node]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
