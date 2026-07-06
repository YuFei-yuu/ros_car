import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def package_path(package_name):
    if os.environ.get('need_compile', 'False') == 'True':
        return get_package_share_directory(package_name)
    return f'/home/ubuntu/ros2_ws/src/{package_name}'


def launch_setup(_context):
    course_design_path = package_path('course_design')
    config_file = os.path.join(course_design_path, 'config', 'course_design.yaml')

    course_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(course_design_path, 'launch', 'course_nav.launch.py')),
    )

    qrcode_node = Node(
        package='course_design',
        executable='qrcode_detect_node',
        output='screen',
        parameters=[{'config_file': config_file}],
    )

    behavior_node = Node(
        package='course_design',
        executable='behavior_node',
        output='screen',
        parameters=[{'config_file': config_file}],
    )

    return [course_nav_launch, qrcode_node, behavior_node]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
