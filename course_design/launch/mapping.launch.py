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
    slam_path = package_path('slam')

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_path, 'launch', 'slam.launch.py')),
        launch_arguments={
            'slam_method': 'slam_toolbox',
            'sim': 'false',
            'enable_save': 'true',
        }.items(),
    )

    map_status_node = Node(
        package='course_design',
        executable='map_status_node',
        output='screen',
        parameters=[{
            'config_file': os.path.join(
                course_design_path, 'config', 'course_design.yaml'),
        }],
    )

    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', os.path.join(slam_path, 'rviz', 'slam_desktop.rviz')],
        output='screen',
    )

    return [slam_launch, map_status_node, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
