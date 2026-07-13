import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def package_path(package_name):
    if os.environ.get('need_compile', 'False') == 'True':
        return get_package_share_directory(package_name)
    return f'/home/ubuntu/ros2_ws/src/{package_name}'


def classes_for_model(model_name):
    garbage_names = [
        'BananaPeel', 'BrokenBones', 'CigaretteEnd', 'DisposableChopsticks',
        'Ketchup', 'Marker', 'OralLiquidBottle', 'Plate', 'PlasticBottle',
        'StorageBattery', 'Toothbrush', 'Umbrella',
    ]
    traffic_names = ['go', 'right', 'park', 'red', 'green', 'crosswalk']
    coco_names = [
        'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train',
        'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
        'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep',
        'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella',
        'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard',
        'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard',
        'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork',
        'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
        'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
        'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv',
        'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
        'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
        'scissors', 'teddy bear', 'hair drier', 'toothbrush',
    ]
    if 'garbage' in model_name:
        return garbage_names, 'obb'
    if 'traffic' in model_name:
        return traffic_names, 'detect'
    return coco_names, 'detect'


def load_yaml(path):
    with open(path, 'r', encoding='utf-8') as stream:
        return yaml.safe_load(stream) or {}


def launch_setup(context):
    yolo_patrol_path = package_path('yolo_patrol')
    navigation_path = package_path('navigation')
    peripherals_path = package_path('peripherals')

    config_file = LaunchConfiguration('config_file').perform(context)
    if not config_file:
        config_file = os.path.join(yolo_patrol_path, 'config', 'yolo_patrol.yaml')

    patrol_config = load_yaml(config_file)
    course_config_file = patrol_config.get(
        'course_config_file',
        os.path.join(yolo_patrol_path, 'config', 'yolo_patrol_course.yaml'),
    )
    course_config = load_yaml(course_config_file)

    model_name = LaunchConfiguration('model_name').perform(context)
    classes, task = classes_for_model(model_name)
    display = LaunchConfiguration('display').perform(context).strip().lower()
    display = display in ('1', 'true', 'yes', 'on')

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_path, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'map': course_config.get('map_name', 'map_01'),
            'use_teb': 'true' if course_config.get('use_teb', True) else 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_navigation')),
    )

    rviz_node = ExecuteProcess(
        cmd=[
            'rviz2',
            '-d',
            os.path.join(navigation_path, 'rviz', 'navigation_desktop.rviz'),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_rviz')),
    )

    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_path, 'launch', 'depth_camera.launch.py')),
        condition=IfCondition(LaunchConfiguration('start_yolo')),
    )

    yolo_node = Node(
        package='example',
        executable='yolo_node',
        name='yolo',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('camera_topic'),
            'classes': classes,
            'engine': model_name,
            'conf': LaunchConfiguration('conf'),
            'task': task,
            'display': display,
        }],
        condition=IfCondition(LaunchConfiguration('start_yolo')),
    )

    patrol_node = Node(
        package='yolo_patrol',
        executable='yolo_patrol_node',
        output='screen',
        parameters=[{'config_file': config_file}],
    )

    return [
        navigation_launch,
        rviz_node,
        depth_camera_launch,
        yolo_node,
        patrol_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=''),
        DeclareLaunchArgument('start_navigation', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('start_yolo', default_value='true'),
        DeclareLaunchArgument('model_name', default_value='yolo26n'),
        DeclareLaunchArgument('conf', default_value='0.6'),
        DeclareLaunchArgument('display', default_value='false'),
        DeclareLaunchArgument('camera_topic', default_value='/depth_cam/rgb0/image_raw'),
        OpaqueFunction(function=launch_setup),
    ])
