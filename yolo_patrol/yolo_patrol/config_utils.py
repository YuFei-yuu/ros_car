import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


PACKAGE_NAME = 'yolo_patrol'
SOURCE_PACKAGE_PATH = Path('/home/ubuntu/ros2_ws/src/yolo_patrol')


class ConfigError(RuntimeError):
    pass


def get_package_path():
    if os.environ.get('need_compile', 'False') == 'True':
        return Path(get_package_share_directory(PACKAGE_NAME))
    if SOURCE_PACKAGE_PATH.exists():
        return SOURCE_PACKAGE_PATH
    return Path(get_package_share_directory(PACKAGE_NAME))


def default_config_path():
    return get_package_path() / 'config' / 'yolo_patrol.yaml'


def load_yaml(path):
    path = Path(path)
    if not path.exists():
        raise ConfigError(f'Config file not found: {path}')
    with path.open('r', encoding='utf-8') as stream:
        return yaml.safe_load(stream) or {}, str(path)


def load_config(config_file=None):
    path = Path(config_file) if config_file else default_config_path()
    return load_yaml(path)


def load_config_from_node(node):
    node.declare_parameter('config_file', '')
    config_file = node.get_parameter('config_file').value
    if not config_file:
        config_file = None
    return load_config(config_file)


def as_bool(value, default=False):
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def deep_get(config, keys, default=None):
    current = config
    for key in keys:
        if not isinstance(current, dict) or key not in current:
            return default
        current = current[key]
    return current
