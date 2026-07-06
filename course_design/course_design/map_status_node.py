import os
import subprocess

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

from course_design.config_utils import load_config_from_node


class MapStatusNode(Node):
    def __init__(self):
        super().__init__('map_status_node')
        self.config, self.config_path = load_config_from_node(self)
        self.mapping_config = self.config.get('mapping', {})

        self.map_topic = self.mapping_config.get('map_topic', '/map')
        self.scan_topic = self.mapping_config.get('scan_topic', '/scan')
        self.save_path = self.mapping_config.get(
            'save_path', '/home/ubuntu/ros2_ws/src/slam/maps/map_01')
        self.save_timeout_sec = float(self.mapping_config.get('save_timeout_sec', 15.0))
        self.status_period_sec = float(self.mapping_config.get('status_period_sec', 5.0))

        self.last_map_time = None
        self.last_scan_time = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.0
        self.callback_group = ReentrantCallbackGroup()

        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, 1)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.create_service(
            Trigger,
            '~/save_map',
            self.save_map_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            '~/init_finish',
            self.init_finish_callback,
            callback_group=self.callback_group,
        )
        self.create_timer(self.status_period_sec, self.print_status)

        self.get_logger().info(f'Course mapping status started. config={self.config_path}')
        self.get_logger().info(
            f'Watching map_topic={self.map_topic}, scan_topic={self.scan_topic}, '
            f'save_path={self.save_path}'
        )

    def map_callback(self, msg):
        self.last_map_time = self.get_clock().now()
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution

    def scan_callback(self, _msg):
        self.last_scan_time = self.get_clock().now()

    def init_finish_callback(self, _request, response):
        response.success = True
        response.message = 'map_status_node ready'
        return response

    def age_text(self, stamp):
        if stamp is None:
            return 'WAITING'
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        return f'OK age={age:.1f}s'

    def print_status(self):
        self.get_logger().info(
            'MAPPING STATUS '
            f'map={self.age_text(self.last_map_time)} '
            f'size={self.map_width}x{self.map_height} '
            f'resolution={self.map_resolution:.3f} '
            f'scan={self.age_text(self.last_scan_time)} '
            f'save_path={self.save_path}'
        )

    def save_map_callback(self, _request, response):
        os.makedirs(os.path.dirname(self.save_path), exist_ok=True)

        if self.last_map_time is None:
            response.success = False
            response.message = f'No map received on {self.map_topic}; cannot save yet'
            self.get_logger().error(response.message)
            return response

        command = [
            'ros2',
            'run',
            'nav2_map_server',
            'map_saver_cli',
            '-f',
            self.save_path,
            '--ros-args',
            '-p',
            'map_subscribe_transient_local:=true',
        ]

        self.get_logger().info('Saving map with nav2_map_server map_saver_cli')
        self.get_logger().info(' '.join(command))
        try:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=self.save_timeout_sec,
                check=False,
            )
        except subprocess.TimeoutExpired:
            response.success = False
            response.message = f'map_saver_cli timeout after {self.save_timeout_sec:.1f}s'
            self.get_logger().error(response.message)
            return response

        yaml_path = f'{self.save_path}.yaml'
        pgm_path = f'{self.save_path}.pgm'
        response.success = (
            result.returncode == 0
            and os.path.exists(yaml_path)
            and os.path.exists(pgm_path)
        )
        if response.success:
            response.message = f'map saved: {yaml_path}, {pgm_path}'
        else:
            details = (result.stderr or result.stdout or '').strip()
            response.message = (
                f'map_saver_cli failed code={result.returncode} path={self.save_path}'
            )
            if details:
                response.message = f'{response.message}; {details[-300:]}'

        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().error(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MapStatusNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
